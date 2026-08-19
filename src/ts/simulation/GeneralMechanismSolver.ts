import type {
  AnalyticSolveStep,
  ComponentId,
  ConstraintDiagnostics,
  SimulationState,
} from '../model';
import {
  captureJointWorldPositions,
  placeUnderconstrainedLinks,
  solveAnalyticConstraints,
  type AnalyticConstraintSolveResult,
} from './analyticConstraintSolver';
import {
  ConstraintGraphError,
  buildConstraintGraph,
  connectedComponents,
  type ConstraintGraph,
} from './ConstraintGraph';
import {
  analyzeConstraintGraph,
  type ConstraintGraphDiagnostics,
} from './constraintAnalysis';
import { validateMechanismInvariants } from './mechanismInvariants';

export interface GeneralMechanismSolveResult {
  valid: boolean;
  graph: ConstraintGraph | null;
  diagnostics: ConstraintDiagnostics;
  analytic: AnalyticConstraintSolveResult | null;
  analyticSteps: AnalyticSolveStep[];
  jointPositions: Map<ComponentId, { x: number; y: number }>;
  message: string;
}

/**
 * Runs topology analysis and the deterministic analytic graph traversal. A
 * constrained analytic remainder is reserved for the bounded numerical stage.
 */
export function solveGeneralMechanism(
  state: SimulationState,
  previousJointPositions: ReadonlyMap<ComponentId, { x: number; y: number }> = new Map(),
): GeneralMechanismSolveResult {
  let graph: ConstraintGraph;
  try {
    graph = buildConstraintGraph({ links: state.links, joints: state.joints, servo: state.servo });
  } catch (error) {
    const message = error instanceof ConstraintGraphError ? error.message : 'Constraint graph construction failed';
    return {
      valid: false,
      graph: null,
      diagnostics: { valid: false, components: [], disconnectedComponentIds: [] },
      analytic: null,
      analyticSteps: [],
      jointPositions: new Map(),
      message,
    };
  }

  // Rank/mobility are structural properties used to decide whether a remaining
  // pose is genuinely free or should be sent to numerical closure.
  const structural = analyzeConstraintGraph(graph);
  const analytic = solveAnalyticConstraints(graph, { previousJointPositions });
  const components = connectedComponents(graph);
  const constrainedRemainders: string[] = [];
  if (analytic.valid) {
    for (const component of components) {
      const analysis = structural.components.find((candidate) => candidate.id === component.id);
      const unresolved = component.linkIds.filter((linkId) => analytic.unresolvedLinkIds.includes(linkId));
      if (unresolved.length === 0) continue;
      if (!component.anchored || (analysis?.drivenDof ?? 1) > 0) {
        placeUnderconstrainedLinks(graph, analytic, new Set(component.linkIds));
      } else {
        constrainedRemainders.push(component.id);
      }
    }
  }

  const finalAnalysis = analyzeConstraintGraph(graph);
  const diagnostics = combineDiagnostics(finalAnalysis, structural, analytic, constrainedRemainders);
  const invariants = validateMechanismInvariants(graph);
  const valid = analytic.valid && constrainedRemainders.length === 0 &&
    diagnostics.valid && invariants.valid;
  if (!invariants.valid) {
    for (const component of diagnostics.components) {
      if (!component.linkIds.some((id) => invariants.invalidLinkIds.includes(id)) &&
          !component.jointIds.some((id) => invariants.invalidJointIds.includes(id))) continue;
      component.inconsistent = true;
      component.messages.push(...invariants.messages);
    }
    diagnostics.valid = false;
  }

  return {
    valid,
    graph,
    diagnostics,
    analytic,
    analyticSteps: analytic.steps,
    jointPositions: captureJointWorldPositions(graph),
    message: summarizeSolverState(diagnostics, analytic, constrainedRemainders),
  };
}

function combineDiagnostics(
  finalAnalysis: ConstraintGraphDiagnostics,
  structural: ConstraintGraphDiagnostics,
  analytic: AnalyticConstraintSolveResult,
  constrainedRemainders: readonly string[],
): ConstraintDiagnostics {
  const components = finalAnalysis.components.map((component) => {
    const structuralComponent = structural.components.find((candidate) => candidate.id === component.id);
    const linkSet = new Set(component.linkIds);
    const jointSet = new Set(component.jointIds);
    const analyticSolveCount = analytic.resolutions.filter((resolution) =>
      resolution.kind !== 'underdetermined' && resolution.linkIds.some((id) => linkSet.has(id)),
    ).length;
    const analyticFailure = analytic.resolutions.some((resolution) =>
      resolution.kind === 'inconsistent' && resolution.linkIds.some((id) => linkSet.has(id)),
    );
    const singular = component.singular ||
      [...analytic.singularJointIds].some((jointId) => jointSet.has(jointId));
    const unresolvedLinkIds = [...new Set([
      ...analytic.unresolvedLinkIds.filter((id) => linkSet.has(id)),
      ...[...analytic.intentionallyFreeLinkIds].filter((id) => linkSet.has(id)),
    ])].sort((left, right) => left.localeCompare(right));
    const messages = [...component.messages];
    for (const resolution of analytic.resolutions) {
      if (!resolution.linkIds.some((id) => linkSet.has(id))) continue;
      if (resolution.kind === 'singular') messages.push(`Analytic singularity at ${resolution.jointId ?? 'component'}`);
      if (resolution.kind === 'inconsistent') messages.push(resolution.message);
    }
    if (constrainedRemainders.includes(component.id)) {
      messages.push('Constrained analytic remainder requires numerical fallback');
    }
    return {
      ...component,
      passiveJacobianRank: structuralComponent?.passiveJacobianRank ?? component.passiveJacobianRank,
      passiveDof: structuralComponent?.passiveDof ?? component.passiveDof,
      drivenDof: structuralComponent?.drivenDof ?? component.drivenDof,
      unresolvedLinkIds,
      inconsistent: component.inconsistent || analyticFailure || constrainedRemainders.includes(component.id),
      singular,
      analyticSolveCount,
      numericalFallbackUsed: false,
      messages: [...new Set(messages)],
    };
  });
  return {
    valid: analytic.valid && constrainedRemainders.length === 0 &&
      components.every((component) => !component.inconsistent),
    components,
    disconnectedComponentIds: [...finalAnalysis.disconnectedComponentIds],
  };
}

function summarizeSolverState(
  diagnostics: ConstraintDiagnostics,
  analytic: AnalyticConstraintSolveResult,
  constrainedRemainders: readonly string[],
): string {
  const singular = diagnostics.components.find((component) => component.singular);
  if (singular) {
    const joint = analytic.singularJointIds.values().next().value as string | undefined;
    return `Singular ${joint ? `dyad · ${joint} · ` : ''}rank ${singular.jacobianRank}/${singular.variableCount}`;
  }
  const inconsistent = diagnostics.components.find((component) => component.inconsistent);
  if (inconsistent) {
    if (constrainedRemainders.includes(inconsistent.id)) {
      return `Numerical fallback required · ${inconsistent.id}`;
    }
    return inconsistent.messages.at(-1) ?? `Constraint residual ${inconsistent.residualNorm.toPrecision(3)}`;
  }
  const disconnected = diagnostics.components.find((component) => !component.anchored);
  if (disconnected) return `Disconnected component · ${disconnected.linkIds.length} link(s)`;
  const passiveDof = diagnostics.components.reduce((sum, component) => sum + component.passiveDof, 0);
  const drivenDof = diagnostics.components.reduce((sum, component) => sum + component.drivenDof, 0);
  return `Solved analytically · DOF ${passiveDof} / driven ${drivenDof}`;
}

