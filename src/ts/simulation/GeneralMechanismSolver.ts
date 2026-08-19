import { normalizeAngle } from '../geometry';
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
  createComponentResidualSystem,
  type ComponentResidualSystem,
  type ConstraintGraphDiagnostics,
} from './constraintAnalysis';
import { validateMechanismInvariants } from './mechanismInvariants';
import { solveBoundedDampedLeastSquares } from './numericalConstraintSolver';
import { SOLVER_TOLERANCES } from './solverTolerances';

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
  const numericalFallbackComponents = new Set<string>();
  const numericalFailures = new Map<string, string>();
  if (analytic.valid) {
    for (const component of components) {
      const analysis = structural.components.find((candidate) => candidate.id === component.id);
      const unresolved = component.linkIds.filter((linkId) => analytic.unresolvedLinkIds.includes(linkId));
      if (unresolved.length === 0) continue;
      if (!component.anchored || (analysis?.drivenDof ?? 1) > 0) {
        placeUnderconstrainedLinks(graph, analytic, new Set(component.linkIds));
      } else {
        numericalFallbackComponents.add(component.id);
        const system = createComponentResidualSystem(graph, component, { includeActuator: true });
        const numerical = solveBoundedDampedLeastSquares(
          system.initialConfiguration,
          (configuration) => system.evaluate(configuration),
          {
            project: (configuration) => projectConfigurationToJointRom(system, configuration),
            variableKinds: system.variables.map((variable) =>
              variable.coordinate === 'angle' ? 'angle' as const : 'translation' as const),
            finiteDifferenceStep: system.finiteDifferenceSteps,
            residualTolerance: SOLVER_TOLERANCES.numericalResidual,
            maxIterations: 60,
            maxTranslationStep: 8,
            maxAngularStep: Math.PI / 12,
          },
        );
        if (numerical.kind === 'converged') {
          applyComponentConfiguration(system, numerical.variables);
          for (const linkId of unresolved) analytic.resolvedLinkIds.add(linkId);
          analytic.unresolvedLinkIds = analytic.unresolvedLinkIds.filter((linkId) => !unresolved.includes(linkId));
        } else {
          numericalFailures.set(component.id, numerical.message);
        }
      }
    }
  }

  const finalAnalysis = analyzeConstraintGraph(graph);
  const diagnostics = combineDiagnostics(
    finalAnalysis,
    structural,
    analytic,
    numericalFallbackComponents,
    numericalFailures,
  );
  const invariants = validateMechanismInvariants(graph);
  const valid = analytic.valid && numericalFailures.size === 0 &&
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
    message: summarizeSolverState(diagnostics, analytic, numericalFailures),
  };
}

function combineDiagnostics(
  finalAnalysis: ConstraintGraphDiagnostics,
  structural: ConstraintGraphDiagnostics,
  analytic: AnalyticConstraintSolveResult,
  numericalFallbackComponents: ReadonlySet<string>,
  numericalFailures: ReadonlyMap<string, string>,
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
    const numericalFailure = numericalFailures.get(component.id);
    if (numericalFailure) messages.push(`Numerical fallback failed · ${numericalFailure}`);
    return {
      ...component,
      passiveJacobianRank: structuralComponent?.passiveJacobianRank ?? component.passiveJacobianRank,
      passiveDof: structuralComponent?.passiveDof ?? component.passiveDof,
      drivenDof: structuralComponent?.drivenDof ?? component.drivenDof,
      unresolvedLinkIds,
      inconsistent: component.inconsistent || analyticFailure || numericalFailure !== undefined,
      singular,
      analyticSolveCount,
      numericalFallbackUsed: numericalFallbackComponents.has(component.id),
      messages: [...new Set(messages)],
    };
  });
  return {
    valid: analytic.valid && numericalFailures.size === 0 &&
      components.every((component) => !component.inconsistent),
    components,
    disconnectedComponentIds: [...finalAnalysis.disconnectedComponentIds],
  };
}

function summarizeSolverState(
  diagnostics: ConstraintDiagnostics,
  analytic: AnalyticConstraintSolveResult,
  numericalFailures: ReadonlyMap<string, string>,
): string {
  const singular = diagnostics.components.find((component) => component.singular);
  if (singular) {
    const joint = analytic.singularJointIds.values().next().value as string | undefined;
    return `Singular ${joint ? `dyad · ${joint} · ` : ''}rank ${singular.jacobianRank}/${singular.variableCount}`;
  }
  const inconsistent = diagnostics.components.find((component) => component.inconsistent);
  if (inconsistent) {
    if (numericalFailures.has(inconsistent.id)) {
      return `Constraint residual ${inconsistent.residualNorm.toPrecision(3)} · numerical fallback failed`;
    }
    return inconsistent.messages.at(-1) ?? `Constraint residual ${inconsistent.residualNorm.toPrecision(3)}`;
  }
  const disconnected = diagnostics.components.find((component) => !component.anchored);
  if (disconnected) return `Disconnected component · ${disconnected.linkIds.length} link(s)`;
  const passiveDof = diagnostics.components.reduce((sum, component) => sum + component.passiveDof, 0);
  const drivenDof = diagnostics.components.reduce((sum, component) => sum + component.drivenDof, 0);
  const method = diagnostics.components.some((component) => component.numericalFallbackUsed)
    ? 'Solved with numerical fallback'
    : 'Solved analytically';
  return `${method} · DOF ${passiveDof} / driven ${drivenDof}`;
}

function applyComponentConfiguration(system: ComponentResidualSystem, configuration: readonly number[]): void {
  for (const variable of system.variables) {
    const body = system.graph.bodies.get(variable.linkId);
    if (body?.kind !== 'link') continue;
    const value = configuration[variable.index];
    if (value === undefined) continue;
    if (variable.coordinate === 'x') body.link.pose.position.x = value;
    else if (variable.coordinate === 'y') body.link.pose.position.y = value;
    else body.link.pose.angle = normalizeAngle(value);
  }
}

function projectConfigurationToJointRom(
  system: ComponentResidualSystem,
  configuration: number[],
): number[] {
  const projected = [...configuration];
  const angleIndices = new Map(system.variables
    .filter((variable) => variable.coordinate === 'angle')
    .map((variable) => [variable.linkId, variable.index]));
  const angleFor = (linkId: ComponentId): number => {
    const index = angleIndices.get(linkId);
    if (index !== undefined) return projected[index]!;
    const body = system.graph.bodies.get(linkId);
    return body?.kind === 'link' ? body.link.pose.angle : 0;
  };

  for (let pass = 0; pass < 2; pass += 1) {
    for (const constraint of system.component.constraints) {
      if (constraint.kind !== 'revolute') continue;
      const joint = constraint.joint;
      if (joint.minAngle === undefined || joint.maxAngle === undefined ||
          joint.maxAngle - joint.minAngle >= Math.PI * 2 - SOLVER_TOLERANCES.jointLimit ||
          Math.abs(joint.maxAngle - joint.minAngle) <= SOLVER_TOLERANCES.lockedAngle ||
          joint.id === constraintGraphActuatorJointId(system)) continue;
      const angleA = joint.linkAId === null ? 0 : angleFor(joint.linkAId);
      const angleB = angleFor(joint.linkBId);
      const relative = normalizeAngle(angleB - angleA);
      const limited = projectAngle(relative, joint.minAngle, joint.maxAngle);
      if (Math.abs(normalizeAngle(limited - relative)) <= SOLVER_TOLERANCES.jointLimit) continue;
      const indexB = angleIndices.get(joint.linkBId);
      if (indexB !== undefined) projected[indexB] = angleA + limited;
      else if (joint.linkAId !== null) {
        const indexA = angleIndices.get(joint.linkAId);
        if (indexA !== undefined) projected[indexA] = angleB - limited;
      }
    }
  }
  return projected;
}

function projectAngle(angle: number, minimum: number, maximum: number): number {
  if (minimum <= maximum) return Math.max(minimum, Math.min(maximum, angle));
  if (angle >= minimum || angle <= maximum) return angle;
  return Math.abs(normalizeAngle(angle - minimum)) <= Math.abs(normalizeAngle(angle - maximum))
    ? minimum
    : maximum;
}

function constraintGraphActuatorJointId(system: ComponentResidualSystem): ComponentId | undefined {
  return system.component.constraints.find((constraint) => constraint.kind === 'actuator')?.servo.revoluteJointId;
}
