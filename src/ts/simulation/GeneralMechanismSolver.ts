import { normalizeAngle, type Pose2D } from '../geometry';
import type {
  AnalyticSolveStep,
  ComponentId,
  ConstraintDiagnostics,
  RevoluteJoint,
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
import { evaluateLinearSlotGeometry } from './linearSlotGeometry';
import { solveBoundedDampedLeastSquares } from './numericalConstraintSolver';
import {
  ANGLE_RESIDUAL_LENGTH_SCALE,
  SOLVER_TOLERANCES,
} from './solverTolerances';

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
  const initialLinkPoses = state.links.map((link) => [link, {
    position: { ...link.pose.position },
    angle: link.pose.angle,
  }] as const);
  let graph: ConstraintGraph;
  try {
    graph = buildConstraintGraph({
      links: state.links,
      joints: state.joints,
      linearSlotJoints: state.linearSlotJoints,
      servo: state.servo,
    });
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
      const expectedDrivenDof = analysis === undefined
        ? 1
        : Math.max(0, analysis.variableCount - analysis.expectedDrivenJacobianRank);
      const hasLinearSlot = component.constraints.some((constraint) => constraint.kind === 'linear-slot');
      // Slot closure is intentionally numerical. A fully constrained slot
      // component is refined even if revolute traversal happened to place all
      // of its bodies; a mobile component keeps its prior free coordinates.
      if (unresolved.length === 0 && !(hasLinearSlot && expectedDrivenDof === 0)) continue;
      if (!component.anchored || expectedDrivenDof > 0) {
        placeUnderconstrainedLinks(graph, analytic, new Set(component.linkIds));
      } else {
        numericalFallbackComponents.add(component.id);
        const system = createComponentResidualSystem(graph, component, { includeActuator: true });
        const numerical = solveBoundedDampedLeastSquares(
          system.initialConfiguration,
          (configuration) => evaluateComponentWithInequalityLimits(system, configuration),
          {
            project: (configuration) => projectConfigurationToLimits(system, configuration),
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

  if (!valid) {
    for (const [link, pose] of initialLinkPoses) {
      link.pose = { position: { ...pose.position }, angle: pose.angle };
    }
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
  analytic: AnalyticConstraintSolveResult,
  numericalFallbackComponents: ReadonlySet<string>,
  numericalFailures: ReadonlyMap<string, string>,
): ConstraintDiagnostics {
  const components = finalAnalysis.components.map((component) => {
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

function projectConfigurationToLimits(
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

  // Simultaneous averaged projections avoid making a coupled limit solve
  // depend on constraint IDs or array order. The active ROM residuals below
  // remain authoritative when several inequalities cannot all be satisfied.
  const maximumPasses = Math.max(8, system.component.jointIds.length * 8);
  for (let pass = 0; pass < maximumPasses; pass += 1) {
    const corrections = new Map<number, { sum: number; count: number }>();
    let largestViolation = 0;
    const addCorrection = (index: number, correction: number): void => {
      const current = corrections.get(index) ?? { sum: 0, count: 0 };
      current.sum += correction;
      current.count += 1;
      corrections.set(index, current);
    };
    for (const constraint of system.component.constraints) {
      if (constraint.kind !== 'revolute') continue;
      const joint = constraint.joint;
      if (!hasActiveRomInequality(system, joint)) continue;
      const angleA = joint.linkAId === null ? 0 : angleFor(joint.linkAId);
      const angleB = angleFor(joint.linkBId);
      const relative = normalizeAngle(angleB - angleA);
      const limited = projectAngle(relative, joint.minAngle, joint.maxAngle);
      const correction = normalizeAngle(limited - relative);
      largestViolation = Math.max(largestViolation, Math.abs(correction));
      if (Math.abs(correction) <= SOLVER_TOLERANCES.jointLimit) continue;
      const indexA = joint.linkAId === null ? undefined : angleIndices.get(joint.linkAId);
      const indexB = angleIndices.get(joint.linkBId);
      if (indexA !== undefined && indexB !== undefined) {
        addCorrection(indexA, -correction / 2);
        addCorrection(indexB, correction / 2);
      } else if (indexB !== undefined) {
        addCorrection(indexB, correction);
      } else if (indexA !== undefined) {
        addCorrection(indexA, -correction);
      }
    }
    if (corrections.size === 0 || largestViolation <= SOLVER_TOLERANCES.jointLimit) break;
    for (const [index, correction] of corrections) {
      projected[index] = normalizeAngle(projected[index]! + correction.sum / correction.count);
    }
  }
  projectLinearSlotTravel(system, projected);
  return projected;
}

/** Adds one constant-dimension active-limit residual per ordinary finite ROM. */
function evaluateComponentWithInequalityLimits(
  system: ComponentResidualSystem,
  configuration: readonly number[],
): number[] {
  const residual = system.evaluate(configuration);
  const angleIndices = new Map(system.variables
    .filter((variable) => variable.coordinate === 'angle')
    .map((variable) => [variable.linkId, variable.index]));
  const angleFor = (linkId: ComponentId): number => {
    const index = angleIndices.get(linkId);
    if (index !== undefined) return configuration[index]!;
    const body = system.graph.bodies.get(linkId);
    return body?.kind === 'link' ? body.link.pose.angle : 0;
  };
  for (const constraint of system.component.constraints) {
    if (constraint.kind !== 'revolute' || !hasActiveRomInequality(system, constraint.joint)) continue;
    const joint = constraint.joint;
    const angleA = joint.linkAId === null ? 0 : angleFor(joint.linkAId);
    const relative = normalizeAngle(angleFor(joint.linkBId) - angleA);
    const limited = projectAngle(relative, joint.minAngle, joint.maxAngle);
    residual.push(normalizeAngle(relative - limited) * ANGLE_RESIDUAL_LENGTH_SCALE);
  }
  for (const constraint of system.component.constraints) {
    if (constraint.kind !== 'linear-slot') continue;
    const geometry = linearSlotGeometryForConfiguration(system, constraint.joint, configuration);
    const limited = Math.max(
      constraint.joint.minTravel,
      Math.min(constraint.joint.maxTravel, geometry.travel),
    );
    residual.push(geometry.travel - limited);
  }
  return residual;
}

function projectLinearSlotTravel(
  system: ComponentResidualSystem,
  configuration: number[],
): void {
  const coordinateIndices = new Map<ComponentId, Partial<Record<'x' | 'y' | 'angle', number>>>();
  for (const variable of system.variables) {
    const indices = coordinateIndices.get(variable.linkId) ?? {};
    indices[variable.coordinate] = variable.index;
    coordinateIndices.set(variable.linkId, indices);
  }
  const constraints = system.component.constraints.filter((constraint) => constraint.kind === 'linear-slot');
  const maximumPasses = Math.max(8, constraints.length * 8);
  for (let pass = 0; pass < maximumPasses; pass += 1) {
    const corrections = new Map<number, number[]>();
    let largestViolation = 0;
    const addCorrection = (index: number | undefined, value: number): void => {
      if (index === undefined) return;
      const values = corrections.get(index) ?? [];
      values.push(value);
      corrections.set(index, values);
    };
    for (const constraint of constraints) {
      const geometry = linearSlotGeometryForConfiguration(system, constraint.joint, configuration);
      const limited = Math.max(
        constraint.joint.minTravel,
        Math.min(constraint.joint.maxTravel, geometry.travel),
      );
      const correction = limited - geometry.travel;
      largestViolation = Math.max(largestViolation, Math.abs(correction));
      if (Math.abs(correction) <= SOLVER_TOLERANCES.slotTravel) continue;
      const pinIndices = coordinateIndices.get(constraint.joint.pinLinkId);
      const slotIndices = constraint.joint.slotLinkId === null
        ? undefined
        : coordinateIndices.get(constraint.joint.slotLinkId);
      const movableBodyCount = Number(pinIndices !== undefined) + Number(slotIndices !== undefined);
      if (movableBodyCount === 0) continue;
      const pinScale = movableBodyCount === 2 ? 0.5 : 1;
      const slotScale = movableBodyCount === 2 ? -0.5 : -1;
      if (pinIndices !== undefined) {
        addCorrection(pinIndices.x, geometry.axis.x * correction * pinScale);
        addCorrection(pinIndices.y, geometry.axis.y * correction * pinScale);
      }
      if (slotIndices !== undefined) {
        addCorrection(slotIndices.x, geometry.axis.x * correction * slotScale);
        addCorrection(slotIndices.y, geometry.axis.y * correction * slotScale);
      }
    }
    if (corrections.size === 0 || largestViolation <= SOLVER_TOLERANCES.slotTravel) break;
    for (const [index, values] of corrections) {
      // Sorting numeric contributions makes simultaneous projection invariant
      // to slot IDs and insertion order, including its floating-point sum.
      values.sort((left, right) => left - right);
      const average = values.reduce((sum, value) => sum + value, 0) / values.length;
      configuration[index] = configuration[index]! + average;
    }
  }
}

function linearSlotGeometryForConfiguration(
  system: ComponentResidualSystem,
  joint: Extract<ConstraintGraph['constraints'][number], { kind: 'linear-slot' }>['joint'],
  configuration: readonly number[],
) {
  const pinPose = componentPoseForConfiguration(system, joint.pinLinkId, configuration);
  const slotPose = joint.slotLinkId === null
    ? null
    : componentPoseForConfiguration(system, joint.slotLinkId, configuration);
  return evaluateLinearSlotGeometry(joint, slotPose, pinPose);
}

function componentPoseForConfiguration(
  system: ComponentResidualSystem,
  linkId: ComponentId,
  configuration: readonly number[],
): Pose2D {
  const body = system.graph.bodies.get(linkId);
  if (body?.kind !== 'link') throw new Error(`Missing slot body ${linkId}`);
  const pose = {
    position: { ...body.link.pose.position },
    angle: body.link.pose.angle,
  };
  for (const variable of system.variables) {
    if (variable.linkId !== linkId) continue;
    const value = configuration[variable.index];
    if (value === undefined) continue;
    if (variable.coordinate === 'x') pose.position.x = value;
    else if (variable.coordinate === 'y') pose.position.y = value;
    else pose.angle = value;
  }
  return pose;
}

function hasActiveRomInequality(
  system: ComponentResidualSystem,
  joint: RevoluteJoint,
): joint is RevoluteJoint & { minAngle: number; maxAngle: number } {
  return joint.minAngle !== undefined && joint.maxAngle !== undefined &&
    joint.maxAngle - joint.minAngle < Math.PI * 2 - SOLVER_TOLERANCES.jointLimit &&
    Math.abs(joint.maxAngle - joint.minAngle) > SOLVER_TOLERANCES.lockedAngle &&
    joint.id !== constraintGraphActuatorJointId(system);
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
