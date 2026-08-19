import {
  localToWorld,
  normalizeAngle,
  type Pose2D,
  type Vec2,
} from '../geometry';
import type {
  ComponentId,
  ConstraintComponentDiagnostics,
  ConstraintDiagnostics,
} from '../model';
import {
  WORLD_BODY_ID,
  connectedComponents,
  type ConstraintGraph,
  type ConstraintGraphComponent,
  type ConstraintGraphConstraint,
  type LinkConstraintBody,
} from './ConstraintGraph';
import {
  centralDifferenceJacobian,
  estimateMatrixRank,
  type MatrixRankEstimate,
} from './linearAlgebra';
import { evaluateLinearSlotGeometry } from './linearSlotGeometry';
import {
  ANGLE_RESIDUAL_LENGTH_SCALE,
  SOLVER_TOLERANCES,
} from './solverTolerances';

export type ConfigurationCoordinate = 'x' | 'y' | 'angle';
export type ResidualCoordinate = 'x' | 'y' | 'angle' | 'normal';

export interface ComponentConfigurationVariable {
  index: number;
  linkId: ComponentId;
  coordinate: ConfigurationCoordinate;
}

export interface ComponentScalarResidual {
  index: number;
  constraintId: string;
  constraintKind: ConstraintGraphConstraint['kind'];
  coordinate: ResidualCoordinate;
  jointId?: ComponentId;
  actuatorId?: ComponentId;
}

export interface ComponentResidualSystemOptions {
  includeActuator?: boolean;
  angleResidualLengthScale?: number;
}

export interface ComponentResidualSystem {
  graph: ConstraintGraph;
  component: ConstraintGraphComponent;
  includeActuator: boolean;
  angleResidualLengthScale: number;
  variables: readonly ComponentConfigurationVariable[];
  scalarResiduals: readonly ComponentScalarResidual[];
  initialConfiguration: readonly number[];
  finiteDifferenceSteps: readonly number[];
  evaluate(configuration: readonly number[]): number[];
}

export interface ConstraintAnalysisOptions {
  residualTolerance?: number;
  rankAbsoluteTolerance?: number;
  rankRelativeTolerance?: number;
  /** Deterministic off-configuration samples used to estimate generic topology rank. */
  expectedRankSamples?: number;
}

export interface ConstraintComponentAnalysis extends ConstraintComponentDiagnostics {
  passiveResidualCount: number;
  passiveResidualNorm: number;
  drivenResidualNorm: number;
  drivenJacobianRank: number;
  expectedPassiveJacobianRank: number;
  expectedDrivenJacobianRank: number;
  passiveRankMetric: number;
  drivenRankMetric: number;
}

export interface ConstraintGraphDiagnostics extends ConstraintDiagnostics {
  components: ConstraintComponentAnalysis[];
}

interface VariableIndices {
  x: number;
  y: number;
  angle: number;
}

const DEFAULT_EXPECTED_RANK_SAMPLES = 3;
const NEAR_SINGULAR_RANK_FACTOR = 10;

function vectorNorm(values: readonly number[]): number {
  return Math.hypot(...values);
}

function linkBody(graph: ConstraintGraph, linkId: ComponentId): LinkConstraintBody {
  const body = graph.bodies.get(linkId);
  if (body?.kind !== 'link') {
    throw new Error(`Constraint component references missing link body ${linkId}`);
  }
  return body;
}

function variableIndexMap(
  variables: readonly ComponentConfigurationVariable[],
): ReadonlyMap<ComponentId, VariableIndices> {
  const indices = new Map<ComponentId, Partial<VariableIndices>>();
  for (const variable of variables) {
    const current = indices.get(variable.linkId) ?? {};
    current[variable.coordinate] = variable.index;
    indices.set(variable.linkId, current);
  }
  return new Map([...indices].map(([linkId, partial]) => {
    if (partial.x === undefined || partial.y === undefined || partial.angle === undefined) {
      throw new Error(`Incomplete configuration variables for link ${linkId}`);
    }
    return [linkId, partial as VariableIndices] as const;
  }));
}

function poseForBody(
  graph: ConstraintGraph,
  bodyId: ComponentId,
  configuration: readonly number[],
  indicesByLinkId: ReadonlyMap<ComponentId, VariableIndices>,
): Pose2D {
  const body = linkBody(graph, bodyId);
  const indices = indicesByLinkId.get(bodyId);
  if (indices === undefined) return body.link.pose;
  return {
    position: {
      x: configuration[indices.x] as number,
      y: configuration[indices.y] as number,
    },
    angle: configuration[indices.angle] as number,
  };
}

function validateConfiguration(configuration: readonly number[], variableCount: number): void {
  if (configuration.length !== variableCount) {
    throw new RangeError(
      `Expected ${variableCount} component variables, received ${configuration.length}`,
    );
  }
  if (!configuration.every(Number.isFinite)) {
    throw new RangeError('Component configuration must contain only finite values');
  }
}

function scalarResidualDescriptors(
  constraints: readonly ConstraintGraphConstraint[],
  includeActuator: boolean,
): ComponentScalarResidual[] {
  const descriptors: ComponentScalarResidual[] = [];
  const append = (
    constraint: ConstraintGraphConstraint,
    coordinate: ResidualCoordinate,
    details: Pick<ComponentScalarResidual, 'jointId' | 'actuatorId'> = {},
  ): void => {
    descriptors.push({
      index: descriptors.length,
      constraintId: constraint.id,
      constraintKind: constraint.kind,
      coordinate,
      ...details,
    });
  };

  for (const constraint of constraints) {
    switch (constraint.kind) {
      case 'revolute':
        append(constraint, 'x', { jointId: constraint.jointId });
        append(constraint, 'y', { jointId: constraint.jointId });
        break;
      case 'locked-angle':
        append(constraint, 'angle', { jointId: constraint.jointId });
        break;
      case 'actuator':
        if (includeActuator) {
          append(constraint, 'angle', { actuatorId: constraint.actuatorId });
        }
        break;
      case 'linear-slot':
        append(constraint, 'normal', { jointId: constraint.jointId });
        break;
      case 'fixed':
        // Fixed bodies have no configuration variables; their stored pose is a seed.
        break;
    }
  }
  return descriptors;
}

function revoluteWorldPoints(
  graph: ConstraintGraph,
  constraint: Extract<ConstraintGraphConstraint, { kind: 'revolute' }>,
  configuration: readonly number[],
  indicesByLinkId: ReadonlyMap<ComponentId, VariableIndices>,
): readonly [Vec2, Vec2] {
  const worldPointA = constraint.bodyAId === WORLD_BODY_ID
    ? constraint.joint.groundPoint
    : constraint.joint.localPointA === undefined
      ? undefined
      : localToWorld(
          constraint.joint.localPointA,
          poseForBody(graph, constraint.bodyAId, configuration, indicesByLinkId),
        );
  if (worldPointA === undefined) {
    throw new Error(`Revolute constraint ${constraint.jointId} has no body-A attachment`);
  }
  const worldPointB = localToWorld(
    constraint.joint.localPointB,
    poseForBody(graph, constraint.bodyBId, configuration, indicesByLinkId),
  );
  return [worldPointA, worldPointB];
}

/** Builds a pure residual function and a deterministic q-vector for one graph component. */
export function createComponentResidualSystem(
  graph: ConstraintGraph,
  component: ConstraintGraphComponent,
  options: ComponentResidualSystemOptions = {},
): ComponentResidualSystem {
  const includeActuator = options.includeActuator ?? true;
  const angleResidualLengthScale = options.angleResidualLengthScale ?? ANGLE_RESIDUAL_LENGTH_SCALE;
  if (!Number.isFinite(angleResidualLengthScale) || angleResidualLengthScale <= 0) {
    throw new RangeError('angleResidualLengthScale must be finite and positive');
  }

  const variables: ComponentConfigurationVariable[] = [];
  const initialConfiguration: number[] = [];
  const finiteDifferenceSteps: number[] = [];
  for (const linkId of component.linkIds) {
    const body = linkBody(graph, linkId);
    if (body.fixed) continue;
    const values = [body.link.pose.position.x, body.link.pose.position.y, body.link.pose.angle];
    const coordinates: readonly ConfigurationCoordinate[] = ['x', 'y', 'angle'];
    for (let offset = 0; offset < coordinates.length; offset += 1) {
      const coordinate = coordinates[offset] as ConfigurationCoordinate;
      variables.push({ index: variables.length, linkId, coordinate });
      initialConfiguration.push(values[offset] as number);
      finiteDifferenceSteps.push(coordinate === 'angle'
        ? SOLVER_TOLERANCES.finiteDifferenceAngle
        : SOLVER_TOLERANCES.finiteDifferenceTranslation);
    }
  }
  validateConfiguration(initialConfiguration, variables.length);
  const indicesByLinkId = variableIndexMap(variables);
  const scalarResiduals = scalarResidualDescriptors(component.constraints, includeActuator);

  const evaluate = (configuration: readonly number[]): number[] => {
    validateConfiguration(configuration, variables.length);
    const residuals: number[] = [];
    for (const constraint of component.constraints) {
      switch (constraint.kind) {
        case 'revolute': {
          const [worldPointA, worldPointB] = revoluteWorldPoints(
            graph,
            constraint,
            configuration,
            indicesByLinkId,
          );
          residuals.push(worldPointA.x - worldPointB.x, worldPointA.y - worldPointB.y);
          break;
        }
        case 'locked-angle': {
          const angleA = constraint.bodyAId === WORLD_BODY_ID
            ? 0
            : poseForBody(graph, constraint.bodyAId, configuration, indicesByLinkId).angle;
          const angleB = poseForBody(
            graph,
            constraint.bodyBId,
            configuration,
            indicesByLinkId,
          ).angle;
          residuals.push(
            normalizeAngle(angleB - angleA - constraint.targetAngle) * angleResidualLengthScale,
          );
          break;
        }
        case 'actuator': {
          if (!includeActuator) break;
          const drivenAngle = poseForBody(
            graph,
            constraint.bodyBId,
            configuration,
            indicesByLinkId,
          ).angle;
          // Servo angle is an absolute world orientation in the existing model.
          residuals.push(
            normalizeAngle(drivenAngle - constraint.targetAngle) * angleResidualLengthScale,
          );
          break;
        }
        case 'linear-slot': {
          const slotPose = constraint.bodyAId === WORLD_BODY_ID
            ? null
            : poseForBody(graph, constraint.bodyAId, configuration, indicesByLinkId);
          const pinPose = poseForBody(
            graph,
            constraint.bodyBId,
            configuration,
            indicesByLinkId,
          );
          residuals.push(
            evaluateLinearSlotGeometry(constraint.joint, slotPose, pinPose).normalOffset,
          );
          break;
        }
        case 'fixed':
          break;
      }
    }
    if (!residuals.every(Number.isFinite)) {
      throw new RangeError('Constraint residual evaluation produced a non-finite value');
    }
    return residuals;
  };

  return {
    graph,
    component,
    includeActuator,
    angleResidualLengthScale,
    variables,
    scalarResiduals,
    initialConfiguration,
    finiteDifferenceSteps,
    evaluate,
  };
}

export function evaluateComponentResiduals(
  system: ComponentResidualSystem,
  configuration: readonly number[] = system.initialConfiguration,
): number[] {
  return system.evaluate(configuration);
}

export function componentJacobian(
  system: ComponentResidualSystem,
  configuration: readonly number[] = system.initialConfiguration,
): number[][] {
  return centralDifferenceJacobian(
    (variables) => system.evaluate(variables),
    configuration,
    system.finiteDifferenceSteps,
  );
}

function rankOptions(options: ConstraintAnalysisOptions): {
  absoluteTolerance: number;
  relativeTolerance: number;
} {
  return {
    absoluteTolerance: options.rankAbsoluteTolerance ?? SOLVER_TOLERANCES.rankAbsolute,
    relativeTolerance: options.rankRelativeTolerance ?? SOLVER_TOLERANCES.rankRelative,
  };
}

function estimateSystemRank(
  system: ComponentResidualSystem,
  configuration: readonly number[],
  options: ConstraintAnalysisOptions,
): MatrixRankEstimate {
  return estimateMatrixRank(componentJacobian(system, configuration), rankOptions(options));
}

function genericRankEstimate(
  system: ComponentResidualSystem,
  current: MatrixRankEstimate,
  options: ConstraintAnalysisOptions,
): MatrixRankEstimate {
  let best = current;
  const sampleCount = Math.max(0, Math.floor(
    options.expectedRankSamples ?? DEFAULT_EXPECTED_RANK_SAMPLES,
  ));
  for (let sample = 0; sample < sampleCount; sample += 1) {
    const configuration = [...system.initialConfiguration];
    for (const variable of system.variables) {
      if (variable.coordinate !== 'angle') continue;
      const bodyIndex = Math.floor(variable.index / 3);
      const direction = ((bodyIndex + sample) % 2) === 0 ? 1 : -1;
      configuration[variable.index] = (configuration[variable.index] as number) +
        direction * (0.271 + 0.113 * sample + 0.037 * bodyIndex);
    }
    const candidate = estimateSystemRank(system, configuration, options);
    if (candidate.rank > best.rank ||
        (candidate.rank === best.rank && candidate.rankMetric > best.rankMetric)) {
      best = candidate;
    }
  }
  return best;
}

function analyzeComponent(
  graph: ConstraintGraph,
  component: ConstraintGraphComponent,
  options: ConstraintAnalysisOptions,
): ConstraintComponentAnalysis {
  const passiveSystem = createComponentResidualSystem(graph, component, { includeActuator: false });
  const drivenSystem = createComponentResidualSystem(graph, component, { includeActuator: true });
  const passiveResidual = passiveSystem.evaluate(passiveSystem.initialConfiguration);
  const drivenResidual = drivenSystem.evaluate(drivenSystem.initialConfiguration);
  const passiveRank = estimateSystemRank(passiveSystem, passiveSystem.initialConfiguration, options);
  const drivenRank = estimateSystemRank(drivenSystem, drivenSystem.initialConfiguration, options);
  const expectedPassiveRank = genericRankEstimate(passiveSystem, passiveRank, options);
  const expectedDrivenRank = genericRankEstimate(drivenSystem, drivenRank, options);
  const variableCount = drivenSystem.variables.length;
  const residualCount = drivenSystem.scalarResiduals.length;
  const residualNorm = vectorNorm(drivenResidual);
  const passiveResidualNorm = vectorNorm(passiveResidual);
  const residualTolerance = options.residualTolerance ?? SOLVER_TOLERANCES.closure;
  const inconsistent = residualNorm > residualTolerance;
  const redundantConstraintCount = drivenRank.redundantRowCount;
  const overconstrained = residualCount > variableCount && drivenRank.rank === variableCount;
  const rankLoss = passiveRank.rank < expectedPassiveRank.rank ||
    drivenRank.rank < expectedDrivenRank.rank;
  const nearSingular = drivenRank.rank > 0 && expectedDrivenRank.rank > 0 &&
    drivenRank.rankMetric <=
      (options.rankRelativeTolerance ?? SOLVER_TOLERANCES.rankRelative) * NEAR_SINGULAR_RANK_FACTOR;
  const singular = rankLoss || nearSingular;
  const passiveDof = Math.max(0, variableCount - passiveRank.rank);
  const drivenDof = Math.max(0, variableCount - drivenRank.rank);
  const messages: string[] = [];

  if (!component.anchored) {
    messages.push(`Disconnected component · ${component.linkIds.length} link(s)`);
  }
  if (drivenDof > 0) {
    messages.push(`${drivenDof} unconstrained DOF remain with actuators prescribed`);
  }
  if (redundantConstraintCount > 0) {
    messages.push(`${redundantConstraintCount} redundant scalar constraint(s)`);
  }
  if (overconstrained) {
    messages.push(`Structurally overconstrained · ${residualCount} residuals / ${variableCount} variables`);
  }
  if (inconsistent) {
    messages.push(`Constraint residual ${residualNorm.toPrecision(4)} exceeds ${residualTolerance}`);
  }
  if (singular) {
    messages.push(
      `Jacobian rank ${drivenRank.rank}/${expectedDrivenRank.rank} at this configuration`,
    );
  }

  return {
    id: component.id,
    linkIds: [...component.linkIds],
    jointIds: [...component.jointIds],
    anchored: component.anchored,
    actuatorIds: [...component.actuatorIds],
    variableCount,
    residualCount,
    passiveResidualCount: passiveSystem.scalarResiduals.length,
    jacobianRank: drivenRank.rank,
    drivenJacobianRank: drivenRank.rank,
    passiveJacobianRank: passiveRank.rank,
    expectedPassiveJacobianRank: expectedPassiveRank.rank,
    expectedDrivenJacobianRank: expectedDrivenRank.rank,
    passiveRankMetric: passiveRank.rankMetric,
    drivenRankMetric: drivenRank.rankMetric,
    passiveDof,
    drivenDof,
    unresolvedLinkIds: [],
    redundantConstraintCount,
    overconstrained,
    inconsistent,
    singular,
    residualNorm,
    passiveResidualNorm,
    drivenResidualNorm: residualNorm,
    analyticSolveCount: 0,
    numericalFallbackUsed: false,
    messages,
  };
}

/** Computes local mobility and closure diagnostics for every topology component. */
export function analyzeConstraintGraph(
  graph: ConstraintGraph,
  options: ConstraintAnalysisOptions = {},
): ConstraintGraphDiagnostics {
  const components = connectedComponents(graph)
    .map((component) => analyzeComponent(graph, component, options));
  return {
    valid: components.every((component) => !component.inconsistent),
    components,
    disconnectedComponentIds: components
      .filter((component) => !component.anchored)
      .map((component) => component.id),
  };
}
