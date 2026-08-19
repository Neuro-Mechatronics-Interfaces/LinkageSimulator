import {
  LinearAlgebraError,
  centralDifferenceJacobian,
  dampedLeastSquaresStep,
  euclideanNorm,
  maximumAbsoluteValue,
  type ResidualEvaluator,
  type VariableProjector,
} from './linearAlgebra';

export type NumericalVariableKind = 'translation' | 'angle' | 'generic';

export const DEFAULT_NUMERICAL_MAX_ITERATIONS = 50;
export const DEFAULT_NUMERICAL_RESIDUAL_TOLERANCE = 1e-8;
export const DEFAULT_NUMERICAL_STEP_TOLERANCE = 1e-10;
export const DEFAULT_NUMERICAL_INITIAL_DAMPING = 1e-4;
export const DEFAULT_NUMERICAL_MINIMUM_DAMPING = 1e-12;
export const DEFAULT_NUMERICAL_MAXIMUM_DAMPING = 1e12;
export const DEFAULT_TRANSLATION_STEP_BOUND = 10;
export const DEFAULT_ANGULAR_STEP_BOUND = Math.PI / 12;
export const DEFAULT_GENERIC_STEP_BOUND = 1;
export const DEFAULT_TRANSLATION_FINITE_DIFFERENCE_STEP = 1e-5;
export const DEFAULT_ANGULAR_FINITE_DIFFERENCE_STEP = 1e-6;
export const DEFAULT_GENERIC_FINITE_DIFFERENCE_STEP = 1e-6;
export const DEFAULT_NUMERICAL_LINE_SEARCH_STEPS = 12;
export const DEFAULT_NUMERICAL_LINE_SEARCH_REDUCTION = 0.5;

export interface NumericalConstraintSolverOptions {
  /** Coordinate-wise projection onto joint limits or other feasible variable bounds. */
  project?: VariableProjector;
  variableKinds?: readonly NumericalVariableKind[];
  /** Overrides the kind-based step bound for every coordinate. */
  maxStepByVariable?: readonly number[];
  maxTranslationStep?: number;
  maxAngularStep?: number;
  maxGenericStep?: number;
  /** Overrides the kind-based central-difference step, globally or per coordinate. */
  finiteDifferenceStep?: number | readonly number[];
  maxIterations?: number;
  residualTolerance?: number;
  stepTolerance?: number;
  initialDamping?: number;
  minimumDamping?: number;
  maximumDamping?: number;
  dampingIncrease?: number;
  dampingDecrease?: number;
  maxLineSearchSteps?: number;
  lineSearchReduction?: number;
  linearAbsoluteTolerance?: number;
  linearRelativeTolerance?: number;
}

export type NumericalConstraintFailureReason =
  | 'invalid-input'
  | 'projection-failed'
  | 'non-finite-state'
  | 'residual-evaluation-failed'
  | 'non-finite-residual'
  | 'jacobian-failed'
  | 'linear-solve-failed'
  | 'stalled'
  | 'line-search-failed'
  | 'iteration-limit';

export interface NumericalConstraintSolverDiagnostics {
  iterations: number;
  residualEvaluations: number;
  acceptedSteps: number;
  rejectedSteps: number;
  lineSearchFailures: number;
  initialResidualNorm: number | null;
  finalResidualNorm: number | null;
  initialMaximumResidual: number | null;
  finalMaximumResidual: number | null;
  finalDamping: number;
  lastStepNorm: number;
  lastLinearRankMetric: number | null;
  /** Contains the initial norm followed by every accepted, strictly decreasing norm. */
  acceptedResidualNorms: number[];
}

export type NumericalConstraintSolveResult =
  | {
      kind: 'converged';
      variables: number[];
      residual: number[];
      diagnostics: NumericalConstraintSolverDiagnostics;
    }
  | {
      kind: 'failed';
      reason: NumericalConstraintFailureReason;
      message: string;
      /** Best finite, feasible iterate. Null only when no valid initial iterate existed. */
      variables: number[] | null;
      residual: number[] | null;
      diagnostics: NumericalConstraintSolverDiagnostics;
    };

interface ResolvedOptions {
  project?: VariableProjector;
  variableKinds: NumericalVariableKind[];
  maximumSteps: number[];
  finiteDifferenceSteps: number[];
  maxIterations: number;
  residualTolerance: number;
  stepTolerance: number;
  initialDamping: number;
  minimumDamping: number;
  maximumDamping: number;
  dampingIncrease: number;
  dampingDecrease: number;
  maxLineSearchSteps: number;
  lineSearchReduction: number;
  linearAbsoluteTolerance: number | undefined;
  linearRelativeTolerance: number | undefined;
}

class NumericalSolverError extends Error {
  constructor(
    readonly reason: NumericalConstraintFailureReason,
    message: string,
  ) {
    super(message);
    this.name = 'NumericalSolverError';
  }
}

const isFiniteNumber = (value: number): boolean => Number.isFinite(value);

function finiteNonNegative(value: number, name: string): number {
  if (!isFiniteNumber(value) || value < 0) {
    throw new NumericalSolverError('invalid-input', `${name} must be a finite non-negative number.`);
  }
  return value;
}

function finitePositive(value: number, name: string): number {
  if (!isFiniteNumber(value) || value <= 0) {
    throw new NumericalSolverError('invalid-input', `${name} must be a finite positive number.`);
  }
  return value;
}

function positiveInteger(value: number, name: string): number {
  if (!Number.isInteger(value) || value <= 0) {
    throw new NumericalSolverError('invalid-input', `${name} must be a positive integer.`);
  }
  return value;
}

function kindValue(
  kind: NumericalVariableKind,
  translation: number,
  angle: number,
  generic: number,
): number {
  if (kind === 'translation') return translation;
  if (kind === 'angle') return angle;
  return generic;
}

function resolvePerVariableValues(
  requested: number | readonly number[] | undefined,
  kinds: readonly NumericalVariableKind[],
  defaults: readonly [translation: number, angle: number, generic: number],
  name: string,
): number[] {
  const values = requested === undefined
    ? kinds.map((kind) => kindValue(kind, ...defaults))
    : typeof requested === 'number'
      ? Array<number>(kinds.length).fill(requested)
      : [...requested];
  if (values.length !== kinds.length) {
    throw new NumericalSolverError('invalid-input', `${name} must contain one value per variable.`);
  }
  return values.map((value, index) => finitePositive(value, `${name}[${index}]`));
}

function resolveOptions(variableCount: number, options: NumericalConstraintSolverOptions): ResolvedOptions {
  const variableKinds = options.variableKinds ? [...options.variableKinds] : Array<NumericalVariableKind>(variableCount).fill('generic');
  if (variableKinds.length !== variableCount) {
    throw new NumericalSolverError('invalid-input', 'variableKinds must contain one entry per variable.');
  }
  for (const kind of variableKinds) {
    if (kind !== 'translation' && kind !== 'angle' && kind !== 'generic') {
      throw new NumericalSolverError('invalid-input', `Unknown numerical variable kind: ${String(kind)}.`);
    }
  }

  const translationStep = finitePositive(
    options.maxTranslationStep ?? DEFAULT_TRANSLATION_STEP_BOUND,
    'maxTranslationStep',
  );
  const angularStep = finitePositive(options.maxAngularStep ?? DEFAULT_ANGULAR_STEP_BOUND, 'maxAngularStep');
  const genericStep = finitePositive(options.maxGenericStep ?? DEFAULT_GENERIC_STEP_BOUND, 'maxGenericStep');
  const maximumSteps = resolvePerVariableValues(
    options.maxStepByVariable,
    variableKinds,
    [translationStep, angularStep, genericStep],
    'maxStepByVariable',
  );
  const finiteDifferenceSteps = resolvePerVariableValues(
    options.finiteDifferenceStep,
    variableKinds,
    [
      DEFAULT_TRANSLATION_FINITE_DIFFERENCE_STEP,
      DEFAULT_ANGULAR_FINITE_DIFFERENCE_STEP,
      DEFAULT_GENERIC_FINITE_DIFFERENCE_STEP,
    ],
    'finiteDifferenceStep',
  );

  const maxIterations = positiveInteger(options.maxIterations ?? DEFAULT_NUMERICAL_MAX_ITERATIONS, 'maxIterations');
  const maxLineSearchSteps = positiveInteger(
    options.maxLineSearchSteps ?? DEFAULT_NUMERICAL_LINE_SEARCH_STEPS,
    'maxLineSearchSteps',
  );
  const residualTolerance = finiteNonNegative(
    options.residualTolerance ?? DEFAULT_NUMERICAL_RESIDUAL_TOLERANCE,
    'residualTolerance',
  );
  const stepTolerance = finiteNonNegative(
    options.stepTolerance ?? DEFAULT_NUMERICAL_STEP_TOLERANCE,
    'stepTolerance',
  );
  const minimumDamping = finitePositive(
    options.minimumDamping ?? DEFAULT_NUMERICAL_MINIMUM_DAMPING,
    'minimumDamping',
  );
  const maximumDamping = finitePositive(
    options.maximumDamping ?? DEFAULT_NUMERICAL_MAXIMUM_DAMPING,
    'maximumDamping',
  );
  if (maximumDamping < minimumDamping) {
    throw new NumericalSolverError('invalid-input', 'maximumDamping must not be smaller than minimumDamping.');
  }
  const initialDamping = Math.max(
    minimumDamping,
    Math.min(maximumDamping, finitePositive(
      options.initialDamping ?? DEFAULT_NUMERICAL_INITIAL_DAMPING,
      'initialDamping',
    )),
  );
  const dampingIncrease = finitePositive(options.dampingIncrease ?? 10, 'dampingIncrease');
  if (dampingIncrease <= 1) {
    throw new NumericalSolverError('invalid-input', 'dampingIncrease must be greater than one.');
  }
  const dampingDecrease = finitePositive(options.dampingDecrease ?? 0.3, 'dampingDecrease');
  if (dampingDecrease > 1) {
    throw new NumericalSolverError('invalid-input', 'dampingDecrease must not exceed one.');
  }
  const lineSearchReduction = finitePositive(
    options.lineSearchReduction ?? DEFAULT_NUMERICAL_LINE_SEARCH_REDUCTION,
    'lineSearchReduction',
  );
  if (lineSearchReduction >= 1) {
    throw new NumericalSolverError('invalid-input', 'lineSearchReduction must be smaller than one.');
  }
  if (options.linearAbsoluteTolerance !== undefined) {
    finiteNonNegative(options.linearAbsoluteTolerance, 'linearAbsoluteTolerance');
  }
  if (options.linearRelativeTolerance !== undefined) {
    finiteNonNegative(options.linearRelativeTolerance, 'linearRelativeTolerance');
  }

  return {
    project: options.project,
    variableKinds,
    maximumSteps,
    finiteDifferenceSteps,
    maxIterations,
    residualTolerance,
    stepTolerance,
    initialDamping,
    minimumDamping,
    maximumDamping,
    dampingIncrease,
    dampingDecrease,
    maxLineSearchSteps,
    lineSearchReduction,
    linearAbsoluteTolerance: options.linearAbsoluteTolerance,
    linearRelativeTolerance: options.linearRelativeTolerance,
  };
}

function emptyDiagnostics(): NumericalConstraintSolverDiagnostics {
  return {
    iterations: 0,
    residualEvaluations: 0,
    acceptedSteps: 0,
    rejectedSteps: 0,
    lineSearchFailures: 0,
    initialResidualNorm: null,
    finalResidualNorm: null,
    initialMaximumResidual: null,
    finalMaximumResidual: null,
    finalDamping: DEFAULT_NUMERICAL_INITIAL_DAMPING,
    lastStepNorm: 0,
    lastLinearRankMetric: null,
    acceptedResidualNorms: [],
  };
}

function failed(
  reason: NumericalConstraintFailureReason,
  message: string,
  variables: readonly number[] | null,
  residual: readonly number[] | null,
  diagnostics: NumericalConstraintSolverDiagnostics,
): NumericalConstraintSolveResult {
  return {
    kind: 'failed',
    reason,
    message,
    variables: variables ? [...variables] : null,
    residual: residual ? [...residual] : null,
    diagnostics: {
      ...diagnostics,
      acceptedResidualNorms: [...diagnostics.acceptedResidualNorms],
    },
  };
}

function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

/**
 * Bounded damped Gauss-Newton/Levenberg-Marquardt solve for a small dense
 * residual system. Residuals are assumed to already share a meaningful scale;
 * mechanism angle residuals should therefore be converted to length-equivalent
 * residuals by the constraint layer before calling this generic routine.
 */
export function solveBoundedDampedLeastSquares(
  initialVariables: readonly number[],
  evaluateResidual: ResidualEvaluator,
  options: NumericalConstraintSolverOptions = {},
): NumericalConstraintSolveResult {
  const diagnostics = emptyDiagnostics();
  let resolved: ResolvedOptions;
  try {
    for (let index = 0; index < initialVariables.length; index += 1) {
      if (!isFiniteNumber(initialVariables[index]!)) {
        throw new NumericalSolverError('invalid-input', `initialVariables[${index}] must be finite.`);
      }
    }
    resolved = resolveOptions(initialVariables.length, options);
    diagnostics.finalDamping = resolved.initialDamping;
  } catch (error) {
    const solverError = error instanceof NumericalSolverError
      ? error
      : new NumericalSolverError('invalid-input', errorMessage(error));
    return failed(solverError.reason, solverError.message, null, null, diagnostics);
  }

  let residualCount: number | undefined;
  const projectChecked = (variables: number[]): number[] => {
    for (let index = 0; index < variables.length; index += 1) {
      if (!isFiniteNumber(variables[index]!)) {
        throw new NumericalSolverError('non-finite-state', `Candidate variable ${index} is non-finite.`);
      }
    }
    if (!resolved.project) return [...variables];
    let projected: number[];
    try {
      projected = resolved.project([...variables]);
    } catch (error) {
      throw new NumericalSolverError('projection-failed', `Variable projection failed: ${errorMessage(error)}`);
    }
    if (!Array.isArray(projected) || projected.length !== initialVariables.length) {
      throw new NumericalSolverError('projection-failed', 'Variable projection changed the variable dimension.');
    }
    for (let index = 0; index < projected.length; index += 1) {
      if (!isFiniteNumber(projected[index]!)) {
        throw new NumericalSolverError('projection-failed', `Projected variable ${index} is non-finite.`);
      }
    }
    return [...projected];
  };
  const evaluateChecked = (variables: number[]): number[] => {
    diagnostics.residualEvaluations += 1;
    let residual: number[];
    try {
      residual = evaluateResidual([...variables]);
    } catch (error) {
      throw new NumericalSolverError(
        'residual-evaluation-failed',
        `Residual evaluation failed: ${errorMessage(error)}`,
      );
    }
    if (!Array.isArray(residual)) {
      throw new NumericalSolverError('residual-evaluation-failed', 'Residual evaluation did not return an array.');
    }
    residualCount ??= residual.length;
    if (residual.length !== residualCount) {
      throw new NumericalSolverError('residual-evaluation-failed', 'Residual evaluation changed its output dimension.');
    }
    for (let index = 0; index < residual.length; index += 1) {
      if (!isFiniteNumber(residual[index]!)) {
        throw new NumericalSolverError('non-finite-residual', `Residual ${index} is non-finite.`);
      }
    }
    return [...residual];
  };

  let variables: number[];
  let residual: number[];
  try {
    variables = projectChecked([...initialVariables]);
    residual = evaluateChecked(variables);
  } catch (error) {
    const solverError = error instanceof NumericalSolverError
      ? error
      : new NumericalSolverError('residual-evaluation-failed', errorMessage(error));
    return failed(solverError.reason, solverError.message, null, null, diagnostics);
  }

  let residualNorm = euclideanNorm(residual);
  let maximumResidual = maximumAbsoluteValue(residual);
  diagnostics.initialResidualNorm = residualNorm;
  diagnostics.finalResidualNorm = residualNorm;
  diagnostics.initialMaximumResidual = maximumResidual;
  diagnostics.finalMaximumResidual = maximumResidual;
  diagnostics.acceptedResidualNorms.push(residualNorm);
  if (residualNorm <= resolved.residualTolerance) {
    return { kind: 'converged', variables: [...variables], residual: [...residual], diagnostics };
  }

  let damping = resolved.initialDamping;
  for (let iteration = 1; iteration <= resolved.maxIterations; iteration += 1) {
    diagnostics.iterations = iteration;
    let jacobian: number[][];
    try {
      jacobian = centralDifferenceJacobian(
        evaluateChecked,
        variables,
        resolved.finiteDifferenceSteps,
        resolved.project ? projectChecked : undefined,
      );
    } catch (error) {
      if (error instanceof NumericalSolverError) {
        return failed(error.reason, error.message, variables, residual, diagnostics);
      }
      const message = error instanceof LinearAlgebraError ? error.message : errorMessage(error);
      return failed('jacobian-failed', `Jacobian evaluation failed: ${message}`, variables, residual, diagnostics);
    }

    let linearStep;
    try {
      linearStep = dampedLeastSquaresStep(jacobian, residual, {
        damping,
        absoluteTolerance: resolved.linearAbsoluteTolerance,
        relativeTolerance: resolved.linearRelativeTolerance,
      });
    } catch (error) {
      return failed(
        'linear-solve-failed',
        `Damped least-squares system failed: ${errorMessage(error)}`,
        variables,
        residual,
        diagnostics,
      );
    }
    diagnostics.lastLinearRankMetric = linearStep.diagnostics.rankMetric;
    if (linearStep.kind === 'singular') {
      return failed('linear-solve-failed', linearStep.message, variables, residual, diagnostics);
    }

    const boundedStep = linearStep.solution.map((value, index) => {
      const bound = resolved.maximumSteps[index]!;
      return Math.max(-bound, Math.min(bound, value));
    });
    diagnostics.lastStepNorm = euclideanNorm(boundedStep);
    if (diagnostics.lastStepNorm <= resolved.stepTolerance) {
      return failed(
        'stalled',
        `The bounded step norm ${diagnostics.lastStepNorm} is below the step tolerance.`,
        variables,
        residual,
        diagnostics,
      );
    }

    let acceptedVariables: number[] | undefined;
    let acceptedResidual: number[] | undefined;
    let acceptedNorm = residualNorm;
    let acceptedMaximum = maximumResidual;
    let acceptedStepNorm = 0;
    let lineScale = 1;
    for (let lineSearch = 0; lineSearch < resolved.maxLineSearchSteps; lineSearch += 1) {
      try {
        const candidate = projectChecked(variables.map((value, index) => value + lineScale * boundedStep[index]!));
        const candidateResidual = evaluateChecked(candidate);
        const candidateNorm = euclideanNorm(candidateResidual);
        if (candidateNorm < residualNorm) {
          acceptedVariables = candidate;
          acceptedResidual = candidateResidual;
          acceptedNorm = candidateNorm;
          acceptedMaximum = maximumAbsoluteValue(candidateResidual);
          acceptedStepNorm = euclideanNorm(candidate.map((value, index) => value - variables[index]!));
          break;
        }
      } catch (error) {
        if (error instanceof NumericalSolverError && error.reason === 'non-finite-residual') {
          // A non-finite trial is rejected; a smaller line-search step can remain valid.
        } else if (error instanceof NumericalSolverError) {
          return failed(error.reason, error.message, variables, residual, diagnostics);
        } else {
          return failed('residual-evaluation-failed', errorMessage(error), variables, residual, diagnostics);
        }
      }
      diagnostics.rejectedSteps += 1;
      lineScale *= resolved.lineSearchReduction;
    }

    if (!acceptedVariables || !acceptedResidual) {
      diagnostics.lineSearchFailures += 1;
      const increasedDamping = Math.min(resolved.maximumDamping, damping * resolved.dampingIncrease);
      if (increasedDamping <= damping) {
        diagnostics.finalDamping = damping;
        return failed(
          'line-search-failed',
          'No residual-decreasing bounded step was found at the maximum damping.',
          variables,
          residual,
          diagnostics,
        );
      }
      damping = increasedDamping;
      diagnostics.finalDamping = damping;
      continue;
    }

    variables = acceptedVariables;
    residual = acceptedResidual;
    residualNorm = acceptedNorm;
    maximumResidual = acceptedMaximum;
    diagnostics.acceptedSteps += 1;
    diagnostics.lastStepNorm = acceptedStepNorm;
    diagnostics.finalResidualNorm = residualNorm;
    diagnostics.finalMaximumResidual = maximumResidual;
    diagnostics.acceptedResidualNorms.push(residualNorm);
    damping = Math.max(resolved.minimumDamping, damping * resolved.dampingDecrease);
    diagnostics.finalDamping = damping;

    if (residualNorm <= resolved.residualTolerance) {
      return { kind: 'converged', variables: [...variables], residual: [...residual], diagnostics };
    }
    if (acceptedStepNorm <= resolved.stepTolerance) {
      return failed(
        'stalled',
        `The accepted step norm ${acceptedStepNorm} is below the step tolerance.`,
        variables,
        residual,
        diagnostics,
      );
    }
  }

  return failed(
    'iteration-limit',
    `The numerical solve did not converge within ${resolved.maxIterations} iterations.`,
    variables,
    residual,
    diagnostics,
  );
}
