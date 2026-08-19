import {
  DEFAULT_FINITE_DIFFERENCE_STEP,
  LinearAlgebraError,
  type LinearToleranceOptions,
  centralDifferenceJacobian,
  dampedLeastSquaresStep,
  estimateMatrixRank,
  euclideanNorm,
  solveLinearSystem,
} from './linearAlgebra';
import { SOLVER_TOLERANCES } from './solverTolerances';

export type DesignResidualEvaluator = (
  configuration: readonly number[],
  design: readonly number[],
) => number[];

export interface ConstraintSensitivityProblem {
  /** A converged configuration q at which r(q, d) = 0 is linearized. */
  configuration: readonly number[];
  /** Design coordinates d, such as link lengths or local attachment coordinates. */
  design: readonly number[];
  evaluateResidual: DesignResidualEvaluator;
}

export interface ConstraintSensitivityOptions extends LinearToleranceOptions {
  configurationStep?: number | readonly number[];
  designStep?: number | readonly number[];
  /** Applied only to rectangular, overdetermined Jacobians. Zero is exact least squares. */
  damping?: number;
  /** Relative tolerance for ||Jq S + Jd|| column by column. */
  linearizedResidualTolerance?: number;
  /** Maximum residual norm at the nominal configuration. */
  baselineResidualTolerance?: number;
}

export interface ConstraintSensitivityDiagnostics {
  configurationCount: number;
  designCount: number;
  residualCount: number;
  configurationRank: number | null;
  configurationNullity: number | null;
  redundantResidualCount: number | null;
  rankMetric: number | null;
  baselineResidualNorm: number | null;
  maximumLinearizedResidualNorm: number | null;
  damping: number;
}

export interface SolvedConstraintSensitivity {
  kind: 'solved';
  /** Jq = partial r / partial q, stored as residual rows by configuration columns. */
  configurationJacobian: number[][];
  /** Jd = partial r / partial d, stored as residual rows by design columns. */
  designJacobian: number[][];
  /** S = dq/dd, stored as configuration rows by design columns. */
  sensitivity: number[][];
  diagnostics: ConstraintSensitivityDiagnostics;
}

export interface FailedConstraintSensitivity {
  kind:
    | 'invalid'
    | 'not-converged'
    | 'underdetermined'
    | 'rank-deficient'
    | 'linear-solve-failed'
    | 'inconsistent-linearization';
  message: string;
  diagnostics: ConstraintSensitivityDiagnostics;
}

export type ConstraintSensitivityResult =
  | SolvedConstraintSensitivity
  | FailedConstraintSensitivity;

export interface ObjectiveGradientResult {
  kind: 'solved';
  /** Total derivative d objective / dd. */
  gradient: number[];
  diagnostics: {
    configurationGradient: number[];
    directDesignGradient: number[];
  };
}

export interface FailedObjectiveGradient {
  kind: 'invalid';
  message: string;
}

const DEFAULT_DAMPING = 0;
const DEFAULT_LINEARIZED_RESIDUAL_TOLERANCE = 1e-6;

function finiteVector(values: readonly number[], label: string): number[] {
  const copy = [...values];
  for (let index = 0; index < copy.length; index += 1) {
    if (!Number.isFinite(copy[index]!)) {
      throw new LinearAlgebraError(`${label}[${index}] must be finite.`);
    }
  }
  return copy;
}

function finiteNonNegative(value: number, label: string): number {
  if (!Number.isFinite(value) || value < 0) {
    throw new LinearAlgebraError(`${label} must be a finite non-negative number.`);
  }
  return value;
}

function initialDiagnostics(
  configurationCount: number,
  designCount: number,
  damping: number,
): ConstraintSensitivityDiagnostics {
  return {
    configurationCount,
    designCount,
    residualCount: 0,
    configurationRank: null,
    configurationNullity: null,
    redundantResidualCount: null,
    rankMetric: null,
    baselineResidualNorm: null,
    maximumLinearizedResidualNorm: null,
    damping,
  };
}

function evaluateFiniteResidual(
  evaluate: DesignResidualEvaluator,
  configuration: readonly number[],
  design: readonly number[],
): number[] {
  const residual = evaluate([...configuration], [...design]);
  if (!Array.isArray(residual)) {
    throw new LinearAlgebraError('Residual evaluation must return an array.');
  }
  return finiteVector(residual, 'residual');
}

function matrixVectorProduct(matrix: readonly (readonly number[])[], vector: readonly number[]): number[] {
  return matrix.map((row) => {
    let value = 0;
    for (let column = 0; column < vector.length; column += 1) {
      value += row[column]! * vector[column]!;
    }
    return value;
  });
}

/**
 * Differentiates a converged constraint system by the implicit-function relation
 *
 *   Jq (dq/dd) + Jd = 0.
 *
 * Jq and Jd are central-difference Jacobians. A square Jq is solved directly;
 * a tall, full-column-rank Jq uses the existing damped least-squares solver.
 * Systems with free configuration coordinates are deliberately rejected because
 * dq/dd is not unique without a gauge or an additional optimization criterion.
 */
export function computeConstraintSensitivity(
  problem: ConstraintSensitivityProblem,
  options: ConstraintSensitivityOptions = {},
): ConstraintSensitivityResult {
  const configurationCount = problem.configuration.length;
  const designCount = problem.design.length;
  let damping = DEFAULT_DAMPING;
  let diagnostics = initialDiagnostics(configurationCount, designCount, damping);

  try {
    damping = finiteNonNegative(options.damping ?? DEFAULT_DAMPING, 'damping');
    const linearizedResidualTolerance = finiteNonNegative(
      options.linearizedResidualTolerance ?? DEFAULT_LINEARIZED_RESIDUAL_TOLERANCE,
      'linearizedResidualTolerance',
    );
    const baselineResidualTolerance = finiteNonNegative(
      options.baselineResidualTolerance ?? SOLVER_TOLERANCES.numericalResidual,
      'baselineResidualTolerance',
    );
    diagnostics = initialDiagnostics(configurationCount, designCount, damping);
    const configuration = finiteVector(problem.configuration, 'configuration');
    const design = finiteVector(problem.design, 'design');
    const baselineResidual = evaluateFiniteResidual(
      problem.evaluateResidual,
      configuration,
      design,
    );
    diagnostics.residualCount = baselineResidual.length;
    diagnostics.baselineResidualNorm = euclideanNorm(baselineResidual);

    if (diagnostics.baselineResidualNorm > baselineResidualTolerance) {
      return {
        kind: 'not-converged',
        message: `The nominal residual norm ${diagnostics.baselineResidualNorm} exceeds ${baselineResidualTolerance}; solve r(q, d) = 0 before differentiating.`,
        diagnostics,
      };
    }

    if (configurationCount === 0) {
      return {
        kind: 'underdetermined',
        message: 'At least one configuration coordinate is required for implicit sensitivity.',
        diagnostics,
      };
    }

    const configurationJacobian = centralDifferenceJacobian(
      (candidate) => evaluateFiniteResidual(problem.evaluateResidual, candidate, design),
      configuration,
      options.configurationStep ?? DEFAULT_FINITE_DIFFERENCE_STEP,
    );
    const designJacobian = centralDifferenceJacobian(
      (candidate) => evaluateFiniteResidual(problem.evaluateResidual, configuration, candidate),
      design,
      options.designStep ?? DEFAULT_FINITE_DIFFERENCE_STEP,
    );

    if (configurationJacobian.length !== baselineResidual.length
      || designJacobian.length !== baselineResidual.length) {
      throw new LinearAlgebraError('Residual evaluation changed its output dimension.');
    }

    const rank = estimateMatrixRank(configurationJacobian, options);
    diagnostics.configurationRank = rank.rank;
    diagnostics.configurationNullity = rank.nullity;
    diagnostics.redundantResidualCount = rank.redundantRowCount;
    diagnostics.rankMetric = rank.rankMetric;

    if (baselineResidual.length < configurationCount) {
      return {
        kind: 'underdetermined',
        message: `The configuration Jacobian has ${baselineResidual.length} residual rows for ${configurationCount} configuration coordinates.`,
        diagnostics,
      };
    }
    if (rank.rank < configurationCount) {
      return {
        kind: 'rank-deficient',
        message: `The configuration Jacobian rank is ${rank.rank}/${configurationCount}; dq/dd is not unique.`,
        diagnostics,
      };
    }

    const sensitivity = Array.from(
      { length: configurationCount },
      () => Array<number>(designCount).fill(0),
    );
    let maximumLinearizedResidualNorm = 0;

    for (let designColumn = 0; designColumn < designCount; designColumn += 1) {
      const designDerivative = designJacobian.map((row) => row[designColumn]!);
      const solve = baselineResidual.length === configurationCount
        ? solveLinearSystem(
            configurationJacobian,
            designDerivative.map((value) => -value),
            options,
          )
        : dampedLeastSquaresStep(configurationJacobian, designDerivative, {
            damping,
            absoluteTolerance: options.absoluteTolerance,
            relativeTolerance: options.relativeTolerance,
          });
      if (solve.kind !== 'solved') {
        return {
          kind: 'linear-solve-failed',
          message: `Sensitivity column ${designColumn} could not be solved: ${solve.message}`,
          diagnostics,
        };
      }

      for (let configurationRow = 0; configurationRow < configurationCount; configurationRow += 1) {
        sensitivity[configurationRow]![designColumn] = solve.solution[configurationRow]!;
      }
      const closureDerivative = matrixVectorProduct(configurationJacobian, solve.solution)
        .map((value, row) => value + designDerivative[row]!);
      const closureNorm = euclideanNorm(closureDerivative);
      maximumLinearizedResidualNorm = Math.max(maximumLinearizedResidualNorm, closureNorm);
      const derivativeScale = Math.max(1, euclideanNorm(designDerivative));
      if (closureNorm > linearizedResidualTolerance * derivativeScale) {
        diagnostics.maximumLinearizedResidualNorm = maximumLinearizedResidualNorm;
        return {
          kind: 'inconsistent-linearization',
          message: `Sensitivity column ${designColumn} leaves a linearized constraint residual of ${closureNorm}.`,
          diagnostics,
        };
      }
    }

    diagnostics.maximumLinearizedResidualNorm = maximumLinearizedResidualNorm;
    return {
      kind: 'solved',
      configurationJacobian,
      designJacobian,
      sensitivity,
      diagnostics,
    };
  } catch (error) {
    return {
      kind: 'invalid',
      message: error instanceof Error ? error.message : 'Sensitivity evaluation failed.',
      diagnostics,
    };
  }
}

/**
 * Applies the chain rule
 * d objective/dd = partial objective/partial d + (dq/dd)^T partial objective/partial q.
 */
export function composeObjectiveDesignGradient(
  sensitivity: readonly (readonly number[])[],
  configurationGradient: readonly number[],
  directDesignGradient: readonly number[],
): ObjectiveGradientResult | FailedObjectiveGradient {
  try {
    const configurationPartial = finiteVector(configurationGradient, 'configurationGradient');
    const designPartial = finiteVector(directDesignGradient, 'directDesignGradient');
    if (sensitivity.length !== configurationPartial.length) {
      throw new LinearAlgebraError('Sensitivity row count must match configurationGradient.');
    }
    const gradient = [...designPartial];
    for (let row = 0; row < sensitivity.length; row += 1) {
      const sensitivityRow = finiteVector(sensitivity[row]!, `sensitivity[${row}]`);
      if (sensitivityRow.length !== gradient.length) {
        throw new LinearAlgebraError('Every sensitivity row must match directDesignGradient.');
      }
      for (let column = 0; column < gradient.length; column += 1) {
        gradient[column] = gradient[column]! + sensitivityRow[column]! * configurationPartial[row]!;
      }
    }
    finiteVector(gradient, 'objectiveDesignGradient');
    return {
      kind: 'solved',
      gradient,
      diagnostics: {
        configurationGradient: configurationPartial,
        directDesignGradient: designPartial,
      },
    };
  } catch (error) {
    return {
      kind: 'invalid',
      message: error instanceof Error ? error.message : 'Objective gradient composition failed.',
    };
  }
}
