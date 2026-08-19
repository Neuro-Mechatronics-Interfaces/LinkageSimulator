export type NumericMatrix = readonly (readonly number[])[];

export const DEFAULT_LINEAR_ABSOLUTE_TOLERANCE = 1e-12;
export const DEFAULT_LINEAR_RELATIVE_TOLERANCE = 1e-9;
export const DEFAULT_FINITE_DIFFERENCE_STEP = 1e-6;
export const DEFAULT_DAMPING_DIAGONAL_FLOOR = 1;

export interface LinearToleranceOptions {
  absoluteTolerance?: number;
  relativeTolerance?: number;
}

export interface MatrixRankEstimate {
  rank: number;
  rowCount: number;
  columnCount: number;
  nullity: number;
  redundantRowCount: number;
  pivotColumns: number[];
  pivotMagnitudes: number[];
  scaledPivots: number[];
  /** Smallest accepted pivot divided by its original row scale; zero when rank is zero. */
  rankMetric: number;
  absoluteTolerance: number;
  relativeTolerance: number;
}

export interface LinearSolveDiagnostics {
  rank: number;
  pivotMagnitudes: number[];
  scaledPivots: number[];
  rankMetric: number;
}

export type LinearSystemSolveResult =
  | {
      kind: 'solved';
      solution: number[];
      diagnostics: LinearSolveDiagnostics;
    }
  | {
      kind: 'singular';
      diagnostics: LinearSolveDiagnostics;
      message: string;
    };

export interface DampedLeastSquaresOptions extends LinearToleranceOptions {
  damping: number;
  diagonalFloor?: number;
}

export class LinearAlgebraError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'LinearAlgebraError';
  }
}

const isFiniteNumber = (value: number): boolean => Number.isFinite(value);

function validateTolerance(value: number, name: string): number {
  if (!isFiniteNumber(value) || value < 0) {
    throw new LinearAlgebraError(`${name} must be a finite non-negative number.`);
  }
  return value;
}

function tolerances(options: LinearToleranceOptions): { absolute: number; relative: number } {
  return {
    absolute: validateTolerance(
      options.absoluteTolerance ?? DEFAULT_LINEAR_ABSOLUTE_TOLERANCE,
      'absoluteTolerance',
    ),
    relative: validateTolerance(
      options.relativeTolerance ?? DEFAULT_LINEAR_RELATIVE_TOLERANCE,
      'relativeTolerance',
    ),
  };
}

function validateVector(values: readonly number[], label: string): void {
  for (let index = 0; index < values.length; index += 1) {
    if (!isFiniteNumber(values[index]!)) {
      throw new LinearAlgebraError(`${label}[${index}] must be finite.`);
    }
  }
}

function validateMatrix(matrix: NumericMatrix, label: string): { rows: number; columns: number } {
  const rows = matrix.length;
  const columns = matrix[0]?.length ?? 0;
  for (let row = 0; row < rows; row += 1) {
    const values = matrix[row]!;
    if (values.length !== columns) {
      throw new LinearAlgebraError(`${label} must be rectangular.`);
    }
    validateVector(values, `${label}[${row}]`);
  }
  return { rows, columns };
}

function rowMaximum(row: readonly number[]): number {
  let maximum = 0;
  for (const value of row) maximum = Math.max(maximum, Math.abs(value));
  return maximum;
}

function acceptedPivotThreshold(rowScale: number, absolute: number, relative: number): number {
  return absolute + relative * rowScale;
}

function rankMetric(scaledPivots: readonly number[]): number {
  return scaledPivots.length === 0 ? 0 : Math.min(...scaledPivots);
}

/**
 * Estimates rank by Gaussian elimination with scaled partial row pivoting.
 * A pivot is accepted relative to the scale of its original row so differently
 * scaled, independent constraint equations are not discarded by one global cutoff.
 */
export function estimateMatrixRank(
  matrix: NumericMatrix,
  options: LinearToleranceOptions = {},
): MatrixRankEstimate {
  const { rows, columns } = validateMatrix(matrix, 'matrix');
  const { absolute, relative } = tolerances(options);
  const work = matrix.map((row) => [...row]);
  const rowScales = work.map(rowMaximum);
  const pivotColumns: number[] = [];
  const pivotMagnitudes: number[] = [];
  const scaledPivots: number[] = [];
  let pivotRow = 0;

  for (let column = 0; column < columns && pivotRow < rows; column += 1) {
    let selectedRow = -1;
    let selectedRatio = -1;
    for (let row = pivotRow; row < rows; row += 1) {
      const magnitude = Math.abs(work[row]![column]!);
      const scale = rowScales[row]!;
      if (magnitude <= acceptedPivotThreshold(scale, absolute, relative)) continue;
      const ratio = scale > 0 ? magnitude / scale : magnitude;
      if (ratio > selectedRatio) {
        selectedRatio = ratio;
        selectedRow = row;
      }
    }
    if (selectedRow < 0) continue;

    if (selectedRow !== pivotRow) {
      [work[pivotRow], work[selectedRow]] = [work[selectedRow]!, work[pivotRow]!];
      [rowScales[pivotRow], rowScales[selectedRow]] = [rowScales[selectedRow]!, rowScales[pivotRow]!];
    }

    const pivot = work[pivotRow]![column]!;
    const pivotMagnitude = Math.abs(pivot);
    const scale = rowScales[pivotRow]!;
    pivotColumns.push(column);
    pivotMagnitudes.push(pivotMagnitude);
    scaledPivots.push(scale > 0 ? pivotMagnitude / scale : pivotMagnitude);

    for (let row = pivotRow + 1; row < rows; row += 1) {
      const factor = work[row]![column]! / pivot;
      work[row]![column] = 0;
      for (let entry = column + 1; entry < columns; entry += 1) {
        work[row]![entry] = work[row]![entry]! - factor * work[pivotRow]![entry]!;
      }
    }
    pivotRow += 1;
  }

  const rank = pivotColumns.length;
  return {
    rank,
    rowCount: rows,
    columnCount: columns,
    nullity: Math.max(0, columns - rank),
    redundantRowCount: Math.max(0, rows - rank),
    pivotColumns,
    pivotMagnitudes,
    scaledPivots,
    rankMetric: rankMetric(scaledPivots),
    absoluteTolerance: absolute,
    relativeTolerance: relative,
  };
}

/** Solves a small dense square system using scaled partial pivoting. */
export function solveLinearSystem(
  matrix: NumericMatrix,
  values: readonly number[],
  options: LinearToleranceOptions = {},
): LinearSystemSolveResult {
  const { rows, columns } = validateMatrix(matrix, 'matrix');
  validateVector(values, 'values');
  if (rows !== columns || values.length !== rows) {
    throw new LinearAlgebraError('solveLinearSystem requires a square matrix and one value per row.');
  }
  const { absolute, relative } = tolerances(options);
  if (rows === 0) {
    return {
      kind: 'solved',
      solution: [],
      diagnostics: { rank: 0, pivotMagnitudes: [], scaledPivots: [], rankMetric: 0 },
    };
  }

  const work = matrix.map((row, index) => [...row, values[index]!]);
  const rowScales = matrix.map(rowMaximum);
  const pivotMagnitudes: number[] = [];
  const scaledPivots: number[] = [];

  for (let column = 0; column < columns; column += 1) {
    let selectedRow = -1;
    let selectedRatio = -1;
    for (let row = column; row < rows; row += 1) {
      const magnitude = Math.abs(work[row]![column]!);
      const scale = rowScales[row]!;
      if (magnitude <= acceptedPivotThreshold(scale, absolute, relative)) continue;
      const ratio = scale > 0 ? magnitude / scale : magnitude;
      if (ratio > selectedRatio) {
        selectedRatio = ratio;
        selectedRow = row;
      }
    }

    if (selectedRow < 0) {
      const rank = estimateMatrixRank(matrix, options);
      const diagnostics = {
        rank: rank.rank,
        pivotMagnitudes: rank.pivotMagnitudes,
        scaledPivots: rank.scaledPivots,
        rankMetric: rank.rankMetric,
      };
      return {
        kind: 'singular',
        diagnostics,
        message: `No numerically independent pivot was available in column ${column}.`,
      };
    }

    if (selectedRow !== column) {
      [work[column], work[selectedRow]] = [work[selectedRow]!, work[column]!];
      [rowScales[column], rowScales[selectedRow]] = [rowScales[selectedRow]!, rowScales[column]!];
    }

    const pivot = work[column]![column]!;
    const pivotMagnitude = Math.abs(pivot);
    const scale = rowScales[column]!;
    pivotMagnitudes.push(pivotMagnitude);
    scaledPivots.push(scale > 0 ? pivotMagnitude / scale : pivotMagnitude);

    for (let row = column + 1; row < rows; row += 1) {
      const factor = work[row]![column]! / pivot;
      work[row]![column] = 0;
      for (let entry = column + 1; entry <= columns; entry += 1) {
        work[row]![entry] = work[row]![entry]! - factor * work[column]![entry]!;
      }
    }
  }

  const solution = Array<number>(columns).fill(0);
  for (let row = rows - 1; row >= 0; row -= 1) {
    let rightHandSide = work[row]![columns]!;
    for (let column = row + 1; column < columns; column += 1) {
      rightHandSide -= work[row]![column]! * solution[column]!;
    }
    solution[row] = rightHandSide / work[row]![row]!;
  }
  validateVector(solution, 'solution');
  return {
    kind: 'solved',
    solution,
    diagnostics: {
      rank: rows,
      pivotMagnitudes,
      scaledPivots,
      rankMetric: rankMetric(scaledPivots),
    },
  };
}

export type ResidualEvaluator = (variables: number[]) => number[];
export type VariableProjector = (variables: number[]) => number[];

function finiteDifferenceSteps(
  variables: readonly number[],
  requested: number | readonly number[],
): number[] {
  const steps = typeof requested === 'number'
    ? Array<number>(variables.length).fill(requested)
    : [...requested];
  if (steps.length !== variables.length) {
    throw new LinearAlgebraError('Finite-difference step count must match the variable count.');
  }
  for (let index = 0; index < steps.length; index += 1) {
    if (!isFiniteNumber(steps[index]!) || steps[index]! <= 0) {
      throw new LinearAlgebraError(`finiteDifferenceStep[${index}] must be finite and positive.`);
    }
  }
  return steps;
}

function projectedVariables(
  variables: number[],
  expectedLength: number,
  project: VariableProjector | undefined,
): number[] {
  const projected = project ? project([...variables]) : variables;
  if (!Array.isArray(projected) || projected.length !== expectedLength) {
    throw new LinearAlgebraError('Variable projection must return one value per input variable.');
  }
  validateVector(projected, 'projectedVariables');
  return [...projected];
}

function evaluatedResidual(
  evaluate: ResidualEvaluator,
  variables: number[],
  expectedLength: number | undefined,
): number[] {
  const residual = evaluate([...variables]);
  if (!Array.isArray(residual)) {
    throw new LinearAlgebraError('Residual evaluation must return an array.');
  }
  if (expectedLength !== undefined && residual.length !== expectedLength) {
    throw new LinearAlgebraError('Residual evaluation changed its output dimension.');
  }
  validateVector(residual, 'residual');
  return [...residual];
}

/**
 * Computes a central finite-difference Jacobian. When a coordinate-wise bounds
 * projector clamps one side of a perturbation, the actual projected span is used,
 * naturally yielding a one-sided difference at that active bound.
 */
export function centralDifferenceJacobian(
  evaluate: ResidualEvaluator,
  variables: readonly number[],
  step: number | readonly number[] = DEFAULT_FINITE_DIFFERENCE_STEP,
  project?: VariableProjector,
): number[][] {
  validateVector(variables, 'variables');
  const steps = finiteDifferenceSteps(variables, step);
  if (variables.length === 0) {
    return evaluatedResidual(evaluate, [], undefined).map(() => []);
  }

  let jacobian: number[][] | undefined;
  let residualCount: number | undefined;
  for (let column = 0; column < variables.length; column += 1) {
    const plus = [...variables];
    const minus = [...variables];
    plus[column] = plus[column]! + steps[column]!;
    minus[column] = minus[column]! - steps[column]!;
    const projectedPlus = projectedVariables(plus, variables.length, project);
    const projectedMinus = projectedVariables(minus, variables.length, project);
    const plusResidual = evaluatedResidual(evaluate, projectedPlus, residualCount);
    residualCount ??= plusResidual.length;
    const minusResidual = evaluatedResidual(evaluate, projectedMinus, residualCount);
    jacobian ??= Array.from({ length: residualCount }, () => Array<number>(variables.length).fill(0));

    const span = projectedPlus[column]! - projectedMinus[column]!;
    const scale = Math.max(1, Math.abs(projectedPlus[column]!), Math.abs(projectedMinus[column]!));
    if (Math.abs(span) <= Number.EPSILON * scale * 8) continue;
    for (let row = 0; row < residualCount; row += 1) {
      const derivative = (plusResidual[row]! - minusResidual[row]!) / span;
      if (!isFiniteNumber(derivative)) {
        throw new LinearAlgebraError(`Jacobian entry [${row}, ${column}] is non-finite.`);
      }
      jacobian[row]![column] = derivative;
    }
  }
  return jacobian ?? [];
}

/** Forms and solves (J^T J + lambda D) step = -J^T residual. */
export function dampedLeastSquaresStep(
  jacobian: NumericMatrix,
  residual: readonly number[],
  options: DampedLeastSquaresOptions,
): LinearSystemSolveResult {
  const { rows, columns } = validateMatrix(jacobian, 'jacobian');
  validateVector(residual, 'residual');
  if (residual.length !== rows) {
    throw new LinearAlgebraError('Residual count must match the Jacobian row count.');
  }
  const damping = options.damping;
  if (!isFiniteNumber(damping) || damping < 0) {
    throw new LinearAlgebraError('damping must be a finite non-negative number.');
  }
  const diagonalFloor = options.diagonalFloor ?? DEFAULT_DAMPING_DIAGONAL_FLOOR;
  if (!isFiniteNumber(diagonalFloor) || diagonalFloor <= 0) {
    throw new LinearAlgebraError('diagonalFloor must be finite and positive.');
  }

  const normalMatrix = Array.from({ length: columns }, () => Array<number>(columns).fill(0));
  const normalValues = Array<number>(columns).fill(0);
  for (let row = 0; row < rows; row += 1) {
    for (let column = 0; column < columns; column += 1) {
      const coefficient = jacobian[row]![column]!;
      normalValues[column] = normalValues[column]! - coefficient * residual[row]!;
      for (let other = column; other < columns; other += 1) {
        normalMatrix[column]![other] = normalMatrix[column]![other]! + coefficient * jacobian[row]![other]!;
      }
    }
  }
  for (let row = 0; row < columns; row += 1) {
    for (let column = 0; column < row; column += 1) {
      normalMatrix[row]![column] = normalMatrix[column]![row]!;
    }
    const diagonalScale = Math.max(normalMatrix[row]![row]!, diagonalFloor);
    normalMatrix[row]![row] = normalMatrix[row]![row]! + damping * diagonalScale;
  }
  return solveLinearSystem(normalMatrix, normalValues, options);
}

export function euclideanNorm(values: readonly number[]): number {
  validateVector(values, 'values');
  const norm = Math.hypot(...values);
  if (!isFiniteNumber(norm)) {
    throw new LinearAlgebraError('The Euclidean norm overflowed for finite input values.');
  }
  return norm;
}

export function maximumAbsoluteValue(values: readonly number[]): number {
  validateVector(values, 'values');
  let maximum = 0;
  for (const value of values) maximum = Math.max(maximum, Math.abs(value));
  return maximum;
}
