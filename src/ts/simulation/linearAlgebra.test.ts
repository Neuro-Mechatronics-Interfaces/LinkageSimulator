import { describe, expect, it } from 'vitest';
import {
  LinearAlgebraError,
  centralDifferenceJacobian,
  dampedLeastSquaresStep,
  estimateMatrixRank,
  euclideanNorm,
  solveLinearSystem,
} from './linearAlgebra';

describe('small dense linear algebra', () => {
  it('solves a finite square system with scaled pivoting', () => {
    const result = solveLinearSystem(
      [
        [1e-8, 1],
        [2, 3],
      ],
      [-3.000000005, -10],
    );
    expect(result.kind).toBe('solved');
    if (result.kind !== 'solved') return;
    expect(result.solution[0]).toBeCloseTo(-0.5, 9);
    expect(result.solution[1]).toBeCloseTo(-3, 9);
    expect(result.diagnostics.rank).toBe(2);
    expect(result.diagnostics.rankMetric).toBeGreaterThan(0);
  });

  it('reports a singular system without manufacturing a solution', () => {
    const result = solveLinearSystem(
      [
        [1, 2],
        [2, 4],
      ],
      [3, 6],
    );
    expect(result.kind).toBe('singular');
    if (result.kind !== 'singular') return;
    expect(result.diagnostics.rank).toBe(1);
    expect(result.message).toContain('column 1');
  });

  it('reports the full rank diagnostic when a singular system has a leading zero column', () => {
    const result = solveLinearSystem(
      [
        [0, 1],
        [0, 0],
      ],
      [1, 0],
    );
    expect(result.kind).toBe('singular');
    if (result.kind !== 'singular') return;
    expect(result.diagnostics.rank).toBe(1);
    expect(result.diagnostics.pivotMagnitudes).toEqual([1]);
  });

  it('distinguishes independent scaling from a nearly redundant row', () => {
    const differentlyScaled = estimateMatrixRank([
      [1e9, 0],
      [0, 1e-6],
    ]);
    expect(differentlyScaled.rank).toBe(2);
    expect(differentlyScaled.nullity).toBe(0);

    const redundant = estimateMatrixRank([
      [1, 1],
      [2, 2 + 1e-12],
      [3, 3],
    ]);
    expect(redundant.rank).toBe(1);
    expect(redundant.redundantRowCount).toBe(2);
    expect(redundant.pivotColumns).toEqual([0]);
    expect(redundant.rankMetric).toBeGreaterThan(0);
  });

  it('computes an accurate central-difference Jacobian with per-variable steps', () => {
    const point = [2, 0.4];
    const jacobian = centralDifferenceJacobian(
      ([x, y]) => [x! ** 2 + 3 * y!, Math.sin(y!)],
      point,
      [1e-5, 1e-6],
    );
    expect(jacobian[0]![0]).toBeCloseTo(4, 7);
    expect(jacobian[0]![1]).toBeCloseTo(3, 7);
    expect(jacobian[1]![0]).toBeCloseTo(0, 10);
    expect(jacobian[1]![1]).toBeCloseTo(Math.cos(point[1]!), 7);
  });

  it('uses the actual projected perturbation span at an active bound', () => {
    const project = ([value]: number[]): number[] => [Math.max(0, Math.min(1, value!))];
    const jacobian = centralDifferenceJacobian(([value]) => [3 * value!], [0], 1e-5, project);
    expect(jacobian).toHaveLength(1);
    expect(jacobian[0]![0]).toBeCloseTo(3, 9);
  });

  it('forms a descent step from the damped normal equations', () => {
    const result = dampedLeastSquaresStep(
      [
        [1, 0],
        [0, 1],
      ],
      [-2, 3],
      { damping: 1e-3 },
    );
    expect(result.kind).toBe('solved');
    if (result.kind !== 'solved') return;
    expect(result.solution[0]).toBeGreaterThan(1.9);
    expect(result.solution[1]).toBeLessThan(-2.9);
  });

  it('damps an unconstrained Jacobian column without a singular normal solve', () => {
    const result = dampedLeastSquaresStep([[1, 0]], [-2], { damping: 1e-3 });
    expect(result.kind).toBe('solved');
    if (result.kind !== 'solved') return;
    expect(result.solution[0]).toBeGreaterThan(1.9);
    expect(result.solution[1]).toBe(0);
  });

  it('rejects ragged and non-finite inputs explicitly', () => {
    expect(() => estimateMatrixRank([[1], [1, 2]])).toThrow(LinearAlgebraError);
    expect(() => centralDifferenceJacobian(() => [Number.NaN], [0])).toThrow(/finite/);
    expect(() => euclideanNorm([Number.MAX_VALUE, Number.MAX_VALUE])).toThrow(/overflowed/);
  });
});
