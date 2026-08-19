import { describe, expect, it } from 'vitest';
import { solveBoundedDampedLeastSquares } from './numericalConstraintSolver';

describe('bounded damped least-squares solver', () => {
  it('converges deterministically on a coupled residual system', () => {
    const solve = () => solveBoundedDampedLeastSquares(
      [0, 0],
      ([x, y]) => [x! + 2 * y! - 5, 3 * x! - y! - 4],
      { variableKinds: ['translation', 'translation'] },
    );
    const first = solve();
    const second = solve();
    expect(first.kind).toBe('converged');
    expect(second.kind).toBe('converged');
    if (first.kind !== 'converged' || second.kind !== 'converged') return;
    expect(first.variables[0]).toBeCloseTo(13 / 7, 8);
    expect(first.variables[1]).toBeCloseTo(11 / 7, 8);
    expect(first.variables).toEqual(second.variables);
    expect(first.diagnostics.acceptedResidualNorms).toEqual(second.diagnostics.acceptedResidualNorms);
  });

  it('backs off an excessive nonlinear Gauss-Newton step until residual decreases', () => {
    const result = solveBoundedDampedLeastSquares(
      [0.1],
      ([x]) => [x! ** 2 - 1],
      { maxStepByVariable: [100], residualTolerance: 1e-10 },
    );
    expect(result.kind).toBe('converged');
    if (result.kind !== 'converged') return;
    expect(result.variables[0]).toBeCloseTo(1, 8);
    expect(result.diagnostics.rejectedSteps).toBeGreaterThan(0);
    for (let index = 1; index < result.diagnostics.acceptedResidualNorms.length; index += 1) {
      expect(result.diagnostics.acceptedResidualNorms[index]!)
        .toBeLessThan(result.diagnostics.acceptedResidualNorms[index - 1]!);
    }
  });

  it('honors translation and angular step bounds at the finite iteration cap', () => {
    const result = solveBoundedDampedLeastSquares(
      [0, 0],
      ([translation, angle]) => [translation! - 100, angle! - 100],
      {
        variableKinds: ['translation', 'angle'],
        maxIterations: 1,
      },
    );
    expect(result.kind).toBe('failed');
    if (result.kind !== 'failed') return;
    expect(result.reason).toBe('iteration-limit');
    expect(result.variables?.[0]).toBeCloseTo(10, 10);
    expect(result.variables?.[1]).toBeCloseTo(Math.PI / 12, 10);
    expect(result.diagnostics.acceptedSteps).toBe(1);
  });

  it('keeps the best finite iterate when bounds make a target infeasible', () => {
    const result = solveBoundedDampedLeastSquares(
      [0],
      ([x]) => [x! - 5],
      {
        project: ([x]) => [Math.max(-2, Math.min(2, x!))],
        maxStepByVariable: [10],
        maxIterations: 4,
      },
    );
    expect(result.kind).toBe('failed');
    if (result.kind !== 'failed') return;
    expect(result.variables).toEqual([2]);
    expect(result.residual).toEqual([-3]);
    expect(result.diagnostics.finalResidualNorm).toBeLessThan(result.diagnostics.initialResidualNorm!);
    expect(result.variables?.every(Number.isFinite)).toBe(true);
  });

  it('returns explicit failures for non-finite residuals and invalid projections', () => {
    const nonFinite = solveBoundedDampedLeastSquares([0], () => [Number.NaN]);
    expect(nonFinite.kind).toBe('failed');
    if (nonFinite.kind === 'failed') {
      expect(nonFinite.reason).toBe('non-finite-residual');
      expect(nonFinite.variables).toBeNull();
    }

    const invalidProjection = solveBoundedDampedLeastSquares(
      [0],
      ([x]) => [x!],
      { project: () => [Number.POSITIVE_INFINITY] },
    );
    expect(invalidProjection.kind).toBe('failed');
    if (invalidProjection.kind === 'failed') {
      expect(invalidProjection.reason).toBe('projection-failed');
      expect(invalidProjection.variables).toBeNull();
    }
  });

  it('reports stalling instead of accepting a non-decreasing zero step', () => {
    const result = solveBoundedDampedLeastSquares(
      [],
      () => [1],
      { maxIterations: 2 },
    );
    expect(result.kind).toBe('failed');
    if (result.kind !== 'failed') return;
    expect(result.reason).toBe('stalled');
    expect(result.variables).toEqual([]);
    expect(result.residual).toEqual([1]);
  });
});
