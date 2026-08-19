import { describe, expect, it } from 'vitest';
import {
  composeObjectiveDesignGradient,
  computeConstraintSensitivity,
} from './constraintSensitivity';

describe('implicit constraint sensitivity', () => {
  it('matches the closed-form derivative of q^2 - d = 0', () => {
    const result = computeConstraintSensitivity({
      configuration: [3],
      design: [9],
      evaluateResidual: ([q], [d]) => [q! * q! - d!],
    });

    expect(result.kind).toBe('solved');
    if (result.kind !== 'solved') return;
    expect(result.sensitivity[0]![0]).toBeCloseTo(1 / 6, 7);
    expect(result.configurationJacobian[0]![0]).toBeCloseTo(6, 7);
    expect(result.designJacobian[0]![0]).toBeCloseTo(-1, 7);
    expect(result.diagnostics.maximumLinearizedResidualNorm).toBeLessThan(1e-8);
  });

  it('matches a coupled two-configuration, two-design system', () => {
    const result = computeConstraintSensitivity({
      configuration: [12 / 7, 8 / 7],
      design: [4, 2],
      evaluateResidual: ([q0, q1], [d0, d1]) => [
        q0! + 2 * q1! - d0!,
        3 * q0! - q1! - d1! ** 2,
      ],
    });

    expect(result.kind).toBe('solved');
    if (result.kind !== 'solved') return;
    expect(result.sensitivity[0]![0]).toBeCloseTo(1 / 7, 7);
    expect(result.sensitivity[0]![1]).toBeCloseTo(8 / 7, 7);
    expect(result.sensitivity[1]![0]).toBeCloseTo(3 / 7, 7);
    expect(result.sensitivity[1]![1]).toBeCloseTo(-4 / 7, 7);
    expect(result.diagnostics.configurationRank).toBe(2);
  });

  it('solves a compatible tall Jacobian through least squares', () => {
    const result = computeConstraintSensitivity({
      configuration: [2],
      design: [2],
      evaluateResidual: ([q], [d]) => [q! - d!, 2 * (q! - d!)],
    });
    expect(result.kind).toBe('solved');
    if (result.kind !== 'solved') return;
    expect(result.sensitivity[0]![0]).toBeCloseTo(1, 8);
    expect(result.diagnostics.redundantResidualCount).toBe(1);
  });

  it('applies the objective chain rule to an implicit configuration', () => {
    const sensitivity = computeConstraintSensitivity({
      configuration: [3],
      design: [9],
      evaluateResidual: ([q], [d]) => [q! * q! - d!],
    });
    expect(sensitivity.kind).toBe('solved');
    if (sensitivity.kind !== 'solved') return;

    // Phi(q, d) = q^3 + 2d, so dPhi/dd = 3q^2/(2q) + 2 = 6.5 at q = 3.
    const gradient = composeObjectiveDesignGradient(sensitivity.sensitivity, [27], [2]);
    expect(gradient.kind).toBe('solved');
    if (gradient.kind !== 'solved') return;
    expect(gradient.gradient[0]).toBeCloseTo(6.5, 7);
  });

  it('rejects an underdetermined configuration Jacobian explicitly', () => {
    const result = computeConstraintSensitivity({
      configuration: [1, 2],
      design: [3],
      evaluateResidual: ([q0, q1], [d]) => [q0! + q1! - d!],
    });
    expect(result.kind).toBe('underdetermined');
    expect(result.diagnostics.configurationRank).toBe(1);
  });

  it('rejects rank deficiency even when the residual count is sufficient', () => {
    const result = computeConstraintSensitivity({
      configuration: [1, 2],
      design: [3],
      evaluateResidual: ([q0, q1], [d]) => {
        const closure = q0! + q1! - d!;
        return [closure, 2 * closure];
      },
    });
    expect(result.kind).toBe('rank-deficient');
    expect(result.diagnostics.configurationRank).toBe(1);
    expect(result.diagnostics.configurationNullity).toBe(1);
  });

  it('returns a typed invalid result for non-finite evaluations', () => {
    const result = computeConstraintSensitivity({
      configuration: [1],
      design: [1],
      evaluateResidual: () => [Number.NaN],
    });
    expect(result.kind).toBe('invalid');
    if (result.kind !== 'invalid') return;
    expect(result.message).toMatch(/finite/);
  });

  it('requires a converged nominal configuration', () => {
    const result = computeConstraintSensitivity({
      configuration: [2],
      design: [3],
      evaluateResidual: ([q], [d]) => [q! - d!],
    });
    expect(result.kind).toBe('not-converged');
    expect(result.diagnostics.baselineResidualNorm).toBe(1);
  });
});
