import { describe, expect, it } from 'vitest';
import { HAND_DEFAULTS, createDefaultState } from '../model';
import { calculateFingerStatics } from './fingerStatics';

describe('calculateFingerStatics', () => {
  it('uses cylindrical mass scaling and balances gravity with holding moment', () => {
    const state = createDefaultState();
    state.hand.mcpAngle = 0;
    state.hand.pipAngle = 0;
    state.hand.dipAngle = 0;
    const result = calculateFingerStatics(state.hand);
    const proximal = result.segmentMasses.find((segment) => segment.segmentId === 'proximal')!;
    const expectedMass = Math.PI * (HAND_DEFAULTS.proximalWidth / 2000) ** 2 *
      (HAND_DEFAULTS.proximalLength / 1000) * HAND_DEFAULTS.tissueDensityKgPerM3;
    expect(proximal.massKg).toBeCloseTo(expectedMass, 12);
    for (const moment of result.jointMoments) {
      expect(moment.requiredHoldingMomentNm).toBeCloseTo(-moment.gravityMomentNm, 12);
      expect(moment.passiveMomentNm).toBe(0);
    }
  });

  it('scales segment mass cubically with hand size and supports future stiffness', () => {
    const state = createDefaultState();
    const baseline = calculateFingerStatics(state.hand);
    state.hand.sizeScale = 1.2;
    state.hand.jointMechanics.mcp.stiffnessNmPerRad = 0.1;
    state.hand.mcpAngle = 0.2;
    const scaled = calculateFingerStatics(state.hand);
    expect(scaled.segmentMasses[0]!.massKg / baseline.segmentMasses[0]!.massKg).toBeCloseTo(1.2 ** 3, 12);
    expect(scaled.jointMoments.find((moment) => moment.jointId === 'mcp')!.passiveMomentNm).toBeCloseTo(-0.02, 12);
  });
});
