import { describe, expect, it } from 'vitest';
import { MechanismSimulation } from '../simulation';
import {
  createDefaultState,
  createSimulationExport,
  digitWidthAtContactor,
  enforceRingWidth,
  ringAffordance,
} from './index';

describe('contactor ring geometry', () => {
  it('never permits a ring narrower than its digit segment', () => {
    const state = createDefaultState();
    const contactor = state.contactors[0]!;
    contactor.ringWidth = 0;
    enforceRingWidth(state.hand, contactor);
    expect(contactor.ringWidth).toBe(digitWidthAtContactor(state.hand, contactor));
    expect(ringAffordance(state.hand, contactor)).toBe(0);

    state.hand.sizeScale = 1.2;
    enforceRingWidth(state.hand, contactor);
    expect(contactor.ringWidth).toBe(digitWidthAtContactor(state.hand, contactor));
    expect(ringAffordance(state.hand, contactor)).toBe(0);
  });
});

describe('simulation JSON export', () => {
  it('exports a versioned, unit-labelled snapshot including statics', () => {
    const state = createDefaultState();
    new MechanismSimulation().solve(state);
    const exported = createSimulationExport(state, new Date('2026-08-19T12:00:00.000Z'));
    expect(exported.schemaVersion).toBe(1);
    expect(exported.exportedAtUtc).toBe('2026-08-19T12:00:00.000Z');
    expect(exported.units.moment).toBe('N·m');
    expect(exported.state.links).toHaveLength(state.links.length);
    expect(exported.state.contactors[0]!.ringWidth).toBeGreaterThan(0);
    expect(exported.state.statics.jointMoments).toHaveLength(3);
    expect(JSON.stringify(exported)).not.toContain('bandClearance');
  });
});
