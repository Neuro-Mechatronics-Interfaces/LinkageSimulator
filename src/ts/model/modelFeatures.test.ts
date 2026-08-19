import { describe, expect, it } from 'vitest';
import { MechanismSimulation } from '../simulation';
import {
  createDefaultState,
  createDefaultDigitStates,
  createMultiDigitSimulationExport,
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

  it('exports all four independently posed digit workspaces', () => {
    const digitStates = createDefaultDigitStates(1.08);
    digitStates.d2.servo.angle = digitStates.d2.servo.maxAngle;
    const exported = createMultiDigitSimulationExport(
      digitStates,
      1.08,
      'd4',
      new Date('2026-08-19T12:00:00.000Z'),
    );
    expect(exported.schema).toBe('linkage-simulator-multidigit-state');
    expect(exported.schemaVersion).toBe(2);
    expect(Object.keys(exported.digits)).toEqual(['d2', 'd3', 'd4', 'd5']);
    expect(exported.digits.d2.servo.angle).not.toBe(exported.digits.d3.servo.angle);
    expect(exported.digits.d5.hand.sizeScale).toBe(1.08);
    digitStates.d2.servo.angle = digitStates.d2.servo.minAngle;
    expect(exported.digits.d2.servo.angle).toBe(exported.digits.d2.servo.maxAngle);
  });
});

describe('digit profiles', () => {
  it('creates D2-D5 at expected relative lengths without sharing servo state', () => {
    const digits = createDefaultDigitStates();
    const totalLength = (digitId: keyof typeof digits): number =>
      digits[digitId].hand.segments.reduce((sum, segment) => sum + segment.baseLength, 0);
    expect(totalLength('d3')).toBeGreaterThan(totalLength('d4'));
    expect(totalLength('d4')).toBeGreaterThan(totalLength('d2'));
    expect(totalLength('d2')).toBeGreaterThan(totalLength('d5'));
    const d3Angle = digits.d3.servo.angle;
    digits.d2.servo.angle = digits.d2.servo.maxAngle;
    expect(digits.d3.servo.angle).toBe(d3Angle);
  });
});
