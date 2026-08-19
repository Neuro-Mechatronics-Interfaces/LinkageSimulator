import { describe, expect, it } from 'vitest';
import type { Link, LinearSlotJoint, SimulationState } from '../model';
import { createDefaultState, createSimulationExport } from '../model';
import { buildConstraintGraph, connectedComponents } from './ConstraintGraph';
import { analyzeConstraintGraph, createComponentResidualSystem } from './constraintAnalysis';
import { solveGeneralMechanism } from './GeneralMechanismSolver';
import { evaluateLinearSlotGeometry } from './linearSlotGeometry';
import { validateMechanismInvariants } from './mechanismInvariants';

function link(id: string, x: number, y: number, angle = 0, fixed = false): Link {
  return {
    id,
    name: id,
    length: 4,
    width: 1,
    pose: { position: { x, y }, angle },
    ...(fixed ? { fixed: true } : {}),
  };
}

function groundSlot(
  id: string,
  pinLinkId: string,
  slotOrigin: { x: number; y: number },
  slotDirection: { x: number; y: number },
  pinLocalPoint: { x: number; y: number },
  minTravel = -10,
  maxTravel = 10,
): LinearSlotJoint {
  return {
    id,
    name: id,
    slotLinkId: null,
    pinLinkId,
    slotOrigin,
    slotDirection,
    pinLocalPoint,
    minTravel,
    maxTravel,
  };
}

function slotFallbackState(prefix: string, reverseSlots = false): SimulationState {
  const state = createDefaultState();
  const actuatorId = `${prefix}-actuator-body`;
  const sliderId = `${prefix}-slider-body`;
  state.links = [
    link(actuatorId, 0, 0),
    link(sliderId, 1.6, 0.4, 0.2),
  ];
  state.joints = [{
    id: `${prefix}-servo-pivot`,
    name: 'Servo pivot',
    linkAId: null,
    linkBId: actuatorId,
    groundPoint: { x: 0, y: 0 },
    localPointB: { x: 0, y: 0 },
  }];
  const slots = [
    groundSlot(`${prefix}-slot-a`, sliderId, { x: 0, y: 0 }, { x: 3, y: 0 }, { x: -1, y: 0 }, -0.25, 0.25),
    groundSlot(`${prefix}-slot-b`, sliderId, { x: 2, y: 0 }, { x: 0, y: 7 }, { x: 1, y: 0 }, -0.25, 0.25),
    groundSlot(`${prefix}-slot-c`, sliderId, { x: 0, y: 1 }, { x: 2, y: 0 }, { x: 0, y: 1 }, 0.75, 1.25),
  ];
  slots[0]!.friction = {
    model: 'coulomb',
    coefficient: 0.2,
    materialPair: 'test fixture pair',
    source: 'user-supplied test value',
  };
  state.linearSlotJoints = reverseSlots ? slots.reverse() : slots;
  state.servo = {
    id: `${prefix}-servo`,
    name: 'Servo',
    groundPoint: { x: 0, y: 0 },
    drivenLinkId: actuatorId,
    revoluteJointId: `${prefix}-servo-pivot`,
    angle: 0,
    minAngle: -Math.PI,
    maxAngle: Math.PI,
    speed: 1,
    direction: 1,
  };
  state.contactors = [];
  return state;
}

describe('linear-slot constraint foundation', () => {
  it('stores an empty serializable collection in the default state', () => {
    const state = createDefaultState();
    expect(state.linearSlotJoints).toEqual([]);
    expect(createSimulationExport(state).state.linearSlotJoints).toEqual([]);
  });

  it('adds one normal equality while retaining travel and friction as metadata', () => {
    const rail = link('rail', 10, 5, Math.PI / 2, true);
    const pin = link('pin', 10, 7);
    const slot: LinearSlotJoint = {
      ...groundSlot('slot', pin.id, { x: 0, y: 0 }, { x: 1, y: 0 }, { x: 0, y: 0 }),
      slotLinkId: rail.id,
      slotOrigin: { x: 0, y: 0 },
      slotDirection: { x: 4, y: 0 },
      minTravel: 1,
      maxTravel: 3,
      friction: { model: 'coulomb', coefficient: 0.15, source: 'fixture' },
    };
    const graph = buildConstraintGraph({ links: [pin, rail], joints: [], linearSlotJoints: [slot] });
    const component = connectedComponents(graph)[0]!;
    const system = createComponentResidualSystem(graph, component, { includeActuator: false });

    expect(graph.linearSlotConstraints.get(slot.id)?.scalarEquationCount).toBe(1);
    expect(component.jointIds).toEqual(['slot']);
    expect(system.scalarResiduals).toMatchObject([
      { constraintKind: 'linear-slot', coordinate: 'normal', jointId: 'slot' },
    ]);
    expect(system.evaluate(system.initialConfiguration)[0]).toBeCloseTo(0, 12);
    const diagnostics = analyzeConstraintGraph(graph).components[0]!;
    expect(diagnostics.variableCount).toBe(3);
    expect(diagnostics.passiveJacobianRank).toBe(1);
    expect(diagnostics.passiveDof).toBe(2);
    expect(diagnostics.residualCount).toBe(1);
  });

  it('rejects degenerate slot geometry and invalid author-supplied friction', () => {
    const pin = link('pin', 0, 0);
    expect(() => buildConstraintGraph({
      links: [pin],
      joints: [],
      linearSlotJoints: [groundSlot('zero-axis', pin.id, { x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 })],
    })).toThrow(/direction must be non-zero/);
    expect(() => buildConstraintGraph({
      links: [pin],
      joints: [],
      linearSlotJoints: [{
        ...groundSlot('bad-friction', pin.id, { x: 0, y: 0 }, { x: 1, y: 0 }, { x: 0, y: 0 }),
        friction: { model: 'coulomb', coefficient: -0.1 },
      }],
    })).toThrow(/friction coefficient/);
  });

  it('checks both normal closure and finite travel as hard invariants', () => {
    const pin = link('pin', 3, 0);
    const slot = groundSlot('bounded', pin.id, { x: 0, y: 0 }, { x: 1, y: 0 }, { x: 0, y: 0 }, 0, 2);
    const graph = buildConstraintGraph({ links: [pin], joints: [], linearSlotJoints: [slot] });
    const outside = validateMechanismInvariants(graph);
    expect(outside.maximumClosureError).toBe(0);
    expect(outside.valid).toBe(false);
    expect(outside.invalidJointIds).toEqual(['bounded']);
    expect(outside.messages.join(' ')).toContain('outside bounds');

    pin.pose.position = { x: 1, y: 0.5 };
    const offCenterline = validateMechanismInvariants(graph);
    expect(offCenterline.valid).toBe(false);
    expect(offCenterline.maximumClosureError).toBeCloseTo(0.5);
    expect(offCenterline.messages.join(' ')).toContain('normal closure error');
  });

  it('closes a zero-DOF slot component numerically and enforces travel bounds', () => {
    const state = slotFallbackState('reference');
    const before = analyzeConstraintGraph(buildConstraintGraph(state));
    const slotComponent = before.components.find((component) =>
      component.linkIds.includes('reference-slider-body'))!;
    expect(slotComponent.passiveDof).toBe(0);
    expect(slotComponent.expectedPassiveJacobianRank).toBe(3);

    const result = solveGeneralMechanism(state);
    expect(result.valid, result.message).toBe(true);
    const solvedComponent = result.diagnostics.components.find((component) =>
      component.linkIds.includes('reference-slider-body'))!;
    expect(solvedComponent.numericalFallbackUsed).toBe(true);
    expect(solvedComponent.residualNorm).toBeLessThan(1e-5);
    expect(validateMechanismInvariants(result.graph!).valid).toBe(true);
    const slider = state.links.find((candidate) => candidate.id === 'reference-slider-body')!;
    for (const slot of state.linearSlotJoints) {
      const geometry = evaluateLinearSlotGeometry(slot, null, slider.pose);
      expect(Math.abs(geometry.normalOffset), slot.id).toBeLessThan(1e-5);
      expect(geometry.travel, slot.id).toBeGreaterThanOrEqual(slot.minTravel - 1e-7);
      expect(geometry.travel, slot.id).toBeLessThanOrEqual(slot.maxTravel + 1e-7);
    }
  });

  it('produces the same bounded solution after IDs and insertion order change', () => {
    const reference = slotFallbackState('a', false);
    const renamed = slotFallbackState('z-random-73', true);
    const referenceResult = solveGeneralMechanism(reference);
    const renamedResult = solveGeneralMechanism(renamed);
    expect(referenceResult.valid, referenceResult.message).toBe(true);
    expect(renamedResult.valid, renamedResult.message).toBe(true);
    const referencePose = reference.links.find((candidate) => candidate.id === 'a-slider-body')!.pose;
    const renamedPose = renamed.links.find((candidate) => candidate.id === 'z-random-73-slider-body')!.pose;
    expect(renamedPose.position.x).toBeCloseTo(referencePose.position.x, 7);
    expect(renamedPose.position.y).toBeCloseTo(referencePose.position.y, 7);
    expect(renamedPose.angle).toBeCloseTo(referencePose.angle, 7);
  });
});
