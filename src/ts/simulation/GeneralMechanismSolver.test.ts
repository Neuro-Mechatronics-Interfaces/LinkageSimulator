import { describe, expect, it } from 'vitest';
import { distance, localToWorld, normalizeAngle } from '../geometry';
import { createDefaultState } from '../model';
import { solveAnalyticConstraints } from './analyticConstraintSolver';
import { buildConstraintGraph } from './ConstraintGraph';
import { analyzeConstraintGraph } from './constraintAnalysis';
import { solveGeneralMechanism } from './GeneralMechanismSolver';

function createNumericalFallbackState(tangent = false): ReturnType<typeof createDefaultState> {
  const state = createDefaultState();
  const rootThree = Math.sqrt(3);
  const compoundLength = Math.sqrt(12);
  state.links = [
    {
      id: 'drive-body', name: 'Drive', length: 4, width: 2,
      pose: { position: { x: 1, y: 0 }, angle: 0 },
    },
    {
      id: 'triad-a', name: 'A', length: 2, width: 2,
      pose: tangent
        ? { position: { x: 3, y: 0 }, angle: -Math.PI / 6 }
        : { position: { x: 3.4, y: -0.3 }, angle: 0.15 },
    },
    {
      id: 'triad-b', name: 'B', length: 2, width: 2,
      pose: tangent
        ? { position: { x: 3 + rootThree, y: -1 }, angle: Math.PI / 6 }
        : { position: { x: 4.7, y: 0.4 }, angle: 0.9 },
    },
    {
      id: 'triad-c', name: 'C', length: 2, width: 2,
      pose: tangent
        ? { position: { x: 3 + 2 * rootThree, y: 0 }, angle: 0 }
        : { position: { x: 6.3, y: 1.4 }, angle: 2.8 },
    },
  ];
  state.joints = [
    {
      id: 'servo-hinge', name: 'Servo hinge', linkAId: null, linkBId: 'drive-body',
      groundPoint: { x: 0, y: 0 }, localPointB: { x: -1, y: 0 },
    },
    {
      id: 'drive-a', name: 'Drive-A', linkAId: 'drive-body', linkBId: 'triad-a',
      localPointA: { x: 2, y: 0 }, localPointB: { x: 0, y: 0 },
    },
    {
      id: 'locked-a-b', name: 'Locked A-B', linkAId: 'triad-a', linkBId: 'triad-b',
      localPointA: { x: 2, y: 0 }, localPointB: { x: 0, y: 0 },
      minAngle: Math.PI / 3, maxAngle: Math.PI / 3,
    },
    {
      id: 'b-c', name: 'B-C', linkAId: 'triad-b', linkBId: 'triad-c',
      localPointA: { x: 2, y: 0 }, localPointB: { x: 0, y: 0 },
      minAngle: tangent ? -0.6 : 2,
      maxAngle: tangent ? -0.4 : 2.2,
    },
    {
      id: 'c-drive', name: 'C-Drive', linkAId: 'triad-c', linkBId: 'drive-body',
      localPointA: { x: 2, y: 0 },
      localPointB: tangent ? { x: 4 + compoundLength, y: 0 } : { x: 3, y: rootThree },
    },
  ];
  state.servo = {
    id: 'command',
    name: 'Command',
    groundPoint: { x: 0, y: 0 },
    drivenLinkId: 'drive-body',
    revoluteJointId: 'servo-hinge',
    angle: 0,
    minAngle: -Math.PI,
    maxAngle: Math.PI,
    speed: 1,
    direction: 1,
  };
  state.contactors = [];
  return state;
}

function renameAndShuffleMechanism(state: ReturnType<typeof createDefaultState>): Map<string, string> {
  const linkIds = new Map(state.links.map((link, index) => [link.id, `renamed-body-${index * 31 + 7}`]));
  const jointIds = new Map(state.joints.map((joint, index) => [joint.id, `renamed-joint-${index * 37 + 5}`]));
  for (const link of state.links) link.id = linkIds.get(link.id)!;
  for (const joint of state.joints) {
    const oldId = joint.id;
    joint.id = jointIds.get(oldId)!;
    joint.linkAId = joint.linkAId === null ? null : linkIds.get(joint.linkAId)!;
    joint.linkBId = linkIds.get(joint.linkBId)!;
  }
  state.servo.id = 'renamed-actuator';
  state.servo.drivenLinkId = linkIds.get(state.servo.drivenLinkId)!;
  state.servo.revoluteJointId = jointIds.get(state.servo.revoluteJointId)!;
  state.links.reverse();
  state.joints.reverse();
  return linkIds;
}

describe('general mechanism solve orchestration', () => {
  it('reports the default graph mobility and solves every mechanical revolute', () => {
    const state = createDefaultState();
    const result = solveGeneralMechanism(state);
    expect(result.valid, result.message).toBe(true);
    expect(result.diagnostics.components).toHaveLength(1);
    const component = result.diagnostics.components[0]!;
    expect(component.variableCount).toBe(18);
    expect(component.passiveJacobianRank).toBe(15);
    expect(component.jacobianRank).toBe(16);
    expect(component.passiveDof).toBe(3);
    expect(component.drivenDof).toBe(2);
    expect(component.unresolvedLinkIds).toEqual(['middle-driver', 'tip-driver']);
    expect(result.analyticSteps).toHaveLength(1);
    expect(result.message).toContain('DOF 3 / driven 2');
    expectMaximumJointError(state, 1e-7);
  });

  it('is invariant to randomized component and joint IDs', () => {
    const reference = createDefaultState();
    const renamed = createDefaultState();
    const linkIds = new Map(renamed.links.map((link, index) => [link.id, `body-z${index * 17 + 3}`]));
    const jointIds = new Map(renamed.joints.map((joint, index) => [joint.id, `hinge-q${index * 19 + 5}`]));
    for (const link of renamed.links) link.id = linkIds.get(link.id)!;
    for (const joint of renamed.joints) {
      const oldId = joint.id;
      joint.id = jointIds.get(oldId)!;
      joint.linkAId = joint.linkAId === null ? null : linkIds.get(joint.linkAId)!;
      joint.linkBId = linkIds.get(joint.linkBId)!;
    }
    renamed.servo.drivenLinkId = linkIds.get(renamed.servo.drivenLinkId)!;
    renamed.servo.revoluteJointId = jointIds.get(renamed.servo.revoluteJointId)!;
    for (const contactor of renamed.contactors) contactor.linkId = linkIds.get(contactor.linkId)!;

    const referenceResult = solveGeneralMechanism(reference);
    const renamedResult = solveGeneralMechanism(renamed);
    expect(referenceResult.valid, referenceResult.message).toBe(true);
    expect(renamedResult.valid, renamedResult.message).toBe(true);
    for (let index = 0; index < reference.links.length; index += 1) {
      expect(renamed.links[index]!.pose.position.x).toBeCloseTo(reference.links[index]!.pose.position.x, 8);
      expect(renamed.links[index]!.pose.position.y).toBeCloseTo(reference.links[index]!.pose.position.y, 8);
      expect(renamed.links[index]!.pose.angle).toBeCloseTo(reference.links[index]!.pose.angle, 8);
    }
    expect(renamedResult.diagnostics.components[0]?.passiveDof).toBe(3);
    expect(renamedResult.diagnostics.components[0]?.drivenDof).toBe(2);
    expectMaximumJointError(renamed, 1e-7);
  });

  it('reports a free-floating component without treating its finite pose as failure', () => {
    const state = createDefaultState();
    state.links.push({
      id: 'floating-body',
      name: 'Floating',
      length: 12,
      width: 3,
      pose: { position: { x: 250, y: 250 }, angle: 0.42 },
    });
    const result = solveGeneralMechanism(state);
    expect(result.valid, result.message).toBe(true);
    expect(result.diagnostics.disconnectedComponentIds).toHaveLength(1);
    const floating = result.diagnostics.components.find((component) =>
      component.linkIds.includes('floating-body'))!;
    expect(floating.anchored).toBe(false);
    expect(floating.passiveDof).toBe(3);
    expect(floating.drivenDof).toBe(3);
    expect(floating.unresolvedLinkIds).toEqual(['floating-body']);
  });

  it('preserves dyad closure throughout the commanded servo interval', () => {
    const state = createDefaultState();
    let history = new Map<string, { x: number; y: number }>();
    for (let index = 0; index <= 100; index += 1) {
      state.servo.angle = state.servo.minAngle +
        ((state.servo.maxAngle - state.servo.minAngle) * index) / 100;
      const result = solveGeneralMechanism(state, history);
      expect(result.valid, `${index}: ${result.message}; ${JSON.stringify(result.analytic?.resolutions)}`).toBe(true);
      expectMaximumJointError(state, 1e-7);
      history = result.jointPositions;
    }
  });

  it('uses bounded numerical closure for a zero-DOF triad not handled by dyad propagation', () => {
    const state = createNumericalFallbackState();
    const graph = buildConstraintGraph(state);
    const analyticOnly = solveAnalyticConstraints(graph);
    expect(analyticOnly.unresolvedLinkIds).toEqual(['triad-a', 'triad-b', 'triad-c']);
    const beforeFallback = analyzeConstraintGraph(graph).components[0]!;
    expect(beforeFallback.passiveJacobianRank).toBe(11);
    expect(beforeFallback.passiveDof).toBe(1);
    expect(beforeFallback.jacobianRank).toBe(12);
    expect(beforeFallback.drivenDof).toBe(0);

    const result = solveGeneralMechanism(state);
    expect(result.valid, result.message).toBe(true);
    expect(result.analytic?.unresolvedLinkIds).toEqual([]);
    const component = result.diagnostics.components[0]!;
    expect(component.passiveJacobianRank).toBe(11);
    expect(component.passiveDof).toBe(1);
    expect(component.jacobianRank).toBe(12);
    expect(component.drivenDof).toBe(0);
    expect(component.residualNorm).toBeLessThan(1e-5);
    expect(component.numericalFallbackUsed).toBe(true);
    expect(result.message).toContain('numerical fallback');
    const triadB = state.links.find((link) => link.id === 'triad-b')!;
    const triadC = state.links.find((link) => link.id === 'triad-c')!;
    expect(normalizeAngle(triadC.pose.angle - triadB.pose.angle)).toBeGreaterThanOrEqual(2);
    expect(normalizeAngle(triadC.pose.angle - triadB.pose.angle)).toBeLessThanOrEqual(2.2);
    expectMaximumJointError(state, 1e-5);
  });

  it('keeps numerical fallback independent of component IDs and joint insertion order', () => {
    const reference = createNumericalFallbackState();
    const renamed = createNumericalFallbackState();
    const renamedLinkIds = renameAndShuffleMechanism(renamed);

    const referenceResult = solveGeneralMechanism(reference);
    const renamedResult = solveGeneralMechanism(renamed);
    expect(referenceResult.valid, referenceResult.message).toBe(true);
    expect(renamedResult.valid, renamedResult.message).toBe(true);
    expect(referenceResult.diagnostics.components[0]?.numericalFallbackUsed).toBe(true);
    expect(renamedResult.diagnostics.components[0]?.numericalFallbackUsed).toBe(true);
    for (const referenceLink of reference.links) {
      const renamedLink = renamed.links.find((link) => link.id === renamedLinkIds.get(referenceLink.id))!;
      expect(renamedLink.pose.position.x).toBeCloseTo(referenceLink.pose.position.x, 6);
      expect(renamedLink.pose.position.y).toBeCloseTo(referenceLink.pose.position.y, 6);
      expect(renamedLink.pose.angle).toBeCloseTo(referenceLink.pose.angle, 6);
    }
    expectMaximumJointError(renamed, 1e-5);
  });

  it('uses generic rank to close a singular zero-mobility remainder numerically', () => {
    const state = createNumericalFallbackState(true);
    const initial = analyzeConstraintGraph(buildConstraintGraph(state)).components[0]!;
    expect(initial.jacobianRank).toBe(11);
    expect(initial.drivenDof).toBe(1);
    expect(initial.expectedDrivenJacobianRank).toBe(12);
    expect(initial.singular).toBe(true);

    const result = solveGeneralMechanism(state);
    expect(result.valid, result.message).toBe(true);
    const component = result.diagnostics.components[0]!;
    expect(component.numericalFallbackUsed).toBe(true);
    expect(component.jacobianRank).toBe(11);
    expect(component.drivenDof).toBe(1);
    expect(component.singular).toBe(true);
    expect(result.analytic?.intentionallyFreeLinkIds.size).toBe(0);
    expectMaximumJointError(state, 1e-7);
  });

  it('reports mobility from the solved local configuration rather than the initial pose', () => {
    const state = createDefaultState();
    state.contactors = [];
    state.links = [
      { id: 'input', name: 'Input', length: 1, width: 1, pose: { position: { x: 0, y: 0 }, angle: 0.4 } },
      { id: 'coupler-body', name: 'Coupler', length: 2, width: 1, pose: { position: { x: 0, y: 0 }, angle: 0.2 } },
      { id: 'output', name: 'Output', length: 1, width: 1, pose: { position: { x: 0, y: 0 }, angle: -0.3 } },
    ];
    state.joints = [
      { id: 'input-ground', name: '', linkAId: null, linkBId: 'input', groundPoint: { x: 0, y: 0 }, localPointB: { x: 0, y: 0 } },
      { id: 'input-coupler', name: '', linkAId: 'input', linkBId: 'coupler-body', localPointA: { x: 1, y: 0 }, localPointB: { x: 1, y: 0 } },
      { id: 'coupler-output', name: '', linkAId: 'coupler-body', linkBId: 'output', localPointA: { x: 3, y: 0 }, localPointB: { x: 3, y: 0 } },
      { id: 'output-ground', name: '', linkAId: null, linkBId: 'output', groundPoint: { x: 4, y: 0 }, localPointB: { x: 4, y: 0 } },
    ];
    state.servo = {
      id: 'toggle-command', name: '', groundPoint: { x: 0, y: 0 },
      drivenLinkId: 'input', revoluteJointId: 'input-ground', angle: 0,
      minAngle: -1, maxAngle: 1, speed: 1, direction: 1,
    };

    const result = solveGeneralMechanism(state);
    expect(result.valid, result.message).toBe(true);
    const component = result.diagnostics.components[0]!;
    expect(component.singular).toBe(true);
    expect(component.passiveJacobianRank).toBe(7);
    expect(component.passiveDof).toBe(2);
    expect(component.jacobianRank).toBe(8);
    expect(component.drivenDof).toBe(1);
    expect(component.passiveDof).toBe(component.variableCount - component.passiveJacobianRank);
    expect(component.drivenDof).toBe(component.variableCount - component.jacobianRank);
  });

  it('restores every link pose when a direct general solve fails', () => {
    const state = createDefaultState();
    const initial = solveGeneralMechanism(state);
    expect(initial.valid, initial.message).toBe(true);
    const previousPoses = new Map(state.links.map((link) => [link.id, structuredClone(link.pose)]));
    const crankCoupler = state.joints.find((joint) => joint.id === 'crank-coupler-joint')!;
    const couplerRocker = state.joints.find((joint) => joint.id === 'coupler-rocker-joint')!;
    const rockerGround = state.joints.find((joint) => joint.id === 'rocker-ground-joint')!;
    couplerRocker.localPointA = { x: crankCoupler.localPointB.x + 1, y: crankCoupler.localPointB.y };
    couplerRocker.localPointB = { x: rockerGround.localPointB.x + 1, y: rockerGround.localPointB.y };
    state.servo.angle += 0.2;

    const failed = solveGeneralMechanism(state);
    expect(failed.valid).toBe(false);
    for (const link of state.links) expect(link.pose).toEqual(previousPoses.get(link.id));
  });
});

function expectMaximumJointError(state: ReturnType<typeof createDefaultState>, tolerance: number): void {
  for (const joint of state.joints) {
    const linkB = state.links.find((link) => link.id === joint.linkBId)!;
    const pointB = localToWorld(joint.localPointB, linkB.pose);
    const pointA = joint.linkAId === null
      ? joint.groundPoint!
      : localToWorld(
        joint.localPointA!,
        state.links.find((link) => link.id === joint.linkAId)!.pose,
      );
    expect(distance(pointA, pointB), joint.id).toBeLessThan(tolerance);
  }
}
