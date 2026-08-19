import { describe, expect, it } from 'vitest';
import { distance, localToWorld } from '../geometry';
import { createDefaultState } from '../model';
import { solveGeneralMechanism } from './GeneralMechanismSolver';

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
