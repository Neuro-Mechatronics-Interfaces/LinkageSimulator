import { describe, expect, it } from 'vitest';
import { distance, localToWorld } from '../geometry';
import { createDefaultState } from '../model';
import { retargetJointEndpoint } from './Inspector';

describe('retargetJointEndpoint', () => {
  it('moves the target attachment into the new link frame without moving the hinge', () => {
    const state = createDefaultState();
    const joint = state.joints.find((candidate) => candidate.id === 'crank-coupler-joint');
    const previousTarget = state.links.find((link) => link.id === joint?.linkBId);
    const nextTarget = state.links.find((link) => link.id === 'tip-driver');
    expect(joint).toBeDefined();
    expect(previousTarget).toBeDefined();
    expect(nextTarget).toBeDefined();
    if (!joint || !previousTarget || !nextTarget) return;

    const previousHinge = localToWorld(joint.localPointB, previousTarget.pose);
    expect(retargetJointEndpoint(state, joint, 'target', nextTarget.id)).toBe(true);

    expect(joint.linkAId).toBe('crank');
    expect(joint.linkBId).toBe(nextTarget.id);
    expect(distance(localToWorld(joint.localPointB, nextTarget.pose), previousHinge)).toBeLessThan(1e-10);
  });

  it('keeps an actuator incident by following an actuated joint target', () => {
    const state = createDefaultState();
    const joint = state.joints.find((candidate) => candidate.id === state.servo.revoluteJointId);
    expect(joint).toBeDefined();
    if (!joint) return;

    expect(retargetJointEndpoint(state, joint, 'target', 'coupler')).toBe(true);
    expect(joint.linkBId).toBe('coupler');
    expect(state.servo.drivenLinkId).toBe('coupler');
  });

  it('does not change an actuator driven by the reference when its target changes', () => {
    const state = createDefaultState();
    const joint = state.joints.find((candidate) => candidate.id === 'crank-coupler-joint');
    expect(joint).toBeDefined();
    if (!joint || joint.linkAId === null) return;
    state.servo.revoluteJointId = joint.id;
    state.servo.drivenLinkId = joint.linkAId;

    expect(retargetJointEndpoint(state, joint, 'target', 'tip-driver')).toBe(true);
    expect(state.servo.drivenLinkId).toBe('crank');
  });

  it('can replace the reference with ground while preserving and normalizing the hinge', () => {
    const state = createDefaultState();
    const joint = state.joints.find((candidate) => candidate.id === 'crank-coupler-joint');
    const target = state.links.find((link) => link.id === joint?.linkBId);
    expect(joint).toBeDefined();
    expect(target).toBeDefined();
    if (!joint || !target) return;
    const previousHinge = localToWorld(joint.localPointB, target.pose);

    expect(retargetJointEndpoint(state, joint, 'reference', null)).toBe(true);
    expect(joint.linkAId).toBeNull();
    expect(joint.localPointA).toBeUndefined();
    expect(joint.groundPoint).toEqual(previousHinge);
    expect(distance(localToWorld(joint.localPointB, target.pose), previousHinge)).toBeLessThan(1e-10);
  });

  it('can replace ground with a reference segment and clears the ground attachment', () => {
    const state = createDefaultState();
    const joint = state.joints.find((candidate) => candidate.id === 'crank-coupler-joint');
    const target = state.links.find((link) => link.id === joint?.linkBId);
    const reference = state.links.find((link) => link.id === 'rocker');
    expect(joint).toBeDefined();
    expect(target).toBeDefined();
    expect(reference).toBeDefined();
    if (!joint || !target || !reference) return;
    const previousHinge = localToWorld(joint.localPointB, target.pose);
    joint.linkAId = null;
    joint.groundPoint = { ...previousHinge };
    delete joint.localPointA;

    expect(retargetJointEndpoint(state, joint, 'reference', reference.id)).toBe(true);
    expect(joint.linkAId).toBe(reference.id);
    expect(joint.groundPoint).toBeUndefined();
    expect(joint.localPointA).toBeDefined();
    expect(distance(localToWorld(joint.localPointA!, reference.pose), previousHinge)).toBeLessThan(1e-10);
  });

  it('rejects a missing target and a target equal to the source side', () => {
    const state = createDefaultState();
    const joint = state.joints.find((candidate) => candidate.id === 'crank-coupler-joint');
    expect(joint).toBeDefined();
    if (!joint) return;
    const originalTarget = joint.linkBId;
    const originalPoint = { ...joint.localPointB };

    expect(retargetJointEndpoint(state, joint, 'target', 'crank')).toBe(false);
    expect(retargetJointEndpoint(state, joint, 'target', 'missing-link')).toBe(false);
    expect(joint.linkBId).toBe(originalTarget);
    expect(joint.localPointB).toEqual(originalPoint);
  });
});
