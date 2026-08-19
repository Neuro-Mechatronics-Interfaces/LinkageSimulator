import { describe, expect, it } from 'vitest';
import { localToWorld } from '../geometry';
import { poseFromPointAndAngle, reconstructRigidPose } from './rigidTransform';

describe('general planar rigid reconstruction', () => {
  it('reconstructs a pose from arbitrary local attachment coordinates', () => {
    const expected = { position: { x: 12.5, y: -8.25 }, angle: 1.137 };
    const localA = { x: -3.2, y: 4.7 };
    const localB = { x: 8.9, y: -1.3 };
    const result = reconstructRigidPose(
      localA,
      localB,
      localToWorld(localA, expected),
      localToWorld(localB, expected),
    );
    expect(result.kind).toBe('resolved');
    if (result.kind !== 'resolved') return;
    expect(result.pose.position.x).toBeCloseTo(expected.position.x, 9);
    expect(result.pose.position.y).toBeCloseTo(expected.position.y, 9);
    expect(result.pose.angle).toBeCloseTo(expected.angle, 9);
  });

  it('distinguishes degenerate local geometry from inconsistent distances', () => {
    expect(reconstructRigidPose(
      { x: 1, y: 2 },
      { x: 1, y: 2 },
      { x: 0, y: 0 },
      { x: 1, y: 0 },
    ).kind).toBe('degenerate');
    expect(reconstructRigidPose(
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      { x: 0, y: 0 },
      { x: 3, y: 0 },
    ).kind).toBe('inconsistent');
  });

  it('places any local point at a prescribed world point and angle', () => {
    const local = { x: 3, y: -4 };
    const world = { x: -7, y: 11 };
    const pose = poseFromPointAndAngle(local, world, -0.82);
    const reconstructed = localToWorld(local, pose);
    expect(reconstructed.x).toBeCloseTo(world.x, 10);
    expect(reconstructed.y).toBeCloseTo(world.y, 10);
  });
});
