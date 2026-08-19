import { describe, expect, it } from 'vitest';
import { localToWorld } from '../geometry';
import { createDefaultState } from '../model';
import { MechanismSimulation } from './MechanismSimulation';

describe('MechanismSimulation', () => {
  it('solves the default mechanism and moves it deterministically', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    simulation.solve(state);
    const initialPoint = { ...state.contactors[0]!.linkagePoint };
    const initialAngle = state.servo.angle;
    simulation.step(state, 0.5);
    expect(state.valid, state.message).toBe(true);
    expect(state.servo.angle).toBeGreaterThan(initialAngle);
    expect(state.contactors[0]!.linkagePoint).not.toEqual(initialPoint);
    expect(state.contactors[0]!.linkagePoint.y).toBeGreaterThan(state.contactors[0]!.fingerPoint.y);
    expect(Math.max(...state.ground.pivotPoints.map((point) => point.x))).toBeLessThan(state.contactors[0]!.linkagePoint.x);
  });

  it('remains finite and dorsal throughout the configured servo sweep', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    for (let index = 0; index <= 100; index += 1) {
      state.servo.angle = state.servo.minAngle +
        ((state.servo.maxAngle - state.servo.minAngle) * index) / 100;
      simulation.solve(state);
      expect(state.valid, `${index}: ${state.message}`).toBe(true);
      expect(state.message, `${index}: contact should remain exact`).toBe('Constraints solved');
      expect(state.contactors[0]!.linkagePoint.y).toBeGreaterThan(state.contactors[0]!.fingerPoint.y);
      for (const link of state.links) {
        expect(Number.isFinite(link.pose.position.x)).toBe(true);
        expect(Number.isFinite(link.pose.position.y)).toBe(true);
        expect(Number.isFinite(link.pose.angle)).toBe(true);
      }
    }
  });

  it('reports impossible geometry without producing non-finite poses', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    state.links.find((link) => link.id === 'coupler')!.length = 2;
    simulation.solve(state);
    expect(state.valid).toBe(false);
    expect(state.message).toContain('Unsolvable');
    for (const link of state.links) {
      expect(Number.isFinite(link.pose.position.x)).toBe(true);
      expect(Number.isFinite(link.pose.position.y)).toBe(true);
      expect(Number.isFinite(link.pose.angle)).toBe(true);
    }
  });

  it('inverse-poses a constrained right endpoint through the servo ROM', () => {
    const targetState = createDefaultState();
    const simulation = new MechanismSimulation();
    targetState.servo.angle = targetState.servo.maxAngle;
    simulation.solve(targetState);
    const targetLink = targetState.links.find((link) => link.id === 'tip-driver')!;
    const target = localToWorld({ x: targetLink.length / 2, y: 0 }, targetLink.pose);

    const state = createDefaultState();
    simulation.solve(state);
    simulation.solveForLinkEndpoint(state, 'tip-driver', target);
    expect(state.valid, state.message).toBe(true);
    expect(state.servo.angle).toBeCloseTo(state.servo.maxAngle, 2);
    expect(state.enabled).toBe(false);
  });

  it('directly drives the crank and rotates a free link from right-end targets', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    simulation.solveForLinkEndpoint(state, 'crank', { x: -88, y: 105 });
    expect(state.servo.angle).toBeCloseTo(state.servo.minAngle, 5);

    state.links.push({
      id: 'free-test',
      name: 'Free test link',
      length: 20,
      width: 4,
      pose: { position: { x: 10, y: 0 }, angle: 0 },
    });
    simulation.solveForLinkEndpoint(state, 'free-test', { x: 0, y: 20 });
    const free = state.links.find((link) => link.id === 'free-test')!;
    const left = localToWorld({ x: -free.length / 2, y: 0 }, free.pose);
    const right = localToWorld({ x: free.length / 2, y: 0 }, free.pose);
    expect(left.x).toBeCloseTo(0);
    expect(left.y).toBeCloseTo(0);
    expect(right.x).toBeCloseTo(0);
    expect(right.y).toBeCloseTo(20);
  });

  it('rejects and restores a rectangular link that would enter finger geometry', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    simulation.solve(state);
    state.links.push({
      id: 'collision-test',
      name: 'Collision test link',
      length: 30,
      width: 4,
      pose: { position: { x: 20, y: 140 }, angle: 0 },
    });
    simulation.solve(state);
    expect(state.valid).toBe(true);
    const safePose = structuredClone(state.links.find((link) => link.id === 'collision-test')!.pose);
    const testLink = state.links.find((link) => link.id === 'collision-test')!;
    testLink.length = 80;
    testLink.pose = { position: { x: -25, y: 55 }, angle: 0 };
    simulation.solve(state);
    expect(state.valid).toBe(false);
    expect(state.message).toContain('Collision');
    expect(testLink.pose).toEqual(safePose);
    expect(testLink.length).toBe(30);
  });

  it('propagates an added link from its left-end joint on a moving parent', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    simulation.solve(state);
    const crank = state.links.find((link) => link.id === 'crank')!;
    state.links.push({
      id: 'attached-test',
      name: 'Attached test link',
      length: 20,
      width: 4,
      pose: {
        position: { x: crank.pose.position.x, y: crank.pose.position.y + 10 },
        angle: Math.PI / 2,
      },
    });
    state.joints.push({
      id: 'attached-test-joint',
      name: 'Attached test joint',
      linkAId: 'crank',
      linkBId: 'attached-test',
      localPointA: { x: 0, y: 0 },
      localPointB: { x: -10, y: 0 },
    });
    simulation.solve(state);
    state.servo.angle = state.servo.maxAngle;
    simulation.solve(state);
    const child = state.links.find((link) => link.id === 'attached-test')!;
    const childLeft = localToWorld({ x: -child.length / 2, y: 0 }, child.pose);
    expect(childLeft.x).toBeCloseTo(crank.pose.position.x);
    expect(childLeft.y).toBeCloseTo(crank.pose.position.y);
  });

  it('solves distinct middle and distal contactors on a jointed driver chain', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    simulation.solve(state);
    expect(state.valid, state.message).toBe(true);
    expect(state.contactors).toHaveLength(2);
    expect(state.contactors.map((contactor) => contactor.fingerSegment)).toEqual(['middle', 'distal']);
    expect(state.contactors.map((contactor) => contactor.linkId)).toEqual(['middle-driver', 'tip-driver']);
    expect(state.joints.find((joint) => joint.linkBId === 'middle-driver')).toBeDefined();
    expect(state.joints.find((joint) => joint.linkBId === 'tip-driver')).toBeDefined();
  });

  it('rejects linkage poses below the base rail and through the dorsal ground plane', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    simulation.solve(state);
    state.links.push({
      id: 'mount-collision-test',
      name: 'Mount collision test',
      length: 20,
      width: 4,
      pose: { position: { x: 20, y: 140 }, angle: 0 },
    });
    simulation.solve(state);
    const testLink = state.links.find((link) => link.id === 'mount-collision-test')!;
    testLink.pose = { position: { x: -90, y: 90 }, angle: 0 };
    simulation.solve(state);
    expect(state.valid).toBe(false);
    expect(state.message).toContain('below dorsal base rail');

    testLink.pose = { position: { x: -122, y: 70 }, angle: Math.PI / 2 };
    simulation.solve(state);
    expect(state.valid).toBe(false);
    expect(state.message).toContain('dorsal ground plane');
  });

  it('derives servo and rocker pivots from the dorsal mount configuration', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    state.ground.servoGroundOffset = 37;
    state.ground.baseRailAngleOffset = 0;
    state.links.find((link) => link.id === 'ground-rail')!.length = 55;
    simulation.solve(state);
    expect(state.servo.groundPoint).toEqual({ x: -115, y: 107 });
    expect(state.fourBar.rockerGroundPoint.x).toBeCloseTo(-60);
    expect(state.fourBar.rockerGroundPoint.y).toBeCloseTo(107);
  });
});
