import { describe, expect, it } from 'vitest';
import { distance, localToWorld, normalizeAngle } from '../geometry';
import { createDefaultDigitStates, createDefaultState, DIGIT_IDS } from '../model';
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
    const mcpAngles: number[] = [];
    const pipAngles: number[] = [];
    for (let index = 0; index <= 100; index += 1) {
      state.servo.angle = state.servo.minAngle +
        ((state.servo.maxAngle - state.servo.minAngle) * index) / 100;
      simulation.solve(state);
      expect(state.valid, `${index}: ${state.message}`).toBe(true);
      expect(state.contactors[0]!.linkagePoint.y).toBeGreaterThan(state.contactors[0]!.fingerPoint.y);
      mcpAngles.push(state.hand.mcpAngle);
      pipAngles.push(state.hand.pipAngle);
      for (const link of state.links) {
        expect(Number.isFinite(link.pose.position.x)).toBe(true);
        expect(Number.isFinite(link.pose.position.y)).toBe(true);
        expect(Number.isFinite(link.pose.angle)).toBe(true);
      }
    }
    const degrees = (radians: number): number => radians * 180 / Math.PI;
    expect(degrees(state.servo.maxAngle - state.servo.minAngle)).toBeGreaterThanOrEqual(130);
    expect(degrees(Math.max(...mcpAngles) - Math.min(...mcpAngles))).toBeGreaterThan(85);
    expect(degrees(Math.max(...pipAngles) - Math.min(...pipAngles))).toBeGreaterThan(85);
  });

  it('keeps every default D2-D5 workspace valid across its independent servo sweep', () => {
    const digits = createDefaultDigitStates();
    const simulation = new MechanismSimulation();
    for (const digitId of DIGIT_IDS) {
      const state = digits[digitId];
      const mcpAngles: number[] = [];
      const pipAngles: number[] = [];
      for (let index = 0; index <= 284; index += 1) {
        state.servo.angle = state.servo.minAngle +
          ((state.servo.maxAngle - state.servo.minAngle) * index) / 284;
        simulation.solve(state);
        expect(state.valid, `${digitId} ${index}: ${state.message}`).toBe(true);
        mcpAngles.push(state.hand.mcpAngle);
        pipAngles.push(state.hand.pipAngle);
      }
      expect((Math.max(...mcpAngles) - Math.min(...mcpAngles)) * 180 / Math.PI, `${digitId} MCP range`)
        .toBeGreaterThan(80);
      expect((Math.max(...pipAngles) - Math.min(...pipAngles)) * 180 / Math.PI, `${digitId} PIP range`)
        .toBeGreaterThan(80);
      expect(Math.min(...mcpAngles) * 180 / Math.PI, `${digitId} MCP extension`).toBeLessThanOrEqual(-14);
      expect(Math.max(...mcpAngles) * 180 / Math.PI, `${digitId} MCP flexion`).toBeGreaterThan(84);
      expect(Math.min(...pipAngles) * 180 / Math.PI, `${digitId} PIP extension`).toBeLessThan(1);
      expect(Math.max(...pipAngles) * 180 / Math.PI, `${digitId} PIP flexion`).toBeGreaterThan(94);
    }
  });

  it('treats an intended driver-phalanx overlap as active contact instead of a collision', () => {
    const state = createDefaultState();
    state.servo.angle = 40 * Math.PI / 180;
    new MechanismSimulation().solve(state);
    expect(state.valid, state.message).toBe(true);
    expect(state.message).not.toContain('Collision');
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
    for (let index = 0; index <= 96; index += 1) {
      targetState.servo.angle = targetState.servo.minAngle +
        ((targetState.servo.maxAngle - targetState.servo.minAngle) * index) / 96;
      simulation.solve(targetState);
    }
    const targetLink = targetState.links.find((link) => link.id === 'tip-driver')!;
    const target = localToWorld({ x: targetLink.length / 2, y: 0 }, targetLink.pose);

    const state = createDefaultState();
    simulation.solve(state);
    simulation.solveForLinkEndpoint(state, 'tip-driver', target);
    expect(state.valid, state.message).toBe(true);
    const solvedLink = state.links.find((link) => link.id === 'tip-driver')!;
    const solvedEndpoint = localToWorld({ x: solvedLink.length / 2, y: 0 }, solvedLink.pose);
    expect(distance(solvedEndpoint, target)).toBeLessThan(0.75);
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
    state.ground.servoGroundOffset += 2;
    state.ground.baseRailAngleOffset = 0;
    state.links.find((link) => link.id === 'ground-rail')!.length += 3;
    simulation.solve(state);
    expect(state.servo.groundPoint.x).toBeCloseTo(state.ground.surfacePoint.x);
    expect(state.servo.groundPoint.y).toBeCloseTo(state.ground.surfacePoint.y + state.ground.servoGroundOffset);
    expect(state.fourBar.rockerGroundPoint.x).toBeCloseTo(state.servo.groundPoint.x + state.links.find((link) => link.id === 'ground-rail')!.length);
    expect(state.fourBar.rockerGroundPoint.y).toBeCloseTo(state.servo.groundPoint.y);
    for (const joint of state.joints) {
      const linkA = joint.linkAId ? state.links.find((link) => link.id === joint.linkAId) : undefined;
      const linkB = state.links.find((link) => link.id === joint.linkBId)!;
      const pointA = linkA && joint.localPointA
        ? localToWorld(joint.localPointA, linkA.pose)
        : joint.groundPoint!;
      const pointB = localToWorld(joint.localPointB, linkB.pose);
      expect(distance(pointA, pointB), joint.name).toBeLessThan(1e-6);
    }
  });

  it('updates every default distal joint circle throughout the servo sweep', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    for (let index = 0; index <= 20; index += 1) {
      state.servo.angle = state.servo.minAngle +
        ((state.servo.maxAngle - state.servo.minAngle) * index) / 20;
      simulation.solve(state);
      expect(state.valid, state.message).toBe(true);
      for (const joint of state.joints) {
        const linkA = joint.linkAId ? state.links.find((link) => link.id === joint.linkAId) : undefined;
        const linkB = state.links.find((link) => link.id === joint.linkBId)!;
        const pointA = linkA && joint.localPointA
          ? localToWorld(joint.localPointA, linkA.pose)
          : joint.groundPoint!;
        const pointB = localToWorld(joint.localPointB, linkB.pose);
        expect(distance(pointA, pointB), `${index}: ${joint.name}`).toBeLessThan(1e-6);
      }
    }
  });

  it('locks only a bounded distal joint while other joints continue moving', () => {
    const state = createDefaultState();
    const simulation = new MechanismSimulation();
    simulation.solve(state);
    const anchor = state.links.find((link) => link.id === 'anchor-driver')!;
    const middle = state.links.find((link) => link.id === 'middle-driver')!;
    const tip = state.links.find((link) => link.id === 'tip-driver')!;
    const lockedRelativeAngle = normalizeAngle(middle.pose.angle - anchor.pose.angle);
    const middleJoint = state.joints.find((joint) => joint.id === 'middle-driver-joint')!;
    middleJoint.minAngle = lockedRelativeAngle;
    middleJoint.maxAngle = lockedRelativeAngle;
    const initialMcp = state.hand.mcpAngle;
    const initialTipAngle = tip.pose.angle;

    let foundContinuingPose = false;
    for (let index = 1; index <= 20; index += 1) {
      state.servo.angle = state.servo.minAngle +
        ((state.servo.maxAngle - state.servo.minAngle) * index) / 20;
      simulation.solve(state);
      if (state.valid && Math.abs(state.hand.mcpAngle - initialMcp) + Math.abs(tip.pose.angle - initialTipAngle) > 1e-3) {
        foundContinuingPose = true;
        break;
      }
    }

    expect(state.valid, state.message).toBe(true);
    expect(foundContinuingPose).toBe(true);
    expect(normalizeAngle(middle.pose.angle - anchor.pose.angle)).toBeCloseTo(lockedRelativeAngle, 7);
    expect(state.jointConstraintStatus.find((status) => status.jointId === 'middle-driver-joint')?.state)
      .not.toBe('free');
    expect(Math.abs(state.hand.mcpAngle - initialMcp) + Math.abs(tip.pose.angle - initialTipAngle)).toBeGreaterThan(1e-3);
    expect(state.servo.angle).toBeGreaterThan(state.servo.minAngle);
  });
});
