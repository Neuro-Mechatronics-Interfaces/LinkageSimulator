import { describe, expect, it } from 'vitest';
import { distance, localToWorld } from '../geometry';
import { createDefaultState, type Link, type RevoluteJoint } from '../model';
import { placeUnderconstrainedLinks, solveAnalyticConstraints } from './analyticConstraintSolver';
import { buildConstraintGraph } from './ConstraintGraph';

function dyad(
  anchorB = { x: 8, y: 0 },
  radiusBLocal = { x: 4, y: 1 },
): { links: Link[]; joints: RevoluteJoint[] } {
  const links: Link[] = [
    {
      id: 'left-link',
      name: 'Left',
      length: 17,
      width: 3,
      pose: { position: { x: 0, y: 0 }, angle: 0 },
    },
    {
      id: 'right-link',
      name: 'Right',
      length: 19,
      width: 3,
      pose: { position: { x: 0, y: 0 }, angle: 0 },
    },
  ];
  const joints: RevoluteJoint[] = [
    {
      id: 'left-ground',
      name: 'Left ground',
      linkAId: null,
      linkBId: 'left-link',
      groundPoint: { x: 0, y: 0 },
      localPointB: { x: 1, y: 2 },
    },
    {
      id: 'right-ground',
      name: 'Right ground',
      linkAId: null,
      linkBId: 'right-link',
      groundPoint: anchorB,
      localPointB: { x: -2, y: 1 },
    },
    {
      id: 'shared-hinge',
      name: 'Shared',
      linkAId: 'left-link',
      linkBId: 'right-link',
      localPointA: { x: 4, y: 6 },
      localPointB: radiusBLocal,
    },
  ];
  return { links, joints };
}

function prefixedDyad(
  mechanism: { links: Link[]; joints: RevoluteJoint[] },
  prefix: string,
  offset: { x: number; y: number },
): { links: Link[]; joints: RevoluteJoint[] } {
  const linkIds = new Map(mechanism.links.map((link) => [link.id, `${prefix}${link.id}`]));
  return {
    links: mechanism.links.map((link) => ({
      ...link,
      id: linkIds.get(link.id)!,
      pose: {
        position: {
          x: link.pose.position.x + offset.x,
          y: link.pose.position.y + offset.y,
        },
        angle: link.pose.angle,
      },
    })),
    joints: mechanism.joints.map((joint) => ({
      ...joint,
      id: `${prefix}${joint.id}`,
      linkAId: joint.linkAId === null ? null : linkIds.get(joint.linkAId)!,
      linkBId: linkIds.get(joint.linkBId)!,
      localPointA: joint.localPointA ? { ...joint.localPointA } : undefined,
      localPointB: { ...joint.localPointB },
      groundPoint: joint.groundPoint
        ? { x: joint.groundPoint.x + offset.x, y: joint.groundPoint.y + offset.y }
        : undefined,
    })),
  };
}

function jointClosure(links: Link[], joint: RevoluteJoint): number {
  const linkB = links.find((link) => link.id === joint.linkBId)!;
  const pointB = localToWorld(joint.localPointB, linkB.pose);
  const pointA = joint.linkAId === null
    ? joint.groundPoint!
    : localToWorld(
      joint.localPointA!,
      links.find((link) => link.id === joint.linkAId)!.pose,
    );
  return distance(pointA, pointB);
}

describe('topology-discovered analytic dyads', () => {
  it('uses prior joint history to choose either assembly branch', () => {
    const upper = dyad();
    const upperResult = solveAnalyticConstraints(
      buildConstraintGraph(upper),
      { previousJointPositions: new Map([['shared-hinge', { x: 3, y: 10 }]]) },
    );
    expect(upperResult.valid).toBe(true);
    expect(upperResult.unresolvedLinkIds).toEqual([]);
    const upperPoint = upperResult.steps[0]?.selectedPoint;
    expect(upperPoint?.y).toBeGreaterThan(0);
    expect(upper.joints.every((joint) => jointClosure(upper.links, joint) < 1e-8)).toBe(true);

    const lower = dyad();
    const lowerResult = solveAnalyticConstraints(
      buildConstraintGraph(lower),
      { previousJointPositions: new Map([['shared-hinge', { x: 3, y: -10 }]]) },
    );
    expect(lowerResult.valid).toBe(true);
    expect(lowerResult.steps[0]?.selectedPoint?.y).toBeLessThan(0);
    expect(lower.joints.every((joint) => jointClosure(lower.links, joint) < 1e-8)).toBe(true);
  });

  it('supports arbitrary off-axis local coordinates rather than link endpoints', () => {
    const mechanism = dyad();
    const result = solveAnalyticConstraints(buildConstraintGraph(mechanism), {
      previousJointPositions: new Map([['shared-hinge', { x: 3, y: 4 }]]),
    });
    expect(result.valid).toBe(true);
    expect(result.analyticSolveCount).toBe(1);
    expect(mechanism.links[0]!.length).toBe(17);
    expect(mechanism.links[1]!.length).toBe(19);
    expect(mechanism.joints.every((joint) => jointClosure(mechanism.links, joint) < 1e-8)).toBe(true);
  });

  it('resolves a tangent dyad and reports its branch-merging singularity', () => {
    const mechanism = dyad({ x: 10, y: 0 }, { x: 3, y: 1 });
    const result = solveAnalyticConstraints(buildConstraintGraph(mechanism));
    expect(result.valid).toBe(true);
    expect(result.singularJointIds.has('shared-hinge')).toBe(true);
    expect(result.steps[0]?.intersectionKind).toBe('tangent');
    expect(result.unresolvedLinkIds).toEqual([]);
  });

  it('distinguishes unreachable, unequal-concentric, and coincident geometry', () => {
    const unreachable = dyad({ x: 12, y: 0 }, { x: 3, y: 1 });
    const failed = solveAnalyticConstraints(buildConstraintGraph(unreachable));
    expect(failed.valid).toBe(false);
    expect(failed.resolutions.some((resolution) =>
      resolution.kind === 'inconsistent' && resolution.reason === 'unreachable-dyad',
    )).toBe(true);

    const concentric = dyad({ x: 0, y: 0 });
    const impossible = solveAnalyticConstraints(buildConstraintGraph(concentric));
    expect(impossible.valid).toBe(false);
    expect(impossible.singularJointIds.has('shared-hinge')).toBe(true);
    expect(impossible.resolutions.some((resolution) =>
      resolution.kind === 'inconsistent' && resolution.reason === 'unreachable-dyad',
    )).toBe(true);

    const coincident = dyad({ x: 0, y: 0 }, { x: 3, y: 1 });
    const free = solveAnalyticConstraints(buildConstraintGraph(coincident));
    expect(free.valid).toBe(true);
    expect(free.singularJointIds.has('shared-hinge')).toBe(true);
    expect(free.unresolvedLinkIds).toEqual(['left-link', 'right-link']);
    expect(free.resolutions.some((resolution) => resolution.kind === 'underdetermined')).toBe(true);
  });

  it('does not let a stalled dyad block a later solvable dyad in either insertion order', () => {
    for (const reverseJoints of [false, true]) {
      const stalled = prefixedDyad(dyad({ x: 0, y: 0 }, { x: 3, y: 1 }), 'stalled-', { x: 0, y: 0 });
      const solvable = prefixedDyad(dyad(), 'solvable-', { x: 30, y: 0 });
      const joints = [...stalled.joints, ...solvable.joints];
      if (reverseJoints) joints.reverse();
      const result = solveAnalyticConstraints(buildConstraintGraph({
        links: [...stalled.links, ...solvable.links],
        joints,
      }));

      expect(result.valid).toBe(true);
      expect(result.resolvedLinkIds.has('solvable-left-link')).toBe(true);
      expect(result.resolvedLinkIds.has('solvable-right-link')).toBe(true);
      expect(result.unresolvedLinkIds).toEqual(['stalled-left-link', 'stalled-right-link']);
      expect(result.steps.some((step) => step.jointId === 'solvable-shared-hinge')).toBe(true);
    }
  });
});

describe('analytic seed and propagation', () => {
  it('recognizes the default mechanism as a dyad without consulting role IDs', () => {
    const state = createDefaultState();
    const graph = buildConstraintGraph(state);
    const result = solveAnalyticConstraints(graph);
    expect(result.valid, result.messages.join('; ')).toBe(true);
    expect(result.steps).toHaveLength(1);
    expect(result.steps[0]?.linkIds).toContain('coupler');
    expect(result.resolvedLinkIds.has('anchor-driver')).toBe(true);
    placeUnderconstrainedLinks(graph, result);
    expect(result.intentionallyFreeLinkIds).toEqual(new Set(['middle-driver', 'tip-driver']));
    for (const joint of state.joints) expect(jointClosure(state.links, joint), joint.id).toBeLessThan(1e-7);
  });

  it('seeds a servo from its exact local revolute point and absolute world angle', () => {
    const driven: Link = {
      id: 'driven-random-id',
      name: 'Driven',
      length: 30,
      width: 4,
      pose: { position: { x: 999, y: 999 }, angle: 0 },
    };
    const joint: RevoluteJoint = {
      id: 'actuated-random-joint',
      name: 'Actuated',
      linkAId: null,
      linkBId: driven.id,
      groundPoint: { x: 7, y: -3 },
      localPointB: { x: 2.5, y: -4 },
    };
    const graph = buildConstraintGraph({
      links: [driven],
      joints: [joint],
      servo: {
        id: 'servo-random',
        name: 'Servo',
        groundPoint: { x: 7, y: -3 },
        drivenLinkId: driven.id,
        revoluteJointId: joint.id,
        angle: 1.2,
        minAngle: -2,
        maxAngle: 2,
        speed: 1,
        direction: 1,
      },
    });
    const result = solveAnalyticConstraints(graph);
    expect(result.valid).toBe(true);
    expect(driven.pose.angle).toBeCloseTo(1.2, 10);
    expect(localToWorld(joint.localPointB, driven.pose).x).toBeCloseTo(7, 10);
    expect(localToWorld(joint.localPointB, driven.pose).y).toBeCloseTo(-3, 10);
  });

  it('propagates a locked relative angle from a resolved parent', () => {
    const parent: Link = {
      id: 'parent', name: 'Parent', length: 10, width: 2,
      fixed: true,
      pose: { position: { x: 3, y: 4 }, angle: 0.7 },
    };
    const child: Link = {
      id: 'child', name: 'Child', length: 9, width: 2,
      pose: { position: { x: 100, y: 100 }, angle: 0 },
    };
    const joint: RevoluteJoint = {
      id: 'locked', name: 'Locked', linkAId: parent.id, linkBId: child.id,
      localPointA: { x: 1, y: -2 }, localPointB: { x: -3, y: 2 },
      minAngle: -0.35, maxAngle: -0.35,
    };
    const result = solveAnalyticConstraints(buildConstraintGraph({ links: [parent, child], joints: [joint] }));
    expect(result.valid).toBe(true);
    expect(child.pose.angle).toBeCloseTo(0.35, 10);
    expect(jointClosure([parent, child], joint)).toBeLessThan(1e-9);
  });
});
