import { describe, expect, it } from 'vitest';
import type { Link, RevoluteJoint, ServoJoint } from '../model';
import {
  WORLD_BODY_ID,
  buildConstraintGraph,
  connectedComponents,
} from './ConstraintGraph';

function link(id: string, fixed = false, x = 0): Link {
  return {
    id,
    name: id,
    length: 10,
    width: 2,
    pose: { position: { x, y: 0 }, angle: 0 },
    ...(fixed ? { fixed: true } : {}),
  };
}

function joint(
  id: string,
  linkAId: string | null,
  linkBId: string,
  x = 0,
): RevoluteJoint {
  return linkAId === null
    ? {
        id,
        name: id,
        linkAId: null,
        linkBId,
        groundPoint: { x, y: 0 },
        localPointB: { x: 0, y: 0 },
      }
    : {
        id,
        name: id,
        linkAId,
        linkBId,
        localPointA: { x: 0, y: 0 },
        localPointB: { x: 0, y: 0 },
      };
}

describe('ConstraintGraph', () => {
  it('represents a single free link and the world as distinct bodies', () => {
    const graph = buildConstraintGraph({ links: [link('floating')], joints: [] });
    expect([...graph.bodies.keys()]).toEqual([WORLD_BODY_ID, 'floating']);
    expect(graph.constraints).toHaveLength(0);
    expect(connectedComponents(graph)).toMatchObject([
      { linkIds: ['floating'], anchored: false, jointIds: [] },
    ]);
  });

  it('represents fixed and ground-revolute anchoring explicitly', () => {
    const graph = buildConstraintGraph({
      links: [link('fixed-body', true), link('pivoting')],
      joints: [
        joint('fixed-to-pivoting', 'fixed-body', 'pivoting'),
        joint('pivoting-to-world', null, 'pivoting'),
      ],
    });
    const constraintsByKind = graph.constraints.map((constraint) => constraint.kind);
    expect(constraintsByKind).toEqual(['fixed', 'revolute', 'revolute']);
    expect(graph.constraintsByBodyId.get(WORLD_BODY_ID)).toHaveLength(2);
    expect(connectedComponents(graph)).toMatchObject([
      {
        bodyIds: [WORLD_BODY_ID, 'fixed-body', 'pivoting'],
        linkIds: ['fixed-body', 'pivoting'],
        fixedLinkIds: ['fixed-body'],
        anchored: true,
      },
    ]);
  });

  it('discovers serial chains from topology independent of insertion order', () => {
    const links = [link('z-child'), link('a-root'), link('m-middle')];
    const joints = [
      joint('second', 'm-middle', 'z-child'),
      joint('first', 'a-root', 'm-middle'),
    ];
    const forward = connectedComponents(buildConstraintGraph({ links, joints }));
    const reversed = connectedComponents(buildConstraintGraph({
      links: [...links].reverse(),
      joints: [...joints].reverse(),
    }));
    expect(forward).toMatchObject([
      {
        linkIds: ['a-root', 'm-middle', 'z-child'],
        jointIds: ['first', 'second'],
        anchored: false,
      },
    ]);
    expect(reversed).toEqual(forward);
  });

  it('keeps independent grounded islands in separate component blocks', () => {
    const graph = buildConstraintGraph({
      links: [link('right'), link('left'), link('free-a'), link('free-b')],
      joints: [
        joint('right-ground', null, 'right', 20),
        joint('left-ground', null, 'left', -20),
        joint('free-pair', 'free-a', 'free-b'),
      ],
    });
    const components = connectedComponents(graph);
    expect(components.map((component) => component.linkIds)).toEqual([
      ['free-a', 'free-b'],
      ['left'],
      ['right'],
    ]);
    expect(components.map((component) => component.anchored)).toEqual([false, true, true]);
  });

  it('represents a closed four-bar without assigning topology roles by ID', () => {
    const links = [link('ground', true), link('one'), link('two'), link('three')];
    const joints = [
      joint('j0', 'ground', 'one'),
      joint('j1', 'one', 'two'),
      joint('j2', 'two', 'three'),
      joint('j3', 'ground', 'three'),
    ];
    const component = connectedComponents(buildConstraintGraph({ links, joints }))[0];
    expect(component).toMatchObject({
      linkIds: ['ground', 'one', 'three', 'two'],
      fixedLinkIds: ['ground'],
      jointIds: ['j0', 'j1', 'j2', 'j3'],
      anchored: true,
    });
  });

  it('adds locked-angle and prescribed-actuator constraints separately', () => {
    const driven = link('driven');
    const lockedJoint = {
      ...joint('locked', null, driven.id),
      minAngle: 0.25,
      maxAngle: 0.25 + 1e-10,
    };
    const servo: ServoJoint = {
      id: 'motor',
      name: 'motor',
      drivenLinkId: driven.id,
      revoluteJointId: lockedJoint.id,
      groundPoint: { x: 0, y: 0 },
      angle: 0.25,
      minAngle: -1,
      maxAngle: 1,
      speed: 1,
      direction: 1,
    };
    const graph = buildConstraintGraph({ links: [driven], joints: [lockedJoint], servo });
    expect(graph.constraints.map((constraint) => constraint.kind)).toEqual([
      'revolute',
      'locked-angle',
      'actuator',
    ]);
    expect(connectedComponents(graph)[0]).toMatchObject({ actuatorIds: ['motor'], anchored: true });
  });

  it('rejects invalid references instead of creating partial graph edges', () => {
    expect(() => buildConstraintGraph({
      links: [link('present')],
      joints: [joint('broken', 'missing', 'present')],
    })).toThrow(/missing linkA/);
  });
});
