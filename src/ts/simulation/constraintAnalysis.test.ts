import { describe, expect, it } from 'vitest';
import type { Vec2 } from '../geometry';
import { createDefaultState, type Link, type RevoluteJoint, type ServoJoint } from '../model';
import { buildConstraintGraph, connectedComponents } from './ConstraintGraph';
import {
  analyzeConstraintGraph,
  componentJacobian,
  createComponentResidualSystem,
} from './constraintAnalysis';
import { MechanismSimulation } from './MechanismSimulation';

function link(id: string, position: Vec2 = { x: 0, y: 0 }, angle = 0): Link {
  return {
    id,
    name: id,
    length: 10,
    width: 2,
    pose: { position, angle },
  };
}

function groundJoint(
  id: string,
  linkBId: string,
  groundPoint: Vec2,
  localPointB: Vec2,
): RevoluteJoint {
  return {
    id,
    name: id,
    linkAId: null,
    linkBId,
    groundPoint,
    localPointB,
  };
}

function linkJoint(
  id: string,
  linkAId: string,
  linkBId: string,
  localPointA: Vec2,
  localPointB: Vec2,
): RevoluteJoint {
  return {
    id,
    name: id,
    linkAId,
    linkBId,
    localPointA,
    localPointB,
  };
}

function servo(id: string, drivenLinkId: string, revoluteJointId: string, angle = 0): ServoJoint {
  return {
    id,
    name: id,
    groundPoint: { x: 0, y: 0 },
    drivenLinkId,
    revoluteJointId,
    angle,
    minAngle: -Math.PI,
    maxAngle: Math.PI,
    speed: 1,
    direction: 1,
  };
}

function fourBar(prefix: string, tangent = false): {
  links: Link[];
  joints: RevoluteJoint[];
  servo: ServoJoint;
} {
  const crank = link(`${prefix}-crank`);
  const coupler = link(`${prefix}-coupler`);
  const rocker = link(`${prefix}-rocker`);
  const crankGround = groundJoint(
    `${prefix}-crank-ground`,
    crank.id,
    { x: 0, y: 0 },
    { x: 0, y: 0 },
  );
  const rockerGround = groundJoint(
    `${prefix}-rocker-ground`,
    rocker.id,
    { x: 4, y: 0 },
    { x: 4, y: 0 },
  );
  const crankCouplerPoint = tangent ? { x: 1, y: 0 } : { x: 1, y: 1 };
  const couplerRockerPoint = tangent ? { x: 3, y: 0 } : { x: 3, y: 2 };
  return {
    links: [rocker, crank, coupler],
    joints: [
      rockerGround,
      linkJoint(
        `${prefix}-crank-coupler`,
        crank.id,
        coupler.id,
        crankCouplerPoint,
        crankCouplerPoint,
      ),
      crankGround,
      linkJoint(
        `${prefix}-coupler-rocker`,
        coupler.id,
        rocker.id,
        couplerRockerPoint,
        couplerRockerPoint,
      ),
    ],
    servo: servo(`${prefix}-servo`, crank.id, crankGround.id),
  };
}

describe('constraint residuals and mobility analysis', () => {
  it('reports one passive DOF for a grounded revolute link', () => {
    const pivot = link('pivot');
    const graph = buildConstraintGraph({
      links: [pivot],
      joints: [groundJoint('grounded', pivot.id, { x: 2, y: 3 }, { x: 2, y: 3 })],
    });
    const diagnostics = analyzeConstraintGraph(graph).components[0]!;
    expect(diagnostics.variableCount).toBe(3);
    expect(diagnostics.passiveResidualCount).toBe(2);
    expect(diagnostics.passiveJacobianRank).toBe(2);
    expect(diagnostics.passiveDof).toBe(1);
    expect(diagnostics.inconsistent).toBe(false);
    expect(diagnostics.singular).toBe(false);
  });

  it('removes the grounded-link DOF when its absolute servo angle is prescribed', () => {
    const driven = link('driven', { x: 0, y: 0 }, 0.4);
    const pivot = groundJoint('pivot', driven.id, { x: 0, y: 0 }, { x: 0, y: 0 });
    const graph = buildConstraintGraph({
      links: [driven],
      joints: [pivot],
      servo: servo('actuator', driven.id, pivot.id, 0.4),
    });
    const diagnostics = analyzeConstraintGraph(graph).components[0]!;
    expect(diagnostics.passiveJacobianRank).toBe(2);
    expect(diagnostics.passiveDof).toBe(1);
    expect(diagnostics.jacobianRank).toBe(3);
    expect(diagnostics.drivenDof).toBe(0);
    expect(diagnostics.residualNorm).toBeLessThan(1e-10);
  });

  it('reports two passive DOFs for a planar serial 2R chain', () => {
    const first = link('first');
    const second = link('second', { x: 10, y: 0 });
    const graph = buildConstraintGraph({
      links: [second, first],
      joints: [
        linkJoint('elbow', first.id, second.id, { x: 10, y: 0 }, { x: 0, y: 0 }),
        groundJoint('base', first.id, { x: 0, y: 0 }, { x: 0, y: 0 }),
      ],
    });
    const diagnostics = analyzeConstraintGraph(graph).components[0]!;
    expect(diagnostics.variableCount).toBe(6);
    expect(diagnostics.passiveJacobianRank).toBe(4);
    expect(diagnostics.passiveDof).toBe(2);
    expect(diagnostics.inconsistent).toBe(false);
  });

  it('finds four-bar mobility by Jacobian rank with and without its actuator', () => {
    for (const prefix of ['descriptive', 'random-7f3']) {
      const mechanism = fourBar(prefix);
      const graph = buildConstraintGraph(mechanism);
      const diagnostics = analyzeConstraintGraph(graph).components[0]!;
      expect(diagnostics.variableCount).toBe(9);
      expect(diagnostics.passiveJacobianRank).toBe(8);
      expect(diagnostics.passiveDof).toBe(1);
      expect(diagnostics.jacobianRank).toBe(9);
      expect(diagnostics.drivenDof).toBe(0);
      expect(diagnostics.inconsistent).toBe(false);
      expect(diagnostics.singular).toBe(false);
    }
  });

  it('evaluates arbitrary local attachment coordinates and a central-difference Jacobian', () => {
    const first = link('first', { x: 3, y: -4 }, Math.PI / 2);
    const second = link('second', { x: 2, y: -2 }, 0);
    const joint = linkJoint(
      'arbitrary-points',
      first.id,
      second.id,
      { x: 2, y: 1 },
      { x: 0, y: 0 },
    );
    const graph = buildConstraintGraph({ links: [first, second], joints: [joint] });
    const system = createComponentResidualSystem(graph, connectedComponents(graph)[0]!, {
      includeActuator: false,
    });
    expect(system.evaluate(system.initialConfiguration)).toEqual([0, 0]);
    const jacobian = componentJacobian(system);
    expect(jacobian).toHaveLength(2);
    expect(jacobian[0]).toHaveLength(6);
    expect(jacobian.flat().every(Number.isFinite)).toBe(true);
  });

  it('treats a locked revolute range as an angular equality', () => {
    const child = link('locked-child', { x: 0, y: 0 }, 0.3);
    const locked = {
      ...groundJoint('locked-pivot', child.id, { x: 0, y: 0 }, { x: 0, y: 0 }),
      minAngle: 0.3,
      maxAngle: 0.3,
    };
    const diagnostics = analyzeConstraintGraph(buildConstraintGraph({
      links: [child],
      joints: [locked],
    })).components[0]!;
    expect(diagnostics.passiveResidualCount).toBe(3);
    expect(diagnostics.passiveJacobianRank).toBe(3);
    expect(diagnostics.passiveDof).toBe(0);
    expect(diagnostics.inconsistent).toBe(false);
  });

  it('identifies disconnected components without treating them as inconsistent', () => {
    const grounded = link('grounded');
    const freeA = link('free-a');
    const freeB = link('free-b');
    const graph = buildConstraintGraph({
      links: [freeB, grounded, freeA],
      joints: [
        groundJoint('pivot', grounded.id, { x: 0, y: 0 }, { x: 0, y: 0 }),
        linkJoint('free-joint', freeA.id, freeB.id, { x: 0, y: 0 }, { x: 0, y: 0 }),
      ],
    });
    const diagnostics = analyzeConstraintGraph(graph);
    expect(diagnostics.components).toHaveLength(2);
    expect(diagnostics.disconnectedComponentIds).toHaveLength(1);
    expect(diagnostics.valid).toBe(true);
    expect(diagnostics.components.find((component) => !component.anchored)?.messages[0])
      .toMatch(/Disconnected component/);
  });

  it('distinguishes duplicate redundancy from structural overconstraint', () => {
    const body = link('body');
    const duplicateA = groundJoint('duplicate-a', body.id, { x: 0, y: 0 }, { x: 0, y: 0 });
    const duplicateB = groundJoint('duplicate-b', body.id, { x: 0, y: 0 }, { x: 0, y: 0 });
    const diagnostics = analyzeConstraintGraph(buildConstraintGraph({
      links: [body],
      joints: [duplicateA, duplicateB],
    })).components[0]!;
    expect(diagnostics.residualCount).toBe(4);
    expect(diagnostics.jacobianRank).toBe(2);
    expect(diagnostics.redundantConstraintCount).toBe(2);
    expect(diagnostics.overconstrained).toBe(false);
    expect(diagnostics.inconsistent).toBe(false);
    expect(diagnostics.singular).toBe(false);
  });

  it('reports a geometrically inconsistent overconstraint independently of redundancy', () => {
    const body = link('short-body', { x: 0.5, y: 0 });
    const graph = buildConstraintGraph({
      links: [body],
      joints: [
        groundJoint('left', body.id, { x: 0, y: 0 }, { x: 0, y: 0 }),
        groundJoint('right', body.id, { x: 2, y: 0 }, { x: 1, y: 0 }),
      ],
    });
    const diagnostics = analyzeConstraintGraph(graph).components[0]!;
    expect(diagnostics.variableCount).toBe(3);
    expect(diagnostics.residualCount).toBe(4);
    expect(diagnostics.jacobianRank).toBe(3);
    expect(diagnostics.redundantConstraintCount).toBe(1);
    expect(diagnostics.overconstrained).toBe(true);
    expect(diagnostics.inconsistent).toBe(true);
    expect(diagnostics.residualNorm).toBeGreaterThan(0.5);
  });

  it('detects the local Jacobian rank loss at a four-bar toggle', () => {
    const mechanism = fourBar('toggle', true);
    const graph = buildConstraintGraph({ links: mechanism.links, joints: mechanism.joints });
    const diagnostics = analyzeConstraintGraph(graph).components[0]!;
    expect(diagnostics.passiveJacobianRank).toBe(7);
    expect(diagnostics.expectedPassiveJacobianRank).toBe(8);
    expect(diagnostics.passiveDof).toBe(2);
    expect(diagnostics.singular).toBe(true);
    expect(diagnostics.inconsistent).toBe(false);
  });

  it('reports the complete demonstrator mechanism mobility before finger constraints', () => {
    const state = createDefaultState();
    new MechanismSimulation().solve(state);
    expect(state.valid, state.message).toBe(true);
    const diagnostics = analyzeConstraintGraph(buildConstraintGraph({
      links: state.links,
      joints: state.joints,
      servo: state.servo,
    })).components[0]!;
    expect(diagnostics.variableCount).toBe(18);
    expect(diagnostics.passiveJacobianRank).toBe(15);
    expect(diagnostics.passiveDof).toBe(3);
    expect(diagnostics.jacobianRank).toBe(16);
    expect(diagnostics.drivenDof).toBe(2);
    expect(diagnostics.residualNorm).toBeLessThan(1e-5);
  });
});
