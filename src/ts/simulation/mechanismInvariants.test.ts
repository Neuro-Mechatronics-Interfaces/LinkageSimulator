import { describe, expect, it } from 'vitest';
import type { Link, RevoluteJoint } from '../model';
import { buildConstraintGraph } from './ConstraintGraph';
import { validateMechanismInvariants } from './mechanismInvariants';

describe('mechanism hard invariants', () => {
  it('rejects a closed revolute whose relative angle is outside its ROM', () => {
    const parent: Link = {
      id: 'parent',
      name: 'Parent',
      length: 10,
      width: 2,
      fixed: true,
      pose: { position: { x: 0, y: 0 }, angle: 0 },
    };
    const child: Link = {
      id: 'child',
      name: 'Child',
      length: 10,
      width: 2,
      pose: { position: { x: 0, y: 0 }, angle: 1 },
    };
    const joint: RevoluteJoint = {
      id: 'limited',
      name: 'Limited',
      linkAId: parent.id,
      linkBId: child.id,
      localPointA: { x: 0, y: 0 },
      localPointB: { x: 0, y: 0 },
      minAngle: -0.5,
      maxAngle: 0.5,
    };
    const graph = buildConstraintGraph({ links: [parent, child], joints: [joint] });

    const result = validateMechanismInvariants(graph);

    expect(result.maximumClosureError).toBe(0);
    expect(result.valid).toBe(false);
    expect(result.invalidJointIds).toEqual(['limited']);
    expect(result.messages.join(' ')).toContain('outside ROM');
  });

  it('accepts a closed revolute inside a wrapped ROM', () => {
    const body: Link = {
      id: 'wrapped-body',
      name: 'Wrapped body',
      length: 10,
      width: 2,
      pose: { position: { x: 0, y: 0 }, angle: Math.PI - 0.05 },
    };
    const joint: RevoluteJoint = {
      id: 'wrapped-limit',
      name: 'Wrapped limit',
      linkAId: null,
      linkBId: body.id,
      groundPoint: { x: 0, y: 0 },
      localPointB: { x: 0, y: 0 },
      minAngle: 2.8,
      maxAngle: -2.8,
    };
    const graph = buildConstraintGraph({ links: [body], joints: [joint] });

    expect(validateMechanismInvariants(graph).valid).toBe(true);
  });
});
