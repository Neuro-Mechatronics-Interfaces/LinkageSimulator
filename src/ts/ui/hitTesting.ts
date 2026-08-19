import { distance, localToWorld, worldToLocal, type Vec2 } from '../geometry';
import type { Selection, SimulationState } from '../model';

export function hitTestLinkEnd(state: SimulationState, worldPoint: Vec2, tolerance: number): Selection {
  for (let index = state.links.length - 1; index >= 0; index -= 1) {
    const link = state.links[index];
    if (!link || link.fixed) continue;
    const rightEnd = localToWorld({ x: link.length / 2, y: 0 }, link.pose);
    if (distance(worldPoint, rightEnd) <= tolerance * 1.35) return { kind: 'link-end', id: link.id };
  }
  return null;
}

export function hitTest(state: SimulationState, worldPoint: Vec2, tolerance: number): Selection {
  for (let index = state.contactors.length - 1; index >= 0; index -= 1) {
    const contactor = state.contactors[index];
    if (contactor && distance(worldPoint, contactor.linkagePoint) <= tolerance * 1.3) {
      return { kind: 'contactor', id: contactor.id };
    }
  }

  if (distance(worldPoint, state.servo.groundPoint) <= tolerance * 1.6) {
    return { kind: 'servo', id: state.servo.id };
  }

  for (let index = state.joints.length - 1; index >= 0; index -= 1) {
    const joint = state.joints[index];
    if (!joint) continue;
    const link = state.links.find((candidate) => candidate.id === joint.linkBId);
    if (!link) continue;
    const jointPoint = joint.groundPoint ?? localToWorld(joint.localPointB, link.pose);
    if (distance(worldPoint, jointPoint) <= tolerance) return { kind: 'joint', id: joint.id };
  }

  for (let index = state.links.length - 1; index >= 0; index -= 1) {
    const link = state.links[index];
    if (!link) continue;
    const localPoint = worldToLocal(worldPoint, link.pose);
    if (
      Math.abs(localPoint.x) <= link.length / 2 + tolerance &&
      Math.abs(localPoint.y) <= link.width / 2 + tolerance
    ) {
      return { kind: 'link', id: link.id };
    }
  }
  return null;
}
