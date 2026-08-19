import { distance, localToWorld } from '../geometry';
import type { ComponentId } from '../model';
import { WORLD_BODY_ID, type ConstraintGraph } from './ConstraintGraph';
import {
  isAngleWithinJointRom,
  isFinitePose,
  relativeJointAngle,
} from './analyticConstraintSolver';
import { evaluateLinearSlotGeometry } from './linearSlotGeometry';
import { SOLVER_TOLERANCES } from './solverTolerances';

export interface MechanismInvariantResult {
  valid: boolean;
  maximumClosureError: number;
  invalidLinkIds: ComponentId[];
  invalidJointIds: ComponentId[];
  messages: string[];
}

/** Verifies finite body poses and hard revolute closure before committing a frame. */
export function validateMechanismInvariants(
  graph: ConstraintGraph,
  closureTolerance = SOLVER_TOLERANCES.closure,
): MechanismInvariantResult {
  const invalidLinkIds: ComponentId[] = [];
  const invalidJointIds = new Set<ComponentId>();
  const messages: string[] = [];
  let maximumClosureError = 0;
  const actuatorJointId = graph.constraints.find((constraint) => constraint.kind === 'actuator')
    ?.revoluteJointId;

  for (const body of graph.bodies.values()) {
    if (body.kind !== 'link' || isFinitePose(body.link.pose)) continue;
    invalidLinkIds.push(body.id);
    messages.push(`Link ${body.id} has a non-finite pose`);
  }
  for (const constraint of graph.jointConstraints.values()) {
    const linkBBody = graph.bodies.get(constraint.bodyBId);
    if (linkBBody?.kind !== 'link') continue;
    const pointB = localToWorld(constraint.joint.localPointB, linkBBody.link.pose);
    const pointA = constraint.bodyAId === WORLD_BODY_ID
      ? constraint.joint.groundPoint
      : constraint.joint.localPointA === undefined
        ? undefined
        : (() => {
          const body = graph.bodies.get(constraint.bodyAId);
          return body?.kind === 'link'
            ? localToWorld(constraint.joint.localPointA, body.link.pose)
            : undefined;
        })();
    const error = pointA ? distance(pointA, pointB) : Number.POSITIVE_INFINITY;
    maximumClosureError = Math.max(maximumClosureError, error);
    if (!Number.isFinite(error) || error > closureTolerance) {
      invalidJointIds.add(constraint.jointId);
      messages.push(`Joint ${constraint.jointId} closure error is ${Number.isFinite(error) ? error.toPrecision(4) : 'non-finite'}`);
    }

    // The actuator's absolute command bounds are authoritative for its
    // revolute. Every other finite ROM is a hard relative-angle inequality.
    const joint = constraint.joint;
    if (constraint.jointId === actuatorJointId ||
        joint.minAngle === undefined || joint.maxAngle === undefined) continue;
    const relativeAngle = relativeJointAngle(graph, joint);
    if (relativeAngle !== null && isAngleWithinJointRom(relativeAngle, joint)) continue;
    invalidJointIds.add(constraint.jointId);
    messages.push(
      `Joint ${constraint.jointId} angle ${relativeAngle === null ? 'unavailable' : relativeAngle.toPrecision(4)} is outside ROM`,
    );
  }
  for (const constraint of graph.linearSlotConstraints.values()) {
    const pinBody = graph.bodies.get(constraint.bodyBId);
    const slotBody = constraint.bodyAId === WORLD_BODY_ID
      ? null
      : graph.bodies.get(constraint.bodyAId);
    if (pinBody?.kind !== 'link' || (slotBody !== null && slotBody?.kind !== 'link')) {
      invalidJointIds.add(constraint.jointId);
      messages.push(`Linear slot ${constraint.jointId} references an unavailable body`);
      continue;
    }
    const geometry = evaluateLinearSlotGeometry(
      constraint.joint,
      slotBody === null ? null : slotBody.link.pose,
      pinBody.link.pose,
    );
    const closureError = Math.abs(geometry.normalOffset);
    maximumClosureError = Math.max(maximumClosureError, closureError);
    if (!Number.isFinite(closureError) || closureError > closureTolerance) {
      invalidJointIds.add(constraint.jointId);
      messages.push(
        `Linear slot ${constraint.jointId} normal closure error is ${Number.isFinite(closureError) ? closureError.toPrecision(4) : 'non-finite'}`,
      );
    }
    if (!Number.isFinite(geometry.travel) ||
        geometry.travel < constraint.joint.minTravel - SOLVER_TOLERANCES.slotTravel ||
        geometry.travel > constraint.joint.maxTravel + SOLVER_TOLERANCES.slotTravel) {
      invalidJointIds.add(constraint.jointId);
      messages.push(
        `Linear slot ${constraint.jointId} travel ${Number.isFinite(geometry.travel) ? geometry.travel.toPrecision(4) : 'non-finite'} is outside bounds`,
      );
    }
  }

  return {
    valid: invalidLinkIds.length === 0 && invalidJointIds.size === 0,
    maximumClosureError,
    invalidLinkIds,
    invalidJointIds: [...invalidJointIds],
    messages,
  };
}
