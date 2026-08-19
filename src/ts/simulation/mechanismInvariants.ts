import { distance, localToWorld } from '../geometry';
import type { ComponentId } from '../model';
import { WORLD_BODY_ID, type ConstraintGraph } from './ConstraintGraph';
import { isFinitePose } from './analyticConstraintSolver';
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
  const invalidJointIds: ComponentId[] = [];
  const messages: string[] = [];
  let maximumClosureError = 0;

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
      invalidJointIds.push(constraint.jointId);
      messages.push(`Joint ${constraint.jointId} closure error is ${Number.isFinite(error) ? error.toPrecision(4) : 'non-finite'}`);
    }
  }

  return {
    valid: invalidLinkIds.length === 0 && invalidJointIds.length === 0,
    maximumClosureError,
    invalidLinkIds,
    invalidJointIds,
    messages,
  };
}

