import {
  distance,
  normalizeAngle,
  rotate,
  subtract,
  type Pose2D,
  type Vec2,
} from '../geometry';
import { SOLVER_TOLERANCES } from './solverTolerances';

export type RigidTransformResolution =
  | { kind: 'resolved'; pose: Pose2D; lengthError: number }
  | { kind: 'degenerate'; reason: 'coincident-local-points' }
  | { kind: 'inconsistent'; localDistance: number; worldDistance: number; lengthError: number };

/** Reconstruct a general planar rigid pose from two local/world point pairs. */
export function reconstructRigidPose(
  localPoint1: Vec2,
  localPoint2: Vec2,
  worldPoint1: Vec2,
  worldPoint2: Vec2,
  tolerance: number = SOLVER_TOLERANCES.closure,
): RigidTransformResolution {
  const localDistance = distance(localPoint1, localPoint2);
  if (localDistance <= SOLVER_TOLERANCES.length) {
    return { kind: 'degenerate', reason: 'coincident-local-points' };
  }

  const worldDistance = distance(worldPoint1, worldPoint2);
  const lengthError = Math.abs(worldDistance - localDistance);
  if (lengthError > tolerance) {
    return { kind: 'inconsistent', localDistance, worldDistance, lengthError };
  }

  const localDelta = subtract(localPoint2, localPoint1);
  const worldDelta = subtract(worldPoint2, worldPoint1);
  const angle = normalizeAngle(
    Math.atan2(worldDelta.y, worldDelta.x) - Math.atan2(localDelta.y, localDelta.x),
  );
  const rotatedLocalPoint = rotate(localPoint1, angle);
  return {
    kind: 'resolved',
    pose: {
      position: subtract(worldPoint1, rotatedLocalPoint),
      angle,
    },
    lengthError,
  };
}

/** Place a local attachment at a known world point for a prescribed angle. */
export function poseFromPointAndAngle(localPoint: Vec2, worldPoint: Vec2, angle: number): Pose2D {
  return {
    position: subtract(worldPoint, rotate(localPoint, angle)),
    angle: normalizeAngle(angle),
  };
}
