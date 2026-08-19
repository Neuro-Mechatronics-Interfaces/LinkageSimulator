import {
  dot,
  localToWorld,
  normalize,
  perpendicular,
  rotate,
  subtract,
  type Pose2D,
  type Vec2,
} from '../geometry';
import type { LinearSlotJoint } from '../model';

export interface LinearSlotWorldGeometry {
  origin: Vec2;
  axis: Vec2;
  normal: Vec2;
  pin: Vec2;
  /** Signed pin offset normal to the infinite slot centerline. */
  normalOffset: number;
  /** Signed pin travel from the slot origin along the centerline. */
  travel: number;
}

/** Evaluates a slot from explicit body poses without reading mutable graph state. */
export function evaluateLinearSlotGeometry(
  joint: LinearSlotJoint,
  slotPose: Pose2D | null,
  pinPose: Pose2D,
): LinearSlotWorldGeometry {
  const localAxis = normalize(joint.slotDirection);
  if (localAxis === null) {
    throw new RangeError(`Linear slot ${joint.id} has a degenerate direction`);
  }
  const origin = slotPose === null
    ? { ...joint.slotOrigin }
    : localToWorld(joint.slotOrigin, slotPose);
  const axis = slotPose === null ? localAxis : rotate(localAxis, slotPose.angle);
  const normal = perpendicular(axis);
  const pin = localToWorld(joint.pinLocalPoint, pinPose);
  const offset = subtract(pin, origin);
  return {
    origin,
    axis,
    normal,
    pin,
    normalOffset: dot(offset, normal),
    travel: dot(offset, axis),
  };
}

