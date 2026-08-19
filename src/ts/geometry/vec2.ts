export interface Vec2 {
  x: number;
  y: number;
}

export const vec = (x = 0, y = 0): Vec2 => ({ x, y });

export const add = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x + b.x, y: a.y + b.y });
export const subtract = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x - b.x, y: a.y - b.y });
export const scale = (value: Vec2, scalar: number): Vec2 => ({ x: value.x * scalar, y: value.y * scalar });
export const dot = (a: Vec2, b: Vec2): number => a.x * b.x + a.y * b.y;
export const magnitudeSquared = (value: Vec2): number => dot(value, value);
export const magnitude = (value: Vec2): number => Math.sqrt(magnitudeSquared(value));
export const distance = (a: Vec2, b: Vec2): number => magnitude(subtract(a, b));

export function normalize(value: Vec2): Vec2 | null {
  const length = magnitude(value);
  return length > Number.EPSILON ? scale(value, 1 / length) : null;
}

export function rotate(value: Vec2, angle: number): Vec2 {
  const cosine = Math.cos(angle);
  const sine = Math.sin(angle);
  return { x: value.x * cosine - value.y * sine, y: value.x * sine + value.y * cosine };
}

export const perpendicular = (value: Vec2): Vec2 => ({ x: -value.y, y: value.x });

export function normalizeAngle(angle: number): number {
  const wrapped = ((angle + Math.PI) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2) - Math.PI;
  return wrapped === -Math.PI ? Math.PI : wrapped;
}

export interface Pose2D {
  position: Vec2;
  angle: number;
}

export const localToWorld = (localPoint: Vec2, pose: Pose2D): Vec2 => add(pose.position, rotate(localPoint, pose.angle));
export const worldToLocal = (worldPoint: Vec2, pose: Pose2D): Vec2 => rotate(subtract(worldPoint, pose.position), -pose.angle);

export function closestPointOnSegment(point: Vec2, start: Vec2, end: Vec2): Vec2 {
  const segment = subtract(end, start);
  const denominator = magnitudeSquared(segment);
  if (denominator <= Number.EPSILON) return { ...start };
  const t = Math.max(0, Math.min(1, dot(subtract(point, start), segment) / denominator));
  return add(start, scale(segment, t));
}

const orientation = (a: Vec2, b: Vec2, c: Vec2): number =>
  (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

function segmentsIntersect(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2): boolean {
  const o1 = orientation(a0, a1, b0);
  const o2 = orientation(a0, a1, b1);
  const o3 = orientation(b0, b1, a0);
  const o4 = orientation(b0, b1, a1);
  const epsilon = 1e-9;
  if (((o1 > epsilon && o2 < -epsilon) || (o1 < -epsilon && o2 > epsilon)) &&
      ((o3 > epsilon && o4 < -epsilon) || (o3 < -epsilon && o4 > epsilon))) return true;
  const onSegment = (point: Vec2, start: Vec2, end: Vec2): boolean =>
    Math.abs(orientation(start, end, point)) <= epsilon &&
    point.x >= Math.min(start.x, end.x) - epsilon && point.x <= Math.max(start.x, end.x) + epsilon &&
    point.y >= Math.min(start.y, end.y) - epsilon && point.y <= Math.max(start.y, end.y) + epsilon;
  return onSegment(b0, a0, a1) || onSegment(b1, a0, a1) ||
    onSegment(a0, b0, b1) || onSegment(a1, b0, b1);
}

export function segmentSegmentDistance(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2): number {
  if (segmentsIntersect(a0, a1, b0, b1)) return 0;
  return Math.min(
    distance(a0, closestPointOnSegment(a0, b0, b1)),
    distance(a1, closestPointOnSegment(a1, b0, b1)),
    distance(b0, closestPointOnSegment(b0, a0, a1)),
    distance(b1, closestPointOnSegment(b1, a0, a1)),
  );
}
