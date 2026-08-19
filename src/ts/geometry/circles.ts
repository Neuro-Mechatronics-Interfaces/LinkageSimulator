import { add, distance, scale, subtract, type Vec2 } from './vec2';

export type CircleIntersectionResult =
  | { kind: 'none'; reason: 'separate' | 'contained' }
  | { kind: 'tangent'; point: Vec2 }
  | { kind: 'two'; points: readonly [Vec2, Vec2] }
  | { kind: 'coincident' }
  | { kind: 'degenerate'; reason: 'negative-radius' | 'concentric' };

const EPSILON = 1e-9;

export function circleCircleIntersection(
  centerA: Vec2,
  radiusA: number,
  centerB: Vec2,
  radiusB: number,
): CircleIntersectionResult {
  if (radiusA < 0 || radiusB < 0) return { kind: 'degenerate', reason: 'negative-radius' };

  const centerDistance = distance(centerA, centerB);
  if (centerDistance <= EPSILON) {
    if (Math.abs(radiusA - radiusB) <= EPSILON) return { kind: 'coincident' };
    return { kind: 'degenerate', reason: 'concentric' };
  }
  if (centerDistance > radiusA + radiusB + EPSILON) return { kind: 'none', reason: 'separate' };
  if (centerDistance < Math.abs(radiusA - radiusB) - EPSILON) return { kind: 'none', reason: 'contained' };

  const along = (radiusA ** 2 - radiusB ** 2 + centerDistance ** 2) / (2 * centerDistance);
  const heightSquared = Math.max(0, radiusA ** 2 - along ** 2);
  const direction = scale(subtract(centerB, centerA), 1 / centerDistance);
  const midpoint = add(centerA, scale(direction, along));
  if (heightSquared <= EPSILON) return { kind: 'tangent', point: midpoint };

  const height = Math.sqrt(heightSquared);
  const offset = { x: -direction.y * height, y: direction.x * height };
  return {
    kind: 'two',
    points: [add(midpoint, offset), subtract(midpoint, offset)],
  };
}
