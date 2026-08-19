import { describe, expect, it } from 'vitest';
import {
  add,
  circleCircleIntersection,
  distance,
  dot,
  localToWorld,
  magnitude,
  normalize,
  normalizeAngle,
  rotate,
  scale,
  segmentSegmentDistance,
  subtract,
} from './index';

describe('Vec2 operations', () => {
  it('adds, subtracts, scales, and takes dot products', () => {
    expect(add({ x: 2, y: -1 }, { x: 3, y: 4 })).toEqual({ x: 5, y: 3 });
    expect(subtract({ x: 2, y: -1 }, { x: 3, y: 4 })).toEqual({ x: -1, y: -5 });
    expect(scale({ x: 2, y: -1 }, 3)).toEqual({ x: 6, y: -3 });
    expect(dot({ x: 2, y: -1 }, { x: 3, y: 4 })).toBe(2);
  });

  it('measures crossing and separated line segments', () => {
    expect(segmentSegmentDistance({ x: 0, y: 0 }, { x: 4, y: 4 }, { x: 0, y: 4 }, { x: 4, y: 0 })).toBe(0);
    expect(segmentSegmentDistance({ x: 0, y: 0 }, { x: 4, y: 0 }, { x: 0, y: 3 }, { x: 4, y: 3 })).toBeCloseTo(3);
  });

  it('computes lengths, distances, and safe normalization', () => {
    expect(magnitude({ x: 3, y: 4 })).toBe(5);
    expect(distance({ x: 1, y: 1 }, { x: 4, y: 5 })).toBe(5);
    const normalized = normalize({ x: 3, y: 4 });
    expect(normalized?.x).toBeCloseTo(0.6);
    expect(normalized?.y).toBeCloseTo(0.8);
    expect(normalize({ x: 0, y: 0 })).toBeNull();
  });

  it('rotates and transforms local points', () => {
    const rotated = rotate({ x: 2, y: 0 }, Math.PI / 2);
    expect(rotated.x).toBeCloseTo(0);
    expect(rotated.y).toBeCloseTo(2);
    const world = localToWorld({ x: 2, y: 0 }, { position: { x: 5, y: 6 }, angle: Math.PI / 2 });
    expect(world.x).toBeCloseTo(5);
    expect(world.y).toBeCloseTo(8);
    expect(normalizeAngle(3 * Math.PI)).toBeCloseTo(Math.PI);
  });
});

describe('circle-circle intersection', () => {
  it('returns two symmetric intersections', () => {
    const result = circleCircleIntersection({ x: 0, y: 0 }, 5, { x: 6, y: 0 }, 5);
    expect(result.kind).toBe('two');
    if (result.kind !== 'two') return;
    expect(result.points[0].x).toBeCloseTo(3);
    expect(Math.abs(result.points[0].y)).toBeCloseTo(4);
    expect(result.points[1].x).toBeCloseTo(3);
    expect(result.points[1].y).toBeCloseTo(-result.points[0].y);
  });

  it('distinguishes tangent, separate, and contained circles', () => {
    expect(circleCircleIntersection({ x: 0, y: 0 }, 2, { x: 4, y: 0 }, 2).kind).toBe('tangent');
    expect(circleCircleIntersection({ x: 0, y: 0 }, 2, { x: 5, y: 0 }, 2)).toEqual({ kind: 'none', reason: 'separate' });
    expect(circleCircleIntersection({ x: 0, y: 0 }, 5, { x: 1, y: 0 }, 1)).toEqual({ kind: 'none', reason: 'contained' });
  });

  it('reports coincident and degenerate inputs explicitly', () => {
    expect(circleCircleIntersection({ x: 0, y: 0 }, 2, { x: 0, y: 0 }, 2)).toEqual({ kind: 'coincident' });
    expect(circleCircleIntersection({ x: 0, y: 0 }, 2, { x: 0, y: 0 }, 1)).toEqual({ kind: 'degenerate', reason: 'concentric' });
    expect(circleCircleIntersection({ x: 0, y: 0 }, -1, { x: 3, y: 0 }, 1)).toEqual({ kind: 'degenerate', reason: 'negative-radius' });
  });
});
