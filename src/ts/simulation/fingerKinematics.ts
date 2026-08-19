import { add, distance, normalize, perpendicular, rotate, scale, subtract, type Vec2 } from '../geometry';
import type { FingerSegmentId, HandModel } from '../model';

export interface FingerLandmarks {
  mcp: Vec2;
  pip: Vec2;
  dip: Vec2;
  tip: Vec2;
}

export interface FingerSegmentFrame {
  start: Vec2;
  end: Vec2;
  tangent: Vec2;
  dorsalNormal: Vec2;
  width: number;
}

const clamp = (value: number, min: number, max: number): number => Math.max(min, Math.min(max, value));
const radians = (degrees: number): number => (degrees * Math.PI) / 180;

export function segmentLength(hand: HandModel, segmentId: FingerSegmentId): number {
  const segment = hand.segments.find((candidate) => candidate.id === segmentId);
  if (!segment) throw new Error(`Unknown finger segment: ${segmentId}`);
  return segment.baseLength * hand.sizeScale;
}

export function fingerLandmarks(hand: HandModel): FingerLandmarks {
  const proximalLength = segmentLength(hand, 'proximal');
  const middleLength = segmentLength(hand, 'middle');
  const distalLength = segmentLength(hand, 'distal');
  const pip = add(hand.mcpPosition, rotate({ x: proximalLength, y: 0 }, hand.mcpAngle));
  const middleAngle = hand.mcpAngle - hand.pipAngle;
  const dip = add(pip, rotate({ x: middleLength, y: 0 }, middleAngle));
  const distalAngle = middleAngle - hand.dipAngle;
  const tip = add(dip, rotate({ x: distalLength, y: 0 }, distalAngle));
  return { mcp: hand.mcpPosition, pip, dip, tip };
}

export function pointOnFinger(hand: HandModel, segmentId: FingerSegmentId, position: number): Vec2 {
  const landmarks = fingerLandmarks(hand);
  const t = clamp(position, 0, 1);
  if (segmentId === 'proximal') return add(landmarks.mcp, scale(subtract(landmarks.pip, landmarks.mcp), t));
  if (segmentId === 'middle') return add(landmarks.pip, scale(subtract(landmarks.dip, landmarks.pip), t));
  return add(landmarks.dip, scale(subtract(landmarks.tip, landmarks.dip), t));
}

export function fingerSegmentFrame(hand: HandModel, segmentId: FingerSegmentId): FingerSegmentFrame {
  const landmarks = fingerLandmarks(hand);
  const segment = hand.segments.find((candidate) => candidate.id === segmentId);
  if (!segment) throw new Error(`Unknown finger segment: ${segmentId}`);
  const [start, end] = segmentId === 'proximal'
    ? [landmarks.mcp, landmarks.pip]
    : segmentId === 'middle'
      ? [landmarks.pip, landmarks.dip]
      : [landmarks.dip, landmarks.tip];
  const tangent = normalize(subtract(end, start)) ?? { x: 1, y: 0 };
  let dorsalNormal = perpendicular(tangent);
  // Positive world Y is the upper/dorsal side of the viewport.
  if (dorsalNormal.y < 0) dorsalNormal = scale(dorsalNormal, -1);
  return { start, end, tangent, dorsalNormal, width: segment.baseWidth * hand.sizeScale };
}

/**
 * Solve the MCP/PIP angles so a point on the middle phalanx follows a target.
 * Targets outside the two-link annulus are projected to the nearest reachable
 * radius; this keeps the renderer finite at geometry and hand-size extremes.
 */
export function solveFingerContact(hand: HandModel, target: Vec2, middlePosition: number): boolean {
  return solveFingerPointContact(hand, target, 'middle', middlePosition);
}

/** Solve a point contact using coupled PIP/DIP flexion for distal targets. */
export function solveFingerPointContact(
  hand: HandModel,
  target: Vec2,
  segmentId: FingerSegmentId,
  segmentPosition: number,
): boolean {
  const proximalLength = segmentLength(hand, 'proximal');
  const targetVector = subtract(target, hand.mcpPosition);
  const rawDistance = distance(target, hand.mcpPosition);
  const targetAngle = Math.atan2(targetVector.y, targetVector.x);

  if (segmentId === 'proximal') {
    const reach = proximalLength * clamp(segmentPosition, 0.05, 1);
    const unclampedMcp = targetAngle;
    hand.mcpAngle = clamp(unclampedMcp, radians(hand.rom.mcp[0]), radians(hand.rom.mcp[1]));
    hand.pipAngle = 0;
    hand.dipAngle = 0;
    return Math.abs(rawDistance - reach) <= 1e-4 && hand.mcpAngle === unclampedMcp;
  }

  if (segmentId === 'distal') {
    return solveDistalPoint(hand, targetAngle, rawDistance, segmentPosition);
  }

  const partialMiddleLength = segmentLength(hand, 'middle') * clamp(segmentPosition, 0.05, 1);
  const minimumReach = Math.abs(proximalLength - partialMiddleLength) + 1e-6;
  const maximumReach = proximalLength + partialMiddleLength - 1e-6;
  const reachableDistance = clamp(rawDistance, minimumReach, maximumReach);
  const cosinePip = clamp(
    (reachableDistance ** 2 - proximalLength ** 2 - partialMiddleLength ** 2) /
      (2 * proximalLength * partialMiddleLength),
    -1,
    1,
  );
  const pipAngle = Math.acos(cosinePip);
  const mcpAngle = targetAngle + Math.atan2(
    partialMiddleLength * Math.sin(pipAngle),
    proximalLength + partialMiddleLength * Math.cos(pipAngle),
  );

  hand.mcpAngle = clamp(mcpAngle, radians(hand.rom.mcp[0]), radians(hand.rom.mcp[1]));
  hand.pipAngle = clamp(pipAngle, radians(hand.rom.pip[0]), radians(hand.rom.pip[1]));
  hand.dipAngle = clamp(pipAngle * 0.62, radians(hand.rom.dip[0]), radians(hand.rom.dip[1]));
  return rawDistance >= minimumReach && rawDistance <= maximumReach;
}

function solveDistalPoint(hand: HandModel, targetAngle: number, rawDistance: number, distalPosition: number): boolean {
  const proximalLength = segmentLength(hand, 'proximal');
  const middleLength = segmentLength(hand, 'middle');
  const distalLength = segmentLength(hand, 'distal') * clamp(distalPosition, 0.05, 1);
  const pipMaximum = radians(hand.rom.pip[1]);
  const dipCoupling = 0.62;
  const localVector = (pipAngle: number): Vec2 => ({
    x: proximalLength + middleLength * Math.cos(pipAngle) + distalLength * Math.cos(pipAngle * (1 + dipCoupling)),
    y: -middleLength * Math.sin(pipAngle) - distalLength * Math.sin(pipAngle * (1 + dipCoupling)),
  });
  const extendedReach = magnitudeOf(localVector(0));
  const flexedReach = magnitudeOf(localVector(pipMaximum));
  const targetReach = clamp(rawDistance, flexedReach, extendedReach);

  let low = 0;
  let high = pipMaximum;
  for (let iteration = 0; iteration < 48; iteration += 1) {
    const middle = (low + high) / 2;
    if (magnitudeOf(localVector(middle)) > targetReach) low = middle;
    else high = middle;
  }
  const pipAngle = (low + high) / 2;
  const vector = localVector(pipAngle);
  const unclampedMcp = targetAngle - Math.atan2(vector.y, vector.x);
  hand.mcpAngle = clamp(unclampedMcp, radians(hand.rom.mcp[0]), radians(hand.rom.mcp[1]));
  hand.pipAngle = pipAngle;
  hand.dipAngle = clamp(pipAngle * dipCoupling, radians(hand.rom.dip[0]), radians(hand.rom.dip[1]));
  return rawDistance >= flexedReach && rawDistance <= extendedReach && hand.mcpAngle === unclampedMcp;
}

const magnitudeOf = (value: Vec2): number => Math.hypot(value.x, value.y);
