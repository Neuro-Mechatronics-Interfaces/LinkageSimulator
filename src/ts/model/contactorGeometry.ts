import type { HandContactor, HandModel } from './types';

export function digitWidthAtContactor(hand: HandModel, contactor: HandContactor): number {
  const segment = hand.segments.find((candidate) => candidate.id === contactor.fingerSegment);
  if (!segment) throw new Error(`Unknown contactor finger segment: ${contactor.fingerSegment}`);
  return segment.baseWidth * hand.sizeScale;
}

export function ringAffordance(hand: HandModel, contactor: HandContactor): number {
  return Math.max(0, (contactor.ringWidth - digitWidthAtContactor(hand, contactor)) / 2);
}

export function enforceRingWidth(hand: HandModel, contactor: HandContactor): void {
  contactor.ringWidth = Math.max(contactor.ringWidth, digitWidthAtContactor(hand, contactor));
}
