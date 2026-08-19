import { vec } from '../geometry';
import { HAND_DEFAULTS } from './anthropometry';
import type { SimulationState } from './types';

const radians = (degrees: number): number => (degrees * Math.PI) / 180;

export function createDefaultState(): SimulationState {
  // Both pivots sit dorsal (positive world Y / upper viewport) and proximal
  // to the D2 metacarpal. A distal driver leaves the four-bar's floating
  // coupler/rocker joint and presents its free right end to the fingertip.
  const crankGroundPoint = vec(-115, 105);
  const rockerGroundPoint = vec(-60, 100);
  const groundRailLength = Math.hypot(
    rockerGroundPoint.x - crankGroundPoint.x,
    rockerGroundPoint.y - crankGroundPoint.y,
  );
  const groundRailAngle = Math.atan2(
    rockerGroundPoint.y - crankGroundPoint.y,
    rockerGroundPoint.x - crankGroundPoint.x,
  );

  return {
    time: 0,
    enabled: false,
    valid: true,
    message: 'Ready',
    links: [
      {
        id: 'ground-rail',
        name: 'Dorsal base rail',
        length: groundRailLength,
        width: 8,
        pose: { position: vec(-87.5, 102.5), angle: groundRailAngle },
        fixed: true,
      },
      {
        id: 'crank',
        name: 'Servo crank',
        length: 28,
        width: 7,
        pose: { position: vec(-102, 99), angle: radians(-25) },
      },
      {
        id: 'coupler',
        name: 'Four-bar coupler',
        length: 60,
        width: 6,
        pose: { position: vec(-60, 86), angle: radians(-13) },
      },
      {
        id: 'rocker',
        name: 'Proximal support rocker',
        length: 35,
        width: 7,
        pose: { position: vec(-45, 90), angle: radians(-63) },
      },
      {
        id: 'anchor-driver',
        name: 'Dorsal contactor anchor',
        length: 22,
        width: 7,
        pose: { position: vec(-17.5, 110.7), angle: radians(-18.2) },
      },
      {
        id: 'middle-driver',
        name: 'Middle-phalanx driver',
        length: 34,
        width: 6,
        pose: { position: vec(3.5, 94), angle: radians(-51.5) },
      },
      {
        id: 'tip-driver',
        name: 'Distal-phalanx driver',
        length: 36,
        width: 6,
        pose: { position: vec(31.3, 75.5), angle: radians(-16.4) },
      },
    ],
    joints: [
      {
        id: 'crank-coupler-joint',
        name: 'Crank / coupler joint',
        linkAId: 'crank',
        linkBId: 'coupler',
        localPointA: vec(14, 0),
        localPointB: vec(-30, 0),
        minAngle: radians(-170),
        maxAngle: radians(170),
      },
      {
        id: 'coupler-rocker-joint',
        name: 'Coupler / rocker joint',
        linkAId: 'coupler',
        linkBId: 'rocker',
        localPointA: vec(30, 0),
        localPointB: vec(17.5, 0),
        minAngle: radians(-170),
        maxAngle: radians(170),
      },
      {
        id: 'anchor-driver-joint',
        name: 'Coupler / dorsal anchor joint',
        linkAId: 'coupler',
        linkBId: 'anchor-driver',
        localPointA: vec(30, 0),
        localPointB: vec(-11, 0),
        minAngle: radians(105),
        maxAngle: radians(105),
      },
      {
        id: 'middle-driver-joint',
        name: 'Anchor / middle driver joint',
        linkAId: 'anchor-driver',
        linkBId: 'middle-driver',
        localPointA: vec(11, 0),
        localPointB: vec(-17, 0),
        minAngle: radians(-180),
        maxAngle: radians(180),
      },
      {
        id: 'tip-driver-joint',
        name: 'Middle / distal driver joint',
        linkAId: 'middle-driver',
        linkBId: 'tip-driver',
        localPointA: vec(17, 0),
        localPointB: vec(-18, 0),
        minAngle: radians(-180),
        maxAngle: radians(180),
      },
      {
        id: 'rocker-ground-joint',
        name: 'Rocker ground pivot',
        linkAId: 'ground-rail',
        linkBId: 'rocker',
        localPointA: vec(groundRailLength / 2, 0),
        localPointB: vec(-17.5, 0),
        minAngle: radians(-120),
        maxAngle: radians(30),
      },
    ],
    ground: {
      id: 'ground',
      name: 'Dorsal hand ground plane',
      pivotPoints: [crankGroundPoint, rockerGroundPoint],
      surfacePoint: vec(-115, 70),
      angle: 0,
      servoGroundOffset: 35,
      baseRailAngleOffset: groundRailAngle,
    },
    servo: {
      id: 'servo',
      name: 'Primary servo',
      groundPoint: crankGroundPoint,
      drivenLinkId: 'crank',
      angle: radians(15),
      minAngle: radians(15),
      maxAngle: radians(35),
      speed: radians(18),
      direction: 1,
    },
    contactors: [
      {
        id: 'middle-band',
        name: 'Dorsal middle-phalanx contact',
        linkId: 'middle-driver',
        localPoint: vec(17, 0),
        fingerSegment: 'middle',
        fingerPosition: 0.55,
        padLength: 13,
        padThickness: 3.5,
        bandClearance: 9.5,
        linkagePoint: vec(14.1, 80.6),
        fingerPoint: vec(10, 67),
      },
      {
        id: 'index-band',
        name: 'Dorsal distal-phalanx contact',
        linkId: 'tip-driver',
        localPoint: vec(18, 0),
        fingerSegment: 'distal',
        fingerPosition: 1,
        padLength: 13,
        padThickness: 3.5,
        bandClearance: 9,
        linkagePoint: vec(48.6, 70.4),
        fingerPoint: vec(44, 60),
      },
    ],
    hand: {
      id: 'left-hand',
      sizeScale: 1,
      mcpPosition: vec(-48, 55),
      palmLength: HAND_DEFAULTS.palmLength,
      palmWidth: HAND_DEFAULTS.palmWidth,
      segments: [
        { id: 'proximal', name: 'Proximal phalanx', baseLength: HAND_DEFAULTS.proximalLength, baseWidth: HAND_DEFAULTS.proximalWidth },
        { id: 'middle', name: 'Middle phalanx', baseLength: HAND_DEFAULTS.middleLength, baseWidth: HAND_DEFAULTS.middleWidth },
        { id: 'distal', name: 'Distal phalanx', baseLength: HAND_DEFAULTS.distalLength, baseWidth: HAND_DEFAULTS.distalWidth },
      ],
      mcpAngle: radians(9.8),
      pipAngle: radians(16.8),
      dipAngle: radians(10.42),
      rom: {
        mcp: HAND_DEFAULTS.mcpRom,
        pip: HAND_DEFAULTS.pipRom,
        dip: HAND_DEFAULTS.dipRom,
      },
    },
    fourBar: {
      crankLinkId: 'crank',
      couplerLinkId: 'coupler',
      rockerLinkId: 'rocker',
      anchorDriverLinkId: 'anchor-driver',
      middleDriverLinkId: 'middle-driver',
      tipDriverLinkId: 'tip-driver',
      anchorDriverAngleOffset: radians(-20),
      couplerJointDistance: 60,
      crankGroundPoint,
      rockerGroundPoint,
      preferredOutputPoint: vec(-28, 114),
    },
    showConstruction: false,
  };
}

export const nextComponentId = (() => {
  let sequence = 0;
  return (prefix: string): string => `${prefix}-${Date.now().toString(36)}-${sequence++}`;
})();
