import { vec } from '../geometry';
import { DIGIT_IDS, DIGIT_PROFILES, HAND_DEFAULTS } from './anthropometry';
import type { DigitId, SimulationState } from './types';

const radians = (degrees: number): number => (degrees * Math.PI) / 180;

export function createDefaultState(digitId: DigitId = 'd2', overallHandScale = 1): SimulationState {
  const profile = DIGIT_PROFILES[digitId];
  const [proximalScale, middleScale, distalScale] = profile.lengthScale;
  const d2TotalLength = HAND_DEFAULTS.proximalLength + HAND_DEFAULTS.middleLength + HAND_DEFAULTS.distalLength;
  const digitTotalLength = HAND_DEFAULTS.proximalLength * proximalScale +
    HAND_DEFAULTS.middleLength * middleScale + HAND_DEFAULTS.distalLength * distalScale;
  const mechanismScale = digitTotalLength / d2TotalLength;
  const scaled = (value: number): number => value * mechanismScale;
  const mcpPosition = vec(-48, 55);
  const groundSurfacePoint = vec(
    mcpPosition.x + scaled(-67),
    mcpPosition.y + scaled(15),
  );
  const servoGroundOffset = scaled(48.923004526179284);
  // Both pivots sit dorsal (positive world Y / upper viewport) and proximal
  // to the D2 metacarpal. A distal driver leaves the four-bar's floating
  // coupler/rocker joint and presents its free right end to the fingertip.
  const crankGroundPoint = vec(groundSurfacePoint.x, groundSurfacePoint.y + servoGroundOffset);
  const groundRailLength = scaled(56.5059012863785);
  const groundRailAngle = radians(-5.2);
  const rockerGroundPoint = vec(
    crankGroundPoint.x + groundRailLength * Math.cos(groundRailAngle),
    crankGroundPoint.y + groundRailLength * Math.sin(groundRailAngle),
  );
  const crankLength = scaled(53.794374946272);
  const couplerLength = scaled(88.55710076633841);
  const rockerLength = scaled(61.26780117978342);
  const anchorLength = scaled(20.966492022853345);
  const middleDriverLength = scaled(42.61799878953025);
  const tipDriverLength = scaled(42.65155897010118);
  const middleContactorLocalX = middleDriverLength * (0.8703099994454533 - 0.5);
  const tipContactorLocalX = tipDriverLength * (0.784450733626727 - 0.5);

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
        width: scaled(8),
        pose: { position: vec((crankGroundPoint.x + rockerGroundPoint.x) / 2, (crankGroundPoint.y + rockerGroundPoint.y) / 2), angle: groundRailAngle },
        fixed: true,
      },
      {
        id: 'crank',
        name: 'Servo crank',
        length: crankLength,
        width: scaled(7),
        pose: { position: vec(crankGroundPoint.x + crankLength / 2, crankGroundPoint.y), angle: radians(38) },
      },
      {
        id: 'coupler',
        name: 'Four-bar coupler',
        length: couplerLength,
        width: scaled(6),
        pose: { position: vec(-60, 102), angle: radians(-13) },
      },
      {
        id: 'rocker',
        name: 'Proximal support rocker',
        length: rockerLength,
        width: scaled(7),
        pose: { position: vec(-45, 105), angle: radians(-63) },
      },
      {
        id: 'anchor-driver',
        name: 'Dorsal contactor anchor',
        length: anchorLength,
        width: scaled(7),
        pose: { position: vec(-17.5, 110.7), angle: radians(-18.2) },
      },
      {
        id: 'middle-driver',
        name: 'Middle-phalanx driver',
        length: middleDriverLength,
        width: scaled(6),
        pose: { position: vec(3.5, 94), angle: radians(-51.5) },
      },
      {
        id: 'tip-driver',
        name: 'Distal-phalanx driver',
        length: tipDriverLength,
        width: scaled(6),
        pose: { position: vec(31.3, 75.5), angle: radians(-16.4) },
      },
    ],
    joints: [
      {
        id: 'servo-crank-joint',
        name: 'Servo / crank revolute joint',
        linkAId: 'ground-rail',
        linkBId: 'crank',
        localPointA: vec(-groundRailLength / 2, 0),
        localPointB: vec(-crankLength / 2, 0),
        minAngle: radians(38),
        maxAngle: radians(180),
      },
      {
        id: 'crank-coupler-joint',
        name: 'Crank / coupler joint',
        linkAId: 'crank',
        linkBId: 'coupler',
        localPointA: vec(crankLength / 2, 0),
        localPointB: vec(-couplerLength / 2, 0),
        minAngle: radians(-170),
        maxAngle: radians(170),
      },
      {
        id: 'coupler-rocker-joint',
        name: 'Coupler / rocker joint',
        linkAId: 'coupler',
        linkBId: 'rocker',
        localPointA: vec(couplerLength / 2, 0),
        localPointB: vec(rockerLength / 2, 0),
        minAngle: radians(-170),
        maxAngle: radians(170),
      },
      {
        id: 'anchor-driver-joint',
        name: 'Coupler / dorsal anchor joint',
        linkAId: 'coupler',
        linkBId: 'anchor-driver',
        localPointA: vec(couplerLength / 2, 0),
        localPointB: vec(-anchorLength / 2, 0),
        minAngle: radians(0.6107389274984598),
        maxAngle: radians(0.6107389274984598),
      },
      {
        id: 'middle-driver-joint',
        name: 'Anchor / middle driver joint',
        linkAId: 'anchor-driver',
        linkBId: 'middle-driver',
        localPointA: vec(anchorLength / 2, 0),
        localPointB: vec(-middleDriverLength / 2, 0),
        minAngle: radians(-180),
        maxAngle: radians(180),
      },
      {
        id: 'tip-driver-joint',
        name: 'Middle / distal driver joint',
        linkAId: 'middle-driver',
        linkBId: 'tip-driver',
        localPointA: vec(middleDriverLength / 2, 0),
        localPointB: vec(-tipDriverLength / 2, 0),
        minAngle: radians(-180),
        maxAngle: radians(180),
      },
      {
        id: 'rocker-ground-joint',
        name: 'Dorsal rail / rocker pivot',
        linkAId: 'ground-rail',
        linkBId: 'rocker',
        localPointA: vec(groundRailLength / 2, 0),
        localPointB: vec(-rockerLength / 2, 0),
        minAngle: radians(-120),
        maxAngle: radians(30),
      },
    ],
    ground: {
      id: 'ground',
      name: 'Dorsal hand ground plane',
      pivotPoints: [crankGroundPoint, rockerGroundPoint],
      surfacePoint: groundSurfacePoint,
      angle: 0,
      servoGroundOffset,
      baseRailAngleOffset: groundRailAngle,
    },
    servo: {
      id: 'servo',
      name: 'Primary servo',
      groundPoint: crankGroundPoint,
      drivenLinkId: 'crank',
      angle: radians(38),
      minAngle: radians(38),
      maxAngle: radians(180),
      speed: radians(30),
      direction: 1,
    },
    contactors: [
      {
        id: 'middle-band',
        name: 'Dorsal middle-phalanx contact',
        linkId: 'middle-driver',
        localPoint: vec(middleContactorLocalX, 0),
        fingerSegment: 'middle',
        fingerPosition: 0.3782606571214273,
        padLength: scaled(13),
        padThickness: scaled(3.5),
        ringWidth: 34 * profile.widthScale,
        linkagePoint: vec(14.1, 80.6),
        fingerPoint: vec(10, 67),
      },
      {
        id: 'index-band',
        name: 'Dorsal distal-phalanx contact',
        linkId: 'tip-driver',
        localPoint: vec(tipContactorLocalX, 0),
        fingerSegment: 'distal',
        fingerPosition: 0.9591809064731933,
        padLength: scaled(13),
        padThickness: scaled(3.5),
        ringWidth: 30 * profile.widthScale,
        linkagePoint: vec(48.6, 70.4),
        fingerPoint: vec(44, 60),
      },
    ],
    hand: {
      id: `left-hand-${digitId}`,
      digitId,
      digitName: profile.name,
      sizeScale: overallHandScale,
      mcpPosition,
      palmLength: HAND_DEFAULTS.palmLength,
      palmWidth: HAND_DEFAULTS.palmWidth,
      segments: [
        { id: 'proximal', name: 'Proximal phalanx', baseLength: HAND_DEFAULTS.proximalLength * proximalScale, baseWidth: HAND_DEFAULTS.proximalWidth * profile.widthScale },
        { id: 'middle', name: 'Middle phalanx', baseLength: HAND_DEFAULTS.middleLength * middleScale, baseWidth: HAND_DEFAULTS.middleWidth * profile.widthScale },
        { id: 'distal', name: 'Distal phalanx', baseLength: HAND_DEFAULTS.distalLength * distalScale, baseWidth: HAND_DEFAULTS.distalWidth * profile.widthScale },
      ],
      mcpAngle: radians(9.8),
      pipAngle: radians(16.8),
      dipAngle: radians(10.42),
      rom: {
        mcp: HAND_DEFAULTS.mcpRom,
        pip: HAND_DEFAULTS.pipRom,
        dip: HAND_DEFAULTS.dipRom,
      },
      jointMechanics: {
        mcp: { stiffnessNmPerRad: 0, restAngle: 0 },
        pip: { stiffnessNmPerRad: 0, restAngle: 0 },
        dip: { stiffnessNmPerRad: 0, restAngle: 0 },
      },
    },
    statics: {
      model: 'gravity-only-cylindrical-segments',
      gravityMPerS2: HAND_DEFAULTS.gravityMPerS2,
      densityKgPerM3: HAND_DEFAULTS.tissueDensityKgPerM3,
      segmentMasses: [],
      jointMoments: [],
    },
    jointConstraintStatus: [],
    fourBar: {
      crankLinkId: 'crank',
      couplerLinkId: 'coupler',
      rockerLinkId: 'rocker',
      anchorDriverLinkId: 'anchor-driver',
      middleDriverLinkId: 'middle-driver',
      tipDriverLinkId: 'tip-driver',
      anchorDriverAngleOffset: radians(0.6107389274984598),
      couplerJointDistance: couplerLength,
      crankGroundPoint,
      rockerGroundPoint,
      preferredOutputPoint: vec(mcpPosition.x + scaled(20), mcpPosition.y + scaled(59)),
    },
    showConstruction: false,
  };
}

export function createDefaultDigitStates(overallHandScale = 1): Record<DigitId, SimulationState> {
  return Object.fromEntries(
    DIGIT_IDS.map((digitId) => [digitId, createDefaultState(digitId, overallHandScale)]),
  ) as Record<DigitId, SimulationState>;
}

export const nextComponentId = (() => {
  let sequence = 0;
  return (prefix: string): string => `${prefix}-${Date.now().toString(36)}-${sequence++}`;
})();
