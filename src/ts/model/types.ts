import type { Pose2D, Vec2 } from '../geometry';

export type ComponentId = string;
export type DigitId = 'd2' | 'd3' | 'd4' | 'd5';

export interface Link {
  id: ComponentId;
  name: string;
  length: number;
  width: number;
  pose: Pose2D;
  fixed?: boolean;
}

export interface RevoluteJoint {
  id: ComponentId;
  name: string;
  linkAId: ComponentId | null;
  linkBId: ComponentId;
  localPointA?: Vec2;
  localPointB: Vec2;
  groundPoint?: Vec2;
  minAngle?: number;
  maxAngle?: number;
}

export interface GroundComponent {
  id: ComponentId;
  name: string;
  pivotPoints: Vec2[];
  /** A point on the dorsal mounting plane and its world-space orientation. */
  surfacePoint: Vec2;
  angle: number;
  /** Normal distance from the dorsal plane to the servo axis. */
  servoGroundOffset: number;
  /** Orientation of the base rail relative to the servo's zero axis. */
  baseRailAngleOffset: number;
}

export interface ServoJoint {
  id: ComponentId;
  name: string;
  groundPoint: Vec2;
  drivenLinkId: ComponentId;
  /** Revolute constraint whose driven-link attachment is centred on the servo axis. */
  revoluteJointId: ComponentId;
  /** Absolute world orientation prescribed for the driven link. */
  angle: number;
  minAngle: number;
  maxAngle: number;
  speed: number;
  direction: 1 | -1;
}

export type FingerSegmentId = 'proximal' | 'middle' | 'distal';

export interface FingerSegment {
  id: FingerSegmentId;
  name: string;
  baseLength: number;
  baseWidth: number;
}

export interface HandModel {
  id: ComponentId;
  digitId: DigitId;
  digitName: string;
  sizeScale: number;
  mcpPosition: Vec2;
  palmLength: number;
  palmWidth: number;
  segments: FingerSegment[];
  mcpAngle: number;
  pipAngle: number;
  dipAngle: number;
  rom: {
    mcp: readonly [number, number];
    pip: readonly [number, number];
    dip: readonly [number, number];
  };
  jointMechanics: Record<FingerJointId, FingerJointMechanics>;
}

export interface HandContactor {
  id: ComponentId;
  name: string;
  linkId: ComponentId;
  localPoint: Vec2;
  fingerSegment: FingerSegmentId;
  fingerPosition: number;
  padLength: number;
  padThickness: number;
  /** Total internal dorsal-to-flexor span of the ring. */
  ringWidth: number;
  linkagePoint: Vec2;
  fingerPoint: Vec2;
}

export type FingerJointId = 'mcp' | 'pip' | 'dip';

export interface FingerJointMechanics {
  /** Passive torsional stiffness in the stored joint-angle convention. */
  stiffnessNmPerRad: number;
  restAngle: number;
}

export interface FingerSegmentMassProperties {
  segmentId: FingerSegmentId;
  lengthMm: number;
  diameterMm: number;
  massKg: number;
  centerOfMass: Vec2;
}

export interface FingerJointMoment {
  jointId: FingerJointId;
  gravityMomentNm: number;
  passiveMomentNm: number;
  requiredHoldingMomentNm: number;
}

export interface FingerStatics {
  model: 'gravity-only-cylindrical-segments';
  gravityMPerS2: number;
  densityKgPerM3: number;
  segmentMasses: FingerSegmentMassProperties[];
  jointMoments: FingerJointMoment[];
}

export interface JointConstraintStatus {
  jointId: string;
  state: 'free' | 'at-minimum' | 'at-maximum';
  angle: number;
  minimum: number;
  maximum: number;
}

export interface AnalyticSolveStep {
  kind: 'dyad';
  jointId: ComponentId;
  linkIds: readonly [ComponentId, ComponentId];
  centerA: Vec2;
  radiusA: number;
  centerB: Vec2;
  radiusB: number;
  intersectionKind: 'none' | 'tangent' | 'two' | 'coincident' | 'degenerate';
  candidatePoints: Vec2[];
  selectedPoint?: Vec2;
  message?: string;
}

export interface ConstraintComponentDiagnostics {
  id: string;
  linkIds: ComponentId[];
  jointIds: ComponentId[];
  anchored: boolean;
  actuatorIds: ComponentId[];
  variableCount: number;
  residualCount: number;
  jacobianRank: number;
  passiveJacobianRank: number;
  passiveDof: number;
  drivenDof: number;
  unresolvedLinkIds: ComponentId[];
  redundantConstraintCount: number;
  overconstrained: boolean;
  inconsistent: boolean;
  singular: boolean;
  residualNorm: number;
  analyticSolveCount: number;
  numericalFallbackUsed: boolean;
  messages: string[];
}

export interface ConstraintDiagnostics {
  valid: boolean;
  components: ConstraintComponentDiagnostics[];
  disconnectedComponentIds: string[];
}

export interface FourBarSolverDefinition {
  crankLinkId: ComponentId;
  couplerLinkId: ComponentId;
  rockerLinkId: ComponentId;
  anchorDriverLinkId: ComponentId;
  middleDriverLinkId: ComponentId;
  tipDriverLinkId: ComponentId;
  /** The anchor rises dorsally from the four-bar output at this angle. */
  anchorDriverAngleOffset: number;
  /** Distance between the two four-bar coupler joints. */
  couplerJointDistance: number;
  crankGroundPoint: Vec2;
  rockerGroundPoint: Vec2;
  preferredOutputPoint: Vec2;
}

export interface SimulationState {
  time: number;
  enabled: boolean;
  valid: boolean;
  message: string;
  links: Link[];
  joints: RevoluteJoint[];
  ground: GroundComponent;
  servo: ServoJoint;
  contactors: HandContactor[];
  hand: HandModel;
  statics: FingerStatics;
  jointConstraintStatus: JointConstraintStatus[];
  /** Transient mathematical diagnostics; omitted from versioned JSON export. */
  solverDiagnostics: ConstraintDiagnostics;
  /** Transient analytic construction geometry; omitted from versioned JSON export. */
  analyticSolveSteps: AnalyticSolveStep[];
  fourBar: FourBarSolverDefinition;
  showConstruction: boolean;
}

export type Selection =
  | { kind: 'link'; id: ComponentId }
  | { kind: 'link-end'; id: ComponentId }
  | { kind: 'joint'; id: ComponentId }
  | { kind: 'servo'; id: ComponentId }
  | { kind: 'contactor'; id: ComponentId }
  | null;

export interface AppStore {
  state: SimulationState;
  digitStates: Record<DigitId, SimulationState>;
  activeDigitId: DigitId;
  overallHandScale: number;
  selection: Selection;
}
