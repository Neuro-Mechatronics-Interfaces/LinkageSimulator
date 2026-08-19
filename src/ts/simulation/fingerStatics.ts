import { add, scale, subtract, type Vec2 } from '../geometry';
import {
  HAND_DEFAULTS,
  type FingerJointId,
  type FingerJointMoment,
  type FingerSegmentId,
  type FingerSegmentMassProperties,
  type FingerStatics,
  type HandModel,
} from '../model';
import { fingerLandmarks, segmentLength } from './fingerKinematics';

const MILLIMETERS_PER_METER = 1000;

/**
 * Estimate gravity loads for a static planar posture. Each phalanx is a
 * uniform cylinder, so its mass scales with the rendered length and width.
 * Moments use the stored joint-angle sign convention and exclude muscle,
 * tendon, contact-force, and linkage-weight contributions.
 */
export function calculateFingerStatics(hand: HandModel): FingerStatics {
  const landmarks = fingerLandmarks(hand);
  const endpoints: Record<FingerSegmentId, readonly [Vec2, Vec2]> = {
    proximal: [landmarks.mcp, landmarks.pip],
    middle: [landmarks.pip, landmarks.dip],
    distal: [landmarks.dip, landmarks.tip],
  };
  const segmentMasses = hand.segments.map((segment): FingerSegmentMassProperties => {
    const lengthMm = segmentLength(hand, segment.id);
    const diameterMm = segment.baseWidth * hand.sizeScale;
    const radiusM = diameterMm / 2 / MILLIMETERS_PER_METER;
    const lengthM = lengthMm / MILLIMETERS_PER_METER;
    const [start, end] = endpoints[segment.id];
    return {
      segmentId: segment.id,
      lengthMm,
      diameterMm,
      massKg: Math.PI * radiusM ** 2 * lengthM * HAND_DEFAULTS.tissueDensityKgPerM3,
      centerOfMass: scale(add(start, end), 0.5),
    };
  });

  const segmentById = new Map(segmentMasses.map((segment) => [segment.segmentId, segment]));
  const jointDefinitions: Array<{
    jointId: FingerJointId;
    point: Vec2;
    distalSegments: FingerSegmentId[];
    generalizedSign: 1 | -1;
    angle: number;
  }> = [
    {
      jointId: 'mcp',
      point: landmarks.mcp,
      distalSegments: ['proximal', 'middle', 'distal'],
      generalizedSign: 1,
      angle: hand.mcpAngle,
    },
    {
      jointId: 'pip',
      point: landmarks.pip,
      distalSegments: ['middle', 'distal'],
      generalizedSign: -1,
      angle: hand.pipAngle,
    },
    {
      jointId: 'dip',
      point: landmarks.dip,
      distalSegments: ['distal'],
      generalizedSign: -1,
      angle: hand.dipAngle,
    },
  ];

  const jointMoments = jointDefinitions.map((joint): FingerJointMoment => {
    const worldGravityMomentNm = joint.distalSegments.reduce((sum, segmentId) => {
      const segment = segmentById.get(segmentId);
      if (!segment) return sum;
      const armM = scale(subtract(segment.centerOfMass, joint.point), 1 / MILLIMETERS_PER_METER);
      return sum + armM.x * (-segment.massKg * HAND_DEFAULTS.gravityMPerS2);
    }, 0);
    const gravityMomentNm = joint.generalizedSign * worldGravityMomentNm;
    const mechanics = hand.jointMechanics[joint.jointId];
    const passiveMomentNm = mechanics.stiffnessNmPerRad === 0
      ? 0
      : -mechanics.stiffnessNmPerRad * (joint.angle - mechanics.restAngle);
    return {
      jointId: joint.jointId,
      gravityMomentNm,
      passiveMomentNm,
      requiredHoldingMomentNm: -(gravityMomentNm + passiveMomentNm),
    };
  });

  return {
    model: 'gravity-only-cylindrical-segments',
    gravityMPerS2: HAND_DEFAULTS.gravityMPerS2,
    densityKgPerM3: HAND_DEFAULTS.tissueDensityKgPerM3,
    segmentMasses,
    jointMoments,
  };
}
