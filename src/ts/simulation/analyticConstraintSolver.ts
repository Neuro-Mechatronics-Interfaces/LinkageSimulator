import {
  circleCircleIntersection,
  distance,
  localToWorld,
  normalizeAngle,
  type Pose2D,
  type Vec2,
} from '../geometry';
import type {
  AnalyticSolveStep,
  ComponentId,
  JointConstraintStatus,
  Link,
  RevoluteJoint,
} from '../model';
import {
  WORLD_BODY_ID,
  type ConstraintGraph,
  type RevoluteGraphConstraint,
} from './ConstraintGraph';
import { poseFromPointAndAngle, reconstructRigidPose } from './rigidTransform';
import { SOLVER_TOLERANCES } from './solverTolerances';

export type AnalyticResolution =
  | {
    kind: 'resolved';
    operation: 'servo-seed' | 'locked-angle' | 'two-point';
    linkIds: ComponentId[];
    jointId?: ComponentId;
  }
  | {
    kind: 'ambiguous';
    operation: 'dyad';
    linkIds: readonly [ComponentId, ComponentId];
    jointId: ComponentId;
    candidateCount: number;
    selectedPoint: Vec2;
  }
  | {
    kind: 'underdetermined';
    linkIds: ComponentId[];
    jointId?: ComponentId;
    reason: 'free-orientation' | 'free-floating' | 'coincident-circles';
  }
  | {
    kind: 'singular';
    linkIds: ComponentId[];
    jointId?: ComponentId;
    reason: 'tangent-dyad' | 'coincident-circles' | 'concentric-circles' | 'degenerate-attachment';
    resolved: boolean;
  }
  | {
    kind: 'inconsistent';
    linkIds: ComponentId[];
    jointId?: ComponentId;
    reason: 'unreachable-dyad' | 'attachment-closure' | 'joint-rom' | 'invalid-servo-joint';
    message: string;
  };

export interface AnalyticConstraintSolveOptions {
  previousJointPositions?: ReadonlyMap<ComponentId, Vec2>;
  closureTolerance?: number;
}

export interface AnalyticConstraintSolveResult {
  valid: boolean;
  resolvedLinkIds: Set<ComponentId>;
  unresolvedLinkIds: ComponentId[];
  intentionallyFreeLinkIds: Set<ComponentId>;
  singularJointIds: Set<ComponentId>;
  steps: AnalyticSolveStep[];
  resolutions: AnalyticResolution[];
  messages: string[];
  analyticSolveCount: number;
}

interface KnownAttachment {
  constraint: RevoluteGraphConstraint;
  localPoint: Vec2;
  worldPoint: Vec2;
  otherBodyId: ComponentId | typeof WORLD_BODY_ID;
}

const finitePoint = (point: Vec2): boolean => Number.isFinite(point.x) && Number.isFinite(point.y);

export function isFinitePose(pose: Pose2D): boolean {
  return finitePoint(pose.position) && Number.isFinite(pose.angle);
}

export function localPointForJoint(joint: RevoluteJoint, linkId: ComponentId): Vec2 | null {
  if (joint.linkBId === linkId) return joint.localPointB;
  if (joint.linkAId === linkId) return joint.localPointA ?? null;
  return null;
}

export function jointWorldPointFromLink(joint: RevoluteJoint, link: Link): Vec2 | null {
  const localPoint = localPointForJoint(joint, link.id);
  return localPoint ? localToWorld(localPoint, link.pose) : null;
}

export function captureJointWorldPositions(graph: ConstraintGraph): Map<ComponentId, Vec2> {
  const result = new Map<ComponentId, Vec2>();
  for (const constraint of graph.jointConstraints.values()) {
    const linkB = linkBody(graph, constraint.bodyBId)?.link;
    const point = linkB ? localToWorld(constraint.joint.localPointB, linkB.pose) : constraint.joint.groundPoint;
    if (point && finitePoint(point)) result.set(constraint.jointId, { ...point });
  }
  return result;
}

export function relativeJointAngle(
  graph: ConstraintGraph,
  joint: RevoluteJoint,
  poseOverrides: ReadonlyMap<ComponentId, Pose2D> = new Map(),
): number | null {
  const poseB = poseOverrides.get(joint.linkBId) ?? linkBody(graph, joint.linkBId)?.link.pose;
  if (!poseB) return null;
  const angleA = joint.linkAId === null
    ? 0
    : (poseOverrides.get(joint.linkAId) ?? linkBody(graph, joint.linkAId)?.link.pose)?.angle;
  return angleA === undefined ? null : normalizeAngle(poseB.angle - angleA);
}

export function isAngleWithinJointRom(angle: number, joint: RevoluteJoint): boolean {
  if (joint.minAngle === undefined || joint.maxAngle === undefined) return true;
  const minimum = joint.minAngle;
  const maximum = joint.maxAngle;
  if (maximum - minimum >= Math.PI * 2 - SOLVER_TOLERANCES.jointLimit) return true;
  const normalized = normalizeAngle(angle);
  if (minimum <= maximum) {
    return normalized >= minimum - SOLVER_TOLERANCES.jointLimit &&
      normalized <= maximum + SOLVER_TOLERANCES.jointLimit;
  }
  return normalized >= minimum - SOLVER_TOLERANCES.jointLimit ||
    normalized <= maximum + SOLVER_TOLERANCES.jointLimit;
}

/**
 * Analytically seeds and traverses the graph until no fixed-angle, two-point,
 * or dyad operation can resolve another body. Link poses are updated in place.
 */
export function solveAnalyticConstraints(
  graph: ConstraintGraph,
  options: AnalyticConstraintSolveOptions = {},
): AnalyticConstraintSolveResult {
  const closureTolerance = options.closureTolerance ?? SOLVER_TOLERANCES.closure;
  const resolvedLinkIds = new Set<ComponentId>();
  const intentionallyFreeLinkIds = new Set<ComponentId>();
  const singularJointIds = new Set<ComponentId>();
  const steps: AnalyticSolveStep[] = [];
  const resolutions: AnalyticResolution[] = [];
  const messages: string[] = [];
  let hardFailure = false;

  for (const body of graph.bodies.values()) {
    if (body.kind === 'link' && body.fixed) resolvedLinkIds.add(body.id);
  }

  for (const constraint of graph.constraints) {
    if (constraint.kind !== 'actuator') continue;
    const servo = constraint.servo;
    const revolute = graph.jointConstraints.get(servo.revoluteJointId);
    const driven = linkBody(graph, servo.drivenLinkId)?.link;
    const localPoint = revolute ? localPointForJoint(revolute.joint, servo.drivenLinkId) : null;
    if (!revolute || !driven || !localPoint ||
        (revolute.bodyAId !== servo.drivenLinkId && revolute.bodyBId !== servo.drivenLinkId)) {
      hardFailure = true;
      const message = `Servo ${servo.id} does not reference a valid driven-link revolute`;
      messages.push(message);
      resolutions.push({
        kind: 'inconsistent',
        linkIds: [servo.drivenLinkId],
        jointId: servo.revoluteJointId,
        reason: 'invalid-servo-joint',
        message,
      });
      continue;
    }
    const pose = poseFromPointAndAngle(localPoint, servo.groundPoint, servo.angle);
    if (!isFinitePose(pose)) {
      hardFailure = true;
      const message = `Servo ${servo.id} produced a non-finite driven pose`;
      messages.push(message);
      resolutions.push({
        kind: 'inconsistent',
        linkIds: [driven.id],
        jointId: revolute.jointId,
        reason: 'invalid-servo-joint',
        message,
      });
      continue;
    }
    driven.pose = pose;
    resolvedLinkIds.add(driven.id);
    resolutions.push({
      kind: 'resolved',
      operation: 'servo-seed',
      linkIds: [driven.id],
      jointId: revolute.jointId,
    });
  }

  for (let pass = 0; pass < graph.bodies.size; pass += 1) {
    let progress = false;

    for (const constraint of graph.constraints) {
      if (constraint.kind !== 'locked-angle') continue;
      const joint = constraint.joint;
      const unresolvedId = resolvedLinkIds.has(constraint.bodyBId)
        ? constraint.bodyAId !== WORLD_BODY_ID && !resolvedLinkIds.has(constraint.bodyAId)
          ? constraint.bodyAId
          : null
        : resolvedLinkIds.has(constraint.bodyAId) || constraint.bodyAId === WORLD_BODY_ID
          ? constraint.bodyBId
          : null;
      if (unresolvedId === null || unresolvedId === WORLD_BODY_ID) continue;
      const attachment = knownAttachment(graph, graph.jointConstraints.get(joint.id), unresolvedId, resolvedLinkIds);
      const link = linkBody(graph, unresolvedId)?.link;
      if (!attachment || !link) continue;
      const otherAngle = attachment.otherBodyId === WORLD_BODY_ID
        ? 0
        : linkBody(graph, attachment.otherBodyId)?.link.pose.angle;
      if (otherAngle === undefined) continue;
      const angle = joint.linkBId === unresolvedId
        ? otherAngle + constraint.targetAngle
        : otherAngle - constraint.targetAngle;
      const pose = poseFromPointAndAngle(attachment.localPoint, attachment.worldPoint, angle);
      const overrides = new Map<ComponentId, Pose2D>([[link.id, pose]]);
      const validation = validateCandidate(graph, overrides, resolvedLinkIds, closureTolerance);
      if (!validation.valid) {
        hardFailure = true;
        messages.push(validation.message);
        resolutions.push({
          kind: 'inconsistent',
          linkIds: [link.id],
          jointId: joint.id,
          reason: validation.rom ? 'joint-rom' : 'attachment-closure',
          message: validation.message,
        });
        continue;
      }
      link.pose = pose;
      resolvedLinkIds.add(link.id);
      resolutions.push({ kind: 'resolved', operation: 'locked-angle', linkIds: [link.id], jointId: joint.id });
      progress = true;
    }

    for (const body of graph.bodies.values()) {
      if (body.kind !== 'link' || resolvedLinkIds.has(body.id)) continue;
      const attachments = knownAttachments(graph, body.id, resolvedLinkIds);
      if (attachments.length < 2) continue;
      let resolved = false;
      let sawInconsistent = false;
      for (let first = 0; first < attachments.length - 1 && !resolved; first += 1) {
        for (let second = first + 1; second < attachments.length && !resolved; second += 1) {
          const a = attachments[first];
          const b = attachments[second];
          if (!a || !b || distance(a.localPoint, b.localPoint) <= SOLVER_TOLERANCES.length) continue;
          const reconstruction = reconstructRigidPose(
            a.localPoint,
            b.localPoint,
            a.worldPoint,
            b.worldPoint,
            closureTolerance,
          );
          if (reconstruction.kind !== 'resolved') {
            sawInconsistent ||= reconstruction.kind === 'inconsistent';
            continue;
          }
          const overrides = new Map<ComponentId, Pose2D>([[body.id, reconstruction.pose]]);
          const validation = validateCandidate(graph, overrides, resolvedLinkIds, closureTolerance);
          if (!validation.valid) {
            sawInconsistent = true;
            continue;
          }
          body.link.pose = reconstruction.pose;
          resolvedLinkIds.add(body.id);
          resolutions.push({ kind: 'resolved', operation: 'two-point', linkIds: [body.id] });
          resolved = true;
          progress = true;
        }
      }
      if (!resolved && sawInconsistent) {
        hardFailure = true;
        const message = `Resolved attachments on link ${body.id} do not close`;
        messages.push(message);
        resolutions.push({
          kind: 'inconsistent',
          linkIds: [body.id],
          reason: 'attachment-closure',
          message,
        });
      }
    }

    const dyadResult = resolveOneDyad(
      graph,
      resolvedLinkIds,
      options.previousJointPositions ?? new Map(),
      closureTolerance,
    );
    if (dyadResult) {
      steps.push(dyadResult.step);
      resolutions.push(...dyadResult.resolutions);
      for (const message of dyadResult.messages) messages.push(message);
      for (const jointId of dyadResult.singularJointIds) singularJointIds.add(jointId);
      if (dyadResult.hardFailure) hardFailure = true;
      if (dyadResult.resolved) progress = true;
    }

    if (!progress) break;
  }

  const unresolvedLinkIds = [...graph.bodies.values()]
    .filter((body) => body.kind === 'link' && !body.fixed && !resolvedLinkIds.has(body.id))
    .map((body) => body.id)
    .sort((left, right) => left.localeCompare(right));
  const analyticSolveCount = resolutions.filter((resolution) =>
    resolution.kind === 'resolved' || resolution.kind === 'ambiguous' ||
    (resolution.kind === 'singular' && resolution.resolved),
  ).length;
  return {
    valid: !hardFailure,
    resolvedLinkIds,
    unresolvedLinkIds,
    intentionallyFreeLinkIds,
    singularJointIds,
    steps,
    resolutions,
    messages,
    analyticSolveCount,
  };
}

/**
 * Preserve prior orientations for a genuinely mobile remainder while placing
 * each body on any newly known hinge. This satisfies positional closure but
 * does not claim that the free angular coordinate was solved.
 */
export function placeUnderconstrainedLinks(
  graph: ConstraintGraph,
  result: AnalyticConstraintSolveResult,
  allowedLinkIds?: ReadonlySet<ComponentId>,
): void {
  for (let pass = 0; pass < graph.bodies.size; pass += 1) {
    let progress = false;
    for (const linkId of [...result.unresolvedLinkIds]) {
      if (allowedLinkIds && !allowedLinkIds.has(linkId)) continue;
      const link = linkBody(graph, linkId)?.link;
      if (!link) continue;
      const attachment = knownAttachments(graph, linkId, result.resolvedLinkIds)[0];
      if (!attachment) continue;
      let angle = link.pose.angle;
      const joint = attachment.constraint.joint;
      if (joint.minAngle !== undefined && joint.maxAngle !== undefined &&
          joint.id !== actuatorJointId(graph)) {
        const otherAngle = attachment.otherBodyId === WORLD_BODY_ID
          ? 0
          : linkBody(graph, attachment.otherBodyId)?.link.pose.angle ?? 0;
        const relative = joint.linkBId === link.id
          ? normalizeAngle(angle - otherAngle)
          : normalizeAngle(otherAngle - angle);
        const projected = projectAngleToRom(relative, joint.minAngle, joint.maxAngle);
        angle = joint.linkBId === link.id ? otherAngle + projected : otherAngle - projected;
      }
      const pose = poseFromPointAndAngle(attachment.localPoint, attachment.worldPoint, angle);
      if (!isFinitePose(pose)) continue;
      link.pose = pose;
      result.resolvedLinkIds.add(link.id);
      result.intentionallyFreeLinkIds.add(link.id);
      result.resolutions.push({
        kind: 'underdetermined',
        linkIds: [link.id],
        jointId: attachment.constraint.jointId,
        reason: 'free-orientation',
      });
      progress = true;
    }
    if (!progress) break;
    result.unresolvedLinkIds = result.unresolvedLinkIds.filter((id) => !result.resolvedLinkIds.has(id));
  }

  for (const linkId of result.unresolvedLinkIds) {
    result.intentionallyFreeLinkIds.add(linkId);
    result.resolutions.push({ kind: 'underdetermined', linkIds: [linkId], reason: 'free-floating' });
  }
}

export function mechanismJointStatuses(graph: ConstraintGraph): JointConstraintStatus[] {
  const statuses: JointConstraintStatus[] = [];
  for (const constraint of graph.jointConstraints.values()) {
    const joint = constraint.joint;
    if (joint.minAngle === undefined || joint.maxAngle === undefined || joint.id === actuatorJointId(graph)) continue;
    const angle = relativeJointAngle(graph, joint);
    if (angle === null) continue;
    const state = Math.abs(normalizeAngle(angle - joint.minAngle)) <= SOLVER_TOLERANCES.jointLimit
      ? 'at-minimum'
      : Math.abs(normalizeAngle(angle - joint.maxAngle)) <= SOLVER_TOLERANCES.jointLimit
        ? 'at-maximum'
        : 'free';
    statuses.push({
      jointId: joint.id,
      state,
      angle,
      minimum: joint.minAngle,
      maximum: joint.maxAngle,
    });
  }
  return statuses;
}

function resolveOneDyad(
  graph: ConstraintGraph,
  resolvedLinkIds: Set<ComponentId>,
  previousJointPositions: ReadonlyMap<ComponentId, Vec2>,
  closureTolerance: number,
): {
  resolved: boolean;
  hardFailure: boolean;
  step: AnalyticSolveStep;
  resolutions: AnalyticResolution[];
  singularJointIds: ComponentId[];
  messages: string[];
} | null {
  for (const shared of graph.jointConstraints.values()) {
    if (shared.bodyAId === WORLD_BODY_ID || resolvedLinkIds.has(shared.bodyAId) ||
        resolvedLinkIds.has(shared.bodyBId)) continue;
    const linkA = linkBody(graph, shared.bodyAId)?.link;
    const linkB = linkBody(graph, shared.bodyBId)?.link;
    const sharedLocalA = shared.joint.localPointA;
    const sharedLocalB = shared.joint.localPointB;
    if (!linkA || !linkB || !sharedLocalA) continue;
    const outerA = knownAttachments(graph, linkA.id, resolvedLinkIds)
      .find((attachment) => attachment.constraint.jointId !== shared.jointId);
    const outerB = knownAttachments(graph, linkB.id, resolvedLinkIds)
      .find((attachment) => attachment.constraint.jointId !== shared.jointId);
    if (!outerA || !outerB) continue;
    const radiusA = distance(outerA.localPoint, sharedLocalA);
    const radiusB = distance(outerB.localPoint, sharedLocalB);
    const intersection = circleCircleIntersection(outerA.worldPoint, radiusA, outerB.worldPoint, radiusB);
    const candidates = intersection.kind === 'two'
      ? [...intersection.points]
      : intersection.kind === 'tangent'
        ? [intersection.point]
        : [];
    const step: AnalyticSolveStep = {
      kind: 'dyad',
      jointId: shared.jointId,
      linkIds: [linkA.id, linkB.id],
      centerA: { ...outerA.worldPoint },
      radiusA,
      centerB: { ...outerB.worldPoint },
      radiusB,
      intersectionKind: intersection.kind,
      candidatePoints: candidates.map((point) => ({ ...point })),
    };

    if (radiusA <= SOLVER_TOLERANCES.length || radiusB <= SOLVER_TOLERANCES.length) {
      const message = `Dyad ${shared.jointId} has a degenerate local attachment span`;
      step.message = message;
      return {
        resolved: false,
        hardFailure: false,
        step,
        resolutions: [{
          kind: 'singular',
          linkIds: [linkA.id, linkB.id],
          jointId: shared.jointId,
          reason: 'degenerate-attachment',
          resolved: false,
        }],
        singularJointIds: [shared.jointId],
        messages: [message],
      };
    }

    if (intersection.kind === 'none') {
      const message = `Dyad ${shared.jointId} is unreachable (${intersection.reason})`;
      step.message = message;
      return {
        resolved: false,
        hardFailure: true,
        step,
        resolutions: [{
          kind: 'inconsistent',
          linkIds: [linkA.id, linkB.id],
          jointId: shared.jointId,
          reason: 'unreachable-dyad',
          message,
        }],
        singularJointIds: [],
        messages: [message],
      };
    }
    if (intersection.kind === 'coincident' || intersection.kind === 'degenerate') {
      const coincident = intersection.kind === 'coincident';
      const message = coincident
        ? `Dyad ${shared.jointId} has coincident circles and retains a free coordinate`
        : `Dyad ${shared.jointId} has concentric circles`;
      step.message = message;
      return {
        resolved: false,
        hardFailure: false,
        step,
        resolutions: [
          {
            kind: 'singular',
            linkIds: [linkA.id, linkB.id],
            jointId: shared.jointId,
            reason: coincident ? 'coincident-circles' : 'concentric-circles',
            resolved: false,
          },
          ...(coincident ? [{
            kind: 'underdetermined' as const,
            linkIds: [linkA.id, linkB.id],
            jointId: shared.jointId,
            reason: 'coincident-circles' as const,
          }] : []),
        ],
        singularJointIds: [shared.jointId],
        messages: [message],
      };
    }

    const validCandidates: Array<{ point: Vec2; poses: Map<ComponentId, Pose2D> }> = [];
    const candidateFailures = new Set<string>();
    for (const candidate of candidates) {
      const reconstructedA = reconstructRigidPose(
        outerA.localPoint,
        sharedLocalA,
        outerA.worldPoint,
        candidate,
        closureTolerance,
      );
      const reconstructedB = reconstructRigidPose(
        outerB.localPoint,
        sharedLocalB,
        outerB.worldPoint,
        candidate,
        closureTolerance,
      );
      if (reconstructedA.kind !== 'resolved' || reconstructedB.kind !== 'resolved') continue;
      const poses = new Map<ComponentId, Pose2D>([
        [linkA.id, reconstructedA.pose],
        [linkB.id, reconstructedB.pose],
      ]);
      const validation = validateCandidate(graph, poses, resolvedLinkIds, closureTolerance);
      if (validation.valid) {
        validCandidates.push({ point: candidate, poses });
      } else {
        candidateFailures.add(validation.message);
      }
    }
    if (validCandidates.length === 0) {
      const detail = [...candidateFailures].join('; ');
      const message = `Dyad ${shared.jointId} has no valid assembly branch${detail ? ` (${detail})` : ''}`;
      step.message = message;
      return {
        resolved: false,
        hardFailure: true,
        step,
        resolutions: [{
          kind: 'inconsistent',
          linkIds: [linkA.id, linkB.id],
          jointId: shared.jointId,
          reason: 'joint-rom',
          message,
        }],
        singularJointIds: [],
        messages: [message],
      };
    }

    const currentPoint = jointWorldPointFromLink(shared.joint, linkB);
    const preference = previousJointPositions.get(shared.jointId) ?? currentPoint ?? validCandidates[0]!.point;
    validCandidates.sort((left, right) => distance(left.point, preference) - distance(right.point, preference));
    const selected = validCandidates[0]!;
    linkA.pose = selected.poses.get(linkA.id)!;
    linkB.pose = selected.poses.get(linkB.id)!;
    resolvedLinkIds.add(linkA.id);
    resolvedLinkIds.add(linkB.id);
    step.selectedPoint = { ...selected.point };
    const tangent = intersection.kind === 'tangent';
    return {
      resolved: true,
      hardFailure: false,
      step,
      resolutions: tangent
        ? [{
          kind: 'singular',
          linkIds: [linkA.id, linkB.id],
          jointId: shared.jointId,
          reason: 'tangent-dyad',
          resolved: true,
        }]
        : [{
          kind: 'ambiguous',
          operation: 'dyad',
          linkIds: [linkA.id, linkB.id],
          jointId: shared.jointId,
          candidateCount: validCandidates.length,
          selectedPoint: { ...selected.point },
        }],
      singularJointIds: tangent ? [shared.jointId] : [],
      messages: tangent ? [`Dyad ${shared.jointId} is tangent (branch-merging singularity)`] : [],
    };
  }
  return null;
}

function knownAttachments(
  graph: ConstraintGraph,
  unresolvedLinkId: ComponentId,
  resolvedLinkIds: ReadonlySet<ComponentId>,
): KnownAttachment[] {
  return (graph.constraintsByBodyId.get(unresolvedLinkId) ?? [])
    .filter((constraint): constraint is RevoluteGraphConstraint => constraint.kind === 'revolute')
    .map((constraint) => knownAttachment(graph, constraint, unresolvedLinkId, resolvedLinkIds))
    .filter((attachment): attachment is KnownAttachment => attachment !== null);
}

function knownAttachment(
  graph: ConstraintGraph,
  constraint: RevoluteGraphConstraint | undefined,
  unresolvedLinkId: ComponentId,
  resolvedLinkIds: ReadonlySet<ComponentId>,
): KnownAttachment | null {
  if (!constraint) return null;
  const joint = constraint.joint;
  if (constraint.bodyBId === unresolvedLinkId) {
    if (constraint.bodyAId === WORLD_BODY_ID) {
      return joint.groundPoint ? {
        constraint,
        localPoint: joint.localPointB,
        worldPoint: joint.groundPoint,
        otherBodyId: WORLD_BODY_ID,
      } : null;
    }
    if (!resolvedLinkIds.has(constraint.bodyAId) || !joint.localPointA) return null;
    const other = linkBody(graph, constraint.bodyAId)?.link;
    return other ? {
      constraint,
      localPoint: joint.localPointB,
      worldPoint: localToWorld(joint.localPointA, other.pose),
      otherBodyId: constraint.bodyAId,
    } : null;
  }
  if (constraint.bodyAId !== unresolvedLinkId || !joint.localPointA ||
      !resolvedLinkIds.has(constraint.bodyBId)) return null;
  const other = linkBody(graph, constraint.bodyBId)?.link;
  return other ? {
    constraint,
    localPoint: joint.localPointA,
    worldPoint: localToWorld(joint.localPointB, other.pose),
    otherBodyId: constraint.bodyBId,
  } : null;
}

function validateCandidate(
  graph: ConstraintGraph,
  poseOverrides: ReadonlyMap<ComponentId, Pose2D>,
  resolvedLinkIds: ReadonlySet<ComponentId>,
  closureTolerance: number,
): { valid: true } | { valid: false; rom: boolean; message: string } {
  for (const [linkId, pose] of poseOverrides) {
    if (!isFinitePose(pose)) return { valid: false, rom: false, message: `Link ${linkId} has a non-finite pose` };
    for (const graphConstraint of graph.constraintsByBodyId.get(linkId) ?? []) {
      if (graphConstraint.kind !== 'revolute') continue;
      const joint = graphConstraint.joint;
      const otherId = graphConstraint.bodyAId === linkId ? graphConstraint.bodyBId : graphConstraint.bodyAId;
      if (otherId !== WORLD_BODY_ID && !resolvedLinkIds.has(otherId) && !poseOverrides.has(otherId)) continue;
      const pointB = pointForLink(graph, joint.linkBId, joint.localPointB, poseOverrides);
      const pointA = joint.linkAId === null
        ? joint.groundPoint
        : joint.localPointA
          ? pointForLink(graph, joint.linkAId, joint.localPointA, poseOverrides)
          : undefined;
      if (!pointA || !pointB || distance(pointA, pointB) > closureTolerance) {
        return { valid: false, rom: false, message: `Joint ${joint.id} does not close` };
      }
      if (joint.id !== actuatorJointId(graph)) {
        const angle = relativeJointAngle(graph, joint, poseOverrides);
        if (angle !== null && !isAngleWithinJointRom(angle, joint)) {
          return { valid: false, rom: true, message: `Joint ${joint.id} violates its angular range` };
        }
      }
    }
  }
  return { valid: true };
}

function pointForLink(
  graph: ConstraintGraph,
  linkId: ComponentId,
  localPoint: Vec2,
  overrides: ReadonlyMap<ComponentId, Pose2D>,
): Vec2 | undefined {
  const pose = overrides.get(linkId) ?? linkBody(graph, linkId)?.link.pose;
  return pose ? localToWorld(localPoint, pose) : undefined;
}

function projectAngleToRom(angle: number, minimum: number, maximum: number): number {
  if (maximum - minimum >= Math.PI * 2 - SOLVER_TOLERANCES.jointLimit) return angle;
  if (minimum <= maximum) return Math.max(minimum, Math.min(maximum, angle));
  if (angle >= minimum || angle <= maximum) return angle;
  return Math.abs(normalizeAngle(angle - minimum)) <= Math.abs(normalizeAngle(angle - maximum))
    ? minimum
    : maximum;
}

function actuatorJointId(graph: ConstraintGraph): ComponentId | undefined {
  return graph.constraints.find((constraint) => constraint.kind === 'actuator')?.servo.revoluteJointId;
}

function linkBody(graph: ConstraintGraph, id: ComponentId): Extract<ReturnType<typeof graph.bodies.get>, { kind: 'link' }> | undefined {
  const body = graph.bodies.get(id);
  return body?.kind === 'link' ? body : undefined;
}
