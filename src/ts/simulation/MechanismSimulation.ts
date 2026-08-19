import {
  add,
  distance,
  dot,
  localToWorld,
  normalizeAngle,
  rotate,
  scale,
  segmentSegmentDistance,
  subtract,
  type Vec2,
} from '../geometry';
import {
  enforceRingWidth,
  ringAffordance,
  type FingerStatics,
  type HandContactor,
  type JointConstraintStatus,
  type Link,
  type RevoluteJoint,
  type SimulationState,
} from '../model';
import { mechanismJointStatuses } from './analyticConstraintSolver';
import { calculateFingerStatics } from './fingerStatics';
import { fingerLandmarks, fingerSegmentFrame, pointOnFinger, solveFingerPointContact } from './fingerKinematics';
import { solveGeneralMechanism } from './GeneralMechanismSolver';
import { validateMechanismInvariants } from './mechanismInvariants';
import { poseFromPointAndAngle } from './rigidTransform';

interface SolverSnapshot {
  servoAngle: number;
  servoDrivenLinkId: string;
  servoRevoluteJointId: string;
  servoGroundPoint: Vec2;
  groundPivots: Vec2[];
  groundSurfacePoint: Vec2;
  servoGroundOffset: number;
  baseRailAngleOffset: number;
  links: Map<string, { position: Vec2; angle: number; length: number; width: number }>;
  joints: Map<string, {
    linkAId: string | null;
    linkBId: string;
    localPointA?: Vec2;
    localPointB: Vec2;
    groundPoint?: Vec2;
    minAngle?: number;
    maxAngle?: number;
  }>;
  jointPositions: Map<string, Vec2>;
  handAngles: readonly [number, number, number];
  contactors: Map<string, { localPoint: Vec2; linkagePoint: Vec2; fingerPoint: Vec2 }>;
  statics: FingerStatics;
  jointConstraintStatus: JointConstraintStatus[];
}

interface ContactorChainResult {
  reachable: boolean;
  activeJointIds: string[];
  linkIds: string[];
}

interface ContactorChainTopology {
  anchorLink: Link;
  middleDriver: Link;
  tipDriver: Link;
  middleContactor: HandContactor;
  tipContactor: HandContactor;
  middleJoint: RevoluteJoint;
  tipJoint: RevoluteJoint;
  middleLocalPoint: Vec2;
  tipLocalPoint: Vec2;
  tipJointLocalOnMiddle: Vec2;
  anchorWorldPoint: Vec2;
}

export class MechanismSimulation {
  private readonly lastValidSnapshots = new WeakMap<SimulationState, SolverSnapshot>();

  step(state: SimulationState, dt: number): void {
    state.time += dt;
    const servo = state.servo;
    servo.angle += servo.speed * servo.direction * dt;
    if (servo.angle >= servo.maxAngle) {
      servo.angle = servo.maxAngle;
      servo.direction = -1;
    } else if (servo.angle <= servo.minAngle) {
      servo.angle = servo.minAngle;
      servo.direction = 1;
    }
    this.solve(state);
    if (!state.valid && state.message.startsWith('Collision')) {
      state.servo.direction = state.servo.direction === 1 ? -1 : 1;
    }
  }

  solve(state: SimulationState): void {
    const rollbackSnapshot = this.lastValidSnapshots.get(state) ?? this.createSnapshot(state);
    for (const contactor of state.contactors) enforceRingWidth(state.hand, contactor);
    this.synchronizeEndpointAttachments(state);
    const mountError = this.synchronizeMount(state);
    if (mountError) {
      this.fail(state, mountError, rollbackSnapshot);
      return;
    }
    const mechanism = solveGeneralMechanism(
      state,
      this.lastValidSnapshots.get(state)?.jointPositions,
    );
    state.solverDiagnostics = mechanism.diagnostics;
    state.analyticSolveSteps = mechanism.analyticSteps;
    if (!mechanism.valid || !mechanism.graph) {
      this.fail(state, `Unsolvable mechanism · ${mechanism.message}`, rollbackSnapshot);
      return;
    }
    const contactChain = this.solveDorsalContactorChain(state);
    const chainLinkIds = new Set(contactChain.linkIds);
    let handContactReachable = contactChain.reachable;
    for (const contactor of state.contactors.filter((candidate) => !chainLinkIds.has(candidate.linkId))) {
      handContactReachable = this.solveDorsalContactor(state, contactor) && handContactReachable;
    }
    const invariants = validateMechanismInvariants(mechanism.graph);
    if (!invariants.valid) {
      this.fail(
        state,
        `Constraint residual · ${invariants.messages[0] ?? 'mechanism closure invariant failed'}`,
        rollbackSnapshot,
      );
      return;
    }

    const collision = this.detectFingerCollision(state);
    if (collision) {
      this.fail(state, collision, rollbackSnapshot);
      return;
    }
    const mountCollision = this.detectMountCollision(state);
    if (mountCollision) {
      this.fail(state, mountCollision, rollbackSnapshot);
      return;
    }

    state.statics = calculateFingerStatics(state.hand);

    state.valid = true;
    if (handContactReachable) {
      state.message = contactChain.activeJointIds.length > 0
        ? `${mechanism.message} · ${contactChain.activeJointIds.length} joint limit(s) active`
        : mechanism.message;
    } else {
      if (contactChain.activeJointIds.length > 0) {
        state.message = `Contact active · ${contactChain.activeJointIds.join(', ')} limited; available joints continued`;
      } else {
        state.message = 'Partial solve · contact target projected; available joints continued';
      }
    }
    state.jointConstraintStatus = mergeJointStatuses(
      mechanismJointStatuses(mechanism.graph),
      state.jointConstraintStatus,
    );
    this.captureSnapshot(state);
  }

  /** Inverse-pose a constrained link endpoint by choosing the nearest servo angle. */
  solveForLinkEndpoint(state: SimulationState, linkId: string, target: Vec2): void {
    state.enabled = false;
    const controlledLink = state.links.find((candidate) => candidate.id === linkId);
    if (!controlledLink || controlledLink.fixed) return;
    if (linkId === state.servo.drivenLinkId) {
      const actuatorJoint = state.joints.find((joint) => joint.id === state.servo.revoluteJointId);
      const localPivot = actuatorJoint
        ? jointLocalPoint(actuatorJoint, state.servo.drivenLinkId)
        : null;
      const endpointFromPivot = localPivot
        ? subtract({ x: controlledLink.length / 2, y: 0 }, localPivot)
        : { x: controlledLink.length, y: 0 };
      const vector = subtract(target, state.servo.groundPoint);
      const localDirection = Math.atan2(endpointFromPivot.y, endpointFromPivot.x);
      state.servo.angle = Math.max(
        state.servo.minAngle,
        Math.min(state.servo.maxAngle, Math.atan2(vector.y, vector.x) - localDirection),
      );
      this.solve(state);
      if (state.valid) state.message = 'Servo endpoint control';
      return;
    }

    const component = state.solverDiagnostics.components.find((candidate) => candidate.linkIds.includes(linkId));
    const isConstrained = component !== undefined && component.actuatorIds.length > 0 &&
      (!component.unresolvedLinkIds.includes(linkId) ||
        state.contactors.some((contactor) => contactor.linkId === linkId));
    if (!isConstrained) {
      this.rotateFreeLinkToTarget(state, linkId, target);
      this.solve(state);
      return;
    }

    const originalAngle = state.servo.angle;
    let bestAngle: number | null = null;
    let bestDistance = Number.POSITIVE_INFINITY;
    const samples = 96;
    for (let index = 0; index <= samples; index += 1) {
      state.servo.angle = state.servo.minAngle +
        ((state.servo.maxAngle - state.servo.minAngle) * index) / samples;
      this.solve(state);
      if (!state.valid) continue;
      const link = state.links.find((candidate) => candidate.id === linkId);
      if (!link) continue;
      const endpoint = localToWorld({ x: link.length / 2, y: 0 }, link.pose);
      const candidateDistance = distance(endpoint, target);
      if (candidateDistance < bestDistance) {
        bestDistance = candidateDistance;
        bestAngle = state.servo.angle;
      }
    }
    state.servo.angle = bestAngle ?? originalAngle;
    this.solve(state);
    if (bestAngle !== null) state.message = 'Link endpoint · nearest valid pose';
  }

  private rotateFreeLinkToTarget(state: SimulationState, linkId: string, target: Vec2): void {
    const link = state.links.find((candidate) => candidate.id === linkId);
    if (!link || link.fixed) return;
    const leftEnd = localToWorld({ x: -link.length / 2, y: 0 }, link.pose);
    link.pose.angle = Math.atan2(target.y - leftEnd.y, target.x - leftEnd.x);
    link.pose.position = add(leftEnd, rotate({ x: link.length / 2, y: 0 }, link.pose.angle));
  }

  /** Preserve endpoint-based editing without overwriting arbitrary joint coordinates. */
  private synchronizeEndpointAttachments(state: SimulationState): void {
    const snapshot = this.lastValidSnapshots.get(state);
    if (!snapshot) return;
    for (const link of state.links) {
      const previous = snapshot.links.get(link.id);
      if (!previous || Math.abs(previous.length - link.length) <= 1e-9) continue;
      const update = (point: Vec2 | undefined): Vec2 | undefined => {
        if (!point || Math.abs(point.y) > 1e-8) return point;
        const half = previous.length / 2;
        if (Math.abs(point.x - half) <= 1e-8) return { x: link.length / 2, y: 0 };
        if (Math.abs(point.x + half) <= 1e-8) return { x: -link.length / 2, y: 0 };
        return point;
      };
      for (const joint of state.joints) {
        if (joint.linkAId === link.id) joint.localPointA = update(joint.localPointA);
        if (joint.linkBId === link.id) joint.localPointB = update(joint.localPointB) ?? joint.localPointB;
      }
    }
  }

  private synchronizeMount(state: SimulationState): string | null {
    const actuatorJoint = state.joints.find((joint) => joint.id === state.servo.revoluteJointId);
    if (!actuatorJoint ||
        (actuatorJoint.linkAId !== state.servo.drivenLinkId && actuatorJoint.linkBId !== state.servo.drivenLinkId)) {
      return `Servo ${state.servo.id} has no valid revolute attachment`;
    }
    const fixedLinkId = actuatorJoint.linkBId === state.servo.drivenLinkId
      ? actuatorJoint.linkAId
      : actuatorJoint.linkBId;
    const rail = fixedLinkId
      ? state.links.find((link) => link.id === fixedLinkId && link.fixed)
      : undefined;
    const normal = rotate({ x: 0, y: 1 }, state.ground.angle);
    const servoPoint = add(state.ground.surfacePoint, scale(normal, state.ground.servoGroundOffset));
    state.servo.groundPoint = servoPoint;
    if (rail) {
      const localPoint = actuatorJoint.linkAId === rail.id
        ? actuatorJoint.localPointA
        : actuatorJoint.localPointB;
      if (!localPoint) return `Servo ${state.servo.id} mount joint is missing a local attachment`;
      rail.pose = poseFromPointAndAngle(
        localPoint,
        servoPoint,
        state.ground.angle + state.ground.baseRailAngleOffset,
      );
    } else if (actuatorJoint.linkAId !== null) {
      return `Servo ${state.servo.id} revolute is not attached to fixed geometry`;
    } else {
      actuatorJoint.groundPoint = { ...servoPoint };
    }

    const fixedIds = new Set(state.links.filter((link) => link.fixed).map((link) => link.id));
    const pivots: Vec2[] = [{ ...servoPoint }];
    for (const joint of state.joints) {
      if (joint.linkAId === null && joint.groundPoint) {
        pivots.push({ ...joint.groundPoint });
      } else if (joint.linkAId && fixedIds.has(joint.linkAId) && joint.localPointA) {
        const fixed = state.links.find((link) => link.id === joint.linkAId);
        if (fixed) pivots.push(localToWorld(joint.localPointA, fixed.pose));
      } else if (fixedIds.has(joint.linkBId)) {
        const fixed = state.links.find((link) => link.id === joint.linkBId);
        if (fixed) pivots.push(localToWorld(joint.localPointB, fixed.pose));
      }
    }
    state.ground.pivotPoints = uniquePoints(pivots);
    return null;
  }

  /**
   * Solve the two default contactors together. The two free revolute angles
   * and the coupled MCP/PIP pose provide four variables for four planar point
   * constraints, including contactors repositioned along either driver.
   */
  private solveDorsalContactorChain(state: SimulationState): ContactorChainResult {
    const topology = this.findContactorChain(state);
    if (!topology) return { reachable: true, activeJointIds: [], linkIds: [] };
    let variables = this.projectChainVariables(
      state,
      topology,
      [
        state.hand.mcpAngle,
        state.hand.pipAngle,
        topology.middleDriver.pose.angle,
        topology.tipDriver.pose.angle,
      ],
    );
    let residual = this.evaluateContactorChain(state, topology, variables);
    const epsilon = 1e-4;
    for (let iteration = 0; iteration < 48 && vectorNorm(residual) > 1e-5; iteration += 1) {
      const jacobian = Array.from({ length: 4 }, () => Array<number>(4).fill(0));
      for (let column = 0; column < 4; column += 1) {
        const plusVariables = [...variables];
        plusVariables[column] = plusVariables[column]! + epsilon;
        const projectedPlus = this.projectChainVariables(state, topology, plusVariables);
        const plusSample = this.evaluateContactorChain(state, topology, projectedPlus);
        const minusVariables = [...variables];
        minusVariables[column] = minusVariables[column]! - epsilon;
        const projectedMinus = this.projectChainVariables(state, topology, minusVariables);
        const minusSample = this.evaluateContactorChain(state, topology, projectedMinus);
        const denominator = projectedPlus[column]! - projectedMinus[column]!;
        if (Math.abs(denominator) <= 1e-10) continue;
        for (let row = 0; row < 4; row += 1) {
          jacobian[row]![column] = (plusSample[row]! - minusSample[row]!) / denominator;
        }
      }
      const delta = leastSquaresStep(jacobian, residual, 1e-6);
      if (!delta) break;
      const largest = Math.max(...delta.map((value) => Math.abs(value)));
      const damping = largest > 0.35 ? 0.35 / largest : 1;
      let accepted = false;
      for (let lineSearch = 0; lineSearch < 8; lineSearch += 1) {
        const stepScale = damping * 0.5 ** lineSearch;
        const candidate = this.projectChainVariables(
          state,
          topology,
          variables.map((value, index) => value + delta[index]! * stepScale),
        );
        const candidateResidual = this.evaluateContactorChain(state, topology, candidate);
        if (vectorNorm(candidateResidual) < vectorNorm(residual) - 1e-10) {
          variables = candidate;
          residual = candidateResidual;
          accepted = true;
          break;
        }
      }
      if (!accepted) break;
    }
    this.evaluateContactorChain(state, topology, variables);
    state.jointConstraintStatus = this.chainJointStatuses(state, topology);
    return {
      reachable: vectorNorm(residual) <= 0.02,
      activeJointIds: state.jointConstraintStatus
        .filter((status) => status.state !== 'free')
        .map((status) => status.jointId),
      linkIds: [topology.middleDriver.id, topology.tipDriver.id],
    };
  }

  private projectChainVariables(
    state: SimulationState,
    topology: ContactorChainTopology,
    variables: number[],
  ): number[] {
    const projected = [...variables];
    projected[0] = clamp(projected[0]!, radians(state.hand.rom.mcp[0]), radians(state.hand.rom.mcp[1]));
    projected[1] = clamp(projected[1]!, radians(state.hand.rom.pip[0]), radians(state.hand.rom.pip[1]));
    projected[2] = projectJointLinkAngle(
      projected[2]!,
      topology.middleJoint,
      topology.middleDriver.id,
      topology.anchorLink.pose.angle,
    );
    projected[3] = projectJointLinkAngle(
      projected[3]!,
      topology.tipJoint,
      topology.tipDriver.id,
      projected[2]!,
    );
    return projected;
  }

  private chainJointStatuses(state: SimulationState, topology: ContactorChainTopology): JointConstraintStatus[] {
    return [
      constraintStatus('hand-mcp', state.hand.mcpAngle, radians(state.hand.rom.mcp[0]), radians(state.hand.rom.mcp[1])),
      constraintStatus('hand-pip', state.hand.pipAngle, radians(state.hand.rom.pip[0]), radians(state.hand.rom.pip[1])),
      constraintStatus('hand-dip', state.hand.dipAngle, radians(state.hand.rom.dip[0]), radians(state.hand.rom.dip[1])),
      constraintStatus(
        topology.middleJoint.id,
        relativeAngleAtJoint(topology.middleJoint, topology.anchorLink, topology.middleDriver),
        topology.middleJoint.minAngle ?? -Math.PI,
        topology.middleJoint.maxAngle ?? Math.PI,
      ),
      constraintStatus(
        topology.tipJoint.id,
        relativeAngleAtJoint(topology.tipJoint, topology.middleDriver, topology.tipDriver),
        topology.tipJoint.minAngle ?? -Math.PI,
        topology.tipJoint.maxAngle ?? Math.PI,
      ),
    ];
  }

  private evaluateContactorChain(
    state: SimulationState,
    topology: ContactorChainTopology,
    variables: number[],
  ): number[] {
    state.hand.mcpAngle = variables[0]!;
    state.hand.pipAngle = variables[1]!;
    state.hand.dipAngle = clamp(
      variables[1]! * 0.62,
      radians(state.hand.rom.dip[0]),
      radians(state.hand.rom.dip[1]),
    );
    topology.middleDriver.pose = poseFromPointAndAngle(
      topology.middleLocalPoint,
      topology.anchorWorldPoint,
      variables[2]!,
    );
    const tipAnchor = localToWorld(topology.tipJointLocalOnMiddle, topology.middleDriver.pose);
    topology.tipDriver.pose = poseFromPointAndAngle(topology.tipLocalPoint, tipAnchor, variables[3]!);
    topology.middleContactor.linkagePoint = localToWorld(
      topology.middleContactor.localPoint,
      topology.middleDriver.pose,
    );
    topology.tipContactor.linkagePoint = localToWorld(topology.tipContactor.localPoint, topology.tipDriver.pose);
    topology.middleContactor.fingerPoint = pointOnFinger(
      state.hand,
      topology.middleContactor.fingerSegment,
      topology.middleContactor.fingerPosition,
    );
    topology.tipContactor.fingerPoint = pointOnFinger(
      state.hand,
      topology.tipContactor.fingerSegment,
      topology.tipContactor.fingerPosition,
    );
    const middleTarget = this.dorsalLinkageTarget(state, topology.middleContactor, topology.middleDriver);
    const tipTarget = this.dorsalLinkageTarget(state, topology.tipContactor, topology.tipDriver);
    return [
      topology.middleContactor.linkagePoint.x - middleTarget.x,
      topology.middleContactor.linkagePoint.y - middleTarget.y,
      topology.tipContactor.linkagePoint.x - tipTarget.x,
      topology.tipContactor.linkagePoint.y - tipTarget.y,
    ];
  }

  private findContactorChain(state: SimulationState): ContactorChainTopology | null {
    const middleContactor = state.contactors.find((contactor) => contactor.fingerSegment === 'middle');
    const tipContactor = state.contactors.find((contactor) => contactor.fingerSegment === 'distal');
    if (!middleContactor || !tipContactor || middleContactor.linkId === tipContactor.linkId) return null;
    const middleDriver = state.links.find((link) => link.id === middleContactor.linkId);
    const tipDriver = state.links.find((link) => link.id === tipContactor.linkId);
    if (!middleDriver || !tipDriver) return null;
    const tipJoint = state.joints.find((joint) => jointConnects(joint, middleDriver.id, tipDriver.id));
    if (!tipJoint) return null;
    const middleJoint = state.joints.find((joint) =>
      joint.id !== tipJoint.id && jointTouches(joint, middleDriver.id) &&
      otherLinkId(joint, middleDriver.id) !== null,
    );
    if (!middleJoint) return null;
    const anchorId = otherLinkId(middleJoint, middleDriver.id);
    const anchorLink = anchorId ? state.links.find((link) => link.id === anchorId) : undefined;
    const middleLocalPoint = jointLocalPoint(middleJoint, middleDriver.id);
    const anchorLocalPoint = anchorId ? jointLocalPoint(middleJoint, anchorId) : null;
    const tipLocalPoint = jointLocalPoint(tipJoint, tipDriver.id);
    const tipJointLocalOnMiddle = jointLocalPoint(tipJoint, middleDriver.id);
    if (!anchorLink || !middleLocalPoint || !anchorLocalPoint || !tipLocalPoint || !tipJointLocalOnMiddle) return null;
    return {
      anchorLink,
      middleDriver,
      tipDriver,
      middleContactor,
      tipContactor,
      middleJoint,
      tipJoint,
      middleLocalPoint,
      tipLocalPoint,
      tipJointLocalOnMiddle,
      anchorWorldPoint: localToWorld(anchorLocalPoint, anchorLink.pose),
    };
  }

  private dorsalLinkageTarget(state: SimulationState, contactor: HandContactor, link: Link): Vec2 {
    const frame = fingerSegmentFrame(state.hand, contactor.fingerSegment);
    const center = pointOnFinger(state.hand, contactor.fingerSegment, contactor.fingerPosition);
    return add(center, scale(
      frame.dorsalNormal,
      frame.width / 2 + link.width / 2 + ringAffordance(state.hand, contactor),
    ));
  }

  private solveDorsalContactor(state: SimulationState, contactor: HandContactor): boolean {
    const attachedLink = state.links.find((link) => link.id === contactor.linkId);
    if (!attachedLink) return false;
    contactor.linkagePoint = localToWorld(contactor.localPoint, attachedLink.pose);

    let targetCenter = subtract(contactor.linkagePoint, { x: 0, y: state.hand.sizeScale * 6 });
    let reachable = true;
    // Finger orientation determines the dorsal surface normal. A few bounded
    // iterations converge the centerline target without a general optimizer.
    for (let iteration = 0; iteration < 4; iteration += 1) {
      reachable = solveFingerPointContact(
        state.hand,
        targetCenter,
        contactor.fingerSegment,
        contactor.fingerPosition,
      );
      const frame = fingerSegmentFrame(state.hand, contactor.fingerSegment);
      const centerlineOffset = frame.width / 2 + attachedLink.width / 2 + ringAffordance(state.hand, contactor);
      targetCenter = subtract(contactor.linkagePoint, scale(frame.dorsalNormal, centerlineOffset));
    }
    contactor.fingerPoint = pointOnFinger(state.hand, contactor.fingerSegment, contactor.fingerPosition);
    return reachable;
  }

  private detectFingerCollision(state: SimulationState): string | null {
    const landmarks = fingerLandmarks(state.hand);
    const fingerSegments = [
      { name: 'proximal phalanx', id: 'proximal' as const, start: landmarks.mcp, end: landmarks.pip },
      { name: 'middle phalanx', id: 'middle' as const, start: landmarks.pip, end: landmarks.dip },
      { name: 'distal phalanx', id: 'distal' as const, start: landmarks.dip, end: landmarks.tip },
    ];
    const clearance = 0.35;
    for (const link of state.links) {
      const linkStart = localToWorld({ x: -link.length / 2, y: 0 }, link.pose);
      const linkEnd = localToWorld({ x: link.length / 2, y: 0 }, link.pose);
      for (const fingerSegment of fingerSegments) {
        // A driver carrying a contactor for this exact phalanx is a unilateral
        // actuator contact: overlap means the driver is loading the digit, not
        // that the whole mechanism pose must be rejected. Cross-segment and
        // non-contactor link penetrations remain hard exclusions.
        const isIntendedDriverContact = state.contactors.some((contactor) =>
          contactor.linkId === link.id && contactor.fingerSegment === fingerSegment.id,
        );
        if (isIntendedDriverContact) continue;
        const frame = fingerSegmentFrame(state.hand, fingerSegment.id);
        const separation = segmentSegmentDistance(linkStart, linkEnd, fingerSegment.start, fingerSegment.end);
        const requiredSeparation = link.width / 2 + frame.width / 2 + clearance;
        if (separation < requiredSeparation - 1e-5) {
          return `Collision · ${link.name} intersects ${fingerSegment.name}`;
        }
      }
    }
    return null;
  }

  private detectMountCollision(state: SimulationState): string | null {
    const rail = this.mountRail(state);
    if (!rail) return null;
    const railStart = localToWorld({ x: -rail.length / 2, y: 0 }, rail.pose);
    const railEnd = localToWorld({ x: rail.length / 2, y: 0 }, rail.pose);
    const railTangent = rotate({ x: 1, y: 0 }, rail.pose.angle);
    let railDorsalNormal = rotate({ x: 0, y: 1 }, rail.pose.angle);
    const railMidpoint = scale(add(railStart, railEnd), 0.5);
    if (dot(railDorsalNormal, subtract(railMidpoint, state.ground.surfacePoint)) < 0) {
      railDorsalNormal = scale(railDorsalNormal, -1);
    }
    const tangent = rotate({ x: 1, y: 0 }, state.ground.angle);
    const scalar = (point: Vec2): number => dot(subtract(point, state.ground.surfacePoint), tangent);
    const minimum = Math.min(0, ...state.ground.pivotPoints.map(scalar)) - 14;
    const maximum = Math.max(0, ...state.ground.pivotPoints.map(scalar)) + 14;
    const groundStart = add(state.ground.surfacePoint, scale(tangent, minimum));
    const groundEnd = add(state.ground.surfacePoint, scale(tangent, maximum));
    const clearance = 0.35;

    for (const link of state.links) {
      if (link.id === rail.id) continue;
      const linkStart = localToWorld({ x: -link.length / 2, y: 0 }, link.pose);
      const linkEnd = localToWorld({ x: link.length / 2, y: 0 }, link.pose);
      if (minimumRailSideDistance(
        linkStart,
        linkEnd,
        railStart,
        railTangent,
        railDorsalNormal,
        rail.length,
      ) < -1e-5) {
        return `Collision · ${link.name} passes below dorsal base rail`;
      }
      if (segmentSegmentDistance(linkStart, linkEnd, groundStart, groundEnd) < link.width / 2 + clearance) {
        return `Collision · ${link.name} intersects dorsal ground plane`;
      }

      const attachment = this.railAttachmentPoint(state, link.id);
      if (attachment) {
        const crossing = segmentIntersectionPoint(linkStart, linkEnd, railStart, railEnd);
        if (crossing && distance(crossing, attachment) > 1e-4) {
          return `Collision · ${link.name} crosses dorsal base rail`;
        }
        continue;
      }
      if (segmentSegmentDistance(linkStart, linkEnd, railStart, railEnd) <
          link.width / 2 + rail.width / 2 + clearance) {
        return `Collision · ${link.name} intersects dorsal base rail`;
      }
    }
    return null;
  }

  private railAttachmentPoint(state: SimulationState, linkId: string): Vec2 | null {
    if (linkId === state.servo.drivenLinkId) return state.servo.groundPoint;
    const rail = this.mountRail(state);
    if (!rail) return null;
    const joint = state.joints.find((candidate) => jointConnects(candidate, rail.id, linkId));
    const railLocalPoint = joint ? jointLocalPoint(joint, rail.id) : null;
    return railLocalPoint ? localToWorld(railLocalPoint, rail.pose) : null;
  }

  private mountRail(state: SimulationState): Link | null {
    const joint = state.joints.find((candidate) => candidate.id === state.servo.revoluteJointId);
    if (!joint) return null;
    const candidateId = joint.linkBId === state.servo.drivenLinkId ? joint.linkAId : joint.linkBId;
    return candidateId
      ? state.links.find((link) => link.id === candidateId && link.fixed) ?? null
      : null;
  }

  private createSnapshot(state: SimulationState): SolverSnapshot {
    return {
      servoAngle: state.servo.angle,
      servoDrivenLinkId: state.servo.drivenLinkId,
      servoRevoluteJointId: state.servo.revoluteJointId,
      servoGroundPoint: { ...state.servo.groundPoint },
      groundPivots: state.ground.pivotPoints.map((point) => ({ ...point })),
      groundSurfacePoint: { ...state.ground.surfacePoint },
      servoGroundOffset: state.ground.servoGroundOffset,
      baseRailAngleOffset: state.ground.baseRailAngleOffset,
      links: new Map(state.links.map((link) => [link.id, {
        position: { ...link.pose.position },
        angle: link.pose.angle,
        length: link.length,
        width: link.width,
      }])),
      joints: new Map(state.joints.map((joint) => [joint.id, {
        linkAId: joint.linkAId,
        linkBId: joint.linkBId,
        localPointA: joint.localPointA ? { ...joint.localPointA } : undefined,
        localPointB: { ...joint.localPointB },
        groundPoint: joint.groundPoint ? { ...joint.groundPoint } : undefined,
        minAngle: joint.minAngle,
        maxAngle: joint.maxAngle,
      }])),
      jointPositions: new Map(state.joints.flatMap((joint) => {
        const link = state.links.find((candidate) => candidate.id === joint.linkBId);
        return link
          ? [[joint.id, localToWorld(joint.localPointB, link.pose)] as const]
          : [];
      })),
      handAngles: [state.hand.mcpAngle, state.hand.pipAngle, state.hand.dipAngle],
      contactors: new Map(state.contactors.map((contactor) => [contactor.id, {
        localPoint: { ...contactor.localPoint },
        linkagePoint: { ...contactor.linkagePoint },
        fingerPoint: { ...contactor.fingerPoint },
      }])),
      statics: structuredClone(state.statics),
      jointConstraintStatus: structuredClone(state.jointConstraintStatus),
    };
  }

  private captureSnapshot(state: SimulationState): void {
    this.lastValidSnapshots.set(state, this.createSnapshot(state));
  }

  private restoreSnapshot(state: SimulationState, snapshot: SolverSnapshot): void {
    state.servo.angle = snapshot.servoAngle;
    state.servo.drivenLinkId = snapshot.servoDrivenLinkId;
    state.servo.revoluteJointId = snapshot.servoRevoluteJointId;
    state.servo.groundPoint = { ...snapshot.servoGroundPoint };
    state.ground.pivotPoints = snapshot.groundPivots.map((point) => ({ ...point }));
    state.ground.surfacePoint = { ...snapshot.groundSurfacePoint };
    state.ground.servoGroundOffset = snapshot.servoGroundOffset;
    state.ground.baseRailAngleOffset = snapshot.baseRailAngleOffset;
    for (const link of state.links) {
      const cached = snapshot.links.get(link.id);
      if (cached) {
        link.pose = { position: { ...cached.position }, angle: cached.angle };
        link.length = cached.length;
        link.width = cached.width;
      }
    }
    for (const joint of state.joints) {
      const cached = snapshot.joints.get(joint.id);
      if (!cached) continue;
      joint.linkAId = cached.linkAId;
      joint.linkBId = cached.linkBId;
      joint.localPointA = cached.localPointA ? { ...cached.localPointA } : undefined;
      joint.localPointB = { ...cached.localPointB };
      joint.groundPoint = cached.groundPoint ? { ...cached.groundPoint } : undefined;
      joint.minAngle = cached.minAngle;
      joint.maxAngle = cached.maxAngle;
    }
    [state.hand.mcpAngle, state.hand.pipAngle, state.hand.dipAngle] = snapshot.handAngles;
    for (const contactor of state.contactors) {
      const cached = snapshot.contactors.get(contactor.id);
      if (!cached) continue;
      contactor.localPoint = { ...cached.localPoint };
      contactor.linkagePoint = { ...cached.linkagePoint };
      contactor.fingerPoint = { ...cached.fingerPoint };
    }
    state.statics = structuredClone(snapshot.statics);
    state.jointConstraintStatus = structuredClone(snapshot.jointConstraintStatus);
  }

  private fail(state: SimulationState, message: string, snapshot: SolverSnapshot): void {
    this.restoreSnapshot(state, snapshot);
    state.valid = false;
    state.message = message;
  }
}

const radians = (degrees: number): number => (degrees * Math.PI) / 180;
const clamp = (value: number, minimum: number, maximum: number): number => Math.max(minimum, Math.min(maximum, value));
const vectorNorm = (values: number[]): number => Math.hypot(...values);

function jointTouches(joint: RevoluteJoint, linkId: string): boolean {
  return joint.linkAId === linkId || joint.linkBId === linkId;
}

function jointConnects(joint: RevoluteJoint, linkAId: string, linkBId: string): boolean {
  return (joint.linkAId === linkAId && joint.linkBId === linkBId) ||
    (joint.linkAId === linkBId && joint.linkBId === linkAId);
}

function otherLinkId(joint: RevoluteJoint, linkId: string): string | null {
  if (joint.linkAId === linkId) return joint.linkBId;
  if (joint.linkBId === linkId) return joint.linkAId;
  return null;
}

function jointLocalPoint(joint: RevoluteJoint, linkId: string): Vec2 | null {
  if (joint.linkAId === linkId) return joint.localPointA ?? null;
  if (joint.linkBId === linkId) return joint.localPointB;
  return null;
}

function relativeAngleAtJoint(joint: RevoluteJoint, first: Link, second: Link): number {
  const angleA = joint.linkAId === null
    ? 0
    : joint.linkAId === first.id
      ? first.pose.angle
      : second.pose.angle;
  const angleB = joint.linkBId === first.id ? first.pose.angle : second.pose.angle;
  return normalizeAngle(angleB - angleA);
}

function projectJointLinkAngle(
  candidateAngle: number,
  joint: RevoluteJoint,
  movingLinkId: string,
  otherAngle: number,
): number {
  const minimum = joint.minAngle ?? -Math.PI;
  const maximum = joint.maxAngle ?? Math.PI;
  if (maximum - minimum >= Math.PI * 2 - 1e-8) return candidateAngle;
  const relative = joint.linkBId === movingLinkId
    ? normalizeAngle(candidateAngle - otherAngle)
    : normalizeAngle(otherAngle - candidateAngle);
  const projected = minimum <= maximum
    ? clamp(relative, minimum, maximum)
    : relative >= minimum || relative <= maximum
      ? relative
      : Math.abs(normalizeAngle(relative - minimum)) <= Math.abs(normalizeAngle(relative - maximum))
        ? minimum
        : maximum;
  return joint.linkBId === movingLinkId ? otherAngle + projected : otherAngle - projected;
}

function uniquePoints(points: readonly Vec2[]): Vec2[] {
  const unique: Vec2[] = [];
  for (const point of points) {
    if (!unique.some((candidate) => distance(candidate, point) <= 1e-7)) unique.push({ ...point });
  }
  return unique;
}

function mergeJointStatuses(
  mechanism: readonly JointConstraintStatus[],
  specialized: readonly JointConstraintStatus[],
): JointConstraintStatus[] {
  const merged = new Map(mechanism.map((status) => [status.jointId, status]));
  for (const status of specialized) merged.set(status.jointId, status);
  return [...merged.values()];
}

function constraintStatus(
  jointId: string,
  angle: number,
  minimum: number,
  maximum: number,
): JointConstraintStatus {
  const tolerance = 1e-5;
  const state = Math.abs(angle - minimum) <= tolerance
    ? 'at-minimum'
    : Math.abs(angle - maximum) <= tolerance
      ? 'at-maximum'
      : 'free';
  return { jointId, state, angle, minimum, maximum };
}

function leastSquaresStep(jacobian: number[][], residual: number[], regularization: number): number[] | null {
  const columnCount = jacobian[0]?.length ?? 0;
  const normalMatrix = Array.from({ length: columnCount }, () => Array<number>(columnCount).fill(0));
  const normalValues = Array<number>(columnCount).fill(0);
  for (let row = 0; row < jacobian.length; row += 1) {
    for (let column = 0; column < columnCount; column += 1) {
      normalValues[column] = normalValues[column]! - jacobian[row]![column]! * residual[row]!;
      for (let other = 0; other < columnCount; other += 1) {
        normalMatrix[column]![other] = normalMatrix[column]![other]! +
          jacobian[row]![column]! * jacobian[row]![other]!;
      }
    }
  }
  for (let index = 0; index < columnCount; index += 1) {
    normalMatrix[index]![index] = normalMatrix[index]![index]! + regularization;
  }
  return solveLinearSystem(normalMatrix, normalValues);
}

function solveLinearSystem(matrix: number[][], values: number[]): number[] | null {
  const size = values.length;
  const augmented = matrix.map((row, index) => [...row, values[index]!]);
  for (let column = 0; column < size; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < size; row += 1) {
      if (Math.abs(augmented[row]![column]!) > Math.abs(augmented[pivot]![column]!)) pivot = row;
    }
    if (Math.abs(augmented[pivot]![column]!) < 1e-10) return null;
    [augmented[column], augmented[pivot]] = [augmented[pivot]!, augmented[column]!];
    const divisor = augmented[column]![column]!;
    for (let entry = column; entry <= size; entry += 1) {
      augmented[column]![entry] = augmented[column]![entry]! / divisor;
    }
    for (let row = 0; row < size; row += 1) {
      if (row === column) continue;
      const factor = augmented[row]![column]!;
      for (let entry = column; entry <= size; entry += 1) {
        augmented[row]![entry] = augmented[row]![entry]! - factor * augmented[column]![entry]!;
      }
    }
  }
  return augmented.map((row) => row[size]!);
}

function segmentIntersectionPoint(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2): Vec2 | null {
  const a = subtract(a1, a0);
  const b = subtract(b1, b0);
  const offset = subtract(b0, a0);
  const denominator = a.x * b.y - a.y * b.x;
  if (Math.abs(denominator) < 1e-9) return null;
  const t = (offset.x * b.y - offset.y * b.x) / denominator;
  const u = (offset.x * a.y - offset.y * a.x) / denominator;
  if (t < -1e-9 || t > 1 + 1e-9 || u < -1e-9 || u > 1 + 1e-9) return null;
  return add(a0, scale(a, t));
}

function minimumRailSideDistance(
  start: Vec2,
  end: Vec2,
  railStart: Vec2,
  railTangent: Vec2,
  dorsalNormal: Vec2,
  railLength: number,
): number {
  const startProjection = dot(subtract(start, railStart), railTangent);
  const endProjection = dot(subtract(end, railStart), railTangent);
  const candidates: Vec2[] = [];
  if (startProjection >= 0 && startProjection <= railLength) candidates.push(start);
  if (endProjection >= 0 && endProjection <= railLength) candidates.push(end);
  const projectionDelta = endProjection - startProjection;
  if (Math.abs(projectionDelta) > 1e-9) {
    for (const boundary of [0, railLength]) {
      const parameter = (boundary - startProjection) / projectionDelta;
      if (parameter > 0 && parameter < 1) {
        candidates.push(add(start, scale(subtract(end, start), parameter)));
      }
    }
  }
  if (candidates.length === 0) return Number.POSITIVE_INFINITY;
  return Math.min(...candidates.map((point) => dot(subtract(point, railStart), dorsalNormal)));
}
