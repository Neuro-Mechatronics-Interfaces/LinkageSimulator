import {
  add,
  circleCircleIntersection,
  distance,
  dot,
  localToWorld,
  rotate,
  scale,
  segmentSegmentDistance,
  subtract,
  type Vec2,
} from '../geometry';
import type { HandContactor, Link, SimulationState } from '../model';
import { fingerLandmarks, fingerSegmentFrame, pointOnFinger, solveFingerPointContact } from './fingerKinematics';

const midpoint = (a: Vec2, b: Vec2): Vec2 => scale(add(a, b), 0.5);

function setLinkFromEndpoints(link: Link, start: Vec2, end: Vec2): void {
  link.length = distance(start, end);
  link.pose.position = midpoint(start, end);
  link.pose.angle = Math.atan2(end.y - start.y, end.x - start.x);
}

function setLinkFromStartAndAngle(link: Link, start: Vec2, angle: number): void {
  link.pose.angle = angle;
  link.pose.position = add(start, rotate({ x: link.length / 2, y: 0 }, angle));
}

interface SolverSnapshot {
  servoAngle: number;
  servoGroundPoint: Vec2;
  crankGroundPoint: Vec2;
  groundPivots: Vec2[];
  groundSurfacePoint: Vec2;
  servoGroundOffset: number;
  baseRailAngleOffset: number;
  rockerGroundPoint: Vec2;
  preferredOutputPoint: Vec2;
  links: Map<string, { position: Vec2; angle: number; length: number; width: number }>;
  handAngles: readonly [number, number, number];
  contactors: Map<string, { localPoint: Vec2; linkagePoint: Vec2; fingerPoint: Vec2 }>;
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
    this.synchronizeMount(state);
    const definition = state.fourBar;
    const crank = state.links.find((candidate) => candidate.id === definition.crankLinkId);
    const coupler = state.links.find((candidate) => candidate.id === definition.couplerLinkId);
    const rocker = state.links.find((candidate) => candidate.id === definition.rockerLinkId);
    const anchorDriver = state.links.find((candidate) => candidate.id === definition.anchorDriverLinkId);
    const middleDriver = state.links.find((candidate) => candidate.id === definition.middleDriverLinkId);
    const tipDriver = state.links.find((candidate) => candidate.id === definition.tipDriverLinkId);
    if (!crank || !coupler || !rocker || !anchorDriver || !middleDriver || !tipDriver) {
      this.fail(state, 'Demonstrator topology is incomplete');
      return;
    }
    definition.couplerJointDistance = coupler.length;

    const crankEnd = add(definition.crankGroundPoint, rotate({ x: crank.length, y: 0 }, state.servo.angle));
    setLinkFromEndpoints(crank, definition.crankGroundPoint, crankEnd);

    const intersection = circleCircleIntersection(
      crankEnd,
      definition.couplerJointDistance,
      definition.rockerGroundPoint,
      rocker.length,
    );

    let rockerCouplerJoint: Vec2 | null = null;
    if (intersection.kind === 'two') {
      rockerCouplerJoint = distance(intersection.points[0], definition.preferredOutputPoint) <=
        distance(intersection.points[1], definition.preferredOutputPoint)
        ? intersection.points[0]
        : intersection.points[1];
    } else if (intersection.kind === 'tangent') {
      rockerCouplerJoint = intersection.point;
    }

    if (!rockerCouplerJoint) {
      this.fail(state, `Unsolvable four-bar geometry (${intersection.kind})`);
      return;
    }

    setLinkFromEndpoints(coupler, crankEnd, rockerCouplerJoint);
    setLinkFromEndpoints(rocker, definition.rockerGroundPoint, rockerCouplerJoint);
    setLinkFromStartAndAngle(
      anchorDriver,
      rockerCouplerJoint,
      coupler.pose.angle + definition.anchorDriverAngleOffset,
    );
    const anchorEnd = localToWorld({ x: anchorDriver.length / 2, y: 0 }, anchorDriver.pose);
    const contactChainReachable = this.solveDorsalContactorChain(
      state,
      anchorEnd,
      middleDriver,
      tipDriver,
    );
    definition.preferredOutputPoint = rockerCouplerJoint;
    this.synchronizeSolverJoints(state, crank, coupler, rocker, anchorDriver, middleDriver, tipDriver);
    this.synchronizePassiveAttachedLinks(state);

    const chainLinkIds = new Set([middleDriver.id, tipDriver.id]);
    let handContactReachable = contactChainReachable;
    for (const contactor of state.contactors.filter((candidate) => !chainLinkIds.has(candidate.linkId))) {
      handContactReachable = this.solveDorsalContactor(state, contactor) && handContactReachable;
    }

    const collision = this.detectFingerCollision(state);
    if (collision) {
      this.fail(state, collision);
      return;
    }
    const mountCollision = this.detectMountCollision(state);
    if (mountCollision) {
      this.fail(state, mountCollision);
      return;
    }

    state.valid = true;
    state.message = handContactReachable ? 'Constraints solved' : 'Linkage solved · contact projected to hand ROM';
    this.captureSnapshot(state);
  }

  /** Inverse-pose a constrained link endpoint by choosing the nearest servo angle. */
  solveForLinkEndpoint(state: SimulationState, linkId: string, target: Vec2): void {
    state.enabled = false;
    const controlledLink = state.links.find((candidate) => candidate.id === linkId);
    if (!controlledLink || controlledLink.fixed) return;
    if (linkId === state.servo.drivenLinkId) {
      const vector = subtract(target, state.servo.groundPoint);
      state.servo.angle = Math.max(
        state.servo.minAngle,
        Math.min(state.servo.maxAngle, Math.atan2(vector.y, vector.x)),
      );
      this.solve(state);
      state.message = 'Crank endpoint control';
      return;
    }

    const isConstrained = [
      state.fourBar.couplerLinkId,
      state.fourBar.rockerLinkId,
      state.fourBar.anchorDriverLinkId,
      state.fourBar.middleDriverLinkId,
      state.fourBar.tipDriverLinkId,
    ].includes(linkId);
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

  private synchronizeMount(state: SimulationState): void {
    const rail = state.links.find((link) => link.id === 'ground-rail');
    if (!rail) return;
    const normal = rotate({ x: 0, y: 1 }, state.ground.angle);
    const servoPoint = add(state.ground.surfacePoint, scale(normal, state.ground.servoGroundOffset));
    const rockerPoint = add(servoPoint, rotate({ x: rail.length, y: 0 }, state.ground.baseRailAngleOffset));
    state.servo.groundPoint = servoPoint;
    state.fourBar.crankGroundPoint = servoPoint;
    state.fourBar.rockerGroundPoint = rockerPoint;
    state.ground.pivotPoints = [{ ...servoPoint }, { ...rockerPoint }];
    setLinkFromEndpoints(rail, servoPoint, rockerPoint);
  }

  /**
   * Solve the two default contactors together. The two free revolute angles
   * and the coupled MCP/PIP pose provide four variables for four planar point
   * constraints, including contactors repositioned along either driver.
   */
  private solveDorsalContactorChain(
    state: SimulationState,
    anchorEnd: Vec2,
    middleDriver: Link,
    tipDriver: Link,
  ): boolean {
    const middleContactor = state.contactors.find((candidate) => candidate.linkId === middleDriver.id);
    const tipContactor = state.contactors.find((candidate) => candidate.linkId === tipDriver.id);
    if (!middleContactor || !tipContactor) return false;
    const variables = [state.hand.mcpAngle, state.hand.pipAngle, middleDriver.pose.angle, tipDriver.pose.angle];
    const minimum = [
      radians(state.hand.rom.mcp[0]),
      radians(state.hand.rom.pip[0]),
      -Math.PI * 2,
      -Math.PI * 2,
    ];
    const maximum = [
      radians(state.hand.rom.mcp[1]),
      radians(state.hand.rom.pip[1]),
      Math.PI * 2,
      Math.PI * 2,
    ];
    let residual = this.evaluateContactorChain(
      state,
      anchorEnd,
      middleDriver,
      tipDriver,
      middleContactor,
      tipContactor,
      variables,
    );
    const epsilon = 1e-4;
    for (let iteration = 0; iteration < 32 && vectorNorm(residual) > 1e-5; iteration += 1) {
      const jacobian = Array.from({ length: 4 }, () => Array<number>(4).fill(0));
      for (let column = 0; column < 4; column += 1) {
        const perturbed = [...variables];
        perturbed[column] = perturbed[column]! + epsilon;
        const sample = this.evaluateContactorChain(
          state,
          anchorEnd,
          middleDriver,
          tipDriver,
          middleContactor,
          tipContactor,
          perturbed,
        );
        for (let row = 0; row < 4; row += 1) jacobian[row]![column] = (sample[row]! - residual[row]!) / epsilon;
      }
      const delta = solveLinearSystem(jacobian, residual.map((value) => -value));
      if (!delta) break;
      const largest = Math.max(...delta.map((value) => Math.abs(value)));
      const damping = largest > 0.35 ? 0.35 / largest : 1;
      for (let index = 0; index < 4; index += 1) {
        variables[index] = clamp(variables[index]! + delta[index]! * damping, minimum[index]!, maximum[index]!);
      }
      residual = this.evaluateContactorChain(
        state,
        anchorEnd,
        middleDriver,
        tipDriver,
        middleContactor,
        tipContactor,
        variables,
      );
    }
    this.evaluateContactorChain(
      state,
      anchorEnd,
      middleDriver,
      tipDriver,
      middleContactor,
      tipContactor,
      variables,
    );
    return vectorNorm(residual) <= 0.02;
  }

  private evaluateContactorChain(
    state: SimulationState,
    anchorEnd: Vec2,
    middleDriver: Link,
    tipDriver: Link,
    middleContactor: HandContactor,
    tipContactor: HandContactor,
    variables: number[],
  ): number[] {
    state.hand.mcpAngle = variables[0]!;
    state.hand.pipAngle = variables[1]!;
    state.hand.dipAngle = clamp(
      variables[1]! * 0.62,
      radians(state.hand.rom.dip[0]),
      radians(state.hand.rom.dip[1]),
    );
    setLinkFromStartAndAngle(middleDriver, anchorEnd, variables[2]!);
    const middleEnd = localToWorld({ x: middleDriver.length / 2, y: 0 }, middleDriver.pose);
    setLinkFromStartAndAngle(tipDriver, middleEnd, variables[3]!);
    middleContactor.linkagePoint = localToWorld(middleContactor.localPoint, middleDriver.pose);
    tipContactor.linkagePoint = localToWorld(tipContactor.localPoint, tipDriver.pose);
    middleContactor.fingerPoint = pointOnFinger(
      state.hand,
      middleContactor.fingerSegment,
      middleContactor.fingerPosition,
    );
    tipContactor.fingerPoint = pointOnFinger(state.hand, tipContactor.fingerSegment, tipContactor.fingerPosition);
    const middleTarget = this.dorsalLinkageTarget(state, middleContactor, middleDriver);
    const tipTarget = this.dorsalLinkageTarget(state, tipContactor, tipDriver);
    return [
      middleContactor.linkagePoint.x - middleTarget.x,
      middleContactor.linkagePoint.y - middleTarget.y,
      tipContactor.linkagePoint.x - tipTarget.x,
      tipContactor.linkagePoint.y - tipTarget.y,
    ];
  }

  private dorsalLinkageTarget(state: SimulationState, contactor: HandContactor, link: Link): Vec2 {
    const frame = fingerSegmentFrame(state.hand, contactor.fingerSegment);
    const center = pointOnFinger(state.hand, contactor.fingerSegment, contactor.fingerPosition);
    return add(center, scale(
      frame.dorsalNormal,
      frame.width / 2 + link.width / 2 + contactor.bandClearance,
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
      const centerlineOffset = frame.width / 2 + attachedLink.width / 2 + contactor.bandClearance;
      targetCenter = subtract(contactor.linkagePoint, scale(frame.dorsalNormal, centerlineOffset));
    }
    contactor.fingerPoint = pointOnFinger(state.hand, contactor.fingerSegment, contactor.fingerPosition);
    return reachable;
  }

  private synchronizePassiveAttachedLinks(state: SimulationState): void {
    const solverJointIds = new Set([
      'servo-crank-joint',
      'crank-coupler-joint',
      'coupler-rocker-joint',
      'rocker-ground-joint',
      'anchor-driver-joint',
      'middle-driver-joint',
      'tip-driver-joint',
    ]);
    const solverLinkIds = new Set([
      state.fourBar.crankLinkId,
      state.fourBar.couplerLinkId,
      state.fourBar.rockerLinkId,
      state.fourBar.anchorDriverLinkId,
      state.fourBar.middleDriverLinkId,
      state.fourBar.tipDriverLinkId,
    ]);
    // Repeated passes propagate short parent-child chains without requiring a
    // general constraint graph in this prototype.
    for (let pass = 0; pass < state.links.length; pass += 1) {
      for (const joint of state.joints) {
        if (solverJointIds.has(joint.id) || solverLinkIds.has(joint.linkBId)) continue;
        const child = state.links.find((link) => link.id === joint.linkBId);
        if (!child || child.fixed) continue;
        const parent = joint.linkAId
          ? state.links.find((link) => link.id === joint.linkAId)
          : undefined;
        const attachment = parent && joint.localPointA
          ? localToWorld(joint.localPointA, parent.pose)
          : joint.groundPoint;
        if (!attachment) continue;
        child.pose.position = subtract(attachment, rotate(joint.localPointB, child.pose.angle));
      }
    }
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
    const rail = state.links.find((link) => link.id === 'ground-rail');
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
    const joint = state.joints.find((candidate) =>
      candidate.linkAId === 'ground-rail' && candidate.linkBId === linkId,
    );
    if (!joint?.localPointA) return null;
    const rail = state.links.find((link) => link.id === 'ground-rail');
    return rail ? localToWorld(joint.localPointA, rail.pose) : null;
  }

  private captureSnapshot(state: SimulationState): void {
    this.lastValidSnapshots.set(state, {
      servoAngle: state.servo.angle,
      servoGroundPoint: { ...state.servo.groundPoint },
      crankGroundPoint: { ...state.fourBar.crankGroundPoint },
      groundPivots: state.ground.pivotPoints.map((point) => ({ ...point })),
      groundSurfacePoint: { ...state.ground.surfacePoint },
      servoGroundOffset: state.ground.servoGroundOffset,
      baseRailAngleOffset: state.ground.baseRailAngleOffset,
      rockerGroundPoint: { ...state.fourBar.rockerGroundPoint },
      preferredOutputPoint: { ...state.fourBar.preferredOutputPoint },
      links: new Map(state.links.map((link) => [link.id, {
        position: { ...link.pose.position },
        angle: link.pose.angle,
        length: link.length,
        width: link.width,
      }])),
      handAngles: [state.hand.mcpAngle, state.hand.pipAngle, state.hand.dipAngle],
      contactors: new Map(state.contactors.map((contactor) => [contactor.id, {
        localPoint: { ...contactor.localPoint },
        linkagePoint: { ...contactor.linkagePoint },
        fingerPoint: { ...contactor.fingerPoint },
      }])),
    });
  }

  private restoreSnapshot(state: SimulationState): void {
    const snapshot = this.lastValidSnapshots.get(state);
    if (!snapshot) return;
    state.servo.angle = snapshot.servoAngle;
    state.servo.groundPoint = { ...snapshot.servoGroundPoint };
    state.fourBar.crankGroundPoint = { ...snapshot.crankGroundPoint };
    state.fourBar.preferredOutputPoint = { ...snapshot.preferredOutputPoint };
    state.ground.pivotPoints = snapshot.groundPivots.map((point) => ({ ...point }));
    state.ground.surfacePoint = { ...snapshot.groundSurfacePoint };
    state.ground.servoGroundOffset = snapshot.servoGroundOffset;
    state.ground.baseRailAngleOffset = snapshot.baseRailAngleOffset;
    state.fourBar.rockerGroundPoint = { ...snapshot.rockerGroundPoint };
    for (const link of state.links) {
      const cached = snapshot.links.get(link.id);
      if (cached) {
        link.pose = { position: { ...cached.position }, angle: cached.angle };
        link.length = cached.length;
        link.width = cached.width;
      }
    }
    [state.hand.mcpAngle, state.hand.pipAngle, state.hand.dipAngle] = snapshot.handAngles;
    for (const contactor of state.contactors) {
      const cached = snapshot.contactors.get(contactor.id);
      if (!cached) continue;
      contactor.localPoint = { ...cached.localPoint };
      contactor.linkagePoint = { ...cached.linkagePoint };
      contactor.fingerPoint = { ...cached.fingerPoint };
    }
  }

  private fail(state: SimulationState, message: string): void {
    this.restoreSnapshot(state);
    state.valid = false;
    state.message = message;
  }

  private synchronizeSolverJoints(
    state: SimulationState,
    crank: Link,
    coupler: Link,
    rocker: Link,
    anchorDriver: Link,
    middleDriver: Link,
    tipDriver: Link,
  ): void {
    const groundRail = state.links.find((link) => link.id === 'ground-rail');
    const servoCrank = state.joints.find((joint) => joint.id === 'servo-crank-joint');
    if (servoCrank && groundRail) {
      servoCrank.localPointA = { x: -groundRail.length / 2, y: 0 };
      servoCrank.localPointB = { x: -crank.length / 2, y: 0 };
      servoCrank.minAngle = state.servo.minAngle;
      servoCrank.maxAngle = state.servo.maxAngle;
    }
    const crankCoupler = state.joints.find((joint) => joint.id === 'crank-coupler-joint');
    if (crankCoupler) {
      crankCoupler.localPointA = { x: crank.length / 2, y: 0 };
      crankCoupler.localPointB = { x: -coupler.length / 2, y: 0 };
    }
    const couplerRocker = state.joints.find((joint) => joint.id === 'coupler-rocker-joint');
    if (couplerRocker) {
      couplerRocker.localPointA = { x: coupler.length / 2, y: 0 };
      couplerRocker.localPointB = { x: rocker.length / 2, y: 0 };
    }
    const rockerGround = state.joints.find((joint) => joint.id === 'rocker-ground-joint');
    if (rockerGround) {
      if (groundRail) rockerGround.localPointA = { x: groundRail.length / 2, y: 0 };
      rockerGround.localPointB = { x: -rocker.length / 2, y: 0 };
    }
    const anchorDriverJoint = state.joints.find((joint) => joint.id === 'anchor-driver-joint');
    if (anchorDriverJoint) {
      anchorDriverJoint.localPointA = { x: coupler.length / 2, y: 0 };
      anchorDriverJoint.localPointB = { x: -anchorDriver.length / 2, y: 0 };
    }
    const middleDriverJoint = state.joints.find((joint) => joint.id === 'middle-driver-joint');
    if (middleDriverJoint) {
      middleDriverJoint.localPointA = { x: anchorDriver.length / 2, y: 0 };
      middleDriverJoint.localPointB = { x: -middleDriver.length / 2, y: 0 };
    }
    const tipDriverJoint = state.joints.find((joint) => joint.id === 'tip-driver-joint');
    if (tipDriverJoint) {
      tipDriverJoint.localPointA = { x: middleDriver.length / 2, y: 0 };
      tipDriverJoint.localPointB = { x: -tipDriver.length / 2, y: 0 };
    }
  }
}

const radians = (degrees: number): number => (degrees * Math.PI) / 180;
const clamp = (value: number, minimum: number, maximum: number): number => Math.max(minimum, Math.min(maximum, value));
const vectorNorm = (values: number[]): number => Math.hypot(...values);

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
