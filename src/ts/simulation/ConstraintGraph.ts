import type {
  ComponentId,
  Link,
  LinearSlotJoint,
  RevoluteJoint,
  ServoJoint,
} from '../model';
import { distance, localToWorld, type Vec2 } from '../geometry';
import { SOLVER_TOLERANCES } from './solverTolerances';

/** Reserved body ID for the inertial world frame. */
export const WORLD_BODY_ID = '__world__' as const;

/** Joint ranges narrower than this are equality constraints, not inequalities. */
export const LOCKED_JOINT_ANGLE_TOLERANCE = SOLVER_TOLERANCES.lockedAngle;

export interface ConstraintGraphInput {
  links: readonly Link[];
  joints: readonly RevoluteJoint[];
  linearSlotJoints?: readonly LinearSlotJoint[];
  servo?: ServoJoint | null;
}

export interface WorldConstraintBody {
  id: typeof WORLD_BODY_ID;
  kind: 'world';
}

export interface LinkConstraintBody {
  id: ComponentId;
  kind: 'link';
  link: Link;
  fixed: boolean;
}

export type ConstraintGraphBody = WorldConstraintBody | LinkConstraintBody;

interface ConstraintBase {
  id: string;
  bodyAId: ComponentId | typeof WORLD_BODY_ID;
  bodyBId: ComponentId;
  scalarEquationCount: number;
}

export interface RevoluteGraphConstraint extends ConstraintBase {
  kind: 'revolute';
  jointId: ComponentId;
  joint: RevoluteJoint;
  scalarEquationCount: 2;
}

export interface LockedAngleGraphConstraint extends ConstraintBase {
  kind: 'locked-angle';
  jointId: ComponentId;
  joint: RevoluteJoint;
  targetAngle: number;
  scalarEquationCount: 1;
}

export interface FixedGraphConstraint extends ConstraintBase {
  kind: 'fixed';
  linkId: ComponentId;
  scalarEquationCount: 0;
}

export interface ActuatorGraphConstraint extends ConstraintBase {
  kind: 'actuator';
  actuatorId: ComponentId;
  revoluteJointId: ComponentId;
  revoluteConstraint: RevoluteGraphConstraint;
  servo: ServoJoint;
  targetAngle: number;
  scalarEquationCount: 1;
}

export interface LinearSlotGraphConstraint extends ConstraintBase {
  kind: 'linear-slot';
  jointId: ComponentId;
  joint: LinearSlotJoint;
  scalarEquationCount: 1;
}

export type ConstraintGraphConstraint =
  | RevoluteGraphConstraint
  | LockedAngleGraphConstraint
  | FixedGraphConstraint
  | ActuatorGraphConstraint
  | LinearSlotGraphConstraint;

export interface ConstraintGraph {
  worldBodyId: typeof WORLD_BODY_ID;
  bodies: ReadonlyMap<ComponentId | typeof WORLD_BODY_ID, ConstraintGraphBody>;
  constraints: readonly ConstraintGraphConstraint[];
  constraintsById: ReadonlyMap<string, ConstraintGraphConstraint>;
  constraintsByBodyId: ReadonlyMap<ComponentId | typeof WORLD_BODY_ID, readonly ConstraintGraphConstraint[]>;
  jointConstraints: ReadonlyMap<ComponentId, RevoluteGraphConstraint>;
  linearSlotConstraints: ReadonlyMap<ComponentId, LinearSlotGraphConstraint>;
}

export interface ConstraintGraphComponent {
  id: string;
  /** Link bodies plus the world body when this component has a world edge. */
  bodyIds: readonly (ComponentId | typeof WORLD_BODY_ID)[];
  linkIds: readonly ComponentId[];
  fixedLinkIds: readonly ComponentId[];
  constraints: readonly ConstraintGraphConstraint[];
  constraintIds: readonly string[];
  jointIds: readonly ComponentId[];
  actuatorIds: readonly ComponentId[];
  anchored: boolean;
}

export class ConstraintGraphError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'ConstraintGraphError';
  }
}

function isLockedJoint(joint: RevoluteJoint): boolean {
  return joint.minAngle !== undefined &&
    joint.maxAngle !== undefined &&
    Math.abs(joint.maxAngle - joint.minAngle) <= LOCKED_JOINT_ANGLE_TOLERANCE;
}

function isFinitePoint(point: Vec2): boolean {
  return Number.isFinite(point.x) && Number.isFinite(point.y);
}

function servoMountPoint(
  bodies: ReadonlyMap<ComponentId | typeof WORLD_BODY_ID, ConstraintGraphBody>,
  revolute: RevoluteGraphConstraint,
  drivenLinkId: ComponentId,
): Vec2 {
  const joint = revolute.joint;
  const otherBodyId = revolute.bodyBId === drivenLinkId
    ? revolute.bodyAId
    : revolute.bodyBId;
  if (otherBodyId === WORLD_BODY_ID) {
    if (joint.groundPoint === undefined) {
      throw new ConstraintGraphError(`Servo revolute ${joint.id} has no ground point`);
    }
    return joint.groundPoint;
  }

  const otherBody = bodies.get(otherBodyId);
  if (otherBody?.kind !== 'link' || !otherBody.fixed) {
    throw new ConstraintGraphError(
      `Servo revolute ${joint.id} must be mounted to world or a fixed link`,
    );
  }
  const localPoint = joint.linkAId === otherBodyId ? joint.localPointA : joint.localPointB;
  if (localPoint === undefined) {
    throw new ConstraintGraphError(`Servo revolute ${joint.id} is missing its fixed mount attachment`);
  }
  return localToWorld(localPoint, otherBody.link.pose);
}

function addConstraint(
  constraint: ConstraintGraphConstraint,
  constraints: ConstraintGraphConstraint[],
  constraintsById: Map<string, ConstraintGraphConstraint>,
  constraintsByBodyId: Map<ComponentId | typeof WORLD_BODY_ID, ConstraintGraphConstraint[]>,
): void {
  if (constraintsById.has(constraint.id)) {
    throw new ConstraintGraphError(`Duplicate graph constraint ID: ${constraint.id}`);
  }
  constraints.push(constraint);
  constraintsById.set(constraint.id, constraint);
  constraintsByBodyId.get(constraint.bodyAId)?.push(constraint);
  if (constraint.bodyBId !== constraint.bodyAId) {
    constraintsByBodyId.get(constraint.bodyBId)?.push(constraint);
  }
}

/**
 * Converts the serializable mechanism model into an explicit topology graph.
 * Model objects remain referenced so a graph built for a solve sees its current
 * poses and actuator command without copying persistent state.
 */
export function buildConstraintGraph(input: ConstraintGraphInput): ConstraintGraph {
  const bodies = new Map<ComponentId | typeof WORLD_BODY_ID, ConstraintGraphBody>();
  const constraints: ConstraintGraphConstraint[] = [];
  const constraintsById = new Map<string, ConstraintGraphConstraint>();
  const constraintsByBodyId = new Map<
    ComponentId | typeof WORLD_BODY_ID,
    ConstraintGraphConstraint[]
  >();
  const jointConstraints = new Map<ComponentId, RevoluteGraphConstraint>();
  const linearSlotConstraints = new Map<ComponentId, LinearSlotGraphConstraint>();

  const worldBody: WorldConstraintBody = { id: WORLD_BODY_ID, kind: 'world' };
  bodies.set(WORLD_BODY_ID, worldBody);
  constraintsByBodyId.set(WORLD_BODY_ID, []);

  for (const link of input.links) {
    if (link.id === WORLD_BODY_ID) {
      throw new ConstraintGraphError(`Link ID ${WORLD_BODY_ID} is reserved for the world body`);
    }
    if (bodies.has(link.id)) {
      throw new ConstraintGraphError(`Duplicate link ID: ${link.id}`);
    }
    const body: LinkConstraintBody = {
      id: link.id,
      kind: 'link',
      link,
      fixed: link.fixed === true,
    };
    bodies.set(link.id, body);
    constraintsByBodyId.set(link.id, []);
  }

  for (const link of input.links) {
    if (!link.fixed) continue;
    addConstraint({
      id: `fixed:${link.id}`,
      kind: 'fixed',
      bodyAId: WORLD_BODY_ID,
      bodyBId: link.id,
      linkId: link.id,
      scalarEquationCount: 0,
    }, constraints, constraintsById, constraintsByBodyId);
  }

  for (const joint of input.joints) {
    if (jointConstraints.has(joint.id)) {
      throw new ConstraintGraphError(`Duplicate revolute joint ID: ${joint.id}`);
    }
    const bodyB = bodies.get(joint.linkBId);
    if (bodyB?.kind !== 'link') {
      throw new ConstraintGraphError(
        `Joint ${joint.id} references missing linkB ${joint.linkBId}`,
      );
    }

    const bodyAId = joint.linkAId ?? WORLD_BODY_ID;
    if (joint.linkAId !== null) {
      const bodyA = bodies.get(joint.linkAId);
      if (bodyA?.kind !== 'link') {
        throw new ConstraintGraphError(
          `Joint ${joint.id} references missing linkA ${joint.linkAId}`,
        );
      }
      if (joint.localPointA === undefined) {
        throw new ConstraintGraphError(
          `Link-link joint ${joint.id} is missing localPointA`,
        );
      }
    } else if (joint.groundPoint === undefined) {
      throw new ConstraintGraphError(
        `Ground joint ${joint.id} is missing groundPoint`,
      );
    }

    const revolute: RevoluteGraphConstraint = {
      id: `joint:${joint.id}`,
      kind: 'revolute',
      jointId: joint.id,
      joint,
      bodyAId,
      bodyBId: joint.linkBId,
      scalarEquationCount: 2,
    };
    addConstraint(revolute, constraints, constraintsById, constraintsByBodyId);
    jointConstraints.set(joint.id, revolute);

    if (isLockedJoint(joint)) {
      const targetAngle = ((joint.minAngle as number) + (joint.maxAngle as number)) / 2;
      addConstraint({
        id: `locked-angle:${joint.id}`,
        kind: 'locked-angle',
        jointId: joint.id,
        joint,
        bodyAId,
        bodyBId: joint.linkBId,
        targetAngle,
        scalarEquationCount: 1,
      }, constraints, constraintsById, constraintsByBodyId);
    }
  }

  for (const joint of input.linearSlotJoints ?? []) {
    if (jointConstraints.has(joint.id) || linearSlotConstraints.has(joint.id)) {
      throw new ConstraintGraphError(`Duplicate mechanism joint ID: ${joint.id}`);
    }
    const pinBody = bodies.get(joint.pinLinkId);
    if (pinBody?.kind !== 'link') {
      throw new ConstraintGraphError(
        `Linear slot ${joint.id} references missing pin link ${joint.pinLinkId}`,
      );
    }
    const slotBodyId = joint.slotLinkId ?? WORLD_BODY_ID;
    if (joint.slotLinkId !== null) {
      const slotBody = bodies.get(joint.slotLinkId);
      if (slotBody?.kind !== 'link') {
        throw new ConstraintGraphError(
          `Linear slot ${joint.id} references missing slot link ${joint.slotLinkId}`,
        );
      }
      if (joint.slotLinkId === joint.pinLinkId) {
        throw new ConstraintGraphError(
          `Linear slot ${joint.id} cannot connect a link to itself`,
        );
      }
    }
    if (!isFinitePoint(joint.slotOrigin) || !isFinitePoint(joint.slotDirection) ||
        !isFinitePoint(joint.pinLocalPoint)) {
      throw new ConstraintGraphError(`Linear slot ${joint.id} geometry must be finite`);
    }
    if (Math.hypot(joint.slotDirection.x, joint.slotDirection.y) <= SOLVER_TOLERANCES.length) {
      throw new ConstraintGraphError(`Linear slot ${joint.id} direction must be non-zero`);
    }
    if (!Number.isFinite(joint.minTravel) || !Number.isFinite(joint.maxTravel) ||
        joint.minTravel > joint.maxTravel) {
      throw new ConstraintGraphError(
        `Linear slot ${joint.id} travel bounds must be finite and ordered`,
      );
    }
    if (joint.friction !== undefined &&
        (!Number.isFinite(joint.friction.coefficient) || joint.friction.coefficient < 0)) {
      throw new ConstraintGraphError(
        `Linear slot ${joint.id} friction coefficient must be finite and non-negative`,
      );
    }
    const slotConstraint: LinearSlotGraphConstraint = {
      id: `linear-slot:${joint.id}`,
      kind: 'linear-slot',
      jointId: joint.id,
      joint,
      bodyAId: slotBodyId,
      bodyBId: joint.pinLinkId,
      scalarEquationCount: 1,
    };
    addConstraint(slotConstraint, constraints, constraintsById, constraintsByBodyId);
    linearSlotConstraints.set(joint.id, slotConstraint);
  }

  if (input.servo !== undefined && input.servo !== null) {
    const drivenBody = bodies.get(input.servo.drivenLinkId);
    if (drivenBody?.kind !== 'link') {
      throw new ConstraintGraphError(
        `Servo ${input.servo.id} references missing driven link ${input.servo.drivenLinkId}`,
      );
    }
    if (drivenBody.fixed) {
      throw new ConstraintGraphError(
        `Servo ${input.servo.id} cannot drive fixed link ${input.servo.drivenLinkId}`,
      );
    }
    const actuatorRevolute = jointConstraints.get(input.servo.revoluteJointId);
    if (actuatorRevolute === undefined) {
      throw new ConstraintGraphError(
        `Servo ${input.servo.id} references missing revolute joint ${input.servo.revoluteJointId}`,
      );
    }
    if (actuatorRevolute.bodyAId !== input.servo.drivenLinkId &&
        actuatorRevolute.bodyBId !== input.servo.drivenLinkId) {
      throw new ConstraintGraphError(
        `Servo ${input.servo.id} joint ${input.servo.revoluteJointId} is not incident to driven link ${input.servo.drivenLinkId}`,
      );
    }
    const mountPoint = servoMountPoint(bodies, actuatorRevolute, input.servo.drivenLinkId);
    if (!isFinitePoint(input.servo.groundPoint) || !isFinitePoint(mountPoint)) {
      throw new ConstraintGraphError(`Servo ${input.servo.id} mount point must be finite`);
    }
    const mountError = distance(mountPoint, input.servo.groundPoint);
    if (mountError > SOLVER_TOLERANCES.closure) {
      throw new ConstraintGraphError(
        `Servo ${input.servo.id} ground point does not match its fixed revolute mount ` +
        `(error ${mountError.toPrecision(4)})`,
      );
    }
    addConstraint({
      id: `actuator:${input.servo.id}`,
      kind: 'actuator',
      actuatorId: input.servo.id,
      revoluteJointId: input.servo.revoluteJointId,
      revoluteConstraint: actuatorRevolute,
      servo: input.servo,
      targetAngle: input.servo.angle,
      bodyAId: WORLD_BODY_ID,
      bodyBId: input.servo.drivenLinkId,
      scalarEquationCount: 1,
    }, constraints, constraintsById, constraintsByBodyId);
  }

  return {
    worldBodyId: WORLD_BODY_ID,
    bodies,
    constraints,
    constraintsById,
    constraintsByBodyId,
    jointConstraints,
    linearSlotConstraints,
  };
}

function otherLinkBodyId(
  constraint: ConstraintGraphConstraint,
  bodyId: ComponentId,
): ComponentId | null {
  const otherId = constraint.bodyAId === bodyId
    ? constraint.bodyBId
    : constraint.bodyBId === bodyId
      ? constraint.bodyAId
      : null;
  return otherId === null || otherId === WORLD_BODY_ID ? null : otherId;
}

/**
 * Extracts independent mechanism islands. The world is deliberately treated as
 * a reference rather than a bridge: two unrelated mechanisms bolted to the
 * same ground remain separate Jacobian blocks.
 */
export function connectedComponents(graph: ConstraintGraph): ConstraintGraphComponent[] {
  const linkIds = [...graph.bodies.values()]
    .filter((body): body is LinkConstraintBody => body.kind === 'link')
    .map((body) => body.id)
    .sort((left, right) => left.localeCompare(right));
  const unvisited = new Set(linkIds);
  const componentLinkSets: Set<ComponentId>[] = [];

  for (const rootId of linkIds) {
    if (!unvisited.has(rootId)) continue;
    const componentLinks = new Set<ComponentId>();
    const pending = [rootId];
    unvisited.delete(rootId);
    while (pending.length > 0) {
      const bodyId = pending.pop();
      if (bodyId === undefined) break;
      componentLinks.add(bodyId);
      for (const constraint of graph.constraintsByBodyId.get(bodyId) ?? []) {
        const adjacentId = otherLinkBodyId(constraint, bodyId);
        if (adjacentId === null || !unvisited.has(adjacentId)) continue;
        unvisited.delete(adjacentId);
        pending.push(adjacentId);
      }
    }
    componentLinkSets.push(componentLinks);
  }

  return componentLinkSets.map((componentLinks, index) => {
    const sortedLinkIds = [...componentLinks].sort((left, right) => left.localeCompare(right));
    const constraints = graph.constraints
      .filter((constraint) =>
        componentLinks.has(constraint.bodyBId) ||
        (constraint.bodyAId !== WORLD_BODY_ID && componentLinks.has(constraint.bodyAId)),
      )
      .sort((left, right) => left.id.localeCompare(right.id));
    const anchored = constraints.some((constraint) =>
      constraint.bodyAId === WORLD_BODY_ID || constraint.bodyBId === WORLD_BODY_ID,
    );
    const fixedLinkIds = sortedLinkIds.filter((linkId) => {
      const body = graph.bodies.get(linkId);
      return body?.kind === 'link' && body.fixed;
    });
    const jointIds = [...new Set(constraints
      .filter((constraint): constraint is RevoluteGraphConstraint | LinearSlotGraphConstraint =>
        constraint.kind === 'revolute' || constraint.kind === 'linear-slot')
      .map((constraint) => constraint.jointId))]
      .sort((left, right) => left.localeCompare(right));
    const actuatorIds = constraints
      .filter((constraint): constraint is ActuatorGraphConstraint => constraint.kind === 'actuator')
      .map((constraint) => constraint.actuatorId)
      .sort((left, right) => left.localeCompare(right));

    return {
      id: `component-${index}`,
      bodyIds: anchored ? [WORLD_BODY_ID, ...sortedLinkIds] : sortedLinkIds,
      linkIds: sortedLinkIds,
      fixedLinkIds,
      constraints,
      constraintIds: constraints.map((constraint) => constraint.id),
      jointIds,
      actuatorIds,
      anchored,
    };
  });
}
