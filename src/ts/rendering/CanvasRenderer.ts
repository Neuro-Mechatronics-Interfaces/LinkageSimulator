import { add, dot, localToWorld, rotate, scale, subtract, type Vec2 } from '../geometry';
import type { FingerSegment, Selection, SimulationState } from '../model';
import { fingerLandmarks, fingerSegmentFrame } from '../simulation';
import { Camera2D } from './Camera2D';

const COLORS = {
  background: '#0c1117',
  gridMinor: 'rgba(143, 168, 187, 0.055)',
  gridMajor: 'rgba(143, 168, 187, 0.12)',
  handFill: '#725e55',
  handBone: '#b99c8e',
  handJoint: '#e2c3b2',
  link: '#53b4c9',
  linkEdge: '#a5e1e9',
  fixedLink: '#52636c',
  fixedLinkEdge: '#9badb5',
  joint: '#e9f2f4',
  jointCore: '#19313a',
  servo: '#f0a55b',
  contact: '#d6ef73',
  selected: '#fff4b0',
  construction: 'rgba(118, 205, 219, 0.35)',
  invalid: '#ef6b73',
};

export class CanvasRenderer {
  readonly camera = new Camera2D();
  private readonly context: CanvasRenderingContext2D;
  private width = 1;
  private height = 1;
  private pixelRatio = 1;

  constructor(private readonly canvas: HTMLCanvasElement) {
    const context = canvas.getContext('2d');
    if (!context) throw new Error('Canvas 2-D context is unavailable.');
    this.context = context;
  }

  get viewport(): Readonly<{ width: number; height: number }> {
    return { width: this.width, height: this.height };
  }

  render(state: SimulationState, selection: Selection): void {
    this.resizeToDisplaySize();
    const context = this.context;
    context.setTransform(1, 0, 0, 1, 0, 0);
    context.clearRect(0, 0, this.canvas.width, this.canvas.height);
    context.setTransform(this.pixelRatio, 0, 0, this.pixelRatio, 0, 0);
    context.fillStyle = COLORS.background;
    context.fillRect(0, 0, this.width, this.height);
    this.drawGrid();
    this.drawHand(state);
    this.drawGround(state);
    if (state.showConstruction) this.drawConstruction(state);
    this.drawLinks(state, selection);
    this.drawJoints(state, selection);
    this.drawServo(state, selection);
    this.drawContactors(state, selection);
    this.drawLinkEndpoints(state, selection);
    if (!state.valid) this.drawInvalidOverlay(state.message);
  }

  screenToWorld(point: Vec2): Vec2 {
    return this.camera.screenToWorld(point, this.width, this.height);
  }

  private resizeToDisplaySize(): void {
    const pixelRatio = window.devicePixelRatio || 1;
    this.pixelRatio = pixelRatio;
    const width = Math.max(1, this.canvas.clientWidth);
    const height = Math.max(1, this.canvas.clientHeight);
    const targetWidth = Math.round(width * pixelRatio);
    const targetHeight = Math.round(height * pixelRatio);
    if (this.canvas.width !== targetWidth || this.canvas.height !== targetHeight) {
      this.canvas.width = targetWidth;
      this.canvas.height = targetHeight;
    }
    this.width = width;
    this.height = height;
  }

  private screen(point: Vec2): Vec2 {
    return this.camera.worldToScreen(point, this.width, this.height);
  }

  private drawGrid(): void {
    const context = this.context;
    const topLeft = this.screenToWorld({ x: 0, y: 0 });
    const bottomRight = this.screenToWorld({ x: this.width, y: this.height });
    const minorStep = 10;
    const startX = Math.floor(topLeft.x / minorStep) * minorStep;
    const endX = Math.ceil(bottomRight.x / minorStep) * minorStep;
    const startY = Math.floor(bottomRight.y / minorStep) * minorStep;
    const endY = Math.ceil(topLeft.y / minorStep) * minorStep;
    context.lineWidth = 1;
    for (let x = startX; x <= endX; x += minorStep) {
      const screenX = this.screen({ x, y: 0 }).x;
      context.strokeStyle = x % 50 === 0 ? COLORS.gridMajor : COLORS.gridMinor;
      context.beginPath();
      context.moveTo(screenX, 0);
      context.lineTo(screenX, this.height);
      context.stroke();
    }
    for (let y = startY; y <= endY; y += minorStep) {
      const screenY = this.screen({ x: 0, y }).y;
      context.strokeStyle = y % 50 === 0 ? COLORS.gridMajor : COLORS.gridMinor;
      context.beginPath();
      context.moveTo(0, screenY);
      context.lineTo(this.width, screenY);
      context.stroke();
    }
  }

  private drawGround(state: SimulationState): void {
    const context = this.context;
    const [first, second] = state.ground.pivotPoints;
    if (!first || !second) return;
    const tangent = rotate({ x: 1, y: 0 }, state.ground.angle);
    const normal = rotate({ x: 0, y: 1 }, state.ground.angle);
    const scalar = (point: Vec2): number => dot(subtract(point, state.ground.surfacePoint), tangent);
    const minimum = Math.min(0, scalar(first), scalar(second)) - 14;
    const maximum = Math.max(0, scalar(first), scalar(second)) + 14;
    const planeStart = add(state.ground.surfacePoint, scale(tangent, minimum));
    const planeEnd = add(state.ground.surfacePoint, scale(tangent, maximum));
    const a = this.screen(planeStart);
    const b = this.screen(planeEnd);
    context.strokeStyle = '#4e5d67';
    context.lineWidth = 4;
    context.beginPath();
    context.moveTo(a.x, a.y);
    context.lineTo(b.x, b.y);
    context.stroke();
    for (const pivot of [first, second]) {
      const height = dot(subtract(pivot, state.ground.surfacePoint), normal);
      const support = subtract(pivot, scale(normal, height));
      const pivotScreen = this.screen(pivot);
      const supportScreen = this.screen(support);
      context.strokeStyle = '#647680';
      context.lineWidth = 5;
      context.beginPath();
      context.moveTo(pivotScreen.x, pivotScreen.y);
      context.lineTo(supportScreen.x, supportScreen.y);
      context.stroke();
    }
    context.lineWidth = 1;
    const hatchCount = Math.max(1, Math.floor((maximum - minimum) / 8));
    for (let index = 0; index <= hatchCount; index += 1) {
      const along = minimum + ((maximum - minimum) * index) / hatchCount;
      const hatchStart = add(state.ground.surfacePoint, scale(tangent, along));
      const hatchEnd = add(add(hatchStart, scale(tangent, -3)), scale(normal, -5));
      const hatchStartScreen = this.screen(hatchStart);
      const hatchEndScreen = this.screen(hatchEnd);
      context.beginPath();
      context.moveTo(hatchStartScreen.x, hatchStartScreen.y);
      context.lineTo(hatchEndScreen.x, hatchEndScreen.y);
      context.stroke();
    }
  }

  private drawHand(state: SimulationState): void {
    const context = this.context;
    const hand = state.hand;
    const landmarks = fingerLandmarks(hand);
    const scaleFactor = hand.sizeScale;
    const mcp = this.screen(hand.mcpPosition);
    const palmBack = this.screen({ x: hand.mcpPosition.x - hand.palmLength * scaleFactor * 0.72, y: hand.mcpPosition.y - 9 * scaleFactor });
    const palmLow = this.screen({ x: hand.mcpPosition.x - hand.palmLength * scaleFactor * 0.48, y: hand.mcpPosition.y - hand.palmWidth * scaleFactor });
    const palmFront = this.screen({ x: hand.mcpPosition.x + 7 * scaleFactor, y: hand.mcpPosition.y - hand.palmWidth * scaleFactor * 0.7 });
    context.fillStyle = COLORS.handFill;
    context.strokeStyle = '#a88b7d';
    context.lineWidth = 1.5;
    context.beginPath();
    context.moveTo(mcp.x, mcp.y + 5 * this.camera.zoom);
    context.bezierCurveTo(palmFront.x, palmFront.y, palmLow.x + 30, palmLow.y, palmLow.x, palmLow.y);
    context.lineTo(palmBack.x, palmBack.y);
    context.bezierCurveTo(palmBack.x - 8, palmBack.y - 14, mcp.x - 25, mcp.y - 12, mcp.x, mcp.y - 4);
    context.closePath();
    context.fill();
    context.stroke();

    const metacarpalStart = this.screen({
      x: hand.mcpPosition.x - 58 * scaleFactor,
      y: hand.mcpPosition.y - 7 * scaleFactor,
    });
    context.strokeStyle = COLORS.handBone;
    context.lineCap = 'round';
    context.lineWidth = Math.max(3, 7 * scaleFactor * this.camera.zoom);
    context.beginPath();
    context.moveTo(metacarpalStart.x, metacarpalStart.y);
    context.lineTo(mcp.x, mcp.y);
    context.stroke();

    const points = [landmarks.mcp, landmarks.pip, landmarks.dip, landmarks.tip];
    for (let index = 0; index < 3; index += 1) {
      const segment = hand.segments[index] as FingerSegment;
      const start = this.screen(points[index] as Vec2);
      const end = this.screen(points[index + 1] as Vec2);
      context.lineCap = 'round';
      context.strokeStyle = COLORS.handFill;
      context.lineWidth = Math.max(5, segment.baseWidth * hand.sizeScale * this.camera.zoom);
      context.beginPath();
      context.moveTo(start.x, start.y);
      context.lineTo(end.x, end.y);
      context.stroke();
      context.strokeStyle = COLORS.handBone;
      context.lineWidth = Math.max(2, (segment.baseWidth * hand.sizeScale - 5) * this.camera.zoom);
      context.stroke();
    }
    context.lineCap = 'butt';
    for (const point of points.slice(0, 3)) {
      const joint = this.screen(point as Vec2);
      context.fillStyle = COLORS.handJoint;
      context.beginPath();
      context.arc(joint.x, joint.y, Math.max(3, 2.2 * this.camera.zoom), 0, Math.PI * 2);
      context.fill();
    }
  }

  private drawLinks(state: SimulationState, selection: Selection): void {
    const context = this.context;
    for (const link of state.links) {
      const center = this.screen(link.pose.position);
      const selected = (selection?.kind === 'link' || selection?.kind === 'link-end') && selection.id === link.id;
      context.save();
      context.translate(center.x, center.y);
      context.rotate(-link.pose.angle);
      const length = link.length * this.camera.zoom;
      const width = link.width * this.camera.zoom;
      context.fillStyle = link.fixed ? COLORS.fixedLink : COLORS.link;
      context.strokeStyle = selected ? COLORS.selected : link.fixed ? COLORS.fixedLinkEdge : COLORS.linkEdge;
      context.lineWidth = selected ? 3 : 1.2;
      this.roundRect(context, -length / 2, -width / 2, length, width, Math.min(width / 2, 6));
      context.fill();
      context.stroke();
      context.restore();
    }
  }

  private drawJoints(state: SimulationState, selection: Selection): void {
    for (const joint of state.joints) {
      const linkB = state.links.find((link) => link.id === joint.linkBId);
      if (!linkB) continue;
      const point = joint.groundPoint ?? localToWorld(joint.localPointB, linkB.pose);
      const selected = selection?.kind === 'joint' && selection.id === joint.id;
      this.drawJoint(point, selected ? COLORS.selected : COLORS.joint, 3.2);
    }
  }

  private drawServo(state: SimulationState, selection: Selection): void {
    const context = this.context;
    const point = this.screen(state.servo.groundPoint);
    const selected = selection?.kind === 'servo' && selection.id === state.servo.id;
    const radius = 8 * this.camera.zoom;
    context.fillStyle = '#26343c';
    context.strokeStyle = selected ? COLORS.selected : COLORS.servo;
    context.lineWidth = selected ? 3 : 2;
    context.beginPath();
    context.arc(point.x, point.y, radius, 0, Math.PI * 2);
    context.fill();
    context.stroke();
    context.strokeStyle = COLORS.servo;
    context.beginPath();
    context.moveTo(point.x, point.y);
    context.lineTo(point.x + Math.cos(-state.servo.angle) * radius, point.y + Math.sin(-state.servo.angle) * radius);
    context.stroke();
  }

  private drawContactors(state: SimulationState, selection: Selection): void {
    const context = this.context;
    for (const contactor of state.contactors) {
      const frame = fingerSegmentFrame(state.hand, contactor.fingerSegment);
      const bandRadius = contactor.ringWidth / 2;
      const halfPad = contactor.padLength / 2;
      const center = contactor.fingerPoint;
      const dorsalLeft = add(add(center, scale(frame.tangent, -halfPad)), scale(frame.dorsalNormal, bandRadius));
      const dorsalRight = add(add(center, scale(frame.tangent, halfPad)), scale(frame.dorsalNormal, bandRadius));
      const flexorLeft = add(add(center, scale(frame.tangent, -halfPad)), scale(frame.dorsalNormal, -bandRadius));
      const flexorRight = add(add(center, scale(frame.tangent, halfPad)), scale(frame.dorsalNormal, -bandRadius));
      const flexorControl = add(center, scale(frame.dorsalNormal, -(bandRadius + 3)));
      const linkage = this.screen(contactor.linkagePoint);
      const dorsalSurface = this.screen(add(center, scale(frame.dorsalNormal, frame.width / 2)));
      const selected = selection?.kind === 'contactor' && selection.id === contactor.id;
      if (state.showConstruction) {
        context.setLineDash([4, 4]);
        context.strokeStyle = 'rgba(214, 239, 115, 0.7)';
        context.lineWidth = 1;
        context.beginPath();
        context.moveTo(linkage.x, linkage.y);
        context.lineTo(dorsalSurface.x, dorsalSurface.y);
        context.stroke();
        context.setLineDash([]);
      }

      const dLeft = this.screen(dorsalLeft);
      const dRight = this.screen(dorsalRight);
      const fLeft = this.screen(flexorLeft);
      const fRight = this.screen(flexorRight);
      const fControl = this.screen(flexorControl);
      context.strokeStyle = selected ? COLORS.selected : COLORS.contact;
      context.lineWidth = selected ? 4 : 2.2;
      context.lineJoin = 'round';
      context.lineCap = 'round';
      context.beginPath();
      context.moveTo(dLeft.x, dLeft.y);
      context.lineTo(fLeft.x, fLeft.y);
      context.quadraticCurveTo(fControl.x, fControl.y, fRight.x, fRight.y);
      context.lineTo(dRight.x, dRight.y);
      context.stroke();

      context.save();
      context.translate(linkage.x, linkage.y);
      context.rotate(-Math.atan2(frame.tangent.y, frame.tangent.x));
      const padLength = contactor.padLength * this.camera.zoom;
      const padThickness = contactor.padThickness * this.camera.zoom;
      context.fillStyle = COLORS.contact;
      context.strokeStyle = selected ? COLORS.selected : '#f1ffae';
      context.lineWidth = selected ? 2.5 : 1;
      this.roundRect(context, -padLength / 2, -padThickness / 2, padLength, padThickness, padThickness / 2);
      context.fill();
      context.stroke();
      context.restore();
      context.lineCap = 'butt';
    }
  }

  private drawLinkEndpoints(state: SimulationState, selection: Selection): void {
    const context = this.context;
    for (const link of state.links) {
      if (link.fixed) continue;
      const endpoint = this.screen(localToWorld({ x: link.length / 2, y: 0 }, link.pose));
      const selected = selection?.kind === 'link-end' && selection.id === link.id;
      context.fillStyle = COLORS.background;
      context.strokeStyle = selected ? COLORS.selected : '#80d7e5';
      context.lineWidth = selected ? 3 : 1.5;
      context.beginPath();
      context.arc(endpoint.x, endpoint.y, selected ? 7 : 5, 0, Math.PI * 2);
      context.fill();
      context.stroke();
      context.fillStyle = selected ? COLORS.selected : '#80d7e5';
      context.beginPath();
      context.arc(endpoint.x, endpoint.y, 1.8, 0, Math.PI * 2);
      context.fill();
    }
  }

  private drawConstruction(state: SimulationState): void {
    const context = this.context;
    context.setLineDash([5, 5]);
    context.strokeStyle = COLORS.construction;
    context.lineWidth = 1;
    for (const step of state.analyticSolveSteps) {
      for (const [center, radius] of [
        [step.centerA, step.radiusA] as const,
        [step.centerB, step.radiusB] as const,
      ]) {
        const screenCenter = this.screen(center);
        context.beginPath();
        context.arc(screenCenter.x, screenCenter.y, radius * this.camera.zoom, 0, Math.PI * 2);
        context.stroke();
      }
      if (step.selectedPoint) {
        const selected = this.screen(step.selectedPoint);
        context.fillStyle = COLORS.construction;
        context.beginPath();
        context.arc(selected.x, selected.y, 3, 0, Math.PI * 2);
        context.fill();
      }
    }
    for (const link of state.links) {
      if (link.fixed) continue;
      const center = localToWorld({ x: -link.length / 2, y: 0 }, link.pose);
      const screenCenter = this.screen(center);
      context.beginPath();
      context.arc(screenCenter.x, screenCenter.y, link.length * this.camera.zoom, 0, Math.PI * 2);
      context.stroke();
    }
    context.setLineDash([]);
  }

  private drawJoint(point: Vec2, color: string, radiusWorld: number): void {
    const context = this.context;
    const screen = this.screen(point);
    context.fillStyle = color;
    context.beginPath();
    context.arc(screen.x, screen.y, Math.max(4, radiusWorld * this.camera.zoom), 0, Math.PI * 2);
    context.fill();
    context.fillStyle = COLORS.jointCore;
    context.beginPath();
    context.arc(screen.x, screen.y, Math.max(1.8, radiusWorld * this.camera.zoom * 0.38), 0, Math.PI * 2);
    context.fill();
  }

  private drawInvalidOverlay(message: string): void {
    const context = this.context;
    context.fillStyle = 'rgba(12, 17, 23, 0.75)';
    context.fillRect(16, this.height - 52, Math.min(420, this.width - 32), 36);
    context.fillStyle = COLORS.invalid;
    context.font = '600 13px Inter, system-ui, sans-serif';
    context.fillText(message, 28, this.height - 29);
  }

  private roundRect(context: CanvasRenderingContext2D, x: number, y: number, width: number, height: number, radius: number): void {
    context.beginPath();
    context.roundRect(x, y, width, height, radius);
  }
}
