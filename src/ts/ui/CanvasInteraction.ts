import { add, dot, rotate, scale, subtract, worldToLocal, type Vec2 } from '../geometry';
import { nextComponentId, type AppStore } from '../model';
import type { CanvasRenderer } from '../rendering';
import type { MechanismSimulation } from '../simulation';
import { hitTest, hitTestLinkEnd } from './hitTesting';
import { RadialMenu, type RadialActionId } from './RadialMenu';

export class CanvasInteraction {
  private panning = false;
  private lastPointer: Vec2 = { x: 0, y: 0 };
  private contextWorld: Vec2 = { x: 0, y: 0 };
  private contextAttachmentLinkId: string | null = null;
  private draggingLinkEndId: string | null = null;
  private repositioning: { kind: 'servo' | 'contactor'; id: string } | null = null;
  private readonly radialMenu: RadialMenu;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly shell: HTMLElement,
    menuElement: HTMLElement,
    private readonly store: AppStore,
    private readonly renderer: CanvasRenderer,
    private readonly simulation: MechanismSimulation,
    private readonly onChange: () => void,
  ) {
    this.radialMenu = new RadialMenu(menuElement, (action) => this.handleRadialAction(action));
    this.bindEvents();
  }

  closeMenu(): void {
    this.radialMenu.close();
    this.finishReposition();
  }

  private bindEvents(): void {
    this.canvas.addEventListener('pointerdown', (event) => this.pointerDown(event));
    this.canvas.addEventListener('pointermove', (event) => this.pointerMove(event));
    this.canvas.addEventListener('pointerup', (event) => this.pointerUp(event));
    this.canvas.addEventListener('pointercancel', (event) => this.pointerUp(event));
    this.canvas.addEventListener('wheel', (event) => this.wheel(event), { passive: false });
    this.canvas.addEventListener('contextmenu', (event) => this.openContextMenu(event));
    window.addEventListener('keydown', (event) => {
      if (event.key === 'Escape') {
        this.radialMenu.close();
        this.finishReposition();
      }
      if ((event.key === 'Delete' || event.key === 'Backspace') && !this.isEditingField()) this.deleteSelection();
    });
  }

  private pointerDown(event: PointerEvent): void {
    if (this.radialMenu.isOpen) this.radialMenu.close();
    this.lastPointer = this.canvasPoint(event);
    if (event.button === 0 && this.repositioning) {
      this.updateReposition(this.renderer.screenToWorld(this.lastPointer));
      this.finishReposition();
      this.onChange();
      return;
    }
    if (event.button === 1 || (event.button === 0 && event.shiftKey)) {
      this.panning = true;
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.classList.add('is-panning');
      return;
    }
    if (event.button !== 0) return;
    const world = this.renderer.screenToWorld(this.lastPointer);
    const linkEnd = hitTestLinkEnd(this.store.state, world, 7 / this.renderer.camera.zoom);
    if (linkEnd?.kind === 'link-end') {
      this.store.selection = linkEnd;
      this.draggingLinkEndId = linkEnd.id;
      this.store.state.enabled = false;
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.classList.add('is-dragging-control');
    } else {
      this.store.selection = hitTest(this.store.state, world, 7 / this.renderer.camera.zoom);
    }
    this.onChange();
  }

  private pointerMove(event: PointerEvent): void {
    const point = this.canvasPoint(event);
    if (this.repositioning) {
      this.updateReposition(this.renderer.screenToWorld(point));
      this.onChange();
      return;
    }
    if (this.draggingLinkEndId) {
      this.simulation.solveForLinkEndpoint(
        this.store.state,
        this.draggingLinkEndId,
        this.renderer.screenToWorld(point),
      );
      this.onChange();
      return;
    }
    if (!this.panning) return;
    this.renderer.camera.panByPixels(point.x - this.lastPointer.x, point.y - this.lastPointer.y);
    this.lastPointer = point;
  }

  private pointerUp(event: PointerEvent): void {
    if (this.draggingLinkEndId) {
      this.draggingLinkEndId = null;
      this.canvas.classList.remove('is-dragging-control');
    }
    if (this.panning) {
      this.panning = false;
      this.canvas.classList.remove('is-panning');
    }
    if (this.canvas.hasPointerCapture(event.pointerId)) this.canvas.releasePointerCapture(event.pointerId);
  }

  private wheel(event: WheelEvent): void {
    event.preventDefault();
    const factor = Math.exp(-event.deltaY * 0.0012);
    const point = this.canvasPoint(event);
    const viewport = this.renderer.viewport;
    this.renderer.camera.zoomAt(point, factor, viewport.width, viewport.height);
  }

  private openContextMenu(event: MouseEvent): void {
    event.preventDefault();
    const canvasPoint = this.canvasPoint(event);
    this.contextWorld = this.renderer.screenToWorld(canvasPoint);
    const hit = hitTest(this.store.state, this.contextWorld, 7 / this.renderer.camera.zoom);
    this.contextAttachmentLinkId = this.resolveAttachmentLinkId(hit);
    if (hit) {
      this.store.selection = hit;
      this.onChange();
    }
    const shellBounds = this.shell.getBoundingClientRect();
    const selectedLink = hit?.kind === 'link';
    const canReposition = this.store.selection?.kind === 'servo' || this.store.selection?.kind === 'contactor';
    this.radialMenu.open(event.clientX - shellBounds.left, event.clientY - shellBounds.top, [
      { id: 'reposition', label: 'Reposition', enabled: canReposition },
      { id: 'add-link', label: 'Add link', enabled: this.contextAttachmentLinkId !== null },
      { id: 'add-joint', label: 'Add joint', enabled: selectedLink },
      { id: 'add-contactor', label: 'Add contact', enabled: selectedLink },
      {
        id: 'delete',
        label: 'Delete',
        enabled: this.store.selection !== null &&
          ['link', 'joint', 'contactor'].includes(this.store.selection.kind),
      },
      { id: 'cancel', label: 'Cancel', enabled: true },
    ]);
  }

  private handleRadialAction(action: RadialActionId): void {
    if (action === 'reposition') this.beginReposition();
    else if (action === 'add-link') this.addLink();
    else if (action === 'add-joint') this.addJoint();
    else if (action === 'add-contactor') this.addContactor();
    else if (action === 'delete') this.deleteSelection();
  }

  private addLink(): void {
    if (!this.contextAttachmentLinkId) return;
    const hostLink = this.store.state.links.find((link) => link.id === this.contextAttachmentLinkId);
    if (!hostLink) return;
    const id = nextComponentId('link');
    const jointId = nextComponentId('joint');
    const length = 45;
    this.store.state.links.push({
      id,
      name: 'Attached link',
      length,
      width: 6,
      pose: {
        position: { x: this.contextWorld.x + length / 2, y: this.contextWorld.y },
        angle: 0,
      },
    });
    this.store.state.joints.push({
      id: jointId,
      name: 'Attached link revolute joint',
      linkAId: hostLink.id,
      linkBId: id,
      localPointA: worldToLocal(this.contextWorld, hostLink.pose),
      localPointB: { x: -length / 2, y: 0 },
      minAngle: -Math.PI,
      maxAngle: Math.PI,
    });
    this.simulation.solve(this.store.state);
    if (!this.store.state.valid && this.store.state.message.startsWith('Collision')) {
      this.store.state.links = this.store.state.links.filter((link) => link.id !== id);
      this.store.state.joints = this.store.state.joints.filter((joint) => joint.id !== jointId);
      this.simulation.solve(this.store.state);
      this.store.selection = null;
      this.onChange();
      return;
    }
    this.store.selection = { kind: 'link', id };
    this.onChange();
  }

  private addJoint(): void {
    if (this.store.selection?.kind !== 'link') return;
    const link = this.store.state.links.find((candidate) => candidate.id === this.store.selection?.id);
    if (!link) return;
    const id = nextComponentId('joint');
    this.store.state.joints.push({
      id,
      name: 'Ground revolute joint',
      linkAId: null,
      linkBId: link.id,
      localPointB: worldToLocal(this.contextWorld, link.pose),
      groundPoint: { ...this.contextWorld },
      minAngle: -Math.PI,
      maxAngle: Math.PI,
    });
    this.store.state.ground.pivotPoints.push({ ...this.contextWorld });
    this.store.selection = { kind: 'joint', id };
    this.onChange();
  }

  private addContactor(): void {
    if (this.store.selection?.kind !== 'link') return;
    const link = this.store.state.links.find((candidate) => candidate.id === this.store.selection?.id);
    if (!link) return;
    const id = nextComponentId('contactor');
    this.store.state.contactors.push({
      id,
      name: 'Finger contact band',
      linkId: link.id,
      localPoint: worldToLocal(this.contextWorld, link.pose),
      fingerSegment: 'middle',
      fingerPosition: 0.65,
      padLength: 13,
      padThickness: 3.5,
      ringWidth: 18,
      linkagePoint: { ...this.contextWorld },
      fingerPoint: { ...this.contextWorld },
    });
    this.store.selection = { kind: 'contactor', id };
    this.simulation.solve(this.store.state);
    if (!this.store.state.valid && this.store.state.message.startsWith('Collision')) {
      this.store.state.contactors = this.store.state.contactors.filter((contactor) => contactor.id !== id);
      this.simulation.solve(this.store.state);
      this.store.selection = null;
      this.onChange();
      return;
    }
    this.onChange();
  }

  private deleteSelection(): void {
    const selection = this.store.selection;
    if (!selection) return;
    if (selection.kind === 'link') {
      this.store.state.links = this.store.state.links.filter((link) => link.id !== selection.id || link.fixed);
      this.store.state.joints = this.store.state.joints.filter((joint) => joint.linkAId !== selection.id && joint.linkBId !== selection.id);
      this.store.state.contactors = this.store.state.contactors.filter((contactor) => contactor.linkId !== selection.id);
    } else if (selection.kind === 'joint') {
      this.store.state.joints = this.store.state.joints.filter((joint) => joint.id !== selection.id);
    } else if (selection.kind === 'contactor') {
      this.store.state.contactors = this.store.state.contactors.filter((contactor) => contactor.id !== selection.id);
    } else {
      return;
    }
    this.store.selection = null;
    this.simulation.solve(this.store.state);
    this.onChange();
  }

  private beginReposition(): void {
    const selection = this.store.selection;
    if (!selection || (selection.kind !== 'servo' && selection.kind !== 'contactor')) return;
    this.repositioning = selection;
    this.store.state.enabled = false;
    this.canvas.classList.add('is-repositioning');
    this.onChange();
  }

  private updateReposition(worldPoint: Vec2): void {
    if (!this.repositioning) return;
    if (this.repositioning.kind === 'servo') {
      const ground = this.store.state.ground;
      const tangent = rotate({ x: 1, y: 0 }, ground.angle);
      const normal = rotate({ x: 0, y: 1 }, ground.angle);
      const relative = subtract(worldPoint, ground.surfacePoint);
      ground.servoGroundOffset = Math.max(0, dot(relative, normal));
      ground.surfacePoint = add(ground.surfacePoint, scale(tangent, dot(relative, tangent)));
    } else {
      const contactor = this.store.state.contactors.find((candidate) => candidate.id === this.repositioning?.id);
      const link = contactor
        ? this.store.state.links.find((candidate) => candidate.id === contactor.linkId)
        : undefined;
      if (contactor && link) contactor.localPoint = worldToLocal(worldPoint, link.pose);
    }
    this.simulation.solve(this.store.state);
  }

  private finishReposition(): void {
    this.repositioning = null;
    this.canvas.classList.remove('is-repositioning');
  }

  private canvasPoint(event: MouseEvent | PointerEvent | WheelEvent): Vec2 {
    const bounds = this.canvas.getBoundingClientRect();
    return { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
  }

  private isEditingField(): boolean {
    const active = document.activeElement;
    return active instanceof HTMLInputElement ||
      active instanceof HTMLTextAreaElement ||
      active instanceof HTMLSelectElement;
  }

  private resolveAttachmentLinkId(selection: ReturnType<typeof hitTest>): string | null {
    if (!selection) return null;
    if (selection.kind === 'link') return selection.id;
    if (selection.kind === 'servo') {
      const servo = this.store.state.servo;
      const joint = this.store.state.joints.find((candidate) => candidate.id === servo.revoluteJointId);
      if (!joint) return null;
      return joint.linkBId === servo.drivenLinkId ? joint.linkAId : joint.linkBId;
    }
    if (selection.kind === 'contactor') {
      return this.store.state.contactors.find((contactor) => contactor.id === selection.id)?.linkId ?? null;
    }
    if (selection.kind === 'joint') {
      return this.store.state.joints.find((joint) => joint.id === selection.id)?.linkBId ?? null;
    }
    return null;
  }
}
