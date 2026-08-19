import type { Vec2 } from '../geometry';

export class Camera2D {
  center: Vec2 = { x: -5, y: 35 };
  zoom = 3.2;
  minZoom = 0.8;
  maxZoom = 9;

  worldToScreen(point: Vec2, viewportWidth: number, viewportHeight: number): Vec2 {
    return {
      x: viewportWidth / 2 + (point.x - this.center.x) * this.zoom,
      y: viewportHeight / 2 - (point.y - this.center.y) * this.zoom,
    };
  }

  screenToWorld(point: Vec2, viewportWidth: number, viewportHeight: number): Vec2 {
    return {
      x: this.center.x + (point.x - viewportWidth / 2) / this.zoom,
      y: this.center.y - (point.y - viewportHeight / 2) / this.zoom,
    };
  }

  panByPixels(deltaX: number, deltaY: number): void {
    this.center.x -= deltaX / this.zoom;
    this.center.y += deltaY / this.zoom;
  }

  zoomAt(screenPoint: Vec2, factor: number, viewportWidth: number, viewportHeight: number): void {
    const before = this.screenToWorld(screenPoint, viewportWidth, viewportHeight);
    this.zoom = Math.max(this.minZoom, Math.min(this.maxZoom, this.zoom * factor));
    const after = this.screenToWorld(screenPoint, viewportWidth, viewportHeight);
    this.center.x += before.x - after.x;
    this.center.y += before.y - after.y;
  }
}
