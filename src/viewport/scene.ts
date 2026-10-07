/**
 * Three.js viewport: Z-up CAD-style camera, orbit/pan/zoom controls, grid,
 * lighting, picking and screen/world conversions. Rendering of model content is
 * done by render.ts; this file owns the scene graph containers.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer } from 'three/addons/renderers/CSS2DRenderer.js';
import type { ID, Vec3 } from '../core/types';
import type { AppSettings } from '../ui/settings';

export type PickType = 'vertex' | 'edge' | 'face' | 'axis' | 'joint' | 'construction' | 'editPoint';

export interface PickResult {
  type: PickType;
  /** Link id for vertex/edge/face/axis; joint id; construction id; point id for editPoint. */
  id: ID;
  linkId?: ID;
  pointIds?: ID[];
  pointId?: ID;
  faceIndex?: number;
  pose?: number;
  point: Vec3;
  distance: number;
}

export interface ViewportPointerEvent {
  kind: 'down' | 'move' | 'up' | 'dblclick' | 'leave';
  button: number;
  clientX: number;
  clientY: number;
  shiftKey: boolean;
  ctrlKey: boolean;
  altKey: boolean;
  pick: PickResult | null;
  original: PointerEvent | MouseEvent;
}

const PICK_PRIORITY: Record<PickType, number> = {
  editPoint: 0,
  vertex: 1,
  joint: 2,
  edge: 3,
  axis: 4,
  face: 5,
  construction: 6,
};

export class Viewport {
  readonly container: HTMLElement;
  readonly renderer: THREE.WebGLRenderer;
  readonly labelRenderer: CSS2DRenderer;
  readonly scene = new THREE.Scene();
  camera: THREE.PerspectiveCamera | THREE.OrthographicCamera;
  controls: OrbitControls;
  private perspective: THREE.PerspectiveCamera;
  private orthographic: THREE.OrthographicCamera;

  /** Scene-graph containers, cleared and rebuilt by render.ts */
  readonly groups = {
    construction: new THREE.Group(),
    model: new THREE.Group(),
    ghost: new THREE.Group(),
    joints: new THREE.Group(),
    paths: new THREE.Group(),
    designSpace: new THREE.Group(),
    editPoints: new THREE.Group(),
    overlay: new THREE.Group(),
    labels: new THREE.Group(),
  };
  private grid: THREE.Group | null = null;
  private raycaster = new THREE.Raycaster();
  private pickables: THREE.Object3D[] = [];
  onPointer: ((ev: ViewportPointerEvent) => void) | null = null;
  onBeforeRender: (() => void) | null = null;
  private pickRadiusPx = 10;
  private disposed = false;

  constructor(container: HTMLElement, settings: AppSettings) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.domElement.className = 'viewport-canvas';
    container.appendChild(this.renderer.domElement);
    this.labelRenderer = new CSS2DRenderer();
    this.labelRenderer.domElement.className = 'viewport-labels';
    container.appendChild(this.labelRenderer.domElement);

    const aspect = Math.max(container.clientWidth, 1) / Math.max(container.clientHeight, 1);
    this.perspective = new THREE.PerspectiveCamera(42, aspect, 0.01, 2000);
    this.perspective.up.set(0, 0, 1);
    this.perspective.position.set(7, -11, 8);
    this.orthographic = new THREE.OrthographicCamera(-10 * aspect, 10 * aspect, 10, -10, -500, 500);
    this.orthographic.up.set(0, 0, 1);
    this.orthographic.position.copy(this.perspective.position);
    this.camera = settings.orthographic ? this.orthographic : this.perspective;
    this.controls = this.makeControls(this.camera);

    for (const g of Object.values(this.groups)) this.scene.add(g);
    this.setupLights();
    this.applySettings(settings);

    const dom = this.renderer.domElement;
    const forward = (kind: ViewportPointerEvent['kind']) => (e: PointerEvent | MouseEvent) => {
      if (!this.onPointer) return;
      const pick = kind === 'leave' ? null : this.pick(e.clientX, e.clientY);
      this.onPointer({ kind, button: (e as PointerEvent).button ?? 0, clientX: e.clientX, clientY: e.clientY, shiftKey: e.shiftKey, ctrlKey: e.ctrlKey || e.metaKey, altKey: e.altKey, pick, original: e });
    };
    dom.addEventListener('pointerdown', forward('down'));
    dom.addEventListener('pointermove', forward('move'));
    dom.addEventListener('pointerup', forward('up'));
    dom.addEventListener('pointerleave', forward('leave'));
    dom.addEventListener('dblclick', forward('dblclick'));
    dom.addEventListener('contextmenu', (e) => e.preventDefault());

    const ro = new ResizeObserver(() => this.resize());
    ro.observe(container);
    this.resize();
    this.loop();
  }

  private makeControls(camera: THREE.Camera): OrbitControls {
    const c = new OrbitControls(camera, this.renderer.domElement);
    // CAD-style mapping: left button is reserved for tools, right orbits, middle pans
    (c.mouseButtons as unknown as Record<string, number | null>).LEFT = null;
    c.mouseButtons.MIDDLE = THREE.MOUSE.PAN;
    c.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
    c.enableDamping = false;
    c.screenSpacePanning = true;
    c.zoomToCursor = true;
    c.minDistance = 0.1;
    c.maxDistance = 500;
    return c;
  }

  private setupLights(): void {
    const hemi = new THREE.HemisphereLight(0xffffff, 0x8899aa, 0.9);
    this.scene.add(hemi);
    const dir = new THREE.DirectionalLight(0xffffff, 1.4);
    dir.position.set(5, -8, 12);
    this.scene.add(dir);
    const dir2 = new THREE.DirectionalLight(0xffffff, 0.5);
    dir2.position.set(-6, 6, -4);
    this.scene.add(dir2);
  }

  applySettings(settings: AppSettings): void {
    this.scene.background = new THREE.Color(settings.colors.background);
    if (this.grid) this.scene.remove(this.grid);
    this.grid = new THREE.Group();
    const minor = new THREE.GridHelper(40, 80, settings.colors.gridMinor, settings.colors.gridMinor);
    const major = new THREE.GridHelper(40, 8, settings.colors.gridMajor, settings.colors.gridMajor);
    for (const g of [minor, major]) {
      g.rotation.x = Math.PI / 2; // GridHelper lies in XZ; rotate into the XY (TOP) plane
      (g.material as THREE.Material).transparent = true;
      (g.material as THREE.Material).depthWrite = false;
    }
    (minor.material as THREE.Material).opacity = 0.6;
    (major.material as THREE.Material).opacity = 0.9;
    this.grid.add(minor, major);
    this.grid.renderOrder = -10;
    this.scene.add(this.grid);
    if (settings.orthographic !== (this.camera === this.orthographic)) this.setOrthographic(settings.orthographic);
  }

  setOrthographic(on: boolean): void {
    const from = this.camera;
    const to = on ? this.orthographic : this.perspective;
    if (from === to) return;
    to.position.copy(from.position);
    to.up.copy(from.up);
    const target = this.controls.target.clone();
    if (to === this.orthographic) {
      const d = from.position.distanceTo(target);
      const h = d * Math.tan(THREE.MathUtils.degToRad(this.perspective.fov / 2));
      this.setOrthoHalfHeight(h);
    }
    this.controls.dispose();
    this.camera = to;
    this.controls = this.makeControls(to);
    this.controls.target.copy(target);
    this.camera.lookAt(target);
    this.controls.update();
  }

  private setOrthoHalfHeight(h: number): void {
    const aspect = Math.max(this.container.clientWidth, 1) / Math.max(this.container.clientHeight, 1);
    this.orthographic.top = h;
    this.orthographic.bottom = -h;
    this.orthographic.left = -h * aspect;
    this.orthographic.right = h * aspect;
    this.orthographic.updateProjectionMatrix();
  }

  resize(): void {
    const w = Math.max(this.container.clientWidth, 1);
    const h = Math.max(this.container.clientHeight, 1);
    this.renderer.setSize(w, h, false);
    this.labelRenderer.setSize(w, h);
    const aspect = w / h;
    this.perspective.aspect = aspect;
    this.perspective.updateProjectionMatrix();
    this.setOrthoHalfHeight(this.orthographic.top);
  }

  private loop = (): void => {
    if (this.disposed) return;
    requestAnimationFrame(this.loop);
    this.onBeforeRender?.();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.labelRenderer.render(this.scene, this.camera);
  };

  dispose(): void {
    this.disposed = true;
    this.controls.dispose();
    this.renderer.dispose();
  }

  // ---------------------------------------------------------------------------
  // Views
  // ---------------------------------------------------------------------------

  setView(view: 'top' | 'front' | 'right' | 'iso'): void {
    const target = this.controls.target.clone();
    const d = this.camera.position.distanceTo(target) || 15;
    const dirs: Record<string, THREE.Vector3> = {
      top: new THREE.Vector3(0, 0, 1),
      front: new THREE.Vector3(0, -1, 0),
      right: new THREE.Vector3(1, 0, 0),
      iso: new THREE.Vector3(1, -1, 0.9).normalize(),
    };
    const dir = dirs[view];
    this.camera.position.copy(target.clone().add(dir.multiplyScalar(d)));
    // keep Z up except for the top view where Y is "up" on screen
    this.camera.up.set(0, 0, 1);
    if (view === 'top') this.camera.up.set(0, 1, 0);
    this.camera.lookAt(target);
    this.controls.update();
  }

  /** Frame the given world-space points. */
  fit(points: Vec3[]): void {
    if (points.length === 0) {
      this.controls.target.set(0, 0, 0);
      this.setView('iso');
      return;
    }
    const box = new THREE.Box3();
    for (const p of points) box.expandByPoint(new THREE.Vector3(p[0], p[1], p[2]));
    const center = box.getCenter(new THREE.Vector3());
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 1);
    const dir = this.camera.position.clone().sub(this.controls.target).normalize();
    if (dir.lengthSq() < 0.5) dir.set(1, -1, 0.9).normalize();
    const dist = this.camera === this.perspective ? (size * 0.7) / Math.tan(THREE.MathUtils.degToRad(this.perspective.fov / 2)) : size;
    this.controls.target.copy(center);
    this.camera.position.copy(center.clone().add(dir.multiplyScalar(dist)));
    if (this.camera === this.orthographic) this.setOrthoHalfHeight(size * 0.7);
    this.camera.lookAt(center);
    this.controls.update();
  }

  // ---------------------------------------------------------------------------
  // Picking & projection
  // ---------------------------------------------------------------------------

  setPickables(objects: THREE.Object3D[]): void {
    this.pickables = objects;
  }

  private ndc(clientX: number, clientY: number): THREE.Vector2 {
    const rect = this.renderer.domElement.getBoundingClientRect();
    return new THREE.Vector2(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
  }

  /**
   * Pick under the pointer with a screen-space tolerance: bars and polygon
   * edges can be only 1–2 px wide at the default zoom, so when the exact ray
   * misses model geometry a ring of rays around the pointer is sampled and the
   * best model feature found is returned. Datum geometry never wins over model
   * geometry.
   */
  pick(clientX: number, clientY: number): PickResult | null {
    if (this.pickables.length === 0) return null;
    const exact = this.castPick(clientX, clientY);
    // an exact hit on a point-like feature (editing point / vertex / joint) wins outright;
    // an exact hit on a body is still compared with nearby point-like features in the ring
    if (exact && exact.type !== 'construction' && PICK_PRIORITY[exact.type] <= PICK_PRIORITY.joint) return exact;
    for (const rad of [this.pickRadiusPx * 0.5, this.pickRadiusPx]) {
      let best: PickResult | null = null;
      for (let k = 0; k < 8; k++) {
        const a = (k / 8) * Math.PI * 2;
        const r = this.castPick(clientX + rad * Math.cos(a), clientY + rad * Math.sin(a));
        if (!r || r.type === 'construction') continue;
        if (!best || PICK_PRIORITY[r.type] < PICK_PRIORITY[best.type]) best = r;
      }
      if (best && (!exact || exact.type === 'construction' || PICK_PRIORITY[best.type] < PICK_PRIORITY[exact.type])) return best;
      if (exact && exact.type !== 'construction') return exact;
    }
    return exact;
  }

  private castPick(clientX: number, clientY: number): PickResult | null {
    this.raycaster.setFromCamera(this.ndc(clientX, clientY), this.camera);
    const hits = this.raycaster.intersectObjects(this.pickables, false);
    if (hits.length === 0) return null;
    const results: PickResult[] = [];
    for (const h of hits) {
      const ud = h.object.userData as Partial<PickResult>;
      if (!ud.type) continue;
      results.push({ ...(ud as PickResult), point: [h.point.x, h.point.y, h.point.z], distance: h.distance });
    }
    if (results.length === 0) return null;
    // datum planes and the invisible pick cylinders of datum axes never occlude model geometry
    const model = results.filter((r) => r.type !== 'construction');
    if (model.length === 0) {
      // only datum geometry under the pointer: points beat axes beat planes (datums have no occlusion semantics)
      results.sort((a, b) => ((a as PickResult & { sub?: number }).sub ?? 9) - ((b as PickResult & { sub?: number }).sub ?? 9) || a.distance - b.distance);
      return results[0];
    }
    const candidates = model;
    candidates.sort((a, b) => a.distance - b.distance);
    const closest = candidates[0];
    const tol = this.worldPerPixel(closest.point) * this.pickRadiusPx * 2;
    let best = closest;
    for (const r of candidates) {
      if (r.distance > closest.distance + tol) break;
      if (PICK_PRIORITY[r.type] < PICK_PRIORITY[best.type]) best = r;
    }
    return best;
  }

  /** Approximate world-space size of one pixel at the given world point. */
  worldPerPixel(at: Vec3): number {
    const h = Math.max(this.container.clientHeight, 1);
    if (this.camera === this.orthographic) return (this.orthographic.top - this.orthographic.bottom) / h;
    const d = this.camera.position.distanceTo(new THREE.Vector3(at[0], at[1], at[2]));
    return (2 * d * Math.tan(THREE.MathUtils.degToRad(this.perspective.fov / 2))) / h;
  }

  /** World-space ray through the pointer position. */
  pointerRay(clientX: number, clientY: number): { o: Vec3; d: Vec3 } {
    this.raycaster.setFromCamera(this.ndc(clientX, clientY), this.camera);
    const o = this.raycaster.ray.origin;
    const d = this.raycaster.ray.direction;
    return { o: [o.x, o.y, o.z], d: [d.x, d.y, d.z] };
  }

  /** Intersect the pointer ray with a world plane. */
  projectToPlane(clientX: number, clientY: number, origin: Vec3, normal: Vec3): Vec3 | null {
    this.raycaster.setFromCamera(this.ndc(clientX, clientY), this.camera);
    const n = new THREE.Vector3(normal[0], normal[1], normal[2]).normalize();
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(n, new THREE.Vector3(origin[0], origin[1], origin[2]));
    const out = new THREE.Vector3();
    const dir = this.raycaster.ray.direction;
    if (Math.abs(dir.dot(n)) < 1e-4) return null; // ray parallel to plane
    const hit = this.raycaster.ray.intersectPlane(plane, out);
    return hit ? [hit.x, hit.y, hit.z] : null;
  }

  /** Intersect with the plane through `origin` facing the camera. */
  projectToViewPlane(clientX: number, clientY: number, origin: Vec3): Vec3 | null {
    const n = this.camera.getWorldDirection(new THREE.Vector3());
    return this.projectToPlane(clientX, clientY, origin, [n.x, n.y, n.z]);
  }

  /** Closest point on a world line (origin, dir) to the pointer ray. */
  projectToLine(clientX: number, clientY: number, origin: Vec3, dir: Vec3): Vec3 {
    this.raycaster.setFromCamera(this.ndc(clientX, clientY), this.camera);
    const o = new THREE.Vector3(...origin);
    const d = new THREE.Vector3(...dir).normalize();
    const ro = this.raycaster.ray.origin;
    const rd = this.raycaster.ray.direction;
    // solve for t minimising distance between o + t d and the ray
    const w0 = o.clone().sub(ro);
    const a = d.dot(d);
    const b = d.dot(rd);
    const c = rd.dot(rd);
    const dd = d.dot(w0);
    const e = rd.dot(w0);
    const den = a * c - b * b;
    const t = Math.abs(den) < 1e-9 ? 0 : (b * e - c * dd) / den;
    const p = o.add(d.multiplyScalar(t));
    return [p.x, p.y, p.z];
  }

  worldToScreen(p: Vec3): { x: number; y: number } {
    const v = new THREE.Vector3(p[0], p[1], p[2]).project(this.camera);
    const rect = this.renderer.domElement.getBoundingClientRect();
    return { x: rect.left + ((v.x + 1) / 2) * rect.width, y: rect.top + ((1 - v.y) / 2) * rect.height };
  }

  viewDirection(): Vec3 {
    const n = this.camera.getWorldDirection(new THREE.Vector3());
    return [n.x, n.y, n.z];
  }

  screenshotDataUrl(): string {
    this.renderer.render(this.scene, this.camera);
    return this.renderer.domElement.toDataURL('image/png');
  }
}
