/**
 * Builds three.js objects for the current render state. The scene is rebuilt
 * from scratch on every update (mechanisms are small, this keeps the code simple
 * and avoids stale-state bugs). Shared unit geometries are reused.
 */
import * as THREE from 'three';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import type { Positions } from '../core/kinematics';
import { modelSize } from '../core/kinematics';
import { linkEdges, linkFaces } from '../core/model';
import { creaseDihedralDeg, creaseMV, isCrease } from '../core/fold';
import type { ID, Joint, Link, Model, Vec3 } from '../core/types';
import { add, centroid, cross, dist, len, normalize, scale, sub } from '../core/geometry';
import type { AppSettings } from '../ui/settings';
import { JOINTS, creaseLabel } from '../ui/strings';
import type { PickResult, PickType, Viewport } from './scene';

export interface EditPointView {
  pointId: ID;
  pose: number;
  pos: Vec3;
  constrained: boolean;
  locked: boolean;
}
export interface PathView {
  pointId: ID;
  points: Vec3[];
  closed: boolean;
}
export interface SurfaceView {
  pointId: ID;
  /** rows[iB][iA] of positions; each row is one sweep of the primary driver. */
  rows: Vec3[][];
  closed: boolean;
}
export interface DesignSpaceView {
  pointId: ID;
  dim: number;
  dirs: Vec3[];
  center: Vec3;
  radius: number;
}
export interface OverlayView {
  rubberBand?: { a: Vec3; b: Vec3 } | null;
  circle?: { center: Vec3; normal: Vec3; radius: number } | null;
  snapPoint?: Vec3 | null;
  marker?: Vec3 | null;
  polyline?: Vec3[] | null;
}
export interface SelectionView {
  type: PickType | 'link';
  id: ID;
  pointId?: ID;
  pose?: number;
}
export interface RenderState {
  model: Model;
  positions: Positions;
  settings: AppSettings;
  selection: SelectionView | null;
  hover: PickResult | null;
  paths: PathView[];
  surfaces: SurfaceView[];
  designSpaces: DesignSpaceView[];
  editPoints: EditPointView[];
  ghost: Positions | null;
  overlay: OverlayView;
  /** Points whose paths/design spaces are displayed (drawn with a marker). */
  displayPointIds: ID[];
}

const unitSphere = new THREE.SphereGeometry(1, 20, 14);
const unitCylinder = new THREE.CylinderGeometry(1, 1, 1, 18, 1);
const unitTorus = new THREE.TorusGeometry(1, 0.18, 10, 32);
const unitBox = new THREE.BoxGeometry(1, 1, 1);
const unitOcta = new THREE.OctahedronGeometry(1, 0);
const unitCone = new THREE.ConeGeometry(1, 1, 16);
const Y = new THREE.Vector3(0, 1, 0);
/** Colour of a crease whose panels are coplanar (no mountain / valley class yet). */
const CREASE_FLAT_COLOR = '#8d949c';
/** Crease lines are drawn this much thicker than the panel edges they lie on (edge radius = 0.55 · r). */
const CREASE_RADIUS_FACTOR = 1.6;

function v(p: Vec3): THREE.Vector3 {
  return new THREE.Vector3(p[0], p[1], p[2]);
}

export class ModelRenderer {
  private disposables: (THREE.BufferGeometry | THREE.Material)[] = [];
  private labels: CSS2DObject[] = [];
  private pickables: THREE.Object3D[] = [];
  private r = 0.06;

  constructor(private vp: Viewport) {}

  private track<T extends THREE.BufferGeometry | THREE.Material>(x: T): T {
    this.disposables.push(x);
    return x;
  }

  private clear(): void {
    for (const g of Object.values(this.vp.groups)) {
      while (g.children.length) g.remove(g.children[0]);
    }
    for (const l of this.labels) l.element.remove();
    this.labels = [];
    for (const d of this.disposables) d.dispose();
    this.disposables = [];
    this.pickables = [];
  }

  update(state: RenderState): void {
    this.clear();
    const { model, settings } = state;
    const size = modelSize(model, state.positions);
    this.r = THREE.MathUtils.clamp(size * 0.012, 0.02, 0.12);
    if (settings.showConstruction) this.buildConstruction(state);
    this.buildModel(state, state.positions, this.vp.groups.model, false);
    if (state.ghost) this.buildModel(state, state.ghost, this.vp.groups.ghost, true);
    this.buildJoints(state);
    this.buildPaths(state);
    this.buildSurfaces(state);
    this.buildDesignSpaces(state);
    this.buildEditPoints(state);
    this.buildOverlay(state);
    this.vp.setPickables(this.pickables);
  }

  // ---------------------------------------------------------------------------

  private material(color: string, opts: { opacity?: number; emissive?: string; flat?: boolean } = {}): THREE.Material {
    const m = opts.flat
      ? new THREE.MeshBasicMaterial({ color, transparent: (opts.opacity ?? 1) < 1, opacity: opts.opacity ?? 1, depthWrite: (opts.opacity ?? 1) >= 1 })
      : new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.05, transparent: (opts.opacity ?? 1) < 1, opacity: opts.opacity ?? 1, side: THREE.DoubleSide });
    if (!opts.flat && opts.emissive) (m as THREE.MeshStandardMaterial).emissive = new THREE.Color(opts.emissive);
    if (!opts.flat && opts.emissive) (m as THREE.MeshStandardMaterial).emissiveIntensity = 0.55;
    return this.track(m);
  }

  private isSelected(state: RenderState, type: string, id: ID, pointId?: ID, pose?: number): boolean {
    const s = state.selection;
    if (!s) return false;
    if (type === 'editPoint') return s.type === 'editPoint' && s.pointId === pointId && s.pose === pose;
    if (type === 'vertex') return s.type === 'vertex' && s.pointId === pointId;
    if (type === 'link') return (s.type === 'link' || s.type === 'edge' || s.type === 'face' || s.type === 'axis') && s.id === id;
    return s.type === type && s.id === id;
  }

  private isHovered(state: RenderState, type: string, id: ID, pointId?: ID, pose?: number): boolean {
    const h = state.hover;
    if (!h) return false;
    if (type === 'editPoint') return h.type === 'editPoint' && h.pointId === pointId && h.pose === pose;
    if (type === 'vertex') return h.type === 'vertex' && h.pointId === pointId;
    return h.type === type && h.id === id;
  }

  private sphere(p: Vec3, radius: number, mat: THREE.Material, pick?: Partial<PickResult>): THREE.Mesh {
    const m = new THREE.Mesh(unitSphere, mat);
    m.position.copy(v(p));
    m.scale.setScalar(radius);
    if (pick) {
      m.userData = pick;
      this.pickables.push(m);
    }
    return m;
  }

  private cylinder(a: Vec3, b: Vec3, radius: number, mat: THREE.Material, pick?: Partial<PickResult>): THREE.Mesh {
    const m = new THREE.Mesh(unitCylinder, mat);
    const d = sub(b, a);
    const L = len(d);
    m.position.copy(v(add(a, scale(d, 0.5))));
    m.scale.set(radius, Math.max(L, 1e-6), radius);
    m.quaternion.setFromUnitVectors(Y, v(normalize(L > 1e-9 ? d : [0, 0, 1])));
    if (pick) {
      m.userData = pick;
      this.pickables.push(m);
    }
    return m;
  }

  /** Visible sphere plus an invisible pick proxy of at least ~10 px radius (vertices must beat edge proxies). */
  private pickableSphere(group: THREE.Group, p: Vec3, radius: number, mat: THREE.Material, pick: Partial<PickResult>): void {
    group.add(this.sphere(p, radius, mat));
    const minR = this.vp.worldPerPixel(p) * 5;
    const proxyMat = this.track(new THREE.MeshBasicMaterial({ visible: false }));
    group.add(this.sphere(p, Math.max(radius, minR), proxyMat, pick));
  }

  /**
   * Visible cylinder plus an invisible, wider pick proxy so thin bars and
   * polygon edges are always at least ~8 px wide for picking at any zoom.
   */
  private pickableCylinder(group: THREE.Group, a: Vec3, b: Vec3, radius: number, mat: THREE.Material, pick?: Partial<PickResult>): void {
    group.add(this.cylinder(a, b, radius, mat));
    if (!pick) return;
    const minR = this.vp.worldPerPixel(a) * 4;
    const proxyMat = this.track(new THREE.MeshBasicMaterial({ visible: false }));
    group.add(this.cylinder(a, b, Math.max(radius, minR), proxyMat, pick));
  }

  private label(text: string, at: Vec3, cls = 'label'): void {
    const div = document.createElement('div');
    div.className = cls;
    div.textContent = text;
    const obj = new CSS2DObject(div);
    obj.position.copy(v(at));
    this.labels.push(obj);
    this.vp.groups.labels.add(obj);
  }

  // ---------------------------------------------------------------------------
  // Construction geometry
  // ---------------------------------------------------------------------------

  private buildConstruction(state: RenderState): void {
    const { model, settings } = state;
    const g = this.vp.groups.construction;
    const r = this.r;
    for (const c of Object.values(model.construction)) {
      const selected = this.isSelected(state, 'construction', c.id);
      const hovered = this.isHovered(state, 'construction', c.id);
      const color = selected ? settings.colors.selection : settings.colors.construction;
      const emissive = hovered ? settings.colors.selection : undefined;
      if (c.kind === 'point') {
        const m = new THREE.Mesh(unitOcta, this.material(color, { emissive }));
        m.position.copy(v(c.origin));
        m.scale.setScalar(r * 1.6);
        m.userData = { type: 'construction', id: c.id, sub: 0 } as Partial<PickResult> & { sub: number };
        this.pickables.push(m);
        g.add(m);
        if (settings.showLabels) this.label(c.name, add(c.origin, [0, 0, r * 4]), 'label label--construction');
      } else if (c.kind === 'axis' && c.dir) {
        const half = c.size ?? 10;
        const a = add(c.origin, scale(c.dir, -half));
        const b = add(c.origin, scale(c.dir, half));
        const mat = this.track(new THREE.LineDashedMaterial({ color, dashSize: 0.35, gapSize: 0.18, transparent: true, opacity: c.builtin ? 0.45 : 0.9 }));
        const geo = this.track(new THREE.BufferGeometry().setFromPoints([v(a), v(b)]));
        const line = new THREE.Line(geo, mat);
        line.computeLineDistances();
        g.add(line);
        // invisible pick cylinder
        const pickMat = this.track(new THREE.MeshBasicMaterial({ visible: false }));
        g.add(this.cylinder(a, b, r * 1.2, pickMat, { type: 'construction', id: c.id, sub: 1 } as Partial<PickResult> & { sub: number }));
        if (settings.showLabels) this.label(c.name, add(b, [0, 0, r * 2]), 'label label--construction');
      } else if (c.kind === 'plane' && c.dir) {
        const s = c.size ?? 6;
        const geo = this.track(new THREE.PlaneGeometry(2 * s, 2 * s));
        const mat = this.track(new THREE.MeshBasicMaterial({ color, transparent: true, opacity: c.builtin ? 0.04 : 0.12, side: THREE.DoubleSide, depthWrite: false }));
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.copy(v(c.origin));
        mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), v(c.dir));
        mesh.userData = { type: 'construction', id: c.id, plane: true, sub: 2 } as Partial<PickResult> & { plane: boolean; sub: number };
        mesh.renderOrder = -5;
        this.pickables.push(mesh);
        g.add(mesh);
        const edges = this.track(new THREE.EdgesGeometry(geo));
        const lmat = this.track(new THREE.LineBasicMaterial({ color, transparent: true, opacity: c.builtin ? 0.35 : 0.9 }));
        const border = new THREE.LineSegments(edges, lmat);
        border.position.copy(mesh.position);
        border.quaternion.copy(mesh.quaternion);
        g.add(border);
        if (settings.showLabels) {
          const xd = c.xDir ?? [1, 0, 0];
          const yd = cross(c.dir, xd);
          this.label(c.name, add(c.origin, add(scale(xd, s * 0.92), scale(yd, s * 0.92))), 'label label--construction');
        }
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Links
  // ---------------------------------------------------------------------------

  private linkColor(link: Link, state: RenderState): string {
    if (link.ground) return state.settings.colors.ground;
    return link.color ?? state.settings.colors.geometry;
  }

  private buildModel(state: RenderState, pos: Positions, group: THREE.Group, ghost: boolean): void {
    const { model, settings } = state;
    const r = this.r;
    const P = (id: ID): Vec3 => pos.get(id) ?? model.points[id].pos;
    for (const link of Object.values(model.links)) {
      if (link.hidden) continue;
      const selected = !ghost && this.isSelected(state, 'link', link.id);
      const hovered = !ghost && state.hover !== null && (state.hover.type === 'edge' || state.hover.type === 'face' || state.hover.type === 'axis') && state.hover.id === link.id;
      const baseColor = ghost ? '#888888' : this.linkColor(link, state);
      const opacity = ghost ? 0.28 : link.kind === 'polygon' || link.kind === 'prism' ? 0.85 : 1;
      const bodyMat = this.material(selected ? settings.colors.selection : baseColor, { opacity, emissive: hovered && !selected ? settings.colors.selection : undefined });
      const edgeMat = this.material(selected ? settings.colors.selection : ghost ? '#888888' : darkenHex(baseColor, 0.25), { opacity: ghost ? 0.28 : 1 });
      const pickBase = (type: PickType, pointIds: ID[], faceIndex?: number): Partial<PickResult> | undefined => (ghost ? undefined : { type, id: link.id, linkId: link.id, pointIds, faceIndex });

      if (link.kind === 'bar') {
        const [a, b] = link.pointIds;
        this.pickableCylinder(group, P(a), P(b), r * (link.flexible ? 0.7 : 1), bodyMat, pickBase('edge', [a, b]));
      } else if (link.kind === 'polygon' || link.kind === 'prism') {
        const faces = linkFaces(model, link);
        faces.forEach((face, fi) => {
          const pts = face.map(P);
          const geo = this.track(polygonGeometry(pts));
          const mesh = new THREE.Mesh(geo, bodyMat);
          if (!ghost) {
            mesh.userData = pickBase('face', face, fi)!;
            this.pickables.push(mesh);
          }
          group.add(mesh);
        });
        for (const [a, b] of linkEdges(model, link)) this.pickableCylinder(group, P(a), P(b), r * 0.55, edgeMat, pickBase('edge', [a, b]));
      } else if (link.kind === 'cylinder') {
        const [a, b] = link.pointIds;
        const radius = link.params.radius ?? 0.5;
        group.add(this.cylinder(P(a), P(b), radius, bodyMat, pickBase('axis', [a, b])));
        // axis line
        const lmat = this.track(new THREE.LineBasicMaterial({ color: '#333333', transparent: true, opacity: ghost ? 0.2 : 0.6 }));
        const lgeo = this.track(new THREE.BufferGeometry().setFromPoints([v(P(a)), v(P(b))]));
        group.add(new THREE.Line(lgeo, lmat));
      }

      // vertices
      if (!ghost) {
        for (const pid of link.pointIds) {
          const pt = model.points[pid];
          const vSel = this.isSelected(state, 'vertex', link.id, pid);
          const vHov = this.isHovered(state, 'vertex', link.id, pid);
          const shown = state.displayPointIds.includes(pid);
          const mat = this.material(vSel ? settings.colors.selection : shown ? settings.colors.outputPath : darkenHex(baseColor, 0.45), { emissive: vHov ? settings.colors.selection : undefined });
          this.pickableSphere(group, P(pid), r * (shown ? 1.9 : 1.5), mat, { type: 'vertex', id: link.id, linkId: link.id, pointId: pid, pointIds: [pid] });
          if (settings.showLabels && (link.kind === 'bar' || shown)) this.label(pt.name, add(P(pid), [0, 0, r * 3]), 'label label--vertex');
        }
        if (settings.showHelpers) {
          for (const pid of link.helperIds) {
            const mat = this.material('#ff00ff', { flat: true });
            group.add(this.sphere(P(pid), r * 0.8, mat));
            const lmat = this.track(new THREE.LineBasicMaterial({ color: '#ff00ff' }));
            // connect to its base (first visible point it is nearest to)
            const base = link.pointIds.reduce((best, id) => (dist(P(id), P(pid)) < dist(P(best), P(pid)) ? id : best), link.pointIds[0]);
            const lgeo = this.track(new THREE.BufferGeometry().setFromPoints([v(P(pid)), v(P(base))]));
            group.add(new THREE.Line(lgeo, lmat));
          }
        }
        // ground symbol
        if (link.ground) {
          for (const pid of link.pointIds) {
            const cone = new THREE.Mesh(unitCone, this.material(settings.colors.ground, { opacity: 0.9 }));
            cone.position.copy(v(add(P(pid), [0, 0, -r * 2.6])));
            cone.scale.set(r * 2.2, r * 3.2, r * 2.2);
            cone.quaternion.setFromUnitVectors(Y, new THREE.Vector3(0, 0, 1));
            group.add(cone);
          }
        }
        if (settings.showLabels) {
          const c = centroid(link.pointIds.map(P));
          this.label(link.name + (link.locked ? ' 🔒' : '') + (link.flexible ? ' ~' : ''), add(c, [0, 0, r * 5]), 'label label--link' + (link.ground ? ' label--ground' : ''));
        }
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Joints
  // ---------------------------------------------------------------------------

  private jointPlacement(model: Model, j: Joint, pos: Positions): { at: Vec3; axis: Vec3 } | null {
    const P = (id: ID): Vec3 => pos.get(id) ?? model.points[id].pos;
    const f = j.a;
    if (f.kind === 'body') {
      return null;
    }
    const pts = f.pointIds.map(P);
    const at = centroid(pts);
    let axis: Vec3 = j.axis ?? [0, 0, 1];
    if ((f.kind === 'edge' || f.kind === 'axis') && pts.length >= 2) axis = normalize(sub(pts[1], pts[0]));
    if (f.kind === 'face' && pts.length >= 3) axis = normalize(cross(sub(pts[1], pts[0]), sub(pts[2], pts[0])));
    return { at, axis };
  }

  private buildJoints(state: RenderState): void {
    const { model, settings } = state;
    const g = this.vp.groups.joints;
    const r = this.r;
    for (const j of Object.values(model.joints)) {
      if (j.type === 'planar' && j.a.kind === 'body') continue; // sketch-plane constraints are implicit
      if (model.links[j.a.linkId]?.hidden) continue;
      if (!isConstructionRefLocal(j.b) && model.links[j.b.linkId]?.hidden) continue;
      const selected = this.isSelected(state, 'joint', j.id);
      const hovered = this.isHovered(state, 'joint', j.id);
      if (isCrease(model, j)) {
        this.buildCrease(state, j, selected, hovered);
        continue;
      }
      const pl = this.jointPlacement(model, j, state.positions);
      if (!pl) continue;
      const color = selected ? settings.colors.selection : '#2b2f33';
      const mat = this.material(color, { emissive: hovered ? settings.colors.selection : undefined });
      const pick: Partial<PickResult> = { type: 'joint', id: j.id };
      let mesh: THREE.Mesh;
      const axisQ = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), v(pl.axis));
      switch (j.type) {
        case 'spherical':
          mesh = new THREE.Mesh(unitSphere, mat);
          mesh.scale.setScalar(r * 1.25);
          break;
        case 'revolute':
          mesh = new THREE.Mesh(unitTorus, mat);
          mesh.scale.setScalar(r * 2.1);
          mesh.quaternion.copy(axisQ);
          break;
        case 'cylindrical':
          mesh = new THREE.Mesh(unitCylinder, mat);
          mesh.scale.set(r * 1.6, r * 4, r * 1.6);
          mesh.quaternion.setFromUnitVectors(Y, v(pl.axis));
          break;
        case 'prismatic':
          mesh = new THREE.Mesh(unitBox, mat);
          mesh.scale.set(r * 2.4, r * 2.4, r * 4);
          mesh.quaternion.copy(axisQ);
          break;
        case 'screw':
          mesh = new THREE.Mesh(unitTorus, mat);
          mesh.scale.set(r * 1.6, r * 1.6, r * 4);
          mesh.quaternion.copy(axisQ);
          break;
        default:
          mesh = new THREE.Mesh(unitBox, mat);
          mesh.scale.set(r * 3, r * 3, r * 0.4);
          mesh.quaternion.copy(axisQ);
      }
      mesh.position.copy(v(pl.at));
      mesh.userData = pick;
      this.pickables.push(mesh);
      g.add(mesh);
      if (settings.showLabels && (selected || hovered)) this.label(JOINTS[j.type]?.label ?? j.type, add(pl.at, [0, 0, r * 4]), 'label label--joint');
    }
  }

  /**
   * A crease (edge–edge revolute with merged end points) is drawn as a line
   * along the shared edge, slightly thicker than the panel edges it covers, in
   * the mountain / valley colour of its current dihedral (creaseMV; grey while
   * the panels are coplanar) instead of the torus glyph. The dihedral is
   * measured in the rendered pose, so Preview and Simulation poses are coloured
   * as they fold. It remains pickable as the joint (selection colour when
   * selected, emissive highlight when hovered) and shows "Crease M 160°" as its
   * label when labels are on and it is selected or hovered.
   */
  private buildCrease(state: RenderState, j: Joint, selected: boolean, hovered: boolean): void {
    const { model, settings } = state;
    const P = (id: ID): Vec3 => state.positions.get(id) ?? model.points[id].pos;
    const [a0, a1] = j.a.pointIds.map(P);
    const mv = creaseMV(model, j, state.positions);
    const color = selected ? settings.colors.selection : mv === 'M' ? settings.colors.mountain : mv === 'V' ? settings.colors.valley : CREASE_FLAT_COLOR;
    const mat = this.material(color, { emissive: hovered ? settings.colors.selection : undefined });
    this.pickableCylinder(this.vp.groups.joints, a0, a1, this.r * 0.55 * CREASE_RADIUS_FACTOR, mat, { type: 'joint', id: j.id });
    if (settings.showLabels && (selected || hovered)) this.label(creaseLabel(mv, creaseDihedralDeg(model, j, state.positions)), add(centroid([a0, a1]), [0, 0, this.r * 4]), 'label label--joint');
  }

  // ---------------------------------------------------------------------------
  // Paths, design space, editing points
  // ---------------------------------------------------------------------------

  private buildPaths(state: RenderState): void {
    const g = this.vp.groups.paths;
    const r = this.r;
    const { settings } = state;
    for (const path of state.paths) {
      if (path.points.length < 2) continue;
      const pts = path.points.map(v);
      const curve = new THREE.CatmullRomCurve3(pts, path.closed, 'centripetal');
      const geo = this.track(new THREE.TubeGeometry(curve, Math.max(pts.length * 3, 24), r * 0.32, 8, path.closed));
      const mat = this.track(new THREE.MeshBasicMaterial({ color: settings.colors.outputPath, transparent: settings.outputPathOpacity < 1, opacity: settings.outputPathOpacity, depthWrite: settings.outputPathOpacity >= 1 }));
      g.add(new THREE.Mesh(geo, mat));
    }
  }

  /** Output surfaces (2-DOF mechanisms): a translucent mesh through the grid of swept positions. */
  private buildSurfaces(state: RenderState): void {
    const g = this.vp.groups.paths;
    const { settings } = state;
    for (const surf of state.surfaces) {
      const rows = surf.rows;
      const nA = Math.min(...rows.map((r) => r.length));
      if (nA < 2 || rows.length < 2) continue;
      const positions: number[] = [];
      const indices: number[] = [];
      for (const row of rows) for (let i = 0; i < nA; i++) positions.push(...row[i]);
      for (let b = 0; b + 1 < rows.length; b++) {
        for (let a = 0; a + 1 < nA; a++) {
          const i0 = b * nA + a;
          const i1 = i0 + 1;
          const i2 = i0 + nA;
          const i3 = i2 + 1;
          indices.push(i0, i1, i2, i1, i3, i2);
        }
        if (surf.closed) {
          const i0 = b * nA + nA - 1;
          const i1 = b * nA;
          const i2 = i0 + nA;
          const i3 = i1 + nA;
          indices.push(i0, i1, i2, i1, i3, i2);
        }
      }
      const geo = this.track(new THREE.BufferGeometry());
      geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
      geo.setIndex(indices);
      geo.computeVertexNormals();
      const mat = this.track(new THREE.MeshStandardMaterial({ color: settings.colors.outputPath, transparent: true, opacity: Math.min(0.75, settings.outputPathOpacity), side: THREE.DoubleSide, roughness: 0.7, depthWrite: false }));
      const mesh = new THREE.Mesh(geo, mat);
      mesh.renderOrder = 1;
      g.add(mesh);
      // iso-lines
      const lmat = this.track(new THREE.LineBasicMaterial({ color: settings.colors.outputPath, transparent: true, opacity: 0.5 }));
      for (const row of rows) {
        const lgeo = this.track(new THREE.BufferGeometry().setFromPoints(row.slice(0, nA).map(v)));
        g.add(surf.closed ? new THREE.LineLoop(lgeo, lmat) : new THREE.Line(lgeo, lmat));
      }
    }
  }

  private buildDesignSpaces(state: RenderState): void {
    const g = this.vp.groups.designSpace;
    const { settings } = state;
    const r = this.r;
    for (const ds of state.designSpaces) {
      if (ds.dim === 0) continue;
      const mat = this.track(new THREE.MeshBasicMaterial({ color: settings.colors.designSpace, transparent: true, opacity: settings.designSpaceOpacity, side: THREE.DoubleSide, depthWrite: false }));
      let mesh: THREE.Mesh;
      if (ds.dim >= 3) {
        mesh = new THREE.Mesh(this.track(new THREE.SphereGeometry(ds.radius, 40, 28)), mat);
        mesh.position.copy(v(ds.center));
      } else if (ds.dim === 2) {
        mesh = new THREE.Mesh(this.track(new THREE.CircleGeometry(ds.radius, 72)), mat);
        mesh.position.copy(v(ds.center));
        const n = normalize(cross(ds.dirs[0], ds.dirs[1]));
        mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), v(n));
        // outline
        const edges = this.track(new THREE.EdgesGeometry(mesh.geometry as THREE.BufferGeometry, 1));
        const line = new THREE.LineSegments(edges, this.track(new THREE.LineBasicMaterial({ color: settings.colors.designSpace, transparent: true, opacity: 0.7 })));
        line.position.copy(mesh.position);
        line.quaternion.copy(mesh.quaternion);
        g.add(line);
      } else {
        const a = add(ds.center, scale(ds.dirs[0], -ds.radius));
        const b = add(ds.center, scale(ds.dirs[0], ds.radius));
        mesh = this.cylinder(a, b, r * 0.9, mat);
      }
      mesh.renderOrder = 2;
      g.add(mesh);
    }
  }

  private buildEditPoints(state: RenderState): void {
    const g = this.vp.groups.editPoints;
    const { settings } = state;
    const r = this.r;
    for (const ep of state.editPoints) {
      const selected = this.isSelected(state, 'editPoint', ep.pointId, ep.pointId, ep.pose);
      const hovered = this.isHovered(state, 'editPoint', ep.pointId, ep.pointId, ep.pose);
      const color = ep.constrained ? settings.colors.editPointConstrained : settings.colors.editPointFree;
      const mat = this.material(selected ? settings.colors.selection : color, { emissive: hovered ? settings.colors.selection : undefined });
      const s = this.sphere(ep.pos, r * (selected || hovered ? 1.7 : 1.35), mat, { type: 'editPoint', id: ep.pointId, pointId: ep.pointId, pose: ep.pose });
      s.renderOrder = 3;
      g.add(s);
      if (ep.locked) {
        const ring = new THREE.Mesh(unitTorus, this.material('#222222', { flat: true }));
        ring.position.copy(v(ep.pos));
        ring.scale.setScalar(r * 2.0);
        const n = this.vp.viewDirection();
        ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), v(normalize(n)));
        g.add(ring);
      }
    }
  }

  private buildOverlay(state: RenderState): void {
    const g = this.vp.groups.overlay;
    const r = this.r;
    const o = state.overlay;
    const col = state.settings.colors.selection;
    if (o.rubberBand) {
      const mat = this.track(new THREE.LineDashedMaterial({ color: col, dashSize: r * 3, gapSize: r * 1.5 }));
      const geo = this.track(new THREE.BufferGeometry().setFromPoints([v(o.rubberBand.a), v(o.rubberBand.b)]));
      const line = new THREE.Line(geo, mat);
      line.computeLineDistances();
      g.add(line);
      g.add(this.sphere(o.rubberBand.b, r * 1.2, this.material(col, { flat: true })));
    }
    if (o.polyline && o.polyline.length > 1) {
      const mat = this.track(new THREE.LineBasicMaterial({ color: col }));
      const geo = this.track(new THREE.BufferGeometry().setFromPoints(o.polyline.map(v)));
      g.add(new THREE.LineLoop(geo, mat));
    }
    if (o.circle) {
      const pts: THREE.Vector3[] = [];
      const n = normalize(o.circle.normal);
      const u = normalize(cross(n, Math.abs(n[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0]));
      const w = cross(n, u);
      for (let i = 0; i <= 64; i++) {
        const t = (i / 64) * Math.PI * 2;
        pts.push(v(add(o.circle.center, add(scale(u, Math.cos(t) * o.circle.radius), scale(w, Math.sin(t) * o.circle.radius)))));
      }
      const mat = this.track(new THREE.LineDashedMaterial({ color: col, dashSize: r * 3, gapSize: r * 1.5 }));
      const geo = this.track(new THREE.BufferGeometry().setFromPoints(pts));
      const line = new THREE.Line(geo, mat);
      line.computeLineDistances();
      g.add(line);
    }
    if (o.snapPoint) {
      const ring = new THREE.Mesh(unitTorus, this.material(col, { flat: true }));
      ring.position.copy(v(o.snapPoint));
      ring.scale.setScalar(r * 2.6);
      ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), v(normalize(this.vp.viewDirection())));
      g.add(ring);
    }
    if (o.marker) {
      g.add(this.sphere(o.marker, r * 1.1, this.material(col, { flat: true })));
    }
  }
}

function isConstructionRefLocal(x: Joint['b']): x is { constructionId: ID } {
  return (x as { constructionId?: ID }).constructionId !== undefined;
}

/** Triangulated (fan) geometry of a convex planar polygon. */
function polygonGeometry(pts: Vec3[]): THREE.BufferGeometry {
  const positions: number[] = [];
  for (let i = 1; i + 1 < pts.length; i++) {
    positions.push(...pts[0], ...pts[i], ...pts[i + 1]);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.computeVertexNormals();
  return geo;
}

function darkenHex(hex: string, f: number): string {
  const c = new THREE.Color(hex);
  c.multiplyScalar(1 - f);
  return `#${c.getHexString()}`;
}

