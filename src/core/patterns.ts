/**
 * Patterning and shape operations: copy, mirror, linear / polar arrays and
 * polygon extrusion. All operate on a single link; joints are never copied
 * (see docs/SPEC.md §14 for the rationale).
 */
import { add, cross, dot, len, normalize, rotateAbout, scale, sub } from './geometry';
import { addCylinder, addJoint, addLinkFromPoints, bodyPlaneJoint, buildRigidity, linkShapePointIds, P, addPoint, removeJoint, rigidityTouches } from './model';
import type { ID, Link, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

export type Transform = (p: Vec3) => Vec3;

export const translation = (d: Vec3): Transform => (p) => add(p, d);

/** Reflection across the plane through `origin` with unit normal `n`. */
export const mirrorAcrossPlane = (origin: Vec3, n: Vec3): Transform => {
  const nn = normalize(n);
  return (p) => sub(p, scale(nn, 2 * dot(sub(p, origin), nn)));
};

/** Rotation by `angle` (radians) about the axis through `origin` with unit direction `axis`. */
export const rotationAboutAxis = (origin: Vec3, axis: Vec3, angle: number): Transform => {
  const k = normalize(axis);
  return (p) => add(origin, rotateAbout(sub(p, origin), k, angle));
};

/**
 * Duplicate a link under a point transform. Copies geometry, params, colour,
 * flexibility and the sketch-plane constraint (when the copy still lies on
 * that plane). Does not copy joints, ground or lock state.
 */
export function duplicateLink(m: Model, link: Link, t: Transform, name: string): Link | null {
  const pts = link.pointIds.map((id) => t(P(m, id)));
  let copy: Link;
  if (link.kind === 'cylinder') {
    copy = addCylinder(m, pts[0], pts[1], link.params.radius ?? 0.5, { name });
  } else {
    copy = addLinkFromPoints(m, link.kind, pts, link.params, { name });
  }
  copy.flexible = link.flexible;
  copy.stiffness = link.stiffness;
  if (link.color) copy.color = link.color;
  // keep the sketch-plane constraint if the transformed body is still on that plane
  const bp = bodyPlaneJoint(m, link.id);
  if (bp && isConstructionRef(bp.b)) {
    const c = m.construction[bp.b.constructionId];
    if (c?.dir) {
      const onPlane = pts.every((p) => Math.abs(dot(sub(p, c.origin), c.dir!)) < 1e-6);
      if (onPlane) addJoint(m, 'planar', { linkId: copy.id, kind: 'body', pointIds: [...copy.pointIds] }, { constructionId: c.id });
    }
  }
  return copy;
}

/** `count` additional copies spaced by `step` (the original is copy 0). */
export function linearArray(m: Model, link: Link, step: Vec3, count: number, nameFor: (i: number) => string): Link[] {
  const out: Link[] = [];
  for (let i = 1; i <= count; i++) {
    const c = duplicateLink(m, link, translation(scale(step, i)), nameFor(i));
    if (c) out.push(c);
  }
  return out;
}

/** `count` additional copies rotated about an axis; `totalAngle` (radians) is spread over count+1 instances (full circle by default). */
export function polarArray(m: Model, link: Link, origin: Vec3, axis: Vec3, count: number, nameFor: (i: number) => string, totalAngle = 2 * Math.PI): Link[] {
  const out: Link[] = [];
  const instances = count + 1;
  const step = Math.abs(totalAngle - 2 * Math.PI) < 1e-9 ? totalAngle / instances : totalAngle / Math.max(1, count);
  for (let i = 1; i <= count; i++) {
    const c = duplicateLink(m, link, rotationAboutAxis(origin, axis, step * i), nameFor(i));
    if (c) out.push(c);
  }
  return out;
}

/** Unit normal of a planar polygon link (right-handed with respect to its vertex order). */
export function polygonNormal(m: Model, link: Link): Vec3 | null {
  const pts = link.pointIds.map((id) => P(m, id));
  if (pts.length < 3) return null;
  // Newell's method is robust for non-convex polygons
  let n: Vec3 = [0, 0, 0];
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i];
    const b = pts[(i + 1) % pts.length];
    n = add(n, [(a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])]);
  }
  return len(n) > 1e-12 ? normalize(n) : null;
}

/**
 * Extrude a planar polygon into a prism of the given height along its normal
 * (negative height extrudes the other way). The bottom face keeps its point
 * ids, so joints attached to the polygon stay valid; the sketch-plane
 * constraint is removed because the body is now three-dimensional.
 */
export function extrudePolygon(m: Model, link: Link, height: number): boolean {
  if (link.kind !== 'polygon' || link.pointIds.length < 3 || Math.abs(height) < 1e-9) return false;
  const n = polygonNormal(m, link);
  if (!n) return false;
  const bottom = [...link.pointIds];
  const top: ID[] = bottom.map((id, i) => addPoint(m, link.id, add(P(m, id), scale(n, height)), 'vertex', `V${i + bottom.length}`).id);
  link.pointIds = [...bottom, ...top];
  link.kind = 'prism';
  link.params = { ...link.params, sides: bottom.length, height: Math.abs(height) };
  // keep helper attachments, rebuild the shape rigidity
  const helperIds = new Set(link.helperIds);
  const helperRigidity = link.rigidity.filter((r) => [...helperIds].some((h) => rigidityTouches(r, h)));
  link.rigidity = [...buildRigidity(m, linkShapePointIds(m, link)), ...helperRigidity];
  const bp = bodyPlaneJoint(m, link.id);
  if (bp) removeJoint(m, bp.id);
  return true;
}

/** Signed area test used by the sketch tool: true when three points are (nearly) collinear. */
export function collinear(a: Vec3, b: Vec3, c: Vec3, tol = 1e-9): boolean {
  return len(cross(sub(b, a), sub(c, a))) < tol * Math.max(1, len(sub(b, a)) * len(sub(c, a)));
}
