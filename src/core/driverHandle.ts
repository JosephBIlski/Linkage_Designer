/**
 * Draggable handles for drivers: where the handle sits in a given pose and how
 * a dragged point maps back to the driver's value. The handle is the point the
 * driver moves (angle: the link tip; fold: the moving panel's point farthest
 * from the hinge; slide: the sliding edge's first point); dragging it in the
 * plane perpendicular to the axis (or along the axis for a slide) reads the
 * value with exactly the formula the driver's measurement uses, so the solver
 * reproduces the pointer position.
 */
import { cross, dist, dot, normalize, scale, sub } from './geometry';
import { jointAxisPoints, offAxisPoint } from './model';
import type { Driver, DriverKind, ID, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

export interface DriverHandle {
  index: number;
  kind: DriverKind;
  /** Where the handle is drawn (the driven point in the given pose). */
  pos: Vec3;
  /** Foot of the handle on the axis (angle: pivot; fold: foot of the handle on the hinge; slide: axis start). */
  pivot: Vec3;
  /** Unit axis (rotation axis, or slide direction). */
  axis: Vec3;
  /** Point whose direction from the axis is the zero of the angle (angle: pivot + ref; fold: the fixed panel's point). */
  zero: Vec3;
}

type Positions = Map<ID, Vec3>;

export function driverHandle(m: Model, d: Driver, index: number, pos?: Positions): DriverHandle | null {
  const P = (id: ID): Vec3 | undefined => pos?.get(id) ?? m.points[id]?.pos;
  if (d.kind === 'angle') {
    if (!d.pivotId || !d.tipId || !d.axis || !d.ref) return null;
    const pivot = P(d.pivotId);
    const tip = P(d.tipId);
    if (!pivot || !tip) return null;
    const n = normalize(d.axis);
    const u = normalize(sub(d.ref, scale(n, dot(d.ref, n))));
    return { index, kind: 'angle', pos: tip, pivot, axis: n, zero: [pivot[0] + u[0], pivot[1] + u[1], pivot[2] + u[2]] };
  }
  const j = d.jointId ? m.joints[d.jointId] : null;
  if (!j) return null;
  const axA = jointAxisPoints(m, j, 'a');
  if (!axA) return null;
  const a0 = P(axA[0]);
  const a1 = P(axA[1]);
  if (!a0 || !a1) return null;
  const e = normalize(sub(a1, a0));
  if (d.kind === 'fold') {
    if (isConstructionRef(j.b)) return null;
    const axB = jointAxisPoints(m, j, 'b');
    if (!axB) return null;
    const offA = offAxisPoint(m, m.links[j.a.linkId], axA);
    const offB = offAxisPoint(m, m.links[j.b.linkId], axB);
    if (!offA || !offB) return null;
    const pa = P(offA);
    const pb = P(offB);
    if (!pa || !pb) return null;
    const foot = (p: Vec3): Vec3 => {
      const t = dot(sub(p, a0), e);
      return [a0[0] + e[0] * t, a0[1] + e[1] * t, a0[2] + e[2] * t];
    };
    return { index, kind: 'fold', pos: pb, pivot: foot(pb), axis: e, zero: pa };
  }
  if (d.kind === 'slide') {
    if (isConstructionRef(j.b)) {
      const c = m.construction[j.b.constructionId];
      if (!c?.dir) return null;
      return { index, kind: 'slide', pos: a0, pivot: c.origin, axis: normalize(c.dir), zero: c.origin };
    }
    const axB = jointAxisPoints(m, j, 'b');
    if (!axB) return null;
    const b0 = P(axB[0]);
    if (!b0) return null;
    return { index, kind: 'slide', pos: b0, pivot: a0, axis: e, zero: a0 };
  }
  return null;
}

/** Driver value (degrees for angle / fold, length units for slide) that puts the handle at `p`. */
export function driverValueFromPoint(h: DriverHandle, p: Vec3): number {
  if (h.kind === 'slide') return dot(sub(p, h.zero), h.axis);
  const e = h.axis;
  // signed angle about e from the zero direction to the dragged direction (both taken perpendicular to e),
  // which is currentAngleDeg for angle drivers and dihedralAngle(a0, a1, zero, p) for fold drivers
  const n1 = cross(e, sub(h.zero, h.pivot));
  const n2 = cross(e, sub(p, h.pivot));
  return (Math.atan2(dot(cross(n1, n2), e), dot(n1, n2)) * 180) / Math.PI;
}

/** Radius of the rotation hint ring (distance from the handle to the axis); 0 for slides. */
export function handleRadius(h: DriverHandle): number {
  return h.kind === 'slide' ? 0 : dist(h.pos, h.pivot);
}
