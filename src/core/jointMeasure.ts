/** Measure fold (dihedral) and slide driver values from a set of positions. */
import { dihedralAngle, dot, normalize, rad2deg, sub } from './geometry';
import { jointAxisPoints, offAxisPoint } from './model';
import type { Driver, ID, Joint, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

/**
 * Signed dihedral angle (degrees) across a revolute joint between two links:
 * the angle about the axis of feature a (a0 → a1) from the panel of link a to
 * the panel of link b, each panel represented by its point farthest from the
 * axis (offAxisPoint, chosen in the construction pose). Two coplanar panels on
 * opposite sides of the axis (an unfolded crease) measure ±180°, and folding
 * brings |angle| toward 0; the sign follows the right-hand rule about a0 → a1,
 * so it depends on the stored orientation of feature a. This is the quantity a
 * fold driver prescribes (compile.ts emits the same dihedral constraint), so
 * the Fold command, the driver measurement and the crease display all agree.
 * Returns null for joints to construction geometry, vertex pins without
 * helpers, or links with no point off the axis (bars along the axis). `pos`
 * overrides the construction pose for the four points.
 */
export function jointDihedralDeg(m: Model, j: Joint, pos?: Map<ID, Vec3>): number | null {
  if (isConstructionRef(j.b)) return null;
  const axA = jointAxisPoints(m, j, 'a');
  const axB = jointAxisPoints(m, j, 'b');
  if (!axA || !axB) return null;
  const linkA = m.links[j.a.linkId];
  const linkB = m.links[j.b.linkId];
  if (!linkA || !linkB) return null;
  const offA = offAxisPoint(m, linkA, axA);
  const offB = offAxisPoint(m, linkB, axB);
  if (!offA || !offB) return null;
  const get = (id: ID): Vec3 => pos?.get(id) ?? m.points[id].pos;
  return rad2deg(dihedralAngle(get(axA[0]), get(axA[1]), get(offA), get(offB)));
}

export function measureJointDriver(m: Model, d: Driver, pos: Map<ID, Vec3>): number | null {
  if (!d.jointId) return null;
  const j = m.joints[d.jointId];
  if (!j) return null;
  const axA = jointAxisPoints(m, j, 'a');
  if (!axA) return null;
  const get = (id: ID): Vec3 => pos.get(id) ?? m.points[id].pos;
  if (d.kind === 'fold') return jointDihedralDeg(m, j, pos);
  if (d.kind === 'slide') {
    if (isConstructionRef(j.b)) {
      const c = m.construction[j.b.constructionId];
      if (!c?.dir) return null;
      return dot(sub(get(axA[0]), c.origin), c.dir);
    }
    const axB = jointAxisPoints(m, j, 'b');
    if (!axB) return null;
    const dir = normalize(sub(get(axA[1]), get(axA[0])));
    return dot(sub(get(axB[0]), get(axA[0])), dir);
  }
  return null;
}
