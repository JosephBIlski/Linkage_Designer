/** Measure fold (dihedral) and slide driver values from a set of positions. */
import { dihedralAngle, dot, normalize, rad2deg, sub } from './geometry';
import { jointAxisPoints, offAxisPoint } from './model';
import type { Driver, ID, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

export function measureJointDriver(m: Model, d: Driver, pos: Map<ID, Vec3>): number | null {
  if (!d.jointId) return null;
  const j = m.joints[d.jointId];
  if (!j) return null;
  const axA = jointAxisPoints(m, j, 'a');
  if (!axA) return null;
  const get = (id: ID): Vec3 => pos.get(id) ?? m.points[id].pos;
  if (d.kind === 'fold') {
    if (isConstructionRef(j.b)) return null;
    const axB = jointAxisPoints(m, j, 'b');
    if (!axB) return null;
    const offA = offAxisPoint(m, m.links[j.a.linkId], axA);
    const offB = offAxisPoint(m, m.links[j.b.linkId], axB);
    if (!offA || !offB) return null;
    return rad2deg(dihedralAngle(get(axA[0]), get(axA[1]), get(offA), get(offB)));
  }
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
