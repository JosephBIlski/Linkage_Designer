/**
 * Vertex editing: move a vertex of existing geometry to a new position (the
 * "Edit" tool). The vertex is released from its link's shape constraints, all
 * other constraints are re-solved, and the link's rest geometry is rebuilt.
 */
import { dot, sub } from './geometry';
import { applyPositions, solveSketch } from './kinematics';
import { addJoint, bodyPlaneJoint, buildRigidity, jointsAtPoint, linkShapePointIds, removeJoint, rigidityTouches, sketchNormal } from './model';
import type { ID, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

export interface MoveVertexResult {
  ok: boolean;
  residual: number;
  joinedTo?: ID;
}

/**
 * Move `pointId` to `dest`. When `joinTo` names a vertex of another link, a
 * default joint is created between the two vertices afterwards.
 */
export function moveVertex(m: Model, pointId: ID, dest: Vec3, joinTo?: ID): MoveVertexResult {
  const pt = m.points[pointId];
  if (!pt) return { ok: false, residual: Infinity };
  const link = m.links[pt.linkId];
  if (!link || link.locked) return { ok: false, residual: Infinity };
  // the sketch-plane constraint must go if the destination leaves that plane
  const bp = bodyPlaneJoint(m, link.id);
  if (bp && isConstructionRef(bp.b)) {
    const c = m.construction[bp.b.constructionId];
    if (c?.dir && Math.abs(dot(sub(dest, c.origin), c.dir)) > 1e-6) removeJoint(m, bp.id);
  }
  const free = new Set([pointId]);
  const res = solveSketch(m, { dragTargets: [{ pointId, pos: dest, weight: 1 }], freePointIds: free, allowGroundMove: link.ground, maxIter: 60 });
  applyPositions(m, res.positions);
  rebuildShapeRigidity(m, link.id);
  let joinedTo: ID | undefined;
  if (joinTo && m.points[joinTo] && m.points[joinTo].linkId !== link.id) {
    const other = m.points[joinTo];
    const already = jointsAtPoint(m, pointId).some((j) => j.pairs?.some(([a, b]) => a === other.id || b === other.id));
    if (!already) {
      const j = addJoint(m, m.settings.defaultJoint, { linkId: link.id, kind: 'vertex', pointIds: [pointId] }, { linkId: other.linkId, kind: 'vertex', pointIds: [other.id] }, { axis: sketchNormal(m) });
      if (j) {
        joinedTo = other.id;
        const res2 = solveSketch(m, { maxIter: 60 });
        applyPositions(m, res2.positions);
      }
    }
  }
  return { ok: res.converged, residual: res.residual, joinedTo };
}

/** Rebuild a link's shape rigidity from its current positions, keeping joint-helper attachments. */
export function rebuildShapeRigidity(m: Model, linkId: ID): void {
  const link = m.links[linkId];
  if (!link || link.kind === 'cylinder') return;
  const helperIds = link.helperIds.filter((h) => m.points[h]?.role === 'helper');
  const helperRigidity = link.rigidity.filter((r) => helperIds.some((h) => rigidityTouches(r, h)));
  link.rigidity = [...buildRigidity(m, linkShapePointIds(m, link)), ...helperRigidity];
}
