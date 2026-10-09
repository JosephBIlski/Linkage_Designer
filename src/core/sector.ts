/**
 * Live sector-angle feedback for the Panel (sketch) tool: while a panel is
 * being drawn onto existing vertices, the running sum of the corner angles of
 * the panels around each shared vertex, with the angle the new panel will add
 * there (docs/CONSTRUCTION_PLAN.md, item 2b). Pure geometry over the model;
 * the viewport turns the result into overlay labels.
 */
import { coincidenceTolerance } from './edit';
import { SECTOR_SUM_TOLERANCE_DEG, interiorAngleDeg, mergedPointGroups } from './feasibility';
import { dist, dot, len, sub } from './geometry';
import { linkEdges } from './model';
import type { ID, Link, Model, Vec3 } from './types';

export interface SectorPreview {
  /** Sum of the interior angles (degrees) of the panels that already share the vertex. */
  existingDeg: number;
  /** Angle (degrees, 0–180) the new panel adds at the vertex: between its two adjacent sketch edges. */
  addedDeg: number;
  /** existingDeg + addedDeg. */
  sumDeg: number;
  /** Both adjacent sketch edges coincide with edges of existing panels at the vertex: the new panel completes the ring. */
  closesRing: boolean;
}

/**
 * Sector-angle preview at a sketch vertex placed on the existing vertex
 * `existingPointId` at `vertexPos`, with `prev` and `next` the far ends of the
 * two sketch edges that meet there (the cursor position for the edge that is
 * still open).
 *
 * `existingDeg` sums interiorAngleDeg over every polygon / prism that has a
 * vertex there: the merged-point group of `existingPointId` (mergedPointGroups,
 * so panels already creased or pinned together count once each) together with
 * any vertex of another link that merely coincides with the position within
 * coincidenceTolerance, because closing the sketch joins the new panel to all
 * of them. `addedDeg` is the unsigned angle between prev − v and next − v (a
 * panel drawn with a reflex corner at a shared vertex is not anticipated; its
 * true interior angle is only known once the polygon is closed). An edge of
 * zero length contributes 0°. `closesRing` holds when an existing panel edge
 * runs from the vertex to `prev` and another to `next` (end points within the
 * tolerance), i.e. the new panel fills the last gap of the ring; the sum must
 * then be 360° for the panels to lie flat around the vertex.
 */
export function sectorPreview(m: Model, vertexPos: Vec3, prev: Vec3, next: Vec3, existingPointId: ID): SectorPreview {
  const tol = coincidenceTolerance(m);
  const groups = mergedPointGroups(m);
  const members = new Set<ID>(groups.group(existingPointId) ?? [existingPointId]);
  for (const q of Object.values(m.points)) {
    if ((q.role === 'vertex' || q.role === 'axis') && dist(q.pos, vertexPos) <= tol) members.add(q.id);
  }
  let existingDeg = 0;
  const counted = new Set<ID>();
  for (const pid of members) {
    const pt = m.points[pid];
    const link = pt ? m.links[pt.linkId] : undefined;
    if (!link || !isPanel(link) || counted.has(link.id)) continue;
    const a = interiorAngleDeg(m, link, pid);
    if (a === null) continue;
    existingDeg += a;
    counted.add(link.id);
  }
  const addedDeg = angleDeg(sub(prev, vertexPos), sub(next, vertexPos));
  const closesRing = hasPanelEdge(m, vertexPos, prev, tol) && hasPanelEdge(m, vertexPos, next, tol);
  return { existingDeg, addedDeg, sumDeg: existingDeg + addedDeg, closesRing };
}

/** How the preview should be presented: `ok` when the ring closes at 360°, `bad` when the sum exceeds 360° or the ring closes elsewhere, `neutral` otherwise. */
export type SectorStatus = 'ok' | 'bad' | 'neutral';

/**
 * Classify a preview with the tolerance of the sector-sum diagnosis
 * (SECTOR_SUM_TOLERANCE_DEG): a ring that closes within it is `ok`; a sum
 * beyond 360° + tolerance can never lie flat, and a ring that closes at any
 * other sum will be refused when the panel is joined, so both are `bad`; an
 * open ring below 360° is `neutral` (more panels may follow).
 */
export function sectorStatus(p: SectorPreview, tolDeg: number = SECTOR_SUM_TOLERANCE_DEG): SectorStatus {
  const off = Math.abs(p.sumDeg - 360);
  if (p.closesRing) return off <= tolDeg ? 'ok' : 'bad';
  return p.sumDeg > 360 + tolDeg ? 'bad' : 'neutral';
}

function isPanel(link: Link): boolean {
  return link.kind === 'polygon' || link.kind === 'prism';
}

/** Unsigned angle in degrees between two vectors, 0 when either is (nearly) zero. */
function angleDeg(u: Vec3, w: Vec3): number {
  const lu = len(u);
  const lw = len(w);
  if (lu < 1e-12 || lw < 1e-12) return 0;
  return (Math.acos(Math.max(-1, Math.min(1, dot(u, w) / (lu * lw)))) * 180) / Math.PI;
}

/** Does some polygon / prism have an edge whose end points lie at `a` and `b` (either order, within tol)? */
function hasPanelEdge(m: Model, a: Vec3, b: Vec3, tol: number): boolean {
  for (const link of Object.values(m.links)) {
    if (!isPanel(link)) continue;
    for (const [x, y] of linkEdges(m, link)) {
      const px = m.points[x]?.pos;
      const py = m.points[y]?.pos;
      if (!px || !py) continue;
      if ((dist(px, a) <= tol && dist(py, b) <= tol) || (dist(px, b) <= tol && dist(py, a) <= tol)) return true;
    }
  }
  return false;
}
