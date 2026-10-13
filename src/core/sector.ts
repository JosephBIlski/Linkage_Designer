/**
 * Live sector-angle feedback for the Panel (sketch) tool: while a panel is
 * being drawn onto existing vertices, the running sum of the corner angles of
 * the panels around each shared vertex, with the angle the new panel adds
 * there (docs/CONSTRUCTION_PLAN.md, item 2b). Pure geometry over the model;
 * the viewport turns the result into overlay labels, and the Panel tool uses
 * the same classification to refuse a panel that would overlap its
 * neighbours in the plane.
 */
import { coincidenceTolerance } from './edit';
import { SECTOR_SUM_TOLERANCE_DEG, interiorAngleDeg, mergedPointGroups } from './feasibility';
import { cross, dist, dot, len, newellNormal, normalize, sub } from './geometry';
import { linkEdges } from './model';
import { collinear } from './patterns';
import type { ID, Link, Model, Vec3 } from './types';

export interface SectorPreview {
  /** Sum of the interior angles (degrees) of the panels that already share the vertex. */
  existingDeg: number;
  /**
   * Angle (degrees) the new panel adds at the vertex, between its two adjacent sketch edges: the true interior
   * angle, 0–360, when the sketch's winding normal is known (a reflex corner of an L-shaped panel counts 270°),
   * otherwise the unsigned angle, 0–180.
   */
  addedDeg: number;
  /** existingDeg + addedDeg. */
  sumDeg: number;
  /** Both adjacent sketch edges coincide with edges of existing panels at the vertex: the new panel completes the ring. */
  closesRing: boolean;
  /** The existing panels (polygons / prisms) whose corner angles make up existingDeg. */
  linkIds: ID[];
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
 * of them. `addedDeg` is the interior angle of the sketch at the vertex: with
 * `normal` (the winding normal of the closed sketch polyline, newellNormal) it
 * is the signed angle from next − v to prev − v about that normal, the same
 * rule interiorAngleDeg applies to finished panels, so a reflex corner counts
 * its full 270° and the sum agrees with the validator once the panel exists;
 * without a normal (fewer than three points, a degenerate polyline) it is the
 * unsigned angle between the two edges. An edge of zero length contributes 0°.
 * `closesRing` holds when an existing panel edge runs from the vertex to `prev`
 * and another to `next` (end points within the tolerance), i.e. the new panel
 * fills the last gap of the ring; the sum must then be 360° for the panels to
 * lie flat around the vertex.
 */
export function sectorPreview(m: Model, vertexPos: Vec3, prev: Vec3, next: Vec3, existingPointId: ID, normal?: Vec3): SectorPreview {
  const tol = coincidenceTolerance(m);
  const groups = mergedPointGroups(m);
  const members = new Set<ID>(groups.group(existingPointId) ?? [existingPointId]);
  for (const q of Object.values(m.points)) {
    if ((q.role === 'vertex' || q.role === 'axis') && dist(q.pos, vertexPos) <= tol) members.add(q.id);
  }
  let existingDeg = 0;
  const linkIds: ID[] = [];
  for (const pid of members) {
    const pt = m.points[pid];
    const link = pt ? m.links[pt.linkId] : undefined;
    if (!link || !isPanel(link) || linkIds.includes(link.id)) continue;
    const a = interiorAngleDeg(m, link, pid);
    if (a === null) continue;
    existingDeg += a;
    linkIds.push(link.id);
  }
  const addedDeg = interiorDeg(sub(prev, vertexPos), sub(next, vertexPos), normal);
  const closesRing = hasPanelEdge(m, vertexPos, prev, tol) && hasPanelEdge(m, vertexPos, next, tol);
  return { existingDeg, addedDeg, sumDeg: existingDeg + addedDeg, closesRing, linkIds };
}

/** How the preview should be presented: `ok` when the ring closes at 360°, `bad` when the sum exceeds 360° or the ring closes elsewhere, `neutral` otherwise. */
export type SectorStatus = 'ok' | 'bad' | 'neutral';

/**
 * Classify a preview with the tolerance of the sector-sum diagnosis
 * (SECTOR_SUM_TOLERANCE_DEG): a ring that closes within it is `ok`; a sum
 * beyond 360° + tolerance can never lie flat, and a ring that closes at any
 * other sum cannot lie flat either, so both are `bad` (in the plane of the
 * existing panels such a panel overlaps them and the Panel tool refuses it,
 * sketchSectors; in 3-D it is a legitimate pyramid-like vertex and is joined);
 * an open ring below 360° is `neutral` (more panels may follow). A straight or
 * zero corner (collinear sketch vertices) is judged by the caller, which has
 * the three points: sketchSectors marks it `bad`, since the panel is refused
 * as degenerate.
 */
export function sectorStatus(p: SectorPreview, tolDeg: number = SECTOR_SUM_TOLERANCE_DEG): SectorStatus {
  const off = Math.abs(p.sumDeg - 360);
  if (p.closesRing) return off <= tolDeg ? 'ok' : 'bad';
  return p.sumDeg > 360 + tolDeg ? 'bad' : 'neutral';
}

/** One sketch vertex of the Panel tool that sits on an existing vertex, with its sector-angle verdict (sketchSectors). */
export interface SketchSector {
  /** Index of the vertex in the sketch polyline. */
  index: number;
  /** The existing vertex it sits on. */
  pointId: ID;
  preview: SectorPreview;
  /** The label colour: sectorStatus of the preview, or `bad` for a collinear (straight or zero) corner. */
  status: SectorStatus;
  /**
   * The sketch and every existing panel around the vertex lie in one plane (within coincidenceTolerance). A `bad`
   * status in the plane means the new panel overlaps its neighbours: closing the ring with a sum other than 360°,
   * or exceeding 360°, is impossible in a plane without overlap, whereas in 3-D it is a consistent folded vertex.
   */
  coplanar: boolean;
}

/**
 * Sector-angle verdict at every vertex of a sketch polyline that sits on an
 * existing vertex: `pts` is the closed polyline (no repeated closing point;
 * the Panel tool includes the cursor as its last vertex while drawing) and
 * `snapped[i]` the existing vertex `pts[i]` was placed on, or null. The
 * winding normal of the polyline (newellNormal) gives sectorPreview its
 * signed corner angles; a corner whose two edges are collinear
 * (patterns.collinear, the predicate the Panel tool refuses degenerate panels
 * with) is `bad`, so the label and the refusal that follows agree. Vertices
 * with a zero-length adjacent edge are skipped. The Panel tool's labels and
 * its in-plane overlap refusal (status `bad` with `coplanar`) share this one
 * computation.
 */
export function sketchSectors(m: Model, pts: Vec3[], snapped: (ID | null | undefined)[]): SketchSector[] {
  const n = pts.length;
  if (n < 3) return [];
  const raw = newellNormal(pts);
  const normal = len(raw) > 1e-18 ? normalize(raw) : null;
  const tol = coincidenceTolerance(m);
  const out: SketchSector[] = [];
  for (let i = 0; i < n; i++) {
    const pid = snapped[i];
    if (!pid || !m.points[pid]) continue;
    const prev = pts[(i + n - 1) % n];
    const next = pts[(i + 1) % n];
    if (dist(prev, pts[i]) < 1e-9 || dist(next, pts[i]) < 1e-9) continue;
    const preview = sectorPreview(m, pts[i], prev, next, pid, normal ?? undefined);
    const status: SectorStatus = collinear(prev, pts[i], next) ? 'bad' : sectorStatus(preview);
    const coplanar = normal !== null && preview.linkIds.every((id) => linkInPlane(m, m.links[id], pts[i], normal, tol));
    out.push({ index: i, pointId: pid, preview, status, coplanar });
  }
  return out;
}

function isPanel(link: Link): boolean {
  return link.kind === 'polygon' || link.kind === 'prism';
}

/** Every vertex of the link lies within `tol` of the plane through `origin` with unit normal `n`. */
function linkInPlane(m: Model, link: Link | undefined, origin: Vec3, n: Vec3, tol: number): boolean {
  if (!link) return false;
  return link.pointIds.every((id) => {
    const p = m.points[id]?.pos;
    return !!p && Math.abs(dot(sub(p, origin), n)) <= tol;
  });
}

/**
 * Corner angle in degrees between the edge vectors u (to the previous vertex)
 * and w (to the next): the signed angle from w to u about `normal` (0–360) when
 * a winding normal is given, as interiorAngleDeg measures it on finished
 * panels, else the unsigned angle (0–180). 0 when either edge is (nearly) zero.
 */
function interiorDeg(u: Vec3, w: Vec3, normal?: Vec3): number {
  const lu = len(u);
  const lw = len(w);
  if (lu < 1e-12 || lw < 1e-12) return 0;
  if (!normal || len(normal) < 1e-18) return (Math.acos(Math.max(-1, Math.min(1, dot(u, w) / (lu * lw)))) * 180) / Math.PI;
  let a = (Math.atan2(dot(cross(w, u), normalize(normal)), dot(u, w)) * 180) / Math.PI;
  if (a < 0) a += 360;
  return a;
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
