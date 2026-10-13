/**
 * Crease-pattern validator: the classical conditions on a single origami
 * vertex, evaluated for every interior vertex of the model
 * (docs/CONSTRUCTION_PLAN.md, item 2d).
 *
 * An interior vertex is a merged vertex (mergedPointGroups, the union-find
 * over the joints' merged pairs) whose panels form a closed ring of creases:
 * exactly the crease loops of fold.ts (findCreaseLoops). Its sector angles
 * are the interior angles of the panels at the vertex (interiorAngleDeg), in
 * the cyclic order of the loop, and the checks are
 *  - DEVELOPABILITY: the sectors sum to 360°, so the panels can lie flat in
 *    one plane (a pyramid apex sums to less, a saddle to more);
 *  - KAWASAKI's condition: a vertex of even degree can fold flat only when
 *    the alternating sum α₁ − α₂ + α₃ − … of its sectors vanishes (the odd
 *    and the even sectors then each sum to 180°). A vertex of odd degree
 *    never folds flat: the paper reverses direction at every crease and has
 *    to come back to itself after one turn, which needs an even number of
 *    reversals, so the alternating sum is reported as NaN and the check
 *    fails;
 *  - MAEKAWA's condition: at a flat-folded vertex the numbers of mountain
 *    and valley creases differ by exactly two. The classes are read from the
 *    current construction pose in the convention the crease display uses
 *    (creaseMV), once every crease of the loop is folded away from flat; a
 *    loop with an unfolded crease has no assignment and the check is not
 *    applied (null).
 *
 * Pure core: no DOM or three.js imports.
 */
import { SECTOR_SUM_TOLERANCE_DEG, interiorAngleDeg, mergedPointGroups } from './feasibility';
import { creaseMV, findCreaseLoops } from './fold';
import type { ID, Model } from './types';

/** The validator's verdict on one interior vertex (validateCreasePattern). */
export interface VertexReport {
  /** A point of the merged vertex (on the ground panel when one is in the ring), as findCreaseLoops names it. */
  vertexPointId: ID;
  /** "Link name + point name" of that point, e.g. "Polygon 1 V0". */
  vertexName: string;
  /** Number of panels (= creases) around the vertex: its degree. */
  panelCount: number;
  /** Interior angles of the panels at the vertex (degrees), in the cyclic order of the crease loop. */
  sectorDeg: number[];
  /** Sum of the sector angles (degrees). */
  sumDeg: number;
  /** |sumDeg − 360| ≤ tol: the panels can lie flat in a plane. */
  developable: boolean;
  /** Alternating sum α₁ − α₂ + α₃ − … of the sectors in loop order (degrees); NaN for a vertex of odd degree. */
  kawasakiDeg: number;
  /** Kawasaki's condition holds: even degree and |kawasakiDeg| ≤ tol. (A vertex folds flat only when it is also developable.) */
  flatFoldable: boolean;
  /** Number of mountain creases in the construction pose, null while any crease of the loop is flat or unmeasurable. */
  mountains: number | null;
  /** Number of valley creases, null under the same condition. */
  valleys: number | null;
  /** Maekawa's condition |M − V| = 2, null while the classes are not assigned. */
  maekawa: boolean | null;
  /** The creases of the loop in cyclic order; crease i and crease i + 1 bound the panel of sectorDeg[i]. */
  creaseIds: ID[];
}

/**
 * Alternating sum α₁ − α₂ + α₃ − … of sector angles in cyclic order (degrees),
 * the quantity Kawasaki's condition requires to vanish. NaN for an odd number
 * of sectors, where the alternating sum depends on the starting sector and
 * flat folding is impossible anyway. The sign depends on which sector is
 * first; only the magnitude is meaningful.
 */
export function kawasakiSumDeg(sectorDeg: number[]): number {
  if (sectorDeg.length % 2 !== 0) return NaN;
  return sectorDeg.reduce((acc, a, i) => acc + (i % 2 === 0 ? a : -a), 0);
}

/**
 * One report per interior vertex of the model (a crease loop whose every
 * panel has a sector angle at the vertex: polygons and prism faces; a loop
 * running through a bar or a cylinder axis has no sector angles and is
 * skipped). `tolDeg` is the tolerance on the sector sum and on the
 * alternating sum (the 0.5° sector tolerance of the joint diagnoses by
 * default). The mountain / valley counts are read from the construction
 * pose, so a flat (unfolded) vertex reports them as null.
 */
export function validateCreasePattern(m: Model, tolDeg: number = SECTOR_SUM_TOLERANCE_DEG): VertexReport[] {
  const groups = mergedPointGroups(m);
  const reports: VertexReport[] = [];
  for (const loop of findCreaseLoops(m)) {
    const members = groups.group(loop.vertexPointId) ?? [loop.vertexPointId];
    const sectorDeg: number[] = [];
    for (const linkId of loop.linkIds) {
      const link = m.links[linkId];
      const pid = members.find((id) => m.points[id]?.linkId === linkId);
      const angle = link && pid !== undefined ? interiorAngleDeg(m, link, pid) : null;
      if (angle === null) break;
      sectorDeg.push(angle);
    }
    if (sectorDeg.length !== loop.linkIds.length) continue;
    const sumDeg = sectorDeg.reduce((a, b) => a + b, 0);
    const kawasakiDeg = kawasakiSumDeg(sectorDeg);
    const classes = loop.creaseIds.map((id) => (m.joints[id] ? creaseMV(m, m.joints[id]) : null));
    const assigned = classes.every((c) => c !== null);
    const mountains = assigned ? classes.filter((c) => c === 'M').length : null;
    const valleys = assigned && mountains !== null ? classes.length - mountains : null;
    const pt = m.points[loop.vertexPointId];
    reports.push({
      vertexPointId: loop.vertexPointId,
      vertexName: pt ? `${m.links[pt.linkId]?.name ?? ''} ${pt.name}`.trim() : '',
      panelCount: loop.linkIds.length,
      sectorDeg,
      sumDeg,
      developable: Math.abs(sumDeg - 360) <= tolDeg,
      kawasakiDeg,
      flatFoldable: !Number.isNaN(kawasakiDeg) && Math.abs(kawasakiDeg) <= tolDeg,
      mountains,
      valleys,
      maekawa: mountains !== null && valleys !== null ? Math.abs(mountains - valleys) === 2 : null,
      creaseIds: [...loop.creaseIds],
    });
  }
  return reports;
}
