/**
 * Pick results and their ranking. DOM- and three.js-free so the ordering of
 * everything under the pointer (the query cycle, docs/CONSTRUCTION_PLAN.md 2a)
 * can be unit-tested; scene.ts re-exports the types.
 */
import type { ID, Vec3 } from '../core/types';

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
  /** Datum geometry only: 0 point, 1 axis, 2 plane (points beat axes beat planes; datums have no occlusion semantics). */
  sub?: number;
  /** Hit by the exact pointer ray (not only by the ring of rays sampled around the pointer). */
  exact?: boolean;
  /**
   * Depth band of a ranked model feature (rankPickCandidates): 0 for the features within the pick tolerance of the
   * nearest hit, 1 for the next band behind them, and so on. Undefined for datum geometry and unranked results.
   */
  band?: number;
  /**
   * The user named this feature explicitly (the query cycle, a model-tree row), not merely moved the pointer over
   * it. Datum planes and axes light up only for such hovers, so an idle pointer over an empty spot of the canvas
   * does not flood the whole sketch plane with the selection colour. Not part of the feature identity (pickKey).
   */
  queried?: boolean;
  point: Vec3;
  distance: number;
}

/** Lower wins: point-like features beat bodies, model geometry beats datum geometry. */
export const PICK_PRIORITY: Record<PickType, number> = {
  editPoint: 0,
  vertex: 1,
  joint: 2,
  edge: 3,
  axis: 4,
  face: 5,
  construction: 6,
};

/**
 * Identity of a pickable feature: the fields that tell one feature from
 * another. The point ids are part of it because every edge and face of a
 * polygon carries the same link id.
 */
export function pickKey(r: PickResult): string {
  return [r.type, r.id, r.pointId ?? '', (r.pointIds ?? []).join(','), r.faceIndex ?? '', r.pose ?? ''].join('|');
}

/** Do two pick results name the same feature? */
export function samePick(a: PickResult | null, b: PickResult | null): boolean {
  if (!a || !b) return a === b;
  return pickKey(a) === pickKey(b);
}

/**
 * Rank everything hit by the exact pointer ray and by the sampled ring, for
 * the query cycle. Duplicates (one feature hit by several rays) collapse into
 * the closest instance, which stays `exact` when any instance was. Model
 * features come first, in depth bands: the nearest hit opens a band that takes
 * every feature within `tolerance` behind it (the same rule the single pick
 * uses, so a vertex far behind the panel under the pointer is not offered
 * before the panel); within a band features are ordered by PICK_PRIORITY, then
 * exact-ray hits before ring hits, then by distance. Datum geometry comes
 * last, points before axes before planes (`sub`), then exact before ring, then
 * by distance. Ties keep the input order. Each model result carries its band
 * index in `band`.
 */
export function rankPickCandidates(results: PickResult[], tolerance: number): PickResult[] {
  const byKey = new Map<string, PickResult>();
  for (const r of results) {
    const k = pickKey(r);
    const prev = byKey.get(k);
    if (!prev) byKey.set(k, { ...r, exact: !!r.exact });
    else if (r.distance < prev.distance) byKey.set(k, { ...r, exact: prev.exact || !!r.exact });
    else if (r.exact && !prev.exact) prev.exact = true;
  }
  const unique = [...byKey.values()];
  const model = unique.filter((r) => r.type !== 'construction').sort((a, b) => a.distance - b.distance);
  const datum = unique.filter((r) => r.type === 'construction');
  const band = new Map<PickResult, number>();
  let bandIndex = -1;
  let bandStart = -Infinity;
  for (const r of model) {
    if (r.distance > bandStart + tolerance) {
      bandIndex++;
      bandStart = r.distance;
    }
    band.set(r, bandIndex);
  }
  const ring = (r: PickResult): number => (r.exact ? 0 : 1);
  model.sort((a, b) => band.get(a)! - band.get(b)! || PICK_PRIORITY[a.type] - PICK_PRIORITY[b.type] || ring(a) - ring(b) || a.distance - b.distance);
  // the band is reported so that callers can confine a choice to the front band (the Joint tool's substitution)
  for (const r of model) r.band = band.get(r);
  datum.sort((a, b) => (a.sub ?? 9) - (b.sub ?? 9) || ring(a) - ring(b) || a.distance - b.distance);
  return [...model, ...datum];
}
