/**
 * Joint descriptions shared by the model tree, the status bar and the query
 * cycle, so the three cannot drift apart: "Crease M 160° · Panel 1 ↔ Panel 2"
 * for a crease (its mountain / valley class and fold angle in the construction
 * pose), "Revolute joint · Link 1 ↔ Link 2" for an ordinary joint. The words
 * come from strings.ts; the measurements from core/fold.
 */
import { creaseDihedralDeg, creaseMV, isCrease } from '../core/fold';
import type { Joint, Model } from '../core/types';
import { isConstructionRef } from '../core/types';
import { JOINTS, creaseLabel } from './strings';

/** "Crease M 160°" for a crease (construction pose), else the joint type's label ("Revolute joint"), or its letter ("R") when `short`. */
export function jointLabel(m: Model, j: Joint, short = false): string {
  if (isCrease(m, j)) return creaseLabel(creaseMV(m, j), creaseDihedralDeg(m, j));
  return short ? JOINTS[j.type].short : JOINTS[j.type].label;
}

/** "<link A> ↔ <link B or datum>" for a joint. */
export function jointEnds(m: Model, j: Joint): string {
  const a = m.links[j.a.linkId]?.name ?? '';
  const b = isConstructionRef(j.b) ? m.construction[j.b.constructionId]?.name ?? '' : m.links[j.b.linkId]?.name ?? '';
  return `${a} ↔ ${b}`;
}

/** "<label> · <ends>": the one-line description of a joint (tree item with `short`, status bar and query cycle without). */
export function jointDescription(m: Model, j: Joint, short = false): string {
  return `${jointLabel(m, j, short)} · ${jointEnds(m, j)}`;
}
