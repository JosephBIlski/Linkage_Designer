/**
 * The DOM-free parts of the Crease pattern tool (docs/CONSTRUCTION_PLAN.md,
 * Phase 3b): which pattern a new polyline draws into, the snap of a pointer
 * position onto a crease, and the polyline state machine — vertex after
 * vertex, each one joined to the previous by a crease of the chosen class,
 * with Backspace undoing the last segment by restoring the model as it was
 * before that click (a click can merge a vertex, split a crease it lands on
 * and split every crease the new crease crosses, so restoring the snapshot is
 * the only exact inverse). ToolManager (tools.ts) wires these to the pointer
 * events and to the App's change tracking; tests/patternTool.test.ts drives
 * them directly.
 */
import { addPatternCrease, addPatternVertex, type AddCreaseResult, type AddVertexResult } from '../core/creasePattern';
import { add, dot, scale, sub } from '../core/geometry';
import { serializeModel } from '../core/model';
import type { CreaseClass, CreasePattern, ID, Model, PatternCrease, Vec3 } from '../core/types';
import type { SelectionView } from './render';

/** The pattern a selection names: the pattern itself, or the one a selected pattern vertex / crease belongs to. */
export function selectedPatternId(sel: SelectionView | null): ID | null {
  return sel && (sel.type === 'pattern' || sel.type === 'pvertex' || sel.type === 'pcrease') ? sel.id : null;
}

/**
 * The pattern a new polyline draws into: the selected pattern (or the pattern
 * of the selected vertex / crease) when it lies on the active sketch plane,
 * hidden or not; else the single visible pattern on that plane; else null,
 * and the tool creates one. A hidden pattern is never chosen implicitly, so
 * drawing after "Generate panels" (which hides the pattern) starts a new one
 * unless the old one is selected again in the model tree.
 */
export function choosePattern(m: Model, sel: SelectionView | null, planeId: ID): CreasePattern | null {
  const selId = selectedPatternId(sel);
  const selected = selId ? m.patterns?.[selId] : undefined;
  if (selected && selected.planeId === planeId) return selected;
  const visible = Object.values(m.patterns ?? {}).filter((p) => p.planeId === planeId && !p.hidden);
  return visible.length === 1 ? visible[0] : null;
}

/** "<base> n" with the smallest n ≥ 1 that no pattern uses. */
export function nextPatternName(m: Model, base: string): string {
  const used = new Set(Object.values(m.patterns ?? {}).map((p) => p.name));
  let n = 1;
  while (used.has(`${base} ${n}`)) n++;
  return `${base} ${n}`;
}

/**
 * The point of a crease nearest to `pos`: the orthogonal projection of `pos`
 * onto the crease's line, clamped to the segment. A click snapped onto a
 * crease is placed here, so that addPatternVertex finds the crease within
 * tolerance and splits it (or merges with an end vertex when the foot lands
 * on one). Null when the crease's vertices are missing or coincide.
 */
export function snapToCrease(p: CreasePattern, crease: PatternCrease, pos: Vec3): Vec3 | null {
  const a = p.vertices[crease.a]?.pos;
  const b = p.vertices[crease.b]?.pos;
  if (!a || !b) return null;
  const ab = sub(b, a);
  const l2 = dot(ab, ab);
  if (l2 <= 0) return null;
  const t = Math.max(0, Math.min(1, dot(sub(pos, a), ab) / l2));
  return add(a, scale(ab, t));
}

/** One click of the polyline: the model before it (serialised) and the vertex the crease started from (null for the first vertex). */
export interface PolylineSegment {
  snapshot: string;
  from: ID | null;
}

export interface AddPointResult {
  vertex: AddVertexResult;
  /** The crease from the previous vertex (null for the first vertex of the polyline). */
  crease: AddCreaseResult | null;
  /** True when the point fell on the polyline's own last vertex: nothing changed and no segment was recorded. */
  same: boolean;
}

/**
 * The polyline being drawn: the pattern it draws into, its last vertex (the
 * rubber band's anchor) and one segment per click, each holding the model as
 * it was before the click so that Backspace can restore it exactly. The
 * state machine mutates the pattern through the core editing operations and
 * leaves undo bookkeeping to the caller.
 */
export class PatternPolyline {
  patternId: ID | null = null;
  /** The vertex the next crease starts from; null while no polyline is being drawn. */
  last: ID | null = null;
  private segments: PolylineSegment[] = [];

  get active(): boolean {
    return this.last !== null;
  }

  /** Number of clicks that can be taken back with Backspace. */
  get length(): number {
    return this.segments.length;
  }

  /** Start a polyline in a pattern (the first click). */
  begin(patternId: ID): void {
    this.patternId = patternId;
    this.last = null;
    this.segments = [];
  }

  /**
   * Add the clicked point: a vertex (merged with one within tolerance, or
   * splitting the crease it lies on) and, from the second point on, a crease
   * of class `cls` from the previous vertex (split at every crossing, or
   * refused with a reason). The vertex becomes the polyline's last one
   * either way, so a refused crease still moves the anchor to where the user
   * clicked. A point on the last vertex itself is ignored. `snapshot` is the
   * model serialised before the click (by default: now); the tool passes the
   * one it took before creating the pattern, so that taking back the first
   * click removes the pattern too.
   */
  addPoint(m: Model, p: CreasePattern, pos: Vec3, cls: CreaseClass, snapshot: string = serializeModel(m)): AddPointResult {
    const from = this.last;
    const vertex = addPatternVertex(m, p, pos);
    if (from !== null && vertex.vertex.id === from) return { vertex, crease: null, same: true };
    const crease = from !== null ? addPatternCrease(m, p, from, vertex.vertex.id, cls) : null;
    this.segments.push({ snapshot, from });
    this.last = vertex.vertex.id;
    return { vertex, crease, same: false };
  }

  /**
   * Backspace: forget the last click and return its segment, whose snapshot
   * the caller restores. The polyline continues from the vertex that click
   * started at; taking back the first vertex ends the polyline. Null when no
   * click is left.
   */
  pop(): PolylineSegment | null {
    const seg = this.segments.pop();
    if (!seg) return null;
    this.last = seg.from;
    if (this.last === null) this.end();
    return seg;
  }

  /** Enter, Esc, a double-click or a tool switch: the next click starts a new polyline. */
  end(): void {
    this.patternId = null;
    this.last = null;
    this.segments = [];
  }
}
