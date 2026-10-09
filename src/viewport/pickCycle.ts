/**
 * Query cycle (Creo-style "query select"): a right-click at one spot steps
 * through everything under the pointer, one candidate at a time. Pure state
 * machine, DOM-free; the ToolManager feeds it ranked candidates
 * (rankPickCandidates) and shows the current candidate as the hover.
 */
import { samePick, type PickResult } from './pickRank';

/** A pointer that stays within this many pixels of the cycle position keeps the cycle alive. */
export const QUERY_CLICK_PX = 4;

export class PickCycle {
  private candidates: PickResult[] = [];
  private i = -1;
  private at: { x: number; y: number } | null = null;

  /**
   * Begin a cycle at `at`. The first candidate shown is the first one that is
   * not the current hover, so the first right-click always reveals something
   * new; when every candidate is the hover (or nothing is hovered) the cycle
   * starts at the top. Returns the candidate to highlight, or null (and stays
   * inactive) when there are no candidates.
   */
  start(candidates: PickResult[], at: { x: number; y: number }, current: PickResult | null): PickResult | null {
    this.candidates = [...candidates];
    this.at = { x: at.x, y: at.y };
    if (this.candidates.length === 0) {
      this.cancel();
      return null;
    }
    const fresh = this.candidates.findIndex((c) => !samePick(c, current));
    this.i = fresh >= 0 ? fresh : 0;
    return this.candidates[this.i];
  }

  /** Advance to the next candidate, wrapping around; null when no cycle is active. */
  next(): PickResult | null {
    if (!this.active) return null;
    this.i = (this.i + 1) % this.candidates.length;
    return this.candidates[this.i];
  }

  get active(): boolean {
    return this.candidates.length > 0;
  }

  /** Index of the highlighted candidate (0-based), -1 when inactive. */
  get index(): number {
    return this.active ? this.i : -1;
  }

  get count(): number {
    return this.candidates.length;
  }

  /** The highlighted candidate, null when inactive. */
  get current(): PickResult | null {
    return this.active ? this.candidates[this.i] : null;
  }

  /** Is the pointer still at the spot the cycle was started at (within `px` pixels)? */
  isSamePlace(x: number, y: number, px = QUERY_CLICK_PX): boolean {
    return this.at !== null && Math.hypot(x - this.at.x, y - this.at.y) <= px;
  }

  cancel(): void {
    this.candidates = [];
    this.i = -1;
    this.at = null;
  }
}
