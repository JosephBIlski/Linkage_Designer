# Mechanism construction workflow: diagnosis and plan (v0.3)

Status: Phase 0 and Phase 1 in progress. Phases 2 and 3 are planned.

This document records why a user could not build a degree-4 origami vertex
from four triangles with the Joint tool, what the evidence showed, which
options were weighed, and the phased plan that was chosen. It is the
specification for the v0.3 changes; `docs/SPEC.md` describes the behaviour of
each feature once implemented and `docs/DESIGN_DECISIONS.md` §8 keeps the
lessons learned.

## 1. The report

> I'm currently unable to create a 4 vertex origami fold. What I'm currently
> doing is creating 4 triangles and connecting the edges together as revolute
> joints. When I go to connect the last joint, the final triangle glitches out,
> and I get an error saying that constraints are violated.

Question asked: is this a problem of how user input is managed, or of how the
backend interprets it, and should the fix be a backend change or a different
way of creating links and geometry?

## 2. Evidence

The workflow was reproduced by driving the exact Joint-tool code path
(`addJoint` → `solveSketch` → `commitSketch`) from scratch vitest files, and
by running the built app headless. All numbers are in model units.

| Scenario | Result |
| --- | --- |
| Four regular (Polygon tool) triangles joined edge to edge in 2-D | Joints 1–3 move the joined triangle by 1.04, 1.85 and 2.48. Joint 4 does not converge, residual 0.654, pose committed anyway, status "Joint created", DOF chip "Constraints violated". |
| Same, edge lengths exactly equal | Joints 1–3 are exact merges (no movement). Joint 4 residual 0.414: the four 60° sectors sum to 240°, not 360°, so the loop cannot close flat. |
| Six regular triangles (sectors sum to 360°) | All six creases close with zero residual. |
| Developable vertex (60°, 60°, 120°, 120°) built flat, 2-D constraint on | Closes with zero residual but reports DOF 0: the sketch-plane constraint freezes z, so no in-plane crease can rotate. |
| Same, 2-D constraint removed | DOF 2 in the flat state (bifurcation). A fold sweep from flat follows the straight-hinge branch: two creases never move. |
| Same, pre-folded by driving a non-collinear crease to 160° | Converges from the exactly flat state, dihedrals (−170°, 160°, 170°, 160°), DOF 1, sweep moves all four panels. Driving a collinear crease instead lands on the hinge branch (170°, 180°, 170°, 180°). |
| Fourth panel sketched with the Sketch tool onto a folded three-panel vertex | Shared edges become creases automatically, zero residual, DOF 1 (existing test `tests/edit.test.ts`). |
| Fourth panel 40° out of plane in 3-D, joined with the Joint tool | Pulled into a consistent loop with zero residual. |

### 2.1 Backend defects found

1. **Unequal-length edge–edge revolute is wrong.** `addJoint`
   (`src/core/model.ts`, edge–edge branch) merges end points only when the two
   edge lengths agree to one part in a million. Otherwise it emits two
   point-on-line constraints plus a slide lock with `offsets.slide = 0` and no
   orientation check, which forces B's *first* edge point onto A's *first*
   edge point. Hand-placed geometry never passes the tolerance, so this is the
   default path, and a reversed edge is flipped to the other side of the axis.
   This is the "glitch".
2. **The Joint tool commits non-converged solves.** `jointTool` in
   `src/viewport/tools.ts` never checks `converged`. The least-squares
   compromise spreads the inconsistency over every non-ground link and the
   status still says "Joint created". The same pattern exists in
   `changeJointType` and the bar-length editor in `src/ui/panels.ts`.
3. **Save and Solidify bake violated poses.** `solidifyAssumptions` and
   `saveToJson` in `src/app.ts` call `refreshRigidity` on every link, so a
   violated pose becomes the new rest geometry (verified: violation 1.58 → 0
   after baking, rest lengths changed).
4. **The 2-D sketch-plane constraint is invisible.** Each polygon drawn in 2-D
   mode carries a planar `body` joint against the sketch plane
   (`bodyPlaneJoint`). It is hidden from rendering, the model tree and the
   vertex pop-up and has no Properties toggle, yet it makes every in-plane
   crease rigid.
5. **Flat vertices need a pre-fold.** The flat state of a degree-4 vertex is a
   bifurcation (numerical DOF 2 flat, 1 folded). The Miura example only works
   because it is created in a closed-form folded pose
   (`foldedVertexPositions`); user-built panels have no equivalent action.

### 2.2 Input / workflow defects found

1. Four regular triangles cannot form a vertex (sector sum 240°) and nothing
   warns the user before the fourth joint.
2. The Polygon tool's circumcircle snap silently adds a pin for the first
   snapped vertex only; other coincident vertices are not joined.
3. Coincident edges are hard to pick apart; the joint glyph blocks re-picking.
4. In 3-D mode clicks are placed on the view plane rather than the sketch
   plane.
5. The workflow that already works (Sketch tool, snapping every vertex onto
   existing vertices so shared edges become creases) is undiscoverable.

## 3. Diagnosis

Both layers are involved, but they separate cleanly.

- The *symptom* (jump, violated constraints, permanent damage on save) is the
  backend: items 1–3 above are robustness defects in otherwise correct
  machinery. The solver closes every consistent loop that was tried.
- The *cause* (a vertex that cannot exist) is the input model: building four
  independent bodies and asking four joints to make them agree asks the user
  to solve a loop-closure problem by hand, and the UI neither prevents nor
  explains an impossible configuration.
- Even a correct flat vertex does not fold without a pre-fold step, which no
  construction workflow can avoid; it has to be an explicit operation.

## 4. Options considered

| Option | Description | Verdict |
| --- | --- | --- |
| A. Independent panels + robust joining | Keep the Joint tool; add pre-flight solve, rollback and diagnostics; tolerant edge matching; Fold command. | Necessary (protects every workflow) but not sufficient: the closing joint stays fragile and non-developability is only discovered at the last joint. |
| B. Shared-vertex "panel" construction | Panels are drawn onto existing vertices; creases are implied by shared edges (what the Sketch tool does today). | Removes the loop-closure problem by construction; still needs the pre-fold and a validator for sector angles. |
| C. Crease-pattern-first | Draw vertices and crease lines with mountain/valley marks on a plane; faces are detected and turned into panels and creases. | Closest to Crane / Origami Simulator and the only option that scales to full Miura sheets; largest effort. |
| D. B plus a crease-pattern validator | Shared-vertex construction with live sector-angle feedback and a developability check. | Chosen as the medium-term target; grows naturally into C. |

Decision: A first (small, backend), then the Fold command and first-class
creases, then D, growing into C. Bars and pins are untouched throughout: a
crease remains an ordinary edge–edge revolute, so DOF counting, sweeps,
inverse design and compliant hinges keep working unchanged.

## 5. Phases

### Phase 0: backend robustness (small)

**0a. Tolerant edge–edge revolute matching** (`addJoint`).
- Choose the end-point pairing by proximity (straight vs crossed) for *all*
  edge pairs, as the equal-length branch already does.
- Treat lengths as equal within a snap-sized tolerance (relative 1e-3 of the
  longer edge, or an explicit tolerance option) and merge the end points.
- When lengths really differ, orient the B edge to match A, pair the nearer
  end points, and *measure* `offsets.slide` from the current pose exactly as
  the construction-axis branch does, instead of assuming zero.
- Acceptance: two triangles sharing an edge with a 1e-5 relative mismatch and
  reversed edge order are a no-op (residual 0, no vertex moves more than
  1e-6). Two triangles whose shared edges differ by 5 % keep the paired apex
  within 1e-6 of its position after the solve. Exactly equal edges still merge
  into two pairs. Existing Miura and edit tests pass.

**0b. Joint-tool pre-flight, rollback and diagnostics.**
- A shared helper (core) performs "add joint, solve, verify": if the sketch
  solve does not converge or its residual exceeds a tolerance scaled by model
  size, the model is restored to the snapshot taken before the joint was
  added and the helper returns a diagnosis.
- The Joint tool, `changeJointType` and the bar-length editor use the helper.
  No tool path may commit a non-converged pose.
- Diagnoses, in order of specificity, each with a clearly labelled string in
  `src/ui/strings.ts`:
  1. The two edges differ in length by Δ (edge–edge joints beyond the
     tolerance where merging was intended).
  2. The panels around vertex V have sector angles summing to S°, not 360°
     (the new joint closes a loop of panels sharing a vertex).
  3. The loop can only close out of the sketch plane: release the 2-D
     constraint (detected by re-solving with the body planar joints of the
     involved links temporarily removed; if that converges, this is the
     message).
  4. Generic: the joint cannot be satisfied (residual r); nothing was moved.
- Acceptance: the four-regular-triangle scenario ends with the first three
  joints created and the fourth refused with the sector-angle message; no
  link moves on the refused attempt; undo history has no entry for it.

**0c. Rest-geometry baking guard.**
- `solidifyAssumptions`, `saveToJson` and any other `refreshRigidity` caller
  that acts on a whole model skip baking when `currentViolation` exceeds the
  tolerance, and the UI reports that assumptions were not solidified.
- Acceptance: saving a violated model preserves the rest lengths; a test
  covers `commitSketch`/solidify behaviour on a violated pose.

**0d. Expose the sketch-plane constraint.**
- Link Properties gain a checkbox "Keep on sketch plane (2-D)" that adds or
  removes the body planar joint (re-solving afterwards through the 0b
  helper).
- The model tree shows a "2-D" badge on constrained links.
- The DOF chip or Mechanism panel shows a hint when the mechanism has
  revolute creases whose axes lie in a sketch plane that all their links are
  constrained to ("creases locked by the 2-D constraint").

### Phase 1: folding (medium)

**1a. Core Fold (pre-fold) command** (`src/core/fold.ts`).
- Detect "crease loops": sets of edge–edge revolute joints with two merged
  pairs that share a common vertex and whose links form a closed cycle.
- Detect the flat/singular state: all crease dihedrals within 1° of 180° (or
  the Jacobian rank deficiency of `computeMobility`).
- Pre-fold: remove body planar joints of the involved links; for each
  candidate crease, preferring creases that are *not* collinear with another
  crease at the vertex, temporarily prescribe its dihedral (default 160°,
  sign chosen by an optional mountain/valley preference) and forward-solve
  from the flat pose; accept the first result that converges, leaves no
  crease within 1° of flat, and reduces the DOF; commit the pose; remove the
  temporary driver. Return the crease used, the dihedrals and the new DOF, or
  a reason for failure (no loop, not flat, no branch found).
- UI: a "Fold" action in the Mechanism panel / joint Properties when a flat
  crease loop exists; the Driver tool pre-folds automatically when a fold
  driver is added on a crease of a flat vertex and says so in the status bar;
  the status bar suggests Fold after a joint closes a flat loop.
- Acceptance: four triangles with sectors (60°, 60°, 120°, 120°) built flat
  and joined → Fold gives dihedrals within 2° of (−170°, 160°, 170°, 160°)
  up to a global sign, DOF 1, and a fold-driver sweep moves all four panels.
  An "X" vertex (90°, 90°, 90°, 90°) reports failure with the reason that no
  generic branch was found. The Miura example is unaffected.

**1b. Creases as first-class presentation.**
- `Joint.fold?: { target?: number; mv?: 'M' | 'V' }` stores the target fold
  angle and mountain/valley assignment.
- Rendering: an edge–edge revolute with two merged pairs is drawn as a line
  along the shared edge, coloured by the sign of the current dihedral
  (mountain / valley colours added to Settings, origami convention red /
  blue), instead of the torus glyph. It remains pickable as the joint.
- Joint Properties for a crease: current fold angle (read-only), M/V toggle,
  target fold angle, "Fold to target" (uses the Fold core), "Drive this
  crease" (adds a fold driver and makes it active).
- Model tree: creases are labelled "Crease" with the current fold angle.

### Phase 2: construction workflow (medium, planned)

- Promote the Sketch tool to the "Panel" tool: snap every vertex, join all
  coincident vertices, and show the live sector-angle sum at shared vertices
  while drawing (red when the loop would not close).
- The Polygon tool joins all snapped vertices, not only the first.
- Pick disambiguation for coincident edges (cycle on repeated click; joint
  glyphs never occlude edges).
- 3-D mode places clicks on the sketch plane, not the view plane.
- Crease-pattern validator: for each interior vertex, sector angles sum to
  360° (developability) and the Kawasaki alternating sum is zero
  (flat-foldability); reported in the Mechanism panel.

### Phase 3: crease-pattern-first (large, planned)

- Draw vertices and crease lines on a plane with mountain/valley marks; faces
  are detected (half-edge face tracing of the planar straight-line graph) and
  generated as polygon links with edge–edge creases through the existing
  core; the pattern entity is kept for regeneration; Fold selects the branch
  from the M/V assignment.

## 6. Data-model changes

| Change | Phase | Compatibility |
| --- | --- | --- |
| `Joint.offsets.slide` measured for unequal edges | 0a | Existing files load unchanged; new joints store the measured value. |
| `Joint.fold?: { target?, mv? }` | 1b | Optional field; absent in old files. |
| Settings colours `mountain`, `valley` | 1b | Defaults applied when missing. |

No change to `Model.version` is needed: all additions are optional.

## 7. Test plan

- Unit (vitest): edge matching cases (0a); feasibility helper rollback and
  each diagnosis (0b); baking guard (0c); body-plane toggle (0d); Fold on the
  developable and X vertices, DOF before/after, sweep motion (1a); crease
  detection and M/V sign (1b).
- Headless browser (`scripts/smoke-features.mjs`): the user's workflow with
  four regular triangles ends with a refused fourth joint and the
  sector-angle message; four developable triangles joined, then Fold, then a
  fold driver animates all panels.
- Regression: all existing tests, `npm run build`, both smoke scripts.

## 8. Open decisions

- Default pre-fold angle 160° and tolerance 1° are starting values; expose
  them in Settings only if users ask.
- Whether the Fold command keeps the temporary driver as the model's driver
  when none exists (current plan: yes, if the model has no driver, so the
  vertex is immediately previewable).
- Mountain/valley colours: red mountain, blue valley follows Lang and the
  Origami Simulator convention; configurable.

## 9. References

- T. Tachi, "Simulation of Rigid Origami", Origami 4 (2009): rigid-origami
  kinematics as constraint projection; degree-4 vertex has one DOF.
- D. Huffman, "Curvature and Creases: A Primer on Paper", IEEE Trans.
  Computers (1976): degree-4 vertex folding, straight-hinge degeneracy.
- K. Suto, Y. Noma, K. Tanimichi, K. Narumi, T. Tachi, "Crane: An Integrated
  Computational Design Platform for Functional, Foldable, and Fabricable
  Origami Products", ACM TOCHI (2022): constraint-based origami design.
- E. Allgower, K. Georg, *Numerical Continuation Methods* (1990): predictor
  steps and branch switching at bifurcations.
- R. J. Lang, *Twists, Tilings, and Tessellations* (2018): Kawasaki and
  Maekawa conditions, mountain/valley conventions.
- E. Demaine, J. O'Rourke, *Geometric Folding Algorithms* (2007):
  developability (sector angles sum to 2π) and flat-foldability conditions.
