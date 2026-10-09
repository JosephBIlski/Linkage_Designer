# Mechanism construction workflow: diagnosis and plan (v0.3)

Status: Phase 0 and Phase 1 are implemented (v0.3). Phases 2 and 3 remain
planned.

This document records why a user could not build a degree-4 origami vertex
from four triangles with the Joint tool, what the evidence showed, which
options were weighed, and the phased plan that was chosen. It is the
specification for the v0.3 changes; `docs/SPEC.md` §17 describes the
behaviour of each feature as implemented and `docs/DESIGN_DECISIONS.md` §8
keeps the lessons learned. The *Implemented* note under each phase in §5
records where the implementation differs from the plan; §2 (evidence) and §4
(options) are left as they were written.

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
  reversed edge order merge with residual 0; the ground triangle does not
  move, the adapting triangle's released end point moves by exactly
  Δ = |la − lb| (2e-5 for a 2-unit edge), its apex by less, and no rest
  distance of the adapting triangle changes by more than Δ (the merged pairs
  force the Δ displacement, so the "1e-6" bound first written here is
  unattainable). Two triangles whose shared edges differ by 5 % keep the
  paired apex within 1e-6 of its position after the solve. Exactly equal
  edges still merge into two pairs. Existing Miura and edit tests pass.
- *Implemented as planned, with these additions.* The mismatch is absorbed
  by one link only (`creaseReleasePoints`: the second-picked link unless it
  is ground or locked, else the first; the Sketch / Edit tools name the link
  being drawn or edited) through a two-step solve, rigid first and then
  released from the rigid pose (`solveSketchWithRelease`), because a single
  released solve leaves a displaced triangle's apex behind. The adaptation
  is accepted only while no rest distance of the adapting link changes by
  more than 2 × the merge tolerance of the longer edge
  (`CREASE_ADAPTATION_TOLERANCE`); beyond that the joint is refused (0b), so
  hand- or mouse-placed four regular triangles are refused instead of
  collapsing the fourth triangle into a line. A merged crease that neither
  panel may absorb (both ground or locked) is refused as an edge-length
  conflict. A panel that the merge would fold flat onto its neighbour
  (dihedral ≈ 0°) is placed beside it (dihedral 180°) instead. For unequal
  lengths the stored feature b lists its end points in the order matching a.
  Tests: `tests/joints.test.ts`.

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
- *Implemented as planned.* The diagnosis order is 1–4 as listed (a merged
  crease whose small mismatch neither panel may absorb also reports 1); the
  helper is `tryAddJoint` / `tryChangeJointType` / `trySolveCommit` in
  `src/core/feasibility.ts`, and it also covers the typed vertex Position,
  the Select-tool drag release, the 2-D checkbox (0d) and the automatic joins
  of the Sketch and Edit tools (a refused re-solve leaves the polygon
  unjoined). Interior angles are signed about the ring's winding normal (a
  reflex corner counts 270°); the 3-D probe runs from the flat pose and again
  from a pose nudged 5 % of the model size out of the plane, since a flat
  start is a bifurcation; the refused joint's residual is the rigid solve's
  gap. An incompatible type change leaves the joint as it was (previously it
  was deleted) and the pop-up cycle stops at a refused type. Messages end
  with "Nothing was moved.", show numbers with four significant digits, and
  are kept as a dismissable notice in the Mechanism panel (the status bar's
  tooltip carries the whole text). Two solver fixes were needed on the way:
  the Levenberg–Marquardt damping is floored at 1e-3 of the largest diagonal
  entry of JᵀJ (crease merges of nearly axis-aligned hand-placed edges
  stalled), and a system with no free variables is converged only when its
  residual is small. Tests: `tests/feasibility.test.ts`,
  `tests/solver.test.ts`.

**0c. Rest-geometry baking guard.**
- `solidifyAssumptions`, `saveToJson` and any other `refreshRigidity` caller
  that acts on a whole model skip baking when `currentViolation` exceeds the
  tolerance, and the UI reports that assumptions were not solidified.
- Acceptance: saving a violated model preserves the rest lengths; a test
  covers `commitSketch`/solidify behaviour on a violated pose.
- *Implemented as planned.* A pose is consistent when `currentViolation` is
  at most max(1e-6, 1e-7 × model size) (`consistencyTolerance`); the DOF
  chip's "Constraints violated" flag uses the same tolerance (previously a
  fixed 1e-5), so it warns exactly when Solidify, Save and Extrude refuse.
  Save still writes the file when the pose is inconsistent (previous rest
  geometry plus the violated positions) and says so; Solidify reports
  success as well as refusal; Extrude is refused on an inconsistent pose;
  `commitSketch` refreshes released rest geometry only from an accepted
  solve and returns whether it did; a rest distance between two frozen
  points is no longer dropped at compile time, so it counts as a violation.
  Tests: `tests/baking.test.ts`.

**0d. Expose the sketch-plane constraint.**
- Link Properties gain a checkbox "Keep on sketch plane (2-D)" that adds or
  removes the body planar joint (re-solving afterwards through the 0b
  helper).
- The model tree shows a "2-D" badge on constrained links.
- The DOF chip or Mechanism panel shows a hint when the mechanism has
  revolute creases whose axes lie in a sketch plane that all their links are
  constrained to ("creases locked by the 2-D constraint").
- *Implemented as planned.* `setBodyPlane` adds, replaces or removes the
  body planar joint (bars and polygons only, against the active sketch
  plane; a link is on one plane at a time; drawing in 2-D and ticking the box
  give byte-identical models). The hint reads "n creases cannot fold: both
  panels are kept on the sketch plane. Untick 'Keep on sketch plane' in
  Properties" and adds ", or use Fold" only while a flat crease loop exists;
  it appears in the Mechanism panel and as the DOF chip's tooltip. The
  link-end pop-up's constraint tooltip names the plane. Tests:
  `tests/model.test.ts`.

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
- *Implemented with these differences.* The Fold button lives in the
  Mechanism panel only (shown in every mode whenever a flat loop exists); a
  crease's Properties offer "Fold to target" and "Drive this crease" (1b)
  instead of a second Fold button. The DOF test compares against the flat
  state *after* the planar joints were removed (2 for a degree-4 vertex; with
  them the flat vertex has DOF 0 and nothing could be lower). Drivers acting
  on the loop are released during the search and re-measured afterwards; a
  locked non-ground panel is a further failure reason ("locked"); the Driver
  tool replaces the driver the pre-fold kept with one on the picked crease,
  never leaves two fold drivers on a crease, and moves the single driver that
  Fold left on another crease of the same vertex. Loop selection: the given
  loop → the loop containing the preferred crease → the first flat loop →
  "notFlat" when loops exist but none is flat → "noLoop". Mountain / valley
  preferences (passed by "Drive this crease" and "Fold to target"; the Fold
  button passes none) are verified on every named crease of the loop, so a
  preference on a collinear crease (never driven first) selects the sign of
  the crease that is driven. Tests: `tests/fold.test.ts`.

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
- *Implemented with these differences.* The mountain / valley class is
  referenced to the first panel's face normal from its vertex winding
  (`creaseNormal`), not to the stored edge direction, so neighbouring creases
  of one sheet are judged from the same side; a degree-4 vertex on its
  generic branch then shows Maekawa's 3 : 1 split with the crease between the
  two equal small sectors as the odd one (the bent pair share a class, the
  collinear pair differ). "Fold to target" is `foldCreaseToTarget`: pre-fold
  of a flat loop preferring the crease and class asked for, unfolding through
  flat when the mirrored side of a folded vertex is requested, and
  continuation in steps of at most 120° (an antipodal target routed through
  flat) so that an open-fan hinge can be flipped M ↔ V; targets of 0° and
  180° skip the class check; a further failure reason "unreachable" was
  added. The target field is written without an undo entry (the fold is the
  undoable change). The tree and the Properties show the construction pose
  (the Properties readout shows the fold-angle magnitude with the class);
  only the viewport colours the rendered (preview) pose. Tests:
  `tests/crease.test.ts`.

### Phase 2: construction workflow (medium, in progress)

**2a. Query pick: right-click cycles through everything under the pointer
(Creo-style).** Picking today returns one feature per pointer position, so two
coincident edges can never both be reached and the Joint tool picks the same
link twice.
- The viewport computes the full candidate list under the pointer (every
  distinct feature hit by the exact ray and the sampled ring: editing points,
  vertices, joints and creases, edges, axes, faces, then datum geometry),
  ordered by pick priority and distance and de-duplicated. The ranking lives
  in a DOM-free module so it can be unit-tested.
- A right-click that is not a drag (pointer moved less than 4 px between down
  and up; right-drag keeps orbiting) starts or advances a *query cycle* at
  that position: the next candidate becomes the highlighted (hovered) feature
  and the status bar reads "2 of 3 · edge V0–V1 of Polygon 2 · right-click:
  next". Each further right-click at the same place advances and wraps.
  Moving the pointer more than a few pixels, or Esc, ends the cycle and normal
  hover resumes.
- A left-click while a cycle is active uses the highlighted candidate as the
  pick for the active tool (Select, Joint, Edit, Ground, Driver, Delete,
  Mirror, Pattern all go through the same event), so any feature under the
  pointer can be chosen.
- The Joint tool additionally prefers, among candidates at the same spot, one
  on a link other than the first feature's link, and says so in the status
  bar; the cycle still allows overriding it.
- Acceptance: two triangles with exactly coincident edges can be joined with
  the Joint tool through the UI (headless test); right-drag still orbits; the
  candidate ranking is unit-tested (priority order, de-duplication, datum
  geometry last); the help text documents the right-click.

**2b. Panel tool and polygon joins.**
- The Sketch tool is presented as "Panel (sketch polygon)" and joins *every*
  vertex that coincides with an existing vertex (snapped or typed) through
  the existing auto-join, so shared edges become creases.
- While sketching, each vertex of the sketch that lies on an existing vertex
  shows an overlay label with the running sector-angle sum at that vertex:
  the interior angles of the existing panels plus the angle the new panel
  adds there (its two adjacent sketch edges; the open edge follows the
  cursor). The label turns red when the sum exceeds 360°, or when the new
  panel closes the ring of panels around the vertex and the sum is not 360°,
  and green when it closes the ring at 360°.
- The Polygon tool joins all vertices of the new polygon that coincide with
  existing vertices (not only the first), through the same auto-join with
  rollback.
- Acceptance: four developable triangles drawn with the Panel tool by
  snapping onto existing vertices end with four creases, zero violation and
  the hint to Fold; the sector label at the centre reads 360° in green on the
  closing panel; four regular triangles show a red label at the centre on the
  fourth panel and the join is refused with the sector message.

**2c. 3-D placement on the sketch plane.** With "Place on sketch plane (2-D)"
off, free clicks are placed where the pointer ray meets the active sketch
plane (the view plane is used only when the ray grazes the sketch plane),
so geometry in 3-D mode is still built on a predictable plane. Typed
coordinates and snapped points are unchanged.

**2d. Crease-pattern validator.** For every interior vertex (a merged vertex
group whose panels form a closed ring of creases) the Mechanism panel reports
developability (sector angles sum to 360°), the Kawasaki condition
(alternating sum of the sector angles is zero, flat-foldability) and, when
mountain/valley classes are assigned, the Maekawa condition (|M − V| = 2),
each as pass or fail with the measured numbers. The checks live in a pure
core module with unit tests on the developable, X and regular-triangle
vertices.

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
| `Joint.b` of an unequal-length hinge stored with its end points in the order matching `Joint.a` | 0a | Order only, no new field; existing files load unchanged. |

No change to `Model.version` is needed: all additions are optional.

## 7. Test plan

- Unit (vitest): `tests/joints.test.ts` (0a: edge matching, release points,
  cylinder axes, side-by-side placement, hand-placed panels),
  `tests/feasibility.test.ts` (0b: acceptance, every diagnosis and its
  precedence, exact / mouse- / hand-placed regular triangles, rollback, type
  change, the automatic joins), `tests/baking.test.ts` (0c),
  `tests/model.test.ts` (0d: `setBodyPlane`, `creasesLockedByPlane`),
  `tests/fold.test.ts` (1a: loops, pre-fold of the developable and X
  vertices, DOF before / after, sweep motion, preferences),
  `tests/crease.test.ts` (1b: M/V convention, persistence, Fold to target,
  Drive this crease, labels and colours), `tests/solver.test.ts` (damping
  floor). 165 tests in all.
- Headless browser (`scripts/smoke-features.mjs`, steps 18–19): four regular
  triangles placed exactly and by mouse end with a refused fourth joint, the
  sector-angle message, an unchanged model and no undo entry; a refused
  Length edit leaves no stale widget; a developable vertex sketched flat
  shows the hints and the Fold button, Fold gives DOF 1 and the 3 : 1
  pattern, the crease is pickable with its Properties, and Preview moves
  every panel.
- Regression: all existing tests, `npm run build`, both smoke scripts.

## 8. Decisions taken (formerly open)

- The default pre-fold angle 160° and the 1° flat / collinear tolerances are
  constants (`DEFAULT_PREFOLD_DEG`, `FLAT_TOLERANCE_DEG`,
  `COLLINEAR_TOLERANCE_DEG` in `src/core/fold.ts`), not Settings; expose them
  only if users ask.
- The Fold command keeps the temporary driver when the model has none, so
  the vertex is previewable at once; the Driver tool moves it to the crease
  the user picks.
- Mountain/valley colours: red mountain (`#d9342b`), blue valley (`#2f6fd6`),
  following Lang and the Origami Simulator convention; configurable in
  Settings.

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
