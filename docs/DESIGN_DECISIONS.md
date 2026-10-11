# Design decisions, assumptions and references

This document records *why* the prototype is built the way it is. The
feature-level behaviour is specified in [SPEC.md](SPEC.md); the original brief
is in [ORIGINAL_SPEC.md](ORIGINAL_SPEC.md); the diagnosis and plan behind the
v0.3 joining / folding work and the v0.4 construction workflow are in
[CONSTRUCTION_PLAN.md](CONSTRUCTION_PLAN.md).

## 1. Point-based constraint formulation (the "Crane" idea)

**Decision.** Every piece of geometry is a set of 3-D points. Links are rigid
bodies expressed as distance / coplanarity / angle constraints between their
own points; joints are constraints between points of two links (or a link and
construction geometry); drivers and inverse-design targets are further
constraints. The solver's only job is to find point coordinates that satisfy a
list of residual functions.

**Why.** This is the formulation used by Tachi's Rigid Origami Simulator and,
following it, by Crane (Suto et al. 2023): vertex coordinates are the
unknowns, constraints (edge lengths, panel planarity, fold angles, design
goals) are residuals, and the configuration is projected onto the constraint
manifold with a Newton step using the pseudo-inverse of the constraint
Jacobian. Three properties made it the right choice here:

1. **Uniformity.** Bars, polygons, prisms, cylinders, origami panels, joints,
   construction-geometry constraints, drivers and inverse-design targets are
   all the same kind of object, so the mobility analysis, the forward
   simulation and the inverse design share one compile step
   (`src/core/compile.ts`).
2. **Numerical degrees of freedom.** DOF = (#variables − rank J) is exact for
   over-constrained and "paradoxical" mechanisms (Bennett linkage, rigid
   origami vertices, parallel pin axes), where the Grübler–Kutzbach count fails
   (Norton; McCarthy & Soh). Rigid origami simulators compute DOF the same way
   (Tachi 2009).
3. **Inverse design falls out.** Letting rest lengths float while stacking many
   poses turns the kinematic solver into a synthesis solver (see §4).

**Implementation details.**

- Coincident points (spherical / revolute vertex pairs, origami crease end
  points) are merged into one variable with a union-find pass, and
  coordinates fixed by axis-aligned sketch planes are frozen, so a planar
  four-bar has only 9–15 variables per pose. All linear algebra is dense
  (Cholesky on the normal equations, Householder QR with column pivoting for
  rank and null space). This is simpler than sparse solvers and fast enough
  for mechanisms with up to a few hundred variables. A constraint whose points
  are all frozen (a rest length between two pinned points) keeps its residual
  row even though it has no variable columns: it can still be violated, and a
  system with no free variables counts as converged only when its residual is
  small (§8, "Never commit a non-converged pose").
- The projection uses Levenberg–Marquardt (Nocedal & Wright, ch. 10) rather
  than plain Gauss–Newton because the Jacobian is rank deficient *by design*
  (the mechanism has motion freedom); the damping yields the minimum-norm step
  and tolerates singular poses. The diagonal damping is floored at 1e-3 of
  the largest diagonal entry of JᵀJ so that a coordinate the residuals hardly
  depend on is damped too (§8, "Levenberg–Marquardt damping floor").
- Residuals are all in length units. Angular constraints (drivers, dihedral
  angles, helper attachments) are multiplied by a characteristic length so one
  tolerance works for everything. The numeric Jacobian of the dihedral
  constraint wraps its difference at ±π, so the derivative stays finite when
  the two samples straddle the discontinuity.

## 2. Rigid bodies, planarity and joint helper points

- **Frame-triangle rule.** A body with ≥ 3 non-collinear points is made rigid
  by three mutual distances among a frame triangle and, for every other point,
  three distances to the frame — *except* when the point is coplanar with the
  frame, where two distances plus a coplanarity (triple product) constraint
  are used. Distance-only constraints on a flat point set are infinitesimally
  flexible (the Jacobian loses rank), which would corrupt the DOF count; the
  coplanarity constraint removes that first-order flex. This is the standard
  cure in rigidity theory.
- **Two-point bars have no roll.** A bar is two points with one distance (5
  DOF). For a bar on spherical joints this is exactly right (the passive spin
  of an S–S link is excluded). For revolute joints the pin direction must be
  carried by the body, so **helper points** are created at the joint
  (offset along the pin axis) and attached with *length-independent*
  constraints: fixed distance to the vertex, fixed cosine to the bar
  direction, and a parallel / twist relation to the other helper of the same
  bar. Because none of these depend on the bar's length, changing the length
  during inverse design never fights the helper attachment and the helper
  geometry never adds spurious design freedom.
- **Revolute between edges** (origami crease). The end points of the two
  edges (or cylinder axes) are paired by proximity in the current pose,
  straight or crossed, so the drawing or picking order never matters. When
  the lengths agree within one part in a thousand of the longer edge the two
  pairs are merged into shared variables (5 equations) — this is a *crease* —
  and a small residual mismatch is absorbed by one link's rest geometry (the
  second-picked link unless it is ground or locked; see §8, "Joining edges of
  unequal length"). When they genuinely differ the second edge is oriented
  like the first and held on its line by two point-on-line constraints plus a
  slide lock whose offset is *measured* from the current pose (5 equations),
  exactly as a revolute against a datum axis; such a hinge is not a crease.
- **Cylindrical** = two point-on-line constraints (4 eq.), **prismatic** =
  cylindrical + a reference point held in a plane containing the axis (5 eq.),
  **screw** = cylindrical + translation−pitch·rotation/2π coupling (5 eq.),
  **planar** = points of one face on the plane of the other (3 eq.) or on a
  construction plane.

## 3. Forward simulation and range detection

A driver is a constraint with a prescribed value: angle of a bar about a
grounded pivot, dihedral angle of a crease, or slide along a prismatic axis.
The sweep steps the driver from the current pose, warm-starting each solve
from the previous pose, and bisects toward the limit when a step fails to
converge or jumps branches (displacement guard). If a full revolution returns
to the start configuration the input is classified as a **crank**, otherwise
as a **rocker** with the discovered range. This reproduces Grashof behaviour
without implementing the Grashof criterion explicitly, and works for inputs
where no closed-form criterion exists (spatial linkages, origami).

For 2-DOF mechanisms the "output path" of a point is a surface; it is sampled
on a grid by sweeping the primary driver at several values of the secondary
driver (`sweepGrid`) and rendered as a translucent mesh, matching the
"spherical joint → 2-D output surface" example in the brief.

## 4. Inverse design: stacked poses with prescribed timing

**Decision.** All `poseCount` sampled poses are solved simultaneously as one
system in which (a) the input value of each pose is prescribed, (b) unlocked
link lengths / polygon shapes are *shared* free variables (equal across poses
via `sharedDistance` constraints to pose 0), (c) unlocked ground pivots are
shared variables, and (d) editing-point targets are hard constraints on
individual poses.

**Why.** This is the multi-pose formulation of Coros et al. 2013 and
Thomaszewski et al. 2014 for linkage characters, and corresponds to
*path generation with prescribed timing* in classical synthesis (McCarthy &
Soh; Norton). Compared with a reduced parameterisation (design parameters only,
inner assembly solve + implicit differentiation, as in LinkEdit's symbolic
approach), the stacked system:

- gives the **remaining design freedom directly as the nullity** of the stacked
  Jacobian, which the UI displays as "Design DOF left";
- degrades gracefully when a pose temporarily cannot be assembled;
- needs no symbolic kinematics, so it works unchanged for spatial and origami
  mechanisms.

**Checked against theory.** A planar four-bar with a triangular coupler and
free ground pivots has 9 design parameters (two pivots, crank, follower and
three coupler distances). The prototype reports 9 design DOF, and each planar
position target removes 2 (test `tests/synthesis.test.ts`). With free timing
the classical limit is nine precision points (Wampler, Morgan & Sommese 1992);
with prescribed timing, as here, it is five (10 equations for 9 parameters is
over-determined, and the UI reports "over-constrained").

**Soft assumptions.** With under-determined designs the solver first runs with
weak regularisers pulling toward the current design (min-change assumption),
then projects exactly onto the hard constraints with the soft terms removed.
This two-phase scheme is what makes hard constraints exactly satisfied while
still choosing the "least surprising" design. When soft assumptions are off
the motion preview is hidden until design DOF = 0, as requested.

**Design space.** The exact reachable set of a point under all admissible
designs is unbounded (lengths are free) and expensive to sample. The
prototype shows a *first-order* design space: the null space of the stacked
Jacobian projected onto the point's coordinates gives, per pose, the local
directions in which the editing point can be dragged (0 = fully determined,
1 = curve, 2 = surface, 3 = volume); the union over poses (plus the tangent
directions of the output surface for multi-DOF mechanisms) sets the global
dimension, drawn as a segment / disc / ball with an extent of about 1.1 × the
current reach. Dragging an editing point is constrained to its local
directions (Shift releases this). This is an approximation and is flagged as
such in SPEC.md.

**Timing.** Prescribed timing makes each editing point "stiff" along the path
(it cannot slide to a different input value). The alternative — free timing
with one extra unknown per pose — is left as future work; it needs an
additional ordering regulariser to keep poses from collapsing.

## 5. Flexibility / compliant mechanisms

A flexible link's design distances become springs: their residuals are scaled
by `0.05 · stiffness` relative to hard constraints, so the least-squares
minimum is a quasi-static equilibrium where hard joints are satisfied (to
≈ 1/400) and flexible members absorb the mismatch. Compliant hinges (origami
creases) are torsional springs on the dihedral angle with a rest angle. This
follows the pseudo-rigid-body view of compliant mechanisms (Howell 2001) and
the spring model of Ghassaei et al.'s origami simulator (axial + crease
springs). Bistability appears naturally: when a flexible member must pass
through a compressed state the solver follows the nearest local minimum and
snaps through at the dead point. The solve is quasi-static — no inertia or
damping dynamics.

## 6. UI decisions

- **Z-up**, Creo-style default datums (ORIGIN, X/Y/Z, TOP/FRONT/RIGHT) and
  right-button orbit / middle-button pan, as in Rhino.
- The 2-D workflow is a *sketch plane*: new bars/polygons receive a planar
  "body" constraint to the active construction plane, which the compiler turns
  into frozen coordinates. Removing that constraint (or sketching in 3-D mode)
  frees the link into space. Since v0.3 the constraint is exposed as the link
  property *Keep on sketch plane (2-D)* with a "2-D" badge in the model tree,
  because an invisible constraint that makes every in-plane crease rigid (a
  flat origami vertex drawn in 2-D reports DOF 0) was the single most
  confusing part of the origami workflow (CONSTRUCTION_PLAN.md §2.1); the
  Mechanism panel names the creases it locks.
- Refusals are explained, not silently absorbed: a construction edit that
  cannot be satisfied is rolled back and its reason is shown in the status
  bar and, because the bar is overwritten by the next hover, kept as a
  dismissable notice in the Mechanism panel until the next successful change.
  The vocabulary of the messages is the user's (edge lengths, corner angles,
  the sketch plane), never the solver's.
- Editing points, output paths and the design space are coloured exactly as
  specified; every colour has an HWB picker with a HEX field and is persisted
  in `localStorage`. Defaults live in `src/ui/settings.ts`. Creases follow the
  origami convention of red mountain / blue valley (Lang 2018; the Origami
  Simulator of Ghassaei et al.), configurable like every other colour.
- **Right-click query select (v0.4).** Where edges or vertices coincide —
  two panels drawn edge on edge, the vertex a fan of panels shares — one
  pick per pointer position cannot reach every feature, and the Joint tool
  kept finding the first link again. Creo resolves this with *query select*:
  the pointer pre-highlights one candidate, right-clicking steps through the
  others, and the left-click confirms. That convention was chosen over
  Rhino's pop-up selection menu because it keeps the eye on the geometry,
  needs no extra widget and composes with every tool (all of them read the
  same pick), and because the right button is already the orbit button, so a
  click without a drag is free. Candidates are ranked in depth bands (the
  rule the single pick already used, applied to successive bands) rather
  than by priority alone, so a vertex hidden behind the panel under the
  pointer is offered after that panel's features; datum geometry comes last
  and lights up only when named through the cycle or the model tree, since
  the idle pointer rests on the sketch plane wherever the canvas is empty and
  a highlighted sketch plane floods the view. The Joint tool's substitution
  of a feature on another link is confined to the front band: a mis-click on
  the first link must never silently join to geometry behind the clicked
  panel, while the cycle still reaches it.
- **Free clicks land on the sketch plane in 3-D mode too (v0.4).** The
  view-aligned plane through the previous point put 3-D geometry at a depth
  that depended on the camera, so a bar built in the default view floated
  above the grid it seemed to be drawn on. CAD sketching happens on a plane;
  the 2-D option now only decides whether the link is *kept* there. The view
  plane remains the fallback for a ray that grazes the sketch plane
  (|cos| < 0.15: a pixel of pointer motion would sweep the point far across
  the plane), misses it, or meets it at the camera, which the Front and
  Right views of the TOP plane do for every off-centre ray.
- **Panels are built onto existing vertices, with live sector-angle
  feedback (v0.4).** Option D of CONSTRUCTION_PLAN.md §4: a panel sketched
  onto the vertices already there needs no loop closure by the Joint tool,
  so the Sketch tool is presented as *Panel* and joins every coincident
  vertex, typed or snapped, under one coincidence tolerance (1e-6 of the
  model size, which separates floating-point noise from a near miss by many
  orders of magnitude). The label at a shared vertex shows the running sector
  sum in the user's terms (degrees; green at 360°, red when the panels cannot
  lie flat), and a panel that would overlap its neighbours in their plane is
  refused with the vocabulary of the Joint tool's sector message, so the
  impossibility of four regular triangles is seen on the fourth panel rather
  than at the fourth joint.
- **The crease-pattern checks live in the Mechanism panel** next to the DOF
  readout, as plain lines with ✓ / ✗ and the measured numbers, because they
  are properties of the current geometry recomputed with the DOF, not model
  data; each classical condition (developability, Kawasaki, Maekawa; Hull
  1994) is reported on its own, so a pyramid apex reads "developable ✗ ·
  Kawasaki 0.0° ✓" rather than one opaque verdict.
- All strings live in `src/ui/strings.ts`.

## 8. Lessons from testing (v0.2 – v0.4)

**Degree-4 vertex example (Miura).** The first version used sector angles
(α, π−α, α, π−α), which makes *both* crease pairs collinear: an "X" vertex,
developable but not flat-foldable, whose rigid folding degenerates into two
independent straight hinges. In addition the start configuration was only a
z-nudge of the far corners projected back onto the constraint manifold, and
the nearest configuration on that manifold is the degenerate straight-hinge
branch (two creases stay flat while the other two act as one hinge), which
is exactly the "one joint rotates in a circle" behaviour that was reported.
Both were fixed: the sectors are now (α, α, π−α, π−α) (one straight pair,
one bent pair, Kawasaki satisfied) and the example is created in an *exact*
folded configuration computed in closed form (`foldedVertexPositions`), on
the generic all-four-creases branch. A regression test checks that all four
creases move, that opposite creases fold equally and that the Miura
relation tan(ρ_bent/2) = tan(ρ_straight/2)/cos α holds.

**Branch following in sweeps.** Any degree-4 vertex passes through the flat
state, which is a bifurcation between the generic branch and the hinge
branch. Warm-starting the next pose from the previous pose let the solver hop
onto the hinge branch there. The sweep now warm-starts from the *secant
extrapolation* of the last two poses (a standard predictor step in numerical
continuation), which keeps it on the current branch; the branch-jump guard
still compares the result against the previous pose.

**Picking thin geometry.** Bars are drawn with a radius of about 1 % of the
model size, which is one or two pixels at the default zoom, so an exact ray
cast missed them and fell through to the datum planes; this made
"unconstrained bodies cannot be dragged" look like a solver bug although the
sketch solve was correct. Picking now (a) never lets datum geometry occlude
model geometry, (b) samples a ring of rays around the pointer when the exact
ray hits nothing, and (c) gives bars and polygon edges an invisible pick proxy
at least ~8 px wide.

**Drag weights.** Soft drag targets had the same weight as hard constraints,
so jointed links visibly stretched while being dragged (the least-squares
compromise violated joints until release). Drag targets now use a weight of
0.05: joints stay satisfied to ≈1e-3 while a free body still follows the
pointer exactly.

**Editing vertices.** Moving a vertex that is part of its polygon's frame
triangle used to tilt the plane of the other vertices; the Edit tool now
re-builds the rigidity with the edited vertex as a dependent point first.
Edits whose destination cannot be reached (e.g. a vertex pinned to a datum
point) are rolled back instead of committing a violated pose. Coincident
vertices are joined automatically, and two consecutive coincident vertices on
a shared edge become one crease rather than two parallel pins (which would
lock the panels, DOF 0).

**Joining edges of unequal length (v0.3).** Hand-placed or mouse-placed
geometry never gives two edges of exactly equal length, so the old "merge if
equal to one part in a million" rule took the slide-lock path for nearly
every crease, and because the slide was assumed zero and the orientation was
never checked, a reversed edge was flipped to the other side of the axis: the
"glitch" of the report. The fix has three parts. (1) End points are matched
by proximity for *every* edge pair and merged when the lengths agree within a
snap-sized tolerance (1e-3 of the longer edge); when they genuinely differ,
the hinge orients the second edge like the first and measures its slide from
the current pose, as the datum-axis branch always did. (2) Merging the end
points of edges whose rest lengths differ by Δ makes the two bodies' rigidity
contradict each other, and a least-squares solve spreads Δ over every
non-ground link and never converges. The solver has to be told which body
yields: the design distances at the adapting edge's end points are released,
the model is solved rigidly first (so the adapting link is carried onto its
partner as a whole), then solved again from that pose with the release, and
the adapting link's rest geometry is refreshed (`solveSketchWithRelease`). A
single released solve is not enough: releasing both end points of a triangle
releases all of its distances, and a scratch run left a displaced triangle's
apex three units from its edge. (3) For the same reason a released solve can
"converge" by reshaping the adapting panel, so the change of its rest
distances (`releaseDrift`) is bounded by twice the merge tolerance; beyond
that the loop is one that rigid panels cannot close, the joint is refused and
the diagnosis is made from the rigid step. Finally, a crease that pulled a
panel drawn elsewhere flat onto its neighbour (dihedral ≈ 0°) is re-solved
from the pose rotated by a half turn about the shared edge, so the panels end
up side by side where a crease can fold.

**Never commit a non-converged pose (v0.3).** The Joint tool, the joint type
change, the bar-length editor and the drag release committed whatever the
sketch solve returned, so an impossible joint produced a least-squares
compromise spread over the whole mechanism, "Joint created" in the status bar
and "Constraints violated" in the chip; Save and Solidify then baked that
compromise into the rest geometry (violation 1.58 → 0 after baking, rest
lengths changed), destroying the design. The rule is now: every re-solved
edit is a snapshot → edit → solve → verify sequence (`src/core/feasibility.ts`);
a result is accepted when the solver converged or its hard residual is within
1e-7 of the model size, otherwise the model is restored byte for byte, no
undo entry is recorded and the user gets a diagnosis. Rest geometry is
refreshed only from an accepted result (`commitSketch`) and only when the
pose is consistent (`solidifyRestGeometry`), with one tolerance,
max(1e-6, 1e-7 × model size), shared by Solidify, Save, Extrude and the DOF
chip: a converged solve legitimately leaves residuals up to 1e-7 (sketch /
forward) or 1e-6 (inverse design), and a pose the app itself just accepted
must never be called inconsistent. Two solver details surfaced on the way: a
distance between two frozen points (both ends pinned) had no variable
columns and was dropped at compile time, hiding a violation that the Length
editor can create; and a system with no free variables reported itself
converged whatever its residual. Both now count.

**Explaining a refused joint (v0.3).** A refusal is only useful with a reason
in the user's vocabulary. Whether a joint closes a loop is a connectivity
question (breadth-first search over link–link joints: a joint between already
connected links closes a loop made of the links on the shortest path); the
vertex is the merged-point group containing one of the joint's end points
that touches the most links, and the panels' interior angles there (signed
about the ring's Newell normal, so a reflex corner counts 270°) are compared
with 360°. Whether the loop closes once the 2-D constraints are removed is
probed on a copy — but an exactly flat model is a bifurcation where every
out-of-plane gradient vanishes, so the probe is run again from a start nudged
5 % of the model size along the plane normal (the three-square tube is
diagnosed only with the nudge). The diagnoses are ordered edge lengths →
sector sum → needs 3-D → generic residual, because the earlier ones name the
thing the user can change, and the probe is consulted only after a failed
solve, so a consistent model trivially "closing" without 2-D is harmless.

**Levenberg–Marquardt damping floor (v0.3).** Marquardt's diagonal scaling
(λ·A_ii) leaves a column whose Jacobian entries are nearly zero almost
undamped: a coordinate the residuals hardly depend on (the x of a point on an
almost vertical distance) was moved by a large amount for no gain, the step
was rejected and the solve stalled, so a jittered square's crease merge never
converged and the joint was refused as infeasible. The scaling is now floored
at 1e-3 of the largest diagonal entry (`DAMPING_FLOOR`); in such directions
the step is the minimum-norm (damped pseudo-inverse) step, which is what a
projection onto the constraint manifold wants. No existing test changed.

**Flat vertices and the pre-fold (branch selection, v0.3).** The flat state
of a degree-4 vertex is a bifurcation of the constraint manifold: the
numerical DOF is 2 there (1 once folded), and a sweep started from it falls
onto the straight-hinge branch where two creases never move (Huffman 1976;
Tachi 2009). The Miura example avoids this only because it is created in a
closed-form folded pose; user-built vertices need an explicit operation,
which no construction workflow can avoid. The pre-fold prescribes a modest
dihedral (160°) on one crease and forward-solves the rest of the mechanism
from the flat pose; the branch is selected by the choice of crease — driving
a crease that is *not* collinear with another one at the vertex lands on the
generic branch, driving a collinear one lands on the hinge branch, which the
acceptance test (no crease of the loop still within 1° of flat, and a DOF
lower than the flat state's) rejects. The sketch-plane constraints of the
panels have to go first (they hold the vertex flat), and drivers acting on
the vertex are released during the search, since holding them at their flat
values forbids any fold. An "X" vertex (four 90° sectors) has every crease
collinear with another and folds only as a straight hinge, so the pre-fold
reports that no branch was found. Prescribing a dihedral is itself a
continuation problem (Allgower & Georg 1990): the constraint wraps at ±180°
from its target, so a single solve cannot cross that discontinuity and
"Fold to target" steps toward its target in increments of at most 120°,
routes an antipodal target (same fold angle, other class) through the flat
state, and returns a folded vertex to *exactly* flat — all creases
prescribed ±180° at once, because prescribing one alone leaves the vertex
anywhere on the hinge branch — before pre-folding to the mirrored side.

**Mountain / valley sign convention (v0.3).** The signed dihedral follows
the right-hand rule about the stored direction of the first edge, so it flips
when the edge happens to be stored the other way round, which the Joint
tool's end-point pairing can do. A crease's class is therefore referenced to
the face normal of its first panel given by its vertex winding (Newell
normal): valley when the second panel bends toward that side, mountain when
away. For panels drawn counter-clockwise on the sketch plane this is the
sketch normal, so neighbouring creases of one sheet are judged from the same
side. With this convention a degree-4 vertex with sectors (α, α, π−α, π−α)
on its generic branch shows Maekawa's 3 : 1 split (Lang 2018; Demaine &
O'Rourke 2007) with the crease between the two equal α sectors as the odd
one: the bent (zigzag) pair share a class and the collinear pair differ — the
plan's expectation that both pairs share a class cannot hold under any
consistent convention. The pre-fold's preference check (used by *Drive this
crease* and *Fold to target*; the Fold button passes no preference) uses an
orientation-independent test (`creaseMountainValley`, the panels' in-plane
directions summed against the loop's normal) and verifies every requested
assignment on the folded pose, so a preference on a collinear crease, which
is never driven first, is honoured through the sign of the crease that is
driven; contradictory preferences on the bent pair are refused.

**Picking coincident features (v0.4).** Two lessons from the query cycle.
First, the identity of a pickable feature must include its point ids and
face index: every edge and face of a polygon carries the same link id, so a
hover comparison on type and id alone collapsed all edges of one panel into
one candidate and the cycle appeared not to move. Second, "the nearest
feature wins" and "priority wins within the depth tolerance" are one rule
applied once; applied to successive depth bands it ranks every hit along the
ray without offering hidden geometry before the panel under the pointer. A
right-click is a drag as soon as the pointer has strayed beyond 4 px at any
time while the button was down — an orbit that swings out and returns to its
press position must not start a cycle — so the excursion is tracked, not the
end position.

**The camera can sit on the sketch plane (v0.4).** The Front and Right views
look along the TOP plane with the camera *on* it, so every off-centre ray
meets the plane at parameter 0: a mathematically valid hit at the camera,
which `rayPlane` and three's `intersectPlane` both accept. Placing points
there produced geometry at the eye, invisible and unusable. A hit now counts
as a miss when its depth is below a thousandth of the working distance (the
depth of the view-plane fallback), a relative threshold that scales with the
model. Related: OrbitControls fixes its orbit axis to the camera's up vector
when it is created, so the Top view, whose up vector is Y, sat at the pole of
a Z-up frame where a sideways right-drag could not orbit; the controls are
rebuilt whenever a view changes the up vector.

**A closed ring in a plane always sums to 360° (v0.4).** The plan expected a
red sector label and a refused join when the fourth regular triangle closes
the flat fan; but in a plane the corner angles around a vertex tile exactly
360°, so a fourth regular triangle either continues the fan (240°, ring
open) or overlaps it. A red closed-ring label is therefore a 3-D phenomenon
(the apex of a square pyramid, 240°), the plane case is an *overlap* and is
refused as such, and the test for the former uses exact 3-D geometry. Two
corrections fell out of the same scrutiny: the corner the new panel adds must
be the signed interior angle about the sketch's winding normal (an L closing
the ring around a square has a 270° corner, which the unsigned angle reads as
90° and colours red), and an edge is a crease between two panels only — the
automatic join put a second crease on an edge that already was one when a
third panel was laid on it.

**Constraints without variable columns, again (v0.4).** The sketch-plane
constraint of a 2-D link is compiled into frozen coordinates, but a vertex
merged with a constant point (a ground vertex, a datum-point pin) is not a
variable, and its distance from the plane was dropped: a 2-D polygon pinned
to a ground vertex off its plane read violation 0 and the pre-flight solve
accepted it. The cure is the one of "Never commit a non-converged pose"
above: emit the residual without columns so that the chip and every verified
edit see it. The Polygon tool now treats a snapped off-plane vertex as
authoritative instead (the polygon tilts to meet it and carries no 2-D
constraint), so the refusal is rarely reached.

**Check the plan's arithmetic (v0.4).** The plan's example of a Kawasaki
failure, sectors (90°, 90°, 60°, 120°), has an alternating sum of 60°, not
the 20° its text stated; the test asserts 60° and adds (90°, 90°, 100°, 80°)
for a 20° defect. Headless steps were wrong before the code was, too: an
"empty" spot of the canvas lay inside the perspective wedge of the RIGHT
plane, and an out-and-back orbit returns the camera to its start, so the
orbit has to be measured mid-drag.

## 9. Known limitations (prototype)

- Patterning copies one link at a time; joints between copies are not
  replicated, and copy / paste / mirror / pattern build the new link's
  rigidity from the visible geometry without a consistency check.
- Dense linear algebra: systems beyond ~1000 variables (large origami
  patterns × many poses) will be slow; reduce `poseCount` or lock panels.
- Design space is first-order (tangent) rather than the true reachable set.
- Prescribed timing only (see §4).
- Prism and cylinder internal geometry is treated as fixed during inverse
  design (only bars and polygons change shape); cylinders are rigid.
- Branch handling in sweeps is heuristic (displacement guard).
- The Fold command pre-folds one vertex (one crease loop) per press, and only
  loops around a single vertex are detected; a vertex where more than one
  cycle of panels meets is skipped, and the vertices of a multi-vertex sheet
  are not folded consistently with each other.
- "Fold to target" follows one continuation path; a crease whose other
  constraints admit several branches may be reported unreachable although a
  different path exists.
- The sector-sum diagnosis counts only polygon / prism panels (bars
  contribute no angle); the needs-3-D probe removes only the sketch-plane
  constraints of the links in the loop.
- The locked-creases hint ignores hinges between edges of different length
  and creases whose panels are kept on two different planes.
- Mountain / valley classes of creases whose first panel is a bar or a
  cylinder use the cross-product normal of the stored edge, so they can flip
  with the edge direction.
- The query cycle lists what the exact ray and a 10 px ring of rays hit; a
  feature farther from the pointer is not offered. The Joint tool's
  other-link substitution looks only in the front depth band and never at
  datum geometry.
- The Panel tool's sector labels appear only at sketch vertices that sit on
  existing vertices, and the overlap refusal applies only while the sketch
  and the panels around the vertex are coplanar (a red closed ring in 3-D is
  joined as a folded vertex). The Edit tool still reports a refused
  automatic join silently.
- The crease-pattern validator reports single-cycle crease loops only (as
  the Fold command detects them), skips loops through bars or cylinder axes,
  reads mountain / valley classes from the construction pose and checks the
  local conditions only — no global flat-foldability (layer ordering,
  self-intersection).
- "Place on sketch plane (2-D)" is one setting for every click-placed tool;
  there is no per-tool placement plane, and no tool places on the view plane
  except as the fallback.
- No collision detection, no dynamics.

## References

- K. Suto, Y. Noma, K. Tanimichi, K. Narumi, T. Tachi. *Crane: An Integrated
  Computational Design Platform for Functional, Foldable, and Fabricable
  Origami Products.* ACM Trans. Comput.-Hum. Interact. 30(4), art. 52, 2023.
  https://doi.org/10.1145/3576856
- T. Tachi. *Simulation of Rigid Origami.* In Origami 4 (4OSME), 2009.
- D. A. Huffman. *Curvature and Creases: A Primer on Paper.* IEEE Trans.
  Computers C-25(10), 1976 (degree-4 vertex folding, straight-hinge
  degeneracy).
- A. Ghassaei, E. Demaine, N. Gershenfeld. *Fast, Interactive Origami
  Simulation using GPU Computation.* In Origami 7 (7OSME), 2018.
- R. J. Lang. *Twists, Tilings, and Tessellations: Mathematical Methods for
  Geometric Origami.* CRC Press, 2018 (Kawasaki and Maekawa conditions,
  mountain / valley conventions).
- E. D. Demaine, J. O'Rourke. *Geometric Folding Algorithms: Linkages,
  Origami, Polyhedra.* Cambridge University Press, 2007 (developability:
  sector angles sum to 2π; flat-foldability conditions).
- T. Hull. *On the Mathematics of Flat Origamis.* Congressus Numerantium
  100, 1994, 215–224 (Kawasaki's and Maekawa's theorems for a flat-foldable
  vertex: the alternating sector sum vanishes; mountains and valleys differ
  by two).
- T. Kawasaki. *On the Relation between Mountain-Creases and
  Valley-Creases of a Flat Origami.* In H. Huzita (ed.), Proceedings of the
  First International Meeting of Origami Science and Technology, 1989,
  229–237.
- E. L. Allgower, K. Georg. *Numerical Continuation Methods: An
  Introduction.* Springer, 1990 (predictor steps, branch switching at
  bifurcations).
- M. Bächer, S. Coros, B. Thomaszewski. *LinkEdit: Interactive Linkage
  Editing using Symbolic Kinematics.* ACM Trans. Graph. 34(4), SIGGRAPH 2015.
- S. Coros, B. Thomaszewski, G. Noris, S. Sueda, M. Forberg, R. Sumner,
  W. Matusik, B. Bickel. *Computational Design of Mechanical Characters.*
  ACM Trans. Graph. 32(4), SIGGRAPH 2013.
- B. Thomaszewski, S. Coros, D. Gauge, V. Megaro, E. Grinspun, M. Gross.
  *Computational Design of Linkage-Based Characters.* ACM Trans. Graph. 33(4),
  SIGGRAPH 2014.
- C. W. Wampler, A. P. Morgan, A. J. Sommese. *Complete Solution of the
  Nine-Point Path Synthesis Problem for Four-Bar Linkages.* J. Mech. Des.
  114(1), 1992.
- J. M. McCarthy, G. S. Soh. *Geometric Design of Linkages*, 2nd ed.
  Springer, 2011.
- R. L. Norton. *Design of Machinery*, McGraw-Hill (Grashof condition,
  Grübler–Kutzbach mobility, four-bar classification).
- L. L. Howell. *Compliant Mechanisms.* Wiley, 2001 (pseudo-rigid-body model).
- J. Nocedal, S. J. Wright. *Numerical Optimization*, 2nd ed. Springer, 2006
  (Levenberg–Marquardt, ch. 10).
