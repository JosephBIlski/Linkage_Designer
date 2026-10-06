# Design decisions, assumptions and references

This document records *why* the prototype is built the way it is. The
feature-level behaviour is specified in [SPEC.md](SPEC.md); the original brief
is in [ORIGINAL_SPEC.md](ORIGINAL_SPEC.md).

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
  for mechanisms with up to a few hundred variables.
- The projection uses Levenberg–Marquardt (Nocedal & Wright, ch. 10) rather
  than plain Gauss–Newton because the Jacobian is rank deficient *by design*
  (the mechanism has motion freedom); the damping yields the minimum-norm step
  and tolerates singular poses.
- Residuals are all in length units. Angular constraints (drivers, dihedral
  angles, helper attachments) are multiplied by a characteristic length so one
  tolerance works for everything.

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
- **Revolute between edges** (origami crease) merges the two edges' end
  points when the edges have equal length, otherwise uses two
  point-on-line constraints plus a slide lock (5 equations).
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
  frees the link into space.
- Editing points, output paths and the design space are coloured exactly as
  specified; every colour has an HWB picker with a HEX field and is persisted
  in `localStorage`. Defaults live in `src/ui/settings.ts`.
- All strings live in `src/ui/strings.ts`.

## 8. Lessons from testing (v0.2)

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

## 9. Known limitations (prototype)

- Patterning copies one link at a time; joints between copies are not
  replicated.
- Dense linear algebra: systems beyond ~1000 variables (large origami
  patterns × many poses) will be slow; reduce `poseCount` or lock panels.
- Design space is first-order (tangent) rather than the true reachable set.
- Prescribed timing only (see §4).
- Prism and cylinder internal geometry is treated as fixed during inverse
  design (only bars and polygons change shape); cylinders are rigid.
- Branch handling in sweeps is heuristic (displacement guard).
- No collision detection, no dynamics.

## References

- K. Suto, Y. Noma, K. Tanimichi, K. Narumi, T. Tachi. *Crane: An Integrated
  Computational Design Platform for Functional, Foldable, and Fabricable
  Origami Products.* ACM Trans. Comput.-Hum. Interact. 30(4), art. 52, 2023.
  https://doi.org/10.1145/3576856
- T. Tachi. *Simulation of Rigid Origami.* In Origami 4 (4OSME), 2009.
- A. Ghassaei, E. Demaine, N. Gershenfeld. *Fast, Interactive Origami
  Simulation using GPU Computation.* In Origami 7 (7OSME), 2018.
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
