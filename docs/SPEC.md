# Linkage Designer — detailed feature specification (prototype v0.4)

This document specifies each implemented feature in detail. Each section
quotes the relevant line of the [original specification](ORIGINAL_SPEC.md)
and marks **Deviations** where the prototype departs from it (with the
reason). Rationale and references are in
[DESIGN_DECISIONS.md](DESIGN_DECISIONS.md); the diagnosis and the plan behind
the v0.3 joining / folding work (§17) and the v0.4 construction workflow
(§18) are in [CONSTRUCTION_PLAN.md](CONSTRUCTION_PLAN.md).

Conventions: world units are unit-less (the grid step is 1); angles are shown
in degrees; the default up axis is +Z and the default sketch plane is TOP (XY).

---

## 1. Application shell

**Original:** "a UI similar to CAD software (such as Creo or Rhino) with a 3D
editing window along with tools …"

- Layout: top bar (brand, mode tabs, File / Examples / View menus, undo /
  redo, zoom-to-fit, settings, help), left tool palette with a contextual
  **Tool options** panel, central 3-D viewport, right side with the
  **Mechanism / Simulation / Preview** panel and the **Properties** panel,
  bottom status bar with hints, a DOF chip and the coordinate entry box.
- Viewport: light grey background (`#e4e6e8`), 40×40 grid in the TOP plane
  with contrasting minor (`#c3c7cb`) and major (`#9aa0a6`) lines, hemisphere +
  directional lighting, labels (CSS overlays) for links, vertices of bars,
  displayed points, construction geometry and hovered joints, and the Panel
  tool's sector-angle labels (§18.2), which are drawn above the others.
- Mouse: left-click picks with the active tool (the feature under the
  pointer is highlighted and named in the status bar as the pointer moves);
  **right-click without dragging** cycles through everything under the
  pointer (the *query cycle*, §18.1); right-drag orbits (in every view, the
  Top view included), middle-drag or Shift+right-drag pans, wheel zooms to
  the cursor. The browser's context menu is suppressed over the viewport.
  View menu: Top / Front / Right / Isometric, zoom to fit (`F`),
  orthographic / perspective.
- Undo / redo (`Ctrl+Z`, `Ctrl+Y` / `Ctrl+Shift+Z`) by whole-model snapshots;
  `Esc` during a drag restores the pre-drag model without touching history.
  A refused edit (§17.1) records no undo entry.
- Status bar: shows the last status message or the tool hint — the feature
  under the pointer as it moves, and the query cycle's count and
  instructions while one runs (§18.1); its tooltip carries the whole text,
  since long diagnoses are clipped in the bar.
  Refusals (a refused joint, edit, fold, Solidify or Save) are also kept as a
  **notice** in the Mechanism panel, with a *Dismiss* button, until dismissed
  or until the next change goes through (undo, redo and loading a file clear
  it as well).
- Keyboard: `Esc` ends a running query cycle, otherwise cancels the tool;
  `Del` delete selection; `1` select, `2` link, `3` polygon, `4` prism,
  `5` cylinder, `S` panel (sketch polygon), `E` edit points, `G` ground,
  `J` joint, `D` driver, `M` mirror, `P` pattern, `F` zoom to fit;
  `Enter` closes the panel being sketched (on the canvas or in the empty
  coordinate box) and `Backspace` removes its last vertex; `Ctrl+C` /
  `Ctrl+V` copy / paste a link; `Space` play/pause in Preview.
- All user-visible text is in `src/ui/strings.ts`; colours and display
  defaults in `src/ui/settings.ts` (`DEFAULT_SETTINGS`).

## 2. Modes

**Original:** Construction / Simulation / Preview modes as described.

| Mode | Purpose | What is shown |
| --- | --- | --- |
| Construction | Build and edit the mechanism | Links, joints (creases as mountain / valley lines, §17.2), construction geometry, ground symbols, live DOF, output paths of displayed points (if a driver exists) |
| Simulation | Inverse design | Everything above plus design spaces, editing points, ghost of the mechanism at a selected/hovered pose, design DOF counters |
| Preview | Motion playback | Mechanism at the slider's input value, traces of displayed points |

Switching to Simulation or Preview automatically adds a default angle driver
(the first link with a pin to the ground link or to construction geometry)
when none exists. Editing tools are disabled outside Construction mode (the
**Fold** command of §17.3 is the one exception: it is offered in every mode).

## 3. Links

### 3.1 Bars ("2D bar links")

**Original:** "Creating … a 4 bar link, should be done by selecting a 'Link'
tool, and clicking points in the 3D space … Clicking on the end of one link
should constrain that link to the newly created one. Links should also be able
to be placed by specifying a second point through coordinates, length and
angle, or just a length and clicking to place the second point."

- Tool **Link**: click the first point, then the second. A free click lands
  where the pointer ray meets the active sketch plane, whether or not *Place
  on sketch plane (2-D)* is ticked; the option decides whether the bar is
  then *kept* on that plane (§17.4). Only when the ray grazes the plane
  (3-D mode), misses it, or meets it at the camera is the click placed on
  the view-aligned plane through the previous point (§18.3). Optional grid
  snapping.
- Snapping: hovering an existing vertex (or construction point) snaps and
  shows a ring; clicking it joins the new end to that vertex with the
  **default joint** (Revolute, axis = sketch-plane normal; Spherical can be
  chosen in the tool options). Joining to a construction point pins the end.
- Coordinate entry (status bar, `Enter` applies) while a point is awaited:
  - `x,y` or `x,y,z` — absolute coordinates;
  - `@dx,dy[,dz]` — relative to the previous point;
  - `L<angle` — length and angle (degrees) in the sketch plane from the
    previous point;
  - `L` — length only: the next click sets the direction, the end is placed
    at distance `L`.
- A bar is two visible points with one design distance (its length). Length
  is editable in Properties.

### 3.2 Polygons (2-D) and prisms (3-D polygons)

**Original:** "2D and 3D Polygons with a specifiable number of sides. Each
vertex/surface/side should be able to be constrained with compatible
constraints."

- Tool **Polygon**: click the centre, then a point on the circumcircle;
  *Sides* (≥ 3) in the tool options. A regular n-gon is created in the sketch
  plane; vertices can then be dragged to any (planar) shape.
- Tool **Prism (3-D polygon)**: same input plus *Height*; a right prism with
  2n vertices, two end faces and n side faces.
- **Automatic joins** (since v0.4): every vertex of the new polygon or prism
  that lands on a vertex of another link — the snapped circumcircle point,
  and any further vertex that the regular shape puts on an existing vertex
  (typed coordinates, grid snapping) — is joined with the rules of §13.1:
  two coincident vertices on one existing edge give a crease, a single one a
  pin, and the status bar reads "Joint created (n)". The clicked
  circumcircle point stays joined to the vertex it was snapped to even when
  rounding leaves it slightly off, and the join is re-solved and verified
  like every other edit (§17.1); a refused re-solve leaves the link unjoined
  and says why ("The link was created, but its snapped vertices were not
  joined: …"). When the snapped circumcircle point lies off the sketch plane
  (the top vertex of a prism, say) the polygon or prism is built in the plane
  through its centre and that vertex that is closest to the sketch plane, so
  the vertex is met exactly; a polygon then carries no 2-D constraint and
  the status bar adds "The snapped vertices define a plane off the sketch
  plane; the polygon was placed on that plane (no 2-D constraint)." Until
  v0.3 only the circumcircle point was pinned, without a solve.
- Features: every **vertex**, **edge** (side) and **face** is pickable and can
  be used by the Joint tool (see §5 for compatibility).
- Rigidity: frame-triangle rule (3 distances for the frame, 3 distances or
  2 distances + coplanarity per further vertex). Polygon shapes are design
  variables in inverse design; prism shapes are fixed.

**Deviation:** "3-D polygon" is interpreted as a right prism (an extruded
polygon). Arbitrary polyhedra are not provided.

### 3.3 Cylinders

- Tool **Cylinder**: click the axis start, then the axis end; *Radius* in the
  tool options. Features: the two axis end points (vertices) and the **axis**
  (for revolute / cylindrical / prismatic / screw joints) and one end-cap face.
- Internally a rigid body of 4 points (axis ends + two hidden surface
  reference points, which also fix roll for prismatic and screw joints).

### 3.4 Link properties (Properties panel)

- Name, type, length (bars), sides / radius / height (read-only after
  creation), colour override with reset to the default colour. A new bar
  **Length** is re-solved through the pre-flight of §17.1: when the mechanism
  cannot be re-assembled with it (a bar in a closed loop given an impossible
  length, a bar pinned at both ends) the model returns to its state before the
  edit and the status bar reads "The constraints cannot all be satisfied with
  this change, so it was not applied."
- **Locked** — "preventing changes to the link in any mode": a locked link
  cannot be dragged, its ends cannot be rubber-banded and its geometry is held
  fixed during inverse design (its lengths are not design variables; a locked
  ground link keeps its pivots). A locked panel can neither adapt its edge to
  a crease partner (§17.1) nor be pre-folded (§17.3).
- **Ground link** — see §6.
- **Keep on sketch plane (2-D)** (bars and polygons only) — whether the link
  carries the sketch-plane constraint that 2-D mode gives new geometry, with
  the plane's name ("on TOP"); see §17.4.
- **Flexible** + **Relative stiffness** — see §9.
- **Show output path & design space for** — per-vertex toggles adding the
  vertex to the displayed points (also available on a selected vertex).
- Delete.

### 3.5 Editing links in Construction mode

**Original:** "Links should also be able to be dragged and relocated by the
user, as well as their properties (such as constraints) edited after
creation."

- Dragging a link body translates it; all other links follow through their
  joints (a constraint re-solve with the dragged points as weak soft targets,
  so joints stay satisfied during the drag). Dragging the ground link moves
  the whole mechanism. Planar links drag in their sketch plane (in the view
  plane when the sketch plane is seen edge-on), others in the view plane.
- Picking is tolerant: bars and edges can be grabbed within about 10 px, and
  datum planes / axes never steal a pick from model geometry; where features
  coincide, a right-click cycles through them (§18.1). Clicking on empty
  space or a datum with the Select tool explains what can be dragged.
- Dragging a link **end** rubber-bands it: the link's own design distances
  touching that vertex are released, so its length / shape changes while all
  joints stay satisfied. Releasing the end on another link's vertex joins them
  with the default joint. The new geometry becomes the design on release.
- Releasing a drag (of a vertex or of a whole link) runs an exact re-solve
  that is verified like every other edit (§17.1): if the constraints cannot
  all be met, the model returns to its pre-drag state and the status bar says
  so; a violated pose is never committed and never becomes the design.
- Vertex position fields in Properties do the same numerically, with the same
  verification.

## 4. Link-end pop-up

**Original:** "link ends (when clicked) should have a small pop-up with 2
side-by-side icons, one with a symbol representing the current type of
constraint, and the other with an 'X', allowing for the constraint to be
removed. Links should also have a 'lock' toggle."

- Clicking a link end in Select mode selects the vertex and opens a pop-up
  anchored to it with: (1) the glyph of the constraint at that end (S / R /
  planar / P / C / screw, or a dashed circle for *none*) — clicking it cycles
  the joint to the next compatible type, through the same verification as the
  Joint tool (§17.1): types the two features cannot take are skipped, and a
  type with which the mechanism cannot be re-assembled stops the cycle with
  the explanation and leaves the joint exactly as it was; (2) an **X** that
  removes the constraint (disabled when there is none); and (3) a lock icon
  toggling the link's *Locked* flag. When the link is kept on a sketch plane
  the glyph's tooltip adds "on sketch plane TOP" (that constraint is a link
  property, §17.4, not a joint at the point). The pop-up follows the point
  when the view moves and closes on `Esc` or the next click.

**Deviation:** a third icon (lock) was added to the two specified ones because
the brief asks for a lock toggle on links and the pop-up is the quickest place
for it; the lock is also in Properties.

## 5. Joints / constraints

**Original:** spherical, revolute, planar, prismatic, cylindrical, screw;
"similar in spirit to … Crane".

Tool **Joint**: choose the type in the tool options, click a feature of the
first link, then a compatible feature of a second link or construction
geometry. The geometry is immediately re-solved so the new joint is satisfied
(the second link snaps). Since v0.3 the re-solve is verified before the joint
is kept: a joint that the existing constraints cannot satisfy is **refused**,
nothing moves, no undo entry is recorded and the status bar explains why
(§17.1). Properties of a joint: connected items, change type (verified the
same way: an incompatible or unsatisfiable type leaves the joint as it was),
axis, pitch (screw), compliant-hinge settings (revolute), delete; a crease has
the additional controls of §17.2.

Picking the second feature where edges or vertices coincide (since v0.4):
when the feature under the pointer belongs to the first link again (two
panels drawn edge on edge, a shared vertex), the tool uses the best-ranked
compatible feature of *another* link at the same spot — among the features
within the pick tolerance of the nearest hit, never geometry hidden behind
the clicked panel and never datum geometry — and prefixes its status with
"Used edge V0-V1 of Polygon 2 (the feature under the pointer was on the first
link)"; only when there is none does it say "That feature belongs to the
first link too. Pick a feature on a different link (right-click cycles
through everything under the pointer)." A feature named explicitly through
the query cycle (§18.1) is never substituted; the cycle steps through every
feature under the pointer, so either of two coincident edges can be chosen.

| Joint | DOF | Compatible features (A ↔ B) | Equations |
| --- | --- | --- | --- |
| Spherical | 3 | vertex ↔ vertex; vertex ↔ datum point | coincidence (merged variable) |
| Revolute | 1 | vertex ↔ vertex (axis = sketch normal, helper points carry the pin); edge/axis ↔ edge/axis (a **crease** when the lengths agree within one part in a thousand: end points paired by proximity and merged; otherwise a **hinge**: the second edge oriented like the first, its end points on the first edge's line, slide locked at its value in the current pose); vertex ↔ edge/axis; vertex/edge/axis ↔ datum axis; vertex ↔ datum point (pin at a fixed point) | 5 |
| Cylindrical | 2 | edge/axis ↔ edge/axis; vertex ↔ edge/axis; … ↔ datum axis | 4 (two point-on-line) |
| Prismatic | 1 | as cylindrical | 4 + reference point in a plane containing the axis |
| Screw | 1 | as cylindrical, *Pitch* per revolution | 4 + slide − pitch·rotation/2π |
| Planar | 3 | face ↔ face; vertex/edge ↔ face; vertex/edge/face/body ↔ datum plane | 1 per constrained point (3 for a face) |

The edge ↔ edge (and cylinder axis) matching rules, the adaptation of one
panel's edge to its partner's length, and the feasibility gate with its
diagnoses are specified in §17.1; the order in which an edge was drawn or
picked never matters. When a new crease closes a flat loop of panels the
status bar adds "The vertex is flat: use Fold to pre-fold it before
simulating." (§17.3).

Internals: joints that merge points reduce the variable count; axis-aligned
datum planes freeze coordinates. Every joint contributes to the numerical DOF
(rank of the constraint Jacobian), so redundant / parallel constraints are
counted correctly.

## 6. Ground link and degrees of freedom

**Original:** "specifying a ground link (that can be later changed). The user
should be able to see the degrees of freedom of their mechanism."

- Tool **Ground** (or the Properties checkbox): one ground link at a time;
  clicking the current ground un-grounds it. Ground links are drawn in the
  ground colour with hatch cones under their vertices.
- The status-bar chip and the Mechanism panel show **DOF = variables − rank**
  of all hard constraints (drivers excluded), recomputed after every edit. With
  no ground link and nothing pinned to construction geometry the count includes
  the rigid-body modes (3 planar / 6 spatial) and says so. A "Constraints
  violated" flag appears when the current pose misses the constraints by more
  than the consistency tolerance of §17.5 (the same tolerance that refuses
  Solidify, Save and Extrude, so the chip warns exactly when they would).
  Since every construction tool refuses a pose that violates its constraints
  (§17.1), the flag can now only result from loading a file, a save without
  solidifying, or inverse design.
- When the mechanism has creases that cannot fold because both of their panels
  are kept on the same sketch plane, the Mechanism panel says so ("4 creases
  cannot fold: both panels are kept on the sketch plane. Untick 'Keep on
  sketch plane' in Properties, or use Fold.") and the chip carries the same
  text as its tooltip; see §17.4. A flat origami vertex is also announced
  there, with the Fold button (§17.3).
- Every **interior vertex** (a vertex whose panels close a ring of creases)
  is listed in a *Crease pattern* block under the hints, with its
  developability, Kawasaki and Maekawa checks (§18.4).

## 7. Construction geometry (datums)

**Original:** "construction geometry (axes, planes, points) that can constrain
the mechanism geometry … similarly to Creo's geometry creation workflow."

- Built-in datums: ORIGIN, axes X/Y/Z, planes TOP / FRONT / RIGHT (cannot be
  deleted).
- Tools: **Datum point** (click / type coordinates), **Datum axis** (two
  points), **Datum plane** (*Through 3 points*, or *Offset from a plane*:
  click a base plane with an offset value). Any plane can be made the active
  **sketch plane** from its Properties; new 2-D geometry is then placed on it.
- Datums constrain geometry through joints (vertex on datum point, revolute /
  cylindrical / prismatic / screw about a datum axis, planar on a datum plane)
  and through **editing-point targets** (§8.5).
- Datums are rendered in the construction colour; large translucent planes
  never steal picks from real geometry. A datum plane or axis is highlighted
  only when it is named explicitly — through the query cycle (§18.1) or a
  model-tree row — since the idle pointer rests on the sketch plane wherever
  the canvas is empty (the status bar names it, nothing lights up).

**Deviation:** Creo's full datum feature set (plane through axis at angle,
tangent planes, etc.) is reduced to the three creation methods above.

## 8. Simulation mode and inverse design

### 8.1 Drivers and motion sampling

- A **driver** is the mechanism input: *angle* of a link about a grounded pin
  (Driver tool on a link), *fold* (dihedral) angle of a revolute crease
  (Driver tool on the joint) or *slide* of a prismatic / cylindrical / screw
  joint. Several drivers can exist; the Simulation panel chooses the active one
  and reports its **range**: *full rotation (crank)* or *limited range
  (rocker)*, discovered by sweeping until assembly fails.
- Driver tool on a crease of a **flat** origami vertex: the vertex is
  pre-folded first, in the same undo step, and the status bar says so
  ("Driver added · The flat vertex was pre-folded first so that every crease
  moves."); without the pre-fold a sweep from the flat state would fold only
  two creases (§17.3). A crease never carries two fold drivers: picking a
  crease that already has one makes that driver the active one ("This crease
  already has a fold driver; it is now the active one."), and picking another
  crease of a vertex whose only driver is a fold driver on that vertex moves
  the driver to the picked crease (§17.2).
- `Sampled poses` (default 16) are spread over the range; each sample is an
  **editing point** on every displayed point's path.

### 8.2 Output path and design space

**Original:** "The 'design space' should show all of the possible points a
selected link or vertex … can reach. … a four bar linkage … a line for the
current output motion, while the 'design space' would be a 2D surface, while a
spherical joint could show a 2D surface as the current output path, with the
'design space' as a 3D sphere."

- Output path: the trajectory of each displayed point over the sweep, drawn
  as a tube in the output-path colour (`#9a0062`, 90 % opacity by default).
  For 2-DOF mechanisms the output path is a **surface** sampled on a grid of
  the two drivers and drawn as a translucent mesh with iso-lines (spherical
  pendulum example).
- Design space: for each displayed point the dimension and principal
  directions of the first-order design space (null space of the stacked
  design system projected onto the point), drawn in `#c7007e` at 30 %
  opacity as a segment (1-D), disc (2-D) or ball (3-D) with an extent of about
  1.1 × the current reach. The panel states the dimension in words. A
  four-bar coupler point reports *surface (2-D)*; the spherical pendulum's
  end reports *volume (3-D)* once its output surface is included.

**Deviation:** the design space is a first-order (tangent) estimate of the
reachable set with a display extent; the exact reachable set under all
admissible link lengths is unbounded and is not computed. See
DESIGN_DECISIONS §4.

### 8.3 Editing points

**Original:** "draggable (and lockable) 'editing points' … The mechanism should
change to enable the path that the user has dragged or constrained."

- Editing points are spheres along the output path: `#0eb062` when free,
  `#94140a` when constrained; a locked point shows a dark ring.
- Dragging a point creates / updates a **position target** for (point, pose)
  and re-solves the design live; on release the full sweep and analysis are
  refreshed. The drag is constrained to the point's local design directions
  (line / plane); hold `Shift` to drag freely. A fully determined point cannot
  be dragged.
- Hovering or selecting an editing point shows the mechanism at that pose as
  a translucent ghost.
- Properties of an editing point: pose index and input value, the target
  (editable coordinates), **Locked**, **Release**, and **Constrain to
  construction geometry** (§8.5).
- *Release all editing points* clears every target.

### 8.4 Design freedom counters and soft assumptions

**Original:** "The user should be able to see how many degrees of freedom they
have left, and how many editing points are constrained. … toggle to allow
'soft-assumptions' to enable a preview … Link and polygon geometry … should
not change throughout the motion path."

- **Design DOF left** = nullity of the stacked system (minus un-driven motion
  freedom). **Editing points constrained** = number of targets. "Design fully
  specified" appears at 0; "Targets cannot all be met" appears when the solve
  does not converge (over-constrained / unreachable).
- Link lengths and polygon shapes are shared across all poses, so they never
  vary along the motion; ground pivots are design variables unless the ground
  link is locked.
- **Soft assumptions** (on by default): remaining freedom is resolved by
  staying closest to the current design and the preview is always shown. Off:
  the output path is hidden until design DOF = 0 (editing points stay
  visible so the design can be completed).
- **Solidify assumptions** commits the current geometry as the rest geometry;
  *Save* does this automatically. Both refuse a pose that violates the
  constraints (§17.5).
- Timing is prescribed: each editing point keeps its input value (noted in the
  panel).

### 8.5 Constraining editing points to construction geometry

**Original:** "The editing points should also be able to be constrained to
construction geometry."

- In the editing point's Properties choose a datum: a datum **point** pins the
  point (3 equations), an **axis** constrains it to a line (2), a **plane** to
  a plane (1). Such targets are shown as constrained points and count toward
  the design freedom like any other target. Targets referencing a deleted datum
  are removed with it.

## 9. Flexibility (compliant links and hinges)

**Original:** "Geometry should have an option to enable 'flexibility', enabling
the simulation of bistable/compliant shapes, and editable stiffness."

- Link property **Flexible** with **Relative stiffness** (0.02–1): the link's
  design distances act as springs in forward simulation and preview (weighted
  residuals, hard constraints dominate ≈ 400:1 at stiffness 1). Over-closed
  or bistable assemblies then simulate as the nearest quasi-static equilibrium
  and snap through dead points.
- Revolute joint property **Compliant hinge**: torsional spring with *Rest
  angle* and *Hinge stiffness* (origami creases, living hinges).
- Flexibility is ignored by the inverse-design solve (links are treated as
  rigid when solving for a design).

**Deviation:** the simulation is quasi-static (energy minimisation), not a
dynamic simulation; stiffness is relative, not in physical units.

## 10. Preview mode

**Original:** "A mode for previewing the motion of the mechanism, through the
range of a selected link."

- Slider over the active driver's range with numeric read-out, Play / Pause
  (`Space`), speed (units per second), trace toggle. Cranks loop, rockers
  ping-pong. The driver is chosen in the panel (a link's angle = "the selected
  link"). Creases change colour as they fold (§17.2).

## 11. Settings

**Original:** "Created user geometry should default to #35a4d3 … customizable
with a HWB color picker in a settings menu … colors should be editable in the
settings menu, and the color picker used should also display a HEX value."

- Settings dialog with an HWB picker (hue / whiteness / blackness sliders +
  editable HEX field + swatch) for: default geometry (`#35a4d3`), ground link,
  construction geometry, design space (`#c7007e`), output path (`#9a0062`),
  editing point free (`#0eb062`) / constrained (`#94140a`), background, grid
  major / minor, selection highlight, mountain crease (`#d9342b`) and valley
  crease (`#2f6fd6`); design-space opacity (default 0.30); show labels /
  construction geometry / hidden helper points; grid snap and step; joint
  helper offset; *Reset to defaults*. Settings persist in `localStorage`
  (stored settings that predate the crease colours receive their defaults).

## 12. Files, examples and export

**Original:** "Saving or exporting the mechanism file should solidify these
assumptions."

- **Save** writes `<name>.linkage.json` (the full model: links with rest
  geometry, joints, datums, drivers, targets, settings) after solidifying the
  current design — provided the pose is consistent. A pose that violates the
  constraints is saved as it is (the previous rest geometry together with the
  violated positions, so reloading shows the same violated pose and the chip
  warning) and the status bar reads "Saved without solidifying: the current
  pose violates the constraints (gap … units), so the file keeps the previous
  design geometry together with the violated positions." (§17.5). **Open**
  loads such a file. Targets stay in the file and are re-imposed on load.
  Files written before v0.3 load unchanged (the crease fields of §17.2 are
  optional, and a missing slide offset still means 0).
- **Export**: output paths as CSV (point, pose index, input value, x, y, z),
  geometry of the current pose as OBJ, the viewport as PNG. Exports never
  solidify.
- **Examples** menu: four-bar crank-rocker, slider-crank, spherical pendulum,
  rigid-origami Miura vertex with sector angles (α, α, π−α, π−α), created in
  an exactly folded configuration so it folds on the generic branch
  (`examples/*.linkage.json` are the same files).

**Deviation:** saving no longer solidifies a violated pose (it would turn the
violation into the design); the file keeps the last consistent design and the
status bar says so.

## 13. Sketching and editing geometry (added in v0.2)

### 13.1 Panel tool (sketch polygon) and extrusion

- Tool **Panel (sketch polygon)** (`S`; called *Sketch polygon* until v0.3):
  click any sequence of vertices on the active sketch plane (points are
  projected onto the plane, so the polygon is always planar); a dashed
  preview follows the pointer. Close the panel by clicking the first vertex
  again, double-clicking, or pressing `Enter` (on the canvas or in the empty
  coordinate box); `Backspace` removes the last vertex and the preview drops
  it at once. At least three non-collinear vertices are required.
  Coordinates can also be typed in the status bar.
- **Snapping and joining**: every vertex of the panel that lies on an
  existing vertex of another link is joined when the panel is closed — a
  vertex snapped onto it and, since v0.4, also a vertex typed in the
  coordinate box or placed by a grid-snapped click that lands on one within
  the coincidence tolerance (one part in a million of the model size, never
  below 1e-6 units; the tolerance of the automatic join). Such a vertex is
  treated exactly like a snapped one: it takes the existing vertex's
  position (a typed 3-D coordinate on a vertex off the sketch plane is not
  projected onto the plane) and defines the panel's plane together with the
  other snapped vertices. Two consecutive such vertices that coincide with
  an edge of one existing link become a single edge–edge revolute joint (a
  crease); an isolated one gets the default joint (hinge axis = the normal of
  the sketch plane both links share) when both links share a sketch plane,
  otherwise a spherical joint; a vertex already merged with the other link's
  vertex (directly, or through the creases and pins of its neighbours) is
  not pinned again. An edge is a crease between two panels only: a third
  panel laid on an edge that is already a crease gets a pin at a shared
  vertex where one applies, never a second crease on that edge. A vertex
  snapped onto a datum point is pinned to it. Free vertices of a panel whose
  snapped vertices define a plane off the sketch plane are placed where the
  click ray meets that plane (the placement rule of §18.3, with the
  orthogonal projection as its fallback). This is how the last panel of an
  origami vertex is "filled in": sketch it by clicking the existing
  vertices. Edges that coincide only within the snap tolerance become
  creases whose sketched edge takes the existing edge's length (the panel
  being sketched adapts, never the existing geometry; §17.1). The status bar
  reads "Joint created (n)". The re-solve for the automatic joints is
  verified like every other edit: when it is not accepted, the panel is
  created unjoined and the refusal is reported in the status bar and as a
  Mechanism-panel notice ("The link was created, but its snapped vertices
  were not joined: <diagnosis of §17.1>").
- **Sector-angle labels and the overlap refusal** are specified in §18.2:
  while the panel is drawn, every vertex that sits on an existing vertex
  shows the running sum of the corner angles around it, and a panel that
  would overlap its neighbours in their plane is not created.
- **Extrude to prism** (Properties of a polygon): enter a height and press
  *Extrude*. The polygon becomes a prism of that height along the normal of
  its sketch plane (so the direction does not depend on the vertex winding;
  negative = other side). Non-planar polygons cannot be extruded, nor can a
  polygon whose pose violates the constraints (§17.5). The bottom face keeps
  its point ids so joints attached to the polygon's vertices / edges / face
  stay valid; the sketch-plane constraint is removed because the body is now
  3-D. The prism's rigidity is rebuilt from the new positions (joint helper
  attachments kept).

### 13.2 Edit tool (snap vertices)

- Tool **Edit points** (`E`): click a vertex of existing geometry, then click
  its destination: another link's vertex (the two are joined with the default
  joint type shown in the tool options), a datum point, any position on the
  sketch plane (placed by the rule of §18.3, in 3-D mode too), or typed
  coordinates.
- The vertex is first made a dependent point of its link's rigidity (so the
  rest of the link keeps its shape), then released from its own shape
  constraints, every other constraint is re-solved, and the link's rest
  geometry is rebuilt from the result. If the destination lies off the link's
  sketch plane, that planar constraint is removed first, so a flat panel can
  be lifted into 3-D. If the destination cannot be reached (the vertex is
  pinned to a datum point, or its joints forbid it) nothing changes and the
  status bar says so. Joint helper attachments that depend on the moved vertex
  are released and rebuilt, so a hinged bar or panel can be lifted out of its
  plane. Coincident vertices are joined with the same rules as the Panel tool
  (shared edges become creases, replacing earlier pins; the edited link adapts
  when the edge lengths differ slightly; a refused re-solve of the automatic
  joints leaves the vertex moved but unjoined); a vertex dropped on a datum
  point is pinned to it. Locked links cannot be edited.

**Deviation / note:** moving one vertex of a quad off its plane makes a bent
(non-planar) panel; the rebuilt rigidity then treats it as a rigid tetrahedral
body. Keep the destination in the panel's plane for rigid origami.

## 14. Patterning (added in v0.2)

All patterning operations copy a single link's geometry, parameters, colour
and flexibility. Joints, ground and lock state are **not** copied; the
sketch-plane constraint is copied only when the copy still lies on that plane.

- **Copy / paste**: `Ctrl+C` remembers the selected link, `Ctrl+V` pastes a
  copy offset diagonally by about 15 % of the model size; Properties →
  *Duplicate* does both at once.
- **Mirror** (`M`): click a link (or select it in the model tree first), then
  a datum plane; a mirror image is created (edge lengths preserved; the
  polygon's vertex order is kept).
- **Pattern** (`P`): click a link (or select it in the model tree first), then
  - *Linear array*: click two points defining the spacing vector (placed
    like any free click, §18.3); *Copies* sets how many copies are added;
  - *Polar array*: click a datum axis (rotation about it) or a datum point /
    any position (rotation about the sketch-plane normal through it);
    *Copies* and *Total angle* (360° spreads copies evenly around the circle).

**Deviation / note:** multi-link selections and copying of the joints between
copied links are not supported in this version; join the copies with the Joint
or Edit tool.

## 15. Model tree (added in v0.2)

A collapsible **Model tree** at the top of the right column lists Links (with
ground / locked / flexible / hidden / **2-D** badges — the last for links kept
on a sketch plane, §17.4 — and an eye toggle), Joints, Construction geometry
(marking the active sketch plane), Drivers and Editing-point constraints.
Joints are listed by their type letter and ends ("R · Link 1 ↔ Link 2");
creases read "Crease M 160° · Panel 1 ↔ Panel 2", the fold angle and
mountain / valley class following the construction pose (the entry updates
when the angle changes by a degree or the class changes), and a fold driver on
a crease reads "Crease · fold". The same description, with the type spelled
out ("Revolute joint · Link 1 ↔ Link 2"), names a joint in the status bar
and in the query cycle (§18.1). Click selects (Drivers: makes it the active
driver), hovering highlights the item in the viewport (a datum plane or axis
lights up for a row hover, unlike for the idle pointer, §7), double-click
renames links and user datums. Hidden links are neither drawn nor pickable but still
take part in the solve (useful for scaffolding geometry).

## 16. Out of scope for this prototype

Collision detection, dynamics, fabrication output (thickening, hinge design
as in Crane), free-timing synthesis, multi-selection, measuring tools.

## 17. Robust joining, creases and folding (added in v0.3)

**Original:** "Goal: To make a design tool for complex linkages and 3-D
origami mechanisms. … Each vertex/surface/side should be able to be
constrained with compatible constraints." The brief says nothing about what
should happen when a constraint cannot be satisfied, nor how an origami vertex
built from separate panels is brought from its flat state into motion.
Building a degree-4 vertex from four triangles with the Joint tool exposed
both gaps (the report, evidence and options are in
[CONSTRUCTION_PLAN.md](CONSTRUCTION_PLAN.md) §1–4). This section specifies
the v0.3 answer: no construction tool commits a pose whose hard constraints
are violated, and panels that share an edge are creases with an explicit
pre-fold.

### 17.1 Joining edges and the feasibility gate

**Edge ↔ edge (and cylinder axis) revolute matching.** The end points of the
two edges are paired by proximity in the current pose: straight (A0–B0,
A1–B1) or crossed (A0–B1, A1–B0), whichever is closer (a tie keeps the
straight pairing), so the order in which an edge was drawn or picked never
matters. Then:

- *Same length within one part in a thousand of the longer edge*
  (`EDGE_LENGTH_TOLERANCE`, overridable per joint through
  `JointOptions.lengthTolerance`): the joint is a **crease**. Both end-point
  pairs are merged into shared solver variables (5 equations, 1 DOF). If the
  lengths differ by more than floating-point noise (relative difference above
  1e-9, `CREASE_EXACT_TOLERANCE`) one panel takes the other's edge length: the
  link of the second-picked feature adapts unless it is ground or locked,
  otherwise the first (when the Panel or Edit tool joins coincident edges
  the link being sketched or edited is the one that adapts). The adapting
  link is first carried rigidly onto its partner and then only its shared
  edge changes length; its rest geometry is updated. No vertex moves by more
  than the length difference Δ and the mechanism is left with zero residual
  (previously a reversed edge with a tiny mismatch was flipped to the other
  side of the axis). The adaptation is accepted only while no rest distance
  of the adapting link changes by more than twice the merge tolerance of the
  longer edge (`CREASE_ADAPTATION_TOLERANCE`, plus the solve tolerance), i.e.
  snap-sized noise: a loop that rigid panels cannot close is refused even
  when the panels were placed by hand or by mouse (previously the released
  solve "closed" it by collapsing a triangle into a line and baked that
  shape). When neither panel may adapt (both ground or locked) the crease is
  refused with the edge-length message below. A crease between two panels
  that were drawn apart is created with the panels side by side (fold angle
  180°, the moving panel placed in the free sector beside its neighbour), not
  folded flat onto each other.
- *Genuinely different lengths*: the joint is a **hinge** along the two
  lines. The second edge is oriented like the first (the stored feature lists
  its end points in the matching order), both of its end points are held on
  the first edge's line and the slide along the axis is locked at its value
  in the current pose, measured as (B0 − A0) · unit(A1 − A0) exactly like a
  revolute against a datum axis, so the second link is pulled onto the line
  but keeps its position along it instead of jumping to the first edge's end
  point. Such a hinge is not a crease: it is drawn with the torus glyph and
  takes no part in crease loops.
- Cylindrical, prismatic and screw joints between edges are unchanged (they
  intentionally allow sliding; cylindrical keeps 2 DOF), and cylinder axes
  behave exactly like edges.

**Pre-flight, rollback and diagnosis.** After the second feature is picked
the joint is added and the geometry re-solved as before, but the result is
verified: the joint is kept only when the solve converges or its remaining
hard residual is within 1e-7 of the model size (never below 1e-9 units,
`feasibilityTolerance`). Otherwise *nothing changes*: no joint, no helper
point, no vertex moves, no undo entry; the model is restored byte for byte
and the status bar explains why (the message is also kept as a notice in the
Mechanism panel, §1). Every refusal ends with "Nothing was moved." and the
diagnosis is the most specific that applies, in this order:

1. **Edge lengths** — "The two edges have different lengths (2 and 2.2
   units), so they cannot fold as one crease. Edit one panel so the edges
   match, or pick two edges of equal length. Nothing was moved." — an
   edge–edge revolute whose lengths differ by more than the merge tolerance
   (and cannot be satisfied as a hinge), or whose small mismatch neither
   panel may absorb (both ground or locked). Numbers are shown with up to
   four significant digits.
2. **Sector sum** — "The panels around vertex T4 V0 have corner angles that
   add up to 240.0°, not 360°, so they cannot lie flat around it. …" — the
   joint closes a loop of three or more polygon / prism panels sharing a
   vertex whose interior angles differ from 360° by more than 0.5°. The
   vertex is named "<link> <point>" after the first-picked link; the sum is
   rounded to 0.1°; interior angles are true interior angles (the reflex
   corner of an L-shaped panel counts 270°, so an L plus two 45° triangles is
   developable). When every one of those panels is kept on a sketch plane the
   message continues "They can only meet by folding out of the sketch plane:
   remove the 2-D constraint from these panels and move them out of the plane
   (build the vertex in its folded shape) before adding this joint, or change
   their shapes."; otherwise "Change the panel shapes so the angles add up to
   360°, or build the vertex in its folded shape."
3. **Needs 3-D** — "This joint can only be satisfied out of the sketch plane.
   Remove the 2-D constraint from the links involved and move them out of the
   plane (with the Edit tool, or sketch them in 3-D mode in their folded
   shape), then add the joint again. Nothing was moved." — the same joint is
   satisfiable once the sketch-plane constraints of the links in the loop (or
   of the two links, when no loop is closed) are removed. This is probed on a
   copy, also from a slightly out-of-plane start, because a perfectly flat
   model cannot leave its plane on its own. Example: three unit squares in a
   row closed into a triangular tube; a panel whose edge must lie on a tilted
   datum axis.
4. **Generic** — "This joint cannot be satisfied together with the existing
   constraints (a gap of 0.6878 units remains). Remove a conflicting joint or
   move the links into place before joining. Nothing was moved." The gap is
   the rigid solve's residual.

Features that cannot take the joint type at all are refused before anything
is solved ("These features cannot be connected with this joint type.").

Acceptance scenario (the report of CONSTRUCTION_PLAN.md §1): four regular
triangles drawn with the Polygon tool in 2-D mode and joined edge to edge
yield three creases and a refused fourth joint with the 240° sector message,
whether the triangles were placed exactly, by mouse (circumradii that differ
by up to 4e-4) or by hand (vertices that agree to a snap distance); the fourth
triangle keeps its shape. Four developable triangles (60°, 60°, 120°, 120°)
join with zero residual: DOF 0 while the 2-D constraint is on (§17.4), DOF 2
without it (the flat state, §17.3).

**Where the gate applies.** The same verification covers every construction
edit that is re-solved: Joint Properties → *Change joint type* and the
link-end pop-up cycle (an incompatible type leaves the joint exactly as it was
— previously it was deleted — and a type with which the mechanism cannot be
re-assembled is refused with the messages above; the cycle skips incompatible
types and stops at a refused one); Link Properties → *Length*; vertex
Properties → *Position*; releasing a Select-tool drag of a vertex or of a
whole link; the *Keep on sketch plane (2-D)* checkbox (§17.4). For these the
model returns to its state before the edit and the status bar reads "The
constraints cannot all be satisfied with this change, so it was not
applied."; the Properties panel is rebuilt from the restored model, so the
widget shows the model's value rather than the refused one and the next edit
reaches the live model (this holds for every path that replaces the model
while a widget has focus: a refusal, undo, redo, load). The automatic joining
of the Panel, Polygon, Prism and Edit tools (§3.2, §13) verifies its re-solve
too: when it is not accepted, the link is created (or the vertex moved)
unjoined; the Panel, Polygon and Prism tools report the diagnosis ("The link
was created, but its snapped vertices were not joined: …", §13.1), the Edit
tool stays silent.

**Internals.** `src/core/feasibility.ts`: `tryAddJoint` and
`tryChangeJointType` (snapshot → edit → `solveSketchWithRelease` → accept, or
`restore` + `diagnoseJoint`), `trySolveCommit` for the other edits,
`linkPath` (breadth-first search over link–link joints; a joint between
already connected links closes a loop, and the links on the shortest path are
the loop the diagnoses look at), `sectorSumAt` / `interiorAngleDeg` over
`mergedPointGroups` (the union-find over merged point pairs, shared since
v0.4 with the crease-loop detection of §17.3, the Panel tool's sector
preview of §18.2 and the validator of §18.4), `closesWithout2d`,
`adaptationLimit` / `isReleaseAccepted`.
`src/core/kinematics.ts`: `solveSketchWithRelease` (a rigid projection, then
a projection warm-started from it with the design distances of the released
end points freed, returning the rigid step and the `releaseDrift`),
`isAccepted`, `feasibilityTolerance`. `src/core/model.ts`:
`creaseReleasePoints`, `mergedCreaseMismatch`, `creaseEdgeLengths`. The
mapping from diagnosis to text is `jointRefusedMessage` in
`src/ui/strings.ts`. Two solver changes were needed on the way: the
Levenberg–Marquardt damping is floored at 1e-3 of the largest diagonal entry
of JᵀJ (`DAMPING_FLOOR`), so a coordinate the residuals hardly depend on
takes the minimum-norm step instead of stalling the solve (crease merges of
nearly axis-aligned hand-placed edges), and a system with no free variables
counts as converged only when its residual is small (a rest distance between
two frozen points is a violation, not a row to drop; §17.5).

### 17.2 Creases

A **crease** is an edge–edge revolute whose two end-point pairs are merged
(the shared edge of two panels, §17.1). It remains an ordinary joint for DOF
counting, sweeps, inverse design and compliant hinges; what follows is its
presentation and its controls.

- **Rendering.** Instead of the torus glyph a crease is drawn as a line along
  the shared edge, 1.6 × thicker than the panel edges it lies on: red for a
  mountain fold, blue for a valley fold (Settings → Colours, defaults
  `#d9342b` / `#2f6fd6`), grey (`#8d949c`) while the two panels are coplanar
  (within 1° of flat). The colour follows the pose being shown, so creases
  change colour while a preview animates. Clicking the line selects the joint,
  hovering highlights it; with labels on, a selected or hovered crease shows
  "Crease M 160°": the class and the **fold angle**, the dihedral magnitude
  rounded to whole degrees (180° = flat, 0° = fully closed).
- **Mountain / valley convention.** Take the face normal of the crease's
  *first* panel given by its vertex winding (for panels drawn
  counter-clockwise on the sketch plane, as the Polygon and Panel tools draw
  them, this is the sketch-plane normal). The crease is a **valley** when the
  second panel is bent toward the side that normal points to (the panels form
  a trough seen from that side) and a **mountain** when it is bent away (a
  ridge). In terms of the signed dihedral that fold drivers prescribe: valley
  when the dihedral is positive and the positive side coincides with the
  panel's face normal, mountain otherwise. The class does not depend on the
  direction in which the edge was stored, only on which panel was picked
  first and its winding, so neighbouring creases of a sheet drawn on one
  sketch plane are classified from the same side. A degree-4 vertex with
  sectors (α, α, π−α, π−α) folded on its generic branch shows Maekawa's 3 : 1
  pattern: the crease between the two equal α sectors has the odd class and
  the other three share the other class; the two bent (zigzag) creases have
  the same class and the two collinear creases opposite classes (verified
  against the closed-form Miura pose of the example).
- **Joint Properties of a crease.** Title "Crease"; "Connects A ↔ B"; **Fold
  angle (current)**, read-only, e.g. "160.0° · Mountain (M)" (the magnitude,
  with "flat" while unfolded); **Mountain / valley** selector (*— not
  assigned —* / *Mountain (M)* / *Valley (V)*), stored with the crease and
  moving nothing by itself: it fixes the direction of *Fold to target* and
  the side a flat vertex is pre-folded to; **Target fold angle (°)** (0–180;
  default 160° for a flat crease, otherwise the current angle; stored with
  the crease, written without an undo entry so that typing a value and
  pressing the button is a single action); **Fold to target**; **Drive this
  crease**. The compliant-hinge, change-type and delete controls remain.
- **Fold to target** re-solves the mechanism so the crease reaches the target
  angle in the chosen class. A flat vertex is pre-folded first (§17.3,
  preferring this crease and the chosen side) and the sketch-plane constraints
  of its panels removed; a vertex folded to the other side is first returned
  to flat and then folded to the requested side; drivers on the other creases
  of the vertex are re-measured rather than held (holding them would fix a
  1-DOF vertex). The target is reached by continuation: steps of at most 120°
  along the shorter arc, each warm-started from the previous pose, and a
  target with the same angle but the other class is routed through flat
  (180°), never through fully closed. Hence a plain hinge (open fan, no loop)
  can be switched from Mountain 90° to Valley 90° and back, a fully closed
  hinge can be opened to flat, and the middle crease of a three-panel fan can
  be flipped while a driver holds the other crease. A target of 0° (closed)
  or 180° (flat) is accepted regardless of the class, since coincident or
  coplanar panels have no class; a target of 180° on a folded degree-4 vertex
  brings the whole vertex (within the 1° flat tolerance) to its flat state,
  the only generic pose in which one crease is unfolded. On success the move
  is one undo entry and the status bar reads "Folded to target: Crease M 120°
  · D1 ↔ D2. Mechanism DOF: 1."; otherwise the model is unchanged and the
  status bar says why — "No pose was found in which this crease has that
  fold angle: a joint, a driver on another crease of the vertex or a locked
  panel holds it. Nothing was changed." (also when the result would be the
  degenerate straight-hinge branch), "A panel of this vertex is locked. Unlock
  it before folding.", or the pre-fold's own reason (§17.3).
- **Drive this crease** adds a fold driver on the crease and makes it the
  active driver, pre-folding a flat vertex first and saying so; the Driver
  tool behaves exactly like this button (§8.1). A crease never carries two
  fold drivers: a crease that already has one keeps it and it becomes the
  active one ("This crease already has a fold driver; it is now the active
  one.", no undo entry); when a vertex's only driver is a fold driver on
  another of its creases (the one Fold kept, §17.3) that driver moves to the
  picked crease, so Preview and Simulation keep sweeping the one-DOF vertex;
  models with several drivers are left alone. If the pre-fold fails the
  driver is still added with the warning "This crease belongs to a flat
  vertex that could not be pre-folded: the simulation may fold only two of
  its creases."
- **Model tree and files.** Creases are listed as "Crease M 160° · Panel 1 ↔
  Panel 2" (§15); drivers on creases read "Crease · fold". The tree and the
  Properties show the construction pose; only the viewport colours the
  rendered (preview) pose. `joint.fold { target, mv }` is optional in the
  `.linkage.json`; files written before this version load unchanged.

**Deviation / note:** the plan (CONSTRUCTION_PLAN.md §5, 1b) expected the two
creases of each Miura pair to share a class; only the bent pair does — under
any consistent convention the collinear pair must differ (Maekawa's theorem).
The plan also placed a Fold action in the joint Properties; a crease's
Properties offer *Fold to target* and *Drive this crease* instead, and the
Fold command lives in the Mechanism panel.

### 17.3 The Fold command (pre-fold)

**Crease loops and the flat state.** Creases that end at the same merged
vertex form a **crease loop** when there are at least three of them and their
panels close a single cycle around that vertex (each panel carries exactly two
of them); an open fan is not a loop and needs no pre-fold. A loop is **flat**
when every crease's dihedral is within 1° of ±180°, i.e. all panels lie
unfolded in one plane. The flat state is a singular (bifurcation) state: the
mechanism reports one extra DOF (2 instead of 1 for a degree-4 vertex) and a
simulation started there folds only two creases (the straight-hinge branch).
Two creases of a loop are **collinear** when their directions from the vertex
are antiparallel within 1°; driving a collinear crease cannot leave the
straight-hinge branch.

**The command.** When the model has a flat crease loop the Mechanism panel
(in every mode, next to the DOF readout) shows "A vertex is flat: all of its
creases are unfolded, a singular state in which a simulation would fold only
two of them." and the button **Fold (pre-fold flat vertex)**. Fold acts on
the first flat loop: it removes the *Keep on sketch plane (2-D)* constraints
of the loop's panels, then drives one crease — preferring creases that are
not collinear with another one — to a dihedral of 160° (20° from flat,
`DEFAULT_PREFOLD_DEG`) with either sign, re-solving the whole mechanism from
the flat pose with every other driver held at its current value (drivers
acting on the vertex itself — fold drivers on its creases, angle drivers on
its panels — are released, since holding them at their flat values would
forbid any fold). The first result that converges, leaves no crease of the
loop within 1° of flat and lowers the mechanism DOF (below that of the flat
state without the 2-D constraints) is committed (the Fold button itself takes
no mountain / valley preference and tries the positive sign first; *Drive this
crease* and *Fold to target* pass the picked crease's assignment, which is then
verified on every crease it names, so a preference on a collinear crease —
never driven first — is honoured through the sign of the crease that is
driven); the driven crease is selected and
the status bar reads "Vertex pre-folded about crease D2 ↔ D3. Mechanism DOF:
1." A model that had no driver keeps a fold driver on the driven crease so it
can be previewed at once, and the status adds "A fold driver was kept on that
crease so the vertex can be previewed; the Driver tool moves it to another
crease." One undo step restores the flat vertex including its 2-D
constraints. For the developable vertex (60°, 60°, 120°, 120°) the result is
fold angles of about 170° on the collinear pair and 160° on the bent pair,
DOF 1, three mountains and one valley (Maekawa), and a fold-driver sweep moves
all four panels.

**Failure** leaves the model exactly as it was and says why:
"Nothing to fold: creases must close a loop of at least three panels around
one vertex." (no loop); "The vertex is already folded; there is nothing to
pre-fold." (loops exist but none is flat — the Miura example); "A panel of
this vertex is locked. Unlock it before folding." (a non-ground panel of the
loop is locked); "No folded state was found for this vertex: every crease is
collinear with another one (an "X" vertex folds only as a straight hinge), or
the panels cannot all leave the plane. Nothing was changed." (e.g. an "X"
vertex with four 90° sectors, whose creases are collinear in pairs).

**Driver tool and Joint tool.** Adding a fold driver on a crease of a flat
vertex (Driver tool, or *Drive this crease*) pre-folds the vertex first, in
the same undo step, with the stored *Mountain / valley* assignment of the
picked crease choosing the side; the status bar adds "The flat vertex was
pre-folded first so that every crease moves." When a new crease closes a flat
loop, the Joint tool's status adds "The vertex is flat: use Fold to pre-fold
it before simulating."

**Deviation / note:** the Fold button is offered in all modes although other
editing tools are disabled outside Construction mode (§2): the flat state is
usually noticed when a preview folds only two creases, and the command is
self-contained. Fold handles one vertex per press (the first flat loop); a
sheet with several flat vertices needs Fold once per vertex, and a vertex
where more than one cycle of panels meets is not detected as a loop.

**Internals.** `src/core/fold.ts`: `isCrease`, `findCreaseLoops`
(`mergedPointGroups` over the merged points, a cycle walk over the panels,
`collinearPairs` measured in the current pose), `loopNormal` (the construction plane of a loop panel
when there is one, else the fitted vertex plane oriented along the active
sketch normal), `prefoldVertex` (candidate order, both signs, the acceptance
test, byte-for-byte restore on failure), `foldCreaseTo` (one forward solve
with a temporary or the existing fold driver, the id counter rewound on
failure), `foldCreaseToTarget` (pre-fold / unfold-through-flat /
continuation), `driveCrease`, `creaseMV` / `creaseNormal` /
`creaseTargetDihedral`. The signed dihedral is `jointDihedralDeg`
(`src/core/jointMeasure.ts`), the same quantity fold drivers prescribe and
measure, so the Fold command, the drivers and the crease display agree. The
Fold button and the hints are rendered from `SimState.creaseLoops` and
`SimState.lockedCreaseIds`, recomputed with the DOF after every edit.

### 17.4 The sketch-plane (2-D) constraint

Every bar or polygon drawn in 2-D mode carries a hidden planar joint between
all of its vertices and the sketch plane (`bodyPlaneJoint`), which the
compiler turns into frozen coordinates (DESIGN_DECISIONS.md §6). Until v0.3
it was invisible, and it is what makes every in-plane crease rigid: a flat
vertex drawn in 2-D reports DOF 0 although every crease is a hinge.

- **Link Properties** (bars and polygons): checkbox **Keep on sketch plane
  (2-D)** showing whether the link carries the constraint, with the plane's
  name ("on TOP"); the tooltip explains it. Ticking it constrains the link to
  the *active* sketch plane, unticking removes the constraint. Either way the
  mechanism is re-solved through the pre-flight of §17.1: a free link is
  moved (rotated / translated) onto the plane keeping its shape, a ground link
  is allowed to move for this edit, and if the constraints cannot all be met
  (for example a vertex pinned to geometry off the plane) nothing changes and
  the status bar says so. The edit is one undo step. A link is on at most one
  plane at a time; to change planes untick, make another plane the sketch
  plane, tick again. Prisms and cylinders have no checkbox (they are 3-D
  bodies; extruding a polygon removes its constraint, §13.1).
- **Model tree:** links with the constraint show a "2-D" badge beside ground
  / locked / flexible / hidden. The constraint is still not drawn in the
  viewport and does not appear in the Joints group (it is presented as a link
  property), but the link-end pop-up's constraint tooltip mentions "on sketch
  plane TOP" (§4).
- **Locked creases.** When the model contains creases whose two panels are
  both kept on the *same* sketch plane and whose axis lies in that plane, the
  Mechanism panel shows "4 creases cannot fold: both panels are kept on the
  sketch plane. Untick 'Keep on sketch plane' in Properties, or use Fold."
  (", or use Fold" only while the Fold button is offered, i.e. a flat crease
  loop exists) and the DOF chip carries the same text as its tooltip. Such
  creases are rigid (rotating about an in-plane axis would lift both panels
  off the plane), which is why a flat vertex drawn in 2-D reports DOF 0;
  releasing one panel of a crease frees it (two triangles on TOP sharing an
  edge: DOF 0 with the hint, DOF 1 and no hint once one triangle is
  released). The count is recomputed with the DOF after every edit. Hinges
  between edges of different length (slide-locked) and creases whose panels
  are on two different planes are not counted.
- **Constant points** (since v0.4). The compiler holds a link kept on an
  axis-aligned plane there by freezing coordinates, but a vertex merged with
  a constant point — a ground vertex, a vertex pinned to a datum point — is
  not a variable, and until v0.4 its distance from the plane was silently
  dropped. It now counts as a residual without variable columns, like the
  frozen–frozen rest distance of §17.5: a 2-D polygon pinned to a ground
  vertex off its plane shows the mismatch in the DOF chip, and the pre-flight
  solves see it (such a join is refused with the needs-3-D diagnosis instead
  of reading a violation of 0).

**Internals.** `setBodyPlane(m, linkId, planeId | null)` in
`src/core/model.ts` adds, replaces or removes the body planar joint (drawing
a link in 2-D mode and ticking the box produce byte-identical models; an
unknown plane or a 3-D link leaves an existing constraint untouched);
`creasesLockedByPlane(m)` lists the rigid creases.

### 17.5 Consistency guard: never bake a violated pose

**Original:** "Saving or exporting the mechanism file should solidify these
assumptions."

Rest geometry (bar lengths, polygon / prism distances and angle values) is
never refreshed ("baked") from a violated pose, because the violation would
silently become the design. A construction pose is **consistent** when its
largest hard-constraint residual (`currentViolation`) is at most
`consistencyTolerance` = max(1e-6, 1e-7 × model size) length units; the DOF
chip's "Constraints violated" flag uses this same tolerance (§6), so the chip
warns exactly when the operations below refuse. A rest distance between two
frozen points (both ends pinned, merged with the ground, or a locked link
joined to the ground) counts like any other constraint — as does, since
v0.4, the sketch-plane constraint on a constant point (§17.4): the chip
shows its mismatch, the Length editor refuses a rest length a bar pinned at both ends
cannot take, and joining a 2.2-unit edge corner by corner to a 2.0-unit
ground edge is refused as infeasible (gap 0.2).

- **Solidify assumptions** (Simulation panel): on a consistent pose every
  link's rest geometry is set from the current positions and the status bar
  reads "Assumptions solidified: the current geometry is now the design."; on
  an inconsistent pose nothing changes (no undo entry) and it reads
  "Assumptions not solidified: the current pose violates the constraints (gap
  0.6878 units). Undo the last edit or release a joint, then solidify again."
- **Save** solidifies first when the pose is consistent, exactly as before;
  otherwise the file is written as it is and the status bar says so (§12).
  CSV, OBJ and PNG exports never solidified and are unchanged.
- **Extrude to prism** is refused on an inconsistent pose — "Not extruded: the
  current pose violates the constraints (gap … units), so the polygon's shape
  cannot be taken as its design. Undo the last edit or release a joint
  first." — because it would rebuild the polygon's design from its distorted
  shape.
- Internally, applying a sketch solve (`commitSketch`) always shows the
  solved positions (so live drags and refused results remain visible) but
  refreshes the rest geometry of released links only when the result is
  accepted, and tells the caller; every committing path (the Joint tool,
  joint type changes, the Length and Position editors, the drag release, the
  2-D checkbox, the Panel, Polygon and Edit tools) verifies acceptance before
  committing and rolls the model back otherwise (§17.1).

**Deviation:** a violated pose is not solidified on Save; the file keeps the
last consistent design together with the positions on screen, and the status
bar says so. Copy / paste, mirror and pattern (§14) build a *new* link's
rigidity from the visible geometry and are not guarded, since they never
change an existing link's design.

## 18. Construction workflow (added in v0.4)

**Original:** "Creating … a 4 bar link, should be done by selecting a 'Link'
tool, and clicking points in the 3D space"; "a UI similar to CAD software
(such as Creo or Rhino)"; "Each vertex/surface/side should be able to be
constrained with compatible constraints." The brief does not say how one of
two coincident edges is picked, how a panel is attached to the panels already
drawn, where a click lands in 3-D, or how a crease pattern is checked before
it is folded. Phase 2 of [CONSTRUCTION_PLAN.md](CONSTRUCTION_PLAN.md) §5
(items 2a–2d) answers the four; this section specifies the behaviour as
implemented and names the deviations from the plan.

### 18.1 Query pick: right-click cycles through everything under the pointer

The single pick under the pointer (the hover) returns one feature, chosen by
priority — editing points, vertices, joints and creases, edges, cylinder
axes, faces, then datum geometry — among the hits within the pick tolerance
of the nearest one; two coincident edges can therefore never both be reached
by hovering, and the Joint tool kept finding the first link again. As with
Creo's *query select*, a **right-click** now steps through the candidates.

- **Query click.** A right-click counts as a query click when the pointer
  never strayed more than 4 px (`QUERY_CLICK_PX`) from the press while the
  button was down; a right-drag — even one that swings out and returns to
  its start — orbits as before and starts no cycle. A right-click during a
  left drag is ignored. The context menu stays suppressed.
- **Candidates and their order.** The viewport collects every feature hit by
  the exact pointer ray and by the ring of rays the normal pick samples
  (radii 5 and 10 px, eight rays each), de-duplicates them (a feature hit by
  several rays counts once, at its closest hit) and ranks them: model
  features first, in **depth bands** — the nearest hit opens a band that
  takes every feature within the pick depth tolerance (twice the pick radius
  in world units) behind it, the next feature behind that opens the next
  band, and so on — so a vertex far behind the panel under the pointer is
  listed after that panel's features; within a band by pick priority,
  exact-ray hits before ring hits, then nearest first; datum geometry last
  (points, then axes, then planes). Datum geometry never occludes model
  geometry.
- **The cycle.** The first right-click highlights the first candidate that
  is not the current hover, so it always shows something new; each further
  right-click at the same spot advances and wraps around. The candidate
  replaces the hover (the usual highlight; the hovered edge or face of a
  polygon or prism is lit more strongly than the rest of its link, and datum
  axes and planes light up in the selection colour) and the status bar reads
  "2 of 3 · edge V0-V1 of Polygon 2 · right-click: next · left-click: use it
  · Esc: stop" (a lone candidate: "1 of 1 · … · the only feature here ·
  left-click: use it"; nothing under the pointer: "Nothing under the pointer
  to cycle through."). A joint or crease is described exactly as the model
  tree describes it ("Crease V 146° · Panel 1 ↔ Panel 2", "Revolute joint ·
  Link 1 ↔ Link 2"), a datum by its name.
- **Using and leaving it.** A left-click while the cycle is active uses the
  highlighted candidate as the pick of the active tool — Select, Joint, Edit,
  Ground, Driver, Delete, Mirror, Pattern, and the vertex / datum-point snap
  of the creation and Panel tools all read the same pick — so any feature
  under the pointer can be chosen; afterwards the status bar shows the plain
  description of the feature used unless the tool writes its own status (the
  Joint tool does). A candidate that no longer exists (deleted or undone
  meanwhile) is ignored and the normal pick is used. Pointer motion within
  4 px keeps the highlighted candidate and the tool's preview follows it;
  moving farther, the pointer leaving the viewport (status cleared), the
  first `Esc` (a second `Esc` cancels the tool), a tool switch, undo / redo
  and any model replacement (New, Open, an example) end the cycle, and the
  normal hover, its description in the status bar and the active tool's
  preview (snap ring, marker, polyline, labels) return.
- **Highlighting datums.** Datum planes and axes are highlighted only for a
  hover the user asked for — the query cycle or a model-tree row — because
  the plain pointer rests on the sketch plane wherever the canvas is empty
  (the status bar names it; nothing lights up).
- **Joint tool.** When the second click's feature belongs to the first link
  again, the best-ranked compatible feature of another link in the front
  depth band is used and the status says so (§5); a feature named through
  the cycle is never substituted.

Acceptance (plan 2a): two triangles with exactly coincident edges are joined
through the UI by left-click, right-click ("2 of N", the other polygon's
edge), left-click → "Joint created" and one crease; a 60 px right-drag still
orbits, in the Top view too.

**Deviation / note:** the plan ordered candidates by priority and distance
only; depth bands were added so that hidden geometry is not offered before
the panel under the pointer (with a tolerance larger than the depth spread
the ranking degenerates to priority, then distance). The plan ended the cycle
on pointer movement and `Esc`; leaving the viewport, a tool switch, undo /
redo and a model replacement end it as well, and `Esc` ends only the cycle
first so that a Joint tool with its first feature picked is not reset. The
single pick (hover) is unchanged. The orbit controls are rebuilt whenever a
view changes the camera's up vector, which lets a right-drag orbit out of the
Top view (previously it sat at the pole of a Z-up frame); orbiting out of the
Top view keeps Y as screen-up until another view is chosen.

**Internals.** `src/viewport/pickRank.ts` (DOM-free: `PickResult` with
`exact`, `sub`, `band` and `queried`, `PICK_PRIORITY`, `pickKey` /
`samePick` — the feature identity includes the point ids and the face index,
since every edge and face of a polygon carries the same link id —
`rankPickCandidates`), `src/viewport/pickCycle.ts` (`PickCycle`, the state
machine, `QUERY_CLICK_PX`), `Viewport.pickCandidates` / `castAll`
(`src/viewport/scene.ts`; the `query` pointer event is emitted before the
matching `up`), `ToolManager.queryClick` / `endCycle` / `endQuery` /
`otherLinkCandidate` (`src/viewport/tools.ts`), `src/ui/labels.ts`
(`jointDescription`, shared by the tree, the status bar and the cycle), the
hover highlights in `src/viewport/render.ts`. `App.setHover` compares the
full feature identity (`pickKey`), so stepping between the edges of one link
re-renders.

### 18.2 Panel tool: sector-angle feedback and the overlap refusal

The Sketch tool is presented as **Panel (sketch polygon)**: its job is to
build an origami vertex panel by panel onto the vertices already there, so
that shared edges become creases without the Joint tool (§13.1 specifies the
joins; the Polygon and Prism tools join their coincident vertices the same
way, §3.2). While a panel is drawn the tool shows whether the panels around
each shared vertex can lie flat.

- **Sector-angle labels.** From the second placed vertex on (a polyline of at
  least three points, the cursor included), every sketch vertex that sits on
  an existing vertex — snapped, typed or grid-snapped onto it, the vertex
  under the cursor included when it is snapped — carries a label "S° (+A°)",
  one decimal each: A is the angle the new panel adds at that vertex between
  its two adjacent sketch edges (the open edge follows the cursor), measured
  as the true interior angle about the sketch's winding normal, so a reflex
  corner of an L-shaped panel counts 270°; S is A plus the sum of the
  interior angles of every existing polygon or prism that has a vertex there
  (joined through creases or pins, or merely coincident; bars have no sector
  angle). The label is **green** ("closes at 360°") when both adjacent sketch
  edges coincide with existing panel edges at that vertex, i.e. the panel
  completes the ring, and S is within 0.5° of 360° (the sector tolerance of
  §17.1); **red** when S exceeds 360.5°, when the ring closes at any other
  sum, or when the corner is collinear (a straight or zero corner, since such
  a panel is refused as degenerate); neutral (white) otherwise. No label is
  shown at a vertex whose adjacent edge has zero length, nor for a cursor
  resting on the first vertex (about to close the panel); labels are shown
  whatever *Show labels* says and are drawn above the datum and link-name
  labels, and a datum point's name label is omitted while a sector label
  sits at that point (ORIGIN under a closing vertex).
- **Overlap refusal.** In a plane the corners of the panels around a vertex
  tile exactly 360°, so a red label at a shared vertex while the sketch and
  every existing panel around that vertex lie in one plane means the new
  panel would overlap its neighbours. On closing, such a panel is not
  created: the status bar and the Mechanism-panel notice read "Panel not
  created: the corner angles of the panels around vertex T1 V0, this one
  included, add up to 300.0°, not 360°, so in this plane they would overlap.
  Change the panel's shape (Backspace removes the last vertex), or build the
  vertex in its folded shape by sketching onto vertices out of the plane.
  Nothing was changed.", the sketch is kept for correction and nothing
  changes. A ring that closes at another sum in 3-D — the fourth face of a
  square pyramid, four 60° sectors, 240° — shows the red label too but is
  joined, since it is a legitimate folded vertex. A straight or zero corner
  is refused as degenerate before the overlap test ("Three consecutive
  vertices are collinear …").
- **Examples.** Three developable triangles (60°, 60°, 120°) flat on the
  sketch plane and the fourth panel drawn onto the centre and the two free
  corners: the centre label reads "360.0° (+120.0°)" in green while the
  cursor rests on the closing vertex, and closing gives two creases ("Joint
  created (2)"), four creases in all, DOF 0 with the locked-creases hint and
  the Fold button (§17.3). Three regular triangles in a flat fan and a fourth
  continuing it: a neutral "240.0° (+60.0°)" with the ring still open, one
  crease; the only fourth regular triangles that close the ring in the plane
  are degenerate (a straight corner: red, refused as collinear) or overlap
  the fan (a 300° label, refused with the message above). Four regular
  triangles therefore cannot form a flat vertex, which §17.1's Joint-tool
  acceptance states from the other side.

**Deviation / note:** the plan expected "four regular triangles show a red
label at the centre on the fourth panel and the join is refused with the
sector message". In a plane a closed ring always sums to 360°, so the fourth
regular triangle either continues the fan (neutral 240°, joined) or overlaps
the fan (red, refused by the overlap rule above); the sector-message refusal
of a non-developable *closed* vertex remains the Joint tool's (§17.1), and a
red closed-ring label arises only in 3-D. The plan's "snapped or typed" join
is implemented through one coincidence tolerance shared with the automatic
join (`coincidenceTolerance`); the colours use the 0.5° sector tolerance of
the joint diagnoses, and the validator of §18.4 reads the same sum once the
panel exists.

**Internals.** `src/core/sector.ts` (`sectorPreview`: the merged-point group
of the vertex plus merely coincident vertices, `interiorAngleDeg` per panel,
the signed corner about the Newell normal of the sketch, `closesRing` as a
positional test on existing panel edges; `sectorStatus`; `sketchSectors`
with its `coplanar` flag — the one computation behind the labels and the
refusal), `src/core/edit.ts` (`coincidenceTolerance`, `coincidentVertex`,
`autoJoinCoincident` returning `{ joints, refused }` with the diagnosis of
§17.1 made on the rigid step before the model is restored, `edgeCreased`),
`ToolManager.withCoincidence` / `sectorLabels` / `finishSketch`
(`src/viewport/tools.ts`), the `labels` of `OverlayView` drawn with a higher
render order (`src/viewport/render.ts`), the `OVERLAY.sectorSum`,
`STATUS.sketchRefusedSector` and `STATUS.autoJoinRefused` strings.

### 18.3 3-D placement on the sketch plane

Until v0.3 a free click with *Place on sketch plane (2-D)* unticked landed
on the view-aligned plane through the previous point, at a depth that
depended on the camera, so geometry built in 3-D mode was rarely where it
looked. Now:

- In both modes a free click is placed where the pointer ray meets the
  active sketch plane. With the option ticked the link is also kept there by
  the 2-D constraint (§17.4); unticked, it is built on the plane but free to
  leave it.
- The view-aligned plane through the previous point (through the origin
  before the first point) is the fallback only when the ray makes less than
  about 8.6° with the sketch plane (|cos| < 0.15, `PLACEMENT_MIN_COS`; a
  grazing ray would fling the point far across the plane), when it misses
  the plane (parallel, or the hit behind the camera), or when it meets the
  plane at the camera itself — in the Front and Right views the camera lies
  on the TOP plane, so every off-centre ray hits it at depth 0 ("at the
  camera" is a depth below a thousandth of the fallback's depth, the
  distance the user is working at). With the option ticked, and for the
  datum and Panel tools, which always use the sketch plane, grazing rays
  still land on it: only a miss or a hit at the camera falls back, so
  off-centre clicks in the Front view make points on the view plane instead
  of at the camera (previously nothing usable was created).
- Typed coordinates, grid snapping, a pending length ("L" then click) and
  snapped vertices / datum points are unchanged; grid snap and the pending
  length are applied to the placed position as before.
- The option is one setting shared by every click-placed tool, so the Prism
  and Cylinder tools, the destination of the Edit tool and the spacing points
  of the Pattern tool follow the same rule (previously the view plane when
  the option was off). The option row's tooltip explains the two modes.

**Deviation / note:** the plan said "the view plane is used only when the
ray grazes the sketch plane"; a miss and a hit at the camera were added as
fallback cases, and the grazing threshold is 0 in 2-D mode. §3.1's former
"otherwise on a view-aligned plane through the previous point" is superseded.
The free-vertex placement of the Panel tool on a fitted plane (§13.1) is the
same rule with the same threshold and the orthogonal projection as its
fallback.

**Internals.** `placementOnPlane(ray, origin, normal, fallback, minCos)` and
`PLACEMENT_MIN_COS` in `src/core/edit.ts` (`placeOnFittedPlane` is built on
it); `ToolManager.place` computes the pointer ray once and passes the
view-plane hit as the fallback (`src/viewport/tools.ts`);
`TOOL_OPTIONS.mode2dHelp` in `src/ui/strings.ts`.

### 18.4 Crease-pattern validator

**Interior vertex.** A merged vertex whose panels close a ring of creases:
exactly a crease loop of §17.3 (at least three creases, each panel carrying
two of them, one cycle). Open fans, bars and loops that run through a bar or
a cylinder axis (which have no sector angle) are not reported.

**The checks.** For each interior vertex the Mechanism panel's summary shows,
in every mode under the DOF readout, the hints and the Fold button, a
**Crease pattern** block (its tooltip explains the checks) with one line per
vertex:

> Polygon 1 V0 · 4 panels · 360.0° developable ✓ · Kawasaki 0.0° ✓ · M/V 3:1 ✓

- The vertex is named by the loop's vertex point, on the ground panel when
  one is in the ring; *n panels* is its degree.
- **Developable**: the sum S of the panels' interior angles at the vertex
  (one decimal) is within 0.5° of 360°, so the panels can lie flat in a
  plane (a pyramid apex sums to less, a saddle to more).
- **Kawasaki**: the magnitude K of the alternating sum α₁ − α₂ + α₃ − … of
  the sector angles in the cyclic order of the loop is within 0.5° of 0°,
  the condition for a vertex of even degree to fold flat; a vertex of odd
  degree never folds flat and reads "Kawasaki ✗ odd degree".
- **M/V**: the numbers of mountain and valley creases, read from the
  construction pose in the convention of §17.2; "M/V not assigned" while any
  crease of the loop is flat (within 1° of 180°) or unmeasurable, otherwise
  checked against **Maekawa**'s condition |M − V| = 2.

Each check is independent: a square-pyramid apex (four equilateral faces)
reads "240.0° developable ✗ · Kawasaki 0.0° ✓ · M/V 4:0 ✗"; a vertex folds
flat only when it is developable and passes Kawasaki. A line whose vertex
fails a check is drawn in the warning colour and followed by a hint naming
each failed check with its measured value: "D1 V0: its sector angles sum to
240.0°, not 360°, so the panels cannot lie flat; it has 4 mountain and 0
valley creases; a flat-folded vertex needs them to differ by two (Maekawa).",
"…: the alternating sum of its sector angles is 20.0°, not 0°, so it cannot
fold flat (Kawasaki).", "…: 3 creases meet there, an odd number, so it cannot
fold flat." Examples: the developable vertex (60°, 60°, 120°, 120°) built
flat reads "360.0° developable ✓ · Kawasaki 0.0° ✓ · M/V not assigned" and
after Fold "… M/V 3:1 ✓" (or 1:3); the X vertex passes both angle checks and
keeps "M/V not assigned", since it cannot be pre-folded; the Miura example
reads "Panel 1 V0 · 4 panels · 360.0° developable ✓ · Kawasaki 0.0° ✓ · M/V
3:1 ✓"; a (90°, 90°, 60°, 120°) vertex is developable with Kawasaki off by
60°, and (90°, 90°, 100°, 80°) by 20°.

The reports are recomputed with the DOF after every change and are not part
of the model tree or the file.

**Deviation / note:** the plan asked for the Kawasaki condition "as
flat-foldability"; the report's `flatFoldable` is the Kawasaki test alone
(even degree and |alternating sum| ≤ 0.5°) and developability is a separate
mark, so the two are never conflated in one verdict. The plan's example of a
Kawasaki failure, (90°, 90°, 60°, 120°), has an alternating sum of 60°, not
the 20° its text stated; both figures are tested. Odd-degree vertices are
reported (NaN alternating sum, not flat-foldable), and loops through bars or
cylinder axes are skipped rather than shown with a missing sector.

**Internals.** `src/core/validate.ts` (`validateCreasePattern(m, tolDeg =
0.5)` → `VertexReport[]` over `findCreaseLoops`, `interiorAngleDeg` and
`creaseMV`; `kawasakiSumDeg`), `SimState.vertexReports` (`src/app.ts`,
cached with the DOF), the block at the end of the Mechanism summary
(`src/ui/panels.ts`), `SIM.vertexReport` / `SIM.vertexFails` /
`SIM.creasePatternHelp` (`src/ui/strings.ts`).
