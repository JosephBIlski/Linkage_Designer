# Linkage Designer — detailed feature specification (prototype v0.3)

This document specifies each implemented feature in detail. Each section
quotes the relevant line of the [original specification](ORIGINAL_SPEC.md)
and marks **Deviations** where the prototype departs from it (with the
reason). Rationale and references are in
[DESIGN_DECISIONS.md](DESIGN_DECISIONS.md); the diagnosis and the plan behind
the v0.3 joining / folding work (§17) are in
[CONSTRUCTION_PLAN.md](CONSTRUCTION_PLAN.md).

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
  displayed points, construction geometry and hovered joints.
- Navigation: right-drag orbit, middle-drag pan, wheel zoom-to-cursor;
  View menu: Top / Front / Right / Isometric, zoom to fit (`F`),
  orthographic / perspective.
- Undo / redo (`Ctrl+Z`, `Ctrl+Y` / `Ctrl+Shift+Z`) by whole-model snapshots;
  `Esc` during a drag restores the pre-drag model without touching history.
  A refused edit (§17.1) records no undo entry.
- Status bar: shows the last status message or the tool hint; its tooltip
  carries the whole text, since long diagnoses are clipped in the bar.
  Refusals (a refused joint, edit, fold, Solidify or Save) are also kept as a
  **notice** in the Mechanism panel, with a *Dismiss* button, until dismissed
  or until the next change goes through (undo, redo and loading a file clear
  it as well).
- Keyboard: `Esc` cancel tool, `Del` delete selection, `1` select, `2` link,
  `3` polygon, `4` prism, `5` cylinder, `G` ground, `J` joint, `D` driver,
  `Space` play/pause in Preview.
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

- Tool **Link**: click the first point, then the second. Points are placed on
  the active sketch plane (option *Place on sketch plane (2-D)*), otherwise on
  a view-aligned plane through the previous point. Optional grid snapping.
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
  datum planes / axes never steal a pick from model geometry. Clicking on
  empty space or a datum with the Select tool explains what can be dragged.
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
  never steal picks from real geometry.

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

### 13.1 Sketch tool (free polygon) and extrusion

- Tool **Sketch polygon** (`S`): click any sequence of vertices on the active
  sketch plane (points are projected onto the plane, so the polygon is always
  planar); a dashed preview follows the pointer. Close the polygon by clicking
  the first vertex again, double-clicking, or pressing `Enter`; `Backspace`
  removes the last vertex. At least three non-collinear vertices are
  required. Coordinates can also be typed in the status bar.
- **Snapping and joining**: vertices snapped onto existing link vertices are
  joined when the polygon is closed. Two consecutive snapped vertices that
  coincide with an edge of one existing link become a single edge–edge
  revolute joint (a crease); an isolated snapped vertex gets the default joint
  (hinge axis = the normal of the sketch plane both links share) when both
  links share a sketch plane, otherwise a spherical joint; a link that is
  already attached to a crease vertex through one of the crease partners is
  not pinned again. A vertex snapped onto a datum point is pinned to it. Free
  vertices of a polygon whose snapped vertices define a plane off the sketch
  plane are placed where the click ray meets that plane. This is how the last
  panel of an origami vertex is "filled in": sketch it by clicking the
  existing vertices. Edges that coincide only within the snap tolerance become
  creases whose sketched edge takes the existing edge's length (the polygon
  being sketched adapts, never the existing geometry; §17.1). The re-solve for
  the automatic joints is verified like every other edit: when it is not
  accepted, the polygon is created unjoined instead of with a violated pose.
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
  sketch plane, or typed coordinates.
- The vertex is first made a dependent point of its link's rigidity (so the
  rest of the link keeps its shape), then released from its own shape
  constraints, every other constraint is re-solved, and the link's rest
  geometry is rebuilt from the result. If the destination lies off the link's
  sketch plane, that planar constraint is removed first, so a flat panel can
  be lifted into 3-D. If the destination cannot be reached (the vertex is
  pinned to a datum point, or its joints forbid it) nothing changes and the
  status bar says so. Joint helper attachments that depend on the moved vertex
  are released and rebuilt, so a hinged bar or panel can be lifted out of its
  plane. Coincident vertices are joined with the same rules as the Sketch tool
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
  - *Linear array*: click two points defining the spacing vector; *Copies*
    sets how many copies are added;
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
a crease reads "Crease · fold". Click selects (Drivers: makes it the active
driver), hovering highlights the item in the viewport, double-click renames
links and user datums. Hidden links are neither drawn nor pickable but still
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
  otherwise the first (when the Sketch or Edit tool joins coincident edges
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
of the Sketch and Edit tools (§13) verifies its re-solve too: when it is not
accepted, the polygon is created (or the vertex moved) unjoined.

**Internals.** `src/core/feasibility.ts`: `tryAddJoint` and
`tryChangeJointType` (snapshot → edit → `solveSketchWithRelease` → accept, or
`restore` + `diagnoseJoint`), `trySolveCommit` for the other edits,
`linkPath` (breadth-first search over link–link joints; a joint between
already connected links closes a loop, and the links on the shortest path are
the loop the diagnoses look at), `sectorSumAt` / `interiorAngleDeg`,
`closesWithout2d`, `adaptationLimit` / `isReleaseAccepted`.
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
  counter-clockwise on the sketch plane, as the Polygon and Sketch tools draw
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

**Internals.** `src/core/fold.ts`: `isCrease`, `findCreaseLoops` (union-find
over merged points, a cycle walk over the panels, `collinearPairs` measured
in the current pose), `loopNormal` (the construction plane of a loop panel
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
joined to the ground) counts like any other constraint: the chip shows its
mismatch, the Length editor refuses a rest length a bar pinned at both ends
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
  2-D checkbox, the Sketch and Edit tools) verifies acceptance before
  committing and rolls the model back otherwise (§17.1).

**Deviation:** a violated pose is not solidified on Save; the file keeps the
last consistent design together with the positions on screen, and the status
bar says so. Copy / paste, mirror and pattern (§14) build a *new* link's
rigidity from the visible geometry and are not guarded, since they never
change an existing link's design.
