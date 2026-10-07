# Linkage Designer — detailed feature specification (prototype v0.1)

This document specifies each implemented feature in detail. Each section
quotes the relevant line of the [original specification](ORIGINAL_SPEC.md)
and marks **Deviations** where the prototype departs from it (with the
reason). Rationale and references are in
[DESIGN_DECISIONS.md](DESIGN_DECISIONS.md).

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
- Keyboard: `Esc` cancel tool, `Del` delete selection, `1` select, `2` link,
  `3` polygon, `4` prism, `5` cylinder, `G` ground, `J` joint, `D` driver,
  `Space` play/pause in Preview.
- All user-visible text is in `src/ui/strings.ts`; colours and display
  defaults in `src/ui/settings.ts` (`DEFAULT_SETTINGS`).

## 2. Modes

**Original:** Construction / Simulation / Preview modes as described.

| Mode | Purpose | What is shown |
| --- | --- | --- |
| Construction | Build and edit the mechanism | Links, joints, construction geometry, ground symbols, live DOF, output paths of displayed points (if a driver exists) |
| Simulation | Inverse design | Everything above plus design spaces, editing points, ghost of the mechanism at a selected/hovered pose, design DOF counters |
| Preview | Motion playback | Mechanism at the slider's input value, traces of displayed points |

Switching to Simulation or Preview automatically adds a default angle driver
(the first link with a pin to the ground link or to construction geometry)
when none exists. Editing tools are disabled outside Construction mode.

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
  creation), colour override with reset to the default colour.
- **Locked** — "preventing changes to the link in any mode": a locked link
  cannot be dragged, its ends cannot be rubber-banded and its geometry is held
  fixed during inverse design (its lengths are not design variables; a locked
  ground link keeps its pivots).
- **Ground link** — see §6.
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
- Vertex position fields in Properties do the same numerically.

## 4. Link-end pop-up

**Original:** "link ends (when clicked) should have a small pop-up with 2
side-by-side icons, one with a symbol representing the current type of
constraint, and the other with an 'X', allowing for the constraint to be
removed. Links should also have a 'lock' toggle."

- Clicking a link end in Select mode selects the vertex and opens a pop-up
  anchored to it with: (1) the glyph of the constraint at that end (S / R /
  planar / P / C / screw, or a dashed circle for *none*) — clicking it cycles
  the joint to the next compatible type; (2) an **X** that removes the
  constraint (disabled when there is none); and (3) a lock icon toggling the
  link's *Locked* flag. The pop-up follows the point when the view moves and
  closes on `Esc` or the next click.

**Deviation:** a third icon (lock) was added to the two specified ones because
the brief asks for a lock toggle on links and the pop-up is the quickest place
for it; the lock is also in Properties.

## 5. Joints / constraints

**Original:** spherical, revolute, planar, prismatic, cylindrical, screw;
"similar in spirit to … Crane".

Tool **Joint**: choose the type in the tool options, click a feature of the
first link, then a compatible feature of a second link or construction
geometry. The geometry is immediately re-solved so the new joint is satisfied
(the second link snaps). Properties of a joint: connected items, change type,
axis, pitch (screw), compliant-hinge settings (revolute), delete.

| Joint | DOF | Compatible features (A ↔ B) | Equations |
| --- | --- | --- | --- |
| Spherical | 3 | vertex ↔ vertex; vertex ↔ datum point | coincidence (merged variable) |
| Revolute | 1 | vertex ↔ vertex (axis = sketch normal, helper points carry the pin); edge/axis ↔ edge/axis (crease: end points merged if equal length, else collinear + slide lock); vertex ↔ edge/axis; vertex/edge/axis ↔ datum axis; vertex ↔ datum point (pin at a fixed point) | 5 |
| Cylindrical | 2 | edge/axis ↔ edge/axis; vertex ↔ edge/axis; … ↔ datum axis | 4 (two point-on-line) |
| Prismatic | 1 | as cylindrical | 4 + reference point in a plane containing the axis |
| Screw | 1 | as cylindrical, *Pitch* per revolution | 4 + slide − pitch·rotation/2π |
| Planar | 3 | face ↔ face; vertex/edge ↔ face; vertex/edge/face/body ↔ datum plane | 1 per constrained point (3 for a face) |

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
  the rigid-body modes (3 planar / 6 spatial) and says so. A "constraints
  violated" flag appears when the current pose does not satisfy the
  constraints (e.g. after an impossible edit).

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
  *Save* does this automatically.
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
  link").

## 11. Settings

**Original:** "Created user geometry should default to #35a4d3 … customizable
with a HWB color picker in a settings menu … colors should be editable in the
settings menu, and the color picker used should also display a HEX value."

- Settings dialog with an HWB picker (hue / whiteness / blackness sliders +
  editable HEX field + swatch) for: default geometry (`#35a4d3`), ground link,
  construction geometry, design space (`#c7007e`), output path (`#9a0062`),
  editing point free (`#0eb062`) / constrained (`#94140a`), background, grid
  major / minor, selection highlight; design-space opacity (default 0.30);
  show labels / construction geometry / hidden helper points; grid snap and
  step; joint helper offset; *Reset to defaults*. Settings persist in
  `localStorage`.

## 12. Files, examples and export

**Original:** "Saving or exporting the mechanism file should solidify these
assumptions."

- **Save** writes `<name>.linkage.json` (the full model: links with rest
  geometry, joints, datums, drivers, targets, settings) after solidifying the
  current design. **Open** loads such a file. Targets stay in the file and are
  re-imposed on load.
- **Export**: output paths as CSV (point, pose index, input value, x, y, z),
  geometry of the current pose as OBJ, the viewport as PNG.
- **Examples** menu: four-bar crank-rocker, slider-crank, spherical pendulum,
  rigid-origami Miura vertex with sector angles (α, α, π−α, π−α), created in
  an exactly folded configuration so it folds on the generic branch
  (`examples/*.linkage.json` are the same files).

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
  existing vertices.
- **Extrude to prism** (Properties of a polygon): enter a height and press
  *Extrude*. The polygon becomes a prism of that height along the normal of
  its sketch plane (so the direction does not depend on the vertex winding;
  negative = other side). Non-planar polygons cannot be extruded. The bottom face keeps its point ids so joints
  attached to the polygon's vertices / edges / face stay valid; the
  sketch-plane constraint is removed because the body is now 3-D. The prism's
  rigidity is rebuilt from the new positions (joint helper attachments kept).

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
  (shared edges become creases, replacing earlier pins); a vertex dropped on a
  datum point is pinned to it. Locked links cannot be edited.

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
ground / locked / flexible / hidden badges and an eye toggle), Joints,
Construction geometry (marking the active sketch plane), Drivers and
Editing-point constraints. Click selects (Drivers: makes it the active
driver), hovering highlights the item in the viewport, double-click renames
links and user datums. Hidden links are neither drawn nor pickable but still
take part in the solve (useful for scaffolding geometry).

## 16. Out of scope for this prototype

Collision detection, dynamics, fabrication output (thickening, hinge design
as in Crane), free-timing synthesis, multi-selection, measuring tools.
