/**
 * ALL user-visible text lives in this file so wording can be edited in one place.
 * Keys are grouped by UI area. Keep values short; tooltips may be longer.
 */
import type { Diagnosis } from '../core/feasibility';
import type { FoldReason } from '../core/fold';

/** Compact number for status messages: up to four significant digits, no trailing zeros ("2", "2.1", "0.6878"). */
const num = (x: number): string => String(Number(x.toPrecision(4)));

export const APP = {
  title: 'Linkage Designer',
  subtitle: 'Inverse design of linkages & origami mechanisms — prototype',
  untitled: 'Untitled mechanism',
};

export const MODES = {
  construction: 'Construction',
  simulation: 'Simulation',
  preview: 'Preview',
  tooltips: {
    construction: 'Build the mechanism: links, joints, construction geometry, ground link.',
    simulation: 'Inverse design: drag editing points along the output path; the mechanism adapts.',
    preview: 'Animate the mechanism through the range of the selected driver.',
  },
};

export const MENU = {
  file: 'File',
  newModel: 'New',
  open: 'Open…',
  save: 'Save (.linkage.json)',
  exportCsv: 'Export output paths (CSV)',
  exportObj: 'Export geometry (OBJ)',
  exportPng: 'Export image (PNG)',
  examples: 'Examples',
  settings: 'Settings',
  undo: 'Undo',
  redo: 'Redo',
  help: 'Help',
  view: 'View',
  viewTop: 'Top',
  viewFront: 'Front',
  viewRight: 'Right',
  viewIso: 'Isometric',
  viewFit: 'Zoom to fit',
  viewOrtho: 'Orthographic',
  viewPersp: 'Perspective',
  confirmNew: 'Discard the current mechanism and start a new one?',
  confirmLoad: 'Replace the current mechanism with the loaded file?',
  loadError: 'Could not read the file: ',
};

export const EXAMPLES: Record<string, { label: string; description: string }> = {
  fourBar: { label: 'Four-bar crank-rocker', description: 'Grashof four-bar with a triangular coupler; coupler point C traces a coupler curve.' },
  sliderCrank: { label: 'Slider-crank', description: 'Crank, connecting rod and a slider on a prismatic joint to a construction axis.' },
  sphericalPendulum: { label: 'Spherical pendulum', description: 'A bar on a spherical joint: output path is a sphere, design space is a ball.' },
  miuraVertex: { label: 'Rigid origami vertex (Miura)', description: 'Four rigid panels around a degree-4 Miura vertex (sector angles α, α, π−α, π−α); one folding degree of freedom driven by a crease angle.' },
};

export const TOOLS = {
  select: { label: 'Select', hint: 'Click to select. Drag links or link ends to relocate them. Click a link end for its constraint pop-up.' },
  bar: { label: 'Link', hint: 'Click the first point, then the second point. Click an existing link end to join to it. Or type a coordinate, "length<angle" or a length below.' },
  polygon: { label: 'Polygon', hint: 'Click the centre, then a point on the circumcircle. Set the number of sides in the tool options.' },
  prism: { label: 'Prism (3-D polygon)', hint: 'Click the centre, then a point on the circumcircle. Height and sides are set in the tool options.' },
  cylinder: { label: 'Cylinder', hint: 'Click the start of the axis, then the end of the axis. Radius is set in the tool options.' },
  cpoint: { label: 'Datum point', hint: 'Click to place a construction point (or type coordinates below).' },
  caxis: { label: 'Datum axis', hint: 'Click two points to define a construction axis.' },
  cplane: { label: 'Datum plane', hint: 'Click three points for a plane, or choose "Offset" in the tool options and click a base plane.' },
  joint: { label: 'Joint', hint: 'Choose a joint type, click a feature on the first link, then a compatible feature on the second link (or construction geometry).' },
  ground: { label: 'Ground', hint: 'Click a link to make it the ground (fixed) link.' },
  driver: { label: 'Driver', hint: 'Click a link with a grounded pivot to drive its angle, or a revolute/prismatic joint to drive its fold angle / slide.' },
  delete: { label: 'Delete', hint: 'Click an item to delete it (Del key deletes the selection).' },
  sketch: { label: 'Sketch polygon', hint: 'Click the vertices of a polygon on the sketch plane. Click the first vertex again, double-click or press Enter to close it. Extrude it later from its Properties.' },
  edit: { label: 'Edit points', hint: 'Click a vertex of existing geometry, then click where it should go: another link\'s vertex (joins them), a datum point, or type coordinates below.' },
  mirror: { label: 'Mirror', hint: 'Click a link, then click a datum plane to create its mirror image.' },
  pattern: { label: 'Pattern', hint: 'Click a link, then two points for the spacing (linear) or a datum axis / centre point (polar). Count and angle are in the tool options.' },
  cancel: 'Esc cancels the current tool.',
};

export const TOOL_OPTIONS = {
  sides: 'Sides',
  radius: 'Radius',
  height: 'Height',
  mode2d: 'Place on sketch plane (2-D)',
  planeMode: 'Plane definition',
  planeThree: 'Through 3 points',
  planeOffset: 'Offset from a plane',
  offset: 'Offset',
  jointType: 'Joint type',
  pitch: 'Pitch (per revolution)',
  snapGrid: 'Snap to grid',
  gridStep: 'Grid step',
  lengthLock: 'Length',
  patternKind: 'Pattern type',
  patternLinear: 'Linear array (two points set the spacing)',
  patternPolar: 'Polar array (about a datum axis or a point)',
  patternCount: 'Copies',
  patternAngle: 'Total angle (°)',
  extrudeHeight: 'Extrude height',
};

export const JOINTS: Record<string, { label: string; short: string; dof: string; description: string }> = {
  spherical: { label: 'Spherical joint', short: 'S', dof: '3 DOF', description: 'Ball joint: two vertices coincide; all rotations free.' },
  revolute: { label: 'Revolute joint', short: 'R', dof: '1 DOF', description: 'Pin / hinge: rotation about one axis (vertex–vertex pin, or edge–edge crease).' },
  planar: { label: 'Planar constraint', short: 'E', dof: '3 DOF', description: 'A face, edge or vertex stays in a plane (another face or a construction plane).' },
  prismatic: { label: 'Prismatic joint', short: 'P', dof: '1 DOF', description: 'Slider: translation along an axis without rotation.' },
  cylindrical: { label: 'Cylindrical joint', short: 'C', dof: '2 DOF', description: 'Rotation about and translation along a common axis.' },
  screw: { label: 'Screw joint', short: 'H', dof: '1 DOF', description: 'Helical: translation coupled to rotation by the pitch.' },
  // not a joint type: the presentation of an edge–edge revolute whose end points are merged (fold.ts isCrease)
  crease: { label: 'Crease', short: 'Cr', dof: '1 DOF', description: 'Origami crease: two panels share an edge and fold about it. Drawn along the edge in the mountain (M) / valley (V) colour of its current fold angle, grey while flat.' },
};

/**
 * "Crease M 160°": a crease's label from its mountain / valley class and its fold angle (the dihedral magnitude,
 * rounded to whole degrees; 180° = flat). The class is omitted while the crease is flat, the angle when it cannot be
 * measured.
 */
export function creaseLabel(mv: 'M' | 'V' | null, deg: number | null): string {
  if (deg === null) return JOINTS.crease.label;
  return `${JOINTS.crease.label} ${mv ? `${mv} ` : ''}${Math.round(Math.abs(deg))}°`;
}

export const FEATURES = {
  vertex: 'vertex',
  edge: 'edge',
  face: 'face',
  axis: 'axis',
  body: 'body',
  construction: 'construction geometry',
  of: 'of',
};

export const PANEL = {
  properties: 'Properties',
  nothingSelected: 'Nothing selected. Pick a tool on the left or select an item in the viewport.',
  name: 'Name',
  type: 'Type',
  length: 'Length',
  sides: 'Sides',
  radius: 'Radius',
  height: 'Height',
  color: 'Colour (override)',
  resetColor: 'Use default colour',
  locked: 'Locked (no edits in any mode)',
  ground: 'Ground link (fixed)',
  // Sketch-plane (2-D body) constraint, presented as a link property (docs/CONSTRUCTION_PLAN.md, item 0d)
  keepOnSketchPlane: 'Keep on sketch plane (2-D)',
  keepOnSketchPlaneHelp: 'Holds every vertex of this link on the active sketch plane, the constraint geometry drawn in 2-D mode gets. Untick it to let the link fold or lift out of the plane. The mechanism is re-solved; a pose that cannot satisfy the change is refused.',
  onPlane: (planeName: string): string => `on ${planeName}`,
  flexible: 'Flexible (compliant) link',
  stiffness: 'Relative stiffness',
  showPath: 'Show output path & design space for',
  deleteItem: 'Delete',
  extrude: 'Extrude to prism',
  extrudeHelp: 'Turns this polygon into a prism of the given height along its normal (negative = other side).',
  extrudeRefused: (gap: number): string =>
    `Not extruded: the current pose violates the constraints (gap ${num(gap)} units), so the polygon's shape cannot be taken as its design. Undo the last edit or release a joint first.`,
  copyLink: 'Duplicate',
  hidden: 'Hidden (not drawn, still simulated)',
  position: 'Position',
  pointConstraints: 'Constraints at this point',
  none: 'none',
  jointAxis: 'Axis',
  jointPitch: 'Pitch',
  // Crease (edge–edge revolute with merged end points) properties; angles are dihedral magnitudes, 180° = flat
  creaseAngle: 'Fold angle (current)',
  creaseAngleValue: (deg: number, mv: 'M' | 'V' | null): string => `${deg.toFixed(1)}° · ${mv === 'M' ? PANEL.creaseMountain : mv === 'V' ? PANEL.creaseValley : PANEL.creaseFlat}`,
  creaseMountain: 'Mountain (M)',
  creaseValley: 'Valley (V)',
  creaseFlat: 'flat',
  creaseMV: 'Mountain / valley',
  creaseMVUnset: '— not assigned —',
  creaseMVHelp: 'Marks the crease as a mountain or a valley fold, seen from the face side of its first panel (the sketch-plane side for panels drawn on it). Moves nothing by itself: it sets the direction of "Fold to target" and the side a flat vertex is pre-folded to.',
  creaseTarget: 'Target fold angle (°)',
  creaseTargetHelp: 'Fold angle the crease should take: 180° = flat, 0° = fully closed. Stored with the crease and applied by "Fold to target".',
  creaseFoldTo: 'Fold to target',
  creaseFoldToHelp: 'Re-solves the mechanism so this crease reaches the target angle in the chosen mountain / valley direction. A flat vertex is pre-folded first and the sketch-plane constraints of its panels are removed. Refused, with nothing changed, when the other constraints and drivers do not allow it.',
  creaseDrive: 'Drive this crease',
  creaseDriveHelp: 'Adds a fold driver on this crease and makes it the active input of the simulation and preview. A flat vertex is pre-folded first so that every crease moves.',
  jointHinge: 'Compliant hinge (torsional spring)',
  hingeRest: 'Rest angle (°)',
  hingeStiffness: 'Hinge stiffness',
  jointLinks: 'Connects',
  changeType: 'Change joint type',
  constructionOrigin: 'Origin',
  constructionDir: 'Direction / normal',
  builtin: 'Built-in datum (cannot be deleted)',
  sketchPlane: 'Use as sketch plane',
  isSketchPlane: 'Active sketch plane',
  driverValue: 'Current value',
  driverKind: 'Driver kind',
  driverRemove: 'Remove driver',
  editPoint: 'Editing point',
  editPointPose: 'Pose',
  editPointTarget: 'Target',
  editPointFree: 'free (follows the design)',
  editPointLocked: 'Locked',
  editPointRelease: 'Release (remove constraint)',
  constrainTo: 'Constrain to construction geometry',
  constrainToNone: '— choose —',
  mechanism: 'Mechanism',
  links: 'links',
  joints: 'joints',
  pointsShown: 'points displayed',
};

export const SIM = {
  panelTitle: 'Simulation & inverse design',
  driver: 'Driver (input)',
  noDriver: 'No driver defined. Use the Driver tool, or a default is chosen automatically.',
  addDriver: 'Add driver',
  range: 'Range',
  crank: 'full rotation (crank)',
  rocker: 'limited range (rocker)',
  poseCount: 'Sampled poses (editing points)',
  softAssumptions: 'Soft assumptions (preview under-specified designs)',
  softAssumptionsHelp: 'When on, remaining design freedom is resolved by staying as close as possible to the current design, so a motion preview is always shown. When off, the preview is only shown once the design is fully specified.',
  solidify: 'Solidify assumptions',
  solidifyHelp: 'Commit the current link geometry as the design. Saving does this automatically. Refused while the pose violates the constraints.',
  // Rest-geometry baking guard: a violated pose is never made the design (docs/CONSTRUCTION_PLAN.md, item 0c)
  solidified: 'Assumptions solidified: the current geometry is now the design.',
  solidifyRefused: (gap: number): string =>
    `Assumptions not solidified: the current pose violates the constraints (gap ${num(gap)} units). Undo the last edit or release a joint, then solidify again.`,
  savedUnsolidified: (gap: number): string =>
    `Saved without solidifying: the current pose violates the constraints (gap ${num(gap)} units), so the file keeps the previous design geometry together with the violated positions.`,
  motionDOF: 'Mechanism DOF',
  designDOF: 'Design DOF left',
  editPointsConstrained: 'Editing points constrained',
  fullySpecified: 'Design fully specified',
  overConstrained: 'Targets cannot all be met (over-constrained or unreachable). Release an editing point.',
  notGrounded: 'No ground link: set one with the Ground tool.',
  underSpecifiedHidden: 'Design not fully specified. Enable soft assumptions to preview the motion.',
  designSpaceDim: ['fully determined (path only)', 'curve (1-D)', 'surface (2-D)', 'volume (3-D)'],
  designSpaceFor: 'Design space of',
  showDesignSpace: 'Show design space',
  showEditPoints: 'Show editing points',
  showPath: 'Show output path',
  clearTargets: 'Release all editing points',
  rigidBodyModes: 'includes rigid-body modes of the floating assembly',
  solveFailed: 'The mechanism cannot be assembled at this input value.',
  timing: 'Timing: prescribed (each editing point keeps its input value)',
  // Shown in the Mechanism panel and as the DOF chip tooltip when creasesLockedByPlane is non-empty
  creasesLocked: (n: number): string =>
    `${n} crease${n === 1 ? '' : 's'} cannot fold: both panels are kept on the sketch plane. Untick 'Keep on sketch plane' in Properties, or use Fold.`,
  // Fold (pre-fold) command for flat origami vertices (docs/CONSTRUCTION_PLAN.md, item 1a)
  flatVertices: (n: number): string =>
    n === 1
      ? 'A vertex is flat: all of its creases are unfolded, a singular state in which a simulation would fold only two of them.'
      : `${n} vertices are flat: all of their creases are unfolded, a singular state in which a simulation would fold only two creases of each.`,
  fold: 'Fold (pre-fold flat vertex)',
  foldHelp: 'Drives one crease of a flat vertex to 160° and re-solves the panels so that every crease leaves the flat state (the generic folding branch). The sketch-plane constraints of the panels involved are removed; a model without a driver keeps the fold driver. Undo restores the flat vertex.',
};

export const PREVIEW = {
  panelTitle: 'Motion preview',
  play: 'Play',
  pause: 'Pause',
  speed: 'Speed',
  value: 'Input value',
  trace: 'Trace displayed points',
  range: 'Range',
};

export const TREE = {
  title: 'Model tree',
  links: 'Links',
  joints: 'Joints',
  construction: 'Construction geometry',
  drivers: 'Drivers',
  targets: 'Editing-point constraints',
  empty: '— none —',
  ground: 'ground',
  locked: 'locked',
  flexible: 'flexible',
  hidden: 'hidden',
  planar2d: '2-D',
  crease: 'Crease',
  show: 'Show',
  hide: 'Hide',
  rename: 'Double-click to rename',
  pose: 'pose',
};

export const STATUS = {
  ready: 'Ready',
  dof: 'DOF',
  violated: 'Constraints violated',
  coordPlaceholder: 'x,y[,z]  |  @dx,dy  |  length<angle  |  length',
  coordHelp: 'Type a coordinate and press Enter while placing points.',
  snapped: 'snapped to',
  pose: 'pose',
  solving: 'solving…',
  dragHint: 'Drag the editing point. Shift: free direction. Esc: cancel.',
  groundSet: 'Ground link set',
  driverSet: 'Driver added',
  jointCreated: 'Joint created',
  jointIncompatible: 'These features cannot be connected with this joint type.',
  // Joint pre-flight refusals (the model is restored; nothing moves)
  jointRefusedIncompatible: 'These features cannot be connected with this joint type. Nothing was changed.',
  jointRefusedEdgeLengths: (la: number, lb: number): string =>
    `The two edges have different lengths (${num(la)} and ${num(lb)} units), so they cannot fold as one crease. Edit one panel so the edges match, or pick two edges of equal length. Nothing was moved.`,
  jointRefusedSector: (vertexName: string, sumDeg: number, constrained2d: boolean): string =>
    `The panels around vertex ${vertexName} have corner angles that add up to ${sumDeg.toFixed(1)}°, not 360°, so they cannot lie flat around it. ` +
    (constrained2d
      ? 'They can only meet by folding out of the sketch plane: remove the 2-D constraint from these panels, or change their shapes. '
      : 'Change the panel shapes so the angles add up to 360°, or build the vertex in its folded shape. ') +
    'Nothing was moved.',
  jointRefusedNeeds3d: 'This joint can only be satisfied out of the sketch plane. Remove the 2-D constraint from the links involved (or sketch them in 3-D mode) and add the joint again. Nothing was moved.',
  jointRefusedInfeasible: (residual: number): string =>
    `This joint cannot be satisfied together with the existing constraints (a gap of ${num(residual)} units remains). Remove a conflicting joint or move the links into place before joining. Nothing was moved.`,
  editRefused: 'The constraints cannot all be satisfied with this change, so it was not applied.',
  // Fold (pre-fold) command and the flat-vertex hints around it
  folded: (creaseName: string, dof: number): string => `Vertex pre-folded about crease ${creaseName}. Mechanism DOF: ${dof}.`,
  foldFailed: (reason: FoldReason): string => {
    switch (reason) {
      case 'noLoop':
        return 'Nothing to fold: creases must close a loop of at least three panels around one vertex.';
      case 'notFlat':
        return 'The vertex is already folded; there is nothing to pre-fold.';
      case 'locked':
        return 'A panel of this vertex is locked. Unlock it before folding.';
      case 'noBranch':
        return 'No folded state was found for this vertex: every crease is collinear with another one (an "X" vertex folds only as a straight hinge), or the panels cannot all leave the plane. Nothing was changed.';
      case 'unreachable':
        return 'The crease cannot be folded to that angle together with the other constraints and drivers (a driver on another crease of the vertex, or a target on the other side of flat, may hold it). Nothing was changed.';
    }
  },
  creaseFolded: (creaseName: string, dof: number): string => `Folded to target: ${creaseName}. Mechanism DOF: ${dof}.`,
  prefolded: 'The flat vertex was pre-folded first so that every crease moves.',
  flatVertexWarning: 'This crease belongs to a flat vertex that could not be pre-folded: the simulation may fold only two of its creases.',
  vertexFlatHint: 'The vertex is flat: use Fold to pre-fold it before simulating.',
  jointSameLink: 'Pick a feature on a different link.',
  pickSecondFeature: 'Now pick a compatible feature on another link or construction geometry.',
  linkLocked: 'This link is locked.',
  nothingToUndo: 'Nothing to undo.',
  dragNothing: 'Nothing to drag here — grab a link body (bar, edge or face); zoom in (F) if the link is too thin to hit.',
  sketchNeedsThree: 'A polygon needs at least three vertices.',
  sketchNotPlanar: 'Vertices must lie on one plane; the point was projected onto the sketch plane.',
  sketchOffPlane: 'The snapped vertices define a plane off the sketch plane; the polygon was placed on that plane (no 2-D constraint).',
  sketchDegenerate: 'Three consecutive vertices are collinear; move or remove one (Backspace removes the last vertex).',
  editUnreachable: 'The vertex cannot reach that position with its current joints. Remove a joint or pick a destination it can reach.',
  editPickTarget: 'Now click the destination (a vertex, a datum point) or type coordinates.',
  editDone: 'Vertex moved',
  mirrorPickPlane: 'Now click a datum plane to mirror across.',
  patternPickFirst: 'Now click the first point of the spacing vector (or a datum axis / point for a polar pattern).',
  patternPickSecond: 'Now click the second point of the spacing vector.',
  patternDone: 'Pattern created',
  copied: 'Copied — press Ctrl+V to paste a copy.',
  pasted: 'Pasted',
  nothingToCopy: 'Select a link to copy.',
  extruded: 'Polygon extruded to a prism',
};

/** Status-bar message for a refused joint (tryAddJoint / tryChangeJointType diagnosis). */
export function jointRefusedMessage(d: Diagnosis): string {
  switch (d.kind) {
    case 'incompatible':
      return STATUS.jointRefusedIncompatible;
    case 'edgeLengths':
      return STATUS.jointRefusedEdgeLengths(d.la, d.lb);
    case 'sectorSum':
      return STATUS.jointRefusedSector(d.vertexName, d.sumDeg, d.constrained2d);
    case 'needs3d':
      return STATUS.jointRefusedNeeds3d;
    case 'infeasible':
      return STATUS.jointRefusedInfeasible(d.residual);
  }
}

export const POPUP = {
  constraintTitle: 'Constraint at this link end',
  removeConstraint: 'Remove constraint',
  addConstraint: 'Add constraint',
  changeConstraint: 'Change constraint type',
  noConstraint: 'No constraint',
  onSketchPlane: (planeName: string): string => `on sketch plane ${planeName}`,
  lockLink: 'Lock link',
  unlockLink: 'Unlock link',
};

export const SETTINGS = {
  title: 'Settings',
  colors: 'Colours',
  geometry: 'Default geometry colour',
  ground: 'Ground link colour',
  construction: 'Construction geometry colour',
  designSpace: 'Design space',
  designSpaceOpacity: 'Design space opacity',
  outputPath: 'Output path',
  editPointFree: 'Editing point (unconstrained)',
  editPointConstrained: 'Editing point (constrained)',
  background: 'Viewport background',
  gridMajor: 'Grid lines (major)',
  gridMinor: 'Grid lines (minor)',
  selection: 'Selection highlight',
  mountain: 'Mountain crease',
  valley: 'Valley crease',
  display: 'Display',
  showLabels: 'Show labels',
  showConstruction: 'Show construction geometry',
  showHelpers: 'Show hidden helper points (debug)',
  gridSnap: 'Snap to grid',
  gridStep: 'Grid step',
  resetDefaults: 'Reset to defaults',
  close: 'Close',
  hue: 'Hue',
  whiteness: 'Whiteness',
  blackness: 'Blackness',
  hex: 'HEX',
  helperOffset: 'Joint helper offset',
};

export const HELP = {
  title: 'Quick help',
  lines: [
    'Right-drag: orbit. Middle-drag (or Shift+right-drag): pan. Wheel: zoom.',
    'Link tool: click two points; click an existing link end to join with a pin (default joint type is set in the tool options).',
    'Select tool: drag a link to relocate it (joined links follow). Drag a link end to change its length. Click an end for the constraint pop-up.',
    'Ground tool: click a link to fix it. The DOF readout updates live.',
    'Simulation mode: pick which points to display (Properties → Show output path). Drag the green editing points; the mechanism adapts. Constrained points turn red; lock them in Properties.',
    'Preview mode: play the motion through the driver range.',
    'Sketch tool: click vertices on the sketch plane, close the loop, then Extrude from Properties to make a prism. Edit tool: click a vertex, then its destination, to snap geometry together.',
    'Patterning: Ctrl+C / Ctrl+V duplicates the selected link; Mirror reflects across a datum plane; Pattern makes linear or polar arrays.',
    'Keyboard: Esc cancel, Del delete, Ctrl+Z / Ctrl+Y undo / redo, Ctrl+C / Ctrl+V copy / paste, 1 select, 2 link, 3 polygon, S sketch, E edit, G ground, J joint, F zoom to fit.',
  ],
};

export const LINK_NAMES: Record<string, string> = {
  bar: 'Link',
  polygon: 'Polygon',
  prism: 'Prism',
  cylinder: 'Cylinder',
};

export const CONSTRUCTION_NAMES: Record<string, string> = {
  point: 'PNT',
  axis: 'AXIS',
  plane: 'DTM',
};
