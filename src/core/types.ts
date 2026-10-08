/**
 * Core data model for Linkage Designer.
 *
 * Everything the solver sees is a set of 3-D points. Links are rigid (or
 * flexible) bodies defined by points plus internal "rigidity" constraints,
 * joints are constraints between features (vertices / edges / faces / axes) of
 * two links (or a link and construction geometry), and drivers / targets are
 * additional constraints used for simulation and inverse design.
 *
 * See docs/DESIGN_DECISIONS.md, section "Point-based constraint formulation".
 */

export type Vec3 = [number, number, number];
export type ID = string;

// ---------------------------------------------------------------------------
// Links
// ---------------------------------------------------------------------------

export type LinkKind = 'bar' | 'polygon' | 'prism' | 'cylinder';

export type PointRole =
  | 'vertex' // visible, selectable vertex of a link
  | 'axis' // visible axis end point of a cylinder
  | 'ref' // hidden reference point on a cylinder surface (fixes roll)
  | 'helper'; // hidden helper point created for a joint axis

export interface Point {
  id: ID;
  /** Current position in the construction pose (world units). */
  pos: Vec3;
  /** Owning link. */
  linkId: ID;
  role: PointRole;
  /** Short label such as "A", "B", "V0". */
  name: string;
}

/** Internal rigid-body constraint of a link (distances + coplanarity). */
export type RigidityConstraint =
  /** |a-b| = length. `fixed` distances (helper attachments) are never design variables. */
  | { kind: 'dist'; a: ID; b: ID; length: number; fixed?: boolean }
  /** p lies in the plane of a, b, c. */
  | { kind: 'coplanar'; a: ID; b: ID; c: ID; p: ID }
  /** cos of the angle between (h-p) and (q-p) equals value (length independent). */
  | { kind: 'cos'; h: ID; p: ID; q: ID; value: number }
  /** (b-a) is parallel to (d-c). */
  | { kind: 'parallel'; a: ID; b: ID; c: ID; d: ID }
  /** normalised triple product ((b-a) × (d-c)) · (c-a) equals value. */
  | { kind: 'twist'; a: ID; b: ID; c: ID; d: ID; value: number };

export interface LinkParams {
  /** Number of sides for polygons / prisms. */
  sides?: number;
  /** Radius for polygons (circumradius), prisms and cylinders. */
  radius?: number;
  /** Height for prisms and cylinders (along the local normal / axis). */
  height?: number;
}

export interface Link {
  id: ID;
  kind: LinkKind;
  name: string;
  /** Visible points (bar: [a, b]; polygon: n vertices; prism: 2n vertices; cylinder: [axisA, axisB]). */
  pointIds: ID[];
  /** Hidden points (cylinder reference points, joint-axis helpers). */
  helperIds: ID[];
  params: LinkParams;
  /** Internal rigidity constraints. Lengths are the *design* (rest) geometry. */
  rigidity: RigidityConstraint[];
  /** Locked links cannot be edited in any mode and their geometry is frozen in inverse design. */
  locked: boolean;
  /** Ground link: all of its points are fixed in the world (one ground link at a time by convention). */
  ground: boolean;
  /** Flexible (compliant) link: rigidity constraints act as springs with the given relative stiffness. */
  flexible: boolean;
  /** Relative stiffness in (0, 1]. 1 = nearly rigid. */
  stiffness: number;
  /** Optional per-link colour override (HEX). */
  color?: string;
  /** Hidden links are not drawn or pickable but still take part in the solve. */
  hidden?: boolean;
}

// ---------------------------------------------------------------------------
// Features (what joints attach to)
// ---------------------------------------------------------------------------

export type FeatureKind = 'vertex' | 'edge' | 'face' | 'axis' | 'body';

export interface Feature {
  linkId: ID;
  kind: FeatureKind;
  /**
   * vertex: [p]
   * edge:   [p, q]
   * face:   [p, q, r, ...] (coplanar vertices, in order)
   * axis:   [a, b] (cylinder axis end points)
   * body:   all visible points of the link (used for sketch-plane constraints)
   */
  pointIds: ID[];
}

export interface ConstructionRef {
  constructionId: ID;
}

export function isConstructionRef(x: Feature | ConstructionRef): x is ConstructionRef {
  return (x as ConstructionRef).constructionId !== undefined;
}

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------

export type JointType = 'spherical' | 'revolute' | 'planar' | 'prismatic' | 'cylindrical' | 'screw';

export interface HingeParams {
  /** When true the hinge acts as a torsional spring in simulation. */
  enabled: boolean;
  /** Relative stiffness in (0, 1]. */
  stiffness: number;
  /** Rest dihedral angle in degrees. */
  restAngle: number;
}

export interface Joint {
  id: ID;
  type: JointType;
  a: Feature;
  b: Feature | ConstructionRef;
  /** Joint axis direction in world coordinates (unit), for axis-type joints created from vertices. */
  axis?: Vec3;
  /** Screw pitch: translation per full revolution (world units). */
  pitch?: number;
  /** Optional torsional spring on a revolute joint (compliant hinge / origami crease). */
  hinge?: HingeParams;
  /**
   * Helper points created to materialise the joint axis on each body:
   * [helperOnA, helperOnB]. Removed together with the joint.
   */
  helpers?: ID[];
  /** Reference points used by prismatic / screw joints to measure rotation: [refOnA, refOnB]. */
  refs?: ID[];
  /** Internal numbers captured at creation (slide offset, rotation offset, prismatic guide-plane normal). */
  offsets?: { slide?: number; angle?: number; planeNormal?: Vec3 };
  /** Point pairs that coincide (merged into one solver variable). */
  pairs?: [ID, ID][];
  /**
   * Origami crease data (edge–edge revolute with two merged pairs; see fold.ts isCrease). Optional, so files
   * written before it existed load unchanged.
   *  target: fold angle the user wants (degrees, the dihedral magnitude: 180 = flat, 0 = fully closed);
   *  mv: mountain / valley assignment in the convention of fold.ts creaseMV (valley = the second panel is bent
   *      toward the side of the first panel's face normal), used as the sign of the target and as the preferred
   *      side when a flat vertex is pre-folded. Neither field moves geometry by itself.
   */
  fold?: { target?: number; mv?: 'M' | 'V' };
}

// ---------------------------------------------------------------------------
// Construction geometry (datum points / axes / planes, Creo style)
// ---------------------------------------------------------------------------

export type ConstructionKind = 'point' | 'axis' | 'plane';

export interface Construction {
  id: ID;
  kind: ConstructionKind;
  name: string;
  origin: Vec3;
  /** Axis direction or plane normal (unit). */
  dir?: Vec3;
  /** In-plane x direction for planes (unit, perpendicular to dir). */
  xDir?: Vec3;
  /** Display size (half-extent) for planes and axes. */
  size?: number;
  /** Built-in datum geometry (TOP / FRONT / RIGHT planes, X/Y/Z axes, origin) cannot be deleted. */
  builtin?: boolean;
}

// ---------------------------------------------------------------------------
// Drivers (inputs that are swept in simulation / preview)
// ---------------------------------------------------------------------------

export type DriverKind = 'angle' | 'fold' | 'slide';

export interface Driver {
  id: ID;
  kind: DriverKind;
  /** angle: rotate tip about pivot around `axis`, measured from `ref`. */
  linkId?: ID;
  pivotId?: ID;
  tipId?: ID;
  axis?: Vec3;
  ref?: Vec3;
  /** fold / slide: joint being driven. */
  jointId?: ID;
  /** Current value: degrees for angle/fold, world units for slide. */
  value: number;
  /** Last computed reachable range [min, max] (same units as value). */
  range?: [number, number];
  /** True when a full revolution is possible (crank). */
  isCrank?: boolean;
}

// ---------------------------------------------------------------------------
// Targets (inverse-design "editing point" constraints)
// ---------------------------------------------------------------------------

export type TargetKind = 'position' | 'onPoint' | 'onAxis' | 'onPlane';

export interface Target {
  id: ID;
  /** Point whose motion is being edited. */
  pointId: ID;
  /** Pose index 0..poseCount-1 along the sampled motion. */
  pose: number;
  kind: TargetKind;
  /** For 'position'. */
  position?: Vec3;
  /** For 'onPoint' / 'onAxis' / 'onPlane'. */
  constructionId?: ID;
  /** Locked editing points cannot be dragged. */
  locked: boolean;
}

// ---------------------------------------------------------------------------
// Model
// ---------------------------------------------------------------------------

export interface ModelSettings {
  /** Number of sampled poses along the driver range used for inverse design. */
  poseCount: number;
  /** Regularise under-determined inverse design toward the current design ("soft assumptions"). */
  softAssumptions: boolean;
  /** Active sketch plane (construction plane id) new 2-D geometry is placed on. */
  sketchPlaneId: ID;
  /** Points whose output path / design space is displayed. */
  displayPointIds: ID[];
  /** Default joint created when a new link end snaps onto an existing link end. */
  defaultJoint: JointType;
  /** Height of helper points above a joint vertex (world units). */
  helperOffset: number;
}

export interface Model {
  version: 1;
  points: Record<ID, Point>;
  links: Record<ID, Link>;
  joints: Record<ID, Joint>;
  construction: Record<ID, Construction>;
  drivers: Driver[];
  targets: Target[];
  settings: ModelSettings;
  nextId: number;
}

export const DEFAULT_SETTINGS: ModelSettings = {
  poseCount: 16,
  softAssumptions: true,
  sketchPlaneId: 'plane_top',
  displayPointIds: [],
  defaultJoint: 'revolute',
  helperOffset: 1,
};
