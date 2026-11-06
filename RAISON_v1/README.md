# RAISON middle-finger reconstruction

`urdf/RAISON_v1.urdf` includes `middle_proximal`, `middle_intermediate`, and
`middle_distal`, connected directly from `base_link` by `joint_middle_MCP`,
`joint_middle_PIP`, and `joint_middle_DIP`. There is no middle ab/ad joint.

The MCP origin is taken from
`../_legacy/FINGER_v5_sizing-assem_pitch12/urdf/FINGER_v5_sizing-assem_pitch12.urdf`:
XYZ `(0, 0, 0)` m, RPY `(pi/2, 0, 0)` rad. The index and ring base mounting
positions agree between that reference and this package, supporting reuse of
its base frame. This assumes the middle MCP mounting has not changed since v5.

The middle finger currently has a -6 mm base-Z offset along the MCP
axis (toward index). To tune this, edit the third value of `joint_middle_MCP`'s
`<origin xyz="0 0 -0.006" ... />` in `urdf/RAISON_v1.urdf`. Units are meters:
positive Z moves toward ring, negative Z toward index. This translates the
whole finger along the same rotation-axis line without changing its direction.
Keep RPY and the joint axis unchanged. The matching CSV field is
`Joint Origin Z` in the `middle_proximal` row.

All three middle joint axes are local `(0, -1, 0)`, following the legacy
middle-finger convention. This is opposite to the current index/ring axes.
Joint types remain continuous, consistent with the source export; physical
travel limits have not been supplied.

The middle links reuse the current index STL paths, visual origins,
masses, centers of mass, and inertia tensors together. No STL files are
modified. PIP XYZ is `(0.0065, 0, -0.045)` m and DIP XYZ is
`(0.0035, 0, -0.035)` m, both with zero RPY. The older middle PIP X offset
of 0.00525 m is deliberately not used with the newer geometry.

Index and ring STL bounds differ by approximately 3.14433 mm along local Y;
their frame-dependent geometry and inertial values must not be mixed.
The reconstructed middle preserves the index mesh placement in its link
frames. Its exact lateral fit to the middle mount remains to be checked
against the current CAD assembly or hardware; the legacy axis alone does
not establish the newer assembly's placement along that axis.

The CSV includes the three reconstructed rows. Their SolidWorks component,
coordinate-system, and axis-name fields are blank because these are manual
URDF additions rather than exported SolidWorks references. Re-exporting from
SolidWorks may overwrite the reconstruction.

Validation performed: XML parsing, 16 links and 15 joints in one connected
acyclic tree, mesh-file resolution, positive-definite inertia tensors,
consistency of reused donor data, CSV joint origins, and controller-list
entries. RViz/Gazebo and physical fit have not been verified.


## Package and collision model

The active package, robot, URDF, CSV and controller configuration are named
`RAISON_v1`. The `_legacy` packages and historical `export.log` are unchanged.
Launch with `roslaunch RAISON_v1 display.launch` in a ROS 1 workspace.

Finger collision geometry uses native primitives as main-body approximations,
not enclosing bounds. Small fasteners, electronics, lips and some outer shell
surfaces are intentionally omitted. `tools/contact_primitives.py` contains the
editable dimensions and geometry rules:

- Index/middle/ring proximal: two motor cylinders, a PIP hinge cylinder, and
  two narrow connecting boxes (five shapes).
- Intermediate: two hinge cylinders and two connecting boxes (four shapes).
- Distal: hinge and rounded-tip cylinders plus a connecting box (three shapes).
- Thumb metacarpal/proximal: a motor cylinder and three trimmed structure
  approximations; ab/ad carriers use three trimmed structure approximations.
- Thumb distal: two 2 mm contact plates, an IP hinge cylinder and a rounded-tip
  cylinder. The front/back plate exterior planes are measured from the STL.
  Tangential edges are inset 0.5 mm to reduce overshoot at rounded corners.
  All other thumb-tip primitives remain behind both contact planes. The side
  walls are intentionally omitted; this is not a complete watertight shell.

`base_link` has one cylinder at XYZ `(0.0185, -0.0803, -0.005)` m, radius
21 mm and length 30 mm. Its axis is parallel to base Y. Its center was moved
31.657 mm along negative base Y from the CMC origin to match the actuator
body. It approximates the actuator, not the full base structure.

The palm uses the supplied `meshes/FINGER_v6_structure_palm.STL` on the fixed,
collision-only `palm_collision` link. Its default export is in millimeters:
`scale="0.001 0.001 0.001"`, `xyz="0 0 0"`, and `rpy="1.570796327 0 0"`
map it into the base frame. Transformed vertices match the original palm in
`base_link.STL` to less than 0.000004 mm. The original visual already includes
the palm, so no duplicate visual or mass was added. Original inertia and joint
elements, including the -6 mm middle MCP offset, remain unchanged.

Tune each named `<collision>` origin and `<box size>` / `<cylinder radius
length>` in `urdf/RAISON_v1.urdf`; dimensions are meters, RPY is radians.
`python tools/simplify_collisions.py` regenerates the model and overwrites
manual collision edits. To persist tuning, edit `tools/contact_primitives.py`
for fingers, or `BASE_CENTER`, `BASE_RADIUS`, and `BASE_LENGTH` in
`tools/simplify_collisions.py` for the base. Regeneration reapplies the known
palm transform. `fit_collision_primitives.py` supplies an initial partition
for structural regions; those regions are then trimmed to approximate the
main shape instead of enforcing full coverage.

The SolidWorks CSV schema cannot represent compound primitive collisions.
Its collision columns are therefore blank; the URDF is authoritative. Joint
origins in the CSV were synchronized, including the user-tuned middle MCP.
The extra collision-only palm frame is maintained only in the URDF.

Collision geometry has been checked structurally, but simulation contact and
self-collision behavior still require validation in the target simulator.


## Collision refinement checks

Run `python tools/validate_collisions.py` (NumPy and Pillow required) to check
thumb contact-plane position/orientation and obstruction, base placement,
palm alignment, unchanged dynamics/kinematics, and mesh paths.
`validation/collision_report.json` records the results;
`validation/collision_overlay.png` shows selected projections. Colored outlines
are collision proxies; gray lines are visual meshes.

The report also measures the fraction of subdivided mesh triangles enclosed
by a single primitive. This is diagnostic only, not an acceptance criterion:
full mesh coverage is deliberately no longer required. It is not area-weighted
and does not measure physical collision accuracy.

Relative to the previous enclosing fit, summed primitive volume decreased
about 40% for index/middle/ring proximal links, 22% for intermediate links,
16% for distal links, and 37-50% for thumb proximal/metacarpal. The thumb-tip
plates and cylinders reduce summed volume by 82% while preserving the two
major contact planes. Overlaps count in this metric; it is not union volume.
Simulator self-collision and contact dynamics remain untested.
