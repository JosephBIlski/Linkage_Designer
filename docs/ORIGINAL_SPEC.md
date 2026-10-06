# Original specification (verbatim)

This is the overview specification the project started from, reproduced
unchanged so that deviations in [SPEC.md](SPEC.md) can be checked against it.

---

Linkage Designer:

Goal: To make a design tool for complex linkages and 3-D origami mechanisms.

Description: A 3D CAD-like tool that is primarily focused on inverse design of mechanisms i.e. aims to let the user manipulate/specify the final output path/motion and determines what the linkage structure should be.

Envisioned workflow and usage for the tool:

- Has a UI similar to CAD software(such as Creo or Rhino) with a 3D editing window along with tools that the user can use to build up mechanisms. Creating and simulating linkages, such as a 4 bar link, should be done by selecting a "Link" tool, and clicking points in the 3D space to generate each link with the user specifying the constraints, and the program handling the simulation of the output motion.

2D Link creation and behavior:
- Clicking on the end of one link should constrain that link to the newly created one. Links should also be able to be placed by specifying a second point through coordinates, length and angle, or just a length and clicking to place the second point. Links should also be able to be dragged and relocated by the user, as well as their properties(such as constraints) edited after creation. To enable easy editing of constraints, link ends(when clicked) should have a small pop-up with 2 side-by-side icons, one with a symbol representing the current type of constraint, and the other with an 'X', allowing for the constraint to be removed. Links should also have a 'lock' toggle, preventing changes to the link in any mode.

Construction Geometry
- The tool should also have the option for 'construction geometry'(axes, planes, points) that can constrain the mechanism geometry that the user inputs. This should function similarly to 'Creo's geometry creation workflow.

- The 3D editor should have gridlines that contrast with a light gray background. Created user geometry should default to #35a4d3, though this should be customizable with a HWB color picker in a settings menu.

Types of constraints:
- Spherical joints
- Revolute joints
- Planar constraints
- Prismatic joint
- Cylindrical joint
- Screw joint

Types of links:
- 2D bar links
- 2D and 3D Polygons with a specifiable number of sides. Each vertex/surface/side should be able to be constrained with compatible constraints.
- Cylinders
- Geometry should have an option to enable 'flexibility', enabling the simulation of bistable/compliant shapes, and editable stiffness.

Program "modes":
- Construction Mode: The user uses the tools at their disposal to create an initial mechanism design, while specifying a ground link(that can be later changed). The user should be able to see the degrees of freedom of their mechanism and the current output path/"design space" of selected links
- Simulation Mode: When the user switches to 'simulation mode' the motion preview should display an interactable 'design space' of the mechanism links they have selected to display. The 'design space' should show all of the possible points a selected link or vertex(or other user created geometry) can reach.

For example, a four bar linkage with an 'end vertex' of a link would show a line for the current output motion, while the 'design space' would be a 2D surface, while a spherical joint could show a 2D surface as the current output path, with the 'design space' as a 3D sphere, and similarly for other joints.

The previewed output motion should have draggable (and lockable) "editing points" within a 'design space' that enable inverse design of the mechanism. The "editing points" should appear along the output path of the mechanism link. The mechanism should change to enable the path that the user has dragged or constrained. The editing points should also be able to be constrained to construction geometry. The program should allow for editing points from multiple links to be constrained until the mechanism is completely specified. The user should be able to see how many degrees of freedom they have left, and how many editing points are constrained. The program should show a toggle to allow 'soft-assumptions' to enable a preview of the motion before the mechanism is completely specified. Link and polygon geometry(shape, length) should be able to be edited by dragging the edit points, though it should not change throughout the motion path of a mechanism. Saving or exporting the mechanism file should solidify these assumptions.

Constraints should be similar in spirit to how they are implemented in Crane for Rhino Grasshopper: https://dl.acm.org/doi/10.1145/3576856

Preview mode: A mode for previewing the motion of the mechanism, through the range of a selected link.

The 'design space' should be a semi-transparent(30% opacity) #c7007e(with the output path in a slightly darker and more opaque hue), and un-constrained editing points should be #0eb062, while constrained points turn #94140a. These colors should be editable in the settings menu, and the color picker used should also display a HEX value(to enable me to change the defaults more easily in future iterations.)

All text and labels should be clearly labeled in the code, to enable me to edit the descriptions more easily.

Prioritize referencing research and textbooks for uncertainties. Document design choices, assumptions, and implementation decisions you make.
