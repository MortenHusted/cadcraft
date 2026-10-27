# 3D solids

> **Last reviewed:** 2026-10-27 · **Last updated:** 2026-10-27 · **Change:** major (first version) · **Target:** Autodesk AutoCAD 2027

CADCraft models 3D solids for parts you print or machine. A 3D solid (3DSOLID) holds a
triangle mesh (`crates/geom/src/solid`) meant to be closed; a boolean whose solids touch only
along an edge or at a point gives an open one, and says so (`closed: false`). Planar faces are
exact, curved faces are faceted when
the solid is made (at most 0.01 drawing units from the true surface, with 24 to 720 facets
around a circle; the primitives take `segments` to choose). Every command below is an ordinary
command: the command line, scripts, the CLI and MCP reach it, and the JSON form never opens a
dialog. JSON points are `[x, y, z]` (a missing z is 0).

## Making solids

| Command | JSON parameters |
|---|---|
| BOX | `{corner, opposite}` or `{corner \| center, length, width?, height?}` (a cube when width and height are left out) |
| WEDGE | like BOX; the top slopes down from the face at the first corner's x |
| CYLINDER | `{center, radius \| diameter, height, segments?}` (`center` of the base; a negative height builds downward) |
| CONE | `{center, radius \| diameter, topRadius?, height, segments?}` (a frustum when `topRadius` > 0) |
| SPHERE | `{center, radius \| diameter, segments?}` |
| TORUS | `{center, radius \| diameter, tubeRadius \| tubeDiameter, segments?}` (`radius` to the tube's center) |
| PYRAMID | `{center, radius, sides? (4), inscribed?, height, topRadius?, rotation?}`: `radius` to the middle of a side (to a corner with `inscribed`); a frustum when `topRadius` > 0 |
| POLYSOLID | `{points \| object, closed?, width?, height?, justify?}`: a wall of `width` and `height` along level points or a line, arc, circle, polyline or spline, centred on the path or standing on its `left` or `right`; corners are mitred |
| EXTRUDE | `{handles, height \| direction, keep?}`: circles, full ellipses, closed polylines (fillets and arcs too), closed splines and planar 3D polylines become solids; a region's islands become holes; TEXT in a TrueType font becomes solid lettering (counters as holes), to emboss with UNION or engrave with SUBTRACT |
| REVOLVE | `{handles, axis: [[x,y],[x,y]], angle? (360)}` about an axis in the profiles' plane |
| SWEEP | `{handles, path, twist? (degrees), scale? (1), base?, keep?}`: each profile carried along `path`, a line, arc, circle, ellipse, polyline, 3D polyline, spline or helix (its handle) or a list of points. The profile stands across the path at its start, its centroid (or `base`) on the path, its Y up and its X to the left looking along the path, and turns as little as it can along it; corners are mitred. `twist` turns it and `scale` grows or shrinks it from start to end |
| LOFT | `{handles, ruled?, keep?}`: a solid through closed cross-sections in the order given, each in a plane parallel to XY (raise them with 3DMOVE); a POINT may come first or last (a tip). Smooth through the sections by default, straight between them with `ruled` |

Like AutoCAD with DELOBJ at its default, EXTRUDE, REVOLVE, SWEEP and LOFT erase what made the
solid (profiles, path, cross-sections) unless `keep` is true; each solid takes its profile's
layer and properties.

A thread is a SWEEP of its tooth along a HELIX, united with the core. Along a
counter-clockwise helix the profile's X points at the axis, so the tooth's tip is at -X; `base`
puts the profile's origin (not its centroid) on the helix, so the tooth reaches from 0.6 inside
it to 0.6 outside, its root well inside the core:

```json
{"command": "helix", "params": {"center": [0, 0], "baseRadius": 4.2, "height": 18, "turns": 12}}
{"command": "pline", "params": {"points": [[0.6, -0.7], [0.6, 0.7], [-0.6, 0]], "closed": true}}
{"command": "sweep", "params": {"handles": ["<tooth>"], "path": "<helix>", "base": [0, 0]}}
{"command": "cylinder", "params": {"center": [0, 0, 0], "radius": 3.8, "height": 18}}
{"command": "union", "params": {"handles": ["<core>", "<thread>"]}}
```

## Combining and placing

| Command | JSON parameters |
|---|---|
| UNION, INTERSECT | `{handles}`: the first solid takes the result (and keeps its handle) |
| SUBTRACT | `{from: [handles], remove: [handles]}` |
| 3DMOVE | `{handles, displacement}` or `{handles, from, to}` |
| 3DROTATE | `{handles, base?, axis: "x" \| "y" \| "z" \| [x,y,z], angle}` (degrees) |
| MIRROR3D | `{handles, plane: "xy" \| "yz" \| "zx" \| normal, point?, erase?}` (keeps the originals by default) |
| 3DARRAY | `{handles, rows?, columns?, levels?, spacing: [dx, dy, dz]}`, or `{handles, polar: true, count, angle? (360), axis?, rotate? (true)}` round an axis through two points (default Z through the origin): the copies |
| 3DALIGN | `{handles, from: [p1, p2?, p3?], to: [q1, q2?, q3?]}`: p1 onto q1, the direction p1→p2 along q1→q2, the plane of the three onto the other's |
| SLICE | `{handles, plane \| normal, point?, keep?: "both" \| "above" \| "below"}` |

The booleans return `closed`: whether the result is one watertight solid. It is false only
where solids touch along an edge or at a point, which no closed mesh can represent. MOVE,
ROTATE, SCALE and MIRROR also work on solids in plan; 3DMOVE, 3DROTATE and MIRROR3D move other
objects only when the transform keeps them in plan, and 3DMOVE raises or lowers those that have
a height of their own (lines, circles, arcs, polylines, text, points…).

## Rounding and bevelling edges

FILLETEDGE rounds edges and CHAMFEREDGE bevels them: a convex edge is cut away, a concave one
(an inside corner, such as the foot of a boss) filled. An edge is where two faces meet at more
than 20°, followed for as long as it turns smoothly, so the rim of a cylinder or of a rounded
rectangle is one edge.

| Command | JSON parameters |
|---|---|
| FILLETEDGE | `{handles?, edges?, radius? (1)}` |
| CHAMFEREDGE | `{handles?, edges?, distance? (1), distance2? (= distance), base? ([0,0,1])}`: `distance` runs along the face whose normal is nearest `base` (either way), `distance2` along the other |

`edges` is `"all"` (the default), `"top"` or `"bottom"` (the edges in the solid's highest or
lowest plane), `"vertical"`, `"convex"`, `"concave"`, or a list of points `[x, y, z]` on or near
edges, each picking the nearest. Typed or chosen from the menu, the commands pick edges in plan
(the highest where edges lie over each other) until Enter; `R` (FILLETEDGE) and `D`
(CHAMFEREDGE) change the radius and distances.

The cut follows a ball of the radius rolling along both faces (a bevel, a flat between the two
distances), mitred where the edge turns. Where an edge ends against a face leaning away from it
(a box corner) the cut runs past that face; against a face leaning over it (a wall) it stops
there. Where three edges rounded to one radius meet at a convex corner, a ball rounds the
corner. Round the vertical edges first and then the top rim, and the rim is one smooth edge:

```json
{"command": "filletedge", "params": {"handles": ["1F"], "edges": "vertical", "radius": 5}}
{"command": "filletedge", "params": {"handles": ["1F"], "edges": "top", "radius": 2}}
{"command": "chamferedge", "params": {"handles": ["1F"], "edges": "bottom", "distance": 0.6}}
```

The last line bevels the bottom rim, which keeps the first layers' squash (elephant's foot)
from sticking out.

## Shells, separate pieces and checks

SOLIDEDIT's body options, each also its own menu item and command:

| Command | JSON parameters |
|---|---|
| `solidedit.shell` (Modify › Solid Editing › Shell) | `{handles, thickness, remove?}`: hollows each solid to walls `thickness` thick; `remove` opens faces: `"top"`, `"bottom"` or points on faces (each picks the flat face nearest it) |
| `solidedit.separate` | `{handles}`: splits a solid into its separate pieces, each hollow staying with the piece round it; the first piece keeps the handle |
| `solidedit.check` | `{handles}`: whether each solid is closed, its open, non-manifold and flipped edges, and its pieces |
| SOLIDEDIT | `{mode: "shell" \| "separate" \| "check", ...}`; typed, it asks for the option first |

The hollow is the solid shrunk by the wall thickness: each corner moves to where its faces,
each moved in by the thickness, meet (faces meeting at less than 25° count as one, so curved
faces shrink along their normal), and the faces to open move out instead, so the hollow breaks
through them. A box opened at the top becomes a tray, a lofted vase a vase you can print with
walls; a wall too thick for the solid (its inside would fold over) is refused.

## Checking

- MASSPROP reports volume (mass at unit density), area, bounding box, centroid, moments and
  products of inertia, radii of gyration and `closed` for each solid.
- INTERFERE `{first, second?, create?}` finds the pairs of solids that overlap and their common
  volume (touching faces don't count), and with `create` adds each overlap as a solid: check
  clearances between mating parts before printing them.
- PRINTCHECK `{handles?, limit?, ceiling?, maxOverhang?, bed?}` says whether a solid prints on
  an FDM printer standing as it is (Z up, its lowest point on the bed): whether it is closed,
  fits the build volume `bed` and lies flat on the bed; the area of overhangs leaning more than
  `limit` (46°) from vertical, summed by 5-unit bands of height so the culprit stands out
  (faces past `ceiling`, 85°, are ceilings, not overhangs); and the faces (of at least 0.01 mm²) that start in mid air
  with nothing of the solid under them (including roofs that can only bridge). `ok` is true
  when it finds nothing; `problems` says what it found, in words.
- RENDER `{handles?, view?, width?, height?, edges?, path?}` (default: every visible solid)
  draws a shaded picture of the solids (default south-east isometric; `top`, `front`, `right`,
  `se`, `nw` … or any direction): the way an agent looks at what it built. The MCP `render` tool with `view` returns it as an image.
  Typed or chosen from a menu, RENDER (and 3DORBIT, the tool bar's Orbit) opens the Render window: the solids shaded, preset view
  buttons, drag the picture to orbit; it follows the drawing as you edit.

## Files

- **STL and 3MF.** STLOUT `{handles?, path}` writes binary STL, 3DPRINT `{handles?, path}` a 3MF
  (one object per solid, in the drawing's units) or STL (both default to the selected solids),
  and EXPORT or the CLI's
  `--export part.3mf` write every solid in model space. Slicers (Bambu Studio, PrusaSlicer,
  Cura) open both. OPEN and IMPORT read an STL (binary or ASCII) as one solid, corners with the
  same coordinates merged, and a 3MF (from Bambu Studio, PrusaSlicer or CADCraft) as a solid
  for each mesh on its build plate, placed where the file puts it and in its units, so a part
  from elsewhere can be combined with new geometry and printed again. Support objects in a 3MF
  are left out.
- **STEP.** EXPORT, SAVEAS and the CLI's `--export part.step` (or `.stp`) write model space's
  solids as STEP (AP214) B-reps, in millimetres: each solid a closed shell of flat faces (its
  triangles merged where they share a plane, so a box has six faces) with shared straight edges,
  which CAD programs such as Fusion, Onshape, FreeCAD and SolidWorks open as solids; curved
  surfaces arrive faceted.
- **DXF and DWG.** A solid is saved as a polyface mesh (POLYLINE flag 64; edges between faces
  that meet smoothly are hidden), the closest object an R2000 DXF has: other programs show its
  surface, and CADCraft xdata makes it a solid again when CADCraft opens the file. A solid with
  more than 32,767 vertices is written in parts that join again. 3DSOLIDs made by other programs
  (ACIS) are kept as they were read but can't be shown or edited.

## How the booleans work

Each triangle of one solid is cut by the planes of the other solid's triangles that cross it,
so each piece lies wholly inside, outside or on the other solid. A piece is cut only where the
segment along which the other surface meets the triangle passes through it, and where that
surface turns inside the triangle a cut across the turn comes first, so no cut runs on past its
segment alongside the next one. Each corner where the two surfaces meet is computed once, from
an edge of one solid and a plane of the other, so both solids put it at exactly the same point.
A piece is classified by casting rays from its centre (from points just in front of and just
behind it where it lies on the other solid's surface); the kept pieces of each face are joined
back into one outline and triangulated again, and everything is welded into one mesh with
T-junctions split. Property tests check exact volumes and watertightness for thousands of
box pairs on a coarse grid (coincident and touching faces) and inclusion-exclusion for rotated
boxes, cylinders and spheres, a curved face tessellated into strips 0.01 wide and sides bent by
a hundred-thousandth of a radian.

Meshes made elsewhere whose curved faces are tessellated into long thin strips, as some CAD
exports on MakerWorld are, combine like any other. An input mesh that touches itself along lines,
or that contains inverted triangles (a sliver facing the wrong way, folding a flat face over
itself), can still give a result that isn't closed (`closed: false`); `solidedit.check` and
PRINTCHECK show this, so check a remixed download before printing it.

## Not yet

No 3D view in the drawing area (solids show as plan wireframes; the Render window shows them in
3D), no UCS, surfaces,
meshes or SOLIDEDIT face and edge editing, and no STEP import (STEP export writes faceted
solids, not curved surfaces). LOFT takes
cross-sections parallel to XY without holes, and no guides or path.
FILLETEDGE has no variable radius and rounds only corners where three edges of one radius
meet with a ball; other corners keep where the cuts cross. See F24 in [gaps.md](gaps.md).
