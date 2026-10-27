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
| EXTRUDE | `{handles, height \| direction, keep?}`: circles, full ellipses, closed polylines (fillets and arcs too), closed splines and planar 3D polylines become solids; a region's islands become holes; TEXT in a TrueType font becomes solid lettering (counters as holes), to emboss with UNION or engrave with SUBTRACT |
| REVOLVE | `{handles, axis: [[x,y],[x,y]], angle? (360)}` about an axis in the profiles' plane |

Like AutoCAD with DELOBJ at its default, EXTRUDE and REVOLVE erase the profiles unless `keep`
is true; each solid takes its profile's layer and properties.

## Combining and placing

| Command | JSON parameters |
|---|---|
| UNION, INTERSECT | `{handles}`: the first solid takes the result (and keeps its handle) |
| SUBTRACT | `{from: [handles], remove: [handles]}` |
| 3DMOVE | `{handles, displacement}` or `{handles, from, to}` |
| 3DROTATE | `{handles, base?, axis: "x" \| "y" \| "z" \| [x,y,z], angle}` (degrees) |
| MIRROR3D | `{handles, plane: "xy" \| "yz" \| "zx" \| normal, point?, erase?}` (keeps the originals by default) |
| SLICE | `{handles, plane \| normal, point?, keep?: "both" \| "above" \| "below"}` |

The booleans return `closed`: whether the result is one watertight solid. It is false only
where solids touch along an edge or at a point, which no closed mesh can represent. MOVE,
ROTATE, SCALE and MIRROR also work on solids in plan; 3DMOVE, 3DROTATE and MIRROR3D move other
objects only when the transform keeps them in plan.

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
  Typed or chosen from a menu, RENDER opens the Render window: the solids shaded, preset view
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
- **DXF and DWG.** A solid is saved as a polyface mesh (POLYLINE flag 64; edges between faces
  that meet smoothly are hidden), the closest object an R2000 DXF has: other programs show its
  surface, and CADCraft xdata makes it a solid again when CADCraft opens the file. A solid with
  more than 32,767 vertices is written in parts that join again. 3DSOLIDs made by other programs
  (ACIS) are kept as they were read but can't be shown or edited.

## How the booleans work

Each triangle of one solid is cut by the planes of the other solid's triangles that cross it,
so each piece lies wholly inside, outside or on the other solid. A piece is classified by casting
rays from points just in front of and just behind it; the kept pieces of each face are joined
back into one outline and triangulated again, and everything is welded into one mesh with
T-junctions split. Property tests check exact volumes and watertightness for thousands of
box pairs on a coarse grid (coincident and touching faces) and inclusion-exclusion for rotated
boxes, cylinders and spheres.

## Not yet

No 3D view in the drawing area (solids show as plan wireframes; the Render window shows them in
3D), no UCS, surfaces,
meshes, PYRAMID, POLYSOLID, SWEEP, LOFT, SOLIDEDIT, 3DALIGN or 3DARRAY, and no STEP import.
FILLETEDGE has no variable radius and rounds only corners where three edges of one radius
meet with a ball; other corners keep where the cuts cross. See F24 in [gaps.md](gaps.md).
