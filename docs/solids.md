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
  same coordinates merged, so a part from elsewhere can be combined with new geometry and
  printed again.
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
meshes, PYRAMID, POLYSOLID, SWEEP, LOFT, solid fillets and chamfers, SOLIDEDIT, 3DALIGN or
3DARRAY, and no 3MF or STEP import. See F24 in [gaps.md](gaps.md).
