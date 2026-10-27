//! Writing 3D solids as STEP (ISO 10303-21 exchange structure, the AP214 automotive design
//! schema), so other CAD programs can open them as solids.
//!
//! Each solid becomes a manifold B-rep: a closed shell of flat faces (its triangles merged
//! where they share a plane, so a box has six) bounded by straight edges, each edge and vertex
//! shared by the faces that meet there. Curved surfaces arrive faceted. Coordinates are written
//! in millimetres.

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;

use cadcraft_doc::Drawing;
use cadcraft_geom::Vec3;
use cadcraft_geom::solid::Mesh;
use cadcraft_geom::solid::shell::faces;

use crate::print3d::PrintObject;
use crate::{IoError, Result};

/// Millimetres per drawing unit for the drawing's `$INSUNITS` (unitless counts as millimetres).
pub fn millimetres_per_unit(d: &Drawing) -> f64 {
    match d.header.i64("INSUNITS", 0) {
        1 => 25.4,
        2 => 304.8,
        5 => 10.0,
        6 => 1000.0,
        13 => 0.001,
        _ => 1.0,
    }
}

/// A STEP real: always with a decimal point.
fn real(v: f64) -> String {
    if !v.is_finite() {
        return "0.".into();
    }
    let s = format!("{v:.10}");
    let s = s.trim_end_matches('0');
    if s == "-." || s == "." || s == "-0." { "0.".into() } else { s.to_string() }
}

/// A STEP string: apostrophes doubled, anything outside printable ASCII left out.
fn text(s: &str) -> String {
    let clean: String = s.chars().filter(|c| c.is_ascii() && !c.is_ascii_control() && *c != '\\').collect();
    format!("'{}'", clean.replace('\'', "''"))
}

/// Numbered entity instances.
#[derive(Default)]
struct Writer {
    lines: Vec<String>,
}

impl Writer {
    fn add(&mut self, body: impl Into<String>) -> usize {
        self.lines.push(body.into());
        self.lines.len()
    }
    fn point(&mut self, p: Vec3) -> usize {
        self.add(format!("CARTESIAN_POINT('',({},{},{}))", real(p.x), real(p.y), real(p.z)))
    }
    fn direction(&mut self, d: Vec3) -> usize {
        self.add(format!("DIRECTION('',({},{},{}))", real(d.x), real(d.y), real(d.z)))
    }
    fn placement(&mut self, origin: Vec3, axis: Vec3, reference: Vec3) -> usize {
        let (o, a, r) = (self.point(origin), self.direction(axis), self.direction(reference));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{a},#{r})"))
    }
}

/// A unit vector across `n`.
fn across(n: Vec3) -> Vec3 {
    let a = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    a.cross(n).normalized()
}

/// The boundary loops of a flat face (triangles `tris` of `m`), each as vertex indices in order,
/// the face on their left.
fn loops(m: &Mesh, tris: &[usize]) -> Vec<Vec<u32>> {
    let mut inside: HashSet<(u32, u32)> = HashSet::new();
    for t in tris.iter().filter_map(|i| m.triangles.get(*i)) {
        inside.extend([(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]);
    }
    let mut boundary: Vec<(u32, u32)> = inside.iter().filter(|(a, b)| !inside.contains(&(*b, *a))).copied().collect();
    // HashSet order varies from run to run; walk the edges in a fixed order.
    boundary.sort_unstable();
    let mut next: HashMap<u32, Vec<u32>> = HashMap::new();
    for (a, b) in &boundary {
        next.entry(*a).or_default().push(*b);
    }
    let mut out = Vec::new();
    for &(a, b) in &boundary {
        // An edge an earlier loop took is gone.
        let Some(list) = next.get_mut(&a) else { continue };
        let Some(k) = list.iter().position(|x| *x == b) else { continue };
        list.swap_remove(k);
        let mut l = vec![a];
        let mut at = b;
        for _ in 0..boundary.len() {
            if at == a {
                break;
            }
            l.push(at);
            let Some(n) = next.get_mut(&at).and_then(Vec::pop) else { break };
            at = n;
        }
        if l.len() >= 3 {
            out.push(l);
        }
    }
    out
}

/// The id of the VERTEX_POINT at mesh vertex `i` (at `p`), written the first time.
fn vertex_id(w: &mut Writer, ids: &mut HashMap<u32, usize>, i: u32, p: Vec3) -> usize {
    if let Some(id) = ids.get(&i) {
        return *id;
    }
    let point = w.point(p);
    let id = w.add(format!("VERTEX_POINT('',#{point})"));
    ids.insert(i, id);
    id
}

/// The STEP file of `objects` (meshes in drawing units, `scale` millimetres per unit), its
/// product called `name`.
pub fn step(objects: &[PrintObject], scale: f64, name: &str) -> Result<Vec<u8>> {
    if objects.iter().all(|o| o.mesh.triangles.is_empty()) {
        return Err(IoError::Format("there are no 3D solids to write (STEP holds solids only)".into()));
    }
    let mut w = Writer::default();
    let app = w.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    w.add(format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{app})"));
    let product_context = w.add(format!("PRODUCT_CONTEXT('',#{app},'mechanical')"));
    let product = w.add(format!("PRODUCT({0},{0},'',(#{product_context}))", text(name)));
    let formation = w.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
    let def_context = w.add(format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"));
    let definition = w.add(format!("PRODUCT_DEFINITION('design','',#{formation},#{def_context})"));
    let shape = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
    let mm = w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
    let rad = w.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
    let sr = w.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
    let uncertainty = w.add(format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#{mm},'distance_accuracy_value','confusion accuracy')"));
    let context = w.add(format!(
        "( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"
    ));
    let origin = w.placement(Vec3::ZERO, Vec3::Z, Vec3::X);
    let mut items = vec![origin];
    for o in objects {
        let m = o.mesh;
        if m.triangles.is_empty() {
            continue;
        }
        let at = |i: u32| m.vertices.get(i as usize).copied().unwrap_or(Vec3::ZERO) * scale;
        let mut vertex: HashMap<u32, usize> = HashMap::new();
        let mut edge: HashMap<(u32, u32), usize> = HashMap::new();
        let mut face_ids = Vec::new();
        for tris in faces(m) {
            let Some(n) =
                tris.first().and_then(|i| m.triangles.get(*i)).and_then(|t| m.corners(t)).map(|[a, b, c]| (b - a).cross(c - a).normalized())
            else {
                continue;
            };
            if n == Vec3::ZERO {
                continue;
            }
            let mut bounds = Vec::new();
            for l in loops(m, &tris) {
                let area = l.iter().zip(l.iter().cycle().skip(1)).fold(Vec3::ZERO, |s, (a, b)| s + at(*a).cross(at(*b)));
                let mut oriented = Vec::with_capacity(l.len());
                for (a, b) in l.iter().zip(l.iter().cycle().skip(1)) {
                    let key = (*a.min(b), *a.max(b));
                    let id = match edge.get(&key) {
                        Some(id) => *id,
                        None => {
                            let (v0, v1) = (vertex_id(&mut w, &mut vertex, key.0, at(key.0)), vertex_id(&mut w, &mut vertex, key.1, at(key.1)));
                            let (p0, p1) = (at(key.0), at(key.1));
                            let start = w.point(p0);
                            let dir = w.direction((p1 - p0).normalized());
                            let vector = w.add(format!("VECTOR('',#{dir},{})", real(p0.dist(p1))));
                            let line = w.add(format!("LINE('',#{start},#{vector})"));
                            let id = w.add(format!("EDGE_CURVE('',#{v0},#{v1},#{line},.T.)"));
                            edge.insert(key, id);
                            id
                        }
                    };
                    let sense = if *a == key.0 { ".T." } else { ".F." };
                    oriented.push(w.add(format!("ORIENTED_EDGE('',*,*,#{id},{sense})")));
                }
                let list = oriented.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
                let lp = w.add(format!("EDGE_LOOP('',({list}))"));
                // The outer loop winds counter-clockwise about the face's normal, holes clockwise.
                let kind = if area.dot(n) > 0.0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                bounds.push(w.add(format!("{kind}('',#{lp},.T.)")));
            }
            let Some(first) = tris.first().and_then(|i| m.triangles.get(*i)).map(|t| at(t[0])) else { continue };
            let placement = w.placement(first, n, across(n));
            let plane = w.add(format!("PLANE('',#{placement})"));
            let list = bounds.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
            face_ids.push(w.add(format!("ADVANCED_FACE('',({list}),#{plane},.T.)")));
        }
        let list = face_ids.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
        let shell = w.add(format!("CLOSED_SHELL('',({list}))"));
        items.push(w.add(format!("MANIFOLD_SOLID_BREP({},#{shell})", text(&o.name))));
    }
    let list = items.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
    let rep = w.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',({list}),#{context})"));
    w.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})"));
    let mut out = String::new();
    let _ = writeln!(out, "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('CADCraft 3D solids'),'2;1');");
    let _ = writeln!(out, "FILE_NAME({},'',(''),(''),'CADCraft','CADCraft','');", text(name));
    let _ = writeln!(out, "FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));\nENDSEC;\nDATA;");
    for (i, l) in w.lines.iter().enumerate() {
        let _ = writeln!(out, "#{}={l};", i + 1);
    }
    out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    Ok(out.into_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_geom::solid::primitives::{cuboid, frustum};
    use cadcraft_geom::solid::{BoolOp, boolean};

    fn count(s: &str, entity: &str) -> usize {
        s.matches(&format!("={entity}(")).count()
    }

    #[test]
    fn a_box_is_six_faces_twelve_edges_and_eight_vertices() {
        let m = cuboid(Vec3::ZERO, Vec3::new(10.0, 20.0, 5.0)).unwrap();
        let s = String::from_utf8(step(&[PrintObject { name: "box".into(), mesh: &m }], 1.0, "part").unwrap()).unwrap();
        assert!(s.starts_with("ISO-10303-21;") && s.trim_end().ends_with("END-ISO-10303-21;"));
        assert_eq!(count(&s, "ADVANCED_FACE"), 6);
        assert_eq!(count(&s, "EDGE_CURVE"), 12);
        assert_eq!(count(&s, "VERTEX_POINT"), 8);
        assert_eq!(count(&s, "ORIENTED_EDGE"), 24);
        assert_eq!(count(&s, "MANIFOLD_SOLID_BREP"), 1);
        assert!(s.contains("CARTESIAN_POINT('',(10.,20.,5.))"));
        // Every instance is defined once, numbered in order.
        for (i, line) in s.lines().filter(|l| l.starts_with('#')).enumerate() {
            assert!(line.starts_with(&format!("#{}=", i + 1)), "{line}");
        }
    }

    #[test]
    fn a_face_with_a_hole_has_an_outer_and_an_inner_bound_and_inches_become_millimetres() {
        let plate = cuboid(Vec3::ZERO, Vec3::new(4.0, 4.0, 1.0)).unwrap();
        let hole = frustum(Vec3::new(2.0, 2.0, -1.0), 1.0, 1.0, 3.0, 24).unwrap();
        let m = boolean(&plate, &hole, BoolOp::Difference).unwrap();
        let s = String::from_utf8(step(&[PrintObject { name: "plate".into(), mesh: &m }], 25.4, "plate").unwrap()).unwrap();
        // Top and bottom each have the hole as an inner bound.
        assert_eq!(count(&s, "FACE_BOUND"), 2);
        assert_eq!(count(&s, "ADVANCED_FACE"), 6 + 24);
        assert!(s.contains("CARTESIAN_POINT('',(101.6,101.6,25.4))"));
        assert!(step(&[], 1.0, "x").is_err());
        // Through `write`: model space's solids, the product named after the file, in inches.
        let mut d = Drawing::new_metric();
        d.header.set_i64("INSUNITS", 1);
        d.add(&cadcraft_doc::Space::Model, cadcraft_doc::Common::default(), cadcraft_doc::EntityKind::Solid3d(cadcraft_doc::Solid3d::new(m)))
            .unwrap();
        let s = String::from_utf8(crate::write(&d, "parts/bracket.stp").unwrap()).unwrap();
        assert!(s.contains("PRODUCT('bracket','bracket'"));
        assert!(s.contains("CARTESIAN_POINT('',(101.6,101.6,25.4))"));
        assert!(crate::write(&Drawing::new_metric(), "empty.step").is_err());
        assert_eq!(real(0.5), "0.5");
        assert_eq!(real(-3.0), "-3.");
        assert_eq!(real(0.0), "0.");
        assert_eq!(text("it's"), "'it''s'");
    }
}
