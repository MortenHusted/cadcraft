//! 3D solids survive DXF save and reopen. 3DSOLID's own data is ACIS, which no public
//! specification covers, so a solid is written as a polyface mesh that every DXF reader shows;
//! `CADCRAFT` xdata on it lets CADCraft read the solid back. A solid with more vertices than a
//! polyface can number is written in parts that join again. Hostile marked polyfaces don't panic.

use cadcraft_doc::{Common, Drawing, EntityKind, Solid3d, Space};
use cadcraft_dxf::{records, sections};
use cadcraft_geom::Vec3;
use cadcraft_geom::solid::{BoolOp, boolean, primitives};

fn solids(d: &Drawing) -> Vec<Solid3d> {
    d.model
        .iter()
        .filter_map(|e| match &e.kind {
            EntityKind::Solid3d(s) => Some(s.clone()),
            _ => None,
        })
        .collect()
}

fn entity_records(dxf: &str) -> Vec<(String, Vec<cadcraft_dxf::Tag>)> {
    let tags = cadcraft_dxf::parse(dxf.as_bytes()).unwrap();
    records(&sections(&tags).into_iter().find(|s| s.name == "ENTITIES").unwrap().tags)
}

#[test]
fn solid_survives_save_and_reopen_as_a_polyface() {
    let plate = primitives::cuboid(Vec3::ZERO, Vec3::new(20.0, 10.0, 3.0)).unwrap();
    let hole = primitives::frustum(Vec3::new(10.0, 5.0, -1.0), 2.0, 2.0, 5.0, 48).unwrap();
    let s = Solid3d::new(boolean(&plate, &hole, BoolOp::Difference).unwrap());
    let mut d = Drawing::new_metric();
    let h = d.add(&Space::Model, Common { layer: "Parts".into(), ..Common::default() }, EntityKind::Solid3d(s.clone())).unwrap();

    let saved = cadcraft_io::write_dxf(&d);
    let back = cadcraft_io::read_dxf(saved.as_bytes()).unwrap();
    assert_eq!(solids(&back), vec![s.clone()], "the mesh comes back exactly");
    assert_eq!(back.entity(h).unwrap().common.layer, "Parts");
    assert!(solids(&back)[0].mesh().check().is_closed());

    // Other readers see one polyface mesh: flag 64, the vertex and face counts, and the edges
    // where faces meet smoothly (the plate's diagonals, the hole's facets) hidden.
    let recs = entity_records(&saved);
    let polylines: Vec<_> = recs.iter().filter(|(k, _)| k == "POLYLINE").collect();
    assert_eq!(polylines.len(), 1);
    let flags = polylines[0].1.iter().find(|t| t.code == 70).unwrap().str();
    assert_eq!(flags.trim(), "64");
    let faces: Vec<_> = recs.iter().filter(|(k, t)| k == "VERTEX" && t.iter().any(|g| g.code == 70 && g.str().trim() == "128")).collect();
    assert_eq!(faces.len(), s.mesh().triangles.len());
    let hidden = faces.iter().flat_map(|(_, t)| t.iter().filter(|g| (71..=73).contains(&g.code))).filter(|g| g.str().trim().starts_with('-')).count();
    assert!(hidden > faces.len(), "smooth and flat edges are hidden ({hidden} of {})", faces.len() * 3);
}

#[test]
fn solids_too_big_for_one_polyface_are_written_in_parts_and_come_back_apart() {
    let balls = [Vec3::new(1.0, 2.0, 3.0), Vec3::new(200.0, 0.0, 0.0)].map(|c| Solid3d::new(primitives::sphere(c, 50.0, 400).unwrap()));
    assert!(balls[0].mesh().vertices.len() > 32_767);
    let mut d = Drawing::new_metric();
    for s in &balls {
        d.add(&Space::Model, Common::default(), EntityKind::Solid3d(s.clone())).unwrap();
    }
    let saved = cadcraft_io::write_dxf(&d);
    let polylines = entity_records(&saved).into_iter().filter(|(k, _)| k == "POLYLINE").count();
    assert!(polylines >= 4, "{polylines} parts");
    let back = solids(&cadcraft_io::read_dxf(saved.as_bytes()).unwrap());
    assert_eq!(back.len(), 2);
    for (b, s) in back.iter().zip(&balls) {
        assert!(b.mesh().check().is_closed());
        assert_eq!(b.mesh().triangles.len(), s.mesh().triangles.len());
        assert!((b.mass_props().volume - s.mass_props().volume).abs() < 1e-6);
        assert!(b.bounds().center().dist(s.bounds().center()) < 1e-9);
    }
}

/// A tetrahedron at `x` as a polyface marked part `index` of `count` of the solid whose first
/// part has the handle `first`.
fn tetrahedron_part(handle: &str, first: &str, index: usize, count: usize, x: f64) -> String {
    let mut s = format!(
        "0\nPOLYLINE\n5\n{handle}\n8\n0\n66\n1\n10\n0.0\n20\n0.0\n30\n0.0\n70\n64\n71\n4\n72\n4\n\
         1001\nCADCRAFT\n1000\nSOLID3D\n1005\n{first}\n1070\n{index}\n1070\n{count}\n"
    );
    for [px, py, pz] in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
        s += &format!("0\nVERTEX\n8\n0\n10\n{}\n20\n{py}\n30\n{pz}\n70\n192\n", px + x);
    }
    for [a, b, c] in [[1, 3, 2], [1, 2, 4], [1, 4, 3], [2, 3, 4]] {
        s += &format!("0\nVERTEX\n8\n0\n10\n0.0\n20\n0.0\n30\n0.0\n70\n128\n71\n{a}\n72\n{b}\n73\n{c}\n");
    }
    s + "0\nSEQEND\n8\n0\n"
}

#[test]
fn a_part_joins_only_the_solid_it_is_the_next_part_of() {
    let read = |parts: &[String]| {
        let dxf = format!("0\nSECTION\n2\nENTITIES\n{}0\nENDSEC\n0\nEOF\n", parts.concat());
        solids(&cadcraft_io::read_dxf(dxf.as_bytes()).unwrap()).iter().map(|s| s.mesh().triangles.len()).collect::<Vec<_>>()
    };
    assert_eq!(read(&[tetrahedron_part("20", "20", 0, 2, 0.0), tetrahedron_part("21", "20", 1, 2, 5.0)]), vec![8]);
    // A part after a whole solid, a part of another solid, a part out of turn and a part read
    // twice each stay a solid of their own.
    for parts in [
        vec![tetrahedron_part("20", "20", 0, 1, 0.0), tetrahedron_part("21", "40", 1, 2, 5.0)],
        vec![tetrahedron_part("20", "20", 0, 2, 0.0), tetrahedron_part("21", "40", 1, 2, 5.0)],
        vec![tetrahedron_part("20", "20", 0, 3, 0.0), tetrahedron_part("21", "20", 2, 3, 5.0)],
    ] {
        assert_eq!(read(&parts), vec![4, 4]);
    }
    let twice = [tetrahedron_part("20", "20", 0, 2, 0.0), tetrahedron_part("21", "20", 1, 2, 5.0), tetrahedron_part("22", "20", 1, 2, 9.0)];
    assert_eq!(read(&twice), vec![8, 4]);
}

#[test]
fn hostile_marked_polyfaces_do_not_panic() {
    let marked = |faces: &str, xdata: &str| {
        format!(
            "0\nSECTION\n2\nENTITIES\n0\nPOLYLINE\n5\n20\n8\n0\n66\n1\n10\n0.0\n20\n0.0\n30\n0.0\n70\n64\n71\n3\n72\n1\n\
             1001\nCADCRAFT\n1000\nSOLID3D\n{xdata}\
             0\nVERTEX\n5\n21\n8\n0\n10\n0.0\n20\n0.0\n30\n0.0\n70\n192\n\
             0\nVERTEX\n5\n22\n8\n0\n10\n1.0\n20\n0.0\n30\n0.0\n70\n192\n\
             0\nVERTEX\n5\n23\n8\n0\n10\nnan\n20\n1.0\n30\n0.0\n70\n192\n\
             {faces}0\nSEQEND\n5\n24\n8\n0\n0\nENDSEC\n0\nEOF\n"
        )
    };
    let face = |a: &str, b: &str, c: &str| format!("0\nVERTEX\n5\n30\n8\n0\n10\n0.0\n20\n0.0\n30\n0.0\n70\n128\n71\n{a}\n72\n{b}\n73\n{c}\n");
    for (faces, xdata) in [
        (face("1", "2", "3"), "1070\n0\n1070\n1\n"),
        (face("1", "99999", "-40000"), "1070\n0\n1070\n1\n"),
        (face("0", "0", "0"), "1070\n0\n1070\n1\n"),
        (face("1", "2", "3"), "1070\n5\n1070\n2\n"),
        (face("1", "2", "3"), "1070\n1\n1070\n99999999\n"),
        (face("1", "2", "3"), "1070\n-1\n"),
        (String::new(), "1070\n0\n1070\n1\n"),
    ] {
        let _ = cadcraft_io::read_dxf(marked(&faces, xdata).as_bytes());
    }
}
