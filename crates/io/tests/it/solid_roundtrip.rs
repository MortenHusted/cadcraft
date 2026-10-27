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
fn a_solid_too_big_for_one_polyface_is_written_in_parts() {
    let ball = primitives::sphere(Vec3::new(1.0, 2.0, 3.0), 50.0, 400).unwrap();
    assert!(ball.vertices.len() > 32_767);
    let s = Solid3d::new(ball);
    let mut d = Drawing::new_metric();
    d.add(&Space::Model, Common::default(), EntityKind::Solid3d(s.clone())).unwrap();
    let saved = cadcraft_io::write_dxf(&d);
    let polylines = entity_records(&saved).into_iter().filter(|(k, _)| k == "POLYLINE").count();
    assert!(polylines >= 2, "{polylines} parts");
    let back = solids(&cadcraft_io::read_dxf(saved.as_bytes()).unwrap());
    assert_eq!(back.len(), 1);
    assert!(back[0].mesh().check().is_closed());
    assert_eq!(back[0].mesh().triangles.len(), s.mesh().triangles.len());
    assert!((back[0].mass_props().volume - s.mass_props().volume).abs() < 1e-6);
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
