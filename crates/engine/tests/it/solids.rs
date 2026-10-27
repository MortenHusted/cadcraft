//! 3D solid modeling commands: primitives, EXTRUDE and REVOLVE, the booleans on solids, 3DMOVE,
//! 3DROTATE, MIRROR3D, SLICE, INTERFERE and MASSPROP. Each result must be a closed solid with the
//! volume the geometry says.

use cadcraft_engine::Session;
use cadcraft_engine::doc::{EntityKind, Handle, Solid3d};
use cadcraft_engine::geom::PI;
use serde_json::{Value, json};

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    s.execute(id, &p).unwrap_or_else(|e| panic!("{id} {p}: {e}"))
}

/// The handle a command returned (`handle`, or the first of `handles`).
fn handle(v: &Value) -> String {
    v["handle"].as_str().or_else(|| v["handles"][0].as_str()).unwrap().to_string()
}

fn solid(s: &Session, h: &str) -> Solid3d {
    match &s.doc().unwrap().entity(Handle::parse_hex(h).unwrap()).unwrap().kind {
        EntityKind::Solid3d(so) => so.clone(),
        k => panic!("{h} is a {}", k.type_name()),
    }
}

fn solids(s: &Session) -> Vec<Solid3d> {
    s.doc()
        .unwrap()
        .model
        .iter()
        .filter_map(|e| match &e.kind {
            EntityKind::Solid3d(so) => Some(so.clone()),
            _ => None,
        })
        .collect()
}

#[track_caller]
fn near(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} vs {b}");
}

/// Area of a regular n-gon inscribed in a circle of radius r.
fn ngon(r: f64, n: f64) -> f64 {
    n / 2.0 * r * r * (2.0 * PI / n).sin()
}

#[test]
fn primitives_have_their_volumes() {
    let mut s = Session::new();
    let b = run(&mut s, "box", json!({"corner": [1, 2, 3], "opposite": [4, 6, 8]}));
    near(b["volume"].as_f64().unwrap(), 60.0, 1e-9);
    let cube = run(&mut s, "box", json!({"center": [0, 0, 0], "length": 2}));
    near(cube["volume"].as_f64().unwrap(), 8.0, 1e-9);
    assert_eq!(solid(&s, &handle(&cube)).bounds().min.z, -1.0);
    let w = run(&mut s, "wedge", json!({"corner": [0, 0, 0], "length": 4, "width": 2, "height": 3}));
    near(w["volume"].as_f64().unwrap(), 12.0, 1e-9);
    let c = run(&mut s, "cylinder", json!({"center": [0, 0, 0], "diameter": 4, "height": 5, "segments": 64}));
    near(c["volume"].as_f64().unwrap(), ngon(2.0, 64.0) * 5.0, 1e-9);
    let k = run(&mut s, "cone", json!({"center": [0, 0, 0], "radius": 3, "topRadius": 1, "height": 3, "segments": 64}));
    let (a0, a1) = (ngon(3.0, 64.0), ngon(1.0, 64.0));
    near(k["volume"].as_f64().unwrap(), 3.0 / 3.0 * (a0 + a1 + (a0 * a1).sqrt()), 1e-9);
    run(&mut s, "sphere", json!({"center": [10, 0, 0], "radius": 2}));
    run(&mut s, "torus", json!({"center": [20, 0, 0], "radius": 3, "tubeRadius": 0.5}));
    let all = solids(&s);
    assert_eq!(all.len(), 7);
    assert!(all.iter().all(|so| so.mesh().check().is_closed()));
    // One undo step per command.
    s.execute("undo", &json!({})).unwrap();
    assert_eq!(solids(&s).len(), 6);
}

#[test]
fn bad_primitive_parameters_are_refused() {
    let mut s = Session::new();
    for (id, p) in [
        ("box", json!({"corner": [0, 0, 0], "opposite": [1, 1, 0]})),
        ("box", json!({"corner": [0, 0, 0]})),
        ("box", json!({"corner": [0, 0, 0], "center": [0, 0, 0], "length": 1})),
        ("cylinder", json!({"center": [0, 0, 0], "radius": 1})),
        ("cylinder", json!({"center": [0, 0, 0], "radius": -1, "height": 1})),
        ("cylinder", json!({"center": [0, 0, 0], "radius": 1, "height": 1, "segments": 2})),
        ("sphere", json!({"center": [0, 0, "x"], "radius": 1})),
        ("torus", json!({"center": [0, 0, 0], "radius": 1, "tubeRadius": 2})),
        ("box", json!({"corner": [0, 0, 0], "length": 1e308, "width": 1e308, "height": 1e308})),
        ("extrude", json!({"handles": [], "height": 1})),
    ] {
        assert!(s.execute(id, &p).is_err(), "{id} {p}");
    }
    assert!(solids(&s).is_empty());
}

#[test]
fn a_plate_with_a_hole_and_a_boss() {
    let mut s = Session::new();
    let plate = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [60, 86, 10]})));
    let hole = handle(&run(&mut s, "cylinder", json!({"center": [30, 40, -1], "radius": 8, "height": 12, "segments": 48})));
    let boss = handle(&run(&mut s, "cylinder", json!({"center": [10, 10, 10], "radius": 3, "height": 4, "segments": 48})));
    let r = run(&mut s, "subtract", json!({"from": [plate], "remove": [hole]}));
    assert_eq!(handle(&r), plate, "the first solid keeps its handle");
    assert_eq!(r["closed"], json!(true));
    let r = run(&mut s, "union", json!({"handles": [plate, boss]}));
    let want = 60.0 * 86.0 * 10.0 - ngon(8.0, 48.0) * 10.0 + ngon(3.0, 48.0) * 4.0;
    near(r["volume"].as_f64().unwrap(), want, 1e-6);
    assert_eq!(solids(&s).len(), 1);
    let m = run(&mut s, "massprop", json!({"handles": [plate]}));
    near(m["solids"][0]["volume"].as_f64().unwrap(), want, 1e-6);
    assert_eq!(m["solids"][0]["closed"], json!(true));
    assert!(m["message"].as_str().unwrap().contains("SOLIDS"));
    // Undo brings the separate solids back.
    s.execute("undo", &json!({})).unwrap();
    assert_eq!(solids(&s).len(), 2);
}

#[test]
fn intersect_keeps_the_common_part_and_regions_still_combine() {
    let mut s = Session::new();
    let a = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "length": 10})));
    let b = handle(&run(&mut s, "box", json!({"corner": [5, 5, 5], "length": 10})));
    let r = run(&mut s, "intersect", json!({"handles": [a, b]}));
    near(r["volume"].as_f64().unwrap(), 125.0, 1e-9);
    // Without solids the region booleans answer as before.
    assert!(s.execute("union", &json!({"handles": []})).is_err());
}

#[test]
fn extrude_profiles_into_solids_on_their_layers() {
    let mut s = Session::new();
    let c = run(&mut s, "circle", json!({"center": [0, 0], "radius": 5}));
    let ch = Handle::parse_hex(&handle(&c)).unwrap();
    s.doc_mut().unwrap().modify_entity(ch, |e| e.common.layer = "Parts".into()).unwrap();
    let rect = run(&mut s, "rectang", json!({"p1": [20, 0], "p2": [80, 86], "fillet": 5}));
    let r = run(&mut s, "extrude", json!({"handles": [handle(&c), handle(&rect)], "height": 10}));
    let made = r["handles"].as_array().unwrap();
    assert_eq!(made.len(), 2);
    let disc = solid(&s, made[0].as_str().unwrap());
    let layer = &s.doc().unwrap().entity(Handle::parse_hex(made[0].as_str().unwrap()).unwrap()).unwrap().common.layer;
    assert_eq!(layer, "Parts", "a solid takes its profile's layer");
    let n = cadcraft_engine::geom::solid::primitives::circle_segments(5.0) as f64;
    near(disc.mass_props().volume, ngon(5.0, n) * 10.0, 1e-6);
    let plate = solid(&s, made[1].as_str().unwrap());
    assert!(plate.mesh().check().is_closed());
    // A 60 x 86 plate with 5 mm round corners: the corners lose (4 - π) 25, less the facets.
    let v = plate.mass_props().volume;
    assert!(v < 60.0 * 86.0 * 10.0 - (4.0 - PI) * 25.0 * 10.0 && v > 60.0 * 86.0 * 10.0 - 4.0 * 25.0 * 10.0, "{v}");
    // The profiles are erased (AutoCAD's DELOBJ default) unless kept.
    assert!(s.doc().unwrap().entity(Handle::parse_hex(c["handle"].as_str().unwrap()).unwrap()).is_none());
    let c2 = run(&mut s, "circle", json!({"center": [0, 0], "radius": 1}));
    run(&mut s, "extrude", json!({"handles": [handle(&c2)], "direction": [0, 0, -3], "keep": true}));
    assert!(s.doc().unwrap().entity(Handle::parse_hex(&handle(&c2)).unwrap()).is_some());
    // Open curves are skipped with a reason.
    let l = run(&mut s, "line", json!({"points": [[0, 0], [5, 5]]}));
    let e = s.execute("extrude", &json!({"handles": [handle(&l)], "height": 1})).unwrap_err().to_string();
    assert!(e.contains("not a closed profile"), "{e}");
}

#[test]
fn revolve_a_profile_into_a_ring() {
    let mut s = Session::new();
    let r = run(&mut s, "rectang", json!({"p1": [2, 0], "p2": [3, 4]}));
    let out = run(&mut s, "revolve", json!({"handles": [handle(&r)], "axis": [[0, 0], [0, 1]], "angle": 360}));
    let v = out["volumes"][0].as_f64().unwrap();
    near(v, PI * (9.0 - 4.0) * 4.0, 0.05 * v);
    assert!(solids(&s)[0].mesh().check().is_closed());
}

#[test]
fn move_rotate_and_mirror_in_3d() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [10, 20, 30]})));
    let line = handle(&run(&mut s, "line", json!({"points": [[0, 0], [5, 0]]})));
    let r = run(&mut s, "3dmove", json!({"handles": [b, line], "displacement": [1, 2, 3]}));
    assert_eq!((r["moved"].as_u64(), r["skipped"].as_u64()), (Some(2), Some(0)));
    let bb = solid(&s, &b).bounds();
    assert_eq!((bb.min.x, bb.min.y, bb.min.z), (1.0, 2.0, 3.0));
    // Standing the box on its side: a rotation about X takes the line out of plan, so it stays.
    let r = run(&mut s, "3drotate", json!({"handles": [b, line], "base": [1, 2, 3], "axis": "x", "angle": 90}));
    assert_eq!((r["moved"].as_u64(), r["skipped"].as_u64()), (Some(1), Some(1)));
    let bb = solid(&s, &b).bounds();
    near(bb.max.z - bb.min.z, 20.0, 1e-9);
    near(bb.max.y - bb.min.y, 30.0, 1e-9);
    // Mirrored copies keep the original; volumes stay positive (the mesh still faces out).
    let r = run(&mut s, "mirror3d", json!({"handles": [b], "plane": "xy", "point": [0, 0, 0]}));
    let copy = r["handles"][0].as_str().unwrap().to_string();
    assert_eq!(solids(&s).len(), 2);
    near(solid(&s, &copy).bounds().max.z, -bb.min.z, 1e-9);
    near(solid(&s, &copy).mass_props().volume, 6000.0, 1e-6);
}

#[test]
fn slice_cuts_a_solid_in_two() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "sphere", json!({"center": [0, 0, 0], "radius": 5})));
    let whole = solid(&s, &b).mass_props().volume;
    let r = run(&mut s, "slice", json!({"handles": [b], "plane": "xy", "point": [0, 0, 1]}));
    let parts: Vec<Solid3d> = r["handles"].as_array().unwrap().iter().map(|h| solid(&s, h.as_str().unwrap())).collect();
    assert_eq!(parts.len(), 2);
    assert!(parts.iter().all(|p| p.mesh().check().is_closed()));
    near(parts.iter().map(|p| p.mass_props().volume).sum(), whole, 1e-6);
    assert!(parts[0].bounds().min.z >= 1.0 - 1e-9 && parts[1].bounds().max.z <= 1.0 + 1e-9);
}

#[test]
fn a_solid_named_twice_is_sliced_once() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [2, 2, 2]})));
    let r = run(&mut s, "slice", json!({"handles": [b.clone(), b], "plane": "xy", "point": [0, 0, 1]}));
    assert_eq!(r["handles"].as_array().unwrap().len(), 2);
    assert_eq!(solids(&s).len(), 2);
    near(solids(&s).iter().map(|p| p.mass_props().volume).sum(), 8.0, 1e-9);
}

#[test]
fn interfere_finds_overlapping_solids() {
    let mut s = Session::new();
    let a = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "length": 10})));
    let b = handle(&run(&mut s, "box", json!({"corner": [8, 0, 0], "length": 10})));
    // Touching face to face is not interference.
    let c = handle(&run(&mut s, "box", json!({"corner": [-10, 0, 0], "length": 10})));
    let r = run(&mut s, "interfere", json!({"first": [a, b, c], "create": true}));
    assert_eq!(r["count"], json!(1));
    near(r["pairs"][0]["volume"].as_f64().unwrap(), 200.0, 1e-9);
    let made = r["pairs"][0]["handle"].as_str().unwrap();
    near(solid(&s, made).mass_props().volume, 200.0, 1e-9);
}

#[test]
fn box_from_the_command_line() {
    let mut s = Session::new();
    s.cmdline("box").unwrap();
    s.cmdline("0,0").unwrap();
    s.cmdline("10,20").unwrap();
    s.cmdline("5").unwrap();
    assert!(s.running.is_none());
    let all = solids(&s);
    assert_eq!(all.len(), 1);
    near(all[0].mass_props().volume, 1000.0, 1e-9);
    s.cmdline("cylinder").unwrap();
    s.cmdline("50,50").unwrap();
    s.cmdline("4").unwrap();
    s.cmdline("10").unwrap();
    assert_eq!(solids(&s).len(), 2);
}

#[test]
fn hostile_solid_params_never_panic() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "length": 10})));
    let ids = [
        "box",
        "wedge",
        "cylinder",
        "cone",
        "sphere",
        "torus",
        "extrude",
        "revolve",
        "3dmove",
        "3drotate",
        "mirror3d",
        "slice",
        "interfere",
        "3dprint",
        "stlout",
        "union",
        "subtract",
        "intersect",
        "massprop",
    ];
    for id in ids {
        for p in [
            json!({"handles": [b], "corner": [1e308, -1e308, 1e308], "opposite": [-1e308, 1e308, -1e308]}),
            json!({"handles": [b], "center": [0, 0, 0], "radius": 1e300, "height": 1e300, "tubeRadius": 1e-300}),
            json!({"handles": [b], "plane": "q", "normal": [0, 0, 0], "axis": [0, 0, 0], "angle": 1e308}),
            json!({"handles": [b], "displacement": [f64::MAX, 0, 0], "segments": 100000000}),
            json!({"from": [b], "remove": [b], "first": ["zz"], "path": ""}),
            json!({"handles": [b], "axis": [[0, 0], [0, 0]], "height": 0, "direction": [1, 0, 0]}),
        ] {
            let _ = s.execute(id, &p);
        }
    }
}

#[test]
fn a_move_out_of_range_leaves_the_solid_where_it_is() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "length": 10})));
    let r = run(&mut s, "3dmove", json!({"handles": [b], "displacement": [1e300, 0, 0]}));
    assert_eq!((r["moved"].as_u64(), r["skipped"].as_u64()), (Some(0), Some(1)));
    near(solid(&s, &b).mass_props().volume, 1000.0, 1e-9);
    s.execute("move", &json!({"handles": [b], "from": [0, 0], "to": [1e300, 0]})).ok();
    near(solid(&s, &b).bounds().min.x, 0.0, 0.0);
}

#[test]
fn a_mixed_boolean_whose_regions_fail_changes_nothing() {
    let mut s = Session::new();
    let a = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "length": 10})));
    let b = handle(&run(&mut s, "box", json!({"corner": [5, 0, 0], "length": 10})));
    // Two regions whose boundaries cross: combining them is not available yet.
    let mut regions = Vec::new();
    for (p1, p2) in [([100, 0], [110, 10]), ([105, 5], [115, 15])] {
        let r = handle(&run(&mut s, "rectang", json!({"p1": p1, "p2": p2})));
        run(&mut s, "region", json!({"handles": [r]}));
        regions.push(s.doc().unwrap().model.iter().last().unwrap().handle.hex());
    }
    let all: Vec<String> = [a, b].into_iter().chain(regions).collect();
    assert!(s.execute("union", &json!({"handles": all})).is_err());
    assert_eq!(solids(&s).len(), 2, "the solids are not united either");
}

#[test]
fn a_slice_that_fails_on_one_solid_changes_none() {
    let mut s = Session::new();
    let near_box = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [2, 2, 2]})));
    // So far out that the half-space reaching it is past the largest coordinate a solid may have.
    let far = handle(&run(&mut s, "box", json!({"corner": [9e11, 0, 0], "opposite": [9e11 + 1.0, 1, 1]})));
    assert!(s.execute("slice", &json!({"handles": [near_box, far], "plane": "xy", "point": [0, 0, 1]})).is_err());
    assert_eq!(solids(&s).len(), 2);
    near(solid(&s, &near_box).mass_props().volume, 8.0, 1e-9);
}
