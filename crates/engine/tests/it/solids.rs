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
    // The line is raised too; a spline, which has no height of its own, stays.
    match &s.doc().unwrap().entity(Handle::parse_hex(&line).unwrap()).unwrap().kind {
        EntityKind::Line(l) => assert_eq!((l.a.z, l.b.z, l.b.x), (3.0, 3.0, 6.0)),
        k => panic!("{}", k.type_name()),
    }
    let sp = handle(&run(&mut s, "spline", json!({"fit": [[0, 0], [5, 5], [10, 0]]})));
    let r = run(&mut s, "3dmove", json!({"handles": [sp], "displacement": [0, 0, 1]}));
    assert_eq!((r["moved"].as_u64(), r["skipped"].as_u64()), (Some(0), Some(1)));
    run(&mut s, "erase", json!({"handles": [sp]}));
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

#[test]
fn printcheck_finds_what_needs_support() {
    let mut s = Session::new();
    let stem = handle(&run(&mut s, "box", json!({"corner": [-1, -1, 0], "opposite": [1, 1, 10]})));
    let bar = handle(&run(&mut s, "box", json!({"corner": [-10, -1, 10], "opposite": [10, 1, 12]})));
    run(&mut s, "union", json!({"handles": [stem, bar]}));
    let r = run(&mut s, "printcheck", json!({"handles": [stem], "bed": [256, 256, 256]}));
    assert_eq!(r["ok"], json!(false));
    assert_eq!(r["solids"][0]["floating"].as_array().unwrap().len(), 2);
    assert!(r["message"].as_str().unwrap().contains("mid air"), "{r}");
    let plate = handle(&run(&mut s, "box", json!({"corner": [50, 0, 0], "opposite": [70, 20, 3]})));
    let r = run(&mut s, "printcheck", json!({"handles": [plate], "bed": [256, 256, 256]}));
    assert_eq!(r["ok"], json!(true), "{r}");
    near(r["solids"][0]["bedContact"].as_f64().unwrap(), 400.0, 1e-9);
    let r = run(&mut s, "printcheck", json!({"handles": [plate], "bed": [10, 10, 10]}));
    assert!(r["message"].as_str().unwrap().contains("larger than the build volume"), "{r}");
    assert!(s.execute("printcheck", &json!({"handles": [plate], "limit": 120})).is_err());
}

#[test]
fn text_extrudes_into_lettering_in_an_outline_font() {
    let mut s = Session::new();
    // The built-in stroke font draws lines, not outlines: nothing to extrude.
    let t = handle(&run(&mut s, "text", json!({"at": [0, 0], "text": "AB", "height": 5})));
    let e = s.execute("extrude", &json!({"handles": [t], "height": 1})).unwrap_err().to_string();
    assert!(e.contains("single-stroke"), "{e}");
    // With a TrueType font (when this machine has Arial) the letters become one closed solid,
    // the counters of O and B holes in it.
    run(&mut s, "style", json!({"name": "Label", "font": "Arial", "current": true}));
    let t = handle(&run(&mut s, "text", json!({"at": [0, 0], "text": "OB", "height": 10})));
    match s.execute("extrude", &json!({"handles": [t], "height": 2})) {
        Ok(r) => {
            let so = solid(&s, r["handles"][0].as_str().unwrap());
            assert!(so.mesh().check().is_closed());
            let b = so.bounds();
            near(b.max.z - b.min.z, 2.0, 1e-9);
            // Well under the box around the letters: the counters are holes.
            assert!(so.mass_props().volume < (b.max.x - b.min.x) * (b.max.y - b.min.y) * 2.0 * 0.8);
        }
        Err(e) => assert!(e.to_string().contains("single-stroke"), "{e}"),
    }
}

#[test]
fn filletedge_and_chamferedge_round_and_bevel_the_picked_edges() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [10, 6, 4]})));
    // The four bottom edges bevelled 0.5: four prisms of 0.125 per unit length, less the four
    // corners where neighbours overlap (0.5³/3 each).
    let r = run(&mut s, "chamferedge", json!({"handles": [b], "edges": "bottom", "distance": 0.5}));
    assert_eq!(r["edges"], 4);
    assert_eq!(r["closed"], true);
    let so = solid(&s, &b);
    assert!(so.mesh().check().is_closed());
    near(so.mass_props().volume, 240.0 - (32.0 * 0.125 - 4.0 * 0.125 / 3.0), 1e-9);
    // One top edge picked by a point on it, rounded with radius 1: (1 − π/4) per unit length,
    // a little more where the arc is faceted.
    let before = so.mass_props().volume;
    let r = run(&mut s, "filletedge", json!({"handles": [b], "edges": [[5, 0, 4]], "radius": 1}));
    assert_eq!(r["edges"], 1);
    let loss = before - solid(&s, &b).mass_props().volume;
    near(loss, (1.0 - PI / 4.0) * 10.0, 0.05 * (1.0 - PI / 4.0) * 10.0);
    // The undo restores the sharp edge.
    s.undo().unwrap();
    near(solid(&s, &b).mass_props().volume, before, 1e-9);
}

#[test]
fn filletedge_fills_an_inside_corner_and_refuses_bad_input() {
    let mut s = Session::new();
    let floor = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [10, 10, 2]})));
    let wall = handle(&run(&mut s, "box", json!({"corner": [0, 8, 0], "opposite": [10, 10, 8]})));
    run(&mut s, "union", json!({"handles": [floor, wall]}));
    let before = solid(&s, &floor).mass_props().volume;
    let r = run(&mut s, "filletedge", json!({"handles": [floor], "edges": "concave", "radius": 2}));
    assert_eq!(r["edges"], 1);
    let gain = solid(&s, &floor).mass_props().volume - before;
    near(gain, (4.0 - PI) * 10.0, 0.05 * (4.0 - PI) * 10.0);
    assert!(solid(&s, &floor).mesh().check().is_closed());
    let line = handle(&run(&mut s, "line", json!({"points": [[0, 0], [1, 1]]})));
    for (p, why) in [
        (json!({"handles": [line]}), "select 3D solids"),
        (json!({"handles": [floor], "edges": "sideways"}), "must be one of"),
        (json!({"handles": [floor], "edges": [[100, 100, 100]]}), "no edge"),
        (json!({"handles": [floor], "radius": -1}), "positive"),
        (json!({"handles": [floor], "edges": 7}), "keyword or a list"),
    ] {
        let e = s.execute("filletedge", &p).unwrap_err().to_string();
        assert!(e.contains(why), "{p}: {e}");
    }
}

#[test]
fn filletedge_from_the_command_line_picks_the_highest_edge_in_plan() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [10, 6, 4]})));
    s.set_selection(Vec::new());
    s.cmdline("filletedge").unwrap();
    s.cmdline("r").unwrap();
    s.cmdline("0.5").unwrap();
    // In plan the top and bottom front edges lie over each other: the top one is picked.
    s.cmdline("5,0").unwrap();
    s.cmdline("").unwrap();
    let so = solid(&s, &b);
    assert!(so.mesh().check().is_closed());
    near(240.0 - so.mass_props().volume, (1.0 - PI / 4.0) * 0.25 * 10.0, 0.05 * (1.0 - PI / 4.0) * 0.25 * 10.0);
    assert!(so.bounds().max.z == 4.0 && so.mesh().vertices.iter().any(|v| v.z < 4.0 && v.z > 3.5 && v.y < 0.5 && v.y > 0.0));
}

#[test]
fn sweep_carries_a_profile_along_a_path() {
    let mut s = Session::new();
    // A circle of radius 1 along an L of polyline (10 then 8): a bent rod, its volume the
    // section's area times the centre line's length (the mitre adds and takes the same).
    let c = handle(&run(&mut s, "circle", json!({"center": [50, 50], "radius": 1})));
    let path = handle(&run(&mut s, "pline", json!({"points": [[0, 0], [10, 0], [10, 8]]})));
    let r = run(&mut s, "sweep", json!({"handles": [c], "path": path}));
    let rod = solid(&s, &handle(&r));
    assert!(rod.mesh().check().is_closed());
    // The first section is the cap across the path at x = 0.
    let n = rod.mesh().vertices.iter().filter(|v| v.x.abs() < 1e-9).count() as f64;
    let area = ngon(1.0, n);
    near(rod.mass_props().volume, area * 18.0, 1e-6 * area * 18.0);
    // The profile and path are erased; the rod starts at the path's start, across it.
    assert_eq!(s.doc().unwrap().model.len(), 1);
    let b = rod.bounds();
    near(b.min.x, 0.0, 1e-9);
    near(b.max.y, 8.0, 1e-9);
    near(b.max.z, 1.0, 1e-3);
    // A coil: a circle along a HELIX, kept.
    let c = handle(&run(&mut s, "circle", json!({"center": [0, 0], "radius": 0.5})));
    let h = handle(&run(&mut s, "helix", json!({"center": [0, 0], "baseRadius": 5, "height": 10, "turns": 4})));
    let r = run(&mut s, "sweep", json!({"handles": [c], "path": h, "keep": true}));
    let coil = solid(&s, &handle(&r));
    assert!(coil.mesh().check().is_closed());
    let b = coil.bounds();
    near(b.max.z - b.min.z, 11.0, 0.05);
    assert!(s.doc().unwrap().entity(Handle::parse_hex(&h).unwrap()).is_some());
    // Inline points, twisted: a square bar twisted a quarter turn.
    let sq = handle(&run(&mut s, "rectang", json!({"p1": [-1, -1], "p2": [1, 1]})));
    let r = run(&mut s, "sweep", json!({"handles": [sq], "path": [[0, 0, 0], [0, 0, 10]], "twist": 90}));
    let bar = solid(&s, &handle(&r));
    assert!(bar.mesh().check().is_closed());
    near(bar.mass_props().volume, 40.0, 0.01 * 40.0);
    let t = handle(&run(&mut s, "text", json!({"at": [0, 0], "text": "x", "height": 1})));
    let sq = handle(&run(&mut s, "rectang", json!({"p1": [-1, -1], "p2": [1, 1]})));
    let e = s.execute("sweep", &json!({"handles": [sq], "path": t})).unwrap_err().to_string();
    assert!(e.contains("not a path"), "{e}");
    let e = s.execute("sweep", &json!({"handles": [sq]})).unwrap_err().to_string();
    assert!(e.contains("`path`"), "{e}");
}

#[test]
fn loft_joins_cross_sections_and_3dmove_raises_them() {
    let mut s = Session::new();
    let sq = handle(&run(&mut s, "rectang", json!({"p1": [-2, -2], "p2": [2, 2]})));
    let c = handle(&run(&mut s, "circle", json!({"center": [0, 0], "radius": 1.5})));
    // 3DMOVE lifts the circle to z = 6.
    run(&mut s, "3dmove", json!({"handles": [c], "displacement": [0, 0, 6]}));
    match &s.doc().unwrap().entity(Handle::parse_hex(&c).unwrap()).unwrap().kind {
        EntityKind::Circle(ci) => near(ci.center.z, 6.0, 1e-12),
        k => panic!("{}", k.type_name()),
    }
    let r = run(&mut s, "loft", json!({"handles": [sq, c], "ruled": true}));
    let m = solid(&s, &handle(&r));
    assert!(m.mesh().check().is_closed());
    let v = m.mass_props().volume;
    assert!(v < 16.0 * 6.0 && v > PI * 1.5 * 1.5 * 6.0 * 0.95, "{v}");
    assert_eq!(s.doc().unwrap().model.len(), 1);
    // To a point: a cone, a third of its base times its height.
    let base = handle(&run(&mut s, "circle", json!({"center": [20, 0], "radius": 3})));
    let tip = run(&mut s, "point", json!({"at": [20, 0]}))["handles"][0].as_str().unwrap().to_string();
    run(&mut s, "3dmove", json!({"handles": [tip], "displacement": [0, 0, 9]}));
    let r = run(&mut s, "loft", json!({"handles": [base, tip]}));
    let cone = solid(&s, &handle(&r));
    assert!(cone.mesh().check().is_closed());
    let n = cone.mesh().vertices.iter().filter(|p| p.z.abs() < 1e-9).count() as f64;
    near(cone.mass_props().volume, ngon(3.0, n) * 3.0, 1e-6 * ngon(3.0, n) * 3.0);
    let one = handle(&run(&mut s, "circle", json!({"center": [0, 0], "radius": 1})));
    let e = s.execute("loft", &json!({"handles": [one]})).unwrap_err().to_string();
    assert!(e.contains("at least two"), "{e}");
}

#[test]
fn pyramid_and_polysolid_have_their_volumes() {
    let mut s = Session::new();
    // Four sides about a radius of 5 to the middle of each: a 10 × 10 base, a third of it times
    // the height.
    let r = run(&mut s, "pyramid", json!({"center": [0, 0, 0], "radius": 5, "height": 6}));
    near(r["volume"].as_f64().unwrap(), 100.0 * 6.0 / 3.0, 1e-9);
    let b = solid(&s, &handle(&r)).bounds();
    near(b.max.x - b.min.x, 10.0, 1e-9);
    // Six sides to the corners, cut flat at half the radius: a frustum.
    let r = run(&mut s, "pyramid", json!({"center": [20, 0, 0], "radius": 4, "sides": 6, "inscribed": true, "topRadius": 2, "height": 3}));
    let hex = |r: f64| 3.0 * 3f64.sqrt() / 2.0 * r * r;
    let (a1, a2) = (hex(4.0), hex(2.0));
    near(r["volume"].as_f64().unwrap(), 3.0 / 3.0 * (a1 + a2 + (a1 * a2).sqrt()), 1e-9);
    assert!(solid(&s, &handle(&r)).mesh().check().is_closed());
    // A wall 0.5 wide and 3 high along an L of 10 then 6: mitred, its centre line 16 long.
    let r = run(&mut s, "polysolid", json!({"points": [[0, 0], [10, 0], [10, 6]], "width": 0.5, "height": 3}));
    near(r["volume"].as_f64().unwrap(), 0.5 * 3.0 * 16.0, 1e-9);
    let w = solid(&s, &handle(&r));
    assert!(w.mesh().check().is_closed());
    near(w.bounds().min.y, -0.25, 1e-9);
    // Standing on the left of the path, closed round a square: a frame 1 thick inside 10 × 10.
    let r =
        run(&mut s, "polysolid", json!({"points": [[0, 0], [10, 0], [10, 10], [0, 10]], "closed": true, "width": 1, "height": 2, "justify": "left"}));
    near(r["volume"].as_f64().unwrap(), (100.0 - 64.0) * 2.0, 1e-9);
    let b = solid(&s, &handle(&r)).bounds();
    assert!(b.min.x.abs() < 1e-9 && (b.max.x - 10.0).abs() < 1e-9);
    // From a circle, which is erased.
    let c = handle(&run(&mut s, "circle", json!({"center": [50, 50], "radius": 10})));
    let r = run(&mut s, "polysolid", json!({"object": c, "width": 1, "height": 1}));
    assert!(solid(&s, &handle(&r)).mesh().check().is_closed());
    assert!(s.doc().unwrap().entity(Handle::parse_hex(&c).unwrap()).is_none());
    assert!(s.execute("pyramid", &json!({"center": [0, 0, 0], "radius": 5, "height": 6, "sides": 2})).is_err());
    assert!(s.execute("polysolid", &json!({"points": [[0, 0], [1, 0]], "justify": "up"})).is_err());
}

#[test]
fn array_and_align_in_3d() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [1, 1, 1]})));
    let line = handle(&run(&mut s, "line", json!({"points": [[0, 0], [1, 0]]})));
    // 2 rows × 3 columns × 2 levels: eleven copies of each.
    let r = run(&mut s, "3darray", json!({"handles": [b, line], "rows": 2, "columns": 3, "levels": 2, "spacing": [2, 3, 4]}));
    assert_eq!(r["handles"].as_array().unwrap().len(), 22);
    let top = solids(&s).iter().map(|so| so.bounds().max).fold(cadcraft_engine::geom::Vec3::ZERO, |m, p| m.max(p));
    assert_eq!((top.x, top.y, top.z), (5.0, 4.0, 5.0));
    // Polar: six round the Z axis through (10, 0), turning with it.
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [12, -0.5, 0], "opposite": [14, 0.5, 1]})));
    let r = run(&mut s, "3darray", json!({"handles": [b], "polar": true, "count": 6, "axis": [[10, 0, 0], [10, 0, 1]]}));
    assert_eq!(r["handles"].as_array().unwrap().len(), 5);
    let opposite = solid(&s, r["handles"][2].as_str().unwrap()).bounds();
    near(opposite.min.x, 6.0, 1e-9);
    near(opposite.max.y - opposite.min.y, 1.0, 1e-9);
    // 3DALIGN: X onto Y and Y onto Z, so the box Z ends up along X (right-handed).
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [4, 2, 1]})));
    run(&mut s, "3dalign", json!({"handles": [b], "from": [[0, 0, 0], [4, 0, 0], [0, 2, 0]], "to": [[10, 10, 10], [10, 14, 10], [10, 10, 12]]}));
    let bb = solid(&s, &b).bounds();
    for (got, want) in [(bb.min.x, 10.0), (bb.max.x, 11.0), (bb.min.y, 10.0), (bb.max.y, 14.0), (bb.min.z, 10.0), (bb.max.z, 12.0)] {
        near(got, want, 1e-9);
    }
    near(solid(&s, &b).mass_props().volume, 8.0, 1e-9);
    let e = s
        .execute("3dalign", &json!({"handles": [b], "from": [[0, 0, 0], [1, 0, 0], [2, 0, 0]], "to": [[0, 0, 0], [1, 0, 0], [0, 1, 0]]}))
        .unwrap_err()
        .to_string();
    assert!(e.contains("line up"), "{e}");
}

#[test]
fn solidedit_shells_separates_and_checks() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [10, 8, 6]})));
    // A tray: walls of 1 with the top opened, its hollow 8 × 6 × 5.
    let r = run(&mut s, "solidedit.shell", json!({"handles": [b], "thickness": 1, "remove": "top"}));
    near(r["volumes"][0].as_f64().unwrap(), 480.0 - 240.0, 1e-9);
    assert!(solid(&s, &b).mesh().check().is_closed());
    // Opened through a side picked by a point on it instead: a sleeve along x.
    let c = handle(&run(&mut s, "box", json!({"corner": [20, 0, 0], "opposite": [30, 8, 6]})));
    let r = run(&mut s, "solidedit", json!({"mode": "shell", "handles": [c], "thickness": 1, "remove": [[20, 4, 3], [30, 4, 3]]}));
    near(r["volumes"][0].as_f64().unwrap(), 480.0 - 10.0 * 6.0 * 4.0, 1e-9);
    // Two solids united into one are separated again.
    let d = handle(&run(&mut s, "box", json!({"corner": [50, 0, 0], "opposite": [51, 1, 1]})));
    let e = handle(&run(&mut s, "box", json!({"corner": [60, 0, 0], "opposite": [62, 2, 2]})));
    run(&mut s, "union", json!({"handles": [d, e]}));
    let r = run(&mut s, "solidedit.check", json!({"handles": [d]}));
    assert_eq!(r["solids"][0]["pieces"], 2);
    assert_eq!(r["solids"][0]["closed"], true);
    let r = run(&mut s, "solidedit.separate", json!({"handles": [d]}));
    assert_eq!(r["handles"].as_array().unwrap().len(), 2);
    assert_eq!(solids(&s).len(), 4);
    for (p, why) in [
        (json!({"mode": "explode", "handles": [b]}), "`mode`"),
        (json!({"mode": "shell", "handles": [b]}), "`thickness`"),
        (json!({"mode": "shell", "handles": [b], "thickness": 50}), "too thick"),
        (json!({"mode": "shell", "handles": [b], "thickness": 0.1, "remove": [[500, 0, 0]]}), "no face"),
    ] {
        let err = s.execute("solidedit", &p).unwrap_err().to_string();
        assert!(err.contains(why), "{p}: {err}");
    }
}

#[test]
fn solidedit_from_the_command_line() {
    let mut s = Session::new();
    let b = handle(&run(&mut s, "box", json!({"corner": [0, 0, 0], "opposite": [10, 8, 6]})));
    s.set_selection(Vec::new());
    s.cmdline("solidedit").unwrap();
    s.cmdline("shell").unwrap();
    // A point on its outline in plan picks the box.
    s.cmdline("0,4").unwrap();
    s.cmdline("").unwrap();
    s.cmdline("1").unwrap();
    s.cmdline("top").unwrap();
    near(solid(&s, &b).mass_props().volume, 240.0, 1e-9);
}
