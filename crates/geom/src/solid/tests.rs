use super::primitives::{circle_segments, cuboid, extrude, frustum, revolve, sphere, torus, wedge};
use super::*;
use crate::{PI, Vec2};

fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z)
}

fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Vec2> {
    vec![Vec2::new(x0, y0), Vec2::new(x1, y0), Vec2::new(x1, y1), Vec2::new(x0, y1)]
}

#[track_caller]
fn assert_closed(m: &Mesh) {
    let c = m.check();
    assert!(c.is_closed(), "{c:?} ({} triangles)", m.triangles.len());
}

#[track_caller]
fn assert_near(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} vs {b} (tolerance {tol})");
}

fn bool_op(a: &Mesh, b: &Mesh, op: BoolOp) -> Mesh {
    boolean(a, b, op).unwrap()
}

#[test]
fn cuboid_is_closed_with_exact_volume_area_and_centroid() {
    let m = cuboid(v(1.0, 2.0, 3.0), v(4.0, 6.0, 8.0)).unwrap();
    assert_closed(&m);
    assert_eq!(m.triangles.len(), 12);
    let p = m.mass_props();
    assert_near(p.volume, 60.0, 1e-9);
    assert_near(p.area, 2.0 * (12.0 + 15.0 + 20.0), 1e-9);
    assert!(p.centroid.dist(v(2.5, 4.0, 5.5)) < 1e-9);
    // Inertia of a 3 x 4 x 5 box about its centroid: m(b² + c²)/12 …
    assert_near(p.centroid_inertia[0][0], 60.0 * (16.0 + 25.0) / 12.0, 1e-6);
    assert_near(p.centroid_inertia[2][2], 60.0 * (9.0 + 16.0) / 12.0, 1e-6);
    assert_near(p.centroid_inertia[0][1], 0.0, 1e-6);
}

#[test]
fn degenerate_primitives_are_errors() {
    assert!(cuboid(v(0.0, 0.0, 0.0), v(1.0, 1.0, 0.0)).is_err());
    assert!(cuboid(v(0.0, 0.0, 0.0), v(f64::NAN, 1.0, 1.0)).is_err());
    assert!(frustum(Vec3::ZERO, 0.0, 0.0, 1.0, 32).is_err());
    assert!(frustum(Vec3::ZERO, 1.0, 1.0, 0.0, 32).is_err());
    assert!(sphere(Vec3::ZERO, -1.0, 32).is_err());
    assert!(torus(Vec3::ZERO, 1.0, 2.0, 32).is_err());
    assert!(extrude(&[square(0.0, 0.0, 1.0, 1.0)], 0.0, v(1.0, 0.0, 0.0)).is_err());
    assert!(extrude(&[vec![Vec2::ZERO, Vec2::new(1.0, 0.0), Vec2::new(2.0, 0.0)]], 0.0, v(0.0, 0.0, 1.0)).is_err());
}

#[test]
fn curved_primitives_are_closed_and_near_their_true_volume() {
    let n = 96;
    let k = n as f64;
    // An inscribed n-gon has area n/2 r² sin(2π/n).
    let polygon = |r: f64| k / 2.0 * r * r * (2.0 * PI / k).sin();
    let cyl = frustum(v(1.0, 1.0, 1.0), 2.0, 2.0, 5.0, n).unwrap();
    assert_closed(&cyl);
    assert_near(cyl.volume(), polygon(2.0) * 5.0, 1e-9);
    let down = frustum(Vec3::ZERO, 2.0, 2.0, -5.0, n).unwrap();
    assert_closed(&down);
    assert_near(down.volume(), polygon(2.0) * 5.0, 1e-9);
    assert_near(down.bounds().min.z, -5.0, 1e-12);
    let cone = frustum(Vec3::ZERO, 3.0, 0.0, 4.0, n).unwrap();
    assert_closed(&cone);
    assert_near(cone.volume(), polygon(3.0) * 4.0 / 3.0, 1e-9);
    let inverted = frustum(Vec3::ZERO, 0.0, 3.0, 4.0, n).unwrap();
    assert_closed(&inverted);
    let s = sphere(Vec3::ZERO, 2.0, n).unwrap();
    assert_closed(&s);
    assert!((s.volume() / (4.0 / 3.0 * PI * 8.0) - 1.0).abs() < 0.01);
    let t = torus(v(5.0, 0.0, 0.0), 3.0, 1.0, n).unwrap();
    assert_closed(&t);
    assert!((t.volume() / (2.0 * PI * PI * 3.0) - 1.0).abs() < 0.02);
    assert!(circle_segments(0.5) >= super::primitives::MIN_SEGMENTS);
    assert!(circle_segments(1000.0) <= super::primitives::MAX_SEGMENTS);
}

#[test]
fn wedge_is_half_its_box_whichever_corner_comes_first() {
    for (a, b) in [(v(0.0, 0.0, 0.0), v(4.0, 2.0, 3.0)), (v(4.0, 2.0, 0.0), v(0.0, 0.0, 3.0)), (v(0.0, 2.0, 3.0), v(4.0, 0.0, 0.0))] {
        let m = wedge(a, b).unwrap();
        assert_closed(&m);
        assert_near(m.volume(), 12.0, 1e-9);
    }
}

#[test]
fn extrusion_with_holes_has_the_profile_area_times_height() {
    let outer = square(0.0, 0.0, 10.0, 6.0);
    // Holes come in either winding; nesting decides they are holes.
    let hole1 = square(1.0, 1.0, 3.0, 3.0);
    let mut hole2 = square(5.0, 1.0, 9.0, 5.0);
    hole2.reverse();
    let island = square(6.0, 2.0, 8.0, 4.0);
    let m = extrude(&[outer, hole1, hole2, island], 2.0, v(0.0, 0.0, 3.0)).unwrap();
    assert_closed(&m);
    assert_near(m.volume(), (60.0 - 4.0 - 16.0 + 4.0) * 3.0, 1e-9);
    let b = m.bounds();
    assert_near(b.min.z, 2.0, 0.0);
    assert_near(b.max.z, 5.0, 0.0);
    // Downward and slanted extrusions keep the volume (base area × height).
    let slanted = extrude(&[square(0.0, 0.0, 2.0, 2.0)], 0.0, v(1.0, 2.0, -3.0)).unwrap();
    assert_closed(&slanted);
    assert_near(slanted.volume(), 12.0, 1e-9);
}

#[test]
fn concave_profile_extrudes_closed() {
    // An L with a notch and a rounded corner approximated by many points.
    let mut l = vec![Vec2::new(0.0, 0.0), Vec2::new(8.0, 0.0), Vec2::new(8.0, 2.0), Vec2::new(2.0, 2.0)];
    for i in 0..=16 {
        let a = -PI / 2.0 + PI / 2.0 * f64::from(i) / 16.0;
        l.push(Vec2::new(2.0 + 0.5 * a.cos() - 0.5, 2.5 + 0.5 * a.sin() + 0.5));
    }
    l.push(Vec2::new(2.0, 8.0));
    l.push(Vec2::new(0.0, 8.0));
    let m = extrude(&[l.clone()], 0.0, v(0.0, 0.0, 1.0)).unwrap();
    assert_closed(&m);
    assert_near(m.volume(), crate::shoelace(&l).abs(), 1e-9);
}

#[test]
fn revolving_a_rectangle_makes_a_tube() {
    let rect = square(2.0, 0.0, 3.0, 4.0);
    let full = revolve(std::slice::from_ref(&rect), Vec2::ZERO, Vec2::new(0.0, 1.0), 2.0 * PI, 128).unwrap();
    assert_closed(&full);
    let n = 128.0;
    let ring = |r: f64| n / 2.0 * r * r * (2.0 * PI / n).sin();
    assert_near(full.volume(), (ring(3.0) - ring(2.0)) * 4.0, 1e-6);
    let half = revolve(&[rect], Vec2::ZERO, Vec2::new(0.0, 1.0), PI, 128).unwrap();
    assert_closed(&half);
    assert_near(half.volume(), full.volume() / 2.0, 1e-6);
    // A profile touching the axis closes into a solid cylinder.
    let solid = revolve(&[square(0.0, 0.0, 1.0, 2.0)], Vec2::ZERO, Vec2::new(0.0, 1.0), 2.0 * PI, 64).unwrap();
    assert_closed(&solid);
    assert!(revolve(&[square(-1.0, 0.0, 1.0, 2.0)], Vec2::ZERO, Vec2::new(0.0, 1.0), PI, 64).is_err());
}

#[test]
fn transforms_keep_meshes_outward() {
    let m = cuboid(Vec3::ZERO, v(1.0, 2.0, 3.0)).unwrap();
    let mirrored = m.transformed(&Mat4::mirror_plane(Vec3::ZERO, v(1.0, 0.0, 0.0)));
    assert_closed(&mirrored);
    assert_near(mirrored.volume(), 6.0, 1e-9);
    assert_near(mirrored.bounds().min.x, -1.0, 1e-12);
    let rotated = m.transformed(&Mat4::rotate_about_axis(v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0), PI / 2.0));
    let b = rotated.bounds();
    assert!(b.min.dist(v(-1.0, -1.0, 0.0)) < 1e-12 && b.max.dist(v(1.0, 0.0, 3.0)) < 1e-12, "{b:?}");
    let scaled = m.transformed(&Mat4::scale_about(Vec3::ZERO, 2.0));
    assert_near(scaled.volume(), 48.0, 1e-9);
}

#[test]
fn wireframe_shows_box_edges_and_a_sphere_outline() {
    let b = cuboid(Vec3::ZERO, v(1.0, 1.0, 1.0)).unwrap();
    assert_eq!(b.wire_edges(FEATURE_ANGLE, Vec3::Z).len(), 12);
    let c = frustum(Vec3::ZERO, 1.0, 1.0, 1.0, 48).unwrap();
    // Two circles; the side seams are smooth and seen edge-on from above they are no outline.
    assert_eq!(c.wire_edges(FEATURE_ANGLE, Vec3::Z).len(), 96);
    let s = sphere(Vec3::ZERO, 1.0, 48).unwrap();
    // Seen from above, a sphere's outline is its equator.
    assert_eq!(s.wire_edges(FEATURE_ANGLE, Vec3::Z).len(), 48);
}

#[test]
fn hostile_meshes_are_sanitized_not_trusted() {
    let m = Mesh { vertices: vec![Vec3::ZERO, v(1.0, 0.0, 0.0), v(f64::NAN, 0.0, 0.0)], triangles: vec![[0, 1, 2], [0, 1, 9], [0, 0, 1]] };
    assert_eq!(m.check().bad_triangles, 2);
    assert!(m.clone().sanitized().triangles.is_empty());
    assert_near(m.volume(), 0.0, 0.0);
    let b = cuboid(Vec3::ZERO, v(1.0, 1.0, 1.0)).unwrap();
    // Booleans with nonsense never panic.
    for op in [BoolOp::Union, BoolOp::Difference, BoolOp::Intersection] {
        let _ = boolean(&m, &b, op);
        let _ = boolean(&b, &m, op);
        let _ = boolean(&Mesh::default(), &b, op);
    }
}

// ---------- booleans ----------

#[test]
fn pocket_through_the_top_face() {
    let a = cuboid(Vec3::ZERO, v(10.0, 10.0, 10.0)).unwrap();
    let b = cuboid(v(2.0, 2.0, 5.0), v(8.0, 8.0, 15.0)).unwrap();
    let d = bool_op(&a, &b, BoolOp::Difference);
    assert_closed(&d);
    assert_near(d.volume(), 1000.0 - 36.0 * 5.0, 1e-6);
    let u = bool_op(&a, &b, BoolOp::Union);
    assert_closed(&u);
    assert_near(u.volume(), 1000.0 + 36.0 * 5.0, 1e-6);
    let i = bool_op(&a, &b, BoolOp::Intersection);
    assert_closed(&i);
    assert_near(i.volume(), 36.0 * 5.0, 1e-6);
}

#[test]
fn flush_pocket_shares_the_top_plane() {
    let a = cuboid(Vec3::ZERO, v(10.0, 10.0, 10.0)).unwrap();
    let b = cuboid(v(2.0, 2.0, 5.0), v(8.0, 8.0, 10.0)).unwrap();
    let d = bool_op(&a, &b, BoolOp::Difference);
    assert_closed(&d);
    assert_near(d.volume(), 1000.0 - 180.0, 1e-6);
    // A slot through a whole side, sharing two faces.
    let slot = cuboid(v(-1.0, 4.0, 6.0), v(11.0, 6.0, 10.0)).unwrap();
    let s = bool_op(&a, &slot, BoolOp::Difference);
    assert_closed(&s);
    assert_near(s.volume(), 1000.0 - 10.0 * 2.0 * 4.0, 1e-6);
}

#[test]
fn boxes_touching_face_to_face_unite_into_one() {
    let a = cuboid(Vec3::ZERO, v(10.0, 10.0, 10.0)).unwrap();
    let b = cuboid(v(10.0, 2.0, 2.0), v(14.0, 8.0, 8.0)).unwrap();
    let u = bool_op(&a, &b, BoolOp::Union);
    assert_closed(&u);
    assert_near(u.volume(), 1000.0 + 4.0 * 36.0, 1e-6);
    let i = bool_op(&a, &b, BoolOp::Intersection);
    assert_near(i.volume(), 0.0, 1e-6);
    let d = bool_op(&a, &b, BoolOp::Difference);
    assert_closed(&d);
    assert_near(d.volume(), 1000.0, 1e-6);
}

#[test]
fn identical_and_disjoint_solids() {
    let a = cuboid(Vec3::ZERO, v(2.0, 3.0, 4.0)).unwrap();
    let u = bool_op(&a, &a, BoolOp::Union);
    assert_closed(&u);
    assert_near(u.volume(), 24.0, 1e-9);
    assert!(bool_op(&a, &a, BoolOp::Difference).is_empty());
    assert_near(bool_op(&a, &a, BoolOp::Intersection).volume(), 24.0, 1e-9);
    let far = cuboid(v(10.0, 0.0, 0.0), v(11.0, 1.0, 1.0)).unwrap();
    assert_near(bool_op(&a, &far, BoolOp::Union).volume(), 25.0, 1e-9);
    assert!(bool_op(&a, &far, BoolOp::Intersection).is_empty());
    assert_eq!(bool_op(&a, &far, BoolOp::Difference), a);
}

#[test]
fn hole_through_a_plate_and_a_pin_on_it() {
    let n = 48;
    let k = n as f64;
    let disc = |r: f64| k / 2.0 * r * r * (2.0 * PI / k).sin();
    let plate = cuboid(Vec3::ZERO, v(20.0, 20.0, 3.0)).unwrap();
    let drill = frustum(v(10.0, 10.0, -1.0), 2.5, 2.5, 5.0, n).unwrap();
    let holed = bool_op(&plate, &drill, BoolOp::Difference);
    assert_closed(&holed);
    assert_near(holed.volume(), 1200.0 - disc(2.5) * 3.0, 1e-6);
    // A pin standing on the plate (touching it) and one sunk into it.
    let pin = frustum(v(4.0, 4.0, 3.0), 1.0, 1.0, 4.0, n).unwrap();
    let sunk = frustum(v(16.0, 16.0, 1.0), 1.0, 1.0, 6.0, n).unwrap();
    let mut part = bool_op(&holed, &pin, BoolOp::Union);
    part = bool_op(&part, &sunk, BoolOp::Union);
    assert_closed(&part);
    assert_near(part.volume(), 1200.0 - disc(2.5) * 3.0 + disc(1.0) * 4.0 + disc(1.0) * 4.0, 1e-6);
    // Cuts stay local: the plate's far faces keep their two triangles each.
    assert!(part.triangles.len() < 800, "{} triangles", part.triangles.len());
}

#[test]
fn sphere_and_cylinder_booleans_add_up() {
    let s = sphere(v(0.3, 0.1, 0.2), 2.0, 48).unwrap();
    let c = frustum(v(0.0, 0.0, -3.0), 1.0, 1.0, 6.0, 40).unwrap();
    let (vs, vc) = (s.volume(), c.volume());
    let u = bool_op(&s, &c, BoolOp::Union);
    let i = bool_op(&s, &c, BoolOp::Intersection);
    let d = bool_op(&s, &c, BoolOp::Difference);
    for m in [&u, &i, &d] {
        assert_closed(m);
    }
    assert_near(u.volume() + i.volume(), vs + vc, 1e-6);
    assert_near(d.volume() + i.volume(), vs, 1e-6);
}

/// Tiny deterministic generator for the property tests.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    /// A coordinate on a coarse grid, so faces often coincide.
    fn grid(&mut self) -> f64 {
        (self.next() * 6.0).floor()
    }
}

#[test]
fn axis_aligned_boxes_on_a_grid_match_exact_volumes() {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    for case in 0..300 {
        let mut corner = || v(rng.grid(), rng.grid(), rng.grid());
        let (p, q, r, s) = (corner(), corner(), corner(), corner());
        let (Ok(a), Ok(b)) = (cuboid(p, q), cuboid(r, s)) else { continue };
        let (ba, bb) = (a.bounds(), b.bounds());
        let overlap = |lo1: f64, hi1: f64, lo2: f64, hi2: f64| (hi1.min(hi2) - lo1.max(lo2)).max(0.0);
        let common = overlap(ba.min.x, ba.max.x, bb.min.x, bb.max.x)
            * overlap(ba.min.y, ba.max.y, bb.min.y, bb.max.y)
            * overlap(ba.min.z, ba.max.z, bb.min.z, bb.max.z);
        let (va, vb) = (a.volume(), b.volume());
        for (op, want) in [(BoolOp::Union, va + vb - common), (BoolOp::Intersection, common), (BoolOp::Difference, va - common)] {
            let m = bool_op(&a, &b, op);
            assert!((m.volume() - want).abs() < 1e-6, "case {case} {op:?}: {} vs {want} ({p:?} {q:?} / {r:?} {s:?})", m.volume());
            // Shapes that touch only along an edge can't be closed; everything else must be.
            let c = m.check();
            assert!(c.open_edges == 0 && c.flipped_edges == 0 && c.bad_triangles == 0, "case {case} {op:?}: {c:?}");
        }
    }
}

#[test]
fn rotated_solids_satisfy_inclusion_exclusion() {
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    for case in 0..40 {
        let rot = |m: Mesh, rng: &mut Rng| {
            let axis = v(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5);
            m.transformed(&Mat4::rotate_about_axis(v(rng.next(), rng.next(), rng.next()), axis, rng.next() * 6.0))
        };
        let a = rot(cuboid(Vec3::ZERO, v(2.0 + rng.next(), 2.0, 1.5)).unwrap(), &mut rng);
        let b = match case % 3 {
            0 => rot(cuboid(v(0.5, 0.5, 0.5), v(2.5, 1.8, 2.7)).unwrap(), &mut rng),
            1 => rot(frustum(v(1.0, 1.0, -1.0), 0.6, 0.6, 3.0, 32).unwrap(), &mut rng),
            _ => rot(sphere(v(1.2, 0.9, 0.7), 1.0, 32).unwrap(), &mut rng),
        };
        let (va, vb) = (a.volume(), b.volume());
        let u = bool_op(&a, &b, BoolOp::Union);
        let i = bool_op(&a, &b, BoolOp::Intersection);
        let d = bool_op(&a, &b, BoolOp::Difference);
        for (name, m) in [("union", &u), ("intersection", &i), ("difference", &d)] {
            assert!(m.check().is_closed(), "case {case} {name}: {:?}", m.check());
        }
        assert!((u.volume() + i.volume() - va - vb).abs() < 1e-6, "case {case}");
        assert!((d.volume() + i.volume() - va).abs() < 1e-6, "case {case}");
    }
}

#[test]
fn many_sequential_cuts_stay_closed() {
    // A plate with a grid of holes and bosses, the way a printed part is built up.
    let mut part = cuboid(Vec3::ZERO, v(60.0, 86.0, 10.0)).unwrap();
    let mut expected = part.volume();
    for i in 0..4 {
        for j in 0..3 {
            let (x, y) = (8.0 + 14.0 * f64::from(i), 10.0 + 30.0 * f64::from(j));
            let hole = frustum(v(x, y, -1.0), 0.8, 0.8, 12.0, 24).unwrap();
            let boss = frustum(v(x + 4.0, y, 10.0), 2.5, 2.5, 3.0, 24).unwrap();
            let cut = bool_op(&part, &hole, BoolOp::Difference);
            expected -= hole.volume() * 10.0 / 12.0;
            part = bool_op(&cut, &boss, BoolOp::Union);
            expected += boss.volume();
            assert_closed(&part);
        }
    }
    assert_near(part.volume(), expected, 1e-6);
    // About what the holes and bosses need on their own (no slivers left over from the cuts).
    assert!(part.triangles.len() < 3000, "{} triangles", part.triangles.len());
}
