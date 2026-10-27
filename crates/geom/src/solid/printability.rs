//! Will a solid print on an FDM printer standing as it is (Z up, its lowest point on the bed)?
//!
//! - **Overhangs**: sloped faces leaning out more than `limit` degrees from vertical droop. Faces
//!   leaning past `ceiling` degrees are ceilings: they bridge or float rather than droop, which
//!   the floating test covers. Measured per triangle, so cones and fillets count too, and summed
//!   by height band, because where the steep area is tells which feature it is.
//! - **Floating faces**: downward faces above the bed with none of the solid anywhere under them:
//!   they start in mid air. A roof over a through slot counts too: whether it prints depends on
//!   how far it has to bridge. Faces smaller than [`MIN_FLOATING_AREA`] are left out: slivers
//!   where curved faces of different facet counts meet, far below a nozzle's line width.
//! - **Bed contact**: the area of the faces lying on the bed.

use std::collections::HashMap;

use super::Mesh;
use super::bvh::Bvh;
use crate::{Bounds3, Vec3};

/// Overhangs in one band of height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OverhangBand {
    /// Bottom of the band.
    pub z: f64,
    pub area: f64,
    /// Shallowest and steepest overhang in the band, degrees from vertical.
    pub min_angle: f64,
    pub max_angle: f64,
}

/// A face that starts in mid air.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Floating {
    pub z: f64,
    pub area: f64,
    /// Where it is: the centre of its bounding box.
    pub at: Vec3,
}

/// What [`check`] found.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Printability {
    pub bed_contact: f64,
    pub overhang_area: f64,
    /// Steep area by height band, the largest first.
    pub overhang_bands: Vec<OverhangBand>,
    /// Faces that start in mid air, the largest first.
    pub floating: Vec<Floating>,
}

/// Height of the bands overhangs are summed in (drawing units).
pub const BAND: f64 = 5.0;
/// Smallest floating face reported (square drawing units, millimetres in practice): a tenth of a
/// millimetre square, under a quarter of a 0.4 mm line.
pub const MIN_FLOATING_AREA: f64 = 0.01;
/// Most ceiling faces examined for support (each casts a few rays).
const MAX_CEILING_TRIANGLES: usize = 200_000;

/// The printability of `m` standing on its lowest point: overhangs steeper than `limit` degrees
/// from vertical, up to `ceiling` degrees, past which a face counts as a ceiling.
pub fn check(m: &Mesh, limit: f64, ceiling: f64) -> Printability {
    let mut out = Printability::default();
    let tris: Vec<[Vec3; 3]> = m.triangle_points().collect();
    let b = Bounds3::from_points(tris.iter().flatten().copied());
    if b.is_empty() {
        return out;
    }
    let size = b.size();
    let tol = (size.x.max(size.y).max(size.z) * 1e-6).max(1e-12);
    let bed = b.min.z;
    let mut bands: HashMap<i64, OverhangBand> = HashMap::new();
    // Downward faces above the bed, for the floating test.
    let mut ceilings: Vec<[Vec3; 3]> = Vec::new();
    for [a, c1, c2] in &tris {
        let cross = (*c1 - *a).cross(*c2 - *a);
        let area = cross.len() / 2.0;
        if area <= tol * tol {
            continue;
        }
        let n = cross.normalized();
        let (low, high) = (a.z.min(c1.z).min(c2.z), a.z.max(c1.z).max(c2.z));
        if n.z >= 0.0 {
            continue;
        }
        if n.z < -0.999 && high - bed <= tol {
            out.bed_contact += area;
            continue;
        }
        let angle = (-n.z).clamp(0.0, 1.0).asin().to_degrees();
        if angle > limit && angle <= ceiling {
            out.overhang_area += area;
            let z = (a.z + c1.z + c2.z) / 3.0;
            let key = ((z - bed) / BAND).floor() as i64;
            let e = bands.entry(key).or_insert(OverhangBand { z: bed + key as f64 * BAND, area: 0.0, min_angle: 90.0, max_angle: 0.0 });
            e.area += area;
            e.min_angle = e.min_angle.min(angle);
            e.max_angle = e.max_angle.max(angle);
        }
        if n.z < -0.95 && low > bed + tol && ceilings.len() < MAX_CEILING_TRIANGLES {
            ceilings.push([*a, *c1, *c2]);
        }
    }
    out.overhang_bands = bands.into_values().collect();
    out.overhang_bands.sort_by(|p, q| q.area.total_cmp(&p.area).then(p.z.total_cmp(&q.z)));
    let boxes: Vec<Bounds3> = tris.iter().map(|t| Bounds3::from_points(*t)).collect();
    let bvh = Bvh::new(&boxes);
    let below = |p: Vec3| {
        let mut hit = false;
        bvh.along(p, Vec3::new(0.0, 0.0, -1.0), tol, f64::INFINITY, |i| hit = hit || tris.get(i as usize).is_some_and(|t| under(t, p, tol)));
        hit
    };
    for face in faces(&ceilings, tol) {
        let held = face.iter().any(|t| probes(t).iter().any(|p| below(*p)));
        let area: f64 = face.iter().map(|[a, b, c]| (*b - *a).cross(*c - *a).len() / 2.0).sum();
        if !held && area >= MIN_FLOATING_AREA {
            let fb = Bounds3::from_points(face.iter().flatten().copied());
            out.floating.push(Floating { z: fb.center().z, area, at: fb.center() });
        }
    }
    out.floating.sort_by(|p, q| q.area.total_cmp(&p.area).then(p.z.total_cmp(&q.z)));
    out
}

/// Triangles joined into faces where they share an edge's end (within `tol`).
fn faces(tris: &[[Vec3; 3]], tol: f64) -> Vec<Vec<[Vec3; 3]>> {
    let key = |p: Vec3| [(p.x / tol).round() as i64, (p.y / tol).round() as i64, (p.z / tol).round() as i64];
    let mut parent: Vec<usize> = (0..tris.len()).collect();
    let root = |parent: &mut Vec<usize>, mut i: usize| {
        // Path halving; every index stays in range, so `get` never misses.
        while let Some(&p) = parent.get(i).filter(|&&p| p != i) {
            let gp = parent.get(p).copied().unwrap_or(p);
            if let Some(x) = parent.get_mut(i) {
                *x = gp;
            }
            i = gp;
        }
        i
    };
    let mut owner: HashMap<[i64; 3], usize> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        for p in t {
            if let Some(&j) = owner.get(&key(*p)) {
                let (a, b) = (root(&mut parent, i), root(&mut parent, j));
                if let Some(x) = parent.get_mut(a) {
                    *x = b;
                }
            } else {
                owner.insert(key(*p), i);
            }
        }
    }
    let mut groups: Vec<Vec<[Vec3; 3]>> = Vec::new();
    let mut slot: HashMap<usize, usize> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        let r = root(&mut parent, i);
        let g = *slot.entry(r).or_insert_with(|| {
            groups.push(Vec::new());
            groups.len() - 1
        });
        if let Some(v) = groups.get_mut(g) {
            v.push(*t);
        }
    }
    groups
}

/// The centroid and points most of the way from it to each corner.
fn probes([a, b, c]: &[Vec3; 3]) -> [Vec3; 4] {
    let m = (*a + *b + *c) * (1.0 / 3.0);
    [m, m.lerp(*a, 0.9), m.lerp(*b, 0.9), m.lerp(*c, 0.9)]
}

/// Whether triangle `t` lies under `p` (straight down, more than `tol` below it).
fn under([a, b, c]: &[Vec3; 3], p: Vec3, tol: f64) -> bool {
    let d = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    if d.abs() < 1e-300 {
        return false;
    }
    let l1 = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / d;
    let l2 = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / d;
    let l3 = 1.0 - l1 - l2;
    l1 >= 0.0 && l2 >= 0.0 && l3 >= 0.0 && l1 * a.z + l2 * b.z + l3 * c.z < p.z - tol
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::primitives::{cuboid, frustum};
    use crate::solid::{BoolOp, boolean};

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }

    #[test]
    fn a_box_sits_on_its_base_with_no_overhangs() {
        let p = check(&cuboid(Vec3::ZERO, v(10.0, 20.0, 5.0)).unwrap(), 46.0, 85.0);
        assert!((p.bed_contact - 200.0).abs() < 1e-9);
        assert_eq!((p.overhang_area, p.floating.len()), (0.0, 0));
    }

    #[test]
    fn an_upside_down_cone_overhangs_and_a_shelf_floats() {
        // A cone standing on its point: its whole side leans out 63° from vertical.
        let cone = frustum(Vec3::ZERO, 0.5, 10.0, 5.0, 64).unwrap();
        let p = check(&cone, 46.0, 85.0);
        assert!(p.overhang_area > 100.0, "{p:?}");
        let band = p.overhang_bands.first().unwrap();
        assert!(band.min_angle > 60.0 && band.max_angle < 65.0, "{band:?}");
        // A T: the bar's underside starts in mid air on either side of the stem.
        let stem = cuboid(v(-1.0, -1.0, 0.0), v(1.0, 1.0, 10.0)).unwrap();
        let bar = cuboid(v(-10.0, -1.0, 10.0), v(10.0, 1.0, 12.0)).unwrap();
        let t = boolean(&stem, &bar, BoolOp::Union).unwrap();
        let p = check(&t, 46.0, 85.0);
        assert_eq!(p.floating.len(), 2, "{p:?}");
        assert!(p.floating.iter().all(|f| (f.z - 10.0).abs() < 1e-9 && (f.area - 18.0).abs() < 1e-6));
    }

    #[test]
    fn a_roof_over_a_through_slot_floats_and_a_cavity_ceiling_is_held() {
        // An arch: the slot's roof has nothing under it (it can only bridge).
        let block = cuboid(Vec3::ZERO, v(20.0, 10.0, 10.0)).unwrap();
        let slot = cuboid(v(5.0, -1.0, -1.0), v(15.0, 11.0, 6.0)).unwrap();
        let arch = boolean(&block, &slot, BoolOp::Difference).unwrap();
        let p = check(&arch, 46.0, 85.0);
        assert_eq!(p.floating.len(), 1, "{p:?}");
        assert!((p.floating[0].area - 100.0).abs() < 1e-6 && (p.floating[0].z - 6.0).abs() < 1e-9);
        assert!((p.bed_contact - 100.0).abs() < 1e-6);
        // A closed cavity: its ceiling has the cavity's floor and the solid under it.
        let cavity = cuboid(v(2.0, 2.0, 3.0), v(18.0, 8.0, 6.0)).unwrap();
        let hollow = boolean(&block, &cavity, BoolOp::Difference).unwrap();
        assert!(check(&hollow, 46.0, 85.0).floating.is_empty());
    }

    #[test]
    fn a_sliver_too_small_to_print_does_not_float() {
        // A block with a tiny tab sticking out halfway up: its underside (0.05 × 0.05) floats but
        // is far below a printed line; a 1 × 1 tab is reported.
        let block = cuboid(Vec3::ZERO, v(10.0, 10.0, 10.0)).unwrap();
        for (w, floats) in [(0.05, false), (1.0, true)] {
            let tab = cuboid(v(10.0 - 1e-3, 5.0, 5.0), v(10.0 + w, 5.0 + w, 6.0)).unwrap();
            let m = boolean(&block, &tab, BoolOp::Union).unwrap();
            assert_eq!(!check(&m, 46.0, 85.0).floating.is_empty(), floats, "{w}");
        }
    }
}
