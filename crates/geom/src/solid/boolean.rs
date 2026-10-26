//! Boolean operations on closed meshes: union, subtraction and intersection.
//!
//! Each triangle of one solid is cut by the planes of the other solid's triangles that cross it
//! (and, where the two touch face to face, by the edges of the touching triangles), so that every
//! piece lies wholly inside the other solid, outside it, or on its surface. A piece is classified
//! by casting rays from points just in front of and just behind it: both outside, both inside,
//! or one of each (the two surfaces touch, facing the same way or opposite ways). The pieces the
//! operation keeps are welded into one closed mesh by [`super::stitch`]. A triangle whose pieces
//! are all kept stays whole, and the kept pieces of the others are joined back into one outline
//! per plane ([`super::merge`]), so cuts don't spread across faces far from where the solids
//! meet.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use super::bvh::Bvh;
use super::merge::merge;
use super::stitch::stitch;
use super::{MAX_TRIANGLES, Mesh, SolidError};
use crate::{Bounds3, Vec3};

/// Which boolean to compute.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BoolOp {
    /// Everything inside either solid.
    Union,
    /// The first solid minus the second.
    Difference,
    /// Only what is inside both.
    Intersection,
}

/// Most pieces one triangle may be cut into.
const MAX_PIECES: usize = 4096;

/// Ray directions for inside tests, chosen to avoid lining up with axis-aligned edges.
const DIRS: [[f64; 3]; 7] = [
    [0.2846, 0.4419, 0.8505],
    [-0.6511, 0.3105, 0.6927],
    [0.5385, -0.7629, 0.3577],
    [-0.1723, -0.5531, -0.8152],
    [0.8803, 0.2087, -0.4262],
    [-0.3359, 0.9073, -0.2531],
    [0.0421, -0.2264, 0.9731],
];

struct Tri {
    p: [Vec3; 3],
    /// Unit normal and plane offset: `n·x = d` on the triangle's plane.
    n: Vec3,
    d: f64,
    bounds: Bounds3,
}

struct Solid {
    tris: Vec<Tri>,
    bvh: Bvh,
    bounds: Bounds3,
}

impl Solid {
    fn new(m: &Mesh, tol: &Tol) -> Solid {
        let tris: Vec<Tri> = m
            .triangle_points()
            .filter_map(|p| {
                let c = (p[1] - p[0]).cross(p[2] - p[0]);
                // Skip slivers with no area; they can't bound anything.
                (c.len() > tol.plane * tol.plane).then(|| {
                    let n = c.normalized();
                    Tri { p, n, d: n.dot(p[0]), bounds: Bounds3::from_points(p) }
                })
            })
            .collect();
        let boxes: Vec<Bounds3> = tris.iter().map(|t| t.bounds).collect();
        let bounds = boxes.iter().fold(Bounds3::EMPTY, |b, t| b.union(t));
        Solid { bvh: Bvh::new(&boxes), tris, bounds }
    }
    fn tri(&self, i: u32) -> Option<&Tri> {
        self.tris.get(i as usize)
    }
}

/// Distances that decide "on the plane", "the same point" and how far to step off a surface.
struct Tol {
    plane: f64,
    weld: f64,
    step_min: f64,
    step_max: f64,
}

impl Tol {
    fn new(b: &Bounds3) -> Tol {
        let size = b.size();
        let extent = size.x.max(size.y).max(size.z).max(1e-300);
        let magnitude = [b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z].iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let plane = (extent * 1e-9).max(magnitude * 1e-12);
        Tol { plane, weld: plane * 10.0, step_min: plane * 10.0, step_max: (extent * 1e-5).max(plane * 10.0) }
    }
}

/// Where a piece of one solid's surface lies relative to the other solid.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Outside,
    Inside,
    /// On the other solid's surface, both facing the same way.
    Same,
    /// On the other solid's surface, facing each other.
    Opposite,
}

fn keep(side: Side, op: BoolOp, first: bool) -> bool {
    match (op, first) {
        (BoolOp::Union, true) => matches!(side, Side::Outside | Side::Same),
        (BoolOp::Union, false) => side == Side::Outside,
        (BoolOp::Intersection, true) => matches!(side, Side::Inside | Side::Same),
        (BoolOp::Intersection, false) => side == Side::Inside,
        (BoolOp::Difference, true) => matches!(side, Side::Outside | Side::Opposite),
        (BoolOp::Difference, false) => side == Side::Inside,
    }
}

/// `a op b`. Both meshes should be closed and outward-facing; the result is, too, except where
/// the solids touch only along an edge or at a point (which no closed mesh can represent).
pub fn boolean(a: &Mesh, b: &Mesh, op: BoolOp) -> Result<Mesh, SolidError> {
    if a.triangles.len() > MAX_TRIANGLES || b.triangles.len() > MAX_TRIANGLES {
        return Err(SolidError::TooComplex);
    }
    let tol = Tol::new(&a.bounds().union(&b.bounds()));
    let (sa, sb) = (Solid::new(a, &tol), Solid::new(b, &tol));
    let disjoint = !sa.bounds.expand(tol.plane).intersects(&sb.bounds);
    if sa.tris.is_empty() || sb.tris.is_empty() || disjoint {
        let mut out = Mesh::default();
        match op {
            BoolOp::Union => {
                out.append(a)?;
                out.append(b)?;
            }
            BoolOp::Difference => out.append(a)?,
            BoolOp::Intersection => {}
        }
        return Ok(out);
    }
    let mut pieces: Vec<Vec<Vec3>> = Vec::new();
    collect(&sa, &sb, op, true, &tol, &mut pieces)?;
    collect(&sb, &sa, op, false, &tol, &mut pieces)?;
    stitch(&pieces, tol.weld)
}

/// Add the pieces of `me`'s surface that `op` keeps (`first`: `me` is the left operand).
fn collect(me: &Solid, other: &Solid, op: BoolOp, first: bool, tol: &Tol, out: &mut Vec<Vec<Vec3>>) -> Result<(), SolidError> {
    let reverse = op == BoolOp::Difference && !first;
    let mut emit = |mut poly: Vec<Vec3>| -> Result<(), SolidError> {
        if out.len() >= MAX_TRIANGLES {
            return Err(SolidError::TooComplex);
        }
        if reverse {
            poly.reverse();
        }
        out.push(poly);
        Ok(())
    };
    let near = other.bounds.expand(tol.plane);
    let mut found = Vec::new();
    // Kept pieces of cut triangles, by plane: (normal, pieces).
    let mut groups: Vec<(Vec3, Vec<Vec<Vec3>>)> = Vec::new();
    let mut group_of: HashMap<[i64; 4], usize> = HashMap::new();
    for t in &me.tris {
        if !t.bounds.intersects(&near) {
            if keep(Side::Outside, op, first) {
                emit(t.p.to_vec())?;
            }
            continue;
        }
        other.bvh.overlapping(&t.bounds.expand(tol.plane), &mut found);
        let planes = cutting_planes(t, other, &found, tol);
        if planes.is_empty() {
            let c = (t.p[0] + t.p[1] + t.p[2]) * (1.0 / 3.0);
            let side = if inside(other, c, tol) { Side::Inside } else { Side::Outside };
            if keep(side, op, first) {
                emit(t.p.to_vec())?;
            }
            continue;
        }
        let parts = cut(t, &planes, tol)?;
        let kept: Vec<(Vec<Vec3>, bool)> = parts.into_iter().filter_map(|p| classify(&p, t.n, other, tol).map(|s| (p, keep(s, op, first)))).collect();
        if kept.iter().all(|(_, k)| *k) {
            emit(t.p.to_vec())?;
            continue;
        }
        // Gather what is kept of cut triangles by plane, to rebuild each face as a whole.
        let key = [t.n.x, t.n.y, t.n.z].map(|c| (c * 1e6).round() as i64);
        let key = [key[0], key[1], key[2], (t.d / (tol.weld * 100.0)).round() as i64];
        let g = *group_of.entry(key).or_insert_with(|| {
            groups.push((t.n, Vec::new()));
            groups.len() - 1
        });
        if let Some((_, pieces)) = groups.get_mut(g) {
            pieces.extend(kept.into_iter().filter(|(_, k)| *k).map(|(p, _)| p));
        }
    }
    for (n, pieces) in groups {
        if pieces.is_empty() {
            continue;
        }
        for p in merge(&pieces, n, tol.weld).unwrap_or(pieces) {
            emit(p)?;
        }
    }
    Ok(())
}

/// The planes that separate `t` into pieces each wholly on one side of the other solid's surface.
fn cutting_planes(t: &Tri, other: &Solid, found: &[u32], tol: &Tol) -> Vec<(Vec3, f64)> {
    let mut planes: Vec<(Vec3, f64)> = Vec::new();
    let mut add = |n: Vec3, d: f64| {
        let dup = planes.iter().any(|(m, e)| {
            let c = m.dot(n);
            (c > 1.0 - 1e-12 && (d - e).abs() <= tol.plane) || (c < -1.0 + 1e-12 && (d + e).abs() <= tol.plane)
        });
        if !dup {
            planes.push((n, d));
        }
    };
    let apart = |ds: [f64; 3]| ds.iter().all(|x| *x > tol.plane) || ds.iter().all(|x| *x < -tol.plane);
    for s in found.iter().filter_map(|i| other.tri(*i)) {
        let ds = s.p.map(|q| t.n.dot(q) - t.d);
        if apart(ds) {
            continue;
        }
        if ds.iter().all(|x| x.abs() <= tol.plane) {
            // Face to face: cut along the other triangle's edges, across `t`'s plane.
            for (u, w) in [(s.p[0], s.p[1]), (s.p[1], s.p[2]), (s.p[2], s.p[0])] {
                let n = t.n.cross(w - u).normalized();
                if n != Vec3::ZERO {
                    add(n, n.dot(u));
                }
            }
            continue;
        }
        if !apart(t.p.map(|q| s.n.dot(q) - s.d)) {
            add(s.n, s.d);
        }
    }
    planes
}

/// Cut triangle `t` by each plane in turn.
fn cut(t: &Tri, planes: &[(Vec3, f64)], tol: &Tol) -> Result<Vec<Vec<Vec3>>, SolidError> {
    let mut parts = vec![t.p.to_vec()];
    for (n, d) in planes {
        let mut next = Vec::with_capacity(parts.len() + 2);
        for p in parts {
            let (front, back) = split(p, *n, *d, tol.plane);
            next.extend([front, back].into_iter().filter(|q| q.len() >= 3));
        }
        if next.len() > MAX_PIECES {
            return Err(SolidError::TooComplex);
        }
        parts = next;
    }
    Ok(parts)
}

/// Split a convex polygon by the plane `n·x = d` into its front and back parts (either may be
/// empty). Points within `tol` of the plane count as on it.
fn split(poly: Vec<Vec3>, n: Vec3, d: f64, tol: f64) -> (Vec<Vec3>, Vec<Vec3>) {
    let side = |x: f64| {
        if x > tol {
            1
        } else if x < -tol {
            -1
        } else {
            0
        }
    };
    let dist: Vec<f64> = poly.iter().map(|p| n.dot(*p) - d).collect();
    if !dist.iter().any(|x| side(*x) < 0) {
        return (poly, Vec::new());
    }
    if !dist.iter().any(|x| side(*x) > 0) {
        return (Vec::new(), poly);
    }
    let (mut front, mut back) = (Vec::new(), Vec::new());
    let len = poly.len();
    for (i, (p, dp)) in poly.iter().zip(&dist).enumerate() {
        let (Some(q), Some(dq)) = (poly.get((i + 1) % len), dist.get((i + 1) % len)) else { continue };
        let (sp, sq) = (side(*dp), side(*dq));
        if sp >= 0 {
            front.push(*p);
        }
        if sp <= 0 {
            back.push(*p);
        }
        if sp * sq < 0 {
            let x = p.lerp(*q, dp / (dp - dq));
            front.push(x);
            back.push(x);
        }
    }
    (front, back)
}

/// Where the convex piece `poly` (of a face with unit normal `n`) lies relative to `other`;
/// `None` for a piece with no area.
fn classify(poly: &[Vec3], n: Vec3, other: &Solid, tol: &Tol) -> Option<Side> {
    let count = poly.len() as f64;
    let c = poly.iter().fold(Vec3::ZERO, |s, p| s + *p) * (1.0 / count);
    let len = poly.len();
    let (mut twice_area, mut perimeter) = (Vec3::ZERO, 0.0);
    for (i, p) in poly.iter().enumerate() {
        let q = poly.get((i + 1) % len).copied().unwrap_or(*p);
        twice_area = twice_area + (*p - c).cross(q - c);
        perimeter += p.dist(q);
    }
    let area = twice_area.len() / 2.0;
    if area <= tol.plane * tol.plane {
        return None;
    }
    // Step off the face by a small fraction of the piece's inradius, so the step can't cross a
    // surface that meets the piece's edges at a shallow angle.
    let step = (0.02 * area / perimeter).clamp(tol.step_min, tol.step_max);
    let front = inside(other, c + n * step, tol);
    let back = inside(other, c - n * step, tol);
    Some(match (front, back) {
        (false, false) => Side::Outside,
        (true, true) => Side::Inside,
        (false, true) => Side::Same,
        (true, false) => Side::Opposite,
    })
}

enum Ray {
    Miss,
    /// Crossed the surface: +1 leaving the solid, -1 entering it.
    Cross(i64),
    /// Through an edge, a corner or along the surface: try another direction.
    Unsure,
}

fn ray_tri(o: Vec3, dir: Vec3, t: &Tri, tol: &Tol) -> Ray {
    const EDGE: f64 = 1e-9;
    let (e1, e2) = (t.p[1] - t.p[0], t.p[2] - t.p[0]);
    let pv = dir.cross(e2);
    let det = e1.dot(pv);
    if det.abs() <= 1e-12 * e1.len() * e2.len() {
        return if (t.n.dot(o) - t.d).abs() <= tol.plane { Ray::Unsure } else { Ray::Miss };
    }
    let inv = 1.0 / det;
    let tv = o - t.p[0];
    let u = tv.dot(pv) * inv;
    let qv = tv.cross(e1);
    let v = dir.dot(qv) * inv;
    if u < -EDGE || v < -EDGE || u + v > 1.0 + EDGE {
        return Ray::Miss;
    }
    let dist = e2.dot(qv) * inv;
    if dist < -tol.plane {
        return Ray::Miss;
    }
    if dist <= tol.plane || u < EDGE || v < EDGE || u + v > 1.0 - EDGE {
        return Ray::Unsure;
    }
    Ray::Cross(if det > 0.0 { -1 } else { 1 })
}

/// Whether `p` is inside `s`: the signed count of surface crossings along a ray, or the winding
/// number when every ray grazes an edge.
fn inside(s: &Solid, p: Vec3, tol: &Tol) -> bool {
    for d in DIRS {
        let dir = Vec3::new(d[0], d[1], d[2]).normalized();
        let (mut winding, mut unsure) = (0i64, false);
        s.bvh.along(p, dir, 0.0, f64::INFINITY, |i| {
            if unsure {
                return;
            }
            match s.tri(i).map(|t| ray_tri(p, dir, t, tol)) {
                Some(Ray::Cross(w)) => winding += w,
                Some(Ray::Unsure) => unsure = true,
                _ => {}
            }
        });
        if !unsure {
            return winding > 0;
        }
    }
    winding_number(s, p) > 0.5
}

/// The generalized winding number of `p` (1 inside a closed outward mesh, 0 outside), from the
/// solid angles of the triangles (Van Oosterom and Strackee).
fn winding_number(s: &Solid, p: Vec3) -> f64 {
    let total: f64 = s
        .tris
        .iter()
        .map(|t| {
            let (a, b, c) = (t.p[0] - p, t.p[1] - p, t.p[2] - p);
            let (la, lb, lc) = (a.len(), b.len(), c.len());
            let num = a.dot(b.cross(c));
            let den = la * lb * lc + a.dot(b) * lc + a.dot(c) * lb + b.dot(c) * la;
            2.0 * num.atan2(den)
        })
        .sum();
    total / (4.0 * crate::PI)
}
