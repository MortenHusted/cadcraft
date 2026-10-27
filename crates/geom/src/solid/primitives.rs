//! Solid primitives: box, wedge, frustum (cylinder and cone), sphere, torus, and solids swept
//! from planar profiles (extrusion and revolution).
//!
//! Each returns an outward-facing closed [`Mesh`], or [`SolidError::Degenerate`] for sizes that
//! enclose no volume.

use super::triangulate::triangulate_with_holes;
use super::{MAX_TRIANGLES, Mesh, SolidError};
use crate::{Mat4, TAU, Vec2, Vec3, point_in_polygon, shoelace};

/// Largest distance (drawing units) between a curved face and its facets.
pub const CHORD_TOL: f64 = 0.01;
/// Fewest facets around a full circle, so curved faces never read as polygons.
pub const MIN_SEGMENTS: usize = 24;
/// Most facets around a full circle.
pub const MAX_SEGMENTS: usize = 720;
/// Most points in an extruded or revolved profile.
pub const MAX_PROFILE_POINTS: usize = 100_000;
/// Largest coordinate, size or distance a solid may have (drawing units): beyond it volumes and
/// inertias overflow.
pub const MAX_COORD: f64 = 1e12;

/// Facets around a full circle of `radius` (see [`CHORD_TOL`]).
pub fn circle_segments(radius: f64) -> usize {
    crate::arc_segments(radius, TAU, CHORD_TOL).clamp(MIN_SEGMENTS, MAX_SEGMENTS)
}

fn degenerate(m: &str) -> SolidError {
    SolidError::Degenerate(m.to_string())
}

fn positive(v: f64, what: &str) -> Result<f64, SolidError> {
    if v.is_finite() && v > crate::EPS && v <= MAX_COORD {
        Ok(v)
    } else {
        Err(degenerate(&format!("The {what} must be a positive number up to {MAX_COORD:e}.")))
    }
}

fn in_range(v: f64) -> bool {
    v.is_finite() && v.abs() <= MAX_COORD
}

fn finite(p: Vec3, what: &str) -> Result<Vec3, SolidError> {
    if in_range(p.x) && in_range(p.y) && in_range(p.z) {
        Ok(p)
    } else {
        Err(degenerate(&format!("The {what} must be a point within {MAX_COORD:e} of the origin.")))
    }
}

fn segment_count(n: usize) -> Result<u32, SolidError> {
    let n = n.clamp(3, MAX_SEGMENTS);
    u32::try_from(n).map_err(|_| SolidError::TooComplex)
}

/// Builds a mesh one vertex and face at a time.
#[derive(Default)]
struct Builder {
    mesh: Mesh,
}

impl Builder {
    fn vertex(&mut self, p: Vec3) -> u32 {
        self.mesh.vertices.push(p);
        u32::try_from(self.mesh.vertices.len() - 1).unwrap_or(u32::MAX)
    }
    fn tri(&mut self, a: u32, b: u32, c: u32) {
        if a != b && b != c && c != a {
            self.mesh.triangles.push([a, b, c]);
        }
    }
    /// A quad `a b c d` (counter-clockwise seen from outside) as two triangles.
    fn quad(&mut self, a: u32, b: u32, c: u32, d: u32) {
        self.tri(a, b, c);
        self.tri(a, c, d);
    }
    fn finish(self) -> Result<Mesh, SolidError> {
        let mut m = self.mesh;
        if m.triangles.len() > MAX_TRIANGLES {
            return Err(SolidError::TooComplex);
        }
        m.orient_outward();
        Ok(m)
    }
}

/// An axis-aligned box between two opposite corners.
pub fn cuboid(a: Vec3, b: Vec3) -> Result<Mesh, SolidError> {
    let (a, b) = (finite(a, "corner")?, finite(b, "corner")?);
    let (lo, hi) = (a.min(b), a.max(b));
    let size = hi - lo;
    positive(size.x, "length")?;
    positive(size.y, "width")?;
    positive(size.z, "height")?;
    let mut m = Builder::default();
    // Vertex i has x from bit 0, y from bit 1, z from bit 2.
    let v: Vec<u32> = (0..8)
        .map(|i| {
            m.vertex(Vec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z }))
        })
        .collect();
    let at = |i: usize| v.get(i).copied().unwrap_or(0);
    for [p, q, r, s] in [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]] {
        m.quad(at(p), at(q), at(r), at(s));
    }
    m.finish()
}

/// AutoCAD's wedge: the box from corner `a` to `b` cut by a slope that falls from the top of the
/// face at `a.x` to the bottom of the face at `b.x`.
pub fn wedge(a: Vec3, b: Vec3) -> Result<Mesh, SolidError> {
    let (a, b) = (finite(a, "corner")?, finite(b, "corner")?);
    positive((b.x - a.x).abs(), "length")?;
    positive((b.y - a.y).abs(), "width")?;
    positive((b.z - a.z).abs(), "height")?;
    let (z0, z1) = (a.z, b.z);
    let mut m = Builder::default();
    let p0 = m.vertex(Vec3::new(a.x, a.y, z0));
    let p1 = m.vertex(Vec3::new(b.x, a.y, z0));
    let p2 = m.vertex(Vec3::new(b.x, b.y, z0));
    let p3 = m.vertex(Vec3::new(a.x, b.y, z0));
    let t0 = m.vertex(Vec3::new(a.x, a.y, z1));
    let t3 = m.vertex(Vec3::new(a.x, b.y, z1));
    // Built with one winding throughout; `finish` turns it outward.
    m.quad(p0, p3, p2, p1);
    m.quad(p0, t0, t3, p3);
    m.tri(p0, p1, t0);
    m.tri(p3, t3, p2);
    m.quad(p1, p2, t3, t0);
    m.finish()
}

/// A frustum along +Z from `base` (radius `r0`) to `base + height·Z` (radius `r1`): a cylinder
/// when the radii are equal, a cone when one is zero. A negative height builds downward.
pub fn frustum(base: Vec3, r0: f64, r1: f64, height: f64, segments: usize) -> Result<Mesh, SolidError> {
    let base = finite(base, "center")?;
    if !(in_range(r0) && in_range(r1)) || r0 < 0.0 || r1 < 0.0 || r0.max(r1) <= crate::EPS {
        return Err(degenerate("The radius must be a positive number."));
    }
    if !in_range(height) || height.abs() <= crate::EPS {
        return Err(degenerate("The height must be a non-zero number."));
    }
    let n = segment_count(segments)?;
    let ring = |m: &mut Builder, r: f64, z: f64| -> Vec<u32> {
        (0..n)
            .map(|i| m.vertex(base + Vec3::new(r * (TAU * f64::from(i) / f64::from(n)).cos(), r * (TAU * f64::from(i) / f64::from(n)).sin(), z)))
            .collect()
    };
    let mut m = Builder::default();
    let bottom = if r0 > crate::EPS { ring(&mut m, r0, 0.0) } else { vec![m.vertex(base)] };
    let top = if r1 > crate::EPS { ring(&mut m, r1, height) } else { vec![m.vertex(base + Vec3::new(0.0, 0.0, height))] };
    let at = |v: &[u32], i: u32| v.get((i % n) as usize).or_else(|| v.first()).copied().unwrap_or(0);
    if bottom.len() > 1 {
        let c = m.vertex(base);
        (0..n).for_each(|i| m.tri(c, at(&bottom, i + 1), at(&bottom, i)));
    }
    if top.len() > 1 {
        let c = m.vertex(base + Vec3::new(0.0, 0.0, height));
        (0..n).for_each(|i| m.tri(c, at(&top, i), at(&top, i + 1)));
    }
    for i in 0..n {
        m.quad(at(&bottom, i), at(&bottom, i + 1), at(&top, i + 1), at(&top, i));
    }
    m.finish()
}

/// A sphere of `radius` about `center`, `segments` facets around its equator.
pub fn sphere(center: Vec3, radius: f64, segments: usize) -> Result<Mesh, SolidError> {
    let c = finite(center, "center")?;
    let r = positive(radius, "radius")?;
    let n = segment_count(segments)?;
    let rings = (n / 2).max(2);
    let mut m = Builder::default();
    let south = m.vertex(c - Vec3::Z * r);
    let north = m.vertex(c + Vec3::Z * r);
    let grid: Vec<Vec<u32>> = (1..rings)
        .map(|j| {
            let phi = -TAU / 4.0 + TAU / 2.0 * f64::from(j) / f64::from(rings);
            (0..n)
                .map(|i| {
                    let u = TAU * f64::from(i) / f64::from(n);
                    m.vertex(c + Vec3::new(r * phi.cos() * u.cos(), r * phi.cos() * u.sin(), r * phi.sin()))
                })
                .collect()
        })
        .collect();
    let at = |v: &[u32], i: u32| v.get((i % n) as usize).copied().unwrap_or(0);
    if let (Some(first), Some(last)) = (grid.first(), grid.last()) {
        for i in 0..n {
            m.tri(south, at(first, i + 1), at(first, i));
            m.tri(at(last, i), at(last, i + 1), north);
        }
    }
    for pair in grid.windows(2) {
        let [lo, hi] = pair else { continue };
        for i in 0..n {
            m.quad(at(lo, i), at(lo, i + 1), at(hi, i + 1), at(hi, i));
        }
    }
    m.finish()
}

/// A ring torus about `center` in the XY plane: `radius` to the tube's center, `tube` its radius.
pub fn torus(center: Vec3, radius: f64, tube: f64, segments: usize) -> Result<Mesh, SolidError> {
    let c = finite(center, "center")?;
    let big = positive(radius, "torus radius")?;
    let small = positive(tube, "tube radius")?;
    if small >= big {
        return Err(degenerate("The tube radius must be smaller than the torus radius."));
    }
    let n = segment_count(segments)?;
    let k = segment_count(circle_segments(small))?;
    let mut m = Builder::default();
    let grid: Vec<Vec<u32>> = (0..n)
        .map(|i| {
            let u = TAU * f64::from(i) / f64::from(n);
            (0..k)
                .map(|j| {
                    let v = TAU * f64::from(j) / f64::from(k);
                    let d = big + small * v.cos();
                    m.vertex(c + Vec3::new(d * u.cos(), d * u.sin(), small * v.sin()))
                })
                .collect()
        })
        .collect();
    let at = |i: u32, j: u32| grid.get((i % n) as usize).and_then(|r| r.get((j % k) as usize)).copied().unwrap_or(0);
    for i in 0..n {
        for j in 0..k {
            m.quad(at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
        }
    }
    m.finish()
}

/// Closed loops cleaned of repeated points, each with at least three points and some area.
fn clean_loops(loops: &[Vec<Vec2>]) -> Result<Vec<Vec<Vec2>>, SolidError> {
    let total: usize = loops.iter().map(Vec::len).sum();
    if total > MAX_PROFILE_POINTS {
        return Err(SolidError::TooComplex);
    }
    let size = loops.iter().flatten().fold(0.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs())).max(1.0);
    let tol = size * 1e-12;
    let mut out = Vec::new();
    for l in loops {
        if l.iter().any(|p| !(in_range(p.x) && in_range(p.y))) {
            return Err(degenerate("The profile has a point that is not a finite number or is too far out."));
        }
        let mut pts: Vec<Vec2> = Vec::with_capacity(l.len());
        for p in l {
            if !pts.last().is_some_and(|q| q.near(*p, tol)) {
                pts.push(*p);
            }
        }
        while pts.len() > 1 && pts.first().zip(pts.last()).is_some_and(|(a, b)| a.near(*b, tol)) {
            pts.pop();
        }
        if pts.len() >= 3 && shoelace(&pts).abs() > tol * size {
            out.push(pts);
        }
    }
    if out.is_empty() {
        return Err(degenerate("The profile encloses no area."));
    }
    Ok(out)
}

/// Triangles covering the region the closed `loops` bound, counter-clockwise: a loop inside an
/// odd number of others is a hole, whichever way each winds. Indices number the loops' points
/// in order.
pub(crate) fn cap(loops: &[Vec<Vec2>]) -> Result<Vec<[usize; 3]>, SolidError> {
    let depth: Vec<usize> = loops
        .iter()
        .enumerate()
        .map(|(i, l)| {
            let probe = l.first().copied().unwrap_or(Vec2::ZERO);
            loops.iter().enumerate().filter(|(j, o)| *j != i && point_in_polygon(o, probe)).count()
        })
        .collect();
    // Each loop wound as the triangulation wants it (outer loops counter-clockwise, holes
    // clockwise), with the index of each of its points.
    let mut start = 0;
    let mut wound: Vec<(Vec<Vec2>, Vec<usize>)> = Vec::with_capacity(loops.len());
    for (l, d) in loops.iter().zip(&depth) {
        let index: Vec<usize> = (start..start + l.len()).collect();
        start += l.len();
        wound.push(if (shoelace(l) > 0.0) != (d % 2 == 0) {
            (l.iter().rev().copied().collect(), index.into_iter().rev().collect())
        } else {
            (l.clone(), index)
        });
    }
    let mut out = Vec::new();
    for (i, (outer, outer_index)) in wound.iter().enumerate() {
        let d = depth.get(i).copied().unwrap_or(0);
        if d % 2 == 1 {
            continue;
        }
        // Holes one level deeper inside this outer loop.
        let holes: Vec<&(Vec<Vec2>, Vec<usize>)> = wound
            .iter()
            .zip(&depth)
            .filter(|((h, _), hd)| **hd == d + 1 && h.first().is_some_and(|p| point_in_polygon(outer, *p)))
            .map(|(w, _)| w)
            .collect();
        let hole_pts: Vec<&[Vec2]> = holes.iter().map(|(h, _)| h.as_slice()).collect();
        // The triangulation numbers the outer loop's points, then each hole's.
        let map: Vec<usize> = outer_index.iter().chain(holes.iter().flat_map(|(_, ix)| ix)).copied().collect();
        for t in triangulate_with_holes(outer, &hole_pts)? {
            if let (Some(a), Some(b), Some(c)) = (map.get(t[0]), map.get(t[1]), map.get(t[2])) {
                out.push([*a, *b, *c]);
            }
        }
    }
    Ok(out)
}

/// A planar profile split into faces: each outer loop (counter-clockwise) with the holes
/// (clockwise) directly inside it, triangulated. Points are numbered in loop order.
struct Profile {
    loops: Vec<Vec<Vec2>>,
    /// Cap triangles, indices into the loops' points taken in order.
    cap: Vec<[usize; 3]>,
}

impl Profile {
    fn new(loops: &[Vec<Vec2>]) -> Result<Profile, SolidError> {
        let mut loops = clean_loops(loops)?;
        // Nesting depth decides outer (even) or hole (odd); orient outer loops CCW, holes CW.
        let depth: Vec<usize> = loops
            .iter()
            .enumerate()
            .map(|(i, l)| {
                let probe = l.first().copied().unwrap_or(Vec2::ZERO);
                loops.iter().enumerate().filter(|(j, o)| *j != i && point_in_polygon(o, probe)).count()
            })
            .collect();
        for (l, d) in loops.iter_mut().zip(&depth) {
            if (shoelace(l) > 0.0) != (d % 2 == 0) {
                l.reverse();
            }
        }
        let cap = cap(&loops)?;
        Ok(Profile { loops, cap })
    }

    fn points(&self) -> impl Iterator<Item = Vec2> + '_ {
        self.loops.iter().flatten().copied()
    }

    /// Each loop edge as point indices `(from, to)`.
    fn edges(&self) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        let mut start = 0;
        for l in &self.loops {
            let n = l.len();
            out.extend((0..n).map(|k| (start + k, start + (k + 1) % n)));
            start += n;
        }
        out
    }
}

/// Extrude the region bounded by `loops` (in the plane `z = elevation`; nested loops are holes)
/// along `direction`, which must leave the plane.
pub fn extrude(loops: &[Vec<Vec2>], elevation: f64, direction: Vec3) -> Result<Mesh, SolidError> {
    if !in_range(elevation) || finite(direction, "direction").is_err() || direction.z.abs() <= crate::EPS {
        return Err(degenerate("The extrusion direction must leave the profile's plane."));
    }
    let p = Profile::new(loops)?;
    let mut m = Builder::default();
    let bottom: Vec<u32> = p.points().map(|q| m.vertex(q.to3(elevation))).collect();
    let top: Vec<u32> = p.points().map(|q| m.vertex(q.to3(elevation) + direction)).collect();
    let (b, t) = (|i: usize| bottom.get(i).copied().unwrap_or(0), |i: usize| top.get(i).copied().unwrap_or(0));
    for [x, y, z] in &p.cap {
        m.tri(b(*x), b(*z), b(*y));
        m.tri(t(*x), t(*y), t(*z));
    }
    for (i, j) in p.edges() {
        m.quad(b(i), b(j), t(j), t(i));
    }
    m.finish()
}

/// Revolve the region bounded by `loops` (in the XY plane) by `angle` radians about the line
/// through `origin` along `axis` (in the same plane). The region must lie on one side of the
/// axis; points on the axis stay single vertices.
pub fn revolve(loops: &[Vec<Vec2>], origin: Vec2, axis: Vec2, angle: f64, segments: usize) -> Result<Mesh, SolidError> {
    let dir = axis.normalized();
    if !(in_range(origin.x) && in_range(origin.y)) || dir == Vec2::ZERO {
        return Err(degenerate("The axis of revolution needs two different points."));
    }
    if !angle.is_finite() || angle.abs() <= 1e-9 {
        return Err(degenerate("The angle of revolution must be a non-zero number."));
    }
    let p = Profile::new(loops)?;
    let size = p.points().fold(0.0f64, |m, q| m.max((q - origin).len())).max(1e-300);
    let side = |q: Vec2| dir.cross(q - origin);
    let (lo, hi) = p.points().fold((0.0f64, 0.0f64), |(lo, hi), q| (lo.min(side(q)), hi.max(side(q))));
    if lo < -size * 1e-9 && hi > size * 1e-9 {
        return Err(degenerate("The profile crosses the axis of revolution."));
    }
    let full = angle.abs() >= TAU - 1e-9;
    let sweep = angle.clamp(-TAU, TAU);
    let n = segment_count(((segments as f64) * sweep.abs() / TAU).ceil() as usize)?;
    let rings = if full { n } else { n + 1 };
    let axis3 = dir.to3(0.0);
    let pts: Vec<Vec2> = p.points().collect();
    if pts.len().saturating_mul(rings as usize) > MAX_TRIANGLES / 2 {
        return Err(SolidError::TooComplex);
    }
    let on_axis: Vec<bool> = pts.iter().map(|q| side(*q).abs() <= size * 1e-9).collect();
    let mut m = Builder::default();
    // vertex[k][i]: point i turned by step k; points on the axis share one vertex.
    let mut grid: Vec<Vec<u32>> = Vec::with_capacity(rings as usize);
    for k in 0..rings {
        let rot = Mat4::rotate_about_axis(origin.to3(0.0), axis3, sweep * f64::from(k) / f64::from(n));
        let row: Vec<u32> = pts
            .iter()
            .enumerate()
            .map(|(i, q)| match (on_axis.get(i), grid.first().and_then(|r| r.get(i))) {
                (Some(true), Some(v)) => *v,
                _ => m.vertex(rot.apply(q.to3(0.0))),
            })
            .collect();
        grid.push(row);
    }
    let at = |k: u32, i: usize| grid.get((k % rings) as usize).and_then(|r| r.get(i)).copied().unwrap_or(0);
    for k in 0..n {
        for (i, j) in p.edges() {
            m.quad(at(k, i), at(k, j), at(k + 1, j), at(k + 1, i));
        }
    }
    if !full {
        for [x, y, z] in &p.cap {
            m.tri(at(0, *x), at(0, *z), at(0, *y));
            m.tri(at(n, *x), at(n, *y), at(n, *z));
        }
    }
    m.finish()
}
