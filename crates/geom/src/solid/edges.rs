//! The edges of a solid, where its faces meet at an angle, and rounding (fillet) or bevelling
//! (chamfer) them.
//!
//! An [`Edge`] is a chain of straight pieces of the mesh that turns smoothly: a box has twelve,
//! and the rim of a cylinder's top is one. Rounding an edge removes the material between its two
//! faces that a ball of the fillet radius rolling along them can't reach (a convex edge) or fills
//! the corner the ball can't (a concave edge); a chamfer cuts or fills a flat bevel instead. The
//! cutter is skinned through one cross-section per point of the chain, mitred where it turns,
//! and subtracted from or united with the solid. Where an edge ends against a face that leans
//! away (a box corner) the cutter runs past it; against a face that leans over it (a wall) it
//! stops there.

use std::collections::HashMap;

use super::boolean::{BoolOp, boolean};
use super::primitives::circle_segments;
use super::skin::{Section, skin};
use super::{FEATURE_ANGLE, Mesh, SolidError};
use crate::{TAU, Vec3};

/// Sharpest turn (degrees) between consecutive pieces of one edge.
const MAX_TURN: f64 = 45.0;

fn degenerate(m: &str) -> SolidError {
    SolidError::Degenerate(m.to_string())
}

/// One straight piece of an edge, with the outward normals of the faces on its left and right
/// (seen from outside, walking from `a` to `b`).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Piece {
    a: Vec3,
    b: Vec3,
    left: Vec3,
    right: Vec3,
}

impl Piece {
    fn dir(&self) -> Vec3 {
        (self.b - self.a).normalized()
    }
    fn convex(&self) -> bool {
        self.left.cross(self.right).dot(self.dir()) > 0.0
    }
    fn distance_to(&self, p: Vec3) -> f64 {
        let ab = self.b - self.a;
        let t = if ab.dot(ab) > 0.0 { ((p - self.a).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0) } else { 0.0 };
        p.dist(self.a + ab * t)
    }
}

/// An edge of a solid: straight pieces end to end, the faces on either side meeting at an angle.
#[derive(Clone, Debug, PartialEq)]
pub struct Edge {
    pieces: Vec<Piece>,
    /// Whether the last piece ends where the first begins (a rim all the way round).
    pub closed: bool,
    /// Whether the material lies inside the angle between the faces (an outside corner).
    pub convex: bool,
    /// The outward normals of the faces round the first and last points (for an open edge).
    start_faces: Vec<Vec3>,
    end_faces: Vec<Vec3>,
}

impl Edge {
    /// The points along the edge, in order (the first not repeated for a closed edge).
    pub fn points(&self) -> Vec<Vec3> {
        let mut out: Vec<Vec3> = self.pieces.iter().map(|p| p.a).collect();
        if let (false, Some(p)) = (self.closed, self.pieces.last()) {
            out.push(p.b);
        }
        out
    }

    pub fn length(&self) -> f64 {
        self.pieces.iter().map(|p| p.a.dist(p.b)).sum()
    }

    /// The shortest distance from `p` to the edge.
    pub fn distance_to(&self, p: Vec3) -> f64 {
        self.pieces.iter().map(|q| q.distance_to(p)).fold(f64::INFINITY, f64::min)
    }

    /// The outward normal of the face on the left of the first piece and on its right.
    pub fn faces(&self) -> (Vec3, Vec3) {
        self.pieces.first().map_or((Vec3::ZERO, Vec3::ZERO), |p| (p.left, p.right))
    }
}

fn unit_normal(m: &Mesh, t: &[u32; 3]) -> Option<Vec3> {
    let [a, b, c] = m.corners(t)?;
    let n = (b - a).cross(c - a);
    (n.len() > 0.0).then(|| n.normalized())
}

/// The solid's edges: where two faces meet at more than [`FEATURE_ANGLE`] degrees, joined into
/// chains where the next piece carries on within [`MAX_TURN`] degrees with the same faces on
/// the same sides.
pub fn edges(m: &Mesh) -> Vec<Edge> {
    let normals: Vec<Option<Vec3>> = m.triangles.iter().map(|t| unit_normal(m, t)).collect();
    // Directed edge → the triangle running along it.
    let mut along: HashMap<(u32, u32), usize> = HashMap::new();
    for (i, t) in m.triangles.iter().enumerate() {
        for k in 0..3 {
            along.insert((t[k], t[(k + 1) % 3]), i);
        }
    }
    let mut around: HashMap<u32, Vec<Vec3>> = HashMap::new();
    for (t, n) in m.triangles.iter().zip(&normals) {
        if let Some(n) = n {
            for v in t {
                around.entry(*v).or_default().push(*n);
            }
        }
    }
    let cos_feature = crate::rad(FEATURE_ANGLE).cos();
    let normal = |u: u32, v: u32| along.get(&(u, v)).and_then(|t| normals.get(*t).copied().flatten());
    // Each feature edge once, by its two vertices (lower first), sorted: HashMap order varies
    // from run to run, and the edges should come out the same every time.
    let mut feature: Vec<(u32, u32)> = along
        .keys()
        .filter(|(u, v)| u < v && matches!((normal(*u, *v), normal(*v, *u)), (Some(a), Some(b)) if a.dot(b) < cos_feature))
        .copied()
        .collect();
    feature.sort_unstable();
    let mut at_vertex: HashMap<u32, Vec<usize>> = HashMap::new();
    for (i, (u, v)) in feature.iter().enumerate() {
        at_vertex.entry(*u).or_default().push(i);
        at_vertex.entry(*v).or_default().push(i);
    }
    let point = |v: u32| m.vertices.get(v as usize).copied().unwrap_or(Vec3::ZERO);
    // The other end of feature edge `i` from vertex `w`.
    let far = |i: usize, w: u32| feature.get(i).map(|&(a, b)| if a == w { b } else { a });
    // Feature edge `i` walked from vertex `from`.
    let piece = |i: usize, from: u32| -> Option<Piece> {
        let to = far(i, from)?;
        Some(Piece { a: point(from), b: point(to), left: normal(from, to)?, right: normal(to, from)? })
    };
    let cos_turn = crate::rad(MAX_TURN).cos();
    // The edge the chain runs on along when it reaches vertex `w` on edge `i`: the only other
    // feature edge there, when it carries on smoothly with the same kind of corner.
    let through = |i: usize, w: u32| -> Option<usize> {
        let [x, y] = at_vertex.get(&w)?.as_slice() else { return None };
        let j = if *x == i { *y } else { *x };
        let (p, q) = (piece(i, far(i, w)?)?, piece(j, w)?);
        (q.convex() == p.convex() && p.dir().dot(q.dir()) >= cos_turn).then_some(j)
    };
    let mut used = vec![false; feature.len()];
    let mut out = Vec::new();
    for s in 0..feature.len() {
        let Some(&(u0, _)) = feature.get(s).filter(|_| !used.get(s).copied().unwrap_or(true)) else { continue };
        // Back up to where the chain begins; round a closed one, begin anywhere.
        let (mut first, mut start) = (s, u0);
        for _ in 0..feature.len() {
            match through(first, start) {
                Some(j) if j != s => {
                    start = far(j, start).unwrap_or(start);
                    first = j;
                }
                Some(_) => {
                    (first, start) = (s, u0);
                    break;
                }
                None => break,
            }
        }
        let (mut i, mut from) = (first, start);
        let mut pieces = Vec::new();
        let mut closed = false;
        loop {
            if let Some(u) = used.get_mut(i) {
                *u = true;
            }
            let (Some(p), Some(to)) = (piece(i, from), far(i, from)) else { break };
            pieces.push(p);
            from = to;
            match through(i, to) {
                Some(j) if j == first => {
                    closed = true;
                    break;
                }
                Some(j) if !used.get(j).copied().unwrap_or(true) => i = j,
                _ => break,
            }
        }
        let faces = |v: u32| around.get(&v).cloned().unwrap_or_default();
        let convex = pieces.first().is_some_and(Piece::convex);
        if !pieces.is_empty() {
            out.push(Edge { pieces, closed, convex, start_faces: faces(start), end_faces: faces(from) });
        }
    }
    out
}

/// How an edge is finished.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Blend {
    /// Rounded to this radius.
    Fillet(f64),
    /// Bevelled: the distances along the faces on the edge's left and right.
    Chamfer(f64, f64),
}

/// The plane the cutter's end lies in where an open edge ends at `v` heading `out`: past a face
/// that leans away from the edge (by `margin`), up to a face that leans over it.
fn end_plane(v: Vec3, out: Vec3, faces: &[Vec3], piece: &Piece, convex: bool, margin: f64) -> (Vec3, Vec3) {
    let own = |n: &Vec3| n.dot(piece.left) > 0.999 || n.dot(piece.right) > 0.999;
    let end = faces.iter().filter(|n| !own(n) && n.dot(out).abs() >= 0.1).max_by(|a, b| a.dot(out).abs().total_cmp(&b.dot(out).abs()));
    match end {
        // A cut runs on through the air past a face leaning away; a fill runs on into the
        // material behind a face leaning over it.
        Some(n) if (n.dot(out) > 0.0) == convex => (v + *n * (if convex { margin } else { -margin }), *n),
        Some(n) => (v, *n),
        None => (v, out),
    }
}

/// The cross-section of the cutter for `p`, as offsets from a point on its line, in the plane
/// across it: the corner between the faces up to the fillet arc or chamfer line, reaching
/// `margin` past the faces so no cutter face lies on them.
fn cross_section(p: &Piece, how: Blend, arc_points: usize, convex: bool) -> Result<Vec<Vec3>, SolidError> {
    let d = p.dir();
    let (wl, wr) = (p.left.cross(d).normalized(), d.cross(p.right).normalized());
    let theta = wl.dot(wr).clamp(-1.0, 1.0).acos();
    if !(1e-3..=std::f64::consts::PI - 1e-3).contains(&theta) {
        return Err(degenerate("The faces at the edge are too nearly flat or folded back."));
    }
    let (tl, tr, arc) = match how {
        Blend::Fillet(r) => {
            let reach = r / (theta / 2.0).tan();
            let center = (wl + wr).normalized() * (r / (theta / 2.0).sin());
            let (tl, tr) = (wl * reach, wr * reach);
            // From the right tangent point to the left one, round the side facing the edge.
            let e1 = (tr - center).normalized();
            let e2 = {
                let t = tl - center;
                (t - e1 * t.dot(e1)).normalized()
            };
            let sweep = std::f64::consts::PI - theta;
            let arc: Vec<Vec3> = (1..arc_points)
                .map(|k| {
                    let t = sweep * k as f64 / arc_points as f64;
                    center + (e1 * t.cos() + e2 * t.sin()) * r
                })
                .collect();
            (tl, tr, arc)
        }
        Blend::Chamfer(dl, dr) => (wl * dl, wr * dr, Vec::new()),
    };
    let margin = 0.1 * tl.len().min(tr.len()) * if convex { 1.0 } else { -1.0 };
    let mut out = vec![tl + p.left * margin, (p.left + p.right) * margin, tr + p.right * margin, tr];
    out.extend(arc);
    out.push(tl);
    Ok(out)
}

/// The points of `offsets` (a cross-section across `along` at `base`) carried along `along` to
/// the plane through `q` with normal `n`.
fn onto_plane(offsets: &[Vec3], base: Vec3, along: Vec3, q: Vec3, n: Vec3) -> Result<Vec<Vec3>, SolidError> {
    let dn = n.dot(along);
    if dn.abs() < 0.05 {
        return Err(degenerate("The edge meets the face at its end too obliquely."));
    }
    Ok(offsets
        .iter()
        .map(|o| {
            let x = base + *o;
            x + along * (n.dot(q - x) / dn)
        })
        .collect())
}

/// The solid that `how` removes along `e` (a convex edge) or adds (a concave one). `stops`
/// overrides the plane (a point and normal) the cutter ends in at the edge's start and end.
fn cutter(e: &Edge, how: Blend, stops: [Option<(Vec3, Vec3)>; 2]) -> Result<Mesh, SolidError> {
    let sizes = match how {
        Blend::Fillet(r) => [r, r],
        Blend::Chamfer(a, b) => [a, b],
    };
    if sizes.iter().any(|v| !(v.is_finite() && *v > 0.0 && *v <= super::primitives::MAX_COORD)) {
        return Err(degenerate("The radius or distance must be a positive number."));
    }
    let size = sizes[0].min(sizes[1]);
    let widest = e.pieces.iter().map(|p| p.left.dot(p.right).clamp(-1.0, 1.0).acos()).fold(0.0, f64::max);
    let arc_points = match how {
        Blend::Fillet(r) => ((circle_segments(r) as f64 * widest / TAU).ceil() as usize).max(2),
        Blend::Chamfer(..) => 1,
    };
    let shapes: Vec<Vec<Vec3>> = e.pieces.iter().map(|p| cross_section(p, how, arc_points, e.convex)).collect::<Result<_, _>>()?;
    let margin = 0.1 * size;
    let n = e.pieces.len();
    let mut sections: Vec<Section> = Vec::with_capacity(n + 1);
    // Where piece `i` begins: the mitre with the piece before it, or the open edge's start.
    for i in 0..n {
        let (Some(p), Some(shape)) = (e.pieces.get(i), shapes.get(i)) else { continue };
        let prev = if i > 0 {
            e.pieces.get(i - 1).zip(shapes.get(i - 1))
        } else if e.closed {
            e.pieces.last().zip(shapes.last())
        } else {
            None
        };
        let section = match prev {
            Some((q, qs)) => {
                let mitre = (q.dir() + p.dir()).normalized();
                let mine = onto_plane(shape, p.a, p.dir(), p.a, mitre)?;
                let theirs = onto_plane(qs, q.a, q.dir(), p.a, mitre)?;
                mine.iter().zip(&theirs).map(|(a, b)| a.lerp(*b, 0.5)).collect()
            }
            None => {
                let (q, nrm) = stops[0].unwrap_or_else(|| end_plane(p.a, p.dir() * -1.0, &e.start_faces, p, e.convex, margin));
                onto_plane(shape, p.a, p.dir(), q, nrm)?
            }
        };
        sections.push(vec![section]);
    }
    if !e.closed
        && let (Some(p), Some(shape)) = (e.pieces.last(), shapes.last())
    {
        let (q, nrm) = stops[1].unwrap_or_else(|| end_plane(p.b, p.dir(), &e.end_faces, p, e.convex, margin));
        sections.push(vec![onto_plane(shape, p.a, p.dir(), q, nrm)?]);
    }
    skin(&sections, e.closed)
}

/// A convex corner where three edges rounded to one radius meet: the ball touching its three
/// faces rounds it, and the edges' cutters stop where the ball takes over.
struct Corner {
    at: Vec3,
    /// The outward normals of the three faces.
    normals: [Vec3; 3],
    /// The centre of the ball.
    center: Vec3,
    radius: f64,
}

/// The point where the three planes `n·x = d` meet, if they do at one point.
fn meet([(n1, d1), (n2, d2), (n3, d3)]: [(Vec3, f64); 3]) -> Option<Vec3> {
    let det = n1.dot(n2.cross(n3));
    (det.abs() > 1e-9).then(|| (n2.cross(n3) * d1 + n3.cross(n1) * d2 + n1.cross(n2) * d3) * (1.0 / det))
}

/// The corners where exactly three convex edges, all rounded to the same radius, end.
fn corners(edges: &[(Edge, Blend)]) -> Vec<Corner> {
    // The ends of open convex rounded edges, by position: the edge, its piece there, the
    // direction into the edge, and the radius.
    let mut ends: HashMap<[u64; 3], Vec<(usize, Piece, Vec3, f64)>> = HashMap::new();
    for (i, (e, how)) in edges.iter().enumerate() {
        let (Blend::Fillet(r), false, true, Some(first), Some(last)) = (how, e.closed, e.convex, e.pieces.first(), e.pieces.last()) else { continue };
        for (at, piece, into) in [(first.a, *first, first.dir()), (last.b, *last, last.dir() * -1.0)] {
            ends.entry([at.x, at.y, at.z].map(f64::to_bits)).or_default().push((i, piece, into, *r));
        }
    }
    let mut out = Vec::new();
    for (key, group) in &ends {
        let [(i, p, _, r), (j, q, _, r2), (k, w, _, r3)] = group.as_slice() else { continue };
        if i == j || j == k || i == k || r != r2 || r != r3 {
            continue;
        }
        let mut normals: Vec<Vec3> = Vec::with_capacity(3);
        for n in [p.left, p.right, q.left, q.right, w.left, w.right] {
            if !normals.iter().any(|m| m.dot(n) > 0.999) {
                normals.push(n);
            }
        }
        let [n1, n2, n3] = normals.as_slice() else { continue };
        let at = Vec3::new(f64::from_bits(key[0]), f64::from_bits(key[1]), f64::from_bits(key[2]));
        let Some(center) = meet([(*n1, n1.dot(at) - r), (*n2, n2.dot(at) - r), (*n3, n3.dot(at) - r)]) else { continue };
        // The ball takes over each edge's end; it must leave room for the other end's.
        let fits = group.iter().all(|(e, _, into, _)| edges.get(*e).is_some_and(|(ed, _)| (center - at).dot(*into) <= ed.length() / 2.0));
        if fits {
            out.push(Corner { at, normals: [*n1, *n2, *n3], center, radius: *r });
        }
    }
    out
}

/// The solid that rounds `c`: the corner's cell (between its faces, moved out by `margin`, and
/// the planes through the ball's centre across each edge) less the ball.
fn corner_cutter(c: &Corner, margin: f64) -> Result<Mesh, SolidError> {
    let [n1, n2, n3] = c.normals;
    let face = |n: Vec3| (n, n.dot(c.at) + margin);
    let across = |a: Vec3, b: Vec3| {
        let d = a.cross(b).normalized();
        (d, d.dot(c.center))
    };
    let (s12, s13, s23) = (across(n1, n2), across(n1, n3), across(n2, n3));
    let pts = [
        meet([face(n1), face(n2), face(n3)]),
        meet([face(n1), face(n2), s12]),
        meet([face(n1), face(n3), s13]),
        meet([face(n2), face(n3), s23]),
        meet([face(n1), s12, s13]),
        meet([face(n2), s12, s23]),
        meet([face(n3), s13, s23]),
        Some(c.center),
    ];
    let Some(vertices) = pts.into_iter().collect::<Option<Vec<Vec3>>>() else { return Err(degenerate("The corner's faces don't meet at a point.")) };
    // corner 0, edge points 1 (12), 2 (13), 3 (23), face points 4 (1), 5 (2), 6 (3), centre 7.
    let quads = [[0, 1, 4, 2], [0, 1, 5, 3], [0, 2, 6, 3], [7, 4, 1, 5], [7, 4, 2, 6], [7, 5, 3, 6]];
    let middle = vertices.iter().fold(Vec3::ZERO, |s, v| s + *v) * (1.0 / 8.0);
    let mut cell = Mesh { vertices, triangles: Vec::new() };
    for q in quads {
        let at = |i: usize| cell.vertices.get(i).copied().unwrap_or(Vec3::ZERO);
        let [a, b, d, e] = q.map(|i| u32::try_from(i).unwrap_or(0));
        let outward = (at(q[1]) - at(q[0])).cross(at(q[2]) - at(q[0])).dot(at(q[0]) - middle) > 0.0;
        let tris = if outward { [[a, b, d], [a, d, e]] } else { [[a, d, b], [a, e, d]] };
        cell.triangles.extend(tris);
    }
    let ball = super::primitives::sphere(c.center, c.radius, circle_segments(c.radius))?;
    boolean(&cell, &ball, BoolOp::Difference)
}

/// `m` with each edge finished as `how` says: convex edges cut, concave edges filled. Where three
/// edges rounded to one radius meet at a convex corner, the corner is rounded by a ball.
pub fn blend(m: &Mesh, edges: &[(Edge, Blend)]) -> Result<Mesh, SolidError> {
    let corners = corners(edges);
    // An edge ending at a rounded corner stops on the plane across it through the ball's centre.
    let stop = |at: Vec3, along: Vec3| corners.iter().find(|c| c.at == at).map(|c| (c.center, along));
    let mut out = m.clone();
    // Fill first, so a later cut can trim what a fill added.
    let (fills, cuts): (Vec<_>, Vec<_>) = edges.iter().partition(|(e, _)| !e.convex);
    for (e, how) in fills.into_iter().chain(cuts) {
        let stops = match (e.pieces.first(), e.pieces.last()) {
            (Some(f), Some(l)) if !e.closed => [stop(f.a, f.dir()), stop(l.b, l.dir())],
            _ => [None, None],
        };
        let c = cutter(e, *how, stops)?;
        out = boolean(&out, &c, if e.convex { BoolOp::Difference } else { BoolOp::Union })?;
    }
    for c in &corners {
        out = boolean(&out, &corner_cutter(c, 0.1 * c.radius)?, BoolOp::Difference)?;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::primitives::{cuboid, extrude, frustum};
    use crate::{Mat4, TAU, Vec2};

    fn near(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "{a} vs {b}");
    }

    #[test]
    fn a_box_has_twelve_straight_convex_edges() {
        let m = cuboid(Vec3::ZERO, Vec3::new(4.0, 3.0, 2.0)).unwrap();
        let es = edges(&m);
        assert_eq!(es.len(), 12);
        assert!(es.iter().all(|e| e.convex && !e.closed && e.pieces.len() == 1));
        near(es.iter().map(Edge::length).sum::<f64>(), 4.0 * (4.0 + 3.0 + 2.0), 1e-12);
        // A cylinder: its two rims, each one closed edge.
        let c = frustum(Vec3::ZERO, 2.0, 2.0, 5.0, 48).unwrap();
        let es = edges(&c);
        assert_eq!(es.len(), 2);
        assert!(es.iter().all(|e| e.closed && e.convex && e.pieces.len() == 48));
    }

    /// The material a fillet of radius r removes from a right-angled edge of length l:
    /// (1 − π/4)·r²·l.
    fn fillet_loss(r: f64, l: f64) -> f64 {
        (1.0 - TAU / 8.0) * r * r * l
    }

    #[test]
    fn filleting_a_box_edge_removes_the_corner_the_ball_cant_reach() {
        let m = cuboid(Vec3::ZERO, Vec3::new(10.0, 6.0, 4.0)).unwrap();
        let top_front = edges(&m).into_iter().find(|e| e.distance_to(Vec3::new(5.0, 0.0, 4.0)) < 1e-9).unwrap();
        let r = 1.5;
        let out = blend(&m, &[(top_front, Blend::Fillet(r))]).unwrap();
        assert!(out.check().is_closed());
        // The arc is faceted within the chord tolerance, its chords cutting a little more.
        near(m.volume() - out.volume(), fillet_loss(r, 10.0), 0.05 * fillet_loss(r, 10.0));
        assert!(out.bounds().min.dist(Vec3::ZERO) < 1e-9 && out.bounds().max.dist(Vec3::new(10.0, 6.0, 4.0)) < 1e-9);
    }

    #[test]
    fn all_top_edges_of_a_box_round_and_the_rim_of_a_cylinder_rounds_all_the_way() {
        let m = cuboid(Vec3::ZERO, Vec3::new(10.0, 6.0, 4.0)).unwrap();
        let tops: Vec<(Edge, Blend)> =
            edges(&m).into_iter().filter(|e| e.points().iter().all(|p| p.z == 4.0)).map(|e| (e, Blend::Fillet(1.0))).collect();
        assert_eq!(tops.len(), 4);
        let out = blend(&m, &tops).unwrap();
        assert!(out.check().is_closed());
        // Four edges' worth, less the overlaps at the corners (where both cutters remove the same).
        let loss = m.volume() - out.volume();
        assert!(loss < fillet_loss(1.0, 32.0) && loss > fillet_loss(1.0, 32.0 - 8.0), "{loss}");
        let c = frustum(Vec3::ZERO, 5.0, 5.0, 4.0, 96).unwrap();
        let rim = edges(&c).into_iter().find(|e| e.points().iter().all(|p| p.z == 4.0)).unwrap();
        let out = blend(&c, &[(rim, Blend::Fillet(1.0))]).unwrap();
        assert!(out.check().is_closed());
        // Pappus: the corner's area times the path its centroid takes round the axis.
        let area = 1.0 - TAU / 8.0;
        let inset = (10.0 - 1.5 * TAU) / (3.0 * (4.0 - TAU / 2.0));
        let exact = area * TAU * (5.0 - inset);
        near(c.volume() - out.volume(), exact, 0.05 * exact);
    }

    #[test]
    fn rounding_every_edge_of_a_box_rounds_its_corners_with_a_ball() {
        let (a, r) = (20.0, 4.0);
        let m = cuboid(Vec3::ZERO, Vec3::new(a, a, a)).unwrap();
        let all: Vec<(Edge, Blend)> = edges(&m).into_iter().map(|e| (e, Blend::Fillet(r))).collect();
        let out = blend(&m, &all).unwrap();
        assert!(out.check().is_closed());
        // The box shrunk by r all round, grown again by a ball of radius r.
        let i = a - 2.0 * r;
        let exact = i * i * i + 2.0 * r * 3.0 * i * i + TAU / 2.0 * r * r * 3.0 * i + 2.0 * TAU / 3.0 * r * r * r;
        near(out.volume(), exact, 0.01 * exact);
        // No point is left at the corner: the nearest is a ball's radius from its centre.
        let corner_gap = out.vertices.iter().map(|v| v.len()).fold(f64::INFINITY, f64::min);
        near(corner_gap, (3.0f64.sqrt() - 1.0) * r, 0.02 * r);
    }

    #[test]
    fn a_concave_edge_is_filled_and_a_chamfer_bevels() {
        // An L: a floor 10 × 10 × 2 and a wall 10 × 2 × 8 standing at its back.
        let floor = cuboid(Vec3::ZERO, Vec3::new(10.0, 10.0, 2.0)).unwrap();
        let wall = cuboid(Vec3::new(0.0, 8.0, 0.0), Vec3::new(10.0, 10.0, 8.0)).unwrap();
        let l = boolean(&floor, &wall, BoolOp::Union).unwrap();
        let inner = edges(&l).into_iter().find(|e| !e.convex).unwrap();
        near(inner.length(), 10.0, 1e-9);
        let out = blend(&l, &[(inner, Blend::Fillet(2.0))]).unwrap();
        assert!(out.check().is_closed());
        near(out.volume() - l.volume(), fillet_loss(2.0, 10.0), 0.05 * fillet_loss(2.0, 10.0));
        let (b, lb) = (out.bounds(), l.bounds());
        assert!(b.min.dist(lb.min) < 1e-9 && b.max.dist(lb.max) < 1e-9);
        // A 45° chamfer of 1 on the bottom front edge removes a prism of 1/2 · 1 · 1 · 10.
        let m = cuboid(Vec3::ZERO, Vec3::new(10.0, 6.0, 4.0)).unwrap();
        let bottom_front = edges(&m).into_iter().find(|e| e.distance_to(Vec3::new(5.0, 0.0, 0.0)) < 1e-9).unwrap();
        let out = blend(&m, &[(bottom_front, Blend::Chamfer(1.0, 1.0))]).unwrap();
        assert!(out.check().is_closed());
        near(m.volume() - out.volume(), 5.0, 1e-9);
    }

    #[test]
    fn a_tilted_hexagonal_prism_and_a_cone_round_all_over() {
        // Corners where the faces meet at 120° and 90°, edges in no axis direction.
        let hex: Vec<Vec2> = (0..6).map(|k| Vec2::polar(Vec2::ZERO, 10.0, TAU * k as f64 / 6.0)).collect();
        let prism =
            extrude(&[hex], 0.0, Vec3::new(0.0, 0.0, 8.0)).unwrap().transformed(&Mat4::rotate_about_axis(Vec3::ZERO, Vec3::new(1.0, 2.0, 0.5), 0.7));
        let all: Vec<(Edge, Blend)> = edges(&prism).into_iter().map(|e| (e, Blend::Fillet(1.5))).collect();
        assert_eq!(all.len(), 18);
        let out = blend(&prism, &all).unwrap();
        assert!(out.check().is_closed());
        assert!(out.volume() < prism.volume() && out.volume() > 0.9 * prism.volume());
        // A cone's base rim (the faces meet at 63°) and a cylinder's two rims.
        for m in [frustum(Vec3::ZERO, 6.0, 0.0, 12.0, 64).unwrap(), frustum(Vec3::ZERO, 4.0, 4.0, 3.0, 64).unwrap()] {
            let all: Vec<(Edge, Blend)> = edges(&m).into_iter().map(|e| (e, Blend::Fillet(1.0))).collect();
            let out = blend(&m, &all).unwrap();
            assert!(out.check().is_closed());
            assert!(out.volume() < m.volume() && out.volume() > 0.8 * m.volume());
        }
    }

    /// Tiny deterministic generator for the property test.
    struct Rng(u64);

    impl Rng {
        fn grid(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            ((self.0 >> 11) as f64 / (1u64 << 53) as f64 * 6.0).floor()
        }
    }

    #[test]
    fn every_edge_of_two_boxes_on_a_grid_rounds_or_bevels_into_a_closed_solid() {
        let cases: usize = std::env::var("EDGE_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(60);
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        let mut tried = 0;
        for case in 0..cases {
            let mut corner = || Vec3::new(rng.grid(), rng.grid(), rng.grid());
            let (p, q, r, s) = (corner(), corner(), corner(), corner());
            let (Ok(a), Ok(b)) = (cuboid(p, q), cuboid(r, s)) else { continue };
            let Ok(u) = boolean(&a, &b, BoolOp::Union) else { continue };
            // Boxes touching only along an edge make no closed solid to begin with.
            if !u.check().is_closed() {
                continue;
            }
            tried += 1;
            for how in [Blend::Fillet(0.25), Blend::Chamfer(0.2, 0.3)] {
                let all: Vec<(Edge, Blend)> = edges(&u).into_iter().map(|e| (e, how)).collect();
                let out = blend(&u, &all).unwrap_or_else(|e| panic!("case {case} {how:?}: {e} ({p:?} {q:?} / {r:?} {s:?})"));
                assert!(out.check().is_closed(), "case {case} {how:?}: {:?} ({p:?} {q:?} / {r:?} {s:?})", out.check());
                assert!(out.volume() > 0.0 && (out.volume() - u.volume()).abs() < 0.2 * u.volume(), "case {case} {how:?}");
            }
        }
        assert!(tried >= cases / 5, "{tried}");
    }

    #[test]
    fn hostile_sizes_are_refused() {
        let m = cuboid(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)).unwrap();
        let e = edges(&m).into_iter().next().unwrap();
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e300] {
            assert!(blend(&m, &[(e.clone(), Blend::Fillet(bad))]).is_err());
            assert!(blend(&m, &[(e.clone(), Blend::Chamfer(bad, 1.0))]).is_err());
        }
    }
}
