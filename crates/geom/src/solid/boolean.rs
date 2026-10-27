//! Boolean operations on closed meshes: union, subtraction and intersection.
//!
//! Each triangle of one solid is cut by the planes of the other solid's triangles that cross it
//! (and, where the two touch face to face, by the outline of the touching face), so that every
//! piece lies wholly inside the other solid, outside it, or on its surface. A piece is cut only
//! where the segment along which the other surface meets the triangle passes through it, so cuts
//! don't run across all of it; where the other surface turns inside the triangle, a cut across
//! the turn goes first, so no cut runs on past the end of its segment alongside the next one. The
//! corners where the surfaces meet are each computed once, from an edge of one solid and a plane
//! of the other, so both solids agree on them to the last bit however thin the triangles. A piece
//! is classified by casting rays from its centre, or, where it lies on the other solid's surface,
//! from points just in front of and just behind it: both outside, both inside, or one of each (the
//! two surfaces touch, facing the same way or opposite ways). The pieces the operation keeps are
//! welded into one closed mesh by [`super::stitch`]. A
//! triangle whose pieces are all kept stays whole, and the kept pieces of the others are joined
//! back into one outline per plane ([`super::merge`]), so cuts don't spread across faces far from
//! where the solids meet.

use std::collections::{BTreeMap, HashMap};

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
const MAX_PIECES: usize = 200_000;

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
    /// Per edge (`p[k]` to `p[k + 1]`): whether the triangle across it lies in the same plane
    /// facing the same way, so the edge is inside a flat face, not on its outline.
    inner: [bool; 3],
}

struct Solid {
    tris: Vec<Tri>,
    bvh: Bvh,
    bounds: Bounds3,
}

impl Solid {
    fn new(m: &Mesh, tol: &Tol) -> Solid {
        let mut ids: Vec<[u32; 3]> = Vec::with_capacity(m.triangles.len());
        let mut tris: Vec<Tri> = Vec::with_capacity(m.triangles.len());
        for t in &m.triangles {
            let Some(p) = m.corners(t) else { continue };
            let c = (p[1] - p[0]).cross(p[2] - p[0]);
            // Skip slivers with no area; they can't bound anything.
            if c.len() > tol.plane * tol.plane {
                let n = c.normalized();
                tris.push(Tri { p, n, d: n.dot(p[0]), bounds: Bounds3::from_points(p), inner: [false; 3] });
                ids.push(*t);
            }
        }
        // An edge is inside a flat face when the two triangles sharing it are coplanar.
        let mut by_edge: HashMap<(u32, u32), Vec<(usize, usize)>> = HashMap::new();
        for (i, t) in ids.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                by_edge.entry((a.min(b), a.max(b))).or_default().push((i, k));
            }
        }
        for uses in by_edge.values() {
            if let [(i, k), (j, l)] = uses.as_slice() {
                let flat = match (tris.get(*i), tris.get(*j)) {
                    (Some(a), Some(b)) => a.n.dot(b.n) > 1.0 - 1e-12 && (a.d - b.d).abs() <= tol.plane,
                    _ => false,
                };
                if let Some(e) = tris.get_mut(*i).and_then(|t| t.inner.get_mut(*k)) {
                    *e = flat;
                }
                if let Some(e) = tris.get_mut(*j).and_then(|t| t.inner.get_mut(*l)) {
                    *e = flat;
                }
            }
        }
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
    /// Points closer than this are one point: enough for those a cut takes as on its plane, and
    /// no more, as a finely tessellated curved face has corners not much further off the line
    /// through their neighbours.
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
        Tol { plane, weld: plane * 2.0, step_min: plane * 10.0, step_max: (extent * 1e-5).max(plane * 10.0) }
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
        let (cuts, flat) = cutters(t, other, &found, tol);
        if cuts.is_empty() {
            // Nothing of the other surface crosses the triangle, so its centroid tells which side
            // it is on, unless it lies on a flat face of the other with no outline near enough to
            // cut along: the centroid is then on that face, and `classify` steps off it, as for a
            // piece, to tell how the two face. (Stepping off every triangle would step through a
            // face hovering nearer than the step.)
            let c = (t.p[0] + t.p[1] + t.p[2]) * (1.0 / 3.0);
            let centroid = || if inside(other, c, tol) { Side::Inside } else { Side::Outside };
            let side = classify(&t.p, t.n, other, &flat, tol).unwrap_or_else(centroid);
            if keep(side, op, first) {
                emit(t.p.to_vec())?;
            }
            continue;
        }
        let parts = cut(t, &cuts, tol)?;
        let kept: Vec<(Vec<Vec3>, bool)> =
            parts.into_iter().filter_map(|p| classify(&p, t.n, other, &flat, tol).map(|s| (p, keep(s, op, first)))).collect();
        if kept.iter().all(|(_, k)| *k) {
            emit(t.p.to_vec())?;
            continue;
        }
        // Gather what is kept of cut triangles by plane, to rebuild each face as a whole.
        let key = [t.n.x, t.n.y, t.n.z].map(|c| (c * 1e6).round() as i64);
        let key = [key[0], key[1], key[2], (t.d / (tol.plane * 1000.0)).round() as i64];
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

/// A plane that cuts a triangle, and the segments along which the other solid's surface
/// actually meets the triangle's plane on it: only pieces those segments pass through need
/// cutting.
struct Cutter {
    n: Vec3,
    d: f64,
    segs: Vec<[Vec3; 2]>,
    /// The box around `segs`.
    reach: Bounds3,
    /// For a cut across a point where two segments meet: the point. It cuts only the piece
    /// around that point.
    across: Option<Vec3>,
}

/// The cuts that separate `t` into pieces each wholly on one side of the other solid's surface,
/// and the other solid's triangles lying in `t`'s plane.
fn cutters<'a>(t: &Tri, other: &'a Solid, found: &[u32], tol: &Tol) -> (Vec<Cutter>, Vec<&'a Tri>) {
    let mut flat: Vec<&Tri> = Vec::new();
    let mut cuts: Vec<Cutter> = Vec::new();
    let mut add = |n: Vec3, d: f64, a: Vec3, b: Vec3| {
        let reach = Bounds3::from_points([a, b]).expand(tol.plane);
        // The same cut only where the segment lies on its plane but for rounding (two triangles
        // of a flat face): nearly flat neighbours keep their own cuts, so the point where their
        // segments meet becomes a corner.
        let same = cuts.iter_mut().find(|c| c.n.dot(n).abs() > 1.0 - 1e-12 && [a, b].iter().all(|p| (c.n.dot(*p) - c.d).abs() <= tol.plane * 1e-3));
        match same {
            Some(c) => {
                c.reach = c.reach.union(&reach);
                c.segs.push([a, b]);
            }
            None => cuts.push(Cutter { n, d, segs: vec![[a, b]], reach, across: None }),
        }
    };
    let apart = |ds: [f64; 3]| ds.iter().all(|x| *x > tol.plane) || ds.iter().all(|x| *x < -tol.plane);
    for s in found.iter().filter_map(|i| other.tri(*i)) {
        let ds = s.p.map(|q| t.n.dot(q) - t.d);
        if apart(ds) {
            continue;
        }
        let edges = [(s.p[0], s.p[1]), (s.p[1], s.p[2]), (s.p[2], s.p[0])];
        if ds.iter().all(|x| x.abs() <= tol.plane) {
            flat.push(s);
            // Face to face: cut along the outline of the other face (its triangles' edges that
            // aren't inside it), across `t`'s plane.
            for (k, (u, w)) in edges.into_iter().enumerate() {
                let n = t.n.cross(w - u).normalized();
                if n != Vec3::ZERO && !s.inner.get(k).copied().unwrap_or(false) {
                    add(n, n.dot(u), u, w);
                }
            }
            continue;
        }
        if apart(t.p.map(|q| s.n.dot(q) - s.d)) {
            continue;
        }
        // Where `s` crosses `t`'s plane: the points of its edges on that plane.
        let dist = |q: Vec3| t.n.dot(q) - t.d;
        let mut on: Vec<Vec3> = Vec::with_capacity(3);
        for (u, w) in edges {
            let (du, dw) = (dist(u), dist(w));
            if du.abs() <= tol.plane {
                on.push(u);
            }
            if (du > tol.plane && dw < -tol.plane) || (du < -tol.plane && dw > tol.plane) {
                on.push(crossing(u, w, t.n, t.d));
            }
        }
        if let Some([a, b]) = farthest(&on) {
            add(s.n, s.d, a, b);
        }
    }
    let mut first = turns(t, &cuts, tol);
    first.append(&mut cuts);
    (first, flat)
}

/// Cuts across the points inside `t` where the segments of two different cuts meet (where an
/// edge of the other solid passes through `t`), to go before all others. The other solid's
/// surface turns there, often by very little (a curved face tessellated into thin strips):
/// without them, the cut along one segment would run on past the point, nearly along the next
/// segment, leaving slivers too thin to tell which side of the surface they are on.
fn turns(t: &Tri, cuts: &[Cutter], tol: &Tol) -> Vec<Cutter> {
    // Each segment end (by its exact coordinates): the point, and per segment there its cut
    // and its other end.
    let mut ends: BTreeMap<[u64; 3], (Vec3, Vec<(usize, Vec3)>)> = BTreeMap::new();
    for (i, c) in cuts.iter().enumerate() {
        for [a, b] in &c.segs {
            for (p, q) in [(*a, *b), (*b, *a)] {
                ends.entry([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).or_insert_with(|| (p, Vec::new())).1.push((i, q));
            }
        }
    }
    let inner = |p: Vec3| t.p.iter().zip(t.p.iter().cycle().skip(1)).all(|(a, b)| t.n.cross(*b - *a).normalized().dot(p - *a) > tol.plane);
    let in_plane = |v: Vec3| (v - t.n * t.n.dot(v)).normalized();
    let mut out = Vec::new();
    for (p, meeting) in ends.into_values() {
        let [(i, u), (j, w)] = meeting.as_slice() else { continue };
        if i == j || !inner(p) {
            continue;
        }
        // Normal to the bisector of the turn, so the two segments lie on either side.
        let n = (in_plane(*w - p) - in_plane(*u - p)).normalized();
        if n != Vec3::ZERO {
            out.push(Cutter { n, d: n.dot(p), segs: vec![[p, p]], reach: Bounds3::from_points([p]).expand(tol.plane), across: Some(p) });
        }
    }
    out
}

/// What an edge of a piece lies along: side `k` of the triangle (`p[k]` to `p[k + 1]`), or a cut.
#[derive(Clone, Copy, PartialEq)]
enum Along {
    Side(usize),
    Cut(usize),
}

/// A convex piece of a triangle: its corners in order, each with the edge leaving it.
type Piece = Vec<(Vec3, Along)>;

/// Cut triangle `t` into pieces no cut crosses: each piece is cut only by the cuts whose
/// segments pass through it, so a cut doesn't spread across the whole triangle.
fn cut(t: &Tri, cuts: &[Cutter], tol: &Tol) -> Result<Vec<Vec<Vec3>>, SolidError> {
    let mut done: Vec<Vec<Vec3>> = Vec::new();
    let whole: Piece = t.p.iter().enumerate().map(|(k, p)| (*p, Along::Side(k))).collect();
    let mut stack: Vec<(Piece, Vec<usize>)> = vec![(whole, (0..cuts.len()).collect())];
    while let Some((piece, list)) = stack.pop() {
        let poly: Vec<Vec3> = piece.iter().map(|(p, _)| *p).collect();
        let near = Bounds3::from_points(poly.iter().copied()).expand(tol.plane);
        let mut halves: Option<(Piece, Piece)> = None;
        let mut rest: Vec<usize> = Vec::with_capacity(list.len());
        for i in list {
            // A cut that doesn't cross this piece can't cross its parts either: drop it.
            let Some(c) = cuts.get(i).filter(|c| c.reach.intersects(&near)) else { continue };
            if halves.is_some() {
                rest.push(i);
                continue;
            }
            let through = match c.across {
                Some(p) => holds(&poly, t.n, p),
                None => c.segs.iter().any(|s| crosses(&poly, t.n, *s)),
            };
            if !through {
                continue;
            }
            let (front, back) = split(&piece, t, cuts, i, tol.plane);
            if front.len() >= 3 && back.len() >= 3 {
                halves = Some((front, back));
            }
        }
        match halves {
            Some((front, back)) => {
                stack.push((front, rest.clone()));
                stack.push((back, rest));
            }
            None => done.push(poly),
        }
        if done.len() + stack.len() > MAX_PIECES {
            return Err(SolidError::TooComplex);
        }
    }
    Ok(done)
}

/// The two points of `pts` farthest apart.
fn farthest(pts: &[Vec3]) -> Option<[Vec3; 2]> {
    let mut best: Option<(f64, [Vec3; 2])> = None;
    for (i, a) in pts.iter().enumerate() {
        for b in pts.iter().skip(i + 1) {
            let d = a.dist(*b);
            if best.is_none_or(|(m, _)| d > m) {
                best = Some((d, [*a, *b]));
            }
        }
    }
    best.map(|(_, s)| s)
}

/// Whether `p` (in the plane of the convex polygon `poly`, unit normal `n`, counter-clockwise)
/// is inside the polygon, not on its outline.
fn holds(poly: &[Vec3], n: Vec3, p: Vec3) -> bool {
    poly.iter().zip(poly.iter().cycle().skip(1)).all(|(a, b)| n.cross(*b - *a).dot(p - *a) > 0.0)
}

/// Whether the segment `a b` (in the plane of the convex polygon `poly`, unit normal `n`,
/// counter-clockwise) passes through the inside of the polygon, not just along its outline.
fn crosses(poly: &[Vec3], n: Vec3, [a, b]: [Vec3; 2]) -> bool {
    let ab = b - a;
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    let len = poly.len();
    for (i, p) in poly.iter().enumerate() {
        let Some(q) = poly.get((i + 1) % len) else { continue };
        let m = n.cross(*q - *p).normalized();
        if m == Vec3::ZERO {
            continue;
        }
        // Inside this edge: m·(x - p) > 0, along a + λ·ab.
        let (f, df) = (m.dot(a - *p), m.dot(ab));
        if df == 0.0 {
            if f <= 0.0 {
                return false;
            }
        } else if df > 0.0 {
            lo = lo.max(-f / df);
        } else {
            hi = hi.min(-f / df);
        }
        if lo >= hi {
            return false;
        }
    }
    hi > lo
}

/// Where the edge `u w` crosses the plane `n·x = d`, computed the same way whichever way round
/// the edge comes, so the triangles on both sides of it, and the other solid (which finds the
/// point as the end of a cut), agree on it to the last bit.
fn crossing(u: Vec3, w: Vec3, n: Vec3, d: f64) -> Vec3 {
    let (u, w) = if [u.x, u.y, u.z] <= [w.x, w.y, w.z] { (u, w) } else { (w, u) };
    let (du, dw) = (n.dot(u) - d, n.dot(w) - d);
    u.lerp(w, du / (du - dw))
}

/// Split a convex piece of `t` by cut `ci` into its front and back parts (either may be empty).
/// Points within `tol` of the plane count as on it.
fn split(piece: &[(Vec3, Along)], t: &Tri, cuts: &[Cutter], ci: usize, tol: f64) -> (Piece, Piece) {
    let Some(c) = cuts.get(ci) else { return (piece.to_vec(), Vec::new()) };
    let side = |x: f64| {
        if x > tol {
            1
        } else if x < -tol {
            -1
        } else {
            0
        }
    };
    let dist: Vec<f64> = piece.iter().map(|(p, _)| c.n.dot(*p) - c.d).collect();
    if !dist.iter().any(|x| side(*x) < 0) {
        return (piece.to_vec(), Vec::new());
    }
    if !dist.iter().any(|x| side(*x) > 0) {
        return (Vec::new(), piece.to_vec());
    }
    let here = Along::Cut(ci);
    let (mut front, mut back) = (Vec::new(), Vec::new());
    let len = piece.len();
    for (i, ((p, along), dp)) in piece.iter().zip(&dist).enumerate() {
        let (Some((q, _)), Some(dq)) = (piece.get((i + 1) % len), dist.get((i + 1) % len)) else { continue };
        let (sp, sq) = (side(*dp), side(*dq));
        // From a corner on the plane, the cut leaves on the side the edge leaving it doesn't go to.
        if sp >= 0 {
            front.push((*p, if sp == 0 && sq < 0 { here } else { *along }));
        }
        if sp <= 0 {
            back.push((*p, if sp == 0 && sq > 0 { here } else { *along }));
        }
        if sp * sq < 0 {
            let x = meet(t, cuts, c, *along, (*p, *dp), (*q, *dq), tol);
            let (to, from) = if sp > 0 { (&mut front, &mut back) } else { (&mut back, &mut front) };
            to.push((x, here));
            from.push((x, *along));
        }
    }
    (front, back)
}

/// Where cut `c` crosses the edge from `p` to `q` (along `along`; `dp`, `dq` their distances
/// from the cut's plane). Where that is on a side of the triangle, or where the edge is another
/// cut whose segment ends where one of `c`'s does, it is the point both solids compute for it,
/// not one found by intersecting two lines that may be nearly parallel.
fn meet(t: &Tri, cuts: &[Cutter], c: &Cutter, along: Along, (p, dp): (Vec3, f64), (q, dq): (Vec3, f64), tol: f64) -> Vec3 {
    let x = p.lerp(q, dp / (dp - dq));
    match along {
        Along::Side(k) => {
            let (Some(u), Some(w)) = (t.p.get(k), t.p.get((k + 1) % 3)) else { return x };
            let (du, dw) = (c.n.dot(*u) - c.d, c.n.dot(*w) - c.d);
            if (du < 0.0) != (dw < 0.0) { crossing(*u, *w, c.n, c.d) } else { x }
        }
        Along::Cut(j) => {
            let Some(o) = cuts.get(j) else { return x };
            let edge = Bounds3::from_points([p, q]).expand(tol);
            let ends = |c: &Cutter| c.segs.iter().flatten().copied().collect::<Vec<Vec3>>();
            let theirs = ends(o);
            ends(c)
                .into_iter()
                .filter(|e| theirs.contains(e) && edge.intersects(&Bounds3::from_points([*e])))
                .min_by(|a, b| a.dist(x).total_cmp(&b.dist(x)))
                .unwrap_or(x)
        }
    }
}

/// Where the convex piece `poly` (of a face with unit normal `n`) lies relative to `other`, whose
/// triangles `flat` lie in the face's plane; `None` for a piece with no area.
fn classify(poly: &[Vec3], n: Vec3, other: &Solid, flat: &[&Tri], tol: &Tol) -> Option<Side> {
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
    // Off the other surface the centre tells: a step off the face could cross a surface that
    // meets a thin piece at a shallow angle.
    let on = |s: &&Tri| s.p.iter().zip(s.p.iter().cycle().skip(1)).all(|(a, b)| (*b - *a).cross(c - *a).dot(s.n) >= -tol.plane * a.dist(*b));
    if !flat.iter().any(on) {
        return Some(if inside(other, c, tol) { Side::Inside } else { Side::Outside });
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
