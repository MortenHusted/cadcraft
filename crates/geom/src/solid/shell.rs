//! Hollowing a solid into walls of one thickness (SOLIDEDIT Shell), splitting a solid into its
//! separate pieces (Separate), and the flat faces they work on.
//!
//! The inside wall is the solid shrunk by the wall thickness, turned inside out: it keeps the
//! solid's triangles, each vertex moved to where the planes of the faces round it, each moved in
//! by the thickness, best meet (faces meeting at less than 25° count as one, so a cylinder or
//! sphere shrinks along its surface normal). The faces to open are left out of both walls, the
//! inside wall's corners there staying in their planes, and a flat rim joins the two walls round
//! each opening. No boolean is needed, so curved walls stay exact.

use std::collections::{HashMap, HashSet};

use super::bvh::Bvh;
use super::primitives::MAX_COORD;
use super::{Mesh, SolidError};
use crate::{Bounds3, Vec3};

/// Faces meeting at less than this (degrees) shrink as one.
const SMOOTH: f64 = 25.0;

fn degenerate(m: &str) -> SolidError {
    SolidError::Degenerate(m.to_string())
}

fn unit_normal(m: &Mesh, t: &[u32; 3]) -> Option<Vec3> {
    let [a, b, c] = m.corners(t)?;
    let n = (b - a).cross(c - a);
    (n.len() > 0.0).then(|| n.normalized())
}

/// The flat faces of `m`: triangles joined across shared edges while they lie in one plane.
pub fn faces(m: &Mesh) -> Vec<Vec<usize>> {
    let normals: Vec<Option<Vec3>> = m.triangles.iter().map(|t| unit_normal(m, t)).collect();
    let mut along: HashMap<(u32, u32), usize> = HashMap::new();
    for (i, t) in m.triangles.iter().enumerate() {
        for k in 0..3 {
            along.insert((t[k], t[(k + 1) % 3]), i);
        }
    }
    let mut face = vec![usize::MAX; m.triangles.len()];
    let mut out: Vec<Vec<usize>> = Vec::new();
    for seed in 0..m.triangles.len() {
        if face.get(seed) != Some(&usize::MAX) {
            continue;
        }
        let id = out.len();
        let mut members = vec![seed];
        if let Some(f) = face.get_mut(seed) {
            *f = id;
        }
        let mut stack = vec![seed];
        while let Some(i) = stack.pop() {
            let (Some(t), Some(Some(n))) = (m.triangles.get(i), normals.get(i)) else { continue };
            for k in 0..3 {
                let Some(&j) = along.get(&(t[(k + 1) % 3], t[k])) else { continue };
                let flat = normals.get(j).copied().flatten().is_some_and(|nj| nj.dot(*n) > 1.0 - 1e-9);
                if flat && face.get(j) == Some(&usize::MAX) {
                    if let Some(f) = face.get_mut(j) {
                        *f = id;
                    }
                    members.push(j);
                    stack.push(j);
                }
            }
        }
        out.push(members);
    }
    out
}

/// The offset `d` from a vertex that best puts it at height `target` above each plane through
/// it with unit normal `n`: exactly for up to three independent planes, by least squares for
/// more; along directions no plane pins down (the planes nearly parallel) it stays put.
fn best_offset(planes: &[(Vec3, f64)]) -> Vec3 {
    match planes {
        [] => Vec3::ZERO,
        [(n, b)] => *n * *b,
        [(n1, b1), (n2, b2)] => {
            // In the span of the two normals: d = x·n1 + y·n2.
            let c = n1.dot(*n2);
            let det = 1.0 - c * c;
            if det < 1e-9 {
                return (*n1 + *n2).normalized() * ((b1 + b2) / 2.0);
            }
            *n1 * ((b1 - c * b2) / det) + *n2 * ((b2 - c * b1) / det)
        }
        _ => {
            let solve = |damping: f64| {
                let mut cols = [Vec3::new(damping, 0.0, 0.0), Vec3::new(0.0, damping, 0.0), Vec3::new(0.0, 0.0, damping)];
                let mut rhs = Vec3::ZERO;
                for (n, b) in planes {
                    cols[0] = cols[0] + *n * n.x;
                    cols[1] = cols[1] + *n * n.y;
                    cols[2] = cols[2] + *n * n.z;
                    rhs = rhs + *n * *b;
                }
                let [c0, c1, c2] = cols;
                let det = c0.dot(c1.cross(c2));
                // Cramer's rule, column by column.
                (det.abs() > 1e-6).then(|| Vec3::new(rhs.dot(c1.cross(c2)) / det, c0.dot(rhs.cross(c2)) / det, c0.dot(c1.cross(rhs)) / det))
            };
            solve(0.0).or_else(|| solve(1e-3)).unwrap_or(Vec3::ZERO)
        }
    }
}

/// A plane: unit normal `n` and offset `d` (`n·x = d`).
#[derive(Clone, Copy)]
struct Plane {
    n: Vec3,
    d: f64,
}

/// Builds the walls' mesh, merging points with the same coordinates.
#[derive(Default)]
struct Walls {
    mesh: Mesh,
    index: HashMap<[u64; 3], u32>,
}

impl Walls {
    fn point(&mut self, p: Vec3) -> u32 {
        let key = [p.x + 0.0, p.y + 0.0, p.z + 0.0].map(f64::to_bits);
        let next = u32::try_from(self.mesh.vertices.len()).unwrap_or(u32::MAX);
        let i = *self.index.entry(key).or_insert(next);
        if i == next {
            self.mesh.vertices.push(p);
        }
        i
    }
    fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3) {
        let t = [self.point(a), self.point(b), self.point(c)];
        if t[0] != t[1] && t[1] != t[2] && t[2] != t[0] {
            self.mesh.triangles.push(t);
        }
    }
}

/// Where the segment from `p` to `q` crosses `plane`, computed the same whichever way round the
/// segment comes, so neighbouring triangles share the point exactly.
fn crossing(p: Vec3, q: Vec3, plane: Plane) -> Vec3 {
    let key = |v: Vec3| [v.x, v.y, v.z].map(f64::to_bits);
    let (a, b) = if key(p) <= key(q) { (p, q) } else { (q, p) };
    let t = (plane.d - plane.n.dot(a)) / plane.n.dot(b - a);
    a + (b - a) * t
}

/// The part of the convex polygon `poly` on or below every plane of `planes`.
fn clip(mut poly: Vec<Vec3>, planes: &[Plane], tol: f64) -> Vec<Vec3> {
    for plane in planes {
        let above = |p: &Vec3| plane.n.dot(*p) - plane.d > tol;
        if !poly.iter().any(above) {
            continue;
        }
        let mut out = Vec::with_capacity(poly.len() + 1);
        for (i, p) in poly.iter().enumerate() {
            let q = poly.get((i + 1) % poly.len()).copied().unwrap_or(*p);
            let (sp, sq) = (plane.n.dot(*p) - plane.d, plane.n.dot(q) - plane.d);
            if sp <= tol {
                out.push(*p);
            }
            if (sp > tol && sq < -tol) || (sp < -tol && sq > tol) {
                out.push(crossing(*p, q, *plane));
            }
        }
        poly = out;
    }
    poly
}

/// `m` hollowed to walls `thickness` thick, the faces holding the triangles `open` removed so
/// the hollow opens through them.
pub fn shell(m: &Mesh, thickness: f64, open: &[usize]) -> Result<Mesh, SolidError> {
    if !(thickness.is_finite() && thickness > 0.0 && thickness <= MAX_COORD) {
        return Err(degenerate("The wall thickness must be a positive number."));
    }
    let b = m.bounds();
    let tol = 1e-9 * b.min.dist(b.max).max(1.0);
    let open: HashSet<usize> = open.iter().copied().collect();
    let normals: Vec<Option<Vec3>> = m.triangles.iter().map(|t| unit_normal(m, t)).collect();
    // The planes of the faces to open.
    let mut planes: Vec<Plane> = Vec::new();
    for i in &open {
        let (Some(t), Some(Some(n))) = (m.triangles.get(*i), normals.get(*i)) else { continue };
        let Some([a, ..]) = m.corners(t) else { continue };
        let plane = Plane { n: *n, d: n.dot(a) };
        if !planes.iter().any(|q| q.n.dot(plane.n) > 1.0 - 1e-9 && (q.d - plane.d).abs() <= tol) {
            planes.push(plane);
        }
    }
    // The planes round each vertex: (normal, open?).
    let mut around: Vec<Vec<(Vec3, bool)>> = vec![Vec::new(); m.vertices.len()];
    for (i, (t, n)) in m.triangles.iter().zip(&normals).enumerate() {
        let Some(n) = n else { continue };
        for v in t {
            if let Some(list) = around.get_mut(*v as usize) {
                list.push((*n, open.contains(&i)));
            }
        }
    }
    let cos_smooth = crate::rad(SMOOTH).cos();
    let mut inner = m.vertices.clone();
    for (v, list) in inner.iter_mut().zip(&around) {
        // Group nearly parallel faces; the faces kept move in by the thickness, open ones stay.
        let mut groups: Vec<(Vec3, bool)> = Vec::new();
        for (n, is_open) in list {
            match groups.iter_mut().find(|(g, o)| o == is_open && g.normalized().dot(*n) > cos_smooth) {
                Some((g, _)) => *g = *g + *n,
                None => groups.push((*n, *is_open)),
            }
        }
        let planes: Vec<(Vec3, f64)> = groups.iter().map(|(g, o)| (g.normalized(), if *o { 0.0 } else { -thickness })).collect();
        *v = *v + best_offset(&planes);
    }
    // The shrunken inside must stay inside: every corner that moved in lies within the solid,
    // or beyond an opening, where it is cut off.
    let tris: Vec<[Vec3; 3]> = m.triangle_points().collect();
    let bvh = Bvh::new(&tris.iter().map(|t| Bounds3::from_points(t.iter().copied())).collect::<Vec<_>>());
    let beyond = |p: Vec3| planes.iter().any(|q| q.n.dot(p) - q.d > tol);
    let moved = inner.iter().zip(&around).filter(|(_, list)| !list.is_empty() && list.iter().all(|(_, o)| !o));
    if moved.clone().any(|(p, _)| !p.is_finite() || !(beyond(*p) || inside(&tris, &bvh, *p))) {
        return Err(degenerate("The wall is too thick for this solid."));
    }
    // The walls: the kept faces, the same faces shrunk, cut off at the openings and turned inside
    // out, and a flat rim filling each opening's plane between the two.
    let mut w = Walls::default();
    let at = |list: &[Vec3], i: u32| list.get(i as usize).copied().unwrap_or(Vec3::ZERO);
    for (i, t) in m.triangles.iter().enumerate() {
        if open.contains(&i) {
            continue;
        }
        w.tri(at(&m.vertices, t[0]), at(&m.vertices, t[1]), at(&m.vertices, t[2]));
        let poly = clip(vec![at(&inner, t[0]), at(&inner, t[2]), at(&inner, t[1])], &planes, tol);
        for k in 1..poly.len().saturating_sub(1) {
            if let (Some(a), Some(b), Some(c)) = (poly.first(), poly.get(k), poly.get(k + 1)) {
                w.tri(*a, *b, *c);
            }
        }
    }
    // Edges used one way only bound the openings; each lies in an opening's plane.
    let mut edges: HashSet<(u32, u32)> = HashSet::new();
    for t in &w.mesh.triangles {
        edges.extend([(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]);
    }
    let open_edges: Vec<(u32, u32)> = edges.iter().filter(|(a, b)| !edges.contains(&(*b, *a))).copied().collect();
    for plane in &planes {
        let on = |v: u32| (plane.n.dot(at(&w.mesh.vertices, v)) - plane.d).abs() <= tol * 1e3;
        // The rim runs against each edge, so it follows them backwards.
        let mut next: HashMap<u32, u32> = HashMap::new();
        for (a, b) in open_edges.iter().filter(|(a, b)| on(*a) && on(*b)) {
            next.insert(*b, *a);
        }
        let mut loops: Vec<Vec<u32>> = Vec::new();
        let mut used: HashSet<u32> = HashSet::new();
        let mut starts: Vec<u32> = next.keys().copied().collect();
        starts.sort_unstable();
        for s in starts {
            if used.contains(&s) {
                continue;
            }
            let mut l = vec![s];
            used.insert(s);
            let mut v = s;
            while let Some(&u) = next.get(&v) {
                if u == s || l.len() > next.len() {
                    break;
                }
                if !used.insert(u) {
                    return Err(degenerate("The opening's edge crosses itself."));
                }
                l.push(u);
                v = u;
            }
            loops.push(l);
        }
        let (u, v) = {
            let a = if plane.n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
            let u = a.cross(plane.n).normalized();
            (u, plane.n.cross(u))
        };
        let flat: Vec<Vec<crate::Vec2>> =
            loops.iter().map(|l| l.iter().map(|i| at(&w.mesh.vertices, *i)).map(|p| crate::Vec2::new(p.dot(u), p.dot(v))).collect()).collect();
        let ids: Vec<u32> = loops.iter().flatten().copied().collect();
        for [a, b, c] in super::primitives::cap(&flat)? {
            let id = |k: usize| ids.get(k).copied().unwrap_or(0);
            if id(a) != id(b) && id(b) != id(c) && id(c) != id(a) {
                w.mesh.triangles.push([id(a), id(b), id(c)]);
            }
        }
    }
    let out = w.mesh;
    if !out.check().is_closed() || !(out.volume() > 0.0 && out.volume() < m.volume()) {
        return Err(degenerate("The wall is too thick for this solid (its inside folds over itself)."));
    }
    Ok(out)
}

/// Whether `p` is inside the closed surface of `tris` (indexed by `bvh`): a ray from it crosses
/// the surface an odd number of times.
fn inside(tris: &[[Vec3; 3]], bvh: &Bvh, p: Vec3) -> bool {
    // A direction no face of a drawing is likely to line up with.
    let dir = Vec3::new(0.5773, 0.5774, 0.5775).normalized();
    let mut crossings = 0;
    bvh.along(p, dir, 0.0, f64::INFINITY, |i| {
        let Some([a, b, c]) = tris.get(i as usize) else { return };
        let (e1, e2) = (*b - *a, *c - *a);
        let h = dir.cross(e2);
        let det = e1.dot(h);
        if det.abs() < 1e-300 {
            return;
        }
        let s = p - *a;
        let u = s.dot(h) / det;
        let q = s.cross(e1);
        let w = dir.dot(q) / det;
        if u >= 0.0 && w >= 0.0 && u + w <= 1.0 && e2.dot(q) / det > 0.0 {
            crossings += 1;
        }
    });
    crossings % 2 == 1
}

/// The separate pieces of `m`: each closed shell enclosing material, with the hollows inside it.
pub fn separate(m: &Mesh) -> Vec<Mesh> {
    // Shells: triangles joined through shared vertices.
    let mut parent: Vec<usize> = (0..m.vertices.len()).collect();
    fn root(parent: &mut [usize], mut x: usize) -> usize {
        while let Some(&p) = parent.get(x) {
            if p == x {
                break;
            }
            let gp = parent.get(p).copied().unwrap_or(p);
            if let Some(slot) = parent.get_mut(x) {
                *slot = gp;
            }
            x = gp;
        }
        x
    }
    for t in &m.triangles {
        let (a, b, c) = (root(&mut parent, t[0] as usize), root(&mut parent, t[1] as usize), root(&mut parent, t[2] as usize));
        for x in [b, c] {
            if let Some(slot) = parent.get_mut(x) {
                *slot = a;
            }
        }
    }
    let mut shells: HashMap<usize, Mesh> = HashMap::new();
    let mut remap: HashMap<(usize, u32), u32> = HashMap::new();
    for t in &m.triangles {
        let Some(corners) = m.corners(t) else { continue };
        let r = root(&mut parent, t[0] as usize);
        let shell = shells.entry(r).or_default();
        let mut tri = [0u32; 3];
        for (k, (v, p)) in t.iter().zip(corners).enumerate() {
            let next = u32::try_from(shell.vertices.len()).unwrap_or(u32::MAX);
            let id = *remap.entry((r, *v)).or_insert(next);
            if id == next {
                shell.vertices.push(p);
            }
            if let Some(slot) = tri.get_mut(k) {
                *slot = id;
            }
        }
        shell.triangles.push(tri);
    }
    let mut list: Vec<(usize, Mesh)> = shells.into_iter().collect();
    // HashMap order varies from run to run; keep the pieces in the order of their first vertex.
    list.sort_by_key(|(r, _)| *r);
    let (mut bodies, holes): (Vec<Mesh>, Vec<Mesh>) = list.into_iter().map(|(_, s)| s).partition(|s| s.volume() > 0.0);
    // Each hollow belongs to the smallest piece round it.
    for hole in holes {
        let Some(probe) = hole.vertices.first().copied() else { continue };
        let home = bodies
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                let tris: Vec<[Vec3; 3]> = b.triangle_points().collect();
                let bvh = Bvh::new(&tris.iter().map(|t| Bounds3::from_points(t.iter().copied())).collect::<Vec<_>>());
                inside(&tris, &bvh, probe)
            })
            .min_by(|a, b| a.1.volume().total_cmp(&b.1.volume()))
            .map(|(i, _)| i);
        if let Some(body) = home.and_then(|i| bodies.get_mut(i)) {
            let _ = body.append(&hole);
        }
    }
    bodies
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::primitives::{cuboid, frustum, sphere};

    fn near(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "{a} vs {b}");
    }

    #[test]
    fn a_box_has_six_faces_and_shells_into_a_closed_hollow_or_an_open_tray() {
        let m = cuboid(Vec3::ZERO, Vec3::new(10.0, 8.0, 6.0)).unwrap();
        let fs = faces(&m);
        assert_eq!(fs.len(), 6);
        // Closed: a hollow 8 × 6 × 4 inside.
        let hollow = shell(&m, 1.0, &[]).unwrap();
        assert!(hollow.check().is_closed());
        near(hollow.volume(), 480.0 - 192.0, 1e-9);
        // The top removed: a tray, its hollow 8 × 6 × 5.
        let top: Vec<usize> = fs.iter().find(|f| f.iter().all(|i| m.corners(&m.triangles[*i]).unwrap().iter().all(|p| p.z == 6.0))).unwrap().clone();
        let tray = shell(&m, 1.0, &top).unwrap();
        assert!(tray.check().is_closed());
        near(tray.volume(), 480.0 - 240.0, 1e-9);
        assert_eq!(separate(&tray).len(), 1);
        assert!(shell(&m, 4.0, &[]).is_err());
        assert!(shell(&m, f64::NAN, &[]).is_err());
    }

    #[test]
    fn round_solids_shell_along_their_surface() {
        let ball = sphere(Vec3::ZERO, 10.0, 64).unwrap();
        let s = shell(&ball, 1.0, &[]).unwrap();
        assert!(s.check().is_closed());
        // About the volume between spheres of 10 and 9.
        let exact = ball.volume() * (1.0 - 0.729);
        near(s.volume(), exact, 0.02 * exact);
        // A cup: a cylinder opened at the top.
        let can = frustum(Vec3::ZERO, 5.0, 5.0, 10.0, 64).unwrap();
        let top: Vec<usize> = (0..can.triangles.len()).filter(|i| can.corners(&can.triangles[*i]).unwrap().iter().all(|p| p.z == 10.0)).collect();
        let cup = shell(&can, 0.5, &top).unwrap();
        assert!(cup.check().is_closed());
        let wall = can.volume() - frustum(Vec3::new(0.0, 0.0, 0.5), 4.5, 4.5, 9.5, 64).unwrap().volume();
        near(cup.volume(), wall, 0.01 * wall);
    }

    #[test]
    fn separate_splits_pieces_and_keeps_hollows_with_theirs() {
        let a = cuboid(Vec3::ZERO, Vec3::new(4.0, 4.0, 4.0)).unwrap();
        let mut both = shell(&a, 1.0, &[]).unwrap();
        both.append(&cuboid(Vec3::new(10.0, 0.0, 0.0), Vec3::new(12.0, 2.0, 2.0)).unwrap()).unwrap();
        let parts = separate(&both);
        assert_eq!(parts.len(), 2);
        let mut volumes: Vec<f64> = parts.iter().map(Mesh::volume).collect();
        volumes.sort_by(f64::total_cmp);
        near(volumes[0], 8.0, 1e-9);
        near(volumes[1], 64.0 - 8.0, 1e-9);
        assert!(parts.iter().all(|p| p.check().is_closed()));
    }
}
