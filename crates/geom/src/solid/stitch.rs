//! Weld a soup of planar polygons into one triangle mesh: merge points closer than the weld
//! tolerance, insert into each edge the points other polygons put on it (T-junctions, which
//! would leave cracks), then triangulate each polygon in its own plane.

use std::collections::HashMap;

use super::bvh::Bvh;
use super::triangulate::triangulate_polygon;
use super::{MAX_TRIANGLES, Mesh, SolidError};
use crate::{Bounds3, Vec2, Vec3};

/// Merges points closer than a tolerance, through a grid of cells twice that size.
struct Welder {
    tol: f64,
    vertices: Vec<Vec3>,
    cells: HashMap<[i64; 3], Vec<u32>>,
}

impl Welder {
    fn new(tol: f64) -> Welder {
        Welder { tol, vertices: Vec::new(), cells: HashMap::new() }
    }
    fn cell(&self, p: Vec3) -> [i64; 3] {
        let c = self.tol * 2.0;
        [(p.x / c).floor() as i64, (p.y / c).floor() as i64, (p.z / c).floor() as i64]
    }
    fn weld(&mut self, p: Vec3) -> Result<u32, SolidError> {
        let [x, y, z] = self.cell(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let near = self.cells.get(&[x.saturating_add(dx), y.saturating_add(dy), z.saturating_add(dz)]);
                    let hit = near.into_iter().flatten().find(|i| self.vertices.get(**i as usize).is_some_and(|q| q.dist(p) <= self.tol));
                    if let Some(i) = hit {
                        return Ok(*i);
                    }
                }
            }
        }
        let i = u32::try_from(self.vertices.len()).map_err(|_| SolidError::TooComplex)?;
        self.vertices.push(p);
        self.cells.entry([x, y, z]).or_default().push(i);
        Ok(i)
    }
}

/// Weld the points of `polys` and number each polygon's corners, dropping repeats and
/// polygons left with fewer than three corners.
pub(crate) fn weld_rings(polys: &[Vec<Vec3>], tol: f64) -> Result<(Vec<Vec3>, Vec<Vec<u32>>), SolidError> {
    let mut w = Welder::new(tol);
    let mut rings: Vec<Vec<u32>> = Vec::with_capacity(polys.len());
    for poly in polys {
        let mut r: Vec<u32> = Vec::with_capacity(poly.len());
        for p in poly {
            let i = w.weld(*p)?;
            if r.last() != Some(&i) {
                r.push(i);
            }
        }
        while r.len() > 1 && r.first() == r.last() {
            r.pop();
        }
        if r.len() >= 3 {
            rings.push(r);
        }
    }
    Ok((w.vertices, rings))
}

/// Insert into each ring's edges the vertices that lie on them (within `tol`), in order.
pub(crate) fn split_t_junctions(verts: &[Vec3], rings: &mut [Vec<u32>], tol: f64) {
    let at = |i: u32| verts.get(i as usize).copied().unwrap_or(Vec3::ZERO);
    let boxes: Vec<Bounds3> = verts.iter().map(|p| Bounds3::from_points([*p]).expand(tol * 2.0)).collect();
    let bvh = Bvh::new(&boxes);
    let mut on_edge: Vec<(f64, u32)> = Vec::new();
    for r in rings.iter_mut() {
        let mut full: Vec<u32> = Vec::with_capacity(r.len());
        for (k, &a) in r.iter().enumerate() {
            let Some(&b) = r.get((k + 1) % r.len()) else { continue };
            full.push(a);
            let (pa, pb) = (at(a), at(b));
            let len = pa.dist(pb);
            if len <= tol {
                continue;
            }
            let dir = (pb - pa) * (1.0 / len);
            on_edge.clear();
            bvh.along(pa, dir, -tol, len + tol, |i| {
                // A corner of this polygon on its own edge would fold the polygon onto itself.
                if r.contains(&i) {
                    return;
                }
                let q = at(i) - pa;
                let t = q.dot(dir);
                if t > tol && t < len - tol && (q - dir * t).len() <= tol {
                    on_edge.push((t, i));
                }
            });
            on_edge.sort_by(|x, y| x.0.total_cmp(&y.0));
            full.extend(on_edge.iter().map(|(_, i)| *i));
        }
        full.dedup();
        while full.len() > 1 && full.first() == full.last() {
            full.pop();
        }
        *r = full;
    }
}

/// One closed mesh from `polys` (each planar, convex or not, wound outward).
pub(crate) fn stitch(polys: &[Vec<Vec3>], tol: f64) -> Result<Mesh, SolidError> {
    let (verts, mut rings) = weld_rings(polys, tol)?;
    split_t_junctions(&verts, &mut rings, tol);
    let at = |i: u32| verts.get(i as usize).copied().unwrap_or(Vec3::ZERO);
    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(rings.len() * 2);
    for r in &rings {
        triangulate_ring(r, &at, &mut triangles);
        if triangles.len() > MAX_TRIANGLES {
            return Err(SolidError::TooComplex);
        }
    }
    Ok(compact(verts, triangles))
}

/// Triangulate a planar ring of vertex indices in the plane it lies in, keeping its winding.
fn triangulate_ring(ring: &[u32], at: &impl Fn(u32) -> Vec3, out: &mut Vec<[u32; 3]>) {
    if let [a, b, c] = ring {
        out.push([*a, *b, *c]);
        return;
    }
    if ring.len() < 3 {
        return;
    }
    // Newell's normal; project along its largest component.
    let len = ring.len();
    let n = ring.iter().enumerate().fold(Vec3::ZERO, |n, (k, &i)| {
        let (p, q) = (at(i), at(ring.get((k + 1) % len).copied().unwrap_or(i)));
        n + Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y))
    });
    let (ax, ay, az) = (n.x.abs(), n.y.abs(), n.z.abs());
    let flat: Vec<Vec2> = ring
        .iter()
        .map(|&i| {
            let p = at(i);
            if az >= ax && az >= ay {
                Vec2::new(p.x, p.y)
            } else if ax >= ay {
                Vec2::new(p.y, p.z)
            } else {
                Vec2::new(p.z, p.x)
            }
        })
        .collect();
    for t in triangulate_polygon(&flat) {
        if let (Some(a), Some(b), Some(c)) = (ring.get(t[0]), ring.get(t[1]), ring.get(t[2])) {
            out.push([*a, *b, *c]);
        }
    }
}

/// Drop vertices no triangle uses and renumber.
fn compact(verts: Vec<Vec3>, triangles: Vec<[u32; 3]>) -> Mesh {
    let mut map: Vec<Option<u32>> = vec![None; verts.len()];
    let mut out = Mesh::default();
    for t in triangles {
        let mut tri = [0u32; 3];
        for (slot, i) in tri.iter_mut().zip(t) {
            let Some(m) = map.get_mut(i as usize) else { continue };
            *slot = match *m {
                Some(j) => j,
                None => {
                    let j = u32::try_from(out.vertices.len()).unwrap_or(u32::MAX);
                    out.vertices.push(verts.get(i as usize).copied().unwrap_or(Vec3::ZERO));
                    *m = Some(j);
                    j
                }
            };
        }
        out.triangles.push(tri);
    }
    out
}
