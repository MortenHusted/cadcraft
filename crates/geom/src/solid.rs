//! 3D solids as closed triangle meshes: the kernel behind 3DSOLID objects.
//!
//! A [`Mesh`] is an indexed triangle mesh whose triangles wind counter-clockwise seen from
//! outside, so their normals point out of the solid. [`primitives`] builds boxes, cylinders,
//! cones, spheres, tori, wedges, extrusions and revolutions; [`boolean`] combines solids (union,
//! subtraction, intersection); [`Mesh::mass_props`] measures them. Curved faces are faceted when
//! a solid is made, within [`primitives::CHORD_TOL`].
//!
//! Every operation is bounded: meshes above [`MAX_TRIANGLES`] are refused, nothing recurses, and
//! hostile meshes (indices out of range, non-finite coordinates) are skipped, never trusted.

use std::collections::HashMap;
use std::fmt;

use serde::{Deserialize, Serialize};

use crate::{Bounds3, Mat4, Vec3};

pub mod boolean;
mod bvh;
pub mod edges;
mod merge;
pub mod primitives;
pub mod printability;
pub mod skin;
mod stitch;
pub mod sweep;
mod triangulate;

pub use boolean::{BoolOp, boolean};
pub use triangulate::triangulate_polygon;

/// Most triangles in one mesh (a 3D-printable part rarely needs a tenth of this).
pub const MAX_TRIANGLES: usize = 4_000_000;

/// Dihedral angle (degrees) above which an edge between two faces is drawn in wireframe.
pub const FEATURE_ANGLE: f64 = 20.0;

/// Why a solid could not be made.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SolidError {
    /// A size, radius, profile or direction that encloses no volume.
    Degenerate(String),
    /// More triangles (above [`MAX_TRIANGLES`]), pieces of a cut face or vertices than the caps
    /// allow.
    TooComplex,
}

impl fmt::Display for SolidError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SolidError::Degenerate(m) => write!(f, "{m}"),
            SolidError::TooComplex => write!(f, "The solids are too complex to combine (too many triangles, or a face cut into too many pieces)."),
        }
    }
}

impl std::error::Error for SolidError {}

/// A closed, outward-oriented triangle mesh.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Mesh {
    pub vertices: Vec<Vec3>,
    /// Vertex indices, counter-clockwise seen from outside.
    pub triangles: Vec<[u32; 3]>,
}

/// Volume, area and inertia of a solid of unit density.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MassProps {
    pub volume: f64,
    pub area: f64,
    pub centroid: Vec3,
    /// Moments of inertia about the WCS axes: ∫(y²+z²), ∫(z²+x²), ∫(x²+y²).
    pub moments: [f64; 3],
    /// Products of inertia about the WCS axes: ∫xy, ∫yz, ∫zx.
    pub products: [f64; 3],
    /// The inertia tensor about the centroid (row-major).
    pub centroid_inertia: [[f64; 3]; 3],
}

/// How well a mesh encloses a volume: a closed solid has no open, overused or inconsistent edges.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MeshCheck {
    /// Edges used by one triangle only (holes in the surface).
    pub open_edges: usize,
    /// Edges shared by more than two triangles.
    pub nonmanifold_edges: usize,
    /// Edges whose two triangles run the same way along them (one triangle faces inward).
    pub flipped_edges: usize,
    /// Triangles with an index out of range or a repeated vertex.
    pub bad_triangles: usize,
}

impl MeshCheck {
    /// True for a watertight, consistently oriented mesh: what slicers call manifold.
    pub fn is_closed(&self) -> bool {
        *self == MeshCheck::default()
    }
}

impl Mesh {
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// The corners of triangle `t`, `None` when an index is out of range or a corner is not
    /// finite.
    pub fn corners(&self, t: &[u32; 3]) -> Option<[Vec3; 3]> {
        let v = |i: u32| self.vertices.get(i as usize).copied().filter(|p| p.is_finite());
        Some([v(t[0])?, v(t[1])?, v(t[2])?])
    }

    /// Every triangle whose indices are in range, as corner points.
    pub fn triangle_points(&self) -> impl Iterator<Item = [Vec3; 3]> + '_ {
        self.triangles.iter().filter_map(|t| self.corners(t))
    }

    pub fn bounds(&self) -> Bounds3 {
        Bounds3::from_points(self.triangle_points().flatten())
    }

    /// Apply `m`; a mirroring transform reverses the triangles so they keep facing out.
    pub fn transform(&mut self, m: &Mat4) {
        for v in &mut self.vertices {
            *v = m.apply(*v);
        }
        if m.det3() < 0.0 {
            self.flip();
        }
    }

    pub fn transformed(&self, m: &Mat4) -> Mesh {
        let mut out = self.clone();
        out.transform(m);
        out
    }

    /// Reverse every triangle (turn the solid inside out).
    pub fn flip(&mut self) {
        for t in &mut self.triangles {
            t.swap(1, 2);
        }
    }

    /// Add `other`'s triangles as a separate shell (no boolean).
    pub fn append(&mut self, other: &Mesh) -> Result<(), SolidError> {
        if self.triangles.len().saturating_add(other.triangles.len()) > MAX_TRIANGLES {
            return Err(SolidError::TooComplex);
        }
        let base = u32::try_from(self.vertices.len()).map_err(|_| SolidError::TooComplex)?;
        self.vertices.extend_from_slice(&other.vertices);
        self.triangles.extend(other.triangles.iter().map(|t| t.map(|i| i.saturating_add(base))));
        Ok(())
    }

    /// Signed volume enclosed (positive for an outward-facing closed mesh).
    pub fn volume(&self) -> f64 {
        self.triangle_points().map(|[a, b, c]| a.dot(b.cross(c))).sum::<f64>() / 6.0
    }

    pub fn area(&self) -> f64 {
        self.triangle_points().map(|[a, b, c]| (b - a).cross(c - a).len()).sum::<f64>() / 2.0
    }

    /// Reverse the triangles when the mesh encloses negative volume (built inside out).
    pub fn orient_outward(&mut self) {
        if self.volume() < 0.0 {
            self.flip();
        }
    }

    /// Mass properties for unit density (Eberly, "Polyhedral Mass Properties (Revisited)").
    pub fn mass_props(&self) -> MassProps {
        // ∫1, ∫x, ∫y, ∫z, ∫x², ∫y², ∫z², ∫xy, ∫yz, ∫zx
        let mut s = [0.0f64; 10];
        let sub = |w0: f64, w1: f64, w2: f64| {
            let t0 = w0 + w1;
            let f1 = t0 + w2;
            let t1 = w0 * w0;
            let t2 = t1 + w1 * t0;
            let f2 = t2 + w2 * f1;
            let f3 = w0 * t1 + w1 * t2 + w2 * f2;
            (f1, f2, f3, f2 + w0 * (f1 + w0), f2 + w1 * (f1 + w1), f2 + w2 * (f1 + w2))
        };
        let mut area = 0.0;
        for [p0, p1, p2] in self.triangle_points() {
            let (e1, e2) = (p1 - p0, p2 - p0);
            let d = e1.cross(e2);
            area += d.len() / 2.0;
            let (f1x, f2x, f3x, g0x, g1x, g2x) = sub(p0.x, p1.x, p2.x);
            let (_, f2y, f3y, g0y, g1y, g2y) = sub(p0.y, p1.y, p2.y);
            let (_, f2z, f3z, g0z, g1z, g2z) = sub(p0.z, p1.z, p2.z);
            s[0] += d.x * f1x;
            s[1] += d.x * f2x;
            s[2] += d.y * f2y;
            s[3] += d.z * f2z;
            s[4] += d.x * f3x;
            s[5] += d.y * f3y;
            s[6] += d.z * f3z;
            s[7] += d.x * (p0.y * g0x + p1.y * g1x + p2.y * g2x);
            s[8] += d.y * (p0.z * g0y + p1.z * g1y + p2.z * g2y);
            s[9] += d.z * (p0.x * g0z + p1.x * g1z + p2.x * g2z);
        }
        let scale = [1.0 / 6.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 60.0, 1.0 / 60.0, 1.0 / 60.0, 1.0 / 120.0, 1.0 / 120.0, 1.0 / 120.0];
        for (v, k) in s.iter_mut().zip(scale) {
            *v *= k;
        }
        let [vol, sx, sy, sz, xx, yy, zz, xy, yz, zx] = s;
        let c = if vol.abs() > 0.0 { Vec3::new(sx / vol, sy / vol, sz / vol) } else { Vec3::ZERO };
        let ixx = yy + zz - vol * (c.y * c.y + c.z * c.z);
        let iyy = zz + xx - vol * (c.z * c.z + c.x * c.x);
        let izz = xx + yy - vol * (c.x * c.x + c.y * c.y);
        let ixy = -(xy - vol * c.x * c.y);
        let iyz = -(yz - vol * c.y * c.z);
        let izx = -(zx - vol * c.z * c.x);
        MassProps {
            volume: vol,
            area,
            centroid: c,
            moments: [yy + zz, zz + xx, xx + yy],
            products: [xy, yz, zx],
            centroid_inertia: [[ixx, ixy, izx], [ixy, iyy, iyz], [izx, iyz, izz]],
        }
    }

    /// Count the edges that keep the mesh from enclosing a volume.
    pub fn check(&self) -> MeshCheck {
        let mut out = MeshCheck::default();
        // Per undirected edge: uses running low→high, uses running high→low.
        let mut edges: HashMap<(u32, u32), (u32, u32)> = HashMap::new();
        for t in &self.triangles {
            let in_range = t.iter().all(|i| (*i as usize) < self.vertices.len());
            if !in_range || t[0] == t[1] || t[1] == t[2] || t[2] == t[0] {
                out.bad_triangles += 1;
                continue;
            }
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let e = edges.entry((a.min(b), a.max(b))).or_default();
                if a < b {
                    e.0 += 1;
                } else {
                    e.1 += 1;
                }
            }
        }
        for (up, down) in edges.into_values() {
            match (up, down) {
                (1, 1) => {}
                (1, 0) | (0, 1) => out.open_edges += 1,
                (2, 0) | (0, 2) => out.flipped_edges += 1,
                _ => out.nonmanifold_edges += 1,
            }
        }
        out
    }

    /// The edges a wireframe shows: where the faces meet at more than `angle_deg`, open edges,
    /// and the outline seen along `view` (where the surface turns from facing the viewer to
    /// facing away; a zero `view` gives no outline).
    pub fn wire_edges(&self, angle_deg: f64, view: Vec3) -> Vec<[Vec3; 2]> {
        let mut out: Vec<[Vec3; 2]> = self
            .shown_edges(angle_deg, view)
            .into_iter()
            .filter_map(|(a, b)| Some([*self.vertices.get(a as usize)?, *self.vertices.get(b as usize)?]))
            .collect();
        out.sort_by(|p, q| {
            let key = |e: &[Vec3; 2]| [e[0].x, e[0].y, e[0].z, e[1].x, e[1].y, e[1].z];
            key(p).iter().zip(key(q)).map(|(a, b)| a.total_cmp(&b)).find(|o| o.is_ne()).unwrap_or(std::cmp::Ordering::Equal)
        });
        out
    }

    /// [`Mesh::wire_edges`] as vertex index pairs, lower index first, sorted.
    pub fn shown_edges(&self, angle_deg: f64, view: Vec3) -> Vec<(u32, u32)> {
        let normal = |t: &[u32; 3]| self.corners(t).map(|[a, b, c]| (b - a).cross(c - a).normalized());
        let mut faces: HashMap<(u32, u32), Vec<Vec3>> = HashMap::new();
        for t in &self.triangles {
            let Some(n) = normal(t) else { continue };
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                faces.entry((a.min(b), a.max(b))).or_default().push(n);
            }
        }
        let cos_limit = angle_deg.to_radians().cos();
        let view = view.normalized();
        let outline = |n1: &Vec3, n2: &Vec3| view != Vec3::ZERO && (n1.dot(view) > 0.0) != (n2.dot(view) > 0.0);
        let mut out: Vec<(u32, u32)> = faces
            .into_iter()
            .filter(|(_, ns)| match ns.as_slice() {
                [n1, n2] => n1.dot(*n2) < cos_limit || outline(n1, n2),
                _ => true,
            })
            .map(|(e, _)| e)
            .collect();
        // HashMap order varies from run to run; sort so drawings and files are reproducible.
        out.sort_unstable();
        out
    }

    /// The mesh without triangles that index past the vertices, repeat a vertex or have
    /// non-finite corners, and capped at [`MAX_TRIANGLES`]: safe for any later operation.
    pub fn sanitized(mut self) -> Mesh {
        let ok = |t: &[u32; 3]| {
            t[0] != t[1] && t[1] != t[2] && t[2] != t[0] && t.iter().all(|i| self.vertices.get(*i as usize).is_some_and(|v| v.is_finite()))
        };
        let mut tris: Vec<[u32; 3]> = self.triangles.iter().filter(|t| ok(t)).copied().collect();
        tris.truncate(MAX_TRIANGLES);
        self.triangles = tris;
        self
    }
}

#[cfg(test)]
mod tests;
