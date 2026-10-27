//! Reading STL (binary or ASCII) into a 3D solid, so parts made elsewhere can be opened,
//! imported, combined and printed again.
//!
//! STL repeats each corner in every triangle that uses it; corners with the same coordinates are
//! merged into one vertex, so a watertight file becomes a closed solid. The facet normals are
//! ignored (the winding says which way a triangle faces).

use std::collections::HashMap;

use cadcraft_doc::{Common, Drawing, EntityKind, Solid3d, Space};
use cadcraft_geom::Vec3;
use cadcraft_geom::solid::{MAX_TRIANGLES, Mesh};

use crate::{IoError, Result};

/// Merges corners with bit-identical coordinates.
#[derive(Default)]
struct Corners {
    mesh: Mesh,
    index: HashMap<[u64; 3], u32>,
}

impl Corners {
    fn corner(&mut self, p: Vec3) -> Result<u32> {
        if !p.is_finite() {
            return Err(IoError::Format("the STL file has a corner that is not a finite number".into()));
        }
        // +0.0 and -0.0 are the same corner.
        let key = [p.x + 0.0, p.y + 0.0, p.z + 0.0].map(f64::to_bits);
        if let Some(i) = self.index.get(&key) {
            return Ok(*i);
        }
        let i = u32::try_from(self.mesh.vertices.len()).map_err(|_| too_big())?;
        self.mesh.vertices.push(p);
        self.index.insert(key, i);
        Ok(i)
    }
    fn triangle(&mut self, [a, b, c]: [Vec3; 3]) -> Result<()> {
        if self.mesh.triangles.len() >= MAX_TRIANGLES {
            return Err(too_big());
        }
        let t = [self.corner(a)?, self.corner(b)?, self.corner(c)?];
        // A triangle with a repeated corner encloses nothing.
        if t[0] != t[1] && t[1] != t[2] && t[2] != t[0] {
            self.mesh.triangles.push(t);
        }
        Ok(())
    }
}

fn too_big() -> IoError {
    IoError::Format(format!("the STL file has more than {MAX_TRIANGLES} triangles"))
}

/// Whether `bytes` are a binary STL: the triangle count after the 80-byte header matches the
/// length exactly (an ASCII file starting with "solid" can't).
fn is_binary(bytes: &[u8]) -> bool {
    let Some(n) = bytes.get(80..84).and_then(|b| <[u8; 4]>::try_from(b).ok()).map(u32::from_le_bytes) else { return false };
    usize::try_from(n).ok().and_then(|n| n.checked_mul(50)).and_then(|n| n.checked_add(84)) == Some(bytes.len())
}

/// The mesh of an STL file.
pub fn mesh(bytes: &[u8]) -> Result<Mesh> {
    let mut c = Corners::default();
    if is_binary(bytes) {
        let f = |b: &[u8], at: usize| b.get(at..at + 4).and_then(|x| <[u8; 4]>::try_from(x).ok()).map(|x| f64::from(f32::from_le_bytes(x)));
        for rec in bytes.get(84..).unwrap_or(&[]).as_chunks::<50>().0 {
            let p = |k: usize| -> Option<Vec3> { Some(Vec3::new(f(rec, 12 + 12 * k)?, f(rec, 16 + 12 * k)?, f(rec, 20 + 12 * k)?)) };
            let (Some(a), Some(b), Some(d)) = (p(0), p(1), p(2)) else { continue };
            c.triangle([a, b, d])?;
        }
    } else {
        let text = std::str::from_utf8(bytes).map_err(|_| IoError::Format("not an STL file (neither binary nor ASCII)".into()))?;
        let mut corners: Vec<Vec3> = Vec::with_capacity(3);
        for line in text.lines() {
            let mut words = line.split_whitespace();
            match words.next() {
                Some(w) if w.eq_ignore_ascii_case("vertex") => {
                    let xyz: Vec<f64> = words.take(3).filter_map(|v| v.parse().ok()).collect();
                    let [x, y, z] = xyz.as_slice() else {
                        return Err(IoError::Format(format!("an STL vertex needs three numbers: `{}`", line.trim())));
                    };
                    corners.push(Vec3::new(*x, *y, *z));
                }
                Some(w) if w.eq_ignore_ascii_case("endloop") => {
                    if let [a, b, d] = corners.as_slice() {
                        c.triangle([*a, *b, *d])?;
                    }
                    corners.clear();
                }
                _ => {}
            }
        }
    }
    if c.mesh.triangles.is_empty() {
        return Err(IoError::Format("the STL file has no triangles".into()));
    }
    Ok(c.mesh)
}

/// A drawing holding the STL file's solid in model space.
pub fn read(bytes: &[u8]) -> Result<Drawing> {
    let mut d = Drawing::new_metric();
    d.add(&Space::Model, Common::default(), EntityKind::Solid3d(Solid3d::new(mesh(bytes)?))).map_err(|e| IoError::Format(e.to_string()))?;
    Ok(d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::print3d::{PrintObject, stl};
    use cadcraft_geom::solid::primitives::{cuboid, sphere};

    #[test]
    fn binary_stl_round_trips_into_a_closed_solid() {
        let m = sphere(Vec3::new(1.0, 2.0, 3.0), 5.0, 48).unwrap();
        let bytes = stl(&[PrintObject { name: "s".into(), mesh: &m }]).unwrap();
        let back = mesh(&bytes).unwrap();
        assert!(back.check().is_closed());
        assert_eq!(back.triangles.len(), m.triangles.len());
        assert_eq!(back.vertices.len(), m.vertices.len());
        assert!((back.volume() - m.volume()).abs() < 1e-3 * m.volume());
    }

    #[test]
    fn ascii_stl_is_read() {
        let m = cuboid(Vec3::ZERO, Vec3::new(1.0, 2.0, 3.0)).unwrap();
        let mut text = String::from("solid box\n");
        for [a, b, c] in m.triangle_points() {
            text.push_str("  facet normal 0 0 0\n    outer loop\n");
            for p in [a, b, c] {
                text.push_str(&format!("      vertex {} {} {}\n", p.x, p.y, p.z));
            }
            text.push_str("    endloop\n  endfacet\n");
        }
        text.push_str("endsolid box\n");
        let back = mesh(text.as_bytes()).unwrap();
        assert!(back.check().is_closed());
        assert!((back.volume() - 6.0).abs() < 1e-12);
        let d = read(text.as_bytes()).unwrap();
        assert_eq!(d.model.len(), 1);
    }

    #[test]
    fn hostile_stl_is_an_error_not_a_crash() {
        for bytes in [&b""[..], b"solid x\nvertex 1 2\n", b"\xff\xfe garbage", &[0u8; 84], b"solid\nendsolid\n"] {
            assert!(mesh(bytes).is_err());
        }
        // A count that claims more triangles than the file holds is not binary.
        let mut lie = vec![0u8; 84];
        lie[80..84].copy_from_slice(&1000u32.to_le_bytes());
        assert!(mesh(&lie).is_err());
    }
}
