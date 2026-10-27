//! Solids skinned through a sequence of cross-sections: the tube a profile sweeps along a path,
//! a loft from one profile to the next, the cutter that rounds or bevels an edge.
//!
//! Every section has the same loops with the same number of points; point `i` of a loop joins
//! point `i` of the same loop in the next section. The ends are capped flat, or, for a closed
//! skin (a sweep round a closed path), the last section joins the first.

use super::primitives::{MAX_COORD, MAX_PROFILE_POINTS, cap};
use super::{MAX_TRIANGLES, Mesh, SolidError};
use crate::{Vec2, Vec3, point_in_polygon, shoelace};

/// A cross-section: closed loops of points in one plane, outer loops and the holes in them.
pub type Section = Vec<Vec<Vec3>>;

fn degenerate(m: &str) -> SolidError {
    SolidError::Degenerate(m.to_string())
}

/// Twice the vector area of a closed loop (Newell's method): normal to its plane, pointing the
/// way it winds counter-clockwise about.
fn newell(l: &[Vec3]) -> Vec3 {
    l.iter().zip(l.iter().cycle().skip(1)).fold(Vec3::ZERO, |n, (a, b)| n + a.cross(*b))
}

fn centroid(s: &Section) -> Vec3 {
    let n = s.iter().map(Vec::len).sum::<usize>().max(1) as f64;
    s.iter().flatten().fold(Vec3::ZERO, |c, p| c + *p) * (1.0 / n)
}

/// Two unit vectors `u`, `v` across the plane with unit normal `n`, `u × v = n`.
fn frame(n: Vec3) -> (Vec3, Vec3) {
    let a = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let u = a.cross(n).normalized();
    (u, n.cross(u))
}

/// A section seen in its plane: its loops in the frame whose normal follows `toward`.
fn flat(s: &Section, toward: Vec3) -> Result<Vec<Vec<Vec2>>, SolidError> {
    let n = s.iter().map(|l| newell(l)).max_by(|a, b| a.len().total_cmp(&b.len())).unwrap_or(Vec3::ZERO).normalized();
    let n = if n.dot(toward) < 0.0 { n * -1.0 } else { n };
    if n == Vec3::ZERO || n.dot(toward.normalized()).abs() < 1e-6 {
        return Err(degenerate("A cross-section lies along the direction the solid runs."));
    }
    let o = centroid(s);
    let (u, v) = frame(n);
    Ok(s.iter().map(|l| l.iter().map(|p| Vec2::new((*p - o).dot(u), (*p - o).dot(v))).collect()).collect())
}

/// Whether every point of `l` is within `tol` of the first: a loop shrunk to a point (the tip
/// of a loft to a point).
fn is_point(l: &[Vec3], tol: f64) -> bool {
    l.first().is_some_and(|a| l.iter().all(|p| p.dist(*a) <= tol))
}

/// The closed solid through `sections` in order (see the module documentation). A loop of the
/// first or last section may shrink to a point.
pub fn skin(sections: &[Section], closed: bool) -> Result<Mesh, SolidError> {
    let (Some(first), true) = (sections.first(), sections.len() >= 2) else {
        return Err(degenerate("A solid through cross-sections needs at least two of them."));
    };
    let counts: Vec<usize> = first.iter().map(Vec::len).collect();
    if counts.is_empty() || counts.iter().any(|n| *n < 3) {
        return Err(degenerate("Each cross-section loop needs at least three points."));
    }
    if sections.iter().any(|s| s.len() != counts.len() || s.iter().zip(&counts).any(|(l, n)| l.len() != *n)) {
        return Err(degenerate("The cross-sections don't have the same loops and points."));
    }
    let per_section: usize = counts.iter().sum();
    if per_section > MAX_PROFILE_POINTS || per_section.saturating_mul(sections.len()).saturating_mul(2) > MAX_TRIANGLES {
        return Err(SolidError::TooComplex);
    }
    if sections.iter().flatten().flatten().any(|p| !p.is_finite() || p.x.abs().max(p.y.abs()).max(p.z.abs()) > MAX_COORD) {
        return Err(degenerate("A cross-section has a point that is not a finite number or is too far out."));
    }
    let size = sections.iter().flatten().flatten().fold(0.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs())).max(1e-300);
    let tol = size * 1e-12;
    let last = sections.len() - 1;
    let pointlike = |k: usize| sections.get(k).is_some_and(|s| s.iter().any(|l| is_point(l, tol)));
    if (1..last).any(pointlike) || (closed && (pointlike(0) || pointlike(last))) {
        return Err(degenerate("Only the first or last cross-section may shrink to a point."));
    }
    // The direction the solid runs at section k.
    let travel = |k: usize| {
        let (a, b) = if k < last { (k, k + 1) } else { (k.saturating_sub(1), k) };
        match (sections.get(a), sections.get(b)) {
            (Some(a), Some(b)) => centroid(b) - centroid(a),
            _ => Vec3::ZERO,
        }
    };
    // Wind outer loops counter-clockwise and holes clockwise about the direction of travel, so
    // the walls all face the same way; a section of full loops decides.
    let r = (0..=last).find(|k| !pointlike(*k)).unwrap_or(0);
    let reference = flat(sections.get(r).unwrap_or(first), travel(r))?;
    let reverse: Vec<bool> = reference
        .iter()
        .enumerate()
        .map(|(i, l)| {
            let probe = l.first().copied().unwrap_or(Vec2::ZERO);
            let depth = reference.iter().enumerate().filter(|(j, o)| *j != i && point_in_polygon(o, probe)).count();
            (shoelace(l) > 0.0) != (depth % 2 == 0)
        })
        .collect();
    let mut m = Mesh::default();
    // vertex[k][j][i]: point i of loop j of section k; a loop shrunk to a point is one vertex.
    let mut vertex: Vec<Vec<Vec<u32>>> = Vec::with_capacity(sections.len());
    for s in sections {
        let mut loops = Vec::with_capacity(s.len());
        for (j, l) in s.iter().enumerate() {
            let ordered: Vec<Vec3> = if reverse.get(j) == Some(&true) { l.iter().rev().copied().collect() } else { l.clone() };
            if is_point(&ordered, tol) {
                let v = u32::try_from(m.vertices.len()).map_err(|_| SolidError::TooComplex)?;
                m.vertices.push(ordered.first().copied().unwrap_or(Vec3::ZERO));
                loops.push(vec![v; ordered.len()]);
            } else {
                let start = u32::try_from(m.vertices.len()).map_err(|_| SolidError::TooComplex)?;
                m.vertices.extend(ordered);
                loops.push((0..l.len()).filter_map(|i| u32::try_from(i).ok()).map(|i| start.saturating_add(i)).collect());
            }
        }
        vertex.push(loops);
    }
    let at = |k: usize, j: usize, i: usize| vertex.get(k).and_then(|s| s.get(j)).and_then(|l| l.get(i % l.len().max(1))).copied().unwrap_or(0);
    let tri = |m: &mut Mesh, a: u32, b: u32, c: u32| {
        if a != b && b != c && c != a {
            m.triangles.push([a, b, c]);
        }
    };
    let joins = if closed { sections.len() } else { last };
    for k in 0..joins {
        let next = (k + 1) % sections.len();
        for (j, n) in counts.iter().enumerate() {
            for i in 0..*n {
                let q = [at(k, j, i), at(k, j, i + 1), at(next, j, i + 1), at(next, j, i)];
                let p = q.map(|v| m.vertices.get(v as usize).copied().unwrap_or(Vec3::ZERO));
                // A twisted quad (a twisted or curving skin) is split from its centre, so it
                // stays true to the ruled surface whichever way it twists; a flat one in two.
                let normal = (p[1] - p[0]).cross(p[2] - p[0]).normalized();
                let twist = normal.dot(p[3] - p[0]).abs();
                if twist > 1e-4 * p[0].dist(p[2]).max(p[1].dist(p[3])) && q[0] != q[1] && q[2] != q[3] {
                    let c = u32::try_from(m.vertices.len()).map_err(|_| SolidError::TooComplex)?;
                    m.vertices.push((p[0] + p[1] + p[2] + p[3]) * 0.25);
                    for e in 0..4 {
                        tri(&mut m, c, q[e], q[(e + 1) % 4]);
                    }
                } else {
                    tri(&mut m, q[0], q[1], q[2]);
                    tri(&mut m, q[0], q[2], q[3]);
                }
            }
        }
    }
    if !closed {
        for (k, flip) in [(0, true), (last, false)] {
            let Some(s) = sections.get(k) else { continue };
            if pointlike(k) {
                continue;
            }
            // `cap` numbers the loops' points in order: map back to this section's vertices.
            let flat_loops = flat(s, travel(k))?;
            let mut index = Vec::with_capacity(per_section);
            for (j, n) in counts.iter().enumerate() {
                let rev = reverse.get(j) == Some(&true);
                index.extend((0..*n).map(|i| at(k, j, if rev { n - 1 - i } else { i })));
            }
            for [a, b, c] in cap(&flat_loops)? {
                let v = |x: usize| index.get(x).copied().unwrap_or(0);
                if flip {
                    tri(&mut m, v(a), v(c), v(b));
                } else {
                    tri(&mut m, v(a), v(b), v(c));
                }
            }
        }
    }
    if m.triangles.len() > MAX_TRIANGLES {
        return Err(SolidError::TooComplex);
    }
    m.orient_outward();
    Ok(m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TAU;

    fn square(z: f64, half: f64) -> Vec<Vec3> {
        vec![Vec3::new(-half, -half, z), Vec3::new(half, -half, z), Vec3::new(half, half, z), Vec3::new(-half, half, z)]
    }

    #[test]
    fn two_squares_make_a_box_whichever_way_they_wind() {
        for reversed in [false, true] {
            let mut top = square(3.0, 1.0);
            if reversed {
                top.reverse();
                top.rotate_left(1);
            }
            let bottom = if reversed { top.iter().map(|p| Vec3::new(p.x, p.y, 0.0)).collect() } else { square(0.0, 1.0) };
            let m = skin(&[vec![bottom], vec![top]], false).unwrap();
            assert!(m.check().is_closed());
            assert!((m.volume() - 12.0).abs() < 1e-12, "{}", m.volume());
        }
    }

    #[test]
    fn a_loft_to_a_point_is_a_pyramid() {
        let apex = vec![Vec3::new(0.0, 0.0, 3.0); 4];
        let m = skin(&[vec![square(0.0, 1.0)], vec![apex]], false).unwrap();
        assert!(m.check().is_closed());
        assert!((m.volume() - 4.0).abs() < 1e-12);
    }

    #[test]
    fn a_frame_with_a_hole_skins_into_a_tube_and_a_ring_closes_on_itself() {
        let m = skin(&[vec![square(0.0, 2.0), square(0.0, 1.0)], vec![square(1.0, 2.0), square(1.0, 1.0)]], false).unwrap();
        assert!(m.check().is_closed());
        assert!((m.volume() - 12.0).abs() < 1e-12);
        // A square section carried round a circle of radius 5: a square torus.
        let n = 64;
        let sections: Vec<Section> = (0..n)
            .map(|k| {
                let a = TAU * k as f64 / n as f64;
                let (c, s) = (a.cos(), a.sin());
                let at = |r: f64, z: f64| Vec3::new(r * c, r * s, z);
                vec![vec![at(4.0, -1.0), at(6.0, -1.0), at(6.0, 1.0), at(4.0, 1.0)]]
            })
            .collect();
        let ring = skin(&sections, true).unwrap();
        assert!(ring.check().is_closed());
        let exact = 4.0 * TAU * 5.0;
        assert!((ring.volume() - exact).abs() < 0.01 * exact);
    }

    #[test]
    fn mismatched_or_flat_sections_are_refused() {
        assert!(skin(&[vec![square(0.0, 1.0)]], false).is_err());
        let tri = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
        assert!(skin(&[vec![square(0.0, 1.0)], vec![tri]], false).is_err());
        // Both in the same plane: the solid runs along neither.
        assert!(skin(&[vec![square(0.0, 1.0)], vec![square(0.0, 2.0)]], false).is_err());
        let nan = vec![Vec3::new(f64::NAN, 0.0, 0.0), Vec3::X, Vec3::Y, Vec3::Z];
        assert!(skin(&[vec![square(0.0, 1.0)], vec![nan]], false).is_err());
    }
}
