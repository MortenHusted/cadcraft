//! Rebuild the part of a face a boolean keeps from its pieces.
//!
//! Cutting a triangle by every plane that crosses it leaves many slivers (each plane cuts the
//! whole triangle, not just where the other solid is). The kept pieces are joined back into
//! their outline, holes included, and that outline is triangulated afresh.

use std::collections::HashMap;

use super::stitch::{split_t_junctions, weld_rings};
use super::triangulate::triangulate_with_holes;
use crate::{Vec2, Vec3, point_in_polygon, shoelace};

/// The kept `pieces` of one planar face (unit normal `n`), as fewer triangles covering the same
/// area; `None` when they can't be joined cleanly (the caller then keeps the pieces).
pub(crate) fn merge(pieces: &[Vec<Vec3>], n: Vec3, tol: f64) -> Option<Vec<Vec<Vec3>>> {
    let (verts, mut rings) = weld_rings(pieces, tol).ok()?;
    // Pieces of different triangles meet with points on each other's edges: split those first.
    split_t_junctions(&verts, &mut rings, tol);
    // Edges between two kept pieces cancel; what is left is the outline.
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for r in &rings {
        for (k, a) in r.iter().enumerate() {
            let b = r.get((k + 1) % r.len())?;
            *edges.entry((*a, *b)).or_default() += 1;
        }
    }
    // Work in the face's plane: (u, v) counter-clockwise seen from the front.
    let u = if n.x.abs() < 0.9 { Vec3::X.cross(n) } else { Vec3::Y.cross(n) }.normalized();
    let v = n.cross(u);
    let pt = |i: u32| verts.get(i as usize).copied();
    let uv = |i: u32| pt(i).map(|p| Vec2::new(p.dot(u), p.dot(v)));
    let mut next: HashMap<u32, Vec<u32>> = HashMap::new();
    let mut boundary: Vec<(u32, u32)> = edges.keys().filter(|(a, b)| !edges.contains_key(&(*b, *a))).copied().collect();
    boundary.sort_unstable();
    for (a, b) in &boundary {
        next.entry(*a).or_default().push(*b);
    }
    // Follow the outline with the face on the left. Where it touches itself, take the sharpest
    // left turn, which keeps each loop simple.
    let mut used: HashMap<(u32, u32), ()> = HashMap::new();
    let mut loops: Vec<Vec<u32>> = Vec::new();
    for &(a0, b0) in &boundary {
        if used.contains_key(&(a0, b0)) {
            continue;
        }
        let mut l = vec![a0];
        used.insert((a0, b0), ());
        let (mut from, mut at) = (a0, b0);
        while at != a0 {
            if l.len() > boundary.len() {
                return None;
            }
            l.push(at);
            let dir = uv(at)? - uv(from)?;
            let turn = |w: &u32| {
                uv(*w).map(|q| {
                    let out = q - uv(at).unwrap_or(q);
                    dir.cross(out).atan2(dir.dot(out))
                })
            };
            let options = next.get(&at)?.iter().filter(|w| !used.contains_key(&(at, **w)));
            let w = *options.max_by(|x, y| turn(x).unwrap_or(f64::MIN).total_cmp(&turn(y).unwrap_or(f64::MIN)))?;
            used.insert((at, w), ());
            (from, at) = (at, w);
        }
        loops.push(l);
    }
    let flat = |l: &[u32]| -> Option<Vec<(Vec2, Vec3)>> {
        let pts: Vec<(Vec2, Vec3)> = l.iter().map(|i| Some((uv(*i)?, pt(*i)?))).collect::<Option<_>>()?;
        Some(drop_collinear(pts, tol))
    };
    let mut outers: Vec<Vec<(Vec2, Vec3)>> = Vec::new();
    let mut holes: Vec<Vec<(Vec2, Vec3)>> = Vec::new();
    for l in &loops {
        let f = flat(l)?;
        if f.len() < 3 {
            continue;
        }
        let area = shoelace(&f.iter().map(|x| x.0).collect::<Vec<_>>());
        if area > 0.0 {
            outers.push(f);
        } else {
            holes.push(f);
        }
    }
    let mut out = Vec::new();
    let mut assigned = vec![false; holes.len()];
    let xy = |l: &[(Vec2, Vec3)]| l.iter().map(|x| x.0).collect::<Vec<_>>();
    for o in &outers {
        let o2 = xy(o);
        let mine: Vec<usize> =
            (0..holes.len()).filter(|&h| holes.get(h).and_then(|l| l.first()).is_some_and(|p| point_in_polygon(&o2, p.0))).collect();
        let hole2: Vec<Vec<Vec2>> = mine.iter().filter_map(|&h| holes.get(h).map(|l| xy(l))).collect();
        let refs: Vec<&[Vec2]> = hole2.iter().map(Vec::as_slice).collect();
        let pts3: Vec<Vec3> = o.iter().map(|x| x.1).chain(mine.iter().filter_map(|&h| holes.get(h)).flat_map(|l| l.iter().map(|x| x.1))).collect();
        for &h in &mine {
            if let Some(a) = assigned.get_mut(h) {
                *a = true;
            }
        }
        for [a, b, c] in triangulate_with_holes(&o2, &refs).ok()? {
            out.push(vec![*pts3.get(a)?, *pts3.get(b)?, *pts3.get(c)?]);
        }
    }
    // Every hole must sit in an outline, and the triangles must cover what the pieces did.
    if assigned.iter().any(|a| !a) {
        return None;
    }
    let area = |ps: &[Vec<Vec3>]| ps.iter().map(|p| polygon_area(p, n)).sum::<f64>();
    let (before, after) = (area(pieces), area(&out));
    ((before - after).abs() <= 1e-9 * before.max(tol * tol) + tol * tol * 100.0).then_some(out)
}

/// Area of a planar polygon with normal `n`.
fn polygon_area(p: &[Vec3], n: Vec3) -> f64 {
    let Some(o) = p.first() else { return 0.0 };
    let len = p.len();
    p.iter().enumerate().map(|(k, a)| (*a - *o).cross(p.get((k + 1) % len).copied().unwrap_or(*o) - *o).dot(n)).sum::<f64>() / 2.0
}

/// Drop points lying on the straight line between their neighbours (within `tol`).
fn drop_collinear(mut pts: Vec<(Vec2, Vec3)>, tol: f64) -> Vec<(Vec2, Vec3)> {
    let mut changed = true;
    while changed && pts.len() > 3 {
        changed = false;
        let len = pts.len();
        let found = (0..len).find(|&k| {
            let (Some(a), Some(b), Some(c)) = (pts.get((k + len - 1) % len), pts.get(k), pts.get((k + 1) % len)) else { return false };
            let ac = c.0 - a.0;
            let l = ac.len();
            l > 0.0 && (b.0 - a.0).cross(ac).abs() / l <= tol && (b.0 - a.0).dot(ac) > 0.0 && (c.0 - b.0).dot(ac) > 0.0
        });
        if let Some(k) = found {
            pts.remove(k);
            changed = true;
        }
    }
    pts
}
