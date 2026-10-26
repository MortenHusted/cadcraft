//! Ear-clipping triangulation of simple polygons, with holes bridged into the outer loop first
//! (Eberly, "Triangulation by Ear Clipping"). Used for the flat caps of extrusions and
//! revolutions and for the faces a boolean rebuilds. Only input points become vertices.

use super::SolidError;
use crate::Vec2;

/// Most points triangulated at once (ear clipping is quadratic).
const MAX_POINTS: usize = 50_000;

/// Triangulate the counter-clockwise polygon `outer` with clockwise `holes` inside it. Indices
/// number `outer`'s points first, then each hole's, in order; triangles are counter-clockwise.
pub(crate) fn triangulate_with_holes(outer: &[Vec2], holes: &[&[Vec2]]) -> Result<Vec<[usize; 3]>, SolidError> {
    let pts: Vec<Vec2> = outer.iter().chain(holes.iter().flat_map(|h| h.iter())).copied().collect();
    if pts.len() > MAX_POINTS {
        return Err(SolidError::TooComplex);
    }
    let mut ring: Vec<usize> = (0..outer.len()).collect();
    let mut start = outer.len();
    let mut rings: Vec<Vec<usize>> = Vec::with_capacity(holes.len());
    for h in holes {
        rings.push((start..start + h.len()).collect());
        start += h.len();
    }
    let at = |i: &usize| pts.get(*i).copied().unwrap_or(Vec2::ZERO);
    let right = |r: &Vec<usize>| r.iter().map(|i| at(i).x).fold(f64::NEG_INFINITY, f64::max);
    // Bridge the hole reaching furthest right first, so later bridges can't cross earlier ones.
    rings.sort_by(|a, b| right(b).total_cmp(&right(a)));
    for h in rings {
        ring = bridge(&pts, ring, &h);
    }
    Ok(ear_clip(&pts, ring))
}

/// Triangulate a simple polygon of either winding; triangles keep the polygon's winding.
pub fn triangulate_polygon(pts: &[Vec2]) -> Vec<[usize; 3]> {
    if pts.len() > MAX_POINTS {
        return Vec::new();
    }
    let ring: Vec<usize> = (0..pts.len()).collect();
    if crate::shoelace(pts) >= 0.0 {
        ear_clip(pts, ring)
    } else {
        let rev: Vec<usize> = ring.into_iter().rev().collect();
        ear_clip(pts, rev).into_iter().map(|[a, b, c]| [a, c, b]).collect()
    }
}

fn cross3(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    (b - a).cross(c - b)
}

/// `p` inside or on triangle `a b c` (counter-clockwise), within `eps` of area.
fn in_triangle(a: Vec2, b: Vec2, c: Vec2, p: Vec2, eps: f64) -> bool {
    (b - a).cross(p - a) >= -eps && (c - b).cross(p - b) >= -eps && (a - c).cross(p - c) >= -eps
}

/// Join hole `hole` (clockwise) to the counter-clockwise `ring` with a two-way cut from the
/// hole's rightmost point to a point of the ring it can see.
fn bridge(pts: &[Vec2], ring: Vec<usize>, hole: &[usize]) -> Vec<usize> {
    let p = |i: usize| pts.get(i).copied().unwrap_or(Vec2::ZERO);
    let n = ring.len();
    let Some(m_pos) = (0..hole.len()).max_by(|a, b| {
        let (pa, pb) = (p(hole.get(*a).copied().unwrap_or(0)), p(hole.get(*b).copied().unwrap_or(0)));
        pa.x.total_cmp(&pb.x).then(pb.y.total_cmp(&pa.y))
    }) else {
        return ring;
    };
    let m = p(hole.get(m_pos).copied().unwrap_or(0));
    let ri = |k: usize| ring.get(k % n.max(1)).copied().unwrap_or(0);
    // The nearest ring edge crossed by the ray from M towards +X.
    let mut best: Option<(f64, usize)> = None;
    for e in 0..n {
        let (a, b) = (p(ri(e)), p(ri(e + 1)));
        if (a.y > m.y) == (b.y > m.y) || a.y == b.y {
            continue;
        }
        let x = a.x + (m.y - a.y) * (b.x - a.x) / (b.y - a.y);
        if x >= m.x && best.is_none_or(|(bx, _)| x < bx) {
            best = Some((x, e));
        }
    }
    let Some((ix, e)) = best else {
        // No edge to the right: not inside the ring. Keep the ring; the hole is dropped.
        return ring;
    };
    let i_pt = Vec2::new(ix, m.y);
    let (a, b) = (p(ri(e)), p(ri(e + 1)));
    let mut target = if a.x > b.x { e % n } else { (e + 1) % n };
    if a.near(i_pt, 0.0) {
        target = e % n;
    } else if b.near(i_pt, 0.0) {
        target = (e + 1) % n;
    } else {
        // A reflex vertex inside triangle M, I, P blocks the view of P: take the one closest in
        // angle to the ray instead.
        let tp = p(ri(target));
        let (t0, t1, t2) = if cross3(m, i_pt, tp) >= 0.0 { (m, i_pt, tp) } else { (m, tp, i_pt) };
        let mut best_angle = f64::INFINITY;
        for k in 0..n {
            let v = p(ri(k));
            if k == target || v.near(tp, 0.0) {
                continue;
            }
            let reflex = cross3(p(ri(k + n - 1)), v, p(ri(k + 1))) <= 0.0;
            if reflex && in_triangle(t0, t1, t2, v, 0.0) {
                let d = v - m;
                let angle = d.y.abs().atan2(d.x);
                if angle < best_angle || (angle == best_angle && d.len() < (p(ri(target)) - m).len()) {
                    best_angle = angle;
                    target = k;
                }
            }
        }
    }
    // A point used twice (by an earlier bridge): take the copy whose corner M sits in.
    let tv = ri(target);
    let in_sector = |k: usize| {
        let (prev, here, next) = (p(ri(k + n - 1)), p(ri(k)), p(ri(k + 1)));
        let (l1, l2) = ((here - prev).cross(m - prev) >= 0.0, (next - here).cross(m - here) >= 0.0);
        if cross3(prev, here, next) >= 0.0 { l1 && l2 } else { l1 || l2 }
    };
    if let Some(k) = (0..n).find(|&k| ri(k) == tv && in_sector(k)) {
        target = k;
    }
    let mut out = Vec::with_capacity(n + hole.len() + 2);
    out.extend(ring.iter().take(target + 1));
    out.extend(hole.iter().skip(m_pos));
    out.extend(hole.iter().take(m_pos + 1));
    out.push(tv);
    out.extend(ring.iter().skip(target + 1));
    out
}

/// Clip ears from the counter-clockwise `ring` until a triangle is left.
fn ear_clip(pts: &[Vec2], mut ring: Vec<usize>) -> Vec<[usize; 3]> {
    let p = |i: usize| pts.get(i).copied().unwrap_or(Vec2::ZERO);
    let size = ring.iter().map(|&i| p(i)).fold(0.0f64, |m, q| m.max(q.x.abs()).max(q.y.abs())).max(1e-300);
    let eps = size * size * 1e-15;
    let mut out = Vec::with_capacity(ring.len().saturating_sub(2));
    let mut k = 0usize;
    let mut misses = 0usize;
    while ring.len() > 3 {
        let n = ring.len();
        let at = |j: usize| ring.get(j % n).copied().unwrap_or(0);
        let (ia, ib, ic) = (at(k + n - 1), at(k), at(k + 1));
        let (a, b, c) = (p(ia), p(ib), p(ic));
        let ear = cross3(a, b, c) > eps
            && ring.iter().all(|&j| {
                if j == ia || j == ib || j == ic {
                    return true;
                }
                let q = p(j);
                q.near(a, 0.0) || q.near(b, 0.0) || q.near(c, 0.0) || !in_triangle(a, b, c, q, eps)
            });
        if ear {
            out.push([ia, ib, ic]);
            ring.remove(k % n);
            k = (k + n - 2) % (n - 1);
            misses = 0;
            continue;
        }
        k = (k + 1) % n;
        misses += 1;
        if misses >= n {
            // No ear (rounding or a self-touching polygon): drop the flattest corner; keep it as
            // a triangle when it still turns the right way.
            let flattest = (0..n).min_by(|&x, &y| {
                cross3(p(at(x + n - 1)), p(at(x)), p(at(x + 1))).abs().total_cmp(&cross3(p(at(y + n - 1)), p(at(y)), p(at(y + 1))).abs())
            });
            let j = flattest.unwrap_or(0);
            let (ja, jb, jc) = (at(j + n - 1), at(j), at(j + 1));
            if cross3(p(ja), p(jb), p(jc)) > eps {
                out.push([ja, jb, jc]);
            }
            ring.remove(j % n);
            k = 0;
            misses = 0;
        }
    }
    if let [a, b, c] = ring[..]
        && cross3(p(a), p(b), p(c)) > eps
    {
        out.push([a, b, c]);
    }
    out
}
