//! Solids swept along a path and lofted through cross-sections, both skinned
//! ([`super::skin`]).
//!
//! A sweep carries a profile along a path, standing across it, upright where the path allows
//! (its Y as near the drawing's Z as it can be) and otherwise turning as little as it can from
//! one piece of the path to the next (a rotation-minimizing frame). At each corner the
//! cross-section lies in the plane halving the turn, so straight runs meet cleanly mitred. A loft joins closed sections one after another, matching their points by
//! length along each, ruled (straight between sections) or smooth.

use super::primitives::{MAX_COORD, MAX_PROFILE_POINTS, clean_loops};
use super::skin::{Section, skin};
use super::{MAX_TRIANGLES, Mesh, SolidError};
use crate::{Mat4, Vec2, Vec3};

/// Sharpest turn (degrees) a sweep path may make at a corner.
const MAX_TURN: f64 = 170.0;
/// Sections a smooth loft adds between two given ones.
const SMOOTH_STEPS: usize = 8;

fn degenerate(m: &str) -> SolidError {
    SolidError::Degenerate(m.to_string())
}

fn finite(p: Vec3) -> bool {
    p.is_finite() && p.x.abs().max(p.y.abs()).max(p.z.abs()) <= MAX_COORD
}

/// `pts` without points repeating the one before (and, closed, the last repeating the first).
fn distinct(pts: &[Vec3], closed: bool, tol: f64) -> Vec<Vec3> {
    let mut out: Vec<Vec3> = Vec::with_capacity(pts.len());
    for p in pts {
        if !out.last().is_some_and(|q| q.dist(*p) <= tol) {
            out.push(*p);
        }
    }
    while closed && out.len() > 1 && out.first().zip(out.last()).is_some_and(|(a, b)| a.dist(*b) <= tol) {
        out.pop();
    }
    out
}

/// Most the profile may twist (radians) or grow by (as a fraction) from one section to the next:
/// straight runs are cut into steps so a twisted or tapered sweep stays smooth.
const MAX_TWIST_STEP: f64 = 0.035;
const MAX_SCALE_STEP: f64 = 0.05;

/// `path` with points added along its pieces, each piece's share of the twist and scale
/// change cut into small enough steps.
fn densified(path: &[Vec3], opts: SweepOptions) -> Vec<Vec3> {
    let total: f64 = path.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>().max(1e-300);
    let change = (opts.twist.abs() / MAX_TWIST_STEP).max((opts.scale - 1.0).abs() / MAX_SCALE_STEP);
    let mut out = Vec::with_capacity(path.len());
    for w in path.windows(2) {
        let [a, b] = w else { continue };
        let steps = ((change * a.dist(*b) / total).ceil() as usize).clamp(1, MAX_PROFILE_POINTS);
        out.extend((0..steps).map(|k| a.lerp(*b, k as f64 / steps as f64)));
    }
    out.extend(path.last());
    out
}

/// How the profile changes along a sweep.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweepOptions {
    /// Turn of the profile from start to end, radians (counter-clockwise looking along the path).
    pub twist: f64,
    /// Size of the profile at the end relative to the start.
    pub scale: f64,
}

impl Default for SweepOptions {
    fn default() -> Self {
        SweepOptions { twist: 0.0, scale: 1.0 }
    }
}

/// The solid `loops` (a profile in its own XY plane, its origin carried along the path; nested
/// loops are holes) sweeps along the polyline `path`, round and back to the start when `closed`.
pub fn sweep(loops: &[Vec<Vec2>], path: &[Vec3], closed: bool, opts: SweepOptions) -> Result<Mesh, SolidError> {
    if path.iter().any(|p| !finite(*p)) || loops.iter().flatten().any(|q| !q.is_finite()) {
        return Err(degenerate("The path or profile has a point that is not a finite number or is too far out."));
    }
    if !(opts.twist.is_finite() && opts.scale.is_finite() && opts.scale > 0.0) {
        return Err(degenerate("The twist must be a number and the scale a positive number."));
    }
    if closed && ((opts.scale - 1.0).abs() > 1e-12 || opts.twist != 0.0) {
        return Err(degenerate("A sweep round a closed path can't twist or change its scale."));
    }
    let size = path.iter().fold(0.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs())).max(1.0);
    let path = distinct(path, closed, size * 1e-12);
    if path.len() < 2 || (closed && path.len() < 3) {
        return Err(degenerate("The path needs two different points (three round a closed path)."));
    }
    let path = densified(&path, opts);
    let n = path.len();
    let loops = clean_loops(loops)?;
    let points: usize = loops.iter().map(Vec::len).sum();
    if points > MAX_PROFILE_POINTS || points.saturating_mul(n).saturating_mul(2) > MAX_TRIANGLES {
        return Err(SolidError::TooComplex);
    }
    let pieces = if closed { n } else { n - 1 };
    let at = |i: usize| path.get(i % n).copied().unwrap_or(Vec3::ZERO);
    let dirs: Vec<Vec3> = (0..pieces).map(|j| (at(j + 1) - at(j)).normalized()).collect();
    let dir = |j: usize| dirs.get(j % pieces).copied().unwrap_or(Vec3::Z);
    // Each piece's frame (across, up). On a path that never runs straight up or down, the
    // profile's Y stays as near the drawing's Z as it can (its X level, to the left looking along
    // the path), so a thread's tooth keeps pointing away from its helix's axis. Otherwise the
    // first frame stands the profile up across the path and each next one turns the last by the
    // least rotation taking one piece's direction to the next.
    let level = |t: Vec3| Vec3::Z.cross(t).normalized();
    let upright = dirs.iter().all(|t| Vec3::Z.cross(*t).len() > 0.05);
    let first = dir(0);
    let across0 = if upright || first.z.abs() <= 0.9 { level(first) } else { (Vec3::X - first * first.x).normalized() };
    let mut across: Vec<Vec3> = Vec::with_capacity(pieces + 1);
    across.push(across0);
    let turn = |a: Vec3, b: Vec3, v: Vec3| {
        let axis = a.cross(b);
        if axis.len() < 1e-12 { v } else { Mat4::rotate_about_axis(Vec3::ZERO, axis, a.dot(b).clamp(-1.0, 1.0).acos()).apply_vec(v) }
    };
    for j in 1..=pieces {
        let prev = across.last().copied().unwrap_or(across0);
        across.push(if upright { level(dir(j)) } else { turn(dir(j - 1), dir(j), prev) });
    }
    // Round a closed path the frame may come back turned: spread the difference along it.
    let lengths: Vec<f64> = std::iter::once(0.0)
        .chain((0..pieces).scan(0.0, |s, j| {
            *s += at(j).dist(at(j + 1));
            Some(*s)
        }))
        .collect();
    let total = lengths.last().copied().unwrap_or(0.0).max(1e-300);
    let unwind = if closed {
        let back = across.last().copied().unwrap_or(across0);
        let t = dir(0);
        -(across0.cross(back).dot(t)).atan2(across0.dot(back))
    } else {
        0.0
    };
    let cos_max = crate::rad(MAX_TURN / 2.0).cos();
    let sections: Vec<Section> = (0..n)
        .map(|i| {
            let f = lengths.get(i).copied().unwrap_or(0.0) / total;
            let (angle, s) = (opts.twist * f + unwind * f, 1.0 + (opts.scale - 1.0) * f);
            // The frame of the piece leaving this point (the last point: the piece arriving).
            let j = if closed || i + 1 < n { i } else { i - 1 };
            let (t, x) = (dir(j), across.get(j).copied().unwrap_or(across0));
            let y = t.cross(x);
            // Cross-sections lie in the plane halving the turn at each corner.
            let mitre = if closed || (i > 0 && i + 1 < n) { (dir(i + pieces - 1) + t).normalized() } else { t };
            let lean = mitre.dot(t);
            if lean < cos_max {
                return Err(degenerate("The path turns back on itself too sharply."));
            }
            let (sin, cos) = angle.sin_cos();
            Ok(loops
                .iter()
                .map(|l| {
                    l.iter()
                        .map(|q| {
                            let q = Vec2::new(q.x * cos - q.y * sin, q.x * sin + q.y * cos) * s;
                            let off = x * q.x + y * q.y;
                            at(i) + off - t * (mitre.dot(off) / lean)
                        })
                        .collect()
                })
                .collect())
        })
        .collect::<Result<_, _>>()?;
    skin(&sections, closed)
}

/// Arc-length position (0 to 1) of each point of the closed loop `l`.
fn stations(l: &[Vec3]) -> Vec<f64> {
    let n = l.len();
    let mut s = Vec::with_capacity(n);
    let mut run = 0.0;
    for i in 0..n {
        s.push(run);
        run += l.get(i).zip(l.get((i + 1) % n)).map_or(0.0, |(a, b)| a.dist(*b));
    }
    s.iter().map(|v| v / run.max(1e-300)).collect()
}

/// The point at arc-length position `t` (0 to 1) round the closed loop `l` with `stations`.
fn at_station(l: &[Vec3], st: &[f64], t: f64) -> Vec3 {
    let n = l.len();
    let k = st.partition_point(|s| *s <= t).saturating_sub(1);
    let (a, b) = (l.get(k).copied().unwrap_or(Vec3::ZERO), l.get((k + 1) % n.max(1)).copied().unwrap_or(Vec3::ZERO));
    let (s0, s1) = (st.get(k).copied().unwrap_or(0.0), if k + 1 < n { st.get(k + 1).copied().unwrap_or(1.0) } else { 1.0 });
    if s1 > s0 { a.lerp(b, (t - s0) / (s1 - s0)) } else { a }
}

fn centroid(l: &[Vec3]) -> Vec3 {
    l.iter().fold(Vec3::ZERO, |c, p| c + *p) * (1.0 / l.len().max(1) as f64)
}

/// The solid through the closed sections in order: ruled (straight from each section to the
/// next) or smooth (a curve through corresponding points). The first or last section may be a
/// single point (the tip of a cone or pyramid).
pub fn loft(sections: &[Vec<Vec3>], ruled: bool) -> Result<Mesh, SolidError> {
    if sections.len() < 2 {
        return Err(degenerate("A loft needs at least two cross-sections."));
    }
    if sections.iter().flatten().any(|p| !finite(*p)) {
        return Err(degenerate("A cross-section has a point that is not a finite number or is too far out."));
    }
    let total: usize = sections.iter().map(Vec::len).sum();
    if total > MAX_PROFILE_POINTS {
        return Err(SolidError::TooComplex);
    }
    let size = sections.iter().flatten().fold(0.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs())).max(1.0);
    let last = sections.len() - 1;
    let mut loops: Vec<Vec<Vec3>> = Vec::with_capacity(sections.len());
    for (k, s) in sections.iter().enumerate() {
        let l = distinct(s, true, size * 1e-12);
        let tip = l.len() == 1;
        if tip && k != 0 && k != last {
            return Err(degenerate("Only the first or last cross-section may be a point."));
        }
        if !tip && l.len() < 3 {
            return Err(degenerate("Each cross-section needs three different points or one."));
        }
        loops.push(l);
    }
    // Wind every loop the same way about the direction the loft runs.
    let centers: Vec<Vec3> = loops.iter().map(|l| centroid(l)).collect();
    for k in 0..loops.len() {
        let (a, b) = if k < last { (k, k + 1) } else { (k - 1, k) };
        let travel = centers.get(b).zip(centers.get(a)).map_or(Vec3::ZERO, |(b, a)| *b - *a);
        let Some(l) = loops.get_mut(k) else { continue };
        let normal = l.iter().zip(l.iter().cycle().skip(1)).fold(Vec3::ZERO, |n, (p, q)| n + p.cross(*q));
        if normal.dot(travel) < 0.0 {
            l.reverse();
        }
    }
    // Start each loop where it best matches the one before: the shift putting corresponding
    // points (by length round, about each centre) nearest.
    const PROBES: usize = 64;
    for k in 1..loops.len() {
        let (Some(prev), Some(cur)) = (loops.get(k - 1), loops.get(k)) else { continue };
        if prev.len() == 1 || cur.len() == 1 {
            continue;
        }
        let (sp, sc) = (stations(prev), stations(cur));
        let (cp, cc) = (centroid(prev), centroid(cur));
        let shift = (0..PROBES)
            .map(|d| {
                let cost: f64 = (0..PROBES)
                    .map(|i| {
                        let t = i as f64 / PROBES as f64;
                        let u = ((i + d) % PROBES) as f64 / PROBES as f64;
                        let (p, q) = (at_station(prev, &sp, t) - cp, at_station(cur, &sc, u) - cc);
                        (p - q).len() * (p - q).len()
                    })
                    .sum();
                (d, cost)
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map_or(0, |(d, _)| d);
        // Begin at the point nearest that position.
        let target = at_station(cur, &sc, shift as f64 / PROBES as f64);
        let start = cur.iter().enumerate().min_by(|a, b| a.1.dist(target).total_cmp(&b.1.dist(target))).map_or(0, |(i, _)| i);
        if let Some(l) = loops.get_mut(k) {
            l.rotate_left(start);
        }
    }
    // Every loop sampled at every loop's stations, so corners stay corners.
    let all: Vec<Vec<f64>> = loops.iter().map(|l| if l.len() == 1 { Vec::new() } else { stations(l) }).collect();
    let mut merged: Vec<f64> = all.iter().flatten().copied().collect();
    merged.sort_by(f64::total_cmp);
    // Stations of different sections that nearly coincide would leave slivers of walls between
    // them: keep one where they are closer than a quarter of the finest section's spacing.
    let finest = all.iter().map(Vec::len).max().unwrap_or(1).max(1);
    let gap = 0.25 / finest as f64;
    merged.dedup_by(|a, b| (*a - *b).abs() < gap);
    if merged.len() > MAX_PROFILE_POINTS {
        return Err(SolidError::TooComplex);
    }
    let rings: Vec<Vec<Vec3>> = loops
        .iter()
        .zip(&all)
        .map(|(l, st)| match l.as_slice() {
            [tip] => vec![*tip; merged.len()],
            _ => merged.iter().map(|t| at_station(l, st, *t)).collect(),
        })
        .collect();
    let rings = if ruled || rings.len() < 3 { rings } else { smooth(&rings) };
    if rings.len().saturating_mul(merged.len()).saturating_mul(2) > MAX_TRIANGLES {
        return Err(SolidError::TooComplex);
    }
    skin(&rings.into_iter().map(|r| vec![r]).collect::<Vec<Section>>(), false)
}

/// Rings between the given ones on Catmull-Rom curves through corresponding points.
fn smooth(rings: &[Vec<Vec3>]) -> Vec<Vec<Vec3>> {
    let n = rings.len();
    let ring = |k: isize| rings.get(k.clamp(0, n as isize - 1) as usize);
    let mut out = Vec::with_capacity((n - 1) * SMOOTH_STEPS + 1);
    for k in 0..n - 1 {
        let ki = k as isize;
        let (Some(p0), Some(p1), Some(p2), Some(p3)) = (ring(ki - 1), ring(ki), ring(ki + 1), ring(ki + 2)) else { continue };
        for step in 0..SMOOTH_STEPS {
            let t = step as f64 / SMOOTH_STEPS as f64;
            let (t2, t3) = (t * t, t * t * t);
            out.push(
                (0..p1.len())
                    .map(|i| {
                        let g = |r: &Vec<Vec3>| r.get(i).copied().unwrap_or(Vec3::ZERO);
                        let (a, b, c, d) = (g(p0), g(p1), g(p2), g(p3));
                        (b * 2.0 + (c - a) * t + (a * 2.0 - b * 5.0 + c * 4.0 - d) * t2 + (b * 3.0 - a - c * 3.0 + d) * t3) * 0.5
                    })
                    .collect(),
            );
        }
    }
    if let Some(l) = rings.last() {
        out.push(l.clone());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TAU;

    fn circle(r: f64, n: usize) -> Vec<Vec2> {
        (0..n).map(|k| Vec2::polar(Vec2::ZERO, r, TAU * k as f64 / n as f64)).collect()
    }

    fn ngon_area(r: f64, n: usize) -> f64 {
        n as f64 / 2.0 * r * r * (TAU / n as f64).sin()
    }

    #[test]
    fn a_straight_sweep_is_an_extrusion_and_a_corner_is_mitred() {
        let square = vec![vec![Vec2::new(-1.0, -1.0), Vec2::new(1.0, -1.0), Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0)]];
        let up = sweep(&square, &[Vec3::ZERO, Vec3::new(0.0, 0.0, 5.0)], false, SweepOptions::default()).unwrap();
        assert!(up.check().is_closed());
        assert!((up.volume() - 20.0).abs() < 1e-9);
        let b = up.bounds();
        assert!(b.min.dist(Vec3::new(-1.0, -1.0, 0.0)) < 1e-9 && b.max.dist(Vec3::new(1.0, 1.0, 5.0)) < 1e-9);
        // An L-shaped path: two square bars meeting in a mitre, each its centre line long.
        let l = sweep(&square, &[Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 8.0, 0.0)], false, SweepOptions::default()).unwrap();
        assert!(l.check().is_closed());
        assert!((l.volume() - 4.0 * 18.0).abs() < 1e-9, "{}", l.volume());
    }

    #[test]
    fn a_circle_swept_round_a_circle_is_a_torus_and_twist_and_scale_apply() {
        let path: Vec<Vec3> = (0..96).map(|k| Vec2::polar(Vec2::ZERO, 10.0, TAU * k as f64 / 96.0).to3(0.0)).collect();
        let ring = sweep(&[circle(2.0, 32)], &path, true, SweepOptions::default()).unwrap();
        assert!(ring.check().is_closed());
        // Pappus: the section's area times the path its centre takes.
        let exact = ngon_area(2.0, 32) * TAU * 10.0;
        assert!((ring.volume() - exact).abs() < 0.01 * exact, "{} vs {exact}", ring.volume());
        // A square tapered to half its size and twisted a quarter turn up a straight path.
        let square = vec![vec![Vec2::new(-1.0, -1.0), Vec2::new(1.0, -1.0), Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0)]];
        let path: Vec<Vec3> = (0..=20).map(|k| Vec3::new(0.0, 0.0, k as f64 * 0.5)).collect();
        let t = sweep(&square, &path, false, SweepOptions { twist: TAU / 4.0, scale: 0.5 }).unwrap();
        assert!(t.check().is_closed());
        // A frustum-like volume: the area shrinks with the square of the scale.
        let exact = 4.0 * 10.0 * (1.0 + 0.5 + 0.25) / 3.0;
        assert!((t.volume() - exact).abs() < 0.02 * exact, "{} vs {exact}", t.volume());
    }

    #[test]
    fn a_sweep_along_a_helix_coils_and_bad_paths_are_refused() {
        let helix: Vec<Vec3> = (0..=240)
            .map(|k| {
                let a = TAU * k as f64 / 48.0;
                Vec3::new(8.0 * a.cos(), 8.0 * a.sin(), 3.0 * a / TAU)
            })
            .collect();
        let coil = sweep(&[circle(1.0, 16)], &helix, false, SweepOptions::default()).unwrap();
        assert!(coil.check().is_closed());
        // A tooth swept along the helix keeps pointing away from the axis all the way up.
        let tooth = [vec![Vec2::new(0.5, -0.5), Vec2::new(0.5, 0.5), Vec2::new(-0.5, 0.0)]];
        let thread = sweep(&tooth, &helix, false, SweepOptions::default()).unwrap();
        let radial = |p: &Vec3| Vec2::new(p.x, p.y).len();
        let (lo, hi) = thread.vertices.iter().fold((f64::INFINITY, 0.0f64), |(lo, hi), p| (lo.min(radial(p)), hi.max(radial(p))));
        assert!(lo > 7.45 && hi < 8.55, "{lo} {hi}");
        let length: f64 = helix.windows(2).map(|w| w[0].dist(w[1])).sum();
        let exact = ngon_area(1.0, 16) * length;
        assert!((coil.volume() - exact).abs() < 0.02 * exact, "{} vs {exact}", coil.volume());
        let sq = [circle(1.0, 8)];
        assert!(sweep(&sq, &[Vec3::ZERO], false, SweepOptions::default()).is_err());
        assert!(sweep(&sq, &[Vec3::ZERO, Vec3::X, Vec3::ZERO], false, SweepOptions::default()).is_err());
        assert!(sweep(&sq, &[Vec3::ZERO, Vec3::new(f64::NAN, 0.0, 0.0)], false, SweepOptions::default()).is_err());
        assert!(sweep(&sq, &[Vec3::ZERO, Vec3::X], false, SweepOptions { twist: 0.0, scale: 0.0 }).is_err());
    }

    #[test]
    fn lofts_join_unlike_sections() {
        // A square at the bottom to a circle at the top: a ruled loft stays closed and between the
        // prism and the cylinder in volume.
        let square: Vec<Vec3> = [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)].iter().map(|(x, y)| Vec3::new(*x, *y, 0.0)).collect();
        let round: Vec<Vec3> = circle(1.5, 40).iter().map(|q| q.to3(6.0)).collect();
        let m = loft(&[square.clone(), round.clone()], true).unwrap();
        assert!(m.check().is_closed());
        assert!(m.volume() < 16.0 * 6.0 && m.volume() > ngon_area(1.5, 40) * 6.0, "{}", m.volume());
        // Wound either way, the result is the same.
        let mut back = round.clone();
        back.reverse();
        let m2 = loft(&[square.clone(), back], true).unwrap();
        assert!((m2.volume() - m.volume()).abs() < 1e-6 * m.volume());
        // To a point: a pyramid, a third of base times height.
        let tip = loft(&[square.clone(), vec![Vec3::new(0.0, 0.0, 3.0)]], true).unwrap();
        assert!(tip.check().is_closed());
        assert!((tip.volume() - 16.0).abs() < 1e-9);
        // Smooth through three: a bulging vase, fuller than the ruled one.
        let mid: Vec<Vec3> = circle(3.0, 40).iter().map(|q| q.to3(3.0)).collect();
        let ruled = loft(&[square.clone(), mid.clone(), round.clone()], true).unwrap();
        let vase = loft(&[square, mid, round], false).unwrap();
        assert!(vase.check().is_closed() && ruled.check().is_closed());
        assert!(vase.volume() > ruled.volume());
        assert!(loft(&[vec![Vec3::ZERO, Vec3::X, Vec3::Y]], true).is_err());
    }
}
