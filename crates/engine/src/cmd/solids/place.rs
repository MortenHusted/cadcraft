//! Combining and placing solids: the solid half of UNION, SUBTRACT and INTERSECT ([`combine`]),
//! 3DMOVE, 3DROTATE, MIRROR3D and SLICE.
//!
//! 3D solids move in 3D. Other objects are 2D objects in plan, so they follow a transform only
//! when it keeps them in plan (a rotation about Z, a mirror plane standing upright); they move by
//! the transform's plan part, and are skipped otherwise.

use cadcraft_doc::{EntityKind, Handle, Solid3d};
use cadcraft_geom::solid::{BoolOp, Mesh, boolean, primitives};
use cadcraft_geom::{Mat3, Mat4, Vec3, rad};
use serde_json::{Value, json};

use super::super::{CommandSpec, bad, bool_or, curves, str_param, targets};
use super::{Ask, SelectAsk, hex, number, point3, solid_error, solids_in};
use crate::{Result, Session};

pub fn specs() -> Vec<CommandSpec> {
    const MOVE: &[Ask] = &[Ask::Point("Specify base point", "from"), Ask::Point("Specify second point (x,y,z)", "to")];
    const ROTATE: &[Ask] = &[
        Ask::Point("Specify base point", "base"),
        Ask::Keyword("Specify rotation axis", "axis", &["Z", "X", "Y"]),
        Ask::Number("Specify angle", "angle"),
    ];
    const MIRROR: &[Ask] = &[
        Ask::Keyword("Specify mirroring plane", "plane", &["YZ", "ZX", "XY"]),
        Ask::Point("Specify point on the plane", "point"),
        Ask::Keyword("Delete source objects?", "erase", &["No", "Yes"]),
    ];
    const ARRAY: &[Ask] = &[
        Ask::Number("Enter the number of rows (---)", "rows"),
        Ask::Number("Enter the number of columns (|||)", "columns"),
        Ask::Number("Enter the number of levels (...)", "levels"),
        Ask::Number("Specify the distance between rows (---)", "rowSpacing"),
        Ask::Number("Specify the distance between columns (|||)", "columnSpacing"),
        Ask::Number("Specify the distance between levels (...)", "levelSpacing"),
    ];
    const ALIGN: &[Ask] = &[
        Ask::Point("Specify base point", "from1"),
        Ask::Point("Specify second point", "from2"),
        Ask::Point("Specify third point", "from3"),
        Ask::Point("Specify first destination point", "to1"),
        Ask::Point("Specify second destination point", "to2"),
        Ask::Point("Specify third destination point", "to3"),
    ];
    const SLICE: &[Ask] = &[
        Ask::Keyword("Specify slicing plane", "plane", &["XY", "YZ", "ZX"]),
        Ask::Point("Specify point on the plane", "point"),
        Ask::Keyword("Keep", "keep", &["Both", "Above", "Below"]),
    ];
    vec![
        CommandSpec::new("3dmove", "3D Move", run_3dmove)
            .menu(&["Modify", "3D Operations", "3D Move"])
            .alias(&["3m"])
            .params("{handles?, from: [x,y,z], to: [x,y,z]} | {handles?, displacement: [x,y,z]} → {moved, skipped}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("3dmove", "3DMOVE", MOVE)))),
        CommandSpec::new("3drotate", "3D Rotate", run_3drotate)
            .menu(&["Modify", "3D Operations", "3D Rotate"])
            .alias(&["3r"])
            .params("{handles?, base?: [x,y,z] (default 0,0,0), axis: \"x\"|\"y\"|\"z\"|[x,y,z] (default z), angle: degrees (counterclockwise looking down the axis)} → {moved, skipped}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("3drotate", "3DROTATE", ROTATE)))),
        CommandSpec::new("mirror3d", "3D Mirror", run_mirror3d)
            .menu(&["Modify", "3D Operations", "3D Mirror"])
            .params("{handles?, plane: \"xy\"|\"yz\"|\"zx\" | normal: [x,y,z], point?: [x,y,z] (on the plane, default 0,0,0), erase?: false (keep the originals)} → {handles, skipped}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("mirror3d", "MIRROR3D", MIRROR)))),
        CommandSpec::new("3darray", "3D Array", run_3darray)
            .menu(&["Modify", "3D Operations", "3D Array"])
            .alias(&["3a"])
            .params("{handles?, rows?: 1, columns?: 1, levels?: 1, spacing?: [dx (columns), dy (rows), dz (levels)] (or rowSpacing, columnSpacing, levelSpacing)} | {handles?, polar: true, count, angle?: 360 (degrees to fill), axis?: [[x,y,z], [x,y,z]] (default the Z axis through 0,0,0), rotate?: true} → {handles (the copies), skipped}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("3darray", "3DARRAY", ARRAY)))),
        CommandSpec::new("3dalign", "3D Align", run_3dalign)
            .menu(&["Modify", "3D Operations", "3D Align"])
            .alias(&["3al"])
            .params("{handles?, from: [p1, p2?, p3?], to: [q1, q2?, q3?]} (or from1..3, to1..3): moves p1 to q1, turns p1→p2 along q1→q2, and the plane of p1, p2, p3 onto that of q1, q2, q3 → {moved, skipped}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("3dalign", "3DALIGN", ALIGN)))),
        CommandSpec::new("slice", "Slice", run_slice)
            .menu(&["Modify", "3D Operations", "Slice"])
            .alias(&["sl"])
            .params("{handles?: solids, plane: \"xy\"|\"yz\"|\"zx\" | normal: [x,y,z], point?: [x,y,z], keep?: \"both\"|\"above\"|\"below\" (above: the side the normal points to)} → {handles}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("slice", "SLICE", SLICE)))),
    ]
}

/// UNION, SUBTRACT or INTERSECT of the 3D solids among `from` (and, for SUBTRACT, `remove`):
/// the first solid takes the result, keeping its handle and properties, and the others are
/// erased. `None` when none of them is a solid (the caller combines regions).
pub(crate) fn combine(s: &mut Session, cmd: &str, from: &[Handle], remove: &[Handle], op: BoolOp) -> Result<Option<Value>> {
    let (a, b) = (solids_in(s, from)?, solids_in(s, remove)?);
    if a.is_empty() && b.is_empty() {
        return Ok(None);
    }
    let Some(((h0, first), rest)) = a.split_first() else { return Err(bad(cmd, "select solids to subtract from")) };
    let mut acc: Mesh = first.mesh().clone();
    let gather = if op == BoolOp::Intersection { BoolOp::Intersection } else { BoolOp::Union };
    for (_, so) in rest {
        acc = boolean(&acc, so.mesh(), gather).map_err(|e| solid_error(cmd, e))?;
    }
    if op == BoolOp::Difference {
        for (h, so) in &b {
            if a.iter().any(|(x, _)| x == h) {
                return Err(bad(cmd, "a solid can't be subtracted from itself"));
            }
            acc = boolean(&acc, so.mesh(), BoolOp::Difference).map_err(|e| solid_error(cmd, e))?;
        }
    }
    let d = s.doc_mut()?;
    for (h, _) in a.iter().chain(&b).filter(|(h, _)| h != h0) {
        d.remove_entity(*h);
    }
    if acc.is_empty() {
        d.remove_entity(*h0);
        return Ok(Some(json!({ "handle": Value::Null, "volume": 0.0, "message": "The result is empty: the solids were erased." })));
    }
    let closed = acc.check().is_closed();
    let volume = acc.volume();
    d.modify_entity(*h0, |e| e.kind = EntityKind::Solid3d(Solid3d::new(acc)))?;
    let message = if closed { String::new() } else { "The result touches itself along an edge or at a point and is not a closed solid.".into() };
    Ok(Some(json!({ "handle": hex(*h0), "volume": volume, "closed": closed, "message": message })))
}

/// The plan part of `m` when it keeps objects in plan (Z stays Z, up stays up).
fn plan_part(m: &Mat4) -> Option<Mat3> {
    let r = &m.m;
    let flat = [r[0][2], r[1][2], r[2][0], r[2][1]].iter().all(|x| x.abs() < 1e-12) && r[2][2] > 0.0;
    flat.then(|| Mat3 { a: r[0][0], b: r[1][0], c: r[0][1], d: r[1][1], e: r[0][3], f: r[1][3] })
}

/// Raise `k` by `dz`; false for an object without a height of its own (splines, regions…).
fn lift(k: &mut EntityKind, dz: f64) -> bool {
    let up = |p: &mut Vec3| p.z += dz;
    match k {
        EntityKind::Line(l) => {
            up(&mut l.a);
            up(&mut l.b);
        }
        EntityKind::Point(p) => up(&mut p.p),
        EntityKind::Circle(c) => up(&mut c.center),
        EntityKind::Arc(a) => up(&mut a.center),
        EntityKind::Ellipse(e) => up(&mut e.center),
        EntityKind::Helix(h) => up(&mut h.center),
        EntityKind::LwPolyline(p) => p.elevation += dz,
        EntityKind::Polyline3d(p) => p.points.iter_mut().for_each(up),
        EntityKind::Hatch(h) => h.elevation += dz,
        EntityKind::Text(t) => {
            up(&mut t.insert);
            t.align_pt.iter_mut().for_each(up);
        }
        EntityKind::MText(t) => up(&mut t.insert),
        EntityKind::Solid(so) | EntityKind::Trace(so) => so.corners.iter_mut().for_each(up),
        EntityKind::Face3d(f) => f.corners.iter_mut().for_each(up),
        _ => return false,
    }
    true
}

/// `kind` transformed by `m`, or `None` for an object `m` would take out of plan (or a solid
/// out of range). A move straight up or down raises the objects that have a height.
fn transformed(kind: &EntityKind, m: &Mat4) -> Option<EntityKind> {
    let mut k = kind.clone();
    match &mut k {
        EntityKind::Solid3d(so) => so.transform(m).then_some(())?,
        other => {
            other.transform(&plan_part(m)?);
            let (dz, scale_z) = (m.m[2][3], m.m[2][2]);
            if dz != 0.0 && (scale_z != 1.0 || !lift(other, dz)) {
                return None;
            }
        }
    }
    Some(k)
}

/// Transform the objects `hs` (skipping locked layers and objects `m` takes out of plan).
fn apply(s: &mut Session, hs: &[Handle], m: &Mat4) -> Result<Value> {
    let (mut moved, mut skipped) = (0, 0);
    for h in hs {
        let e = curves::entity(s, *h)?;
        match transformed(&e.kind, m).filter(|_| !curves::is_locked(s, *h)) {
            Some(k) => {
                s.doc_mut()?.modify_entity(*h, |e| e.kind = k)?;
                moved += 1;
            }
            None => skipped += 1,
        }
    }
    Ok(json!({ "moved": moved, "skipped": skipped }))
}

fn run_3dmove(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let d = match (point3("3dmove", p, "displacement")?, point3("3dmove", p, "from")?, point3("3dmove", p, "to")?) {
        (Some(d), _, _) => d,
        (None, Some(a), Some(b)) => b - a,
        _ => return Err(bad("3dmove", "`displacement` or `from` and `to` are required")),
    };
    apply(s, &hs, &Mat4::translate(d))
}

/// `axis`: "x", "y", "z" or a direction (default Z).
fn axis(cmd: &str, p: &Value) -> Result<Vec3> {
    match p.get("axis") {
        None | Some(Value::Null) => Ok(Vec3::Z),
        Some(Value::String(a)) => match a.trim().to_ascii_lowercase().as_str() {
            "x" => Ok(Vec3::X),
            "y" => Ok(Vec3::Y),
            "z" => Ok(Vec3::Z),
            _ => Err(bad(cmd, "`axis` must be \"x\", \"y\", \"z\" or a direction [x,y,z]")),
        },
        Some(_) => point3(cmd, p, "axis")?.filter(|v| v.len() > 1e-12).ok_or_else(|| bad(cmd, "`axis` must be a non-zero direction")),
    }
}

fn run_3drotate(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let base = point3("3drotate", p, "base")?.unwrap_or(Vec3::ZERO);
    let dir = axis("3drotate", p)?;
    let angle = number("3drotate", p, "angle")?.ok_or_else(|| bad("3drotate", "`angle` (degrees) is required"))?;
    apply(s, &hs, &Mat4::rotate_about_axis(base, dir, rad(angle)))
}

/// Copies of `hs` placed by each of `placements` (objects a placement takes out of plan are
/// skipped).
fn copies(s: &mut Session, cmd: &str, hs: &[Handle], placements: &[Mat4]) -> Result<Value> {
    if hs.len().saturating_mul(placements.len()) > curves::MAX_GEN {
        return Err(bad(cmd, format!("more than {} copies", curves::MAX_GEN)));
    }
    let (mut made, mut skipped) = (Vec::new(), 0);
    let space = s.space();
    for h in hs {
        let e = curves::entity(s, *h)?;
        for m in placements {
            match transformed(&e.kind, m) {
                Some(k) => {
                    let mut common = e.common.clone();
                    common.xdata.clear();
                    made.push(hex(s.doc_mut()?.add(&space, common, k)?));
                }
                None => skipped += 1,
            }
        }
    }
    Ok(json!({ "handles": made, "skipped": skipped }))
}

/// A whole number of copies `key` (at least 1).
fn count(cmd: &str, p: &Value, key: &str, default: u64) -> Result<u64> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v
            .as_u64()
            .or_else(|| v.as_f64().filter(|f| f.fract() == 0.0 && *f >= 1.0 && *f <= 1e6).map(|f| f as u64))
            .filter(|n| *n >= 1)
            .ok_or_else(|| bad(cmd, format!("`{key}` must be a whole number of at least 1"))),
    }
}

fn run_3darray(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "3darray";
    let hs = targets(s, p)?;
    let mut placements = Vec::new();
    if bool_or(p, "polar", false) {
        let n = count(CMD, p, "count", 0)?;
        let fill = rad(number(CMD, p, "angle")?.unwrap_or(360.0));
        let (a, b) = match p
            .get("axis")
            .and_then(Value::as_array)
            .map(|a| (a.first().and_then(super::point3_value), a.get(1).and_then(super::point3_value)))
        {
            None => (Vec3::ZERO, Vec3::Z),
            Some((Some(a), Some(b))) if a.dist(b) > 1e-12 => (a, b),
            Some(_) => return Err(bad(CMD, "`axis` must be two different points [[x,y,z], [x,y,z]]")),
        };
        // A full turn spaces the items evenly round it; a part turn puts the last at its end.
        let step = if (fill.abs() - std::f64::consts::TAU).abs() < 1e-9 { fill / n as f64 } else { fill / (n.max(2) - 1) as f64 };
        let rotate = bool_or(p, "rotate", true);
        for k in 1..n {
            let turn = Mat4::rotate_about_axis(a, b - a, step * k as f64);
            placements.push(if rotate { turn } else { Mat4::IDENTITY });
        }
        if !rotate {
            // Each copy moves with the middle of its object, without turning.
            let mut out = Vec::new();
            for h in &hs {
                let e = curves::entity(s, *h)?;
                let middle = match &e.kind {
                    EntityKind::Solid3d(so) => so.bounds().center(),
                    _ => cadcraft_doc::entity_bounds(s.doc()?, &e, 0).center().to3(0.0),
                };
                let moves: Vec<Mat4> =
                    (1..n).map(|k| Mat4::translate(Mat4::rotate_about_axis(a, b - a, step * k as f64).apply(middle) - middle)).collect();
                let r = copies(s, CMD, &[*h], &moves)?;
                out.extend(r.get("handles").and_then(Value::as_array).cloned().unwrap_or_default());
            }
            return Ok(json!({ "handles": out, "skipped": 0 }));
        }
    } else {
        let (rows, cols, levels) = (count(CMD, p, "rows", 1)?, count(CMD, p, "columns", 1)?, count(CMD, p, "levels", 1)?);
        let spacing = point3(CMD, p, "spacing")?.unwrap_or_else(|| {
            let g = |k: &str| p.get(k).and_then(Value::as_f64).unwrap_or(0.0);
            Vec3::new(g("columnSpacing"), g("rowSpacing"), g("levelSpacing"))
        });
        if rows.saturating_mul(cols).saturating_mul(levels) > curves::MAX_GEN as u64 {
            return Err(bad(CMD, format!("more than {} copies", curves::MAX_GEN)));
        }
        for l in 0..levels {
            for r in 0..rows {
                for c in 0..cols {
                    if (l, r, c) != (0, 0, 0) {
                        placements.push(Mat4::translate(Vec3::new(spacing.x * c as f64, spacing.y * r as f64, spacing.z * l as f64)));
                    }
                }
            }
        }
    }
    copies(s, CMD, &hs, &placements)
}

/// Up to three points `key` (`[p1, p2?, p3?]`, or `key1`..`key3`).
fn align_points(cmd: &str, p: &Value, key: &str) -> Result<Vec<Vec3>> {
    let listed = p.get(key).and_then(Value::as_array).map(|a| a.iter().map(super::point3_value).collect::<Option<Vec<_>>>());
    let pts = match listed {
        Some(pts) => pts.ok_or_else(|| bad(cmd, format!("`{key}` must be points [x, y, z]")))?,
        None => (1..=3).map(|i| point3(cmd, p, &format!("{key}{i}"))).collect::<Result<Vec<_>>>()?.into_iter().flatten().collect(),
    };
    if pts.is_empty() || pts.len() > 3 {
        return Err(bad(cmd, format!("`{key}` must be one to three points")));
    }
    Ok(pts)
}

/// An orthonormal frame from up to three points: along p1→p2, then towards p3. `None` when
/// the points repeat or line up.
fn frame(pts: &[Vec3]) -> Option<[Vec3; 3]> {
    let (p1, p2, p3) = (pts.first()?, pts.get(1)?, pts.get(2)?);
    let e1 = (*p2 - *p1).normalized();
    let t = *p3 - *p1;
    let e2 = (t - e1 * t.dot(e1)).normalized();
    (e1 != Vec3::ZERO && e2 != Vec3::ZERO).then(|| [e1, e2, e1.cross(e2)])
}

fn run_3dalign(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "3dalign";
    let hs = targets(s, p)?;
    let (from, to) = (align_points(CMD, p, "from")?, align_points(CMD, p, "to")?);
    if from.len() != to.len() {
        return Err(bad(CMD, "`from` and `to` need as many points"));
    }
    let (Some(p1), Some(q1)) = (from.first().copied(), to.first().copied()) else { return Err(bad(CMD, "`from` and `to` are required")) };
    // The turn: the least taking p1→p2 along q1→q2, or one frame onto the other.
    let turn = match (from.len(), frame(&from), frame(&to)) {
        (1, ..) => Mat4::IDENTITY,
        (2, ..) => {
            let (a, b) = (from.get(1).map_or(Vec3::ZERO, |v| (*v - p1).normalized()), to.get(1).map_or(Vec3::ZERO, |v| (*v - q1).normalized()));
            if a == Vec3::ZERO || b == Vec3::ZERO {
                return Err(bad(CMD, "the first and second points must differ"));
            }
            let axis = a.cross(b);
            if axis.len() < 1e-12 {
                if a.dot(b) > 0.0 {
                    Mat4::IDENTITY
                } else {
                    Mat4::rotate_about_axis(Vec3::ZERO, if a.x.abs() < 0.9 { a.cross(Vec3::X) } else { a.cross(Vec3::Y) }, std::f64::consts::PI)
                }
            } else {
                Mat4::rotate_about_axis(Vec3::ZERO, axis, a.dot(b).clamp(-1.0, 1.0).acos())
            }
        }
        (_, Some(f), Some(g)) => {
            let mut m = Mat4::IDENTITY;
            for (i, row) in m.m.iter_mut().take(3).enumerate() {
                for (j, cell) in row.iter_mut().take(3).enumerate() {
                    *cell = (0..3).map(|k| comp(g[k], i) * comp(f[k], j)).sum();
                }
            }
            m
        }
        _ => return Err(bad(CMD, "the three points must not repeat or line up")),
    };
    let m = Mat4::translate(p1 * -1.0).then(&turn).then(&Mat4::translate(q1));
    apply(s, &hs, &m)
}

fn comp(v: Vec3, i: usize) -> f64 {
    match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// The plane `{plane: "xy"|"yz"|"zx" | normal: [x,y,z], point?}` as a point and unit normal.
fn plane(cmd: &str, p: &Value) -> Result<(Vec3, Vec3)> {
    let point = point3(cmd, p, "point")?.unwrap_or(Vec3::ZERO);
    let normal = match (str_param(p, "plane").map(|n| n.trim().to_ascii_lowercase()), point3(cmd, p, "normal")?) {
        (Some(n), _) if n == "xy" => Vec3::Z,
        (Some(n), _) if n == "yz" => Vec3::X,
        (Some(n), _) if n == "zx" || n == "xz" => Vec3::Y,
        (None, Some(n)) if n.len() > 1e-12 => n.normalized(),
        _ => return Err(bad(cmd, "`plane` (\"xy\", \"yz\" or \"zx\") or a non-zero `normal` [x,y,z] is required")),
    };
    Ok((point, normal))
}

fn run_mirror3d(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let (point, normal) = plane("mirror3d", p)?;
    let m = Mat4::mirror_plane(point, normal);
    let erase = bool_or(p, "erase", false) || str_param(p, "erase").is_some_and(|v| v.eq_ignore_ascii_case("yes"));
    if erase {
        return apply(s, &hs, &m);
    }
    let (mut made, mut skipped) = (Vec::new(), 0);
    let space = s.space();
    for h in &hs {
        let e = curves::entity(s, *h)?;
        match transformed(&e.kind, &m) {
            Some(k) => {
                let mut common = e.common.clone();
                common.xdata.clear();
                made.push(hex(s.doc_mut()?.add(&space, common, k)?));
            }
            None => skipped += 1,
        }
    }
    Ok(json!({ "handles": made, "skipped": skipped }))
}

/// A box filling the half-space in front of the plane through `point` with unit normal `n`, out
/// to `reach` in every direction.
fn half_space(point: Vec3, n: Vec3, reach: f64) -> std::result::Result<Mesh, cadcraft_geom::solid::SolidError> {
    let u = if n.x.abs() < 0.9 { Vec3::X.cross(n) } else { Vec3::Y.cross(n) }.normalized();
    let v = n.cross(u);
    let frame = Mat4 { m: [[u.x, v.x, n.x, point.x], [u.y, v.y, n.y, point.y], [u.z, v.z, n.z, point.z], [0.0, 0.0, 0.0, 1.0]] };
    Ok(primitives::cuboid(Vec3::new(-reach, -reach, 0.0), Vec3::new(reach, reach, reach))?.transformed(&frame))
}

fn run_slice(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let (point, normal) = plane("slice", p)?;
    let keep = str_param(p, "keep").map_or("both".to_string(), |k| k.trim().to_ascii_lowercase());
    if !["both", "above", "below"].contains(&keep.as_str()) {
        return Err(bad("slice", "`keep` must be \"both\", \"above\" or \"below\""));
    }
    let mut solids = solids_in(s, &hs)?;
    solids.retain(|(h, _)| !curves::is_locked(s, *h));
    if solids.is_empty() {
        return Err(bad("slice", "select 3D solids to slice"));
    }
    // Cut every solid before changing the drawing, so a failure leaves them all as they were.
    let mut cuts = Vec::with_capacity(solids.len());
    for (h, so) in solids {
        let b = so.bounds();
        let reach = 2.0 * (b.size().len() + b.center().dist(point)) + 1.0;
        let cut = half_space(point, normal, reach).map_err(|e| solid_error("slice", e))?;
        let above =
            if keep != "below" { boolean(so.mesh(), &cut, BoolOp::Intersection).map_err(|e| solid_error("slice", e))? } else { Mesh::default() };
        let below =
            if keep != "above" { boolean(so.mesh(), &cut, BoolOp::Difference).map_err(|e| solid_error("slice", e))? } else { Mesh::default() };
        cuts.push((h, curves::entity(s, h)?.common, [above, below]));
    }
    let mut out = Vec::new();
    let space = s.space();
    let d = s.doc_mut()?;
    for (h, common, pieces) in cuts {
        d.remove_entity(h);
        for piece in pieces.into_iter().filter(|m| !m.is_empty()) {
            out.push(hex(d.add(&space, common.clone(), EntityKind::Solid3d(Solid3d::new(piece)))?));
        }
    }
    Ok(json!({ "handles": out }))
}
