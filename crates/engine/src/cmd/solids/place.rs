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
