use super::*;
use cadcraft_geom::{Mat4, Vec3};

// ---------------- core operations ----------------

pub(crate) fn transform_entities(s: &mut Session, hs: &[Handle], m: &Mat3, copy: bool) -> Result<Vec<Handle>> {
    if copy {
        return copy_entities(s, hs, |k| {
            k.transform(m);
            true
        })
        .map(|(made, _)| made);
    }
    let d = s.doc_mut()?;
    let mut out = Vec::new();
    for h in hs {
        if d.space_of(*h).is_none() {
            continue;
        }
        let locked = d.entity(*h).and_then(|e| d.layer(&e.common.layer)).is_some_and(|l| l.locked);
        if locked {
            continue;
        }
        d.modify_entity(*h, |e| e.kind.transform(m))?;
        out.push(*h);
    }
    Ok(out)
}

/// Copies of `hs` in their own spaces, each changed by `place` (false: that one is not copied).
/// The new handles, and how many `place` refused.
fn copy_entities(s: &mut Session, hs: &[Handle], place: impl Fn(&mut EntityKind) -> bool) -> Result<(Vec<Handle>, usize)> {
    let d = s.doc_mut()?;
    let (mut out, mut refused) = (Vec::new(), 0);
    for h in hs {
        let (Some(sp), Some(mut e)) = (d.space_of(*h), d.entity(*h).map(|e| (**e).clone())) else { continue };
        if !place(&mut e.kind) {
            refused += 1;
            continue;
        }
        let nh = d.new_handle();
        if let Some(st) = d.space_mut(&sp) {
            st.push(Entity { handle: nh, ..e });
            out.push(nh);
        }
    }
    Ok((out, refused))
}

pub(super) fn erase(s: &mut Session, hs: &[Handle]) -> Result<usize> {
    let d = s.doc_mut()?;
    let mut n = 0;
    for h in hs {
        let locked = d.entity(*h).and_then(|e| d.layer(&e.common.layer)).is_some_and(|l| l.locked);
        if !locked && d.remove_entity(*h).is_some() {
            n += 1;
        }
    }
    s.set_selection(Vec::new());
    Ok(n)
}

pub(super) fn run_erase(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let n = erase(s, &hs)?;
    Ok(json!({ "erased": n }))
}

pub(super) fn delta_of(cmd: &str, p: &Value) -> Result<Vec2> {
    if let Some(d) = point_param(p, "delta") {
        return Ok(d);
    }
    let a = point_req(cmd, p, "from")?;
    let b = point_req(cmd, p, "to")?;
    Ok(b - a)
}

pub(super) fn run_move(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let d = delta_of("move", p)?;
    let moved = transform_entities(s, &hs, &Mat3::translate(d), false)?;
    Ok(json!({ "moved": moved.len() }))
}

/// COPY's displacement: `delta` ([dx,dy] or [dx,dy,dz]) or `to` − `from` (with or without z).
fn delta3_of(cmd: &str, p: &Value) -> Result<Vec3> {
    let point = |key: &str| p.get(key).and_then(|v| crate::cmd::solids::point3_value(v).or_else(|| crate::cmd::point_value(v).map(|q| q.to3(0.0))));
    if let Some(d) = point("delta") {
        return Ok(d);
    }
    let a = point("from").ok_or_else(|| bad(cmd, "`from` ([x, y] or [x, y, z]) is required"))?;
    let b = point("to").ok_or_else(|| bad(cmd, "`to` ([x, y] or [x, y, z]) is required"))?;
    Ok(b - a)
}

pub(super) fn run_copy(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let d = delta3_of("copy", p)?;
    let n = p.get("count").and_then(Value::as_u64).unwrap_or(1).clamp(1, 10_000);
    let (mut out, mut skipped) = (Vec::new(), 0);
    for i in 1..=n {
        let step = d * i as f64;
        let (made, refused) = if d.z == 0.0 {
            (transform_entities(s, &hs, &Mat3::translate(step.xy()), true)?, 0)
        } else {
            // Up or down too: 3D solids move in 3D and objects with a height of their own are
            // raised or lowered, as 3DMOVE moves them; the others are skipped.
            let m = Mat4::translate(step);
            copy_entities(s, &hs, |k| crate::cmd::solids::transformed(k, &m).map(|t| *k = t).is_some())?
        };
        out.extend(made.into_iter().map(|h| h.hex()));
        skipped += refused;
    }
    Ok(json!({ "handles": out, "skipped": skipped }))
}

pub(super) fn run_rotate(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let base = point_req("rotate", p, "base")?;
    let a = f64_req("rotate", p, "angle")?.to_radians();
    let r = transform_entities(s, &hs, &Mat3::rotate_about(base, a), bool_or(p, "copy", false))?;
    Ok(json!({ "handles": r.iter().map(|h| h.hex()).collect::<Vec<_>>() }))
}

pub(super) fn run_scale(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let base = point_req("scale", p, "base")?;
    let f = f64_req("scale", p, "factor")?;
    if f <= 0.0 {
        return Err(bad("scale", "factor must be positive"));
    }
    let r = transform_entities(s, &hs, &Mat3::scale_about(base, f), bool_or(p, "copy", false))?;
    Ok(json!({ "handles": r.iter().map(|h| h.hex()).collect::<Vec<_>>() }))
}

pub(super) fn run_mirror(s: &mut Session, p: &Value) -> Result<Value> {
    let hs = targets(s, p)?;
    let a = point_req("mirror", p, "p1")?;
    let b = point_req("mirror", p, "p2")?;
    if a.near(b, 1e-12) {
        return Err(bad("mirror", "mirror line points coincide"));
    }
    let erase_src = bool_or(p, "erase", false);
    let r = transform_entities(s, &hs, &Mat3::mirror(a, b), !erase_src)?;
    Ok(json!({ "handles": r.iter().map(|h| h.hex()).collect::<Vec<_>>() }))
}
