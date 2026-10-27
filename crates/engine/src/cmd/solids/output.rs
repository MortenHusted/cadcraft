//! Measuring and checking solids, and writing them for 3D printing: the solid half of MASSPROP
//! ([`massprop`]), INTERFERE, STLOUT and 3DPRINT.
//!
//! STLOUT and 3DPRINT write the selected solids through the host's io hooks (the writers live in
//! the io layer), never opening a dialog from the JSON form.

use cadcraft_doc::{Common, Drawing, EntityKind, Handle, Solid3d, Space};
use cadcraft_geom::solid::{BoolOp, boolean};
use serde_json::{Value, json};

use super::super::machines::SelectRun;
use super::super::{CommandSpec, bad, curves, str_param, targets};
use super::{Ask, SelectAsk, hex, solid_error, solids_in};
use crate::{Result, Session};

/// Interference below this fraction of the smaller solid's volume is rounding, not contact.
const MIN_INTERFERENCE: f64 = 1e-9;

pub fn specs() -> Vec<CommandSpec> {
    const PATH: &[Ask] = &[Ask::Text("Enter file name", "path")];
    vec![
        CommandSpec::new("interfere", "Interference Checking", run_interfere)
            .menu(&["Modify", "3D Operations", "Interference Checking"])
            .alias(&["inf"])
            .params("{first?: [handles] (default: the selection), second?: [handles] (default: check the first set against itself), create?: false (add each interference as a solid)} → {pairs: [{a, b, volume, handle?}], count}")
            .interactive(|_| Ok(Box::new(SelectRun::new("interfere", "INTERFERE")))),
        CommandSpec::new("render", "Render", run_render)
            .alias(&["rr"])
            .params("{handles?: solids (default: every visible solid in model space), view?: top|bottom|front|back|left|right|se|sw|ne|nw (default se) | [x,y,z] (towards the viewer), width?: 1200, height?: 900, edges?: true, path?: \"NAME.png\" (else base64 `data`)} → a shaded PNG of the solids")
            .noundo(),
        CommandSpec::new("stlout", "STL Out", run_stlout)
            .params("{handles?: solids (default: the selection), path: \"NAME.stl\"} → {path, bytes, solids, open}: binary STL")
            .noundo()
            .interactive(|_| Ok(Box::new(SelectAsk::new("stlout", "STLOUT", PATH)))),
        CommandSpec::new("3dprint", "3D Print", run_3dprint)
            .menu(&["File", "3D Print..."])
            .params("{handles?: solids (default: the selection), path: \"NAME.3mf\" | \"NAME.stl\"} → {path, bytes, solids, open}: a 3MF (one object per solid, in the drawing's units) or binary STL for a slicer")
            .noundo()
            .interactive(|_| Ok(Box::new(SelectAsk::new("3dprint", "3DPRINT", PATH)))),
    ]
}

/// MASSPROP for the 3D solids among `hs` (unit density, as AutoCAD reports it): one JSON object
/// per solid and the report text.
pub(crate) fn massprop(s: &Session, hs: &[Handle]) -> Result<(Vec<Value>, Vec<String>)> {
    let f = |v: f64| super::super::inquiry::fmt_d(s, v);
    let (mut out, mut lines) = (Vec::new(), Vec::new());
    for (h, so) in solids_in(s, hs)? {
        let m = so.mass_props();
        let b = so.bounds();
        let c = m.centroid;
        let gyr = |i: f64| if m.volume > 0.0 { (i / m.volume).abs().sqrt() } else { 0.0 };
        let [ix, iy, iz] = m.moments;
        let [pxy, pyz, pzx] = m.products;
        let closed = so.mesh().check().is_closed();
        out.push(json!({
            "handle": hex(h), "mass": m.volume, "volume": m.volume, "area": m.area,
            "boundingBox": [[b.min.x, b.min.y, b.min.z], [b.max.x, b.max.y, b.max.z]],
            "centroid": [c.x, c.y, c.z], "momentsOfInertia": m.moments, "productsOfInertia": m.products,
            "radiiOfGyration": [gyr(ix), gyr(iy), gyr(iz)], "centroidInertia": m.centroid_inertia, "closed": closed,
        }));
        lines.push(format!(
            "----------------   SOLIDS    ----------------\nMass:                    {}\nVolume:                  {}\nBounding box:         X: {}  --  {}\n                      Y: {}  --  {}\n                      Z: {}  --  {}\nCentroid:             X: {}\n                      Y: {}\n                      Z: {}\nMoments of inertia:   X: {}\n                      Y: {}\n                      Z: {}\nProducts of inertia: XY: {}\n                     YZ: {}\n                     ZX: {}\nRadii of gyration:    X: {}\n                      Y: {}\n                      Z: {}{}",
            f(m.volume),
            f(m.volume),
            f(b.min.x),
            f(b.max.x),
            f(b.min.y),
            f(b.max.y),
            f(b.min.z),
            f(b.max.z),
            f(c.x),
            f(c.y),
            f(c.z),
            f(ix),
            f(iy),
            f(iz),
            f(pxy),
            f(pyz),
            f(pzx),
            f(gyr(ix)),
            f(gyr(iy)),
            f(gyr(iz)),
            if closed { "" } else { "\nThe surface is not closed: the volume is not reliable." },
        ));
    }
    Ok((out, lines))
}

/// Handles from `key` (an array), or `None` when absent.
fn handle_list(p: &Value, key: &str) -> Result<Option<Vec<Handle>>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Array(a)) => a
            .iter()
            .map(|v| curves::handle_value(v).ok_or_else(|| bad("interfere", format!("`{key}`: {v} is not a handle"))))
            .collect::<Result<Vec<_>>>()
            .map(Some),
        Some(_) => Err(bad("interfere", format!("`{key}` must be an array of handles"))),
    }
}

fn run_interfere(s: &mut Session, p: &Value) -> Result<Value> {
    let first = match handle_list(p, "first")? {
        Some(hs) => hs,
        None => targets(s, p)?,
    };
    let a = solids_in(s, &first)?;
    let b = match handle_list(p, "second")? {
        Some(hs) => solids_in(s, &hs)?,
        None => Vec::new(),
    };
    if a.is_empty() {
        return Err(bad("interfere", "select 3D solids to check"));
    }
    // Each pair once: the first set against the second, or the first set against itself.
    let mut pairs: Vec<(&(Handle, Solid3d), &(Handle, Solid3d))> = Vec::new();
    if b.is_empty() {
        for (i, x) in a.iter().enumerate() {
            pairs.extend(a.iter().skip(i + 1).map(|y| (x, y)));
        }
    } else {
        pairs.extend(a.iter().flat_map(|x| b.iter().filter(move |y| y.0 != x.0).map(move |y| (x, y))));
    }
    let create = super::super::bool_or(p, "create", false);
    let mut found = Vec::new();
    let mut made: Vec<cadcraft_geom::solid::Mesh> = Vec::new();
    for ((ha, sa), (hb, sb)) in pairs {
        if !sa.bounds().intersects(&sb.bounds()) {
            continue;
        }
        let common = boolean(sa.mesh(), sb.mesh(), BoolOp::Intersection).map_err(|e| solid_error("interfere", e))?;
        let volume = common.volume();
        if volume <= MIN_INTERFERENCE * sa.mass_props().volume.abs().min(sb.mass_props().volume.abs()) {
            continue;
        }
        found.push(json!({ "a": hex(*ha), "b": hex(*hb), "volume": volume }));
        if create {
            made.push(common);
        }
    }
    for (pair, mesh) in found.iter_mut().zip(made) {
        let h = super::add_solid(s, mesh)?;
        if let Some(o) = pair.as_object_mut() {
            o.insert("handle".into(), hex(h));
        }
    }
    let count = found.len();
    let message = if count == 0 { "No interference found.".to_string() } else { format!("{count} interfering pair(s) found.") };
    Ok(json!({ "pairs": found, "count": count, "message": message }))
}

/// Write the selected solids to `path` (the io layer picks STL or 3MF from its extension).
fn write_solids(s: &mut Session, cmd: &str, p: &Value, formats: &[&str]) -> Result<Value> {
    let path = str_param(p, "path").map(str::trim).filter(|x| !x.is_empty()).ok_or_else(|| bad(cmd, "`path` is required"))?;
    let ext = std::path::Path::new(path).extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
    if !formats.contains(&ext.as_str()) {
        return Err(bad(cmd, format!("`path` must end in .{}", formats.join(" or ."))));
    }
    let solids = solids_in(s, &targets(s, p)?)?;
    if solids.is_empty() {
        return Err(bad(cmd, "select 3D solids to write"));
    }
    let hooks = super::super::file::io().ok_or_else(|| bad(cmd, "file formats are not available in this build"))?;
    // Only the selected solids, in the drawing's units.
    let src = s.doc()?;
    let mut d = Drawing::new_metric();
    d.header.set_i64("INSUNITS", src.header.i64("INSUNITS", 4));
    let open: Vec<Value> = solids.iter().filter(|(_, so)| !so.mesh().check().is_closed()).map(|(h, _)| hex(*h)).collect();
    for (h, so) in &solids {
        let layer = src.entity(*h).map_or_else(|| "0".to_string(), |e| e.common.layer.clone());
        d.add(&Space::Model, Common { layer, ..Common::default() }, EntityKind::Solid3d(so.clone()))?;
    }
    let bytes = (hooks.write)(&d, path).map_err(|e| bad(cmd, e))?;
    super::super::exchange::write_file(cmd, path, &bytes)?;
    let mut message = format!("Wrote {} solid(s) to {path}.", solids.len());
    if !open.is_empty() {
        message.push_str(&format!(" {} of them not closed (a slicer may need to repair them).", open.len()));
    }
    Ok(json!({ "path": path, "bytes": bytes.len(), "solids": solids.len(), "open": open, "message": message }))
}

/// The direction a named view looks from (AutoCAD's preset 3D views), or a direction `[x,y,z]`.
fn view_direction(p: &Value) -> Result<(String, cadcraft_geom::Vec3)> {
    use cadcraft_geom::Vec3;
    let named = |n: &str| -> Option<Vec3> {
        Some(match n {
            "top" => Vec3::Z,
            "bottom" => -Vec3::Z,
            "front" => -Vec3::Y,
            "back" => Vec3::Y,
            "left" => -Vec3::X,
            "right" => Vec3::X,
            "se" | "seiso" => Vec3::new(1.0, -1.0, 1.0),
            "sw" | "swiso" => Vec3::new(-1.0, -1.0, 1.0),
            "ne" | "neiso" => Vec3::new(1.0, 1.0, 1.0),
            "nw" | "nwiso" => Vec3::new(-1.0, 1.0, 1.0),
            _ => return None,
        })
    };
    match p.get("view") {
        None | Some(Value::Null) => Ok(("se".into(), Vec3::new(1.0, -1.0, 1.0))),
        Some(Value::String(n)) => {
            let n = n.trim().to_ascii_lowercase();
            let d = named(&n)
                .ok_or_else(|| bad("render", "`view` must be top, bottom, front, back, left, right, se, sw, ne, nw or a direction [x,y,z]"))?;
            Ok((n, d))
        }
        Some(v) => {
            let d = super::point3_value(v).filter(|d| d.len() > 1e-12).ok_or_else(|| bad("render", "`view` must be a non-zero direction [x,y,z]"))?;
            Ok((format!("[{}, {}, {}]", d.x, d.y, d.z), d))
        }
    }
}

/// A solid's display colour: its own or its layer's; white (ACI 7) and ByBlock as light grey,
/// which reads on the white background.
fn solid_rgb(d: &Drawing, c: &Common) -> cadcraft_color::Rgb {
    use cadcraft_color::{Color, Rgb, aci_rgb};
    let color = match c.color {
        Color::ByLayer => d.layer(&c.layer).map_or(Color::WHITE, |l| l.color),
        other => other,
    };
    match color {
        Color::True(rgb) => rgb,
        Color::Index(i) if i != 7 => aci_rgb(i),
        _ => Rgb(196, 202, 212),
    }
}

fn run_render(s: &mut Session, p: &Value) -> Result<Value> {
    use cadcraft_render::shade::{MAX_SIDE, ShadeItem, ShadeOptions, shade_png};
    let (name, view) = view_direction(p)?;
    let side = |key: &str, default: u32| -> Result<u32> {
        match p.get(key) {
            None | Some(Value::Null) => Ok(default),
            Some(v) => v
                .as_u64()
                .and_then(|n| u32::try_from(n).ok())
                .filter(|n| (16..=MAX_SIDE).contains(n))
                .ok_or_else(|| bad("render", format!("`{key}` must be 16 to {MAX_SIDE} pixels"))),
        }
    };
    let (width, height) = (side("width", 1200)?, side("height", 900)?);
    let hs = match p.get("handles") {
        None | Some(Value::Null) => s.doc()?.model.iter().map(|e| e.handle).collect(),
        Some(_) => targets(s, p)?,
    };
    let d = s.doc()?;
    let solids: Vec<(cadcraft_color::Rgb, Solid3d)> = solids_in(s, &hs)?
        .into_iter()
        .filter_map(|(h, so)| {
            let e = d.entity(h)?;
            let shown = e.common.visible && d.layer(&e.common.layer).is_none_or(|l| l.on && !l.frozen);
            shown.then(|| (solid_rgb(d, &e.common), so))
        })
        .collect();
    if solids.is_empty() {
        return Err(bad("render", "there are no visible 3D solids to render"));
    }
    let items: Vec<ShadeItem> = solids.iter().map(|(color, so)| ShadeItem { mesh: so.mesh(), color: *color }).collect();
    let o = ShadeOptions { width, height, view, edges: super::super::bool_or(p, "edges", true), ..ShadeOptions::default() };
    let png = shade_png(&items, &o).ok_or_else(|| bad("render", "the picture could not be made"))?;
    let out = json!({ "width": width, "height": height, "view": name, "solids": items.len(), "bytes": png.len() });
    match str_param(p, "path").map(str::trim).filter(|x| !x.is_empty()) {
        Some(path) => {
            super::super::exchange::write_file("render", path, &png)?;
            Ok(json!({ "path": path, "message": format!("Rendered {} solid(s) to {path}.", items.len()), "picture": out }))
        }
        None => Ok(json!({ "data": super::super::file::base64_encode(&png), "picture": out })),
    }
}

fn run_stlout(s: &mut Session, p: &Value) -> Result<Value> {
    write_solids(s, "stlout", p, &["stl"])
}

fn run_3dprint(s: &mut Session, p: &Value) -> Result<Value> {
    write_solids(s, "3dprint", p, &["3mf", "stl"])
}
