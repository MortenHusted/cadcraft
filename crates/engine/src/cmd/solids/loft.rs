//! Solids along paths and through cross-sections: SWEEP carries closed profiles along a path
//! (a line, arc, circle, ellipse, polyline, 3D polyline, spline or helix, or points in the JSON
//! form), LOFT joins closed cross-sections one after another, POLYSOLID raises a wall of a width
//! and height along a path. Like EXTRUDE, they erase what made the solid unless `keep` is set.

use cadcraft_doc::{Entity, EntityKind, Handle};
use cadcraft_geom::solid::primitives::{CHORD_TOL, circle_segments};
use cadcraft_geom::solid::sweep::{SweepOptions, loft, sweep};
use cadcraft_geom::{Polyline, TAU, Vec2, Vec3, polygon_centroid, rad};
use serde_json::{Value, json};

use super::super::machines::{SelOutcome, SelectPhase, number as parse_number};
use super::super::{CommandSpec, bad, bool_or, curves, targets};
use super::sweep::profile;
use super::{hex, number, point3_value, solid_error};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

pub fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("sweep", "Sweep", run_sweep)
            .menu(&["Draw", "3D Modeling", "Sweep"])
            .params("{handles?: closed profiles (as EXTRUDE), path: handle (line, arc, circle, ellipse, polyline, 3D polyline, spline, helix) | [[x,y,z], ...], twist?: degrees, scale?: 1 (size at the end), base?: [x,y] (the profile point carried along the path; default its centroid), keep?: false (erase the profiles and path)} → {handles, volumes, skipped}")
            .interactive(|_| Ok(Box::new(SweepM::default()))),
        CommandSpec::new("polysolid", "Polysolid", run_polysolid)
            .menu(&["Draw", "3D Modeling", "Polysolid"])
            .params("{points: [[x,y], ...] | object: handle (a line, arc, circle, polyline or spline), closed?, width?, height? (default PSOLWIDTH, PSOLHEIGHT: 0.25 and 4, or 5 and 80 in a metric drawing), justify?: \"center\" | \"left\" | \"right\" (the side of the path the wall stands on), keep?: false (erase the object)} → {handle, volume}")
            .interactive(|_| Ok(Box::new(PolysolidM::default()))),
        CommandSpec::new("loft", "Loft", run_loft)
            .menu(&["Draw", "3D Modeling", "Loft"])
            .params("{handles: cross-sections in order (closed profiles, each in a plane parallel to XY; a POINT may come first or last), ruled?: false (straight between sections instead of a smooth curve through them), keep?: false (erase the sections)} → {handle, volume}")
            .interactive(|_| Ok(Box::new(LoftM::default()))),
    ]
}

/// Chord tolerance for a curve of `size`.
fn tol(size: f64) -> f64 {
    CHORD_TOL.min(size * 1e-3).max(size * 1e-9)
}

/// The path `e` runs along, as points, and whether it closes.
fn path_of(e: &Entity) -> std::result::Result<(Vec<Vec3>, bool), &'static str> {
    let n = e.common.extrusion;
    let flat = n.x.abs() <= 1e-9 && n.y.abs() <= 1e-9 && n.z > 0.0;
    let at = |pts: Vec<Vec2>, z: f64| pts.into_iter().map(|p| p.to3(z)).collect::<Vec<_>>();
    match &e.kind {
        EntityKind::Line(l) => Ok((vec![l.a, l.b], false)),
        EntityKind::Polyline3d(p) => Ok((p.points.clone(), p.closed)),
        EntityKind::Helix(h) => {
            let h = h.clone().sanitized();
            let k = ((h.turns * circle_segments(h.base_radius.max(h.top_radius)) as f64).ceil() as usize).max(8);
            Ok(((0..=k).map(|i| h.point_at(i as f64 / k as f64)).collect(), false))
        }
        _ if !flat => Err("not in a plane parallel to XY"),
        EntityKind::Arc(a) => {
            let mut pts = Vec::new();
            cadcraft_geom::Arc::new(a.center.xy(), a.radius, a.start, a.end).tessellate(tol(a.radius), &mut pts);
            Ok((at(pts, a.center.z), false))
        }
        EntityKind::Circle(c) => {
            let k = circle_segments(c.radius);
            Ok(((0..k).map(|i| Vec2::polar(c.center.xy(), c.radius, TAU * i as f64 / k as f64).to3(c.center.z)).collect(), true))
        }
        EntityKind::Ellipse(el) => {
            let g = cadcraft_geom::Ellipse { center: el.center.xy(), major: el.major.xy(), ratio: el.ratio, start: el.start, end: el.end };
            let full = (cadcraft_geom::ccw_sweep(el.start, el.end) - TAU).abs() < 1e-9;
            let mut pts = Vec::new();
            g.tessellate(tol(g.bounds().width().max(g.bounds().height())), &mut pts);
            Ok((at(pts, el.center.z), full))
        }
        EntityKind::LwPolyline(p) => {
            let pl = Polyline { vertices: p.vertices.clone(), closed: p.closed };
            let b = pl.bounds();
            Ok((at(pl.tessellate(tol(b.width().max(b.height()))), p.elevation), p.closed))
        }
        EntityKind::Spline(sp) => {
            let b = cadcraft_geom::Bounds2::from_points(sp.control.iter().copied());
            Ok((at(sp.tessellate(tol(b.width().max(b.height()))), 0.0), sp.closed))
        }
        _ => Err("not a path (a line, arc, circle, ellipse, polyline, 3D polyline, spline or helix)"),
    }
}

/// The handle `key` names, if it is one.
fn handle_param(p: &Value, key: &str) -> Option<Handle> {
    p.get(key).and_then(Value::as_str).and_then(Handle::parse_hex)
}

fn run_sweep(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "sweep";
    let path_handle = handle_param(p, "path");
    let (path, closed) = match (p.get("path"), path_handle) {
        (_, Some(h)) => path_of(&curves::entity(s, h)?).map_err(|why| bad(CMD, format!("the path is {why}")))?,
        (Some(Value::Array(pts)), None) => {
            let pts: Option<Vec<Vec3>> = pts.iter().map(point3_value).collect();
            (pts.ok_or_else(|| bad(CMD, "`path` points must be [x, y, z]"))?, false)
        }
        _ => return Err(bad(CMD, "`path` (a handle or a list of points) is required")),
    };
    let opts = SweepOptions { twist: rad(number(CMD, p, "twist")?.unwrap_or(0.0)), scale: number(CMD, p, "scale")?.unwrap_or(1.0) };
    let base = match p.get("base") {
        None | Some(Value::Null) => None,
        Some(v) => Some(point3_value(v).ok_or_else(|| bad(CMD, "`base` must be a point [x, y]"))?.xy()),
    };
    let keep = bool_or(p, "keep", false);
    let hs: Vec<Handle> = targets(s, p)?.into_iter().filter(|h| Some(*h) != path_handle).collect();
    let (mut made, mut volumes, mut skipped) = (Vec::new(), Vec::new(), Vec::new());
    for h in hs {
        let e = curves::entity(s, h)?;
        if curves::is_locked(s, h) {
            skipped.push(json!({ "handle": hex(h), "reason": "on a locked layer" }));
            continue;
        }
        let prof = match profile(s.doc()?, &e) {
            Ok(pr) => pr,
            Err(why) => {
                skipped.push(json!({ "handle": hex(h), "reason": why }));
                continue;
            }
        };
        // The profile's base point rides along the path: by default the centroid of a single
        // outline, the middle of the box round several.
        let middle = || cadcraft_geom::Bounds2::from_points(prof.loops.iter().flatten().copied()).center();
        let origin = base.unwrap_or_else(|| match prof.loops.as_slice() {
            [l] => polygon_centroid(l).unwrap_or_else(middle),
            _ => middle(),
        });
        let loops: Vec<Vec<Vec2>> = prof.loops.iter().map(|l| l.iter().map(|q| *q - origin).collect()).collect();
        let mesh = match sweep(&loops, &path, closed, opts) {
            Ok(m) => m,
            Err(err) => {
                skipped.push(json!({ "handle": hex(h), "reason": err.to_string() }));
                continue;
            }
        };
        volumes.push(mesh.volume());
        let mut common = e.common.clone();
        common.xdata.clear();
        let space = s.space();
        let d = s.doc_mut()?;
        made.push(hex(d.add(&space, common, EntityKind::Solid3d(cadcraft_doc::Solid3d::new(mesh)))?));
        if !keep {
            d.remove_entity(h);
        }
    }
    if made.is_empty() {
        let why = skipped.first().and_then(|x| x.get("reason")).and_then(Value::as_str).unwrap_or("nothing selected");
        return Err(bad(CMD, format!("no solid made ({why}): select closed profiles to sweep")));
    }
    if let (false, Some(h)) = (keep, path_handle) {
        s.doc_mut()?.remove_entity(h);
    }
    Ok(json!({ "handles": made, "volumes": volumes, "skipped": skipped }))
}

/// POLYSOLID's width and height when the call gives none (AutoCAD's PSOLWIDTH and PSOLHEIGHT).
fn wall_size(s: &Session) -> Result<(f64, f64)> {
    let h = &s.doc()?.header;
    let metric = h.i64("MEASUREMENT", 0) == 1;
    Ok((h.f64("PSOLWIDTH", if metric { 5.0 } else { 0.25 }), h.f64("PSOLHEIGHT", if metric { 80.0 } else { 4.0 })))
}

fn run_polysolid(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "polysolid";
    let object = handle_param(p, "object");
    let (path, closed) = match (object, p.get("points").and_then(Value::as_array)) {
        (Some(h), _) => path_of(&curves::entity(s, h)?).map_err(|why| bad(CMD, format!("the object is {why}")))?,
        (None, Some(pts)) => {
            let pts: Option<Vec<Vec3>> = pts.iter().map(point3_value).collect();
            (pts.ok_or_else(|| bad(CMD, "`points` must be [x, y] points"))?, bool_or(p, "closed", false))
        }
        (None, None) => return Err(bad(CMD, "`points` or `object` is required")),
    };
    let (dw, dh) = wall_size(s)?;
    let (w, h) = (number(CMD, p, "width")?.unwrap_or(dw), number(CMD, p, "height")?.unwrap_or(dh));
    if !(w > 0.0 && h > 0.0) {
        return Err(bad(CMD, "`width` and `height` must be positive numbers"));
    }
    // The wall's section across the path: X to the left of it, Y up from it.
    let (left, right) = match p.get("justify").and_then(Value::as_str).map(str::to_ascii_lowercase).as_deref() {
        None | Some("center") => (w / 2.0, -w / 2.0),
        Some("left") => (w, 0.0),
        Some("right") => (0.0, -w),
        Some(_) => return Err(bad(CMD, "`justify` must be \"center\", \"left\" or \"right\"")),
    };
    let section = vec![Vec2::new(right, 0.0), Vec2::new(left, 0.0), Vec2::new(left, h), Vec2::new(right, h)];
    if path.windows(2).any(|w| (w[1].z - w[0].z).abs() > 1e-9) {
        return Err(bad(CMD, "the path must be level (in a plane parallel to XY)"));
    }
    let mesh = sweep(&[section], &path, closed, SweepOptions::default()).map_err(|e| solid_error(CMD, e))?;
    let volume = mesh.volume();
    let h = super::add_solid(s, mesh)?;
    if let (Some(o), false) = (object, bool_or(p, "keep", false)) {
        s.doc_mut()?.remove_entity(o);
    }
    Ok(json!({ "handle": hex(h), "volume": volume }))
}

fn run_loft(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "loft";
    let hs = targets(s, p)?;
    let mut sections: Vec<Vec<Vec3>> = Vec::with_capacity(hs.len());
    // The solid takes the properties of the first section that isn't a point.
    let mut common = None;
    for h in &hs {
        let e = curves::entity(s, *h)?;
        if curves::is_locked(s, *h) {
            return Err(bad(CMD, format!("cross-section {} is on a locked layer", h.hex())));
        }
        if let EntityKind::Point(pt) = &e.kind {
            sections.push(vec![pt.p]);
            continue;
        }
        let prof = profile(s.doc()?, &e).map_err(|why| bad(CMD, format!("cross-section {} is {why}", h.hex())))?;
        let [l] = prof.loops.as_slice() else {
            return Err(bad(CMD, format!("cross-section {} has holes: loft single closed outlines", h.hex())));
        };
        sections.push(l.iter().map(|q| q.to3(prof.z)).collect());
        common.get_or_insert_with(|| {
            let mut c = e.common.clone();
            c.xdata.clear();
            c
        });
    }
    if sections.len() < 2 {
        return Err(bad(CMD, "select at least two cross-sections, in order"));
    }
    let mesh = loft(&sections, bool_or(p, "ruled", false)).map_err(|e| solid_error(CMD, e))?;
    let volume = mesh.volume();
    let space = s.space();
    let d = s.doc_mut()?;
    let h = d.add(&space, common.unwrap_or_default(), EntityKind::Solid3d(cadcraft_doc::Solid3d::new(mesh)))?;
    if !bool_or(p, "keep", false) {
        for x in &hs {
            d.remove_entity(*x);
        }
    }
    Ok(json!({ "handle": hex(h), "volume": volume }))
}

/// SWEEP: select the profiles, then the path (Twist and Scale on the way).
#[derive(Default)]
struct SweepM {
    sel: SelectPhase,
    picked: Option<Vec<Handle>>,
    asking: Option<&'static str>,
    twist: f64,
    scale: f64,
}

impl Interactive for SweepM {
    fn name(&self) -> &'static str {
        "SWEEP"
    }
    fn begin(&mut self, s: &mut Session) -> Result<Step> {
        self.scale = 1.0;
        self.sel = SelectPhase::begin(s);
        if self.sel.done {
            self.picked = Some(self.sel.picked.clone());
        }
        Ok(Step::Continue)
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (&self.picked, self.asking) {
            (None, _) => self.sel.prompt(),
            (Some(_), Some("twist")) => Prompt::new("Enter twist angle", Accept::NUMBER).default(format!("{}", self.twist)),
            (Some(_), Some(_)) => Prompt::new("Enter scale factor", Accept::NUMBER).default(format!("{}", self.scale)),
            (Some(_), None) => Prompt::new("Select sweep path or [Twist/Scale]", Accept { text: true, ..Accept::POINT }).kw(&["Twist", "Scale"]),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let Some(picked) = self.picked.clone() else {
            return match self.sel.feed(s, &i)? {
                SelOutcome::More => Ok(Step::Continue),
                SelOutcome::Empty => Ok(Step::Done),
                SelOutcome::Done(hs) => {
                    self.picked = Some(hs);
                    Ok(Step::Continue)
                }
            };
        };
        match (self.asking, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (Some(k), Input::Text(t)) => {
                let v = parse_number(&t).ok_or_else(|| EngineError::Other("Requires a number.".into()))?;
                if k == "twist" {
                    self.twist = v;
                } else {
                    self.scale = v;
                }
                self.asking = None;
                Ok(Step::Continue)
            }
            (Some(_), Input::Enter) => {
                self.asking = None;
                Ok(Step::Continue)
            }
            (None, Input::Keyword(w) | Input::Text(w)) => {
                self.asking = match w.trim().to_ascii_lowercase().chars().next() {
                    Some('t') => Some("twist"),
                    Some('s') => Some("scale"),
                    _ => return Err(EngineError::Other("Invalid option keyword.".into())),
                };
                Ok(Step::Continue)
            }
            (None, Input::Point(pt)) => {
                let Some(path) = curves::pick_at(s, pt) else {
                    s.echo("No path there.".to_string());
                    return Ok(Step::Continue);
                };
                let p = json!({ "handles": picked.iter().map(|h| h.hex()).collect::<Vec<_>>(), "path": path.hex(), "twist": self.twist, "scale": self.scale });
                let r = run_sweep(s, &p)?;
                let n = r.get("handles").and_then(Value::as_array).map_or(0, Vec::len);
                s.echo(format!("{n} solid(s) created."));
                s.set_selection(Vec::new());
                Ok(Step::Done)
            }
            (None, Input::Enter) => Ok(Step::Cancel),
            _ => Ok(Step::Continue),
        }
    }
}

/// POLYSOLID: points like a polyline, Enter to finish, Close to close it; Height, Width and
/// Justify change the wall first.
#[derive(Default)]
struct PolysolidM {
    pts: Vec<Vec2>,
    asking: Option<&'static str>,
    width: Option<f64>,
    height: Option<f64>,
    justify: Option<&'static str>,
}

impl PolysolidM {
    fn finish(&self, s: &mut Session, closed: bool) -> Result<Step> {
        let z = s.doc()?.header.f64("ELEVATION", 0.0);
        let mut p = json!({ "points": self.pts.iter().map(|q| [q.x, q.y, z]).collect::<Vec<_>>(), "closed": closed });
        if let Some(o) = p.as_object_mut() {
            o.extend(self.width.map(|w| ("width".to_string(), json!(w))));
            o.extend(self.height.map(|h| ("height".to_string(), json!(h))));
            o.extend(self.justify.map(|j| ("justify".to_string(), json!(j))));
        }
        run_polysolid(s, &p)?;
        Ok(Step::Done)
    }
}

impl Interactive for PolysolidM {
    fn name(&self) -> &'static str {
        "POLYSOLID"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        let accept = Accept { text: true, ..Accept::POINT };
        match (self.asking, self.pts.len()) {
            (Some("height"), _) => Prompt::new("Specify height", Accept::NUMBER),
            (Some("width"), _) => Prompt::new("Specify width", Accept::NUMBER),
            (Some(_), _) => Prompt::new("Enter justification", Accept::TEXT).kw(&["Left", "Center", "Right"]).default("Center"),
            (None, 0) => Prompt::new("Specify start point or [Height/Width/Justify]", accept).kw(&["Height", "Width", "Justify"]),
            (None, 1) => Prompt::new("Specify next point", Accept::POINT).base_opt(self.pts.last().copied()),
            (None, _) => Prompt::new("Specify next point or [Close]", accept).kw(&["Close"]).base_opt(self.pts.last().copied()),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let num = |t: &str| parse_number(t).filter(|v| *v > 0.0).ok_or_else(|| EngineError::Other("Requires a positive number.".into()));
        match (self.asking, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (Some("height"), Input::Text(t)) => {
                self.height = Some(num(&t)?);
                self.asking = None;
                Ok(Step::Continue)
            }
            (Some("width"), Input::Text(t)) => {
                self.width = Some(num(&t)?);
                self.asking = None;
                Ok(Step::Continue)
            }
            (Some(_), Input::Keyword(w) | Input::Text(w)) => {
                self.justify = match w.trim().to_ascii_lowercase().chars().next() {
                    Some('l') => Some("left"),
                    Some('r') => Some("right"),
                    _ => Some("center"),
                };
                self.asking = None;
                Ok(Step::Continue)
            }
            (Some(_), Input::Enter) => {
                self.asking = None;
                Ok(Step::Continue)
            }
            (None, Input::Point(q)) => {
                self.pts.push(q);
                Ok(Step::Continue)
            }
            (None, Input::Keyword(w) | Input::Text(w)) => {
                match (w.trim().to_ascii_lowercase().chars().next(), self.pts.len()) {
                    (Some('h'), 0) => self.asking = Some("height"),
                    (Some('w'), 0) => self.asking = Some("width"),
                    (Some('j'), 0) => self.asking = Some("justify"),
                    (Some('c'), n) if n >= 3 => return self.finish(s, true),
                    _ => return Err(EngineError::Other("Invalid option keyword.".into())),
                }
                Ok(Step::Continue)
            }
            (None, Input::Enter) if self.pts.len() >= 2 => self.finish(s, false),
            (None, Input::Enter) => Ok(Step::Cancel),
            _ => Ok(Step::Continue),
        }
    }
    fn preview(&self, _s: &Session, c: Vec2) -> Vec<EntityKind> {
        let mut pts = self.pts.clone();
        if pts.is_empty() {
            return Vec::new();
        }
        pts.push(c);
        vec![super::super::helpers::lwpoly(pts.into_iter().map(cadcraft_geom::PolyVertex::new).collect(), false)]
    }
}

/// LOFT: select the cross-sections in order, then Enter.
#[derive(Default)]
struct LoftM {
    sel: SelectPhase,
}

impl Interactive for LoftM {
    fn name(&self) -> &'static str {
        "LOFT"
    }
    fn begin(&mut self, s: &mut Session) -> Result<Step> {
        self.sel = SelectPhase::begin(s);
        Ok(Step::Continue)
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        self.sel.prompt()
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match self.sel.feed(s, &i)? {
            SelOutcome::More => Ok(Step::Continue),
            SelOutcome::Empty => Ok(Step::Done),
            SelOutcome::Done(hs) => {
                let r = run_loft(s, &json!({ "handles": hs.iter().map(|h| h.hex()).collect::<Vec<_>>() }))?;
                s.echo(format!("Lofted solid {} created.", r.get("handle").and_then(Value::as_str).unwrap_or("")));
                s.set_selection(Vec::new());
                Ok(Step::Done)
            }
        }
    }
}
