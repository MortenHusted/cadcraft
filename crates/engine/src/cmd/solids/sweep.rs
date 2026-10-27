//! Solids from closed profiles: EXTRUDE and REVOLVE. A profile is a closed object in a plane
//! parallel to XY: a circle, a full ellipse, a closed polyline or spline, a planar 3D polyline,
//! a region (whose islands become holes), or single-line text in a TrueType font (its letters'
//! outlines, the counters of letters such as O as holes), to emboss or engrave lettering. Like AutoCAD with DELOBJ at its default, the
//! profiles are erased unless `keep` is set; each solid takes its profile's layer and properties.

use cadcraft_doc::{Drawing, Entity, EntityKind, Handle};
use cadcraft_geom::solid::primitives::{self, CHORD_TOL};
use cadcraft_geom::{Bounds2, Polyline, TAU, Vec2, Vec3, rad};
use serde_json::{Value, json};

use super::super::machines::{SelOutcome, SelectPhase, number as parse_number};
use super::super::{CommandSpec, bad, bool_or, curves, point_value, targets};
use super::{hex, number, point3, solid_error};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

pub fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("extrude", "Extrude", run_extrude)
            .menu(&["Draw", "3D Modeling", "Extrude"])
            .alias(&["ext"])
            .params("{handles?: closed profiles (circles, closed polylines/splines, ellipses, regions, TEXT in a TrueType font), height | direction: [x,y,z], keep?: false (erase the profiles)} → {handles, volumes, skipped}")
            .interactive(|_| Ok(Box::new(SweepM::new(Kind::Extrude)))),
        CommandSpec::new("revolve", "Revolve", run_revolve)
            .menu(&["Draw", "3D Modeling", "Revolve"])
            .alias(&["rev"])
            .params("{handles?: closed profiles, axis: [[x,y], [x,y]] (two points in the profiles' plane), angle?: degrees (default 360), keep?: false} → {handles, volumes, skipped}")
            .interactive(|_| Ok(Box::new(SweepM::new(Kind::Revolve)))),
    ]
}

/// A closed profile: its loops (outer and holes) at height `z`.
pub(super) struct Profile {
    pub(super) loops: Vec<Vec<Vec2>>,
    pub(super) z: f64,
}

/// The profile of `e`, or why it has none.
pub(super) fn profile(d: &Drawing, e: &Entity) -> std::result::Result<Profile, &'static str> {
    let n = e.common.extrusion;
    if n.x.abs() > 1e-9 || n.y.abs() > 1e-9 || n.z <= 0.0 {
        return Err("not in a plane parallel to XY");
    }
    let tol = |b: Bounds2| {
        let size = b.width().max(b.height());
        CHORD_TOL.min(size * 1e-3).max(size * 1e-9)
    };
    let closed_poly = |pts: &[Vec2], closed: bool| closed || pts.first().zip(pts.last()).is_some_and(|(a, b)| a.near(*b, 1e-9));
    match &e.kind {
        EntityKind::Circle(c) => {
            let k = primitives::circle_segments(c.radius);
            let pts = (0..k).map(|i| Vec2::polar(c.center.xy(), c.radius, TAU * i as f64 / k as f64)).collect();
            Ok(Profile { loops: vec![pts], z: c.center.z })
        }
        EntityKind::Ellipse(el) if (cadcraft_geom::ccw_sweep(el.start, el.end) - TAU).abs() < 1e-9 => {
            let g = cadcraft_geom::Ellipse { center: el.center.xy(), major: el.major.xy(), ratio: el.ratio, start: 0.0, end: TAU };
            let mut pts = Vec::new();
            g.tessellate(tol(g.bounds()), &mut pts);
            Ok(Profile { loops: vec![pts], z: el.center.z })
        }
        EntityKind::LwPolyline(p) if closed_poly(&p.vertices.iter().map(|v| v.p).collect::<Vec<_>>(), p.closed) => {
            let pl = Polyline { vertices: p.vertices.clone(), closed: true };
            Ok(Profile { loops: vec![pl.tessellate(tol(pl.bounds()))], z: p.elevation })
        }
        EntityKind::Spline(sp) if sp.closed || closed_poly(&sp.control, false) => {
            let b = Bounds2::from_points(sp.control.iter().copied());
            Ok(Profile { loops: vec![sp.tessellate(tol(b))], z: 0.0 })
        }
        EntityKind::Polyline3d(p) if closed_poly(&p.points.iter().map(|q| q.xy()).collect::<Vec<_>>(), p.closed) => {
            let z = p.points.first().map_or(0.0, |q| q.z);
            if p.points.iter().any(|q| (q.z - z).abs() > 1e-9) {
                return Err("not flat");
            }
            Ok(Profile { loops: vec![p.points.iter().map(|q| q.xy()).collect()], z })
        }
        EntityKind::Region(r) => Ok(Profile { loops: r.tessellate(tol(r.bounds())), z: 0.0 }),
        // Text in an outline (TrueType) font: its glyphs' contours, counters as holes.
        EntityKind::Text(t) => {
            let (shaped, _) = cadcraft_render::place_text_entity(d, t, &t.value);
            let loops: Vec<Vec<Vec2>> = shaped.glyphs.into_iter().flatten().filter(|c| c.len() >= 3).collect();
            if loops.is_empty() {
                return Err("text in a single-stroke font has no outline: give its text style a TrueType font");
            }
            Ok(Profile { loops, z: t.insert.z })
        }
        _ => Err("not a closed profile"),
    }
}

/// Make a solid from each profile among the targets with `make`; erase the profiles unless
/// `keep`. Objects on locked layers and objects that are not profiles are skipped.
fn sweep(s: &mut Session, cmd: &str, p: &Value, make: impl Fn(&Profile) -> Result<cadcraft_geom::solid::Mesh>) -> Result<Value> {
    let hs = targets(s, p)?;
    let keep = bool_or(p, "keep", false);
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
        let mesh = match make(&prof) {
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
        return Err(bad(
            cmd,
            format!("no solid made ({why}): select closed profiles, such as circles, closed polylines or splines, ellipses or regions"),
        ));
    }
    Ok(json!({ "handles": made, "volumes": volumes, "skipped": skipped }))
}

fn run_extrude(s: &mut Session, p: &Value) -> Result<Value> {
    let dir = match (point3("extrude", p, "direction")?, number("extrude", p, "height")?) {
        (Some(d), _) => d,
        (None, Some(h)) => Vec3::new(0.0, 0.0, h),
        (None, None) => return Err(bad("extrude", "`height` or `direction` is required")),
    };
    sweep(s, "extrude", p, |pr| primitives::extrude(&pr.loops, pr.z, dir).map_err(|e| solid_error("extrude", e)))
}

fn run_revolve(s: &mut Session, p: &Value) -> Result<Value> {
    let axis = p.get("axis").and_then(Value::as_array);
    let (a, b) = match axis.map(|a| (a.first().and_then(point_value), a.get(1).and_then(point_value))) {
        Some((Some(a), Some(b))) => (a, b),
        _ => return Err(bad("revolve", "`axis` ([[x,y], [x,y]]: two points on the axis) is required")),
    };
    let angle = rad(number("revolve", p, "angle")?.unwrap_or(360.0));
    sweep(s, "revolve", p, |pr| {
        let n = primitives::circle_segments(pr.loops.iter().flatten().map(|q| q.dist(a)).fold(0.0, f64::max));
        let mut m = primitives::revolve(&pr.loops, a, b - a, angle, n).map_err(|e| solid_error("revolve", e))?;
        m.transform(&cadcraft_geom::Mat4::translate(Vec3::new(0.0, 0.0, pr.z)));
        Ok(m)
    })
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Extrude,
    Revolve,
}

/// Select the profiles, then (EXTRUDE) the height, or (REVOLVE) the axis and angle.
struct SweepM {
    kind: Kind,
    sel: SelectPhase,
    picked: Option<Vec<Handle>>,
    axis: Vec<Vec2>,
}

impl SweepM {
    fn new(kind: Kind) -> SweepM {
        SweepM { kind, sel: SelectPhase::default(), picked: None, axis: Vec::new() }
    }
    fn finish(&self, s: &mut Session, extra: Value) -> Result<Step> {
        let mut p = json!({ "handles": self.picked.iter().flatten().map(|h| h.hex()).collect::<Vec<_>>() });
        if let (Some(o), Some(e)) = (p.as_object_mut(), extra.as_object()) {
            o.extend(e.clone());
        }
        let r = if self.kind == Kind::Extrude { run_extrude(s, &p)? } else { run_revolve(s, &p)? };
        let n = r.get("handles").and_then(Value::as_array).map_or(0, Vec::len);
        s.echo(format!("{n} solid(s) created."));
        s.set_selection(Vec::new());
        Ok(Step::Done)
    }
}

impl Interactive for SweepM {
    fn name(&self) -> &'static str {
        if self.kind == Kind::Extrude { "EXTRUDE" } else { "REVOLVE" }
    }
    fn begin(&mut self, s: &mut Session) -> Result<Step> {
        self.sel = SelectPhase::begin(s);
        if self.sel.done {
            self.picked = Some(self.sel.picked.clone());
        }
        Ok(Step::Continue)
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        if self.picked.is_none() {
            return self.sel.prompt();
        }
        match (self.kind, self.axis.len()) {
            (Kind::Extrude, _) => Prompt::new("Specify height of extrusion", Accept::NUMBER),
            (Kind::Revolve, 0) => Prompt::new("Specify axis start point", Accept::POINT),
            (Kind::Revolve, 1) => Prompt::new("Specify axis endpoint", Accept::POINT).base_opt(self.axis.first().copied()),
            (Kind::Revolve, _) => Prompt::new("Specify angle of revolution", Accept::NUMBER).default("360"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if self.picked.is_none() {
            return match self.sel.feed(s, &i)? {
                SelOutcome::More => Ok(Step::Continue),
                SelOutcome::Empty => Ok(Step::Done),
                SelOutcome::Done(hs) => {
                    self.picked = Some(hs);
                    Ok(Step::Continue)
                }
            };
        }
        let num = |t: &str| parse_number(t).ok_or_else(|| EngineError::Other("Requires a number.".into()));
        match (self.kind, self.axis.len(), i) {
            (Kind::Extrude, _, Input::Text(t)) => self.finish(s, json!({ "height": num(&t)? })),
            (Kind::Revolve, 0 | 1, Input::Point(p)) => {
                self.axis.push(p);
                Ok(Step::Continue)
            }
            (Kind::Revolve, 2, Input::Text(t)) => {
                self.finish(s, json!({ "axis": self.axis.iter().map(|q| [q.x, q.y]).collect::<Vec<_>>(), "angle": num(&t)? }))
            }
            (Kind::Revolve, 2, Input::Enter) => {
                self.finish(s, json!({ "axis": self.axis.iter().map(|q| [q.x, q.y]).collect::<Vec<_>>(), "angle": 360.0 }))
            }
            (_, _, Input::Enter | Input::Cancel) => Ok(Step::Cancel),
            _ => Ok(Step::Continue),
        }
    }
}
