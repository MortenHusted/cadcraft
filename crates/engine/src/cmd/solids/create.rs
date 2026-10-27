//! Solid primitives: BOX, WEDGE, CYLINDER, CONE, SPHERE and TORUS.

use cadcraft_geom::solid::{Mesh, SolidError, primitives};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::super::machines::number as parse_number;
use super::super::{CommandSpec, bad};
use super::{add_solid, hex, number, point3, point3_req, segments, solid_error};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

pub fn specs() -> Vec<CommandSpec> {
    const BOXLIKE: &str = "{corner: [x,y,z], opposite: [x,y,z]} | {corner | center: [x,y,z], length, width? (= length), height? (= length)}; negative sizes build the other way → {handle, volume}";
    vec![
        CommandSpec::new("box", "Box", run_box)
            .menu(&["Draw", "3D Modeling", "Box"])
            .params(BOXLIKE)
            .interactive(|_| Ok(Box::new(Steps::new("BOX", &BOX_ASKS, build_box)))),
        CommandSpec::new("wedge", "Wedge", run_wedge)
            .menu(&["Draw", "3D Modeling", "Wedge"])
            .alias(&["we"])
            .params("like box; the top slopes down from the face at the first corner's x to the opposite face → {handle, volume}")
            .interactive(|_| Ok(Box::new(Steps::new("WEDGE", &BOX_ASKS, build_wedge)))),
        CommandSpec::new("cylinder", "Cylinder", run_cylinder)
            .menu(&["Draw", "3D Modeling", "Cylinder"])
            .alias(&["cyl"])
            .params("{center: [x,y,z] (of the base), radius | diameter, height (negative: downward), segments?} → {handle, volume}")
            .interactive(|_| Ok(Box::new(Steps::new("CYLINDER", &ROUND_ASKS, build_cylinder)))),
        CommandSpec::new("cone", "Cone", run_cone)
            .menu(&["Draw", "3D Modeling", "Cone"])
            .params("{center: [x,y,z] (of the base), radius | diameter, topRadius? (default 0: pointed), height, segments?} → {handle, volume}")
            .interactive(|_| Ok(Box::new(Steps::new("CONE", &ROUND_ASKS, build_cone)))),
        CommandSpec::new("sphere", "Sphere", run_sphere)
            .menu(&["Draw", "3D Modeling", "Sphere"])
            .params("{center: [x,y,z], radius | diameter, segments?} → {handle, volume}")
            .interactive(|_| Ok(Box::new(Steps::new("SPHERE", &SPHERE_ASKS, build_sphere)))),
        CommandSpec::new("torus", "Torus", run_torus)
            .menu(&["Draw", "3D Modeling", "Torus"])
            .alias(&["tor"])
            .params("{center: [x,y,z], radius | diameter (to the tube's center), tubeRadius | tubeDiameter, segments?} → {handle, volume}")
            .interactive(|_| Ok(Box::new(Steps::new("TORUS", &TORUS_ASKS, build_torus)))),
    ]
}

fn created(s: &mut Session, cmd: &str, mesh: std::result::Result<Mesh, SolidError>) -> Result<Value> {
    let mesh = mesh.map_err(|e| solid_error(cmd, e))?;
    let volume = mesh.volume();
    let h = add_solid(s, mesh)?;
    Ok(json!({ "handle": hex(h), "volume": volume }))
}

/// Opposite corners from `{corner, opposite}`, `{corner, length, width?, height?}` or
/// `{center, length, width?, height?}` (a missing width or height is the length: a cube).
fn box_corners(cmd: &str, p: &Value) -> Result<(Vec3, Vec3)> {
    let (corner, center) = (point3(cmd, p, "corner")?, point3(cmd, p, "center")?);
    if let Some(o) = point3(cmd, p, "opposite")? {
        let c = corner.ok_or_else(|| bad(cmd, "`opposite` needs `corner`"))?;
        return Ok((c, o));
    }
    let l = number(cmd, p, "length")?.ok_or_else(|| bad(cmd, "`opposite` or `length` is required"))?;
    let size = Vec3::new(l, number(cmd, p, "width")?.unwrap_or(l), number(cmd, p, "height")?.unwrap_or(l));
    match (corner, center) {
        (Some(c), None) => Ok((c, c + size)),
        (None, Some(m)) => Ok((m - size * 0.5, m + size * 0.5)),
        _ => Err(bad(cmd, "give `corner` or `center` (not both)")),
    }
}

fn run_box(s: &mut Session, p: &Value) -> Result<Value> {
    let (a, b) = box_corners("box", p)?;
    created(s, "box", primitives::cuboid(a, b))
}

fn run_wedge(s: &mut Session, p: &Value) -> Result<Value> {
    let (a, b) = box_corners("wedge", p)?;
    created(s, "wedge", primitives::wedge(a, b))
}

/// `radius`, or half of `diameter` (also for the `tube` prefix).
fn radius(cmd: &str, p: &Value, prefix: &str) -> Result<Option<f64>> {
    let (rk, dk) =
        if prefix.is_empty() { ("radius".to_string(), "diameter".to_string()) } else { (format!("{prefix}Radius"), format!("{prefix}Diameter")) };
    Ok(match (number(cmd, p, &rk)?, number(cmd, p, &dk)?) {
        (Some(r), _) => Some(r),
        (None, Some(d)) => Some(d / 2.0),
        (None, None) => None,
    })
}

fn radius_req(cmd: &str, p: &Value, prefix: &str) -> Result<f64> {
    radius(cmd, p, prefix)?.ok_or_else(|| {
        bad(
            cmd,
            if prefix.is_empty() {
                "`radius` or `diameter` is required".to_string()
            } else {
                format!("`{prefix}Radius` or `{prefix}Diameter` is required")
            },
        )
    })
}

fn run_cylinder(s: &mut Session, p: &Value) -> Result<Value> {
    let c = point3_req("cylinder", p, "center")?;
    let r = radius_req("cylinder", p, "")?;
    let h = number("cylinder", p, "height")?.ok_or_else(|| bad("cylinder", "`height` is required"))?;
    let n = segments("cylinder", p, r)?;
    created(s, "cylinder", primitives::frustum(c, r, r, h, n))
}

fn run_cone(s: &mut Session, p: &Value) -> Result<Value> {
    let c = point3_req("cone", p, "center")?;
    let r = radius_req("cone", p, "")?;
    let top = number("cone", p, "topRadius")?.unwrap_or(0.0);
    let h = number("cone", p, "height")?.ok_or_else(|| bad("cone", "`height` is required"))?;
    let n = segments("cone", p, r.max(top))?;
    created(s, "cone", primitives::frustum(c, r, top, h, n))
}

fn run_sphere(s: &mut Session, p: &Value) -> Result<Value> {
    let c = point3_req("sphere", p, "center")?;
    let r = radius_req("sphere", p, "")?;
    let n = segments("sphere", p, r)?;
    created(s, "sphere", primitives::sphere(c, r, n))
}

fn run_torus(s: &mut Session, p: &Value) -> Result<Value> {
    let c = point3_req("torus", p, "center")?;
    let r = radius_req("torus", p, "")?;
    let tube = radius_req("torus", p, "tube")?;
    let n = segments("torus", p, r + tube)?;
    created(s, "torus", primitives::torus(c, r, tube, n))
}

// ---------- interactive ----------

/// What a prompt of a primitive asks for. Points come first, then distances.
#[derive(Clone, Copy)]
enum Ask {
    Point(&'static str),
    /// A number, or a point whose distance from the first point is used.
    Distance(&'static str),
}

const BOX_ASKS: [Ask; 3] = [Ask::Point("Specify first corner"), Ask::Point("Specify other corner"), Ask::Distance("Specify height")];
const ROUND_ASKS: [Ask; 3] = [Ask::Point("Specify center point of base"), Ask::Distance("Specify base radius"), Ask::Distance("Specify height")];
const SPHERE_ASKS: [Ask; 2] = [Ask::Point("Specify center point"), Ask::Distance("Specify radius")];
const TORUS_ASKS: [Ask; 3] = [Ask::Point("Specify center point"), Ask::Distance("Specify radius"), Ask::Distance("Specify tube radius")];

type Build = fn(&[Vec2], &[f64], f64) -> std::result::Result<Mesh, SolidError>;

fn build_box(pts: &[Vec2], d: &[f64], z: f64) -> std::result::Result<Mesh, SolidError> {
    let (a, b, h) = corners(pts, d)?;
    primitives::cuboid(a.to3(z), b.to3(z + h))
}

fn build_wedge(pts: &[Vec2], d: &[f64], z: f64) -> std::result::Result<Mesh, SolidError> {
    let (a, b, h) = corners(pts, d)?;
    primitives::wedge(a.to3(z), b.to3(z + h))
}

fn corners(pts: &[Vec2], d: &[f64]) -> std::result::Result<(Vec2, Vec2, f64), SolidError> {
    match (pts, d) {
        ([a, b], [h]) => Ok((*a, *b, *h)),
        _ => Err(SolidError::Degenerate("Two corners and a height are needed.".into())),
    }
}

fn build_cylinder(pts: &[Vec2], d: &[f64], z: f64) -> std::result::Result<Mesh, SolidError> {
    match (pts, d) {
        ([c], [r, h]) => primitives::frustum(c.to3(z), *r, *r, *h, primitives::circle_segments(*r)),
        _ => Err(SolidError::Degenerate("A center, a radius and a height are needed.".into())),
    }
}

fn build_cone(pts: &[Vec2], d: &[f64], z: f64) -> std::result::Result<Mesh, SolidError> {
    match (pts, d) {
        ([c], [r, h]) => primitives::frustum(c.to3(z), *r, 0.0, *h, primitives::circle_segments(*r)),
        _ => Err(SolidError::Degenerate("A center, a radius and a height are needed.".into())),
    }
}

fn build_sphere(pts: &[Vec2], d: &[f64], z: f64) -> std::result::Result<Mesh, SolidError> {
    match (pts, d) {
        ([c], [r]) => primitives::sphere(c.to3(z), *r, primitives::circle_segments(*r)),
        _ => Err(SolidError::Degenerate("A center and a radius are needed.".into())),
    }
}

fn build_torus(pts: &[Vec2], d: &[f64], z: f64) -> std::result::Result<Mesh, SolidError> {
    match (pts, d) {
        ([c], [r, t]) => primitives::torus(c.to3(z), *r, *t, primitives::circle_segments(*r + *t)),
        _ => Err(SolidError::Degenerate("A center, a radius and a tube radius are needed.".into())),
    }
}

/// Points picked in plan at the current elevation, then distances typed or picked.
struct Steps {
    name: &'static str,
    asks: &'static [Ask],
    pts: Vec<Vec2>,
    dists: Vec<f64>,
    build: Build,
}

impl Steps {
    fn new(name: &'static str, asks: &'static [Ask], build: Build) -> Steps {
        Steps { name, asks, pts: Vec::new(), dists: Vec::new(), build }
    }
    fn ask(&self) -> Option<Ask> {
        self.asks.get(self.pts.len() + self.dists.len()).copied()
    }
}

impl Interactive for Steps {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.ask() {
            Some(Ask::Point(m)) => Prompt::new(m, Accept::POINT).base_opt(self.pts.last().copied()),
            Some(Ask::Distance(m)) => Prompt::new(m, Accept::POINT_OR_NUMBER).base_opt(self.pts.first().copied()).distance(),
            None => Prompt::new("", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.ask(), i) {
            (Some(Ask::Point(_)), Input::Point(p)) => self.pts.push(p),
            (Some(Ask::Distance(_)), Input::Point(p)) => self.dists.push(self.pts.first().map_or(0.0, |b| b.dist(p))),
            (Some(Ask::Distance(_)), Input::Text(t)) => {
                self.dists.push(parse_number(&t).ok_or_else(|| EngineError::Other("Requires numeric distance or second point.".into()))?);
            }
            (Some(Ask::Point(_)), Input::Text(_)) => return Err(EngineError::Other("Invalid point.".into())),
            (_, Input::Enter | Input::Cancel) => return Ok(Step::Cancel),
            _ => return Ok(Step::Continue),
        }
        if self.ask().is_some() {
            return Ok(Step::Continue);
        }
        let z = s.doc()?.header.f64("ELEVATION", 0.0);
        let mesh = (self.build)(&self.pts, &self.dists, z).map_err(|e| EngineError::Other(e.to_string()))?;
        add_solid(s, mesh)?;
        Ok(Step::Done)
    }
    fn preview(&self, _s: &Session, c: Vec2) -> Vec<cadcraft_doc::EntityKind> {
        use super::super::helpers::{circle, line, lwpoly, rect_vertices};
        // The base outline while it is being picked, then a line from the base point.
        match (self.ask(), self.pts.as_slice(), self.dists.len()) {
            (Some(Ask::Point(_)), [a], _) => vec![lwpoly(rect_vertices(*a, c), true)],
            (Some(Ask::Distance(_)), [a], 0) => vec![circle(*a, a.dist(c)), line(*a, c)],
            (Some(Ask::Distance(_)), [a, ..], _) => vec![line(*a, c)],
            _ => Vec::new(),
        }
    }
}
