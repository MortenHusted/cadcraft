//! SOLIDEDIT's body options: Shell hollows solids into walls of one thickness, opening the
//! faces asked for; Separate splits a solid into its separate pieces; Check says whether each
//! solid is closed. The menu reaches each option directly (`solidedit.shell`, …).

use cadcraft_doc::{EntityKind, Handle, Solid3d};
use cadcraft_geom::Vec3;
use cadcraft_geom::solid::shell::{faces, separate, shell};
use cadcraft_geom::solid::{FEATURE_ANGLE, Mesh};
use serde_json::{Value, json};

use super::super::{CommandSpec, bad, curves, str_param, targets};
use super::{Ask, SelectAsk, hex, number, point3_value, solid_error, solids_in};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

const SHELL: &[Ask] =
    &[Ask::Number("Enter the shell offset distance", "thickness"), Ask::Keyword("Remove faces", "remove", &["None", "Top", "Bottom"])];

pub fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("solidedit", "Solid Editing", run_solidedit)
            .params("{mode: \"shell\" | \"separate\" | \"check\", handles?, ...the option's parameters} → the option's result")
            .interactive(|_| Ok(Box::new(SolideditM::default()))),
        CommandSpec::new("solidedit.shell", "Shell", run_shell)
            .menu(&["Modify", "Solid Editing", "Shell"])
            .params("{handles?: solids, thickness, remove?: \"none\" | \"top\" | \"bottom\" | [[x,y,z], ...] (points on the faces to open)} → {handles, volumes}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("solidedit.shell", "SOLIDEDIT", SHELL)))),
        CommandSpec::new("solidedit.separate", "Separate", run_separate)
            .menu(&["Modify", "Solid Editing", "Separate"])
            .params("{handles?: solids} → {handles (the first piece keeps each solid's handle)}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("solidedit.separate", "SOLIDEDIT", &[])))),
        CommandSpec::new("solidedit.check", "Check", run_check)
            .menu(&["Modify", "Solid Editing", "Check"])
            .params("{handles?: solids} → {solids: [{handle, closed, openEdges, nonmanifoldEdges, flippedEdges, pieces}], message}")
            .interactive(|_| Ok(Box::new(SelectAsk::new("solidedit.check", "SOLIDEDIT", &[])))),
    ]
}

fn run_solidedit(s: &mut Session, p: &Value) -> Result<Value> {
    match str_param(p, "mode").map(str::to_ascii_lowercase).as_deref() {
        Some("shell") => run_shell(s, p),
        Some("separate") => run_separate(s, p),
        Some("check") => run_check(s, p),
        _ => Err(bad("solidedit", "`mode` must be \"shell\", \"separate\" or \"check\"")),
    }
}

/// The target solids, or an error naming `cmd`.
fn target_solids(s: &Session, cmd: &str, p: &Value) -> Result<Vec<(Handle, Solid3d)>> {
    let hs = targets(s, p)?;
    let out: Vec<(Handle, Solid3d)> = solids_in(s, &hs)?.into_iter().filter(|(h, _)| !curves::is_locked(s, *h)).collect();
    if out.is_empty() {
        return Err(bad(cmd, "select 3D solids (on unlocked layers)"));
    }
    Ok(out)
}

/// The triangles of the faces of `m` that `which` names: none, the top or bottom face, or the
/// faces nearest each point.
fn faces_to_open(cmd: &str, m: &Mesh, which: Option<&Value>) -> Result<Vec<usize>> {
    let all = faces(m);
    let b = m.bounds();
    let tol = 1e-9 * b.min.dist(b.max).max(1.0);
    let corners = |f: &Vec<usize>| f.iter().filter_map(|i| m.triangles.get(*i)).filter_map(|t| m.corners(t)).flatten().collect::<Vec<Vec3>>();
    let flat_at = |f: &Vec<usize>, z: f64| corners(f).iter().all(|p| (p.z - z).abs() <= tol);
    match which {
        None | Some(Value::Null) => Ok(Vec::new()),
        Some(Value::String(k)) => match k.to_ascii_lowercase().as_str() {
            "none" => Ok(Vec::new()),
            "top" => Ok(all.into_iter().filter(|f| flat_at(f, b.max.z)).flatten().collect()),
            "bottom" => Ok(all.into_iter().filter(|f| flat_at(f, b.min.z)).flatten().collect()),
            _ => Err(bad(cmd, "`remove` must be \"none\", \"top\", \"bottom\" or a list of points")),
        },
        Some(Value::Array(points)) => {
            let mut out = Vec::new();
            for v in points {
                let p = point3_value(v).ok_or_else(|| bad(cmd, "`remove` points must be [x, y, z]"))?;
                let nearest = all
                    .iter()
                    .map(|f| (f, corners(f).chunks(3).map(|t| triangle_distance(p, t)).fold(f64::INFINITY, f64::min)))
                    .min_by(|a, b| a.1.total_cmp(&b.1));
                match nearest {
                    Some((f, d)) if d <= 0.05 * b.min.dist(b.max) => out.extend(f.iter().copied()),
                    _ => return Err(bad(cmd, format!("no face of the solid near [{}, {}, {}]", p.x, p.y, p.z))),
                }
            }
            Ok(out)
        }
        Some(_) => Err(bad(cmd, "`remove` must be a keyword or a list of points")),
    }
}

/// The distance from `p` to the triangle with corners `t`.
fn triangle_distance(p: Vec3, t: &[Vec3]) -> f64 {
    let [a, b, c] = t else { return f64::INFINITY };
    let n = (*b - *a).cross(*c - *a);
    let len = n.len();
    let seg = |u: Vec3, v: Vec3| {
        let d = v - u;
        let k = if d.dot(d) > 0.0 { ((p - u).dot(d) / d.dot(d)).clamp(0.0, 1.0) } else { 0.0 };
        p.dist(u + d * k)
    };
    let edges = seg(*a, *b).min(seg(*b, *c)).min(seg(*c, *a));
    if len == 0.0 {
        return edges;
    }
    let n = n * (1.0 / len);
    let q = p - n * n.dot(p - *a);
    // Inside when q is on the inner side of all three edges.
    let side = |u: Vec3, v: Vec3| (v - u).cross(q - u).dot(n) >= 0.0;
    if side(*a, *b) && side(*b, *c) && side(*c, *a) { n.dot(p - *a).abs() } else { edges }
}

fn run_shell(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "solidedit";
    let t = number(CMD, p, "thickness")?.ok_or_else(|| bad(CMD, "`thickness` is required"))?;
    // Hollow every solid before changing the drawing, so a failure leaves them all as they were.
    let mut done = Vec::new();
    for (h, so) in target_solids(s, CMD, p)? {
        let open = faces_to_open(CMD, so.mesh(), p.get("remove"))?;
        done.push((h, shell(so.mesh(), t, &open).map_err(|e| solid_error(CMD, e))?));
    }
    let (mut handles, mut volumes) = (Vec::new(), Vec::new());
    for (h, mesh) in done {
        volumes.push(mesh.volume());
        s.doc_mut()?.modify_entity(h, |e| e.kind = EntityKind::Solid3d(Solid3d::new(mesh)))?;
        handles.push(hex(h));
    }
    Ok(json!({ "handles": handles, "volumes": volumes }))
}

fn run_separate(s: &mut Session, p: &Value) -> Result<Value> {
    let solids = target_solids(s, "solidedit", p)?;
    let mut handles = Vec::new();
    let space = s.space();
    for (h, so) in solids {
        let mut pieces = separate(so.mesh()).into_iter();
        let Some(first) = pieces.next() else { continue };
        let common = curves::entity(s, h)?.common;
        let d = s.doc_mut()?;
        d.modify_entity(h, |e| e.kind = EntityKind::Solid3d(Solid3d::new(first)))?;
        handles.push(hex(h));
        for piece in pieces {
            handles.push(hex(d.add(&space, common.clone(), EntityKind::Solid3d(Solid3d::new(piece)))?));
        }
    }
    let n = handles.len();
    Ok(json!({ "handles": handles, "message": format!("{n} solid(s).") }))
}

fn run_check(s: &mut Session, p: &Value) -> Result<Value> {
    let solids = target_solids(s, "solidedit", p)?;
    let mut out = Vec::new();
    let mut bad_count = 0;
    for (h, so) in solids {
        let c = so.mesh().check();
        if !c.is_closed() {
            bad_count += 1;
        }
        out.push(json!({
            "handle": hex(h),
            "closed": c.is_closed(),
            "openEdges": c.open_edges,
            "nonmanifoldEdges": c.nonmanifold_edges,
            "flippedEdges": c.flipped_edges,
            "pieces": separate(so.mesh()).len(),
            "edges": so.mesh().wire_edges(FEATURE_ANGLE, Vec3::Z).len(),
        }));
    }
    let message = if bad_count == 0 { "Every solid is closed: a valid solid.".to_string() } else { format!("{bad_count} solid(s) are not closed.") };
    Ok(json!({ "solids": out, "message": message }))
}

/// SOLIDEDIT typed: the body option first, then that option's prompts.
#[derive(Default)]
struct SolideditM {
    inner: Option<SelectAsk>,
}

impl Interactive for SolideditM {
    fn name(&self) -> &'static str {
        "SOLIDEDIT"
    }
    fn prompt(&self, s: &Session) -> Prompt {
        match &self.inner {
            Some(m) => m.prompt(s),
            None => Prompt::new("Enter a body editing option", Accept::TEXT).kw(&["Shell", "seParate", "Check"]).default("Shell"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if let Some(m) = &mut self.inner {
            return m.input(s, i);
        }
        let w = match i {
            Input::Keyword(w) | Input::Text(w) => w.trim().to_ascii_lowercase(),
            Input::Enter => "shell".to_string(),
            Input::Cancel => return Ok(Step::Cancel),
            _ => return Ok(Step::Continue),
        };
        let mut m = match w.as_str() {
            "s" | "shell" => SelectAsk::new("solidedit.shell", "SOLIDEDIT", SHELL),
            "p" | "separate" => SelectAsk::new("solidedit.separate", "SOLIDEDIT", &[]),
            "c" | "check" => SelectAsk::new("solidedit.check", "SOLIDEDIT", &[]),
            _ => return Err(EngineError::Other("Invalid option keyword.".into())),
        };
        let step = m.begin(s)?;
        self.inner = Some(m);
        Ok(step)
    }
}
