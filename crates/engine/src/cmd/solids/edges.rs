//! Rounding and bevelling the edges of 3D solids: FILLETEDGE and CHAMFEREDGE.
//!
//! The JSON forms pick edges by keyword (`all`, `top`, `bottom`, `vertical`, `convex`,
//! `concave`) or by points on or near them; the interactive forms pick edges in plan, the
//! highest one under the cursor where edges lie over each other. Convex edges are cut away,
//! concave ones filled.

use cadcraft_doc::{EntityKind, Handle, Solid3d};
use cadcraft_geom::solid::edges::{Blend, Edge, blend, edges};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::super::machines::number as parse_number;
use super::super::{CommandSpec, bad, curves, targets};
use super::{hex, number, point3, point3_value, solid_error, solids_in};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

const KEYWORDS: &[&str] = &["all", "top", "bottom", "vertical", "convex", "concave"];

pub fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("filletedge", "Fillet Edge", run_filletedge)
            .menu(&["Modify", "Solid Editing", "Fillet edges"])
            .params("{handles?: solids (default: the selection), edges?: \"all\" (default) | \"top\" | \"bottom\" | \"vertical\" | \"convex\" | \"concave\" | [[x,y,z], ...] (points on or near edges: each picks the nearest), radius?: 1} → {handles, edges, volumes, closed}")
            .interactive(|_| Ok(Box::new(EdgeM::new(Kind::Fillet)))),
        CommandSpec::new("chamferedge", "Chamfer Edge", run_chamferedge)
            .menu(&["Modify", "Solid Editing", "Chamfer edges"])
            .params("{handles?, edges? (as FILLETEDGE), distance?: 1 (along the base face), distance2?: (along the other face; default distance), base?: [x,y,z] (the base face is the one whose normal is nearest this direction, either way; default [0,0,1])} → {handles, edges, volumes, closed}")
            .interactive(|_| Ok(Box::new(EdgeM::new(Kind::Chamfer)))),
    ]
}

/// A positive size parameter, `default` when absent.
fn size(cmd: &str, p: &Value, key: &str, default: f64) -> Result<f64> {
    let v = number(cmd, p, key)?.unwrap_or(default);
    if v > 0.0 { Ok(v) } else { Err(bad(cmd, format!("`{key}` must be a positive number"))) }
}

/// For each solid, the edges `which` picks.
fn chosen(cmd: &str, solids: &[(Handle, Solid3d)], which: Option<&Value>) -> Result<Vec<Vec<Edge>>> {
    let all: Vec<Vec<Edge>> = solids.iter().map(|(_, so)| edges(so.mesh())).collect();
    match which {
        None | Some(Value::Null) => Ok(all),
        Some(Value::String(k)) => {
            let k = k.to_ascii_lowercase();
            if !KEYWORDS.contains(&k.as_str()) {
                return Err(bad(cmd, format!("`edges` must be one of {} or a list of points", KEYWORDS.join(", "))));
            }
            Ok(all
                .into_iter()
                .zip(solids)
                .map(|(es, (_, so))| {
                    let b = so.bounds();
                    let tol = 1e-6 * b.min.dist(b.max).max(1.0);
                    es.into_iter()
                        .filter(|e| {
                            let pts = e.points();
                            let first = pts.first().copied().unwrap_or(Vec3::ZERO);
                            match k.as_str() {
                                "top" => pts.iter().all(|q| q.z >= b.max.z - tol),
                                "bottom" => pts.iter().all(|q| q.z <= b.min.z + tol),
                                "vertical" => pts.iter().all(|q| q.xy().near(first.xy(), tol)),
                                "convex" => e.convex,
                                "concave" => !e.convex,
                                _ => true,
                            }
                        })
                        .collect()
                })
                .collect())
        }
        Some(Value::Array(points)) => {
            let mut out: Vec<Vec<Edge>> = vec![Vec::new(); solids.len()];
            for v in points {
                let p = point3_value(v).ok_or_else(|| bad(cmd, "`edges` points must be [x, y, z]"))?;
                let best =
                    all.iter().enumerate().flat_map(|(i, es)| es.iter().map(move |e| (i, e, e.distance_to(p)))).min_by(|a, b| a.2.total_cmp(&b.2));
                let reach = |i: usize| solids.get(i).map_or(0.0, |(_, so)| 0.1 * so.bounds().min.dist(so.bounds().max));
                match best {
                    Some((i, e, dist)) if dist <= reach(i) => {
                        if let Some(list) = out.get_mut(i).filter(|l| !l.contains(e)) {
                            list.push(e.clone());
                        }
                    }
                    _ => return Err(bad(cmd, format!("no edge of the selected solids near [{}, {}, {}]", p.x, p.y, p.z))),
                }
            }
            Ok(out)
        }
        Some(_) => Err(bad(cmd, "`edges` must be a keyword or a list of points")),
    }
}

/// Finish the chosen edges of the target solids as `how` says for each.
fn finish(s: &mut Session, cmd: &str, p: &Value, how: impl Fn(&Edge) -> Blend) -> Result<Value> {
    let hs = targets(s, p)?;
    let solids: Vec<(Handle, Solid3d)> = solids_in(s, &hs)?.into_iter().filter(|(h, _)| !curves::is_locked(s, *h)).collect();
    if solids.is_empty() {
        return Err(bad(cmd, "select 3D solids (on unlocked layers)"));
    }
    let picked = chosen(cmd, &solids, p.get("edges"))?;
    // Finish every solid before changing the drawing, so a failure leaves them all as they were.
    let (mut done, mut count) = (Vec::new(), 0);
    for ((h, so), es) in solids.iter().zip(picked) {
        if es.is_empty() {
            continue;
        }
        count += es.len();
        let jobs: Vec<(Edge, Blend)> = es.into_iter().map(|e| (how(&e), e)).map(|(b, e)| (e, b)).collect();
        done.push((*h, blend(so.mesh(), &jobs).map_err(|e| solid_error(cmd, e))?));
    }
    if count == 0 {
        return Err(bad(cmd, "no edges picked"));
    }
    let (mut handles, mut volumes, mut closed) = (Vec::new(), Vec::new(), true);
    for (h, mesh) in done {
        closed &= mesh.check().is_closed();
        volumes.push(mesh.volume());
        s.doc_mut()?.modify_entity(h, |e| e.kind = EntityKind::Solid3d(Solid3d::new(mesh)))?;
        handles.push(hex(h));
    }
    Ok(json!({ "handles": handles, "edges": count, "volumes": volumes, "closed": closed }))
}

fn run_filletedge(s: &mut Session, p: &Value) -> Result<Value> {
    let r = size("filletedge", p, "radius", 1.0)?;
    finish(s, "filletedge", p, |_| Blend::Fillet(r))
}

fn run_chamferedge(s: &mut Session, p: &Value) -> Result<Value> {
    let d1 = size("chamferedge", p, "distance", 1.0)?;
    let d2 = size("chamferedge", p, "distance2", d1)?;
    let base = point3("chamferedge", p, "base")?.unwrap_or(Vec3::Z).normalized();
    // The base distance goes along the face whose normal is nearest `base`.
    finish(s, "chamferedge", p, |e| {
        let (left, right) = e.faces();
        if left.dot(base).abs() >= right.dot(base).abs() { Blend::Chamfer(d1, d2) } else { Blend::Chamfer(d2, d1) }
    })
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Fillet,
    Chamfer,
}

/// What the prompt asks for after an option keyword.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Asking {
    Edge,
    Radius,
    Distance1,
    Distance2,
}

/// Pick edges in plan, the radius or distances on request, then Enter to apply.
struct EdgeM {
    kind: Kind,
    asking: Asking,
    sizes: [f64; 2],
    /// The solid and a point on each picked edge.
    picks: Vec<(Handle, Vec3)>,
}

impl EdgeM {
    fn new(kind: Kind) -> EdgeM {
        EdgeM { kind, asking: Asking::Edge, sizes: [1.0, 1.0], picks: Vec::new() }
    }

    /// The point on the edge of a solid in the current space nearest `p` in plan, within the
    /// pick aperture; among edges lying over each other, the highest.
    fn pick(s: &Session, p: Vec2) -> Option<(Handle, Vec3)> {
        let aperture = curves::pick_aperture(s);
        let d = s.doc().ok()?;
        let mut best: Option<(f64, f64, Handle, Vec3)> = None;
        for e in d.space(&s.space())?.iter() {
            let EntityKind::Solid3d(so) = &e.kind else { continue };
            let b = so.bounds();
            if p.x < b.min.x - aperture || p.x > b.max.x + aperture || p.y < b.min.y - aperture || p.y > b.max.y + aperture {
                continue;
            }
            for edge in edges(so.mesh()) {
                let pts = edge.points();
                let pairs = pts
                    .windows(2)
                    .map(|w| (w[0], w[1]))
                    .chain(edge.closed.then(|| (pts.last().copied(), pts.first().copied())).and_then(|(a, b)| a.zip(b)));
                for (a, c) in pairs {
                    let ab = c.xy() - a.xy();
                    let t = if ab.len2() > 0.0 { ((p - a.xy()).dot(ab) / ab.len2()).clamp(0.0, 1.0) } else { 0.0 };
                    let q = a.lerp(c, t);
                    let dist = q.xy().dist(p);
                    if dist > aperture {
                        continue;
                    }
                    // Higher first; then nearer.
                    if best.is_none_or(|(bz, bd, ..)| q.z > bz + 1e-9 || (q.z >= bz - 1e-9 && dist < bd)) {
                        best = Some((q.z, dist, e.handle, q));
                    }
                }
            }
        }
        best.map(|(_, _, h, q)| (h, q))
    }

    fn apply(&self, s: &mut Session) -> Result<Step> {
        let mut handles: Vec<String> = self.picks.iter().map(|(h, _)| h.hex()).collect();
        handles.dedup();
        let points: Vec<[f64; 3]> = self.picks.iter().map(|(_, q)| [q.x, q.y, q.z]).collect();
        let p = match self.kind {
            Kind::Fillet => json!({ "handles": handles, "edges": points, "radius": self.sizes[0] }),
            Kind::Chamfer => json!({ "handles": handles, "edges": points, "distance": self.sizes[0], "distance2": self.sizes[1] }),
        };
        let r = if self.kind == Kind::Fillet { run_filletedge(s, &p)? } else { run_chamferedge(s, &p)? };
        let n = r.get("edges").and_then(Value::as_u64).unwrap_or(0);
        s.echo(format!("{n} edge(s) {}.", if self.kind == Kind::Fillet { "filleted" } else { "chamfered" }));
        Ok(Step::Done)
    }
}

impl Interactive for EdgeM {
    fn name(&self) -> &'static str {
        if self.kind == Kind::Fillet { "FILLETEDGE" } else { "CHAMFEREDGE" }
    }
    fn begin(&mut self, _s: &mut Session) -> Result<Step> {
        Ok(Step::Continue)
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        let current = |v: f64| format!("{v:.4}");
        match (self.kind, self.asking) {
            (_, Asking::Radius) => Prompt::new("Enter fillet radius", Accept::NUMBER).default(current(self.sizes[0])),
            (_, Asking::Distance1) => Prompt::new("Specify Distance1", Accept::NUMBER).default(current(self.sizes[0])),
            (_, Asking::Distance2) => Prompt::new("Specify Distance2", Accept::NUMBER).default(current(self.sizes[1])),
            (Kind::Fillet, Asking::Edge) => Prompt::new("Select an edge or [Radius]", Accept { text: true, ..Accept::POINT }).kw(&["Radius"]),
            (Kind::Chamfer, Asking::Edge) => Prompt::new("Select an edge or [Distance]", Accept { text: true, ..Accept::POINT }).kw(&["Distance"]),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let next = |a: Asking| match a {
            Asking::Distance1 => Asking::Distance2,
            _ => Asking::Edge,
        };
        match (self.asking, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (Asking::Edge, Input::Point(p)) => {
                match EdgeM::pick(s, p) {
                    Some(hit) => {
                        self.picks.push(hit);
                        s.echo(format!("{} edge(s) selected.", self.picks.len()));
                    }
                    None => s.echo("No edge of a solid there.".to_string()),
                }
                Ok(Step::Continue)
            }
            (Asking::Edge, Input::Keyword(w) | Input::Text(w)) => {
                let w = w.trim().to_ascii_lowercase();
                self.asking = match (self.kind, w.chars().next()) {
                    (Kind::Fillet, Some('r')) => Asking::Radius,
                    (Kind::Chamfer, Some('d')) => Asking::Distance1,
                    _ => return Err(EngineError::Other("Invalid option keyword.".into())),
                };
                Ok(Step::Continue)
            }
            (Asking::Edge, Input::Enter) if self.picks.is_empty() => Ok(Step::Cancel),
            (Asking::Edge, Input::Enter) => self.apply(s),
            (a, Input::Text(t)) => {
                let v = parse_number(&t).filter(|v| *v > 0.0).ok_or_else(|| EngineError::Other("Requires a positive number.".into()))?;
                // A new Distance1 is offered as Distance2 too.
                self.sizes = match a {
                    Asking::Distance2 => [self.sizes[0], v],
                    Asking::Distance1 => [v, v],
                    _ => [v, self.sizes[1]],
                };
                self.asking = next(a);
                Ok(Step::Continue)
            }
            (a, Input::Enter) => {
                self.asking = next(a);
                Ok(Step::Continue)
            }
            _ => Ok(Step::Continue),
        }
    }
}
