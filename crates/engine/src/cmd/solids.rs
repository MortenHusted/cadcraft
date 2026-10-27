//! 3D solid modeling: primitives (BOX, WEDGE, CYLINDER, CONE, SPHERE, TORUS, PYRAMID), solids
//! from profiles (EXTRUDE, REVOLVE, SWEEP, LOFT, POLYSOLID), placing solids in 3D (3DMOVE,
//! 3DROTATE, MIRROR3D, 3DARRAY, 3DALIGN), rounding and bevelling edges (FILLETEDGE,
//! CHAMFEREDGE), SOLIDEDIT's Shell, Separate and Check, SLICE, INTERFERE, and output for 3D printing (STLOUT, 3DPRINT). UNION,
//! SUBTRACT, INTERSECT and MASSPROP (`inquiry::regions`) hand the 3D solids they are given to [`combine`] and
//! [`massprop`].
//!
//! Solids are closed meshes ([`cadcraft_geom::solid`]). The JSON forms take 3D points as
//! `[x, y, z]` (a missing z is 0); the interactive forms pick points in plan at the current
//! elevation and ask for heights.

use cadcraft_doc::{EntityKind, Handle, Solid3d};
use cadcraft_geom::Vec3;
use cadcraft_geom::solid::{Mesh, SolidError};
use serde_json::{Value, json};

use super::machines::{self, SelOutcome, SelectPhase};
use super::{CommandSpec, bad};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

mod create;
mod edges;
mod loft;
mod output;
mod place;
mod solidedit;
mod sweep;

pub(crate) use output::massprop;
pub(crate) use place::{combine, transformed};

pub fn specs() -> Vec<CommandSpec> {
    [create::specs(), sweep::specs(), loft::specs(), place::specs(), edges::specs(), solidedit::specs(), output::specs()]
        .into_iter()
        .flatten()
        .collect()
}

/// A 3D point: `[x, y, z]` or `[x, y]` (z = 0), or `{x, y, z?}`.
pub(crate) fn point3_value(v: &Value) -> Option<Vec3> {
    let p = match v {
        Value::Array(a) => Vec3::new(a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2).map_or(Some(0.0), Value::as_f64)?),
        Value::Object(_) => Vec3::new(v.get("x")?.as_f64()?, v.get("y")?.as_f64()?, v.get("z").map_or(Some(0.0), Value::as_f64)?),
        _ => return None,
    };
    p.is_finite().then_some(p)
}

/// Optional 3D point `key`: `None` when absent, an error when present but not a point.
pub(crate) fn point3(cmd: &str, p: &Value, key: &str) -> Result<Option<Vec3>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => point3_value(v).map(Some).ok_or_else(|| bad(cmd, format!("`{key}` must be a point [x, y, z]"))),
    }
}

pub(crate) fn point3_req(cmd: &str, p: &Value, key: &str) -> Result<Vec3> {
    point3(cmd, p, key)?.ok_or_else(|| bad(cmd, format!("`{key}` ([x, y, z]) is required")))
}

/// An optional finite number `key`.
fn number(cmd: &str, p: &Value, key: &str) -> Result<Option<f64>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v.as_f64().filter(|x| x.is_finite()).map(Some).ok_or_else(|| bad(cmd, format!("`{key}` must be a number"))),
    }
}

/// `segments`: facets around a full circle (default: from the radius, see
/// [`cadcraft_geom::solid::primitives::circle_segments`]).
fn segments(cmd: &str, p: &Value, radius: f64) -> Result<usize> {
    use cadcraft_geom::solid::primitives::{MAX_SEGMENTS, circle_segments};
    match p.get("segments") {
        None | Some(Value::Null) => Ok(circle_segments(radius)),
        Some(v) => v
            .as_u64()
            .and_then(|n| usize::try_from(n).ok())
            .filter(|n| (3..=MAX_SEGMENTS).contains(n))
            .ok_or_else(|| bad(cmd, format!("`segments` must be a whole number from 3 to {MAX_SEGMENTS}"))),
    }
}

pub(crate) fn solid_error(cmd: &str, e: SolidError) -> EngineError {
    bad(cmd, e.to_string())
}

/// Add a solid on the current layer.
pub(crate) fn add_solid(s: &mut Session, mesh: Mesh) -> Result<Handle> {
    s.add_entity(EntityKind::Solid3d(Solid3d::new(mesh)))
}

/// The 3D solids among `hs`, with their handles, each once in the order first named (other
/// objects are left out).
pub(crate) fn solids_in(s: &Session, hs: &[Handle]) -> Result<Vec<(Handle, Solid3d)>> {
    let d = s.doc()?;
    let mut seen = std::collections::HashSet::new();
    Ok(hs
        .iter()
        .filter(|h| seen.insert(**h))
        .filter_map(|h| match &d.entity(*h)?.kind {
            EntityKind::Solid3d(so) => Some((*h, so.clone())),
            _ => None,
        })
        .collect())
}

/// A handle as JSON.
fn hex(h: Handle) -> Value {
    Value::String(h.hex())
}

/// What an interactive solid command asks for after "Select objects", and the JSON key the
/// answer goes in.
#[derive(Clone, Copy)]
pub(crate) enum Ask {
    /// A point picked in plan (z = 0) or typed as `x,y,z`.
    Point(&'static str, &'static str),
    /// A number.
    Number(&'static str, &'static str),
    /// One of the keywords, or the first on Enter.
    Keyword(&'static str, &'static str, &'static [&'static str]),
    /// Free text (a file name).
    Text(&'static str, &'static str),
}

/// "Select objects", then each [`Ask`] in turn, then the JSON command `id` on the selection and
/// the answers.
pub(crate) struct SelectAsk {
    id: &'static str,
    name: &'static str,
    asks: &'static [Ask],
    sel: SelectPhase,
    picked: Option<Vec<Handle>>,
    answers: serde_json::Map<String, Value>,
}

impl SelectAsk {
    pub(crate) fn new(id: &'static str, name: &'static str, asks: &'static [Ask]) -> SelectAsk {
        SelectAsk { id, name, asks, sel: SelectPhase::default(), picked: None, answers: serde_json::Map::new() }
    }
    fn ask(&self) -> Option<Ask> {
        self.asks.get(self.answers.len()).copied()
    }
    fn answer(&mut self, s: &mut Session, key: &str, v: Value) -> Result<Step> {
        self.answers.insert(key.to_string(), v);
        if self.ask().is_some() {
            return Ok(Step::Continue);
        }
        self.run(s)
    }
    /// The selection is made: ask on, or run the command when there is nothing to ask.
    fn picked(&mut self, s: &mut Session, hs: Vec<Handle>) -> Result<Step> {
        self.picked = Some(hs);
        if self.ask().is_some() { Ok(Step::Continue) } else { self.run(s) }
    }
    fn run(&mut self, s: &mut Session) -> Result<Step> {
        let mut p = self.answers.clone();
        p.insert("handles".into(), self.picked.iter().flatten().map(|h| hex(*h)).collect());
        let spec = super::find_command(self.id).ok_or_else(|| EngineError::UnknownCommand(self.id.into()))?;
        let r = (spec.run)(s, &Value::Object(p))?;
        if let Some(m) = r.get("message").and_then(Value::as_str).filter(|m| !m.is_empty()) {
            s.echo(m.to_string());
        }
        s.set_selection(Vec::new());
        Ok(Step::Done)
    }
}

/// `x,y,z` (or `x,y`) typed at a prompt.
fn typed_point3(t: &str) -> Option<Vec3> {
    let parts: Vec<f64> = t.split(',').map(|c| crate::units::parse_distance(c.trim())).collect::<Option<_>>()?;
    match parts.as_slice() {
        [x, y] => Some(Vec3::new(*x, *y, 0.0)),
        [x, y, z] => Some(Vec3::new(*x, *y, *z)),
        _ => None,
    }
}

/// The keyword of `kws` that `w` names in full or by its first letter.
fn keyword<'a>(kws: &[&'a str], w: &str) -> Option<&'a str> {
    let w = w.trim();
    kws.iter().copied().find(|kw| kw.eq_ignore_ascii_case(w) || (w.len() == 1 && kw.get(..1).is_some_and(|c| c.eq_ignore_ascii_case(w))))
}

impl Interactive for SelectAsk {
    fn name(&self) -> &'static str {
        self.name
    }
    fn begin(&mut self, s: &mut Session) -> Result<Step> {
        self.sel = SelectPhase::begin(s);
        if self.sel.done {
            return self.picked(s, self.sel.picked.clone());
        }
        Ok(Step::Continue)
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        if self.picked.is_none() {
            return self.sel.prompt();
        }
        match self.ask() {
            Some(Ask::Point(m, _)) => Prompt::new(m, Accept { text: true, ..Accept::POINT }),
            Some(Ask::Number(m, _)) => Prompt::new(m, Accept::NUMBER),
            Some(Ask::Keyword(m, _, kws)) => Prompt::new(m, Accept::TEXT).kw(kws).default(kws.first().copied().unwrap_or_default()),
            Some(Ask::Text(m, _)) => Prompt::new(m, Accept::TEXT),
            None => Prompt::new("", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if self.picked.is_none() {
            return match self.sel.feed(s, &i)? {
                SelOutcome::More => Ok(Step::Continue),
                SelOutcome::Empty => Ok(Step::Done),
                SelOutcome::Done(hs) => self.picked(s, hs),
            };
        }
        let fail = |m: &str| Err(EngineError::Other(m.to_string()));
        match (self.ask(), i) {
            (Some(Ask::Point(_, k)), Input::Point(p)) => self.answer(s, k, json!([p.x, p.y, 0.0])),
            (Some(Ask::Point(_, k)), Input::Text(t)) => match typed_point3(&t) {
                Some(p) => self.answer(s, k, json!([p.x, p.y, p.z])),
                None => fail("Requires a point (x,y,z)."),
            },
            (Some(Ask::Number(_, k)), Input::Text(t)) => match machines::number(&t) {
                Some(n) => self.answer(s, k, json!(n)),
                None => fail("Requires a number."),
            },
            (Some(Ask::Keyword(_, k, kws)), Input::Keyword(w) | Input::Text(w)) => match keyword(kws, &w) {
                Some(kw) => self.answer(s, k, Value::String(kw.to_ascii_lowercase())),
                None => fail("Invalid option keyword."),
            },
            (Some(Ask::Keyword(_, k, kws)), Input::Enter) => {
                self.answer(s, k, Value::String(kws.first().copied().unwrap_or_default().to_ascii_lowercase()))
            }
            (Some(Ask::Text(_, k)), Input::Text(t)) => self.answer(s, k, Value::String(t)),
            (_, Input::Enter | Input::Cancel) => Ok(Step::Cancel),
            _ => Ok(Step::Continue),
        }
    }
}
