//! The Render window: RENDER typed or chosen from a menu shows the drawing's 3D solids shaded,
//! with their edges ([`cadcraft_render::shade`]). Drag in the picture to orbit; the buttons give
//! the preset views. It follows the drawing as it changes. RENDER with parameters (scripts, the
//! control channel, MCP) never gets here: it writes or returns the PNG.

use cadcraft_color::{Color, Rgb, aci_rgb};
use cadcraft_doc::{Common, Drawing, EntityKind};
use cadcraft_geom::Vec3;
use cadcraft_render::shade::{ShadeItem, ShadeOptions, shade};
use egui::TextureHandle;

use crate::CadApp;

/// Picture size, in pixels.
const SIZE: [u32; 2] = [720, 540];
/// Degrees of orbit per point dragged.
const DEGREES_PER_POINT: f32 = 0.5;

/// Where the viewer is: degrees around Z from +X, and above the XY plane.
#[derive(Clone, Copy, PartialEq)]
struct Orbit {
    azimuth: f32,
    elevation: f32,
}

impl Orbit {
    fn direction(self) -> Vec3 {
        let (a, e) = (f64::from(self.azimuth).to_radians(), f64::from(self.elevation).to_radians());
        Vec3::new(e.cos() * a.cos(), e.cos() * a.sin(), e.sin())
    }
}

/// The preset views: label, azimuth, elevation (the isometric views are 35.264° up).
const PRESETS: &[(&str, f32, f32)] = &[
    ("Top", -90.0, 90.0),
    ("Front", -90.0, 0.0),
    ("Right", 0.0, 0.0),
    ("SE Isometric", -45.0, 35.264),
    ("SW Isometric", -135.0, 35.264),
    ("NE Isometric", 45.0, 35.264),
    ("NW Isometric", 135.0, 35.264),
];

/// The window's view, and the picture drawn for which view and drawing revision.
#[derive(Clone)]
struct State {
    orbit: Orbit,
    drawn: Option<(Orbit, u64)>,
    texture: Option<TextureHandle>,
    solids: usize,
}

fn state_id() -> egui::Id {
    egui::Id::unique("cc_render_window")
}

/// Open the Render window (RENDER typed or from a menu).
pub fn open(app: &mut CadApp) {
    app.ui.dialog = Some("render".into());
}

/// A solid's colour as RENDER shows it: its own or its layer's, white and ByBlock as light grey.
fn solid_color(d: &Drawing, c: &Common) -> Rgb {
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

/// A shaded picture of model space's visible solids from `orbit`, and how many there were.
fn picture(app: &CadApp, ctx: &egui::Context, orbit: Orbit) -> (Option<TextureHandle>, usize) {
    let Ok(d) = app.session.doc() else { return (None, 0) };
    let items: Vec<ShadeItem> = d
        .model
        .iter()
        .filter(|e| e.common.visible && d.layer(&e.common.layer).is_none_or(|l| l.on && !l.frozen))
        .filter_map(|e| match &e.kind {
            EntityKind::Solid3d(s) => Some(ShadeItem { mesh: s.mesh(), color: solid_color(d, &e.common) }),
            _ => None,
        })
        .collect();
    if items.is_empty() {
        return (None, 0);
    }
    let o = ShadeOptions { width: SIZE[0], height: SIZE[1], view: orbit.direction(), ..ShadeOptions::default() };
    let texture = shade(&items, &o).map(|px| {
        let image = egui::ColorImage::from_rgba_unmultiplied([px.width() as usize, px.height() as usize], px.data());
        ctx.load_texture("render-window", image, egui::TextureOptions::LINEAR)
    });
    (texture, items.len())
}

/// The Render window.
pub fn dialog(app: &mut CadApp, ctx: &egui::Context, open: &mut bool) {
    let revision = app.session.state().map(|s| s.revision).unwrap_or(0);
    let se = PRESETS.get(3).map_or(Orbit { azimuth: -45.0, elevation: 35.264 }, |p| Orbit { azimuth: p.1, elevation: p.2 });
    let mut st = ctx.data(|d| d.get_temp::<State>(state_id())).unwrap_or(State { orbit: se, drawn: None, texture: None, solids: 0 });
    if st.drawn != Some((st.orbit, revision)) {
        (st.texture, st.solids) = picture(app, ctx, st.orbit);
        st.drawn = Some((st.orbit, revision));
    }
    egui::Window::new(crate::tl!("Render")).id(state_id()).open(open).resizable(false).show(ctx, |ui| {
        ui.horizontal_wrapped(|ui| {
            for (label, azimuth, elevation) in PRESETS {
                if ui.button(crate::i18n::t(label)).clicked() {
                    st.orbit = Orbit { azimuth: *azimuth, elevation: *elevation };
                }
            }
        });
        match &st.texture {
            Some(tex) => {
                let size = egui::vec2(SIZE[0] as f32, SIZE[1] as f32);
                let r = ui.add(egui::Image::new((tex.id(), size)).sense(egui::Sense::drag()));
                let drag = r.drag_delta();
                if drag != egui::Vec2::ZERO {
                    st.orbit.azimuth = (st.orbit.azimuth - drag.x * DEGREES_PER_POINT).rem_euclid(360.0);
                    st.orbit.elevation = (st.orbit.elevation + drag.y * DEGREES_PER_POINT).clamp(-90.0, 90.0);
                }
                ui.label(crate::tf!("{count} solid(s); drag the picture to orbit", count = st.solids));
            }
            None => {
                ui.label(crate::tl!("There are no visible 3D solids in model space."));
            }
        }
    });
    if st.drawn != Some((st.orbit, revision)) {
        ctx.request_repaint();
    }
    ctx.data_mut(|d| d.insert_temp(state_id(), st));
}
