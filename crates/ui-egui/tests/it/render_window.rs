//! RENDER typed at the command line opens the Render window with the shaded solids; a preset
//! view button and dragging the picture change the view; RENDER with parameters (scripts, MCP)
//! returns a PNG and opens nothing.

use cadcraft_engine::Session;
use cadcraft_ui_egui::{CadApp, Services};
use egui::vec2;
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;

fn harness() -> Harness<'static, CadApp> {
    let mut app = CadApp::new(Session::default(), Services::default());
    app.run("new", json!({})).unwrap();
    app.ui.start_tab = false;
    app.ui.in_window_menu = false;
    let mut h = Harness::builder().with_size(vec2(1400.0, 900.0)).build_ui_state(
        |ui, app: &mut CadApp| {
            app.logic(&ui.ctx().clone());
            app.ui(ui);
        },
        app,
    );
    h.run_steps(3);
    h
}

#[test]
fn typed_render_opens_the_window_and_presets_change_the_view() {
    let mut h = harness();
    h.state_mut().run("box", json!({"corner": [0, 0, 0], "opposite": [20, 10, 5]})).unwrap();
    h.state_mut().cmdline("render");
    h.run_steps(3);
    assert_eq!(h.state().ui.dialog.as_deref(), Some("render"));
    h.get_by_label("1 solid(s); drag the picture to orbit");
    h.get_by_label("Top").click();
    h.run_steps(3);
    h.get_by_label("NW Isometric").click();
    h.run_steps(3);
    assert_eq!(h.state().ui.dialog.as_deref(), Some("render"));
}

#[test]
fn an_empty_drawing_says_there_is_nothing_to_render() {
    let mut h = harness();
    h.state_mut().cmdline("render");
    h.run_steps(3);
    h.get_by_label("There are no visible 3D solids in model space.");
}

#[test]
fn render_with_parameters_returns_a_picture_and_opens_nothing() {
    let mut h = harness();
    h.state_mut().run("sphere", json!({"center": [0, 0, 0], "radius": 3})).unwrap();
    let r = h.state_mut().run("render", json!({"view": "front", "width": 64, "height": 48})).unwrap();
    assert!(r["data"].as_str().is_some_and(|d| d.len() > 100), "{r}");
    h.run_steps(2);
    assert_eq!(h.state().ui.dialog, None);
}

#[test]
fn the_view_menu_opens_the_render_window() {
    let mut h = harness();
    h.state_mut().run("render", serde_json::Value::Null).unwrap();
    h.run_steps(2);
    assert_eq!(h.state().ui.dialog.as_deref(), Some("render"));
}

#[test]
fn orbit_opens_the_render_window() {
    let mut h = harness();
    h.state_mut().run("sphere", json!({"center": [0, 0, 0], "radius": 5})).unwrap();
    h.state_mut().cmdline("3dorbit");
    h.run_steps(3);
    assert_eq!(h.state().ui.dialog.as_deref(), Some("render"));
    h.get_by_label("1 solid(s); drag the picture to orbit");
}
