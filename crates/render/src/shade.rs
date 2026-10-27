//! Shaded pictures of 3D solids: a parallel projection along a view direction, triangles filled
//! through a depth buffer and lit by a light over the viewer's shoulder, and the solids' edges
//! (sharp edges and the outline from this side) drawn on top where they are not hidden. RENDER
//! uses it, so a person or an agent driving CADCraft can see what a model looks like.
//!
//! Drawn at twice the size and averaged down, so edges are smooth.

use cadcraft_color::Rgb;
use cadcraft_geom::solid::{FEATURE_ANGLE, Mesh};
use cadcraft_geom::{Mat4, Vec3};
use tiny_skia::{IntSize, Pixmap};

/// Samples per pixel along each axis.
const SUPERSAMPLE: u32 = 2;
/// Largest picture side, in pixels: at most 4096 × 4096 samples, about 117 MB while drawing.
pub const MAX_SIDE: u32 = 2048;

/// One solid to draw and its colour.
pub struct ShadeItem<'a> {
    pub mesh: &'a Mesh,
    pub color: Rgb,
}

/// What a shaded picture shows and how big it is.
#[derive(Clone, Copy, Debug)]
pub struct ShadeOptions {
    pub width: u32,
    pub height: u32,
    /// Direction from the model towards the viewer (e.g. `[1, -1, 1]`: south-east isometric).
    pub view: Vec3,
    /// The drawing's up, which is up in the picture: Z, or Y for a model built Y up.
    pub up: Vec3,
    pub edges: bool,
    pub background: Rgb,
    pub edge_color: Rgb,
}

impl Default for ShadeOptions {
    fn default() -> Self {
        ShadeOptions {
            width: 1200,
            height: 900,
            view: Vec3::new(1.0, -1.0, 1.0),
            up: Vec3::Z,
            edges: true,
            background: Rgb(255, 255, 255),
            edge_color: Rgb(40, 44, 52),
        }
    }
}

/// `v`, a direction in a drawing with Z up, for a drawing with `up` up: turned by the least
/// rotation taking Z to `up` (half a turn about X when `up` is -Z). With Y up, -Y (the front)
/// becomes +Z and Z (the top) becomes +Y.
pub fn from_z_up(v: Vec3, up: Vec3) -> Vec3 {
    let up = up.normalized();
    let axis = Vec3::Z.cross(up);
    if axis.normalized() != Vec3::ZERO {
        Mat4::rotate_about_axis(Vec3::ZERO, axis, axis.len().atan2(up.z)).apply_vec(v)
    } else if up.z < 0.0 {
        Vec3::new(v.x, -v.y, -v.z)
    } else {
        v
    }
}

/// The picture's axes: right and up on screen, and back (towards the viewer).
struct Camera {
    right: Vec3,
    up: Vec3,
    back: Vec3,
}

impl Camera {
    fn new(view: Vec3, up: Vec3) -> Camera {
        let back = if view.len() > 1e-12 { view.normalized() } else { Vec3::new(1.0, -1.0, 1.0).normalized() };
        let up = Some(up.normalized()).filter(|u| *u != Vec3::ZERO).unwrap_or(Vec3::Z);
        // `up` is up on screen, unless looking straight along it (plan views: Y, turned with
        // `up`, is up).
        let world_up = if back.cross(up).len() < 1e-6 { from_z_up(Vec3::Y, up) } else { up };
        let right = world_up.cross(back).normalized();
        Camera { right, up: back.cross(right), back }
    }
    /// Screen x, screen y, and depth (larger is nearer the viewer).
    fn project(&self, p: Vec3) -> Vec3 {
        Vec3::new(p.dot(self.right), p.dot(self.up), p.dot(self.back))
    }
}

/// Colour, depth and size of the supersampled picture.
struct Canvas {
    w: usize,
    h: usize,
    color: Vec<[u8; 3]>,
    depth: Vec<f32>,
}

impl Canvas {
    fn set(&mut self, x: usize, y: usize, z: f32, c: [u8; 3], bias: f32) {
        if x >= self.w || y >= self.h {
            return;
        }
        let i = y * self.w + x;
        if let (Some(d), Some(px)) = (self.depth.get_mut(i), self.color.get_mut(i))
            && z + bias >= *d
        {
            *d = z.max(*d);
            *px = c;
        }
    }
}

fn rgb(c: Rgb) -> [u8; 3] {
    [c.0, c.1, c.2]
}

/// Fill triangle `a b c` (screen x, y, depth) with `c` through the depth buffer.
fn fill(cv: &mut Canvas, [a, b, c]: [Vec3; 3], color: [u8; 3]) {
    let area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if area.abs() < 1e-12 {
        return;
    }
    let (w, h) = (cv.w as f64, cv.h as f64);
    let x0 = a.x.min(b.x).min(c.x).floor().max(0.0);
    let x1 = a.x.max(b.x).max(c.x).ceil().min(w - 1.0);
    let y0 = a.y.min(b.y).min(c.y).floor().max(0.0);
    let y1 = a.y.max(b.y).max(c.y).ceil().min(h - 1.0);
    if x0 > x1 || y0 > y1 {
        return;
    }
    let edge = |p: Vec3, q: Vec3, x: f64, y: f64| (q.x - p.x) * (y - p.y) - (q.y - p.y) * (x - p.x);
    let mut y = y0;
    while y <= y1 {
        let mut x = x0;
        let py = y + 0.5;
        while x <= x1 {
            let px = x + 0.5;
            let (w0, w1, w2) = (edge(b, c, px, py) / area, edge(c, a, px, py) / area, edge(a, b, px, py) / area);
            if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 {
                let z = (w0 * a.z + w1 * b.z + w2 * c.z) as f32;
                cv.set(x as usize, y as usize, z, color, 0.0);
            }
            x += 1.0;
        }
        y += 1.0;
    }
}

/// Draw a line `a`–`b` (screen x, y, depth) `width` samples wide where it is not behind a face
/// by more than `bias`.
fn line(cv: &mut Canvas, a: Vec3, b: Vec3, width: f64, color: [u8; 3], bias: f32) {
    let len = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
    let steps = (len.ceil() as usize).clamp(1, 1 << 16);
    let r = (width / 2.0).max(0.5);
    let ri = r.ceil() as i64;
    for k in 0..=steps {
        let t = k as f64 / steps as f64;
        let p = a.lerp(b, t);
        for dy in -ri..=ri {
            for dx in -ri..=ri {
                let (fx, fy) = (dx as f64, dy as f64);
                if fx * fx + fy * fy > r * r {
                    continue;
                }
                let (x, y) = ((p.x + fx).floor(), (p.y + fy).floor());
                if x >= 0.0 && y >= 0.0 {
                    cv.set(x as usize, y as usize, p.z as f32, color, bias);
                }
            }
        }
    }
}

/// A shaded picture of `items` (`None` for a picture size of zero).
pub fn shade(items: &[ShadeItem], o: &ShadeOptions) -> Option<Pixmap> {
    let (out_w, out_h) = (o.width.clamp(1, MAX_SIDE), o.height.clamp(1, MAX_SIDE));
    let cam = Camera::new(o.view, o.up);
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = -lo;
    for it in items {
        for p in it.mesh.triangle_points().flatten() {
            let q = cam.project(p);
            lo = lo.min(q);
            hi = hi.max(q);
        }
    }
    let (w, h) = ((out_w * SUPERSAMPLE) as usize, (out_h * SUPERSAMPLE) as usize);
    let mut cv = Canvas { w, h, color: vec![rgb(o.background); w * h], depth: vec![f32::NEG_INFINITY; w * h] };
    if lo.x.is_finite() && hi.x.is_finite() {
        let margin = 0.06;
        let (bw, bh) = ((hi.x - lo.x).max(1e-9), (hi.y - lo.y).max(1e-9));
        let scale = (w as f64 / bw).min(h as f64 / bh) * (1.0 - 2.0 * margin);
        let (cx, cy) = ((lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0);
        let screen = |p: Vec3| {
            let q = cam.project(p);
            Vec3::new(w as f64 / 2.0 + (q.x - cx) * scale, h as f64 / 2.0 - (q.y - cy) * scale, q.z)
        };
        // A key light from above left of the viewer, and some light from everywhere.
        let light = (cam.back + cam.up * 0.6 - cam.right * 0.4).normalized();
        for it in items {
            let base = rgb(it.color);
            for [a, b, c] in it.mesh.triangle_points() {
                let n = (b - a).cross(c - a).normalized();
                let facing = n.dot(cam.back);
                if facing <= 0.0 {
                    continue;
                }
                let k = (0.32 + 0.68 * n.dot(light).max(0.0)) as f32;
                fill(&mut cv, [screen(a), screen(b), screen(c)], base.map(|v| (f32::from(v) * k).round().clamp(0.0, 255.0) as u8));
            }
        }
        if o.edges {
            let bias = ((hi.z - lo.z).abs() * 2e-3).max(1e-9) as f32;
            let ec = rgb(o.edge_color);
            for it in items {
                for [a, b] in it.mesh.wire_edges(FEATURE_ANGLE, cam.back) {
                    line(&mut cv, screen(a), screen(b), 1.3 * f64::from(SUPERSAMPLE), ec, bias);
                }
            }
        }
    }
    // Average each block of samples into one pixel.
    let mut data = Vec::with_capacity((out_w * out_h * 4) as usize);
    let s = SUPERSAMPLE as usize;
    for y in 0..out_h as usize {
        for x in 0..out_w as usize {
            let mut sum = [0u32; 3];
            for dy in 0..s {
                for dx in 0..s {
                    if let Some(px) = cv.color.get((y * s + dy) * w + x * s + dx) {
                        sum.iter_mut().zip(px).for_each(|(a, b)| *a += u32::from(*b));
                    }
                }
            }
            let n = (s * s) as u32;
            data.extend(sum.map(|v| ((v + n / 2) / n).min(255) as u8));
            data.push(255);
        }
    }
    Pixmap::from_vec(data, IntSize::from_wh(out_w, out_h)?)
}

/// [`shade`] as PNG bytes.
pub fn shade_png(items: &[ShadeItem], o: &ShadeOptions) -> Option<Vec<u8>> {
    shade(items, o)?.encode_png().ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_geom::solid::primitives::cuboid;

    #[test]
    fn a_box_seen_from_above_fills_the_middle_and_leaves_the_margin() {
        let m = cuboid(Vec3::ZERO, Vec3::new(10.0, 10.0, 2.0)).unwrap();
        let o = ShadeOptions { width: 100, height: 100, view: Vec3::Z, edges: false, ..ShadeOptions::default() };
        let px = shade(&[ShadeItem { mesh: &m, color: Rgb(200, 0, 0) }], &o).unwrap();
        let at = |x: u32, y: u32| px.pixel(x, y).unwrap();
        // Lit from over the viewer's shoulder: red, darker than full.
        assert!(at(50, 50).red() > 100 && at(50, 50).green() == 0, "{:?}", at(50, 50));
        assert_eq!((at(1, 1).red(), at(1, 1).green()), (255, 255), "the margin is background");
    }

    #[test]
    fn nearer_faces_hide_farther_ones() {
        let near = cuboid(Vec3::new(0.0, 0.0, 5.0), Vec3::new(10.0, 10.0, 6.0)).unwrap();
        let far = cuboid(Vec3::new(2.0, 2.0, 0.0), Vec3::new(8.0, 8.0, 1.0)).unwrap();
        let o = ShadeOptions { width: 64, height: 64, view: Vec3::Z, edges: false, ..ShadeOptions::default() };
        let items = [ShadeItem { mesh: &far, color: Rgb(0, 0, 255) }, ShadeItem { mesh: &near, color: Rgb(0, 255, 0) }];
        let px = shade(&items, &o).unwrap();
        let c = px.pixel(32, 32).unwrap();
        assert!(c.green() > 0 && c.blue() == 0, "{c:?}");
        assert!(shade_png(&items, &o).is_some_and(|b| b.starts_with(&[0x89, b'P', b'N', b'G'])));
    }

    #[test]
    fn from_z_up_turns_directions_with_up() {
        let near = |a: Vec3, b: Vec3| assert!(a.dist(b) < 1e-12, "{a:?} vs {b:?}");
        // Y up: the front (-Y) is seen from +Z, the top from +Y, and right stays right.
        near(from_z_up(-Vec3::Y, Vec3::Y), Vec3::Z);
        near(from_z_up(Vec3::Z, Vec3::Y), Vec3::Y);
        near(from_z_up(Vec3::X, Vec3::Y), Vec3::X);
        // Z goes to `up` whichever way it points; Z, or no up at all, changes nothing.
        for up in [Vec3::X, -Vec3::X, Vec3::new(1.0, 2.0, -3.0), -Vec3::Z] {
            near(from_z_up(Vec3::Z, up), up.normalized());
        }
        let se = Vec3::new(1.0, -1.0, 1.0);
        near(from_z_up(se, Vec3::Z), se);
        near(from_z_up(se, Vec3::ZERO), se);
    }

    #[test]
    fn up_is_up_in_the_picture() {
        // A post along Y, seen from +X: standing with Y up, lying down with Z up.
        let m = cuboid(Vec3::ZERO, Vec3::new(2.0, 10.0, 2.0)).unwrap();
        let red_at = |up: Vec3, x: u32, y: u32| {
            let o = ShadeOptions { width: 100, height: 100, view: Vec3::X, up, edges: false, ..ShadeOptions::default() };
            shade(&[ShadeItem { mesh: &m, color: Rgb(200, 0, 0) }], &o).unwrap().pixel(x, y).unwrap().green() == 0
        };
        assert!(red_at(Vec3::Y, 50, 10) && !red_at(Vec3::Y, 10, 50));
        assert!(red_at(Vec3::Z, 10, 50) && !red_at(Vec3::Z, 50, 10));
    }

    #[test]
    fn empty_and_huge_requests_are_safe() {
        let o = ShadeOptions { width: 0, height: 100_000, ..ShadeOptions::default() };
        // Clamped to 1 × MAX_SIDE.
        let px = shade(&[], &o).unwrap();
        assert_eq!((px.width(), px.height()), (1, MAX_SIDE));
    }
}
