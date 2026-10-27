//! Reading 3MF packages, the 3D Manufacturing Format slicers such as Bambu Studio and
//! PrusaSlicer save, into 3D solids: each mesh placed on the build plate becomes one solid in
//! model space, so parts made elsewhere can be measured, changed and printed again.
//!
//! A 3MF file is a ZIP package whose model part lists mesh objects (shared vertices, triangles)
//! and the build items that place them. An object may instead be an assembly of components,
//! each another object with a transform, possibly kept in another model part of the package
//! (the production extension's `path`, which slicers use for one part per object).

use std::collections::HashMap;

use cadcraft_doc::{Common, Drawing, EntityKind, Solid3d, Space};
use cadcraft_geom::solid::{MAX_TRIANGLES, Mesh};
use cadcraft_geom::{Mat4, Vec3};
use quick_xml::Reader;
use quick_xml::XmlVersion;
use quick_xml::events::{BytesStart, Event};

use crate::{IoError, Result};

/// Most files a package may list.
const MAX_ENTRIES: usize = 100_000;
/// Largest size of one part once decompressed.
const MAX_PART: usize = 1 << 30;
/// Deepest nesting of components in components.
const MAX_DEPTH: usize = 32;
/// Most objects placed, components included: shared components multiply.
const MAX_PLACEMENTS: usize = 1_000_000;
/// The model part when the package doesn't name one.
const DEFAULT_MODEL: &str = "3D/3dmodel.model";
const MODEL_REL: &str = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";

fn damaged(what: &str) -> IoError {
    IoError::Format(format!("this 3MF file is damaged: {what}"))
}

fn u16_at(b: &[u8], at: usize) -> Option<usize> {
    b.get(at..at.checked_add(2)?).and_then(|x| <[u8; 2]>::try_from(x).ok()).map(|x| usize::from(u16::from_le_bytes(x)))
}

fn u32_at(b: &[u8], at: usize) -> Option<usize> {
    b.get(at..at.checked_add(4)?).and_then(|x| <[u8; 4]>::try_from(x).ok()).and_then(|x| usize::try_from(u32::from_le_bytes(x)).ok())
}

fn u64_at(b: &[u8], at: usize) -> Option<usize> {
    b.get(at..at.checked_add(8)?).and_then(|x| <[u8; 8]>::try_from(x).ok()).and_then(|x| usize::try_from(u64::from_le_bytes(x)).ok())
}

/// One file in the ZIP package.
struct Entry {
    method: usize,
    compressed: usize,
    offset: usize,
}

/// The package's files by name (without a leading `/`), from the ZIP central directory.
fn entries(b: &[u8]) -> Result<HashMap<String, Entry>> {
    // The end-of-central-directory record: 22 bytes, then a comment of up to 65,535.
    let last = b.len().checked_sub(22).ok_or_else(|| IoError::Format("not a 3MF file (too short to be a ZIP package)".into()))?;
    let eocd = (last.saturating_sub(0xFFFF)..=last)
        .rev()
        .find(|&i| b.get(i..i + 4) == Some(&[0x50, 0x4b, 0x05, 0x06]))
        .ok_or_else(|| IoError::Format("not a 3MF file (not a ZIP package)".into()))?;
    let count = u16_at(b, eocd + 10).ok_or_else(|| damaged("its directory is cut short"))?;
    let mut at = u32_at(b, eocd + 16).ok_or_else(|| damaged("its directory is cut short"))?;
    let mut out = HashMap::new();
    for _ in 0..count.min(MAX_ENTRIES) {
        if b.get(at..at + 4) != Some(&[0x50, 0x4b, 0x01, 0x02]) {
            return Err(damaged("its directory is cut short"));
        }
        let field = |off: usize| u16_at(b, at + off).ok_or_else(|| damaged("its directory is cut short"));
        let (name_len, extra_len, comment_len) = (field(28)?, field(30)?, field(32)?);
        let method = field(10)?;
        let mut compressed = u32_at(b, at + 20).ok_or_else(|| damaged("its directory is cut short"))?;
        let mut size = u32_at(b, at + 24).ok_or_else(|| damaged("its directory is cut short"))?;
        let mut offset = u32_at(b, at + 42).ok_or_else(|| damaged("its directory is cut short"))?;
        let name_at = at + 46;
        let name = b.get(name_at..name_at + name_len).ok_or_else(|| damaged("its directory is cut short"))?;
        // ZIP64: sizes and offset too big for 32 bits are in an extra field, in this order.
        let mut extra = b.get(name_at + name_len..name_at + name_len + extra_len).unwrap_or(&[]);
        while let (Some(id), Some(len)) = (u16_at(extra, 0), u16_at(extra, 2)) {
            let data = extra.get(4..4 + len).unwrap_or(&[]);
            if id == 1 {
                let mut k = 0;
                let mut wide = |v: &mut usize| -> Result<()> {
                    if *v == 0xFFFF_FFFF {
                        *v = u64_at(data, k).ok_or_else(|| damaged("a ZIP64 field is cut short"))?;
                        k += 8;
                    }
                    Ok(())
                };
                wide(&mut size)?;
                wide(&mut compressed)?;
                wide(&mut offset)?;
            }
            extra = extra.get(4 + len..).unwrap_or(&[]);
        }
        let name = String::from_utf8_lossy(name).trim_start_matches('/').to_string();
        out.insert(name, Entry { method, compressed, offset });
        at = name_at + name_len + extra_len + comment_len;
    }
    Ok(out)
}

/// The contents of one file in the package.
fn unpack(b: &[u8], e: &Entry) -> Result<Vec<u8>> {
    let o = e.offset;
    if b.get(o..o + 4) != Some(&[0x50, 0x4b, 0x03, 0x04]) {
        return Err(damaged("a file in it is missing"));
    }
    let start = (|| o.checked_add(30)?.checked_add(u16_at(b, o + 26)?)?.checked_add(u16_at(b, o + 28)?))()
        .ok_or_else(|| damaged("a file in it is cut short"))?;
    let data = start.checked_add(e.compressed).and_then(|end| b.get(start..end)).ok_or_else(|| damaged("a file in it is cut short"))?;
    match e.method {
        0 if data.len() <= MAX_PART => Ok(data.to_vec()),
        8 => miniz_oxide::inflate::decompress_to_vec_with_limit(data, MAX_PART).map_err(|_| damaged("a compressed file in it can't be read")),
        0 => Err(IoError::Format("a file in this 3MF package is too big".into())),
        m => Err(IoError::Format(format!("this 3MF file uses ZIP compression method {m}, which CADCraft can't read"))),
    }
}

fn attr(e: &BytesStart, name: &str) -> Option<String> {
    e.attributes()
        .flatten()
        .find(|a| a.key.local_name().into_inner() == name)
        .and_then(|a| a.normalized_value(XmlVersion::Explicit1_0).ok().map(|v| v.into_owned()))
}

fn num(e: &BytesStart, name: &str) -> Option<f64> {
    attr(e, name).and_then(|v| v.trim().parse::<f64>().ok()).filter(|v| v.is_finite())
}

fn index(e: &BytesStart, name: &str) -> Option<usize> {
    attr(e, name).and_then(|v| v.trim().parse::<usize>().ok())
}

/// A 3MF transform: twelve numbers, the 3×3 linear part row by row then the translation,
/// applied to row vectors (`p' = p·M`).
fn transform(e: &BytesStart) -> Result<Mat4> {
    let Some(text) = attr(e, "transform") else { return Ok(Mat4::IDENTITY) };
    let v: Vec<f64> = text.split_whitespace().filter_map(|x| x.parse::<f64>().ok()).filter(|x| x.is_finite()).collect();
    let [m00, m01, m02, m10, m11, m12, m20, m21, m22, m30, m31, m32] = v.as_slice() else {
        return Err(damaged(&format!("a transform isn't twelve numbers: `{text}`")));
    };
    Ok(Mat4 { m: [[*m00, *m10, *m20, *m30], [*m01, *m11, *m21, *m31], [*m02, *m12, *m22, *m32], [0.0, 0.0, 0.0, 1.0]] })
}

struct Component {
    /// The model part holding the object; `None` for the same part.
    path: Option<String>,
    object: usize,
    transform: Mat4,
}

#[derive(Default)]
struct Object {
    mesh: Mesh,
    components: Vec<Component>,
    /// Supports and other non-model objects aren't part of the print.
    printed: bool,
}

#[derive(Default)]
struct Model {
    unit: Option<String>,
    objects: HashMap<usize, Object>,
    /// The build plate: objects placed by the root model part.
    build: Vec<Component>,
}

fn component(e: &BytesStart) -> Result<Component> {
    let object = index(e, "objectid").ok_or_else(|| damaged("a component or build item names no object"))?;
    Ok(Component { path: attr(e, "path").map(|p| p.trim_start_matches('/').to_string()), object, transform: transform(e)? })
}

fn parse_model(bytes: &[u8]) -> Result<Model> {
    let text = std::str::from_utf8(bytes).map_err(|_| damaged("its model isn't UTF-8 text"))?;
    let mut reader = Reader::from_str(text);
    reader.config_mut().trim_text(true);
    let mut model = Model::default();
    // The object being read, and its vertex count when the triangles begin.
    let mut current: Option<(usize, Object)> = None;
    let mut triangles = 0usize;
    loop {
        let (e, open) = match reader.read_event().map_err(|e| damaged(&e.to_string()))? {
            Event::Start(e) => (e, true),
            Event::Empty(e) => (e, false),
            Event::End(e) => {
                if e.local_name().into_inner() == "object"
                    && let Some((id, o)) = current.take()
                {
                    model.objects.insert(id, o);
                }
                continue;
            }
            Event::Eof => break,
            _ => continue,
        };
        match e.local_name().into_inner() {
            "model" => model.unit = attr(&e, "unit"),
            "object" => {
                let id = index(&e, "id").ok_or_else(|| damaged("an object has no id"))?;
                let printed = attr(&e, "type").is_none_or(|t| t == "model");
                let o = Object { printed, ..Object::default() };
                if open {
                    current = Some((id, o));
                } else {
                    model.objects.insert(id, o);
                }
            }
            "vertex" => {
                let Some((_, o)) = current.as_mut() else { continue };
                let (Some(x), Some(y), Some(z)) = (num(&e, "x"), num(&e, "y"), num(&e, "z")) else {
                    return Err(damaged("a vertex isn't three numbers"));
                };
                if o.mesh.vertices.len() >= MAX_TRIANGLES {
                    return Err(too_big());
                }
                o.mesh.vertices.push(Vec3::new(x, y, z));
            }
            "triangle" => {
                let Some((_, o)) = current.as_mut() else { continue };
                let n = o.mesh.vertices.len();
                let v = |k: &str| index(&e, k).filter(|i| *i < n).and_then(|i| u32::try_from(i).ok());
                let (Some(a), Some(b), Some(c)) = (v("v1"), v("v2"), v("v3")) else {
                    return Err(damaged(&format!("a triangle names a vertex the object doesn't have ({n} vertices)")));
                };
                triangles += 1;
                if triangles > MAX_TRIANGLES {
                    return Err(too_big());
                }
                // A triangle with a repeated corner encloses nothing.
                if a != b && b != c && c != a {
                    o.mesh.triangles.push([a, b, c]);
                }
            }
            "component" => {
                if let Some((_, o)) = current.as_mut() {
                    o.components.push(component(&e)?);
                }
            }
            "item" => model.build.push(component(&e)?),
            _ => {}
        }
    }
    Ok(model)
}

fn too_big() -> IoError {
    IoError::Format(format!("the 3MF file has more than {MAX_TRIANGLES} triangles"))
}

/// The root model part named by the package relationships.
fn root_part(b: &[u8], files: &HashMap<String, Entry>) -> Result<String> {
    let Some(rels) = files.get("_rels/.rels") else { return Ok(DEFAULT_MODEL.into()) };
    let rels = unpack(b, rels)?;
    let text = String::from_utf8_lossy(&rels);
    let mut reader = Reader::from_str(&text);
    loop {
        match reader.read_event().map_err(|e| damaged(&e.to_string()))? {
            Event::Start(e) | Event::Empty(e) if e.local_name().into_inner() == "Relationship" && attr(&e, "Type").as_deref() == Some(MODEL_REL) => {
                if let Some(target) = attr(&e, "Target") {
                    return Ok(target.trim_start_matches('/').to_string());
                }
            }
            Event::Eof => return Ok(DEFAULT_MODEL.into()),
            _ => {}
        }
    }
}

/// Places the build's meshes, reading the model parts components refer to as needed.
struct Package<'a> {
    bytes: &'a [u8],
    files: HashMap<String, Entry>,
    models: HashMap<String, Model>,
    meshes: Vec<Mesh>,
    triangles: usize,
    placements: usize,
}

impl Package<'_> {
    fn load(&mut self, part: &str) -> Result<()> {
        if !self.models.contains_key(part) {
            let entry = self.files.get(part).ok_or_else(|| damaged(&format!("its model part {part} is missing")))?;
            let model = parse_model(&unpack(self.bytes, entry)?)?;
            self.models.insert(part.to_string(), model);
        }
        Ok(())
    }

    /// Every mesh of object `id` in `part`, placed by `to_build`, as one solid each.
    fn place(&mut self, part: &str, id: usize, to_build: &Mat4, depth: usize) -> Result<()> {
        if depth > MAX_DEPTH {
            return Err(damaged(&format!("its components nest more than {MAX_DEPTH} deep")));
        }
        self.placements += 1;
        if self.placements > MAX_PLACEMENTS {
            return Err(IoError::Format(format!("the 3MF file places more than {MAX_PLACEMENTS} objects")));
        }
        self.load(part)?;
        let (mesh, components) = match self.models.get(part).and_then(|m| m.objects.get(&id)) {
            Some(o) if !o.printed => return Ok(()),
            Some(o) => {
                let components: Vec<(String, usize, Mat4)> =
                    o.components.iter().map(|c| (c.path.clone().unwrap_or_else(|| part.to_string()), c.object, c.transform)).collect();
                ((!o.mesh.triangles.is_empty()).then(|| o.mesh.clone()), components)
            }
            None => return Err(damaged(&format!("object {id} is missing from {part}"))),
        };
        if let Some(mut mesh) = mesh {
            self.triangles = self.triangles.saturating_add(mesh.triangles.len());
            if self.triangles > MAX_TRIANGLES {
                return Err(too_big());
            }
            mesh.transform(to_build);
            self.meshes.push(mesh);
        }
        for (path, object, t) in components {
            self.place(&path, object, &t.then(to_build), depth + 1)?;
        }
        Ok(())
    }
}

/// The `$INSUNITS` code for a 3MF unit (millimetres when it names none).
fn insunits(unit: Option<&str>) -> i64 {
    match unit {
        Some("micron") => 13,
        Some("centimeter") => 5,
        Some("inch") => 1,
        Some("foot") => 2,
        Some("meter") => 6,
        _ => 4,
    }
}

/// The meshes a 3MF package places on its build plate, and the file's `$INSUNITS`.
pub fn meshes(bytes: &[u8]) -> Result<(Vec<Mesh>, i64)> {
    let files = entries(bytes)?;
    let root = root_part(bytes, &files)?;
    let mut p = Package { bytes, files, models: HashMap::new(), meshes: Vec::new(), triangles: 0, placements: 0 };
    p.load(&root)?;
    let (build, unit) = match p.models.get(&root) {
        Some(m) => (
            m.build.iter().map(|c| (c.path.clone().unwrap_or_else(|| root.clone()), c.object, c.transform)).collect::<Vec<_>>(),
            insunits(m.unit.as_deref()),
        ),
        None => (Vec::new(), 4),
    };
    for (part, object, t) in build {
        p.place(&part, object, &t, 0)?;
    }
    if p.meshes.is_empty() {
        return Err(IoError::Format("the 3MF file places no mesh on its build plate".into()));
    }
    Ok((p.meshes, unit))
}

/// A drawing holding each mesh of the 3MF package's build as a solid in model space, in the
/// file's units.
pub fn read(bytes: &[u8]) -> Result<Drawing> {
    let (meshes, unit) = meshes(bytes)?;
    let mut d = Drawing::new_metric();
    d.header.set_i64("INSUNITS", unit);
    for m in meshes {
        d.add(&Space::Model, Common::default(), EntityKind::Solid3d(Solid3d::new(m))).map_err(|e| IoError::Format(e.to_string()))?;
    }
    Ok(d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::print3d::{PrintObject, three_mf};
    use cadcraft_geom::solid::primitives::{cuboid, sphere};

    #[test]
    fn a_3mf_written_by_3dprint_comes_back_as_closed_solids() {
        let a = cuboid(Vec3::ZERO, Vec3::new(10.0, 20.0, 5.0)).unwrap();
        let b = sphere(Vec3::new(30.0, 0.0, 5.0), 5.0, 48).unwrap();
        let bytes = three_mf(&[PrintObject { name: "box".into(), mesh: &a }, PrintObject { name: "ball".into(), mesh: &b }], "inch").unwrap();
        let (back, unit) = meshes(&bytes).unwrap();
        assert_eq!(unit, 1);
        assert_eq!(back.len(), 2);
        for (m, orig) in back.iter().zip([&a, &b]) {
            assert!(m.check().is_closed());
            assert_eq!(m.triangles.len(), orig.triangles.len());
            assert!((m.volume() - orig.volume()).abs() < 1e-6 * orig.volume());
        }
        let d = read(&bytes).unwrap();
        assert_eq!(d.model.len(), 2);
        assert_eq!(d.header.i64("INSUNITS", 0), 1);
    }

    /// A deflated package in the layout slicers write: the root part places an assembly whose
    /// components live in another part, moved and mirrored.
    #[test]
    fn components_in_other_parts_are_placed_by_their_transforms() {
        let unit = cuboid(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)).unwrap();
        let mut mesh = String::from("<vertices>");
        for p in &unit.vertices {
            mesh.push_str(&format!("<vertex x=\"{}\" y=\"{}\" z=\"{}\"/>", p.x, p.y, p.z));
        }
        mesh.push_str("</vertices><triangles>");
        for [a, b, c] in &unit.triangles {
            mesh.push_str(&format!("<triangle v1=\"{a}\" v2=\"{b}\" v3=\"{c}\"/>"));
        }
        mesh.push_str("</triangles>");
        let part = format!(
            "<?xml version=\"1.0\"?><model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\"><resources><object id=\"7\" type=\"model\"><mesh>{mesh}</mesh></object><object id=\"8\" type=\"support\"><mesh>{mesh}</mesh></object></resources><build/></model>"
        );
        let root = "<?xml version=\"1.0\"?><model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\" xmlns:p=\"http://schemas.microsoft.com/3dmanufacturing/production/2015/06\"><resources><object id=\"1\" type=\"model\"><components><component p:path=\"/3D/Objects/part.model\" objectid=\"7\" transform=\"2 0 0 0 2 0 0 0 2 10 0 0\"/><component p:path=\"/3D/Objects/part.model\" objectid=\"7\" transform=\"-1 0 0 0 1 0 0 0 1 0 0 0\"/><component p:path=\"/3D/Objects/part.model\" objectid=\"8\"/></components></object></resources><build><item objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 0 0 5\"/></build></model>";
        let rels = "<?xml version=\"1.0\"?><Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\"><Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/></Relationships>";
        let bytes =
            zip_deflated(&[("_rels/.rels", rels.as_bytes()), ("3D/3dmodel.model", root.as_bytes()), ("3D/Objects/part.model", part.as_bytes())]);
        let (back, unit) = meshes(&bytes).unwrap();
        assert_eq!(unit, 4);
        // The support object is left out.
        assert_eq!(back.len(), 2);
        let (big, mirrored) = (&back[0], &back[1]);
        assert!((big.volume() - 8.0).abs() < 1e-12);
        let b = big.bounds();
        assert_eq!((b.min, b.max), (Vec3::new(10.0, 0.0, 5.0), Vec3::new(12.0, 2.0, 7.0)));
        // Mirrored components keep facing outward.
        assert!((mirrored.volume() - 1.0).abs() < 1e-12);
        assert_eq!(mirrored.bounds().min, Vec3::new(-1.0, 0.0, 5.0));
        assert!(mirrored.check().is_closed());
    }

    #[test]
    fn hostile_3mf_is_an_error_not_a_crash() {
        let unit = cuboid(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)).unwrap();
        let good = three_mf(&[PrintObject { name: "x".into(), mesh: &unit }], "millimeter").unwrap();
        for n in [0, 1, 21, 22, 40, good.len() / 2, good.len() - 30, good.len() - 1] {
            assert!(meshes(&good[..n]).is_err(), "{n}");
        }
        let mut flipped = good.clone();
        for i in (0..flipped.len()).step_by(7) {
            flipped[i] ^= 0x5a;
        }
        let _ = meshes(&flipped);
        let model = |body: &str| {
            let xml = format!("<model unit=\"millimeter\"><resources>{body}</resources><build><item objectid=\"1\"/></build></model>");
            meshes(&zip_deflated(&[("3D/3dmodel.model", xml.as_bytes())]))
        };
        let bad_index = "<object id=\"1\"><mesh><vertices><vertex x=\"0\" y=\"0\" z=\"0\"/></vertices><triangles><triangle v1=\"0\" v2=\"1\" v3=\"99999999999999999999\"/></triangles></mesh></object>";
        let cycle = "<object id=\"1\"><components><component objectid=\"1\"/></components></object>";
        let nan = "<object id=\"1\"><mesh><vertices><vertex x=\"NaN\" y=\"0\" z=\"0\"/></vertices></mesh></object>";
        let short = "<object id=\"1\"><components><component objectid=\"2\" transform=\"1 0 0\"/></components></object>";
        for body in [bad_index, cycle, nan, short, "", "<object id=\"1\"/>"] {
            assert!(model(body).is_err(), "{body}");
        }
    }

    /// A ZIP package with each file deflated, as slicers write them.
    fn zip_deflated(files: &[(&str, &[u8])]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut central = Vec::new();
        for (name, data) in files {
            let packed = miniz_oxide::deflate::compress_to_vec(data, 6);
            let offset = out.len() as u32;
            let header = |sig: u32, central: bool| {
                let mut h = Vec::new();
                h.extend_from_slice(&sig.to_le_bytes());
                if central {
                    h.extend_from_slice(&20u16.to_le_bytes());
                }
                h.extend_from_slice(&[20, 0, 0, 0, 8, 0, 0, 0, 0, 0]);
                h.extend_from_slice(&0u32.to_le_bytes()); // CRC (not checked)
                h.extend_from_slice(&(packed.len() as u32).to_le_bytes());
                h.extend_from_slice(&(data.len() as u32).to_le_bytes());
                h.extend_from_slice(&(name.len() as u16).to_le_bytes());
                h.extend_from_slice(&0u16.to_le_bytes());
                if central {
                    h.extend_from_slice(&[0; 10]); // comment length, disk, attributes
                    h.extend_from_slice(&offset.to_le_bytes());
                }
                h.extend_from_slice(name.as_bytes());
                h
            };
            out.extend(header(0x0403_4b50, false));
            out.extend_from_slice(&packed);
            central.extend(header(0x0201_4b50, true));
        }
        let at = out.len() as u32;
        out.extend_from_slice(&central);
        out.extend_from_slice(&0x0605_4b50u32.to_le_bytes());
        out.extend_from_slice(&[0; 4]);
        out.extend_from_slice(&(files.len() as u16).to_le_bytes());
        out.extend_from_slice(&(files.len() as u16).to_le_bytes());
        out.extend_from_slice(&(central.len() as u32).to_le_bytes());
        out.extend_from_slice(&at.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out
    }
}
