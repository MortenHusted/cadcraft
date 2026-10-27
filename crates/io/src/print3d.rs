//! 3D printing formats: binary STL and 3MF, written from a drawing's 3D solids.
//!
//! STL (the de facto stereolithography format) is a list of triangles with no units; slicers
//! read it in millimetres. 3MF (3MF Consortium core specification) is an OPC package (a ZIP)
//! holding an XML model with shared vertices, one object per solid and the drawing's units.
//! Both are what slicers such as Bambu Studio, PrusaSlicer and Cura open.

use cadcraft_doc::{Drawing, EntityKind, Space};
use cadcraft_geom::solid::Mesh;

use crate::{IoError, Result};

/// A solid to print: a name for slicers' object lists and its mesh.
pub struct PrintObject<'a> {
    pub name: String,
    pub mesh: &'a Mesh,
}

/// The 3D solids in `space`, named after their layer and handle.
pub fn solids<'a>(d: &'a Drawing, space: &Space) -> Vec<PrintObject<'a>> {
    d.space(space)
        .into_iter()
        .flat_map(|s| s.iter())
        .filter_map(|e| match &e.kind {
            EntityKind::Solid3d(s) => Some(PrintObject { name: format!("{} {}", e.common.layer, e.handle.hex()), mesh: s.mesh() }),
            _ => None,
        })
        .collect()
}

fn no_solids() -> IoError {
    IoError::Format("there are no 3D solids to write (STL and 3MF hold solids only)".into())
}

/// Binary STL of every triangle of `objects`: an 80-byte header, the triangle count, then per
/// triangle its unit normal and three corners as 32-bit floats and a zero attribute word.
pub fn stl(objects: &[PrintObject]) -> Result<Vec<u8>> {
    let count: usize = objects.iter().map(|o| o.mesh.triangles.len()).sum();
    if count == 0 {
        return Err(no_solids());
    }
    let n = u32::try_from(count).map_err(|_| IoError::Format("too many triangles for STL".into()))?;
    let mut out = Vec::with_capacity(84 + count * 50);
    let mut header = [b' '; 80];
    let title = b"CADCraft binary STL";
    header.iter_mut().zip(title).for_each(|(h, t)| *h = *t);
    out.extend_from_slice(&header);
    out.extend_from_slice(&n.to_le_bytes());
    for o in objects {
        for t in &o.mesh.triangles {
            let [a, b, c] = o.mesh.corners(t).unwrap_or_default();
            for v in [(b - a).cross(c - a).normalized(), a, b, c] {
                for x in [v.x, v.y, v.z] {
                    out.extend_from_slice(&(x as f32).to_le_bytes());
                }
            }
            out.extend_from_slice(&[0, 0]);
        }
    }
    Ok(out)
}

/// The 3MF unit for the drawing's `$INSUNITS` (millimetres when it declares none or another).
pub fn unit_name(d: &Drawing) -> &'static str {
    match d.header.i64("INSUNITS", 0) {
        1 => "inch",
        2 => "foot",
        5 => "centimeter",
        6 => "meter",
        13 => "micron",
        _ => "millimeter",
    }
}

fn xml_escape(s: &str) -> String {
    s.chars()
        .map(|c| match c {
            '&' => "&amp;".to_string(),
            '<' => "&lt;".to_string(),
            '>' => "&gt;".to_string(),
            '"' => "&quot;".to_string(),
            '\'' => "&apos;".to_string(),
            c if c.is_control() => " ".to_string(),
            c => c.to_string(),
        })
        .collect()
}

/// The 3MF model part: one mesh object per solid, each placed once on the build plate.
fn model_xml(objects: &[PrintObject], unit: &str) -> String {
    use std::fmt::Write as _;
    let mut x = String::new();
    x.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    let _ = writeln!(x, "<model unit=\"{unit}\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">");
    x.push_str(" <metadata name=\"Application\">CADCraft</metadata>\n <resources>\n");
    for (i, o) in objects.iter().enumerate() {
        let _ = writeln!(x, "  <object id=\"{}\" type=\"model\" name=\"{}\">\n   <mesh>\n    <vertices>", i + 1, xml_escape(&o.name));
        for v in &o.mesh.vertices {
            let _ = writeln!(x, "     <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>", v.x, v.y, v.z);
        }
        x.push_str("    </vertices>\n    <triangles>\n");
        for t in &o.mesh.triangles {
            let _ = writeln!(x, "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>", t[0], t[1], t[2]);
        }
        x.push_str("    </triangles>\n   </mesh>\n  </object>\n");
    }
    x.push_str(" </resources>\n <build>\n");
    for i in 0..objects.len() {
        let _ = writeln!(x, "  <item objectid=\"{}\"/>", i + 1);
    }
    x.push_str(" </build>\n</model>\n");
    x
}

const CONTENT_TYPES: &str = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n</Types>\n";
const RELS: &str = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>\n</Relationships>\n";

/// A 3MF package of `objects` in `unit` (see [`unit_name`]).
pub fn three_mf(objects: &[PrintObject], unit: &str) -> Result<Vec<u8>> {
    if objects.iter().all(|o| o.mesh.triangles.is_empty()) {
        return Err(no_solids());
    }
    let model = model_xml(objects, unit);
    zip_stored(&[("[Content_Types].xml", CONTENT_TYPES.as_bytes()), ("_rels/.rels", RELS.as_bytes()), ("3D/3dmodel.model", model.as_bytes())])
}

/// CRC-32 (IEEE 802.3, reflected, as ZIP uses it).
fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for b in data {
        crc ^= u32::from(*b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

/// A ZIP archive (PKWARE APPNOTE) of `files`, stored without compression.
fn zip_stored(files: &[(&str, &[u8])]) -> Result<Vec<u8>> {
    let too_big = || IoError::Format("the 3MF package would be larger than 4 GB".into());
    let u32_of = |n: usize| u32::try_from(n).map_err(|_| too_big());
    let u16_of = |n: usize| u16::try_from(n).map_err(|_| too_big());
    // MS-DOS date 1980-01-01, time 00:00: the files carry no time, so the output is reproducible.
    let (time, date) = (0u16, 0x21u16);
    let mut out = Vec::new();
    let mut central = Vec::new();
    for (name, data) in files {
        let offset = u32_of(out.len())?;
        let (crc, size, name_len) = (crc32(data), u32_of(data.len())?, u16_of(name.len())?);
        let fixed = |v: &mut Vec<u8>| {
            v.extend_from_slice(&20u16.to_le_bytes()); // version needed to extract
            v.extend_from_slice(&0u16.to_le_bytes()); // flags
            v.extend_from_slice(&0u16.to_le_bytes()); // stored
            v.extend_from_slice(&time.to_le_bytes());
            v.extend_from_slice(&date.to_le_bytes());
            v.extend_from_slice(&crc.to_le_bytes());
            v.extend_from_slice(&size.to_le_bytes());
            v.extend_from_slice(&size.to_le_bytes());
            v.extend_from_slice(&name_len.to_le_bytes());
            v.extend_from_slice(&0u16.to_le_bytes()); // extra field length
        };
        out.extend_from_slice(&0x0403_4b50u32.to_le_bytes());
        fixed(&mut out);
        out.extend_from_slice(name.as_bytes());
        out.extend_from_slice(data);
        central.extend_from_slice(&0x0201_4b50u32.to_le_bytes());
        central.extend_from_slice(&20u16.to_le_bytes()); // version made by
        fixed(&mut central);
        central.extend_from_slice(&0u16.to_le_bytes()); // comment length
        central.extend_from_slice(&0u16.to_le_bytes()); // disk number
        central.extend_from_slice(&0u16.to_le_bytes()); // internal attributes
        central.extend_from_slice(&0u32.to_le_bytes()); // external attributes
        central.extend_from_slice(&offset.to_le_bytes());
        central.extend_from_slice(name.as_bytes());
    }
    let (cd_offset, cd_size, n) = (u32_of(out.len())?, u32_of(central.len())?, u16_of(files.len())?);
    out.extend_from_slice(&central);
    out.extend_from_slice(&0x0605_4b50u32.to_le_bytes());
    out.extend_from_slice(&[0; 4]); // this disk, the central directory's disk
    out.extend_from_slice(&n.to_le_bytes());
    out.extend_from_slice(&n.to_le_bytes());
    out.extend_from_slice(&cd_size.to_le_bytes());
    out.extend_from_slice(&cd_offset.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes()); // comment length
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_geom::Vec3;
    use cadcraft_geom::solid::primitives::cuboid;

    #[test]
    fn crc32_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn stl_has_header_count_and_fifty_bytes_per_triangle() {
        let m = cuboid(Vec3::ZERO, Vec3::new(1.0, 2.0, 3.0)).unwrap();
        let bytes = stl(&[PrintObject { name: "a".into(), mesh: &m }]).unwrap();
        assert_eq!(bytes.len(), 84 + 12 * 50);
        assert_eq!(u32::from_le_bytes(bytes[80..84].try_into().unwrap()), 12);
        assert!(stl(&[]).is_err());
    }

    #[test]
    fn three_mf_is_a_zip_with_the_model_part() {
        let m = cuboid(Vec3::ZERO, Vec3::new(1.0, 2.0, 3.0)).unwrap();
        let bytes = three_mf(&[PrintObject { name: "Part <1> & co".into(), mesh: &m }], "millimeter").unwrap();
        assert_eq!(&bytes[..4], &[0x50, 0x4b, 0x03, 0x04]);
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("3D/3dmodel.model") && text.contains("[Content_Types].xml") && text.contains("_rels/.rels"));
        assert!(text.contains("name=\"Part &lt;1&gt; &amp; co\""));
        assert_eq!(text.matches("<vertex ").count(), 8);
        assert_eq!(text.matches("<triangle ").count(), 12);
        // End of central directory: three entries.
        let eocd = bytes.len() - 22;
        assert_eq!(&bytes[eocd..eocd + 4], &[0x50, 0x4b, 0x05, 0x06]);
        assert_eq!(u16::from_le_bytes([bytes[eocd + 10], bytes[eocd + 11]]), 3);
    }
}
