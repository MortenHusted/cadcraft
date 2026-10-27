//! 3D solids in DXF as polyface meshes.
//!
//! A 3DSOLID's own data is ACIS, which CADCraft doesn't write. Its closed mesh is written as a
//! polyface mesh (POLYLINE with flag 64, the DXF Reference's "polyface mesh"): every program
//! reading R2000 DXF sees the surface, with the edges between faces that meet smoothly hidden.
//! `CADCRAFT` xdata on the POLYLINE (`1000 SOLID3D`, then the part number and the number of
//! parts as `1070`s) makes it a solid again when CADCraft reads it. A polyface numbers its
//! vertices and faces with 16-bit integers, so a bigger solid is written as consecutive parts.

use std::collections::{HashMap, HashSet};

use cadcraft_doc::Solid3d;
use cadcraft_dxf::Tag;
use cadcraft_geom::Vec3;
use cadcraft_geom::solid::{FEATURE_ANGLE, Mesh};

use crate::dxf_ext;

/// Most vertices, and most faces, in one polyface (16-bit indices).
pub(crate) const MAX_PART: usize = 32_767;

/// Most parts read back into one solid (hostile files).
const MAX_PARTS: usize = 4096;

/// The xdata marker of a solid's polyface.
pub(crate) const MARKER: &str = "SOLID3D";

/// One polyface: its vertices, and its faces as 1-based vertex numbers, negative where the edge
/// that starts at that vertex is hidden.
pub(crate) struct Part {
    pub vertices: Vec<Vec3>,
    pub faces: Vec<[i64; 3]>,
}

/// The polyfaces a solid is written as.
pub(crate) fn parts(s: &Solid3d) -> Vec<Part> {
    let m = s.mesh();
    let shown: HashSet<(u32, u32)> = m.shown_edges(FEATURE_ANGLE, Vec3::ZERO).into_iter().collect();
    let mut out: Vec<Part> = Vec::new();
    let mut part = Part { vertices: Vec::new(), faces: Vec::new() };
    let mut local: HashMap<u32, i64> = HashMap::new();
    for t in &m.triangles {
        let new = t.iter().filter(|i| !local.contains_key(i)).count();
        if part.vertices.len() + new > MAX_PART || part.faces.len() >= MAX_PART {
            out.push(std::mem::replace(&mut part, Part { vertices: Vec::new(), faces: Vec::new() }));
            local.clear();
        }
        let mut face = [0i64; 3];
        for (k, (slot, i)) in face.iter_mut().zip(t).enumerate() {
            let Some(p) = m.vertices.get(*i as usize) else { continue };
            let n = *local.entry(*i).or_insert_with(|| {
                part.vertices.push(*p);
                part.vertices.len() as i64
            });
            let next = t.get((k + 1) % 3).copied().unwrap_or(*i);
            *slot = if shown.contains(&(*i.min(&next), *i.max(&next))) { n } else { -n };
        }
        part.faces.push(face);
    }
    if !part.faces.is_empty() {
        out.push(part);
    }
    out
}

/// The xdata marking part `index` of `count`.
pub(crate) fn xdata(index: usize, count: usize) -> Vec<Tag> {
    vec![Tag::s(1001, dxf_ext::APP), Tag::s(1000, MARKER), Tag::i(1070, index as i64), Tag::i(1070, count as i64)]
}

/// Part number and count of a polyface POLYLINE written for a solid, from its tags.
pub(crate) fn part_of(tags: &[Tag]) -> Option<(usize, usize)> {
    let x = dxf_ext::xdata(tags, dxf_ext::APP);
    let at = x.iter().position(|t| t.code == 1000 && t.str() == MARKER)?;
    let mut nums = x.iter().skip(at + 1).filter(|t| t.code == 1070).filter_map(|t| usize::try_from(t.str().trim().parse::<i64>().ok()?).ok());
    let (index, count) = (nums.next()?, nums.next()?);
    (index < count && count <= MAX_PARTS).then_some((index, count))
}

/// The solid of one part's vertices and face records (indices 71–74; a quad gives two
/// triangles). Faces naming a vertex that doesn't exist are skipped.
pub(crate) fn solid(verts: &[Vec3], faces: &[[i64; 4]]) -> Solid3d {
    let index = |ix: i64| usize::try_from(ix.unsigned_abs()).ok().and_then(|n| n.checked_sub(1)).filter(|n| *n < verts.len());
    let mut triangles = Vec::with_capacity(faces.len());
    for f in faces {
        let ids: Vec<u32> = f.iter().filter(|ix| **ix != 0).filter_map(|ix| index(*ix)).filter_map(|n| u32::try_from(n).ok()).collect();
        match ids.as_slice() {
            [a, b, c] => triangles.push([*a, *b, *c]),
            [a, b, c, d] => triangles.extend([[*a, *b, *c], [*a, *c, *d]]),
            _ => {}
        }
    }
    Solid3d::new(Mesh { vertices: verts.to_vec(), triangles })
}

/// `whole` with the next part's triangles added, its vertices merged with the ones written
/// twice (parts repeat the vertices they share, with the same coordinates).
pub(crate) fn join(whole: &Solid3d, part: &Solid3d) -> Solid3d {
    let (a, b) = (whole.mesh(), part.mesh());
    let key = |p: &Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
    let mut mesh = a.clone();
    let mut seen: HashMap<[u64; 3], u32> = a.vertices.iter().enumerate().filter_map(|(i, p)| Some((key(p), u32::try_from(i).ok()?))).collect();
    let map: Vec<u32> = b
        .vertices
        .iter()
        .map(|p| {
            *seen.entry(key(p)).or_insert_with(|| {
                mesh.vertices.push(*p);
                u32::try_from(mesh.vertices.len() - 1).unwrap_or(u32::MAX)
            })
        })
        .collect();
    mesh.triangles.extend(b.triangles.iter().filter_map(|t| Some([*map.get(t[0] as usize)?, *map.get(t[1] as usize)?, *map.get(t[2] as usize)?])));
    Solid3d::new(mesh)
}
