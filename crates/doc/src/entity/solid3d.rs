//! 3D solids (3DSOLID): closed triangle meshes made by the modeling commands (BOX, EXTRUDE,
//! UNION …), see [`cadcraft_geom::solid`].
//!
//! The mesh is shared, so the copy of a drawing kept for undo doesn't copy it. The wireframe
//! drawn in plan view (the edges where faces meet at an angle, and the outline seen from above)
//! is worked out once, whenever the solid changes.

use std::sync::Arc;

use cadcraft_geom::solid::{FEATURE_ANGLE, MassProps, Mesh};
use cadcraft_geom::{Bounds3, Mat3, Mat4, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(from = "Solid3dData", into = "Solid3dData")]
pub struct Solid3d {
    mesh: Arc<Mesh>,
    wire: Arc<Vec<[Vec3; 2]>>,
}

/// What is saved of a solid: its mesh (the wireframe is derived).
#[derive(Clone, Serialize, Deserialize)]
struct Solid3dData {
    mesh: Mesh,
}

impl From<Solid3dData> for Solid3d {
    fn from(d: Solid3dData) -> Solid3d {
        Solid3d::new(d.mesh)
    }
}

impl From<Solid3d> for Solid3dData {
    fn from(s: Solid3d) -> Solid3dData {
        Solid3dData { mesh: Arc::unwrap_or_clone(s.mesh) }
    }
}

impl Solid3d {
    /// A solid bounded by `mesh` (sanitized: bad triangles are dropped).
    pub fn new(mesh: Mesh) -> Solid3d {
        let mesh = mesh.sanitized();
        let wire = mesh.wire_edges(FEATURE_ANGLE, Vec3::Z);
        Solid3d { mesh: Arc::new(mesh), wire: Arc::new(wire) }
    }

    pub fn mesh(&self) -> &Mesh {
        &self.mesh
    }

    /// The edges drawn in plan view.
    pub fn wire(&self) -> &[[Vec3; 2]] {
        &self.wire
    }

    pub fn bounds(&self) -> Bounds3 {
        self.mesh.bounds()
    }

    pub fn mass_props(&self) -> MassProps {
        self.mesh.mass_props()
    }

    /// Apply a 3D transform.
    pub fn transform(&mut self, m: &Mat4) {
        *self = Solid3d::new(self.mesh.transformed(m));
    }

    /// Apply a 2D transform (MOVE, ROTATE, SCALE, MIRROR in plan): X and Y as `m` maps them, Z
    /// scaled by its scale factor so the solid keeps its proportions.
    pub fn transform_2d(&mut self, m: &Mat3) {
        let mut t = Mat4::from_mat3(m);
        t.m[2][2] = m.scale_factor();
        self.transform(&t);
    }
}
