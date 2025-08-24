use math::Matrix4f32;

use crate::spaces::*;

pub enum Integrator {
    LightPathIntegrator(LightPathIntegrator),
    PathIntegrator(PathIntegrator),
    PathGuidingIntegrator(PathGuidingIntegrator),
}

pub enum PathIntegratorMode {
    PT,
    NEE,
    MIS,
}

pub struct LightPathIntegrator {
    pub max_depth: usize,
}

pub struct PathIntegrator {
    pub max_depth: usize,
    pub mode: PathIntegratorMode,
}

pub struct PathGuidingIntegrator {
    pub max_depth: usize,
    pub spatial_threshold: f32,
    pub directional_threshold: f32,
    pub spatial_filter: Filter,
    pub directional_filter: Filter,
    pub mode: PathIntegratorMode,
}

pub enum Filter {
    Nearest,
    Box,
    Stochastic,
}

pub struct CameraOptions {
    pub fov: f32,
    pub image_size: [usize; 2],
    pub world_to_camera: Matrix4f32<Worldspace, Cameraspace>,
    pub spp: usize,
}

pub struct Mesh {
    pub points: Box<[Point3World]>,
    pub normals: Box<[Vec3World]>,
    pub triangles: Box<[[u32; 3]]>,
    pub emission: RGBf32,
    pub material_index: Option<usize>,
}

pub struct Lambertian {
    pub albedo: RGBf32,
}
pub struct Dielectric {
    pub eta: f32,
}

pub enum Material {
    Lambertian {
        albedo: Spectrum,
    },
    Dielectric {
        eta: Spectrum,
    },
    Conductor {
        roughness: f32,
        eta: Spectrum,
        k: Spectrum,
    },
}
#[derive(Debug, Clone, Copy)]
pub enum Spectrum {
    RGB(RGBf32),
    Constant(f32),
}
