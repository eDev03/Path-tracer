pub mod light_path_tracer;
pub mod path_guiding;
pub mod path_tracer;
pub use light_path_tracer::*;
pub use path_guiding::*;
pub use path_tracer::*;

use crate::{
    camera::{Camera, Film},
    io_bridge,
    scene::Scene,
};

pub enum Integrator {
    LightPathIntegrator(LightPathIntegrator),
    PathIntegrator(PathIntegrator),
    PathGuidingIntegrator(PathGuidingIntegrator),
}

impl Integrator {
    pub fn new(integrator: &io_bridge::Integrator) -> Self {
        match integrator {
            io_bridge::Integrator::LightPathIntegrator(light_path_integrator) => {
                Self::LightPathIntegrator(LightPathIntegrator::new(light_path_integrator))
            }
            io_bridge::Integrator::PathIntegrator(path_integrator) => {
                Self::PathIntegrator(PathIntegrator::new(path_integrator))
            }
            io_bridge::Integrator::PathGuidingIntegrator(path_guiding_integrator) => {
                Self::PathGuidingIntegrator(PathGuidingIntegrator::new(path_guiding_integrator))
            }
        }
    }
    pub fn render(&mut self, scene: &Scene, spp: usize, camera: &Camera, film: &mut Film) {
        match self {
            Integrator::LightPathIntegrator(light_path_integrator) => {
                light_path_integrator.render(scene, spp, camera, film)
            }
            Integrator::PathIntegrator(path_integrator) => {
                path_integrator.render(scene, spp, camera, film)
            }
            Integrator::PathGuidingIntegrator(bdptintegrator) => {
                bdptintegrator.render(scene, spp, camera, film)
            }
        }
    }
    pub fn film_scale(&self, spp: usize) -> f64 {
        match self {
            Integrator::LightPathIntegrator(_light_path_integrator) => 1.0 / spp as f64,
            Integrator::PathIntegrator(_path_integrator) => 1.0 / spp as f64,
            Integrator::PathGuidingIntegrator(_path_guiding_integrator) => {
                1.0 / *wave_sample_counts(spp).last().unwrap() as f64
            }
        }
    }
}
