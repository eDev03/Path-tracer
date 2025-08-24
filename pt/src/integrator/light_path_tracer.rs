use crate::bsdfs::TransportMode;
use crate::camera::*;
use crate::geom::*;
use crate::io_bridge;
use crate::sampling::Sampler;
use crate::scene::*;
use crate::spectrum::*;

use rayon::prelude::*;

pub struct LightPathIntegrator {
    pub max_depth: usize,
}

impl LightPathIntegrator {
    pub fn new(opt: &io_bridge::LightPathIntegrator) -> Self {
        Self {
            max_depth: opt.max_depth,
        }
    }
    pub fn render(&self, scene: &Scene, spp: usize, camera: &Camera, film: &mut Film) {
        film.tiles.par_iter().for_each(|tile| {
            let mut sampler = Sampler::default();
            for (_pixel, offset) in tile.iter() {
                let [ix, iy] = [tile.offset()[0] + offset[0], tile.offset()[1] + offset[1]];
                for i in 0..spp {
                    sampler.init_pixel(ix, iy, i);
                    let mut swl = SampledWavelengths::sample_visible(sampler.wavelength_sample());
                    let Some((le_sample, pmf)) = scene.sample_le(
                        &swl,
                        sampler.sample1d(),
                        sampler.sample2d(),
                        sampler.sample2d(),
                    ) else {
                        continue;
                    };

                    if let Some(wi_sample) = camera.sample_wi(le_sample.p()) {
                        let f = wi_sample.importance
                            * le_sample.l
                            * le_sample.n().dot(wi_sample.wc).abs()
                            / (wi_sample.pdf * le_sample.pdf_pos * pmf);
                        if !f.is_black() && !scene.occluded_le_sample(&le_sample, camera.p()) {
                            film.add_splat(wi_sample.p_raster, &f, &swl);
                        }
                    }
                    self.random_walk(
                        scene,
                        le_sample.l * le_sample.n().dot(le_sample.dir).abs()
                            / (le_sample.pdf_pos * le_sample.pdf_dir * pmf),
                        le_sample.spawn_ray(),
                        camera,
                        film,
                        &mut swl,
                        &mut sampler,
                    );
                }
            }
        });
    }
    fn random_walk(
        &self,
        scene: &Scene,
        beta: SampledSpectrum,
        ray: Ray,
        camera: &Camera,
        film: &Film,
        swl: &mut SampledWavelengths,
        sampler: &mut Sampler,
    ) {
        let mut ray = ray;
        let mut beta = beta;

        for _depth in 0..self.max_depth {
            let Some(interaction) = scene.intersect(ray) else {
                break;
            };
            let wo = -ray.d;
            let Some(bsdf) = scene.compute_bsdf(&interaction, swl) else {
                ray = interaction.spawn_ray(ray.d);
                continue;
            };

            if !bsdf.is_specular()
                && let Some(wi_sample) = camera.sample_wi(interaction.p())
            {
                let f = bsdf.eval(
                    interaction.bsdf_eval_ctx(),
                    wo,
                    wi_sample.wc,
                    TransportMode::Importance,
                ) * wi_sample.importance
                    * beta
                    / wi_sample.pdf;
                if !f.is_black() && !scene.occluded_interaction(&interaction, camera.p()) {
                    film.add_splat(wi_sample.p_raster, &f, swl);
                }
            }

            let Some(sample) = bsdf.sample(
                interaction.bsdf_eval_ctx(),
                wo,
                sampler.sample1d(),
                sampler.sample2d(),
                TransportMode::Importance,
            ) else {
                break;
            };
            beta *= sample.f / sample.pdf;
            if beta.is_black() {
                break;
            }

            if sample.terminate_secondary {
                swl.terminate_secondary();
            }
            ray = interaction.spawn_ray(sample.wi);
        }
    }
}
