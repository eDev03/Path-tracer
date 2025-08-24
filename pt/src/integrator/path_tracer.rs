use crate::bsdfs::TransportMode;
use crate::camera::*;
use crate::geom::*;
use crate::io_bridge;
use crate::sampling::Sampler;
use crate::scene::*;
use crate::spectrum::*;

use rayon::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum PathIntegratorMode {
    PT,
    NEE,
    MIS,
}

pub struct PathIntegrator {
    pub max_depth: usize,
    pub mode: PathIntegratorMode,
}

impl PathIntegrator {
    pub fn new(opt: &io_bridge::PathIntegrator) -> Self {
        Self {
            max_depth: opt.max_depth,
            mode: match opt.mode {
                io_bridge::PathIntegratorMode::PT => PathIntegratorMode::PT,
                io_bridge::PathIntegratorMode::NEE => PathIntegratorMode::NEE,
                io_bridge::PathIntegratorMode::MIS => PathIntegratorMode::MIS,
            },
        }
    }
    pub fn render(&self, scene: &Scene, spp: usize, camera: &Camera, film: &mut Film) {
        film.tiles.par_iter_mut().for_each(|tile| {
            let tile_offset = tile.offset();
            let mut sampler = Sampler::default();
            for (pixel, offset) in tile.iter_mut() {
                let [ix, iy] = [tile_offset[0] + offset[0], tile_offset[1] + offset[1]];
                for i in 0..spp {
                    sampler.init_pixel(ix, iy, i);
                    let mut swl = SampledWavelengths::sample_visible(sampler.wavelength_sample());
                    let we_sample = camera.sample_we(ix, iy, sampler.sample2d());

                    let l = self.li(scene, we_sample.ray, &mut swl, &mut sampler);
                    l.0.iter().for_each(|i| assert!(i.is_finite()));
                    if !l.is_black() {
                        pixel.add(&l, &swl);
                    }
                }
            }
        });
    }
    fn li(
        &self,
        scene: &Scene,
        ray: Ray,
        swl: &mut SampledWavelengths,
        sampler: &mut Sampler,
    ) -> SampledSpectrum {
        let mut ray = ray;
        let mut l = SampledSpectrum::ZERO;
        let mut beta = SampledSpectrum::ONE;
        let mut specular_bounce = false;
        let mut prev_interaction = SceneInteraction::default();
        let mut prev_bsdf_pdf = 0.0;
        let mut depth = 0;
        while depth < self.max_depth {
            let Some(interaction) = scene.intersect(ray) else {
                let r = scene.eval_envmap(ray.d, &swl);
                r.0.iter().for_each(|i| assert!(i.is_finite()));
                l += beta * r;
                break;
            };
            let wo = -ray.d;
            let Some(bsdf) = scene.compute_bsdf(&interaction, swl) else {
                ray = interaction.spawn_ray(ray.d);
                specular_bounce = true;
                continue;
            };
            // light from intersected surface
            match self.mode {
                PathIntegratorMode::PT => {
                    if let Some(light) = scene.get_light(&interaction) {
                        l += light.emitted(interaction.p(), ray.o, swl) * beta
                    }
                }
                PathIntegratorMode::NEE => {
                    if (depth == 0 || specular_bounce)
                        && let Some(light) = scene.get_light(&interaction)
                    {
                        l += light.emitted(interaction.p(), ray.o, swl) * beta
                    }
                }
                PathIntegratorMode::MIS => {
                    if let Some(light) = scene.get_light(&interaction) {
                        if depth == 0 || specular_bounce {
                            l += light.emitted(interaction.p(), ray.o, swl) * beta
                        } else {
                            let pb = prev_bsdf_pdf;
                            let pl =
                                light.pdf_li_solid_angle(prev_interaction.p(), interaction.p());
                            let w = (pb * pb) / (pb * pb + pl * pl);
                            l += w * light.emitted(interaction.p(), ray.o, swl) * beta
                        }
                    }
                }
            }

            // direct lighting
            match self.mode {
                PathIntegratorMode::PT => {}
                PathIntegratorMode::NEE => {
                    if !bsdf.is_specular()
                        && let Some((li_sample, pmf)) = scene.sample_li(
                            interaction.p(),
                            swl,
                            sampler.sample1d(),
                            sampler.sample2d(),
                        )
                    {
                        let f = bsdf.eval(
                            interaction.bsdf_eval_ctx(),
                            wo,
                            li_sample.wl,
                            TransportMode::Radiance,
                        ) / (li_sample.pdf * pmf);
                        if !f.is_black() && !scene.occluded_interaction(&interaction, li_sample.p())
                        {
                            l += f * beta * li_sample.l;
                        }
                    }
                }
                PathIntegratorMode::MIS => {
                    if !bsdf.is_specular()
                        && let Some((li_sample, pmf)) = scene.sample_li(
                            interaction.p(),
                            swl,
                            sampler.sample1d(),
                            sampler.sample2d(),
                        )
                    {
                        let f = bsdf.eval(
                            interaction.bsdf_eval_ctx(),
                            wo,
                            li_sample.wl,
                            TransportMode::Radiance,
                        ) / (li_sample.pdf * pmf);
                        if !f.is_black() && !scene.occluded_interaction(&interaction, li_sample.p())
                        {
                            let pl = li_sample.pdf;
                            let pb = bsdf.pdf(interaction.bsdf_eval_ctx(), wo, li_sample.wl);
                            let w = (pl * pl) / (pl * pl + pb * pb);
                            l += w * f * beta * li_sample.l;
                        }
                    }
                }
            }

            let Some(sample) = bsdf.sample(
                interaction.bsdf_eval_ctx(),
                wo,
                sampler.sample1d(),
                sampler.sample2d(),
                TransportMode::Radiance,
            ) else {
                break;
            };
            beta *= sample.f / sample.pdf;
            if beta.is_black() {
                break;
            }
            specular_bounce = sample.specular;
            if sample.terminate_secondary {
                swl.terminate_secondary();
            }
            ray = interaction.spawn_ray(sample.wi);
            prev_interaction = interaction;
            prev_bsdf_pdf = sample.pdf;
            depth += 1;
        }
        l
    }
}
