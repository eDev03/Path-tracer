use crate::bsdfs::BSDF;
use crate::bsdfs::BSDFEvalCtx;
use crate::bsdfs::BSDFSample;
use crate::bsdfs::TransportMode;
use crate::camera::*;
use crate::geom::*;
use crate::io_bridge;
use crate::sampling::Sampler;
use crate::scene::*;
use crate::spaces::*;
use crate::spectrum::*;

use math::*;
use rayon::prelude::*;

mod dtree;
mod stree;
use dtree::*;
use stree::*;

#[derive(Clone, Copy)]
pub enum Filter {
    Nearest,
    Box,
    Stochastic,
}

#[derive(Clone, Copy)]
pub enum Mode {
    PT,
    NEE,
    MIS,
}

impl From<&io_bridge::Filter> for Filter {
    fn from(value: &io_bridge::Filter) -> Self {
        match value {
            io_bridge::Filter::Nearest => Self::Nearest,
            io_bridge::Filter::Box => Self::Box,
            io_bridge::Filter::Stochastic => Self::Stochastic,
        }
    }
}

pub struct PathGuidingIntegrator {
    pub max_depth: usize,
    pub spatial_threshold: f32,
    pub directional_threshold: f32,
    pub spatial_filter: Filter,
    pub directional_filter: Filter,
    pub mode: Mode,
}

impl PathGuidingIntegrator {
    pub fn new(opt: &io_bridge::PathGuidingIntegrator) -> Self {
        Self {
            max_depth: opt.max_depth,
            spatial_threshold: opt.spatial_threshold,
            directional_threshold: opt.directional_threshold,
            spatial_filter: (&opt.spatial_filter).into(),
            directional_filter: (&opt.directional_filter).into(),
            mode: match opt.mode {
                io_bridge::PathIntegratorMode::PT => Mode::PT,
                io_bridge::PathIntegratorMode::NEE => Mode::NEE,
                io_bridge::PathIntegratorMode::MIS => Mode::MIS,
            },
        }
    }

    pub fn render(&self, scene: &Scene, spp: usize, camera: &Camera, film: &mut Film) {
        let mut guider = Guider::new(
            scene.bounds(),
            0.01,
            self.spatial_threshold,
            self.spatial_filter,
            self.directional_filter,
        );
        let mut wave_index = 0;
        for wave_size in wave_sample_counts(spp) {
            film.clear();
            guider.new_iteration();
            dbg!(guider.iteration, wave_size);
            for _ in 0..wave_size {
                let i = wave_index;
                wave_index += 1;
                film.tiles.par_iter_mut().for_each(|tile| {
                    let mut path = vec![];
                    let tile_offset = tile.offset();
                    let mut sampler = Sampler::default();
                    for (pixel, offset) in tile.iter_mut() {
                        let [ix, iy] = [tile_offset[0] + offset[0], tile_offset[1] + offset[1]];
                        let mut swl =
                            SampledWavelengths::sample_visible(sampler.wavelength_sample());
                        sampler.init_pixel(ix, iy, i);
                        let camera_sample = camera.sample_we(ix, iy, sampler.sample2d());

                        path.clear();
                        self.random_walk(
                            scene,
                            camera_sample.ray,
                            &mut swl,
                            &mut path,
                            &guider,
                            &mut sampler,
                        );
                        let mut l = SampledSpectrum::ZERO;
                        for v in path.iter().rev() {
                            if !v.is_specular {
                                let r = l.average() / v.wi_pdf;
                                guider.add(r, v.p, v.wi, &mut sampler);
                            }
                            l *= v.f;
                            l += v.li_sample_contribution;
                            l += v.emitted;
                        }
                        pixel.add(&l, &swl);
                    }
                });
            }
        }
        dbg!(guider.stree.len());
    }

    fn random_walk(
        &self,
        scene: &Scene,
        ray: Ray,
        swl: &mut SampledWavelengths,
        path: &mut Vec<PathVertex>,
        guider: &Guider,
        sampler: &mut Sampler,
    ) {
        let mut ray = ray;
        let mut beta = SampledSpectrum::ONE;
        let mut depth = 0;
        let mut specular_bounce = false;
        let mut prev_interaction = SceneInteraction::default();
        let mut prev_bsdf_pdf = 0.0;
        while depth < self.max_depth {
            let Some(interaction) = scene.intersect(ray) else {
                path.push(PathVertex {
                    p: ray.o + ray.d * 10000.0,
                    wi: ray.d,
                    wi_pdf: 1.0,
                    f: SampledSpectrum::ONE,
                    emitted: scene.eval_envmap(ray.d, swl),
                    is_specular: false,
                    li_sample_contribution: SampledSpectrum::ZERO,
                });
                break;
            };
            let wo = -ray.d;
            let Some(bsdf) = scene.compute_bsdf(&interaction, swl) else {
                ray = interaction.spawn_ray(ray.d);
                continue;
            };

            let emitted = match self.mode {
                Mode::PT => {
                    if let Some(light) = scene.get_light(&interaction) {
                        light.emitted(interaction.p(), ray.o, swl)
                    } else {
                        SampledSpectrum::ZERO
                    }
                }
                Mode::NEE => {
                    if (depth == 0 || specular_bounce)
                        && let Some(light) = scene.get_light(&interaction)
                    {
                        light.emitted(interaction.p(), ray.o, swl)
                    } else {
                        SampledSpectrum::ZERO
                    }
                }
                Mode::MIS => {
                    if let Some(light) = scene.get_light(&interaction) {
                        if depth == 0 || specular_bounce {
                            light.emitted(interaction.p(), ray.o, swl)
                        } else {
                            let pb = prev_bsdf_pdf;
                            let pl =
                                light.pdf_li_solid_angle(prev_interaction.p(), interaction.p());
                            let w = (pb * pb) / (pb * pb + pl * pl);
                            w * light.emitted(interaction.p(), ray.o, swl)
                        }
                    } else {
                        SampledSpectrum::ZERO
                    }
                }
            };

            let li_sample_contribution = match self.mode {
                Mode::PT => SampledSpectrum::ZERO,
                Mode::NEE => {
                    let mut contribution = SampledSpectrum::ZERO;
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
                            contribution = f * li_sample.l;
                        }
                    }
                    contribution
                }
                Mode::MIS => {
                    let mut contribution = SampledSpectrum::ZERO;
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
                            contribution = w * f * li_sample.l;
                        }
                    }
                    contribution
                }
            };

            let Some(sample) = guider.sample(
                &bsdf,
                interaction.bsdf_eval_ctx(),
                interaction.p(),
                wo,
                sampler,
                TransportMode::Radiance,
            ) else {
                break;
            };
            specular_bounce = bsdf.is_specular();
            if !bsdf.is_specular() {
                path.push(PathVertex {
                    p: interaction.p(),
                    wi_pdf: sample.pdf,
                    wi: sample.wi,
                    f: sample.f / sample.pdf,
                    is_specular: bsdf.is_specular(),
                    emitted,
                    li_sample_contribution,
                });
            }
            beta *= sample.f / sample.pdf;
            if beta.is_black() {
                break;
            }
            if sample.terminate_secondary {
                swl.terminate_secondary();
            }
            ray = interaction.spawn_ray(sample.wi);
            prev_interaction = interaction;
            prev_bsdf_pdf = sample.pdf;
            depth += 1;
        }
    }
}
struct PathVertex {
    p: Point3World,
    wi: Vec3World,
    wi_pdf: f32,
    f: SampledSpectrum,
    emitted: SampledSpectrum,
    is_specular: bool,
    li_sample_contribution: SampledSpectrum,
}

struct Guider {
    stree: STree,
    directional_subdivision_threshold: f32,
    spatial_subdivision_threshold: f32,
    iteration: usize,
    has_started: bool,
}

impl Guider {
    fn new(
        bounds: Bounds3f32<Worldspace>,
        directional_subdivision_threshold: f32,
        spatial_subdivision_threshold: f32,
        spatial_filter: Filter,
        directional_filter: Filter,
    ) -> Self {
        Self {
            stree: STree::new(bounds, spatial_filter, directional_filter),
            directional_subdivision_threshold,
            spatial_subdivision_threshold,
            iteration: 0,
            has_started: false,
        }
    }
    fn sample(
        &self,
        bsdf: &BSDF,
        bsdf_eval_ctx: BSDFEvalCtx,
        p: Point3World,
        wo: Vec3World,
        sampler: &mut Sampler,
        mode: TransportMode,
    ) -> Option<BSDFSample<Worldspace>> {
        if self.iteration == 0 || bsdf.is_specular() {
            return bsdf.sample(
                bsdf_eval_ctx,
                wo,
                sampler.sample1d(),
                sampler.sample2d(),
                mode,
            );
        }
        let p_guided = 0.5;
        let p_bsdf = 1.0 - p_guided;
        let dtree = self.stree.sampler(p);
        if sampler.sample1d() < p_guided {
            let p = dtree.sample(sampler);
            let wi = square_to_dir(p);
            let bsdf_pdf = bsdf.pdf(bsdf_eval_ctx, wo, wi) * p_bsdf;
            // don't want to generate samples that bsdf-sampling cant
            // this value is opaque to the light transport
            // can only be prevented here
            if bsdf_pdf == 0.0 {
                return None;
            }
            let pdf = bsdf_pdf + dtree.pdf(p) * p_guided;
            let f = bsdf.eval(bsdf_eval_ctx, wo, wi, mode);
            Some(BSDFSample {
                f,
                wi,
                pdf,
                specular: false,
                terminate_secondary: false,
            })
        } else {
            bsdf.sample(
                bsdf_eval_ctx,
                wo,
                sampler.sample1d(),
                sampler.sample2d(),
                mode,
            )
            .map(|mut sample| {
                sample.pdf = p_bsdf * sample.pdf + p_guided * dtree.pdf(dir_to_square(sample.wi));
                sample
            })
        }
    }
    fn add(&self, radiance: f32, p: Point3World, wi: Vec3World, sampler: &mut Sampler) {
        self.stree.record(p, wi, radiance, sampler);
    }
    fn new_iteration(&mut self) {
        if self.has_started {
            self.stree.new_iteration(
                self.directional_subdivision_threshold,
                self.spatial_subdivision_threshold,
                self.iteration,
            );
            self.iteration += 1;
        } else {
            self.has_started = true
        }
    }
}

pub fn wave_sample_counts(total_samples: usize) -> Vec<usize> {
    let mut currently_added = 0;
    let mut wave_size = 1;
    let mut wave_counts = vec![];
    loop {
        wave_counts.push(wave_size);
        currently_added += wave_size;
        let remaining = total_samples - currently_added;
        wave_size *= 2;
        if remaining < wave_size {
            let last_index = wave_counts.len() - 1;
            wave_counts[last_index] += remaining;
            break;
        }
    }
    wave_counts
}

fn square_to_dir(p: Point2Directional) -> Vec3World {
    let cos_theta = 2.0 * p.x - 1.0;
    let phi = TWO_PI * p.y;
    let sin_theta = (1.0 - cos_theta * cos_theta).safe_sqrt();
    let (sin_phi, cos_phi) = phi.sin_cos();
    return Vec3World::new(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
}

fn dir_to_square(dir: Vec3World) -> Point2Directional {
    assert!(dir.array_ref().iter().all(|v| v.is_finite()));
    let cos_theta = dir.z.clamp(-1.0, 1.0);
    let mut phi = dir.y.atan2(dir.x);
    while phi < 0.0 {
        phi += TWO_PI;
    }
    return Point2Directional::new((cos_theta + 1.0) / 2.0, phi * INV_TWO_PI)
        .clamp(Point2f32::splat(0.0), Point2f32::splat(1.0));
}
