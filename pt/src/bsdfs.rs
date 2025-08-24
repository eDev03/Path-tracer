use crate::sampling;
use crate::sg;

use crate::spaces::*;
use crate::spectrum::*;

use math::*;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TransportMode {
    Radiance,
    Importance,
}

#[derive(Debug, Clone, Copy)]
pub struct BSDFEvalCtx {
    pub geometric_normal: Vec3World,
    pub shading_frame: ONB3f32<Worldspace, Shadingspace>,
}
impl BSDFEvalCtx {
    fn ns(&self) -> Vec3World {
        self.shading_frame.z
    }
    fn ng(&self) -> Vec3World {
        self.geometric_normal
    }
}

pub enum BSDF {
    Lambertian {
        albedo: SampledSpectrum,
    },
    Dielectric {
        eta: SampledSpectrum,
    },
    Conductor {
        roughness: f32,
        eta: SampledSpectrum,
        k: SampledSpectrum,
    },
}

pub struct BSDFSample<Space> {
    pub f: SampledSpectrum,
    pub wi: Vector3f32<Space>,
    pub pdf: f32,
    pub specular: bool,
    pub terminate_secondary: bool,
}

impl BSDF {
    pub fn is_specular(&self) -> bool {
        match self {
            BSDF::Lambertian { .. } => false,
            BSDF::Dielectric { .. } => true,
            BSDF::Conductor { roughness, .. } => ggx_effectively_smooth(*roughness),
        }
    }
    pub fn specular_transmission(&self) -> Option<f32> {
        match self {
            BSDF::Lambertian { .. } => None,
            BSDF::Dielectric { eta } => Some(eta[0]),
            BSDF::Conductor { .. } => None,
        }
    }
    pub fn sample(
        &self,
        ctx: BSDFEvalCtx,
        wo: Vec3World,
        u1d: f32,
        u2d: [f32; 2],
        mode: TransportMode,
    ) -> Option<BSDFSample<Worldspace>> {
        let local_sample = {
            let wo = ctx.shading_frame.apply_vector(wo);
            match *self {
                BSDF::Lambertian { albedo } => sample_lambertian(albedo, wo, u2d),
                BSDF::Dielectric { eta } => sample_dielectric(eta, wo, u1d, mode),
                BSDF::Conductor { roughness, eta, k } => {
                    sample_conductor(eta, k, roughness, wo, u2d)
                }
            }
        };
        local_sample.map(|sample| {
            let wi = ctx.shading_frame.apply_vector_inverse(sample.wi);
            let mut f = sample.f;
            if mode == TransportMode::Importance {
                f *= correct_shading_normals(wo, wi, ctx.ng(), ctx.ns());
            };
            BSDFSample { f, wi, ..sample }
        })
    }
    pub fn eval(
        &self,
        ctx: BSDFEvalCtx,
        wo: Vec3World,
        wi: Vec3World,
        mode: TransportMode,
    ) -> SampledSpectrum {
        let mut spec = {
            let [wo, wi] = [
                ctx.shading_frame.apply_vector(wo),
                ctx.shading_frame.apply_vector(wi),
            ];

            match *self {
                BSDF::Lambertian { albedo } => eval_lambertian(albedo, wo, wi),
                BSDF::Dielectric { eta } => eval_dielectric(eta, wo, wi, mode),
                BSDF::Conductor { roughness, eta, k } => eval_conducdor(eta, k, roughness, wo, wi),
            }
        };
        if mode == TransportMode::Importance {
            spec *= correct_shading_normals(wo, wi, ctx.ns(), ctx.ng());
        }
        spec
    }
    pub fn pdf(&self, ctx: BSDFEvalCtx, wo: Vec3World, wi: Vec3World) -> f32 {
        let [wo, wi] = [
            ctx.shading_frame.apply_vector(wo),
            ctx.shading_frame.apply_vector(wi),
        ];
        match *self {
            BSDF::Lambertian { .. } => pdf_lambertian(wo, wi),
            BSDF::Dielectric { eta } => pdf_dielectric(eta, wo, wi),
            BSDF::Conductor { roughness, .. } => pdf_conductor(roughness, wo, wi),
        }
    }
}

fn correct_shading_normals(wo: Vec3World, wi: Vec3World, ng: Vec3World, ns: Vec3World) -> f32 {
    let a = wo.dot(ns) * wi.dot(ng);
    let b = wo.dot(ng) * wi.dot(ns);
    if b == 0.0 { 0.0 } else { (a / b).abs() }
}

fn sample_lambertian(
    albedo: SampledSpectrum,
    wo: Vec3Shading,
    u2d: [f32; 2],
) -> Option<BSDFSample<Shadingspace>> {
    let mut wi = sampling::cosine_hemisphere(u2d).into();
    if !sg::same_hemisphere(wo, wi) {
        wi = -wi;
    }
    let pdf = sampling::cosine_hemisphere_pdf(sg::abs_cos_theta(wi));
    if pdf == 0.0 {
        return None;
    }
    Some(BSDFSample {
        f: albedo * sg::abs_cos_theta(wi) * INV_PI,
        wi,
        pdf,
        specular: false,
        terminate_secondary: false,
    })
}
fn eval_lambertian(albedo: SampledSpectrum, wo: Vec3Shading, wi: Vec3Shading) -> SampledSpectrum {
    if !sg::same_hemisphere(wo, wi) {
        return SampledSpectrum::ZERO;
    }
    albedo * sg::abs_cos_theta(wi) * INV_PI
}

fn pdf_lambertian(wo: Vec3Shading, wi: Vec3Shading) -> f32 {
    if sg::same_hemisphere(wo, wi) {
        sampling::cosine_hemisphere_pdf(sg::abs_cos_theta(wi))
    } else {
        0.0
    }
}

fn sample_dielectric(
    eta: SampledSpectrum,
    wo: Vec3Shading,
    u1d: f32,
    mode: TransportMode,
) -> Option<BSDFSample<Shadingspace>> {
    let reflectance = fresnel(sg::cos_theta(wo), eta[0]);
    if u1d < reflectance {
        let wi = Vec3Shading::new(-wo.x, -wo.y, wo.z);
        let f = reflectance;
        let pdf = reflectance;
        Some(BSDFSample {
            f: SampledSpectrum::splat(f),
            wi,
            pdf,
            specular: true,
            terminate_secondary: false,
        })
    } else {
        let (etap, wi) = refract(wo, Vec3Shading::new(0.0, 0.0, 1.0), eta[0]);
        let mut f = 1.0 - reflectance;
        if mode == TransportMode::Radiance {
            f /= etap * etap;
        }
        let pdf = 1.0 - reflectance;
        Some(BSDFSample {
            f: SampledSpectrum::splat(f),
            wi,
            pdf,
            specular: true,
            terminate_secondary: !eta.is_uniform(),
        })
    }
}

fn eval_dielectric(
    _eta: SampledSpectrum,
    _wo: Vec3Shading,
    _wi: Vec3Shading,
    _mode: TransportMode,
) -> SampledSpectrum {
    SampledSpectrum::ZERO
}

fn pdf_dielectric(_eta: SampledSpectrum, _wo: Vec3Shading, _wi: Vec3Shading) -> f32 {
    0.0
}

fn sample_conductor(
    eta: SampledSpectrum,
    k: SampledSpectrum,
    a: f32,
    wo: Vec3Shading,
    u2d: [f32; 2],
) -> Option<BSDFSample<Shadingspace>> {
    if ggx_effectively_smooth(a) {
        let wi = Vec3Shading::new(-wo.x, -wo.y, wo.z);
        let f = complex_fresnel_spectrum(sg::cos_theta(wi), eta, k);
        let pdf = 1.0;
        return Some(BSDFSample {
            f,
            wi,
            pdf,
            specular: true,
            terminate_secondary: false,
        });
    }
    let wm = ggx_sample_wm(a, wo, u2d);
    let wi = reflect(wo, wm);
    if !sg::same_hemisphere(wo, wi) {
        return None;
    }
    let pdf = ggx_pdf(a, wo, wm) / (4.0 * wo.dot(wm).abs());
    let cos_theta_o = sg::abs_cos_theta(wo);
    let cos_theta_i = sg::abs_cos_theta(wi);
    let f = sg::abs_cos_theta(wi)
        * complex_fresnel_spectrum(wo.dot(wm).abs(), eta, k)
        * ggx_dwm(a, wm)
        * ggx_g(a, wo, wi)
        / (4.0 * cos_theta_o * cos_theta_i);
    Some(BSDFSample {
        f,
        wi,
        pdf,
        specular: false,
        terminate_secondary: false,
    })
}

fn eval_conducdor(
    eta: SampledSpectrum,
    k: SampledSpectrum,
    a: f32,
    wo: Vec3Shading,
    wi: Vec3Shading,
) -> SampledSpectrum {
    if ggx_effectively_smooth(a) || !sg::same_hemisphere(wo, wi) || (wo + wi).mag_sq() == 0.0 {
        return SampledSpectrum::ZERO;
    }
    let wm = (wo + wi).normalized();
    let cos_theta_o = sg::abs_cos_theta(wo);
    let cos_theta_i = sg::abs_cos_theta(wi);
    sg::abs_cos_theta(wi)
        * complex_fresnel_spectrum(wo.dot(wm).abs(), eta, k)
        * ggx_dwm(a, wm)
        * ggx_g(a, wo, wi)
        / (4.0 * cos_theta_o * cos_theta_i)
}

fn pdf_conductor(a: f32, wo: Vec3Shading, wi: Vec3Shading) -> f32 {
    if ggx_effectively_smooth(a) || !sg::same_hemisphere(wo, wi) || (wo + wi).mag_sq() == 0.0 {
        return 0.0;
    }
    let wm = (wo + wi).normalized();
    ggx_pdf(a, wo, wm) / (4.0 * wo.dot(wm).abs())
}

fn reflect(w: Vec3Shading, n: Vec3Shading) -> Vec3Shading {
    return -w + 2.0 * w.dot(n) * n;
}

fn refract<Space: Copy>(
    w: Vector3f32<Space>,
    mut n: Vector3f32<Space>,
    mut eta: f32,
) -> (f32, Vector3f32<Space>) {
    let mut cos_theta_i = w.dot(n);
    if cos_theta_i < 0.0 {
        eta = eta.recip();
        cos_theta_i = -cos_theta_i;
        n = -n;
    }
    let sin2theta_i = 0f32.max(1.0 - cos_theta_i * cos_theta_i);
    let sin2theta_t = sin2theta_i / (eta * eta);
    if sin2theta_t < 1.0 {
        let cos_theta_t = (1.0 - sin2theta_t).safe_sqrt();
        let wt = -w / eta + (cos_theta_i / eta - cos_theta_t) * n;
        (eta, wt)
    } else {
        assert!(false, "{:}", sin2theta_t);
        unreachable!();
    }
}

pub fn fresnel(mut cos_theta_i: f32, mut eta: f32) -> f32 {
    cos_theta_i = cos_theta_i.clamp(-1.0, 1.0);
    if cos_theta_i < 0.0 {
        eta = eta.recip();
        cos_theta_i = -cos_theta_i;
    }
    let sin2theta_i = 1.0 - cos_theta_i * cos_theta_i;
    let sin2theta_t = sin2theta_i / (eta * eta);
    if sin2theta_t >= 1.0 {
        return 1.0;
    }
    let cos_theta_t = (1.0 - sin2theta_t).safe_sqrt();
    let r_parl = (eta * cos_theta_i - cos_theta_t) / (eta * cos_theta_i + cos_theta_t);
    let r_perp = (cos_theta_i - eta * cos_theta_t) / (cos_theta_i + eta * cos_theta_t);
    return (r_parl * r_parl + r_perp * r_perp) / 2.0;
}

fn complex_fresnel(mut cos_theta_i: f32, eta: num::Complex<f32>) -> f32 {
    cos_theta_i = cos_theta_i.clamp(0.0, 1.0);
    let sin2theta_i = 1f32 - cos_theta_i * cos_theta_i;
    let sin2theta_t = num::Complex::new(sin2theta_i, 0.0) / (eta * eta);
    let cos_theta_t = (num::Complex::new(1.0, 0.0) - sin2theta_t).sqrt();
    let r_parl = (eta * num::Complex::new(cos_theta_i, 0.0) - cos_theta_t)
        / (eta * num::Complex::new(cos_theta_i, 0.0) + cos_theta_t);
    let r_perp = (num::Complex::new(cos_theta_i, 0.0) - eta * cos_theta_t)
        / (num::Complex::new(cos_theta_i, 0.0) + eta * cos_theta_t);
    return (r_parl.norm_sqr() + r_perp.norm_sqr()) / 2.0;
}
fn complex_fresnel_spectrum(
    cos_theta_i: f32,
    eta: SampledSpectrum,
    k: SampledSpectrum,
) -> SampledSpectrum {
    let mut res = SampledSpectrum::ZERO;
    for i in 0..N_SAMPLED_WAVELENGTHS {
        res[i] = complex_fresnel(cos_theta_i, num::Complex::new(eta[i], k[i]));
    }
    res
}

fn ggx_effectively_smooth(a: f32) -> bool {
    a < 0.0001
}
fn ggx_sample_wm(a: f32, wo: Vec3Shading, u2d: [f32; 2]) -> Vec3Shading {
    let wi = if wo.z < 0f32 { -wo } else { wo };
    let wi_std = Vec3Shading::new(wi.x * a, wi.y * a, wi.z);
    let phi = (2.0 * u2d[0] - 1.0) * PI;
    let z = (1.0 - u2d[1]).mul_add(1.0 + wi_std.z, -wi_std.z);
    let sin_theta = (1.0 - z * z).safe_sqrt();
    let x = sin_theta * phi.cos();
    let y = sin_theta * phi.sin();
    let wm_std = wi_std + Vec3Shading::new(x, y, z);
    Vec3Shading::new(wm_std.x * a, wm_std.y * a, wm_std.z).normalized()
}
fn ggx_dwm(a: f32, wm: Vec3Shading) -> f32 {
    let tan2theta = sg::tan2theta(wm);
    if tan2theta.is_infinite() {
        return 0.0;
    }
    let cos4theta = sg::cos2theta(wm).powi(2);
    let e = tan2theta * ((sg::cos_phi(wm) / a).powi(2) + (sg::sin_phi(wm) / a).powi(2));
    return 1.0 / (PI * a * a * cos4theta * (1.0 + e).powi(2));
}
fn ggx_dwwm(a: f32, w: Vec3Shading, wm: Vec3Shading) -> f32 {
    return ggx_g1(a, w) / sg::abs_cos_theta(w) * ggx_dwm(a, wm) * w.dot(wm).abs();
}
fn ggx_pdf(a: f32, w: Vec3Shading, wm: Vec3Shading) -> f32 {
    return ggx_dwwm(a, w, wm);
}
fn ggx_g(a: f32, wo: Vec3Shading, wi: Vec3Shading) -> f32 {
    return 1.0 / (1.0 + ggx_lambda(a, wo) + ggx_lambda(a, wi));
}
fn ggx_g1(a: f32, w: Vec3Shading) -> f32 {
    return 1.0 / (1.0 + ggx_lambda(a, w));
}
fn ggx_lambda(a: f32, w: Vec3Shading) -> f32 {
    let tan2theta = sg::tan2theta(w);
    if tan2theta.is_infinite() {
        return 0.0;
    }
    return ((1.0 + a * a * tan2theta).sqrt() - 1.0) / 2.0;
}
