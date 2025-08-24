use itertools::Itertools;
use math::*;
use sky;

use crate::geom::*;
use crate::sampling;
use crate::sampling::PiecewiseConstant2d;
use crate::sg;
use crate::spaces::*;
use crate::spectrum::*;

pub struct AreaLight<'spectrum> {
    pub p0: Point3World,
    pub p1: Point3World,
    pub p2: Point3World,
    pub onb: ONB3f32<Worldspace, Lightspace>,
    pub emission: &'spectrum Spectrum,
}

pub struct Envmap {
    p2d: PiecewiseConstant2d,
    to_world: Matrix4f32<Lightspace, Worldspace>,
    to_light: Matrix4f32<Worldspace, Lightspace>,
    image: Box<[TristimulusIlluminantSpectrum]>,
    size: [usize; 2],
}

pub struct LiSample {
    pub l: SampledSpectrum,
    pub bounds: Bounds3World,
    pub n: Vec3World,
    pub wl: Vec3World,
    pub pdf: f32,
}
pub struct LeSample {
    pub l: SampledSpectrum,
    pub bounds: Bounds3World,
    pub n: Vec3World,
    pub dir: Vec3World,
    pub pdf_dir: f32,
    pub pdf_pos: f32,
}
impl LiSample {
    pub fn p(&self) -> Point3World {
        self.bounds.centroid()
    }
}
impl LeSample {
    pub fn p(&self) -> Point3World {
        self.bounds.centroid()
    }
    pub fn n(&self) -> Vec3World {
        self.n
    }
    pub fn spawn_ray(&self) -> Ray {
        Ray {
            o: offset_ray_origin(self.bounds, self.n(), self.dir),
            d: self.dir,
        }
    }
}
impl<'spectrum> AreaLight<'spectrum> {
    pub fn new(
        p0: Point3World,
        p1: Point3World,
        p2: Point3World,
        emission: &'spectrum Spectrum,
    ) -> Self {
        Self {
            p0,
            p1,
            p2,
            onb: ONB3f32::init_z(triangle_normal(p0, p1, p2)),
            emission,
        }
    }
    pub fn n(&self) -> Vec3World {
        self.onb.z
    }
    pub fn emitted(
        &self,
        from: Point3World,
        to: Point3World,
        swl: &SampledWavelengths,
    ) -> SampledSpectrum {
        let incoming = to - from;
        if incoming.dot(self.n()) <= 0.0 {
            return SampledSpectrum::ZERO;
        }
        return self.emission.eval(swl.lambdas);
    }
    pub fn sample_le(
        &self,
        swl: &SampledWavelengths,
        u2d0: [f32; 2],
        u2d1: [f32; 2],
    ) -> Option<LeSample> {
        let b = sampling::uniform_barycentric(u2d0);
        let p = triangle_point(self.p0, self.p1, self.p2, b);
        let pdf_pos = 1.0 / triangle_area(self.p0, self.p1, self.p2);
        let dir = Vec3Light::from(sampling::cosine_hemisphere(u2d1));
        let pdf_dir = sampling::cosine_hemisphere_pdf(sg::cos_theta(dir));
        if pdf_dir == 0.0 {
            return None;
        };
        let abs_sum = self.p0.abs().to_vector() * b[0]
            + self.p1.abs().to_vector() * b[1]
            + self.p2.abs().to_vector() * b[2];
        let err = f32::numeric_gamma(6.0) * abs_sum;
        let dir = self.onb.apply_vector_inverse(dir);
        Some(LeSample {
            l: self.emission.eval(swl.lambdas),
            bounds: Bounds3f32 {
                min: p - err,
                max: p + err,
            },
            n: self.n(),
            dir,
            pdf_dir,
            pdf_pos,
        })
    }
    pub fn sample_li_solid_angle(
        &self,
        from_p: Point3World,
        swl: &SampledWavelengths,
        u2d: [f32; 2],
    ) -> Option<LiSample> {
        let b = sampling::uniform_barycentric(u2d);
        let p = triangle_point(self.p0, self.p1, self.p2, b);
        let wl = p - from_p;
        if self.n().dot(wl) >= 0.0 {
            return None;
        }
        let dist2 = wl.mag_sq();
        let wl = wl.normalized();
        let area_pdf = triangle_area(self.p0, self.p1, self.p2).recip();
        let cos_theta = wl.dot(self.n()).abs();
        let solid_angle_pdf = area_pdf * dist2 / cos_theta;
        if solid_angle_pdf == 0.0 {
            return None;
        }
        let abs_sum = self.p0.abs().to_vector() * b[0]
            + self.p1.abs().to_vector() * b[1]
            + self.p2.abs().to_vector() * b[2];
        let err = f32::numeric_gamma(6.0) * abs_sum;
        Some(LiSample {
            l: self.emission.eval(swl.lambdas),
            bounds: Bounds3f32 {
                min: p - err,
                max: p + err,
            },
            n: self.n(),
            wl: wl,
            pdf: solid_angle_pdf,
        })
    }
    pub fn pdf_li_solid_angle(&self, from_p: Point3World, to_p: Point3World) -> f32 {
        let area_pdf = 1.0 / triangle_area(self.p0, self.p1, self.p2);
        let wl = to_p - from_p;
        let dist2 = wl.mag_sq();
        let wl = wl.normalized();
        let cos_theta = wl.dot(self.n()).abs();
        if cos_theta == 0.0 {
            return 0.0;
        }
        return area_pdf * dist2 / cos_theta;
    }
}

impl Envmap {
    pub fn from_sunsky(
        turbidity: f32,
        latitude: f32,
        longitude: f32,
        timezone: f32,
        year: i32,
        month: i32,
        day: i32,
        hour: f32,
        minute: f32,
        second: f32,
        sun_direction: Option<Vec3Object>,
        _sun_scale: f32,
        _sky_scale: f32,
        _sun_aperture: f32,
        to_world: Matrix4f32<Lightspace, Worldspace>,
    ) -> Self {
        let width = 1024 * 2;
        let height = 512 * 2;

        let sun_dir = calculate_sun_dir(
            sky::EarthLocation {
                latitude,
                longitude,
                timezone,
            },
            sky::DateTime {
                year: year as f32,
                month: month as f32,
                day: day as f32,
                hour,
                minute,
                second,
            },
            sun_direction,
        )
        .normalized();
        let mut img = Vec::new();
        let theta_s = acos(sun_dir.y);
        let state = sky::hosek::create_rgb_model(turbidity, 0.3, theta_s, 4.0);
        let image = (0..height)
            .cartesian_product(0..width)
            .map(|(h, w)| {
                let u = w as f32 / width as f32;
                let v = h as f32 / height as f32;
                let view_dir = uv_to_dir([u, v]);
                let theta = acos(view_dir.y);
                let gamma = acos(view_dir.dot(sun_dir));
                let sun_rgb: RGBf32 = sky::hosek::sun_radiance(&state, theta, gamma).into();
                let sky_rgb: RGBf32 = sky::hosek::sky_radiance(&state, theta, gamma).into();
                let sum = (sky_rgb + sun_rgb).array();
                img.push(sum);
                TristimulusIlluminantSpectrum::new(sum.into())
            })
            .collect::<Box<_>>();

        let distribution_values = image
            .iter()
            .map(|s| {
                s.eval(400.0)
                    + s.eval(450.0)
                    + s.eval(500.0)
                    + s.eval(550.0)
                    + s.eval(600.0)
                    + s.eval(650.0)
            })
            .collect::<Box<_>>();

        let data = img
            .iter()
            .flat_map(|rgb| {
                let [r, g, b] = rgb;
                [
                    (r.clamp(0.0, 1.0).powf(1.0 / 2.4) * 255.999) as u8,
                    (g.clamp(0.0, 1.0).powf(1.0 / 2.4) * 255.999) as u8,
                    (b.clamp(0.0, 1.0).powf(1.0 / 2.4) * 255.999) as u8,
                    (255.999) as u8,
                ]
            })
            .collect_vec();
        let mut writer = stb_image_write_rust::ImageWriter::ImageWriter::new("out.png");
        writer.write_png(width as i32, height as i32, 4, data.as_ptr());

        Self {
            p2d: PiecewiseConstant2d::new(&distribution_values, [width, height]),
            image,
            size: [width, height],
            to_world,
            to_light: to_world.inverse(),
        }
    }
    pub fn eval(&self, dir: Vec3World, swl: &SampledWavelengths) -> SampledSpectrum {
        // let d =
        // [-0.8794779, 0.47549742, 0.020514969].into()
        // ;
        // if dir.dot(d) < -0.9 {
        //     return SampledSpectrum::ONE;
        // }
        // return SampledSpectrum::ZERO;
        let dir = self.to_light * dir;
        let [u, v] = dir_to_uv(dir);
        let x = ((u * self.size[0] as f32) as usize).min(self.size[0] - 1);
        let y = ((v * self.size[1] as f32) as usize).min(self.size[1] - 1);
        let i = x + y * self.size[0];
        let l: SampledSpectrum = swl.lambdas.map(|l| self.image[i].eval(l)).into();
        if !l.0.iter().all(|i| i.is_finite()) {
            dbg!(&self.image[i]);
            panic!();
        }
        l
    }
}

fn dir_to_uv(v: Vec3Light) -> [f32; 2] {
    let [x, y, z] = [v.x, v.y, v.z];
    let theta = z.clamp(-1.0, 1.0).acos();
    let mut phi = y.atan2(x);
    if phi < 0.0 {
        phi += TWO_PI
    }
    [INV_TWO_PI * phi, INV_PI * theta]
}

fn uv_to_dir([u, v]: [f32; 2]) -> Vec3Light {
    let phi = u * TWO_PI;
    let theta = v * PI;
    let cos_theta = theta.cos();
    let sin_theta = theta.sin();
    let cos_phi = phi.cos();
    let sin_phi = phi.sin();
    [sin_theta * cos_phi, sin_theta * sin_phi, cos_theta].into()
}

fn calculate_sun_dir(
    location: sky::EarthLocation,
    time: sky::DateTime,
    sun_direction: Option<Vec3Object>,
) -> Vec3Light {
    if let Some(dir) = sun_direction {
        return dir.array().into();
    }
    sky::compute_solar_vector(location, time).array().into()
}

fn perez_params(t: f32) -> [[f32; 5]; 3] {
    [
        [
            0.1787 * t - 1.4630,
            -0.3554 * t + 0.4275,
            -0.0227 * t + 5.3251,
            0.1206 * t - 2.5771,
            -0.0670 * t + 0.3703,
        ],
        [
            -0.0193 * t - 0.2592,
            -0.0665 * t + 0.0008,
            -0.0004 * t + 0.2125,
            -0.0641 * t - 0.8989,
            -0.0033 * t + 0.0452,
        ],
        [
            -0.0167 * t - 0.2608,
            -0.0950 * t + 0.0092,
            -0.0079 * t + 0.2102,
            -0.0441 * t - 1.6537,
            -0.0109 * t + 0.0529,
        ],
    ]
}

fn perez_luminance(theta: f32, gamma: f32, [a, b, c, d, e]: [f32; 5]) -> f32 {
    let cos_theta = cos(theta);
    assert_ne!(cos_theta, 0.0);
    assert!(a.is_finite());
    assert!(b.is_finite());
    assert!(c.is_finite());
    assert!(d.is_finite());
    assert!(e.is_finite());
    assert!(gamma.is_finite());
    assert!(theta.is_finite());
    if !(exp(b / cos_theta).is_finite()) {
        dbg!(b, cos_theta, b / cos_theta);
    }
    let x = 1.0 + a * exp(b / cos_theta);
    assert!(x.is_finite());
    let y = 1.0 + c * exp(d * gamma) + e * cos(gamma) * cos(gamma);
    assert!(y.is_finite());
    x * y
}

#[allow(non_snake_case)]
fn Yxy_coeffs(t: f32, theta: f32) -> [f32; 3] {
    let chi = (4.0 / 9.0 - t / 120.0) * (PI - 2.0 * theta);
    let Yz = (4.0453 * t - 4.9710) * tan(chi) - 0.2155 * t + 2.4192;

    let theta2 = theta * theta;
    let theta3 = theta2 * theta;
    let t2 = t * t;

    let xz = (0.00165 * theta3 - 0.00375 * theta2 + 0.00209 * theta + 0.0) * t2
        + (-0.02903 * theta3 + 0.06377 * theta2 - 0.03202 * theta + 0.00394) * t
        + (0.11693 * theta3 - 0.21196 * theta2 + 0.06052 * theta + 0.25886);

    let yz = (0.00275 * theta3 - 0.00610 * theta2 + 0.00317 * theta + 0.0) * t2
        + (-0.04214 * theta3 + 0.08970 * theta2 - 0.04153 * theta + 0.00516) * t
        + (0.15346 * theta3 - 0.26756 * theta2 + 0.06670 * theta + 0.26688);

    return [Yz, xz, yz];
}

#[allow(non_snake_case)]
fn preetham(
    up: Vec3Light,
    sun_dir: Vec3Light,
    view_dir: Vec3Light,
    turbidity: f32,
    params: [[f32; 5]; 3],
) -> RGBf32 {
    fn angle(a: Vec3Light, b: Vec3Light) -> f32 {
        let dot = a.dot(b);
        let temp = 2.0 * asin(0.5 * (b - a * dot.signum()).mag());
        if dot >= 0.0 { temp } else { PI_OVER_TWO }
    }

    let [Y_params, x_params, y_params] = params;

    let theta_sun = angle(sun_dir, up);
    let theta_view = angle(view_dir, up);
    let gamma = angle(sun_dir, view_dir);
    if gamma < 2f32.to_radians() {
        return [1000.0; 3].into();
    }
    if cos(theta_view).abs() < 0.0000001 {
        return [0.0; 3].into();
    }

    let [Y, x, y] = Yxy_coeffs(turbidity, theta_sun);
    let [Y_luminance, x_luminance, y_luminance] = [Y_params, x_params, y_params].map(|params| {
        let num = perez_luminance(theta_view, gamma, params);
        let denom = perez_luminance(0.0, theta_sun, params);
        assert!(num.is_finite());
        assert_ne!(denom, 0.0);
        num / denom
    });
    assert!(Y_luminance.is_finite());
    assert!(x_luminance.is_finite());
    assert!(y_luminance.is_finite());
    let [x, y, z] = Yxy_to_xyz(Y * Y_luminance, x * x_luminance, y * y_luminance).array();
    assert!(x.is_finite());
    assert!(y.is_finite());
    assert!(z.is_finite());
    let [r, g, b] = xyz_to_rgb([x as f64, y as f64, z as f64].into()).array();
    assert!(r.is_finite());
    assert!(g.is_finite());
    assert!(b.is_finite());
    [r as f32, g as f32, b as f32].into()
}

#[allow(non_snake_case)]
fn Yxy_to_xyz(Y: f32, x: f32, y: f32) -> XYZf32 {
    let X = x * (Y / y);
    let Z = (1.0 - x - y) * (Y / y);
    [X, Y, Z].into()
}
