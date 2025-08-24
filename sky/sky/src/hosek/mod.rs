pub mod data_rgb;
pub mod data_sun;
use data_rgb::*;
use data_sun::*;
use math::*;

use crate::{xyz_to_rgb, xyz_x, xyz_y, xyz_z};

const TERRESTRIAL_SOLAR_ANGULAR_RADIUS: f32 = (0.51f32.to_radians()) / 2.0;
const LAMBDA_MIN: usize = 320;
const LAMBDA_MAX: usize = 720;
const LAMBDA_STEP: usize = 40;
const CIE_Y_INTEGRAL: f32 = 107.49157;

pub fn tonemap(c: f32) -> f32 {
    let exposure = 0.01;
    (2.0) / ((1.0) + exp(-exposure * c)) - (1.0)
}

pub type Config = [f32; 9];
pub struct HosekModel {
    configs: [Config; 3],
    radiances: [f32; 3],
    turbidity: f32,
    sun_angular_radius: f32,
}

pub fn create_config(dataset: &[f32], turbidity: f32, albedo: f32, solar_elevation: f32) -> Config {
    let int_turbidity = turbidity as usize;
    assert!(int_turbidity > 0);
    let turbidity_rem = turbidity - turbidity as f32;
    let solar_elevation = (solar_elevation * 2.0 * INV_PI).powf(1.0 / 3.0);
    let elev_matrix = &dataset[9 * 6 * (int_turbidity - 1)..];
    let mut config = Config::default();
    for i in 0..9 {
        config[i] +=
            (1.0 - albedo) * (1.0 - turbidity_rem) * spline(solar_elevation, elev_matrix, i, 9);
    }
    let elev_matrix = &dataset[9 * 6 * 10 + 9 * 6 * (int_turbidity - 1)..];
    for i in 0..9 {
        config[i] += albedo * (1.0 - turbidity_rem) * spline(solar_elevation, elev_matrix, i, 9);
    }
    if int_turbidity == 10 {
        return config;
    }
    let elev_matrix = &dataset[9 * 6 * int_turbidity..];
    for i in 0..9 {
        config[i] += (1.0 - albedo) * turbidity_rem * spline(solar_elevation, elev_matrix, i, 9);
    }
    let elev_matrix = &dataset[9 * 6 * 10 + 9 * 6 * int_turbidity..];
    for i in 0..9 {
        config[i] += albedo * turbidity_rem * spline(solar_elevation, elev_matrix, i, 9);
    }
    config
}

pub fn create_rad_config(
    dataset: &[f32],
    turbidity: f32,
    albedo: f32,
    solar_elevation: f32,
) -> f32 {
    let int_turbidity = turbidity as usize;
    let turbidity_rem = turbidity - int_turbidity as f32;
    let solar_elevation = (solar_elevation * 2.0 * INV_PI).powf(1.0 / 3.0);

    let elev_matrix = &dataset[6 * (int_turbidity - 1)..];
    let mut res =
        (1.0 - albedo) * (1.0 - turbidity_rem) * spline(solar_elevation, elev_matrix, 0, 1);
    let elev_matrix = &dataset[6 * 10 + 6 * (int_turbidity - 1)..];
    res += albedo * (1.0 - turbidity_rem) * spline(solar_elevation, elev_matrix, 0, 1);
    if int_turbidity == 10 {
        return res;
    }
    let elev_matrix = &dataset[6 * int_turbidity..];
    res += (1.0 - albedo) * turbidity_rem * spline(solar_elevation, elev_matrix, 0, 1);
    let elev_matrix = &dataset[6 * 10 + 6 * int_turbidity..];
    res += albedo * turbidity_rem * spline(solar_elevation, elev_matrix, 0, 1);
    res
}

pub fn create_rgb_model(
    turbidity: f32,
    albedo: f32,
    elevation: f32,
    sun_radius_scale: f32,
) -> HosekModel {
    let mut state = HosekModel {
        configs: Default::default(),
        radiances: Default::default(),
        turbidity,
        sun_angular_radius: TERRESTRIAL_SOLAR_ANGULAR_RADIUS * sun_radius_scale,
    };
    for channel in 0..3 {
        state.configs[channel] =
            create_config(&DATASETS_RGB[channel], turbidity, albedo, elevation);
        state.radiances[channel] =
            create_rad_config(&DATASETS_RGBRAD[channel], turbidity, albedo, elevation);
    }
    state
}

pub fn sky_radiance(state: &HosekModel, theta: f32, gamma: f32) -> [f32; 3] {
    let rgb = [0, 1, 2].map(|channel| {
        get_radiance(&state.configs[channel], theta, gamma) * state.radiances[channel]
    });
    if rgb.iter().all(|i| i.is_finite()) {
        [
            rgb[0] / CIE_Y_INTEGRAL,
            rgb[1] / CIE_Y_INTEGRAL,
            rgb[2] / CIE_Y_INTEGRAL,
        ]
    } else {
        [0.0; 3]
    }
}

// const BB_SCALE: f32 = 3.19992 * 10E-11;

// fn blackbody(temperature: f32, lambda: f32) -> f32 {
//     let c1 = 3.74177 * 10E-17;
//     let c2 = 0.0143878;

//     (c1 / (lambda.powi(5))) * (1.0 / (exp(c2 / (lambda * temperature)) - 1.0))
// }

// const SOLAR_RADIANCE_SCALE: [f32; 11] = [
//     7500.0, 12500.0, 21127.5, 26760.5, 30663.7, 27825.0, 25503.8, 25134.2, 23212.1, 21526.7,
//     19870.8,
// ];

fn solar_radiance_internal2(
    state: &HosekModel,
    wavelength: f32,
    elevation: f32,
    gamma: f32,
) -> f32 {
    let mut turb_low = state.turbidity as usize;
    let mut turb_frac = state.turbidity - (turb_low + 1) as f32;
    if turb_low == 9 {
        turb_low = 8;
        turb_frac = 1.0;
    }
    let mut wl_low = ((wavelength - 320.0) / 40.0) as usize;
    let mut wl_frac = (wavelength % 40.0) / 40.0;
    if wl_low == 10 {
        wl_low = 9;
        wl_frac = 1.0;
    }
    let direct_radiance = (1.0 - turb_frac)
        * ((1.0 - wl_frac) * arhosekskymodel_sr_internal(state, turb_low, wl_low, elevation)
            + wl_frac * arhosekskymodel_sr_internal(state, turb_low, wl_low + 1, elevation))
        + turb_frac
            * ((1.0 - wl_frac)
                * arhosekskymodel_sr_internal(state, turb_low + 1, wl_low, elevation)
                + wl_frac
                    * arhosekskymodel_sr_internal(state, turb_low + 1, wl_low + 1, elevation));
    let mut ld_coefficient = [0.0; 6];

    for i in 0..6 {
        ld_coefficient[i] = (1.0 - wl_frac) * limbDarkeningDatasets[wl_low][i]
            + wl_frac * limbDarkeningDatasets[wl_low + 1][i];
    }

    let sol_rad_sin = sin(state.sun_angular_radius);
    let ar2 = 1.0 / (sol_rad_sin * sol_rad_sin);
    let singamma = sin(gamma);
    let mut sc2 = 1.0 - ar2 * singamma * singamma;
    if sc2 < 0.0 {
        sc2 = 0.0;
    }
    let sample_cosine = sc2.sqrt();

    //   The following will be improved in future versions of the model:
    //   here, we directly use fitted 5th order polynomials provided by the
    //   astronomical community for the limb darkening effect. Astronomers need
    //   such accurate fittings for their predictions. However, this sort of
    //   accuracy is not really needed for CG purposes, so an approximated
    //   dataset based on quadratic polynomials will be provided in a future
    //   release.

    let darkening_factor = ld_coefficient[0]
        + ld_coefficient[1] * sample_cosine
        + ld_coefficient[2] * sample_cosine.powf(2.0)
        + ld_coefficient[3] * sample_cosine.powf(3.0)
        + ld_coefficient[4] * sample_cosine.powf(4.0)
        + ld_coefficient[5] * sample_cosine.powf(5.0);

    return direct_radiance * darkening_factor;
}

fn arhosekskymodel_sr_internal(
    _state: &HosekModel,
    turbidity: usize,
    wl: usize,
    elevation: f32,
) -> f32 {
    let pieces = 45;
    let order = 4;
    let mut pos = ((2.0 * elevation / PI).powf(1.0 / 3.0) * pieces as f32) as usize; // floor

    if pos > 44 {
        pos = 44
    };

    let break_x = (pos as f32 / pieces as f32).powf(3.0) * (PI * 0.5);

    let mut coefs = order * pieces * turbidity + order * (pos + 1) - 1;

    let mut res = 0.0;
    let x = elevation - break_x;
    let mut x_exp = 1.0;

    for _ in 0..order {
        res += x_exp * solarDatasets[wl][coefs];
        coefs -= 1;
        x_exp *= x;
    }

    return res;
}

fn get_radiance(config: &[f32], theta: f32, gamma: f32) -> f32 {
    let cos_theta = cos(theta).max(0.0);
    if cos_theta <= 0.0 {
        return 0.0;
    }
    let exp_ = (config[4] * gamma).exp();
    let ray_ = cos(gamma).powi(2);
    let mie_ = (1.0 + cos(gamma) * cos(gamma))
        / (1.0 + config[8] * config[8] - 2.0 * config[8] * cos(gamma)).powf(1.5);
    let zenith = cos_theta.sqrt();
    (1.0 + config[0] * exp(config[1] / (cos_theta + 0.01)))
        * (config[2] + config[3] * exp_ + config[5] * ray_ + config[6] * mie_ + config[7] * zenith)
}

fn spline(solar_elevation: f32, elev_matrix: &[f32], offset: usize, stride: usize) -> f32 {
    (1.0 - solar_elevation).powi(5) * elev_matrix[offset]
        + 5.0 * (1.0 - solar_elevation).powi(4) * solar_elevation * elev_matrix[offset + stride * 1]
        + 10.0
            * (1.0 - solar_elevation).powi(3)
            * solar_elevation.powi(2)
            * elev_matrix[offset + stride * 2]
        + 10.0
            * (1.0 - solar_elevation).powi(2)
            * solar_elevation.powi(3)
            * elev_matrix[offset + stride * 3]
        + 5.0 * (1.0 - solar_elevation) * solar_elevation.powi(4) * elev_matrix[offset + stride * 4]
        + solar_elevation.powi(5) * elev_matrix[offset + stride * 5]
}

pub fn sun_radiance(state: &HosekModel, theta: f32, gamma: f32) -> [f32; 3] {
    if gamma > state.sun_angular_radius {
        return [0.0; 3];
    }
    let elevation = PI_OVER_TWO - theta;
    let mut xyz = [0.0; 3];
    for wl in (LAMBDA_MIN..=LAMBDA_MAX).step_by(LAMBDA_STEP) {
        let rad = solar_radiance_internal2(state, wl as f32, elevation, gamma);
        xyz[0] += xyz_x(wl as f32) * rad;
        xyz[1] += xyz_y(wl as f32) * rad;
        xyz[2] += xyz_z(wl as f32) * rad;
    }
    let rgb = xyz_to_rgb(xyz.into());
    (rgb * LAMBDA_STEP as f32
        / CIE_Y_INTEGRAL
        / (state.sun_angular_radius / TERRESTRIAL_SOLAR_ANGULAR_RADIUS).powi(2))
    .array()
}
