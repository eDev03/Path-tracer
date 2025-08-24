use crate::common::*;

use math::*;

pub const LUMINANCE_SCALE: f32 = 106.856980;

/// t is turbidity, how "hayzy" the atmosphere is
/// theta_s is the solar angle from zenith in radians
/// theta is the same for view vector
/// gamma is angle between sun and view vector
#[allow(non_snake_case)]
pub fn sky_xyz(t: f32, theta_s: f32, theta: f32, gamma: f32) -> [f32; 3] {
    // A Practical Analytic Model for Daylight by Preetma et al
    // https://courses.cs.duke.edu/cps124/spring08/assign/07_papers/p91-preetham.pdf
    let Y_params = Y_params(t);
    let Y = Y_zenith(t, theta_s) * F(theta, gamma, Y_params) / F(0.0, theta_s, Y_params);
    let x_params = x_params(t);
    let x = x_zenith(t, theta_s) * F(theta, gamma, x_params) / F(0.0, theta_s, x_params);
    let y_params = y_params(t);
    let y = y_zenith(t, theta_s) * F(theta, gamma, y_params) / F(0.0, theta_s, y_params);
    Yxy_to_xyz(Y, x, y)
}

#[allow(non_snake_case)]
fn F(theta: f32, gamma: f32, [A, B, C, D, E]: [f32; 5]) -> f32 {
    let res = (1.0 + A * exp(B / cos(theta))) * (1.0 + C * exp(D * gamma) + E * cos(gamma).powi(2));
    if res.is_finite() { res } else { 0.0 }
}

#[allow(non_snake_case)]
fn Y_zenith(T: f32, theta_s: f32) -> f32 {
    (4.0453 * T - 4.9710) * tan((4.0 / 9.0 - T * 120f32.recip()) * (PI - 2.0 * theta_s))
        - 0.2155 * T
        + 2.4192
}
#[allow(non_snake_case)]
fn x_zenith(T: f32, theta_s: f32) -> f32 {
    let theta2 = theta_s * theta_s;
    let theta3 = theta2 * theta_s;
    (0.00165 * theta3 - 0.00375 * theta2 + 0.00209 * theta_s + 0.0) * T * T
        + (-0.02903 * theta3 + 0.06377 * theta2 - 0.03202 * theta_s + 0.00394) * T
        + (0.11693 * theta3 - 0.21196 * theta2 + 0.06052 * theta_s + 0.25886)
}
#[allow(non_snake_case)]
fn y_zenith(T: f32, theta_s: f32) -> f32 {
    let theta2 = theta_s * theta_s;
    let theta3 = theta2 * theta_s;
    (0.00275 * theta3 - 0.00610 * theta2 + 0.00317 * theta_s + 0.0) * T * T
        + (-0.04214 * theta3 + 0.08970 * theta2 - 0.04153 * theta_s + 0.00516) * T
        + (0.15346 * theta3 - 0.26756 * theta2 + 0.06670 * theta_s + 0.26688)
}

#[allow(non_snake_case)]
fn Y_params(t: f32) -> [f32; 5] {
    [
        0.1787 * t - 1.4630,
        -0.3554 * t + 0.4275,
        -0.0227 * t + 5.3251,
        0.1206 * t - 2.5771,
        -0.0670 * t + 0.3703,
    ]
}
fn x_params(t: f32) -> [f32; 5] {
    [
        -0.0193 * t - 0.2592,
        -0.0665 * t + 0.0008,
        -0.0004 * t + 0.2125,
        -0.0641 * t - 0.8989,
        -0.0033 * t + 0.0452,
    ]
}
fn y_params(t: f32) -> [f32; 5] {
    [
        -0.0167 * t - 0.2608,
        -0.0950 * t + 0.0092,
        -0.0079 * t + 0.2102,
        -0.0441 * t - 1.6537,
        -0.0109 * t + 0.0529,
    ]
}
