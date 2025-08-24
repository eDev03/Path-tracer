use math::*;

use crate::spaces::Localspace;

pub fn same_hemisphere<S: Localspace>(u: Vector3f32<S>, v: Vector3f32<S>) -> bool {
    u.z * v.z > 0.0
}

pub fn cos_theta<S: Localspace>(v: Vector3f32<S>) -> f32 {
    v.z
}
pub fn abs_cos_theta<S: Localspace>(v: Vector3f32<S>) -> f32 {
    v.z.abs()
}
pub fn cos2theta<S: Localspace>(v: Vector3f32<S>) -> f32 {
    v.z * v.z
}

pub fn sin_theta<S: Localspace>(v: Vector3f32<S>) -> f32 {
    sin2theta(v).sqrt()
}
pub fn sin2theta<S: Localspace>(v: Vector3f32<S>) -> f32 {
    (1.0 - cos2theta(v)).max(0.0)
}
pub fn tan2theta<S: Localspace + Copy>(v: Vector3f32<S>) -> f32 {
    sin2theta(v) / cos2theta(v)
}
pub fn cos_phi<S: Localspace + Copy>(v: Vector3f32<S>) -> f32 {
    let st = sin_theta(v);
    if st == 0.0 {
        1.0
    } else {
        (v.x / st).clamp(-1.0, 1.0)
    }
}
pub fn sin_phi<S: Localspace + Copy>(v: Vector3f32<S>) -> f32 {
    let st = sin_theta(v);
    if st == 0.0 {
        0.0
    } else {
        (v.y / st).clamp(-1.0, 1.0)
    }
}
