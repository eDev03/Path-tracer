pub mod bounds;
pub mod matrix;
pub mod numerics;
pub mod onb;
pub mod vector;

use core::{f32, f64};

pub use bounds::*;
pub use matrix::*;
pub use numerics::*;
pub use onb::*;
pub use vector::*;

pub const PI: f32 = f32::consts::PI;
pub const TWO_PI: f32 = 2.0 * PI;
pub const FOUR_PI: f32 = 4.0 * PI;
pub const INV_PI: f32 = 1.0 / PI;
pub const INV_TWO_PI: f32 = 1.0 / TWO_PI;
pub const INV_FOUR_PI: f32 = 1.0 / FOUR_PI;
pub const PI_OVER_TWO: f32 = PI / 2.0;
pub const PI_OVER_FOUR: f32 = PI / 4.0;

pub const PI_F64: f64 = f64::consts::PI;
pub const TWO_PI_F64: f64 = 2.0 * PI_F64;
pub const FOUR_PI_F64: f64 = 4.0 * PI_F64;
pub const INV_PI_F64: f64 = 1.0 / PI_F64;
pub const INV_TWO_PI_F64: f64 = 1.0 / TWO_PI_F64;
pub const INV_FOUR_PI_F64: f64 = 1.0 / FOUR_PI_F64;
pub const PI_OVER_TWO_F64: f64 = PI_F64 / 2.0;
pub const PI_OVER_FOUR_F64: f64 = PI_F64 / 4.0;

pub trait SafeSqrt {
    fn safe_sqrt(self) -> Self;
}
impl SafeSqrt for f32 {
    fn safe_sqrt(self) -> Self {
        (0f32.max(self)).sqrt()
    }
}

pub trait Remap {
    fn remap(self, old_min: Self, old_max: Self, new_min: Self, new_max: Self) -> Self;
}
impl Remap for f32 {
    fn remap(self, old_min: Self, old_max: Self, new_min: Self, new_max: Self) -> Self {
        new_min + (self - old_min) * (new_max - new_min) / (old_max - old_min)
    }
}

pub fn cos<T: num_traits::Float>(x: T) -> T {
    x.cos()
}
pub fn sin<T: num_traits::Float>(x: T) -> T {
    x.sin()
}
pub fn tan<T: num_traits::Float>(x: T) -> T {
    x.tan()
}

pub fn asin<T: num_traits::Float>(x: T) -> T {
    x.asin()
}
pub fn acos<T: num_traits::Float>(x: T) -> T {
    x.acos()
}
pub fn atan<T: num_traits::Float>(x: T) -> T {
    x.atan()
}
pub fn atan2<T: num_traits::Float>(y: T, x: T) -> T {
    y.atan2(x)
}
pub fn exp<T: num_traits::Float>(x: T) -> T {
    x.exp()
}

pub fn lerp<T: num_traits::Float>(a: T, b: T, t: T) -> T {
    (T::one() - t) * a + t * b
}
