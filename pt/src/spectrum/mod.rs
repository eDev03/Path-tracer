use crate::{io_bridge, spaces::*};

pub mod cie_data;
pub mod rgb2spec;

pub const N_SAMPLED_WAVELENGTHS: usize = 8;
pub const LAMBDA_MIN: f32 = 360.0;
pub const LAMBDA_MAX: f32 = 830.0;
fn lambda_index(l: f32) -> usize {
    (l - LAMBDA_MIN) as usize
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SampledSpectrum(pub [f32; N_SAMPLED_WAVELENGTHS]);
pub struct SampledWavelengths {
    pub lambdas: [f32; N_SAMPLED_WAVELENGTHS],
    pub pdfs: [f32; N_SAMPLED_WAVELENGTHS],
}

#[derive(Debug, Clone)]
pub enum Spectrum {
    TristimulusSpectrum(TristimulusSpectrum),
    TristimulusIlluminantSpectrum(TristimulusIlluminantSpectrum),
}

impl Spectrum {
    pub fn new(spec: io_bridge::Spectrum, illuminant: bool) -> Self {
        match spec {
            io_bridge::Spectrum::RGB(rgb) => {
                if illuminant {
                    Self::tristimulus_illuminant(rgb)
                } else {
                    Self::tristimulus(rgb)
                }
            }
            io_bridge::Spectrum::Constant(x) => {
                if illuminant {
                    Self::tristimulus_illuminant([x, x, x].into())
                } else {
                    Self::tristimulus([x, x, x].into())
                }
            }
        }
    }
    pub fn tristimulus(rgb: RGBf32) -> Self {
        Self::TristimulusSpectrum(TristimulusSpectrum::new(rgb))
    }
    pub fn tristimulus_illuminant(rgb: RGBf32) -> Self {
        Self::TristimulusIlluminantSpectrum(TristimulusIlluminantSpectrum::new(rgb))
    }
    pub fn eval(&self, lambdas: [f32; N_SAMPLED_WAVELENGTHS]) -> SampledSpectrum {
        SampledSpectrum(match self {
            Spectrum::TristimulusSpectrum(tristimulus_spectrum) => {
                lambdas.map(|l| tristimulus_spectrum.eval(l))
            }
            Spectrum::TristimulusIlluminantSpectrum(tristimulus_illuminant_spectrum) => {
                lambdas.map(|l| tristimulus_illuminant_spectrum.eval(l))
            }
        })
    }
}

impl From<[f32; N_SAMPLED_WAVELENGTHS]> for SampledSpectrum {
    fn from(value: [f32; N_SAMPLED_WAVELENGTHS]) -> Self {
        Self(value)
    }
}

impl SampledSpectrum {
    pub const ZERO: Self = Self([0.0; N_SAMPLED_WAVELENGTHS]);
    pub const ONE: Self = Self([1.0; N_SAMPLED_WAVELENGTHS]);
    pub fn splat(v: f32) -> Self {
        Self([v; N_SAMPLED_WAVELENGTHS])
    }
    pub fn is_uniform(&self) -> bool {
        self.0[1..].iter().all(|&v| v == self.0[0])
    }
    pub fn is_black(&self) -> bool {
        self.0.iter().all(|&v| v == 0.0)
    }
    pub fn to_xyz(&self, swl: &SampledWavelengths) -> XYZf64 {
        let mut acc = XYZf64::ZERO;
        for i in 0..N_SAMPLED_WAVELENGTHS {
            let xyz = XYZf64::from(cie_data::CIE_XYZ_F64[lambda_index(swl.lambdas[i])]);
            if swl.pdfs[i] != 0.0 {
                acc += xyz * (self.0[i] / swl.pdfs[i]) as f64;
            }
        }
        acc / (N_SAMPLED_WAVELENGTHS as f64)
    }
    pub fn exp(self) -> Self {
        Self(self.0.map(|x| x.exp()))
    }
    pub fn average(&self) -> f32 {
        self.0.iter().sum::<f32>() / N_SAMPLED_WAVELENGTHS as f32
    }
    pub fn clamp0(self) -> Self {
        Self(self.0.map(|f| f.max(0.0)))
    }
}

impl SampledWavelengths {
    pub fn sample_visible(u1d: f32) -> Self {
        let mut res = Self {
            lambdas: [0.0; N_SAMPLED_WAVELENGTHS],
            pdfs: [0.0; N_SAMPLED_WAVELENGTHS],
        };
        for i in 0..N_SAMPLED_WAVELENGTHS {
            let mut u = u1d + i as f32 / N_SAMPLED_WAVELENGTHS as f32;
            if u >= 1.0 {
                u -= 1.0;
            }
            res.lambdas[i] = sample_visible(u);
            res.pdfs[i] = visble_pdf(res.lambdas[i]);
        }
        res
    }
    pub fn terminate_secondary(&mut self) {
        if self.secondary_terminated() {
            return;
        }
        self.pdfs[0] /= N_SAMPLED_WAVELENGTHS as f32;
        self.pdfs[1..].fill(0.0);
    }
    pub fn secondary_terminated(&self) -> bool {
        self.pdfs[1..].iter().all(|&v| v == 0.0)
    }
}

pub fn xyz_to_rgb(xyz: XYZf64) -> RGBf64 {
    RGBf64::from([
        xyz.dot(XYZf64::from([3.240479, -1.537150, -0.498535])),
        xyz.dot(XYZf64::from([-0.969256, 1.875991, 0.041556])),
        xyz.dot(XYZf64::from([0.055648, -0.204043, 1.057311])),
    ])
}

pub fn xyz_to_rgb_f32(xyz: XYZf32) -> RGBf32 {
    RGBf32::from([
        xyz.dot(XYZf32::from([3.240479, -1.537150, -0.498535])),
        xyz.dot(XYZf32::from([-0.969256, 1.875991, 0.041556])),
        xyz.dot(XYZf32::from([0.055648, -0.204043, 1.057311])),
    ])
}

pub fn rgb_to_xyz(rgb: RGBf64) -> XYZf64 {
    XYZf64::from([
        rgb.dot(RGBf64::from([0.4124564, 0.3575761, 0.1804375])),
        rgb.dot(RGBf64::from([0.2126729, 0.7151522, 0.0721750])),
        rgb.dot(RGBf64::from([0.0193339, 0.1191920, 0.9503041])),
    ])
}

fn sample_visible(u: f32) -> f32 {
    538.0 - 138.888889 * (0.85691062 - 1.82750197 * u).atanh()
}
fn visble_pdf(lambda: f32) -> f32 {
    0.0039398042 / (0.0072 * (lambda - 538.0)).cosh().powi(2)
}

impl std::ops::Add for SampledSpectrum {
    type Output = SampledSpectrum;

    fn add(self, rhs: Self) -> Self::Output {
        let mut res = self;
        for i in 0..N_SAMPLED_WAVELENGTHS {
            res.0[i] += rhs.0[i];
        }
        res
    }
}
impl std::ops::Mul for SampledSpectrum {
    type Output = SampledSpectrum;

    fn mul(self, rhs: Self) -> Self::Output {
        let mut res = self;
        for i in 0..N_SAMPLED_WAVELENGTHS {
            res.0[i] *= rhs.0[i];
        }
        res
    }
}
impl std::ops::Mul<f32> for SampledSpectrum {
    type Output = SampledSpectrum;

    fn mul(self, rhs: f32) -> Self::Output {
        let mut res = self;
        for i in 0..N_SAMPLED_WAVELENGTHS {
            res.0[i] *= rhs;
        }
        res
    }
}
impl std::ops::Mul<SampledSpectrum> for f32 {
    type Output = SampledSpectrum;

    fn mul(self, spec: SampledSpectrum) -> Self::Output {
        spec * self
    }
}
impl std::ops::AddAssign for SampledSpectrum {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}
impl std::ops::MulAssign for SampledSpectrum {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs
    }
}
impl std::ops::Div for SampledSpectrum {
    type Output = SampledSpectrum;

    fn div(self, rhs: Self) -> Self::Output {
        let mut res = self;
        for i in 0..N_SAMPLED_WAVELENGTHS {
            res.0[i] /= rhs.0[i];
        }
        res
    }
}
impl std::ops::Div<f32> for SampledSpectrum {
    type Output = SampledSpectrum;

    fn div(self, rhs: f32) -> Self::Output {
        let mut res = self;
        for i in 0..N_SAMPLED_WAVELENGTHS {
            res.0[i] /= rhs;
        }
        res
    }
}
impl std::ops::MulAssign<f32> for SampledSpectrum {
    fn mul_assign(&mut self, rhs: f32) {
        *self *= Self::splat(rhs)
    }
}
impl std::ops::Sub for SampledSpectrum {
    type Output = SampledSpectrum;

    fn sub(self, rhs: Self) -> Self::Output {
        let mut res = self;
        for i in 0..N_SAMPLED_WAVELENGTHS {
            res.0[i] -= rhs.0[i];
        }
        res
    }
}
impl std::ops::Index<usize> for SampledSpectrum {
    type Output = f32;

    fn index(&self, index: usize) -> &Self::Output {
        self.0.index(index)
    }
}
impl std::ops::IndexMut<usize> for SampledSpectrum {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self.0.index_mut(index)
    }
}

#[derive(Debug, Clone)]
pub struct TristimulusSpectrum {
    sp: SigmoidPolynomial,
    pub scale: f32,
}

#[derive(Debug, Clone)]
pub struct TristimulusIlluminantSpectrum(pub TristimulusSpectrum);

impl TristimulusSpectrum {
    pub fn new(rgb: RGBf32) -> Self {
        let mut scale = rgb[0].max(rgb[1]).max(rgb[2]);
        assert!(scale.is_finite());
        if scale == 0.0 {
            return Self {
                sp: SigmoidPolynomial {
                    c0: 0.0,
                    c1: 0.0,
                    c2: 0.0,
                },
                scale,
            };
        }
        scale *= 2.0;
        let rgb = [rgb[0] / scale, rgb[1] / scale, rgb[2] / scale];
        if rgb[0] == rgb[1] && rgb[1] == rgb[2] {
            return Self {
                sp: SigmoidPolynomial {
                    c0: 0.0,
                    c1: 0.0,
                    c2: (rgb[0] - 0.5) / (rgb[0] * (1.0 - rgb[0])).powi(2),
                },
                scale,
            };
        };
        let [c0, c1, c2] = rgb2spec::fetch_coeffs(rgb);
        Self {
            sp: SigmoidPolynomial { c0, c1, c2 },
            scale,
        }
    }
    pub fn eval(&self, lambda: f32) -> f32 {
        self.sp.eval(lambda) * self.scale
    }
}

impl TristimulusIlluminantSpectrum {
    pub fn new(rgb: RGBf32) -> Self {
        Self(TristimulusSpectrum::new(rgb))
    }
    pub fn eval(&self, lambda: f32) -> f32 {
        self.0.eval(lambda) * cie_data::CIE_D65_F32[lambda_index(lambda)]
    }
}

#[derive(Debug, Clone)]
struct SigmoidPolynomial {
    c0: f32,
    c1: f32,
    c2: f32,
}

impl SigmoidPolynomial {
    fn eval(&self, lambda: f32) -> f32 {
        let x = lambda.mul_add(lambda.mul_add(self.c0, self.c1), self.c2);
        if x.is_infinite() && x > 0.0 {
            return 1.0;
        }
        if x.is_infinite() && x < 0.0 {
            return 0.0;
        }
        return 0.5 + x / (2.0 * (1.0 + x * x).sqrt());
    }
}
