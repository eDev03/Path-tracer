use math::*;

pub struct PCG {
    pub state: u64,
    pub inc: u64,
}

pub struct ARLDS {
    pub state: f32,
    pub inc: f32,
}

impl Default for PCG {
    fn default() -> Self {
        Self {
            state: 0x853c49e6748fea9b,
            inc: 0xda3e39cb94b95bdb,
        }
    }
}

impl Default for ARLDS {
    fn default() -> Self {
        Self {
            state: 0.0,
            inc: 2f32.sqrt() - 1.0,
        }
    }
}

impl PCG {
    pub const MULT: u64 = 0x5851f42d4c957f2d;
    pub fn set_sequence(&mut self, seq_index: u64) {
        self.set_sequence_seeded(seq_index, mix_bits(seq_index));
    }
    pub fn set_sequence_seeded(&mut self, seq_index: u64, seed: u64) {
        self.state = 0;
        self.inc = (seq_index << 1) | 1;
        _ = self.u32();
        self.state = self.state.wrapping_add(seed);
        _ = self.u32();
    }
    pub fn advance(&mut self, delta: u64) {
        let mut cur_mult = Self::MULT;
        let mut cur_plus = self.inc;
        let mut acc_mult: u64 = 1;
        let mut acc_plus: u64 = 0;
        let mut delta = delta;
        while delta > 0 {
            if delta & 1 != 0 {
                acc_mult = acc_mult.wrapping_mul(cur_mult);
                acc_plus = acc_plus.wrapping_mul(cur_mult).wrapping_add(cur_plus);
            }
            cur_plus = cur_mult.wrapping_add(1).wrapping_mul(cur_plus);
            cur_mult = cur_mult.wrapping_mul(cur_mult);
            delta /= 2;
        }
        self.state = acc_mult.wrapping_mul(self.state).wrapping_add(acc_plus);
    }
    pub fn u32(&mut self) -> u32 {
        let old_state = self.state;
        self.state = old_state.wrapping_mul(Self::MULT).wrapping_add(self.inc);
        let xorshifted = (((old_state >> 18) ^ old_state) >> 27) as u32;
        let rot = (old_state >> 59) as u32;
        xorshifted.rotate_right(rot)
    }
    pub fn f32(&mut self) -> f32 {
        (1f32 - f32::EPSILON / 2.0).min(self.u32() as f32 * 2f32.powi(-32))
    }
}

pub struct PiecewiseConstant1d {
    cdf: Box<[f32]>,
    values: Box<[f32]>,
    integral: f32,
    size: f32,
}
pub struct PiecewiseConstant2d {
    marginal: PiecewiseConstant1d,
    conditional: Box<[PiecewiseConstant1d]>,
}

impl PiecewiseConstant1d {
    pub fn new(values: &[f32]) -> Self {
        assert!(values.len() > 1);
        let values = values.iter().map(|f| f.abs()).collect::<Box<_>>();
        let mut cdf = vec![0.0; values.len() + 1].into_boxed_slice();
        let size = values.len() as f32;
        for i in 1..cdf.len() {
            cdf[i] = cdf[i - 1] + values[i - 1] / size;
        }
        let integral = cdf[values.len()];
        if integral == 0.0 {
            for i in 1..cdf.len() {
                cdf[i] = i as f32 / size;
            }
        } else {
            for i in 1..cdf.len() {
                cdf[i] /= integral;
            }
        }
        Self {
            cdf,
            values,
            integral,
            size,
        }
    }
    pub fn sample(&self, u: f32) -> (f32, f32, usize) {
        let offset = self
            .cdf
            .partition_point(|&x| x < u)
            .min(self.values.len() - 2);
        let mut du = u - self.cdf[offset];
        if self.cdf[offset + 1] - self.cdf[offset] > 0.0 {
            du /= self.cdf[offset + 1] - self.cdf[offset];
        }
        let pdf = if self.integral > 0.0 {
            self.values[offset] / self.integral
        } else {
            0.0
        };
        ((offset as f32 + du) / self.size, pdf, offset)
    }
}

impl PiecewiseConstant2d {
    pub fn new(values: &[f32], [nu, nv]: [usize; 2]) -> Self {
        let conditional = (0..nv)
            .map(|v| PiecewiseConstant1d::new(&values[v * nu..][..nu]))
            .collect::<Box<_>>();
        let margianl_values = conditional
            .iter()
            .map(|p1d| p1d.integral)
            .collect::<Box<_>>();
        let marginal = PiecewiseConstant1d::new(&margianl_values);
        Self {
            marginal,
            conditional,
        }
    }
    pub fn sample(&self, u2d: [f32; 2]) -> ([f32; 2], f32, [usize; 2]) {
        let (m_u, m_pdf, m_offset) = self.marginal.sample(u2d[0]);
        let (c_u, c_pdf, c_offset) = self.conditional[m_offset].sample(u2d[1]);
        ([c_u, m_u], m_pdf * c_pdf, [c_offset, m_offset])
    }
}

pub fn mix_bits(mut v: u64) -> u64 {
    v ^= v >> 31;
    v = v.wrapping_mul(0x7fb5d329728ea185);
    v ^= v >> 27;
    v = v.wrapping_mul(0x81dadef4bc2dd44d);
    v ^= v >> 33;
    v
}

pub fn floats_to_u64(c: [f32; 2]) -> u64 {
    unsafe { std::mem::transmute(c) }
}

impl ARLDS {
    fn new(n: usize) -> Self {
        let n = n as f32;
        let inc = (n + (n * n + 4.0).sqrt()) / 2.0;
        let inc = inc - inc.floor();
        Self { state: 0.0, inc }
    }
    fn advance(&mut self) -> f32 {
        self.state += self.inc;
        self.state -= self.state.floor();
        self.state
    }
}

#[derive(Default)]
pub struct Sampler {
    pcg: PCG,
    arlds: ARLDS,
}

impl Sampler {
    pub fn init_pixel(&mut self, x: usize, y: usize, i: usize) {
        let seed = mix_bits(((x as u64) << 32) | y as u64);
        self.pcg.set_sequence(seed);
        self.pcg.advance(0xffffu64.wrapping_mul(i as u64));
        if i == 0 {
            self.arlds = ARLDS::new(1);
        }
    }
    pub fn wavelength_sample(&mut self) -> f32 {
        self.arlds.advance()
    }
    pub fn sample1d(&mut self) -> f32 {
        self.pcg.f32()
    }
    pub fn sample2d(&mut self) -> [f32; 2] {
        [self.sample1d(), self.sample1d()]
    }
}

pub fn uniform_barycentric(u2d: [f32; 2]) -> [f32; 3] {
    let mut b: [f32; 3] = Default::default();
    if u2d[0] < u2d[1] {
        b[0] = u2d[0] / 2.0;
        b[1] = u2d[1] - b[0];
    } else {
        b[1] = u2d[1] / 2.0;
        b[0] = u2d[0] - b[1];
    }
    b[2] = 1.0 - b[0] - b[1];
    b
}

pub fn unifor_disk_polar(u2d: [f32; 2]) -> [f32; 2] {
    let r = u2d[0].sqrt();
    let theta = TWO_PI * u2d[1];
    [r * theta.cos(), r * theta.sin()]
}

pub fn unifor_disk_concentric(u2d: [f32; 2]) -> [f32; 2] {
    let offset = [2.0 * u2d[0] - 1.0, 2.0 * u2d[1] - 1.0];
    if offset == [0.0, 0.0] {
        return [0.0, 0.0];
    }
    let (theta, r): (f32, f32);
    if offset[0].abs() > offset[1].abs() {
        r = offset[0];
        theta = PI_OVER_FOUR * offset[1] / offset[0];
    } else {
        r = offset[1];
        theta = PI_OVER_TWO - PI_OVER_FOUR * offset[0] / offset[1];
    }
    [r * theta.cos(), r * theta.sin()]
}

pub fn uniform_sphere(u2d: [f32; 2]) -> [f32; 3] {
    let z = 1.0 - 2.0 * u2d[0];
    let r = (1.0 - z * z).safe_sqrt();
    let phi = TWO_PI * u2d[1];
    return [r * phi.cos(), r * phi.sin(), z];
}

pub fn uniform_hemisphere(u2d: [f32; 2]) -> [f32; 3] {
    let z = u2d[0];
    let r = (1.0 - z * z).safe_sqrt();
    let phi = TWO_PI * u2d[1];
    [r * phi.cos(), r * phi.sin(), z]
}

pub fn cosine_hemisphere(u2d: [f32; 2]) -> [f32; 3] {
    let disk = unifor_disk_polar(u2d);
    let z = (1.0 - disk[0] * disk[0] - disk[1] * disk[1]).safe_sqrt();
    [disk[0], disk[1], z]
}

pub fn uniform_sphere_pdf() -> f32 {
    INV_FOUR_PI
}

pub fn uniform_hemisphere_pdf() -> f32 {
    INV_TWO_PI
}

pub fn cosine_hemisphere_pdf(cos_theta: f32) -> f32 {
    cos_theta * INV_PI
}

pub fn sample_exponential(u1d: f32, a: f32) -> f32 {
    -(1.0 - u1d).ln() / a
}

pub fn sample_discrete(u: f32, vals: &[f32]) -> usize {
    let inv_total = vals.iter().sum::<f32>().recip();
    let mut sum = 0.0;
    for (index, val) in vals.iter().enumerate() {
        sum += val * inv_total;
        if sum >= u {
            return index;
        }
    }
    return vals.len() - 1;
}
