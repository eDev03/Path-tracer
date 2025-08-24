pub trait NumericGamma {
    fn numeric_gamma(n: Self) -> Self;
}

impl NumericGamma for f32 {
    fn numeric_gamma(n: f32) -> f32 {
        const EPS: f32 = f32::EPSILON / 2.0;
        (n * EPS) / (1.0 - n * EPS)
    }
}
