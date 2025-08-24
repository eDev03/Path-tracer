const COEFF_DATA: &[f32] = unsafe {
    #[repr(align(4))]
    struct Aligner([u8; include_bytes!("srgb.coeffs").len()]);
    const BYTES: Aligner = Aligner(*include_bytes!("srgb.coeffs"));
    std::slice::from_raw_parts(BYTES.0.as_ptr().cast(), BYTES.0.len() / size_of::<f32>())
};

pub fn fetch_coeffs(rgb: [f32; 3]) -> [f32; 3] {
    let model_scale = &COEFF_DATA[2..][..64];
    let model_data = &COEFF_DATA[66..];

    let res = model_scale.len() as i32;
    let rgb = rgb.map(|v| v.clamp(0.0, 1.0));
    let i = (0..3).max_by(|i, j| rgb[*i].total_cmp(&rgb[*j])).unwrap();
    let z = rgb[i];
    let scale = (res as f32 - 1.0) / z;
    let x = rgb[(i + 1) % 3] * scale;
    let y = rgb[(i + 2) % 3] * scale;
    let xi = (x as i32).min(res - 2);
    let yi = (y as i32).min(res - 2);
    let zi = find_interval(model_scale, z);
    let dx = 3;
    let dy = 3 * res;
    let dz = 3 * res * res;
    let x1 = x - xi as f32;
    let x0 = 1.0 - x1;
    let y1 = y - yi as f32;
    let y0 = 1.0 - y1;
    let z1 =
        (z - model_scale[zi as usize]) / (model_scale[zi as usize + 1] - model_scale[zi as usize]);
    let z0 = 1.0 - z1;

    let mut offset = (((i as i32 * res + zi) * res + yi) * res + xi) * 3;
    let mut out = [0.0; 3];

    for j in 0..3 {
        out[j] = ((model_data[offset as usize] * x0 + model_data[(offset + dx) as usize] * x1)
            * y0
            + (model_data[(offset + dy) as usize] * x0
                + model_data[(offset + dy + dx) as usize] * x1)
                * y1)
            * z0
            + ((model_data[(offset + dz) as usize] * x0
                + model_data[(offset + dz + dx) as usize] * x1)
                * y0
                + (model_data[(offset + dz + dy) as usize] * x0
                    + model_data[(offset + dz + dy + dx) as usize] * x1)
                    * y1)
                * z1;
        offset += 1;
    }

    out
}

fn find_interval(values: &[f32], x: f32) -> i32 {
    let last = values.len() - 2;
    let mut left = 0;
    let mut size = last;
    while size > 0 {
        let half = size >> 1;
        let middle = left + half;
        if values[middle] <= x {
            left = middle;
            size -= half + 1;
        } else {
            size = half;
        }
    }
    left.min(last) as i32
}
