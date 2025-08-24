use crate::spaces::*;

use math::*;

pub fn spherical_direction<S: Copy>(sin_theta: f32, cos_theta: f32, phi: f32) -> Vector3f32<S> {
    Vector3f32::new(
        sin_theta.clamp(-1.0, 1.0) * phi.cos(),
        sin_theta.clamp(-1.0, 1.0) * phi.sin(),
        cos_theta.clamp(-1.0, 1.0),
    )
}

#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub o: Point3World,
    pub d: Vec3World,
}

pub fn triangle_cross<S: Copy>(
    p0: Point3f32<S>,
    p1: Point3f32<S>,
    p2: Point3f32<S>,
) -> Vector3f32<S> {
    (p0 - p1).cross(p2 - p1)
}

pub fn triangle_area<S: Copy>(p0: Point3f32<S>, p1: Point3f32<S>, p2: Point3f32<S>) -> f32 {
    triangle_cross(p0, p1, p2).mag() * 0.5
}

pub fn triangle_normal<S: Copy>(
    p0: Point3f32<S>,
    p1: Point3f32<S>,
    p2: Point3f32<S>,
) -> Vector3f32<S> {
    triangle_cross(p0, p1, p2).normalized()
}

pub fn triangle_point<S: Copy>(
    p0: Point3f32<S>,
    p1: Point3f32<S>,
    p2: Point3f32<S>,
    [b0, b1, b2]: [f32; 3],
) -> Point3f32<S> {
    (p0.to_vector() * b0 + p1.to_vector() * b1 + p2.to_vector() * b2).to_point()
}

pub fn offset_ray_origin(bounds: Bounds3World, n: Vec3World, w: Vec3World) -> Point3World {
    let extent = bounds
        .extent()
        .max_by_component(Vec3World::splat(f32::EPSILON));
    let d = n.dot(extent).abs();
    let mut offset = d * n;
    if w.dot(n) < 0.0 {
        offset = -offset;
    }
    let mut po = bounds.centroid() + offset;
    for i in 0..3 {
        if offset[i] > 0.0 {
            po[i] = po[i].next_up()
        } else {
            po[i] = po[i].next_down()
        }
    }
    po
}

#[derive(Default, Debug, Clone, Copy)]
pub struct RayTriangleIntersection<S> {
    pub t: f32,
    pub b: [f32; 3],
    _marker: std::marker::PhantomData<S>,
}

pub fn ray_triangle_hit<S: Copy>(
    ro: Point3f32<S>,
    rd: Vector3f32<S>,
    p0: Point3f32<S>,
    p1: Point3f32<S>,
    p2: Point3f32<S>,
    max_t: f32,
) -> bool {
    fn permute<S: Copy>(v: Vector3f32<S>, kz: usize) -> Vector3f32<S> {
        match kz {
            0 => Vector3f32::new(v.y, v.z, v.x),
            1 => Vector3f32::new(v.z, v.x, v.y),
            _ => Vector3f32::new(v.x, v.y, v.z),
        }
    }
    fn difference_of_products(a: f32, b: f32, c: f32, d: f32) -> f32 {
        let cd = c * d;
        let dop = a.mul_add(b, -cd);
        let e = (-c).mul_add(d, cd);
        dop + e
    }

    let mut p0t = p0 - ro;
    let mut p1t = p1 - ro;
    let mut p2t = p2 - ro;

    let kz = rd.abs().max_axis();
    let d = permute(rd, kz);
    p0t = permute(p0t, kz);
    p1t = permute(p1t, kz);
    p2t = permute(p2t, kz);

    let sx = -d.x / d.z;
    let sy = -d.y / d.z;
    let sz = 1.0 / d.z;

    p0t.x += sx * p0t.z;
    p0t.y += sy * p0t.z;
    p1t.x += sx * p1t.z;
    p1t.y += sy * p1t.z;
    p2t.x += sx * p2t.z;
    p2t.y += sy * p2t.z;

    let mut e0 = difference_of_products(p1t.x, p2t.y, p1t.y, p2t.x);
    let mut e1 = difference_of_products(p2t.x, p0t.y, p2t.y, p0t.x);
    let mut e2 = difference_of_products(p0t.x, p1t.y, p0t.y, p1t.x);

    if e0 == 0.0 || e1 == 0.0 || e2 == 0.0 {
        let p2txp1ty = p2t.x as f64 * p1t.y as f64;
        let p2typ1tx = p2t.y as f64 * p1t.x as f64;
        e0 = (p2typ1tx - p2txp1ty) as f32;
        let p0txp2ty = p0t.x as f64 * p2t.y as f64;
        let p0typ2tx = p0t.y as f64 * p2t.x as f64;
        e1 = (p0typ2tx - p0txp2ty) as f32;
        let p1txp0ty = p1t.x as f64 * p0t.y as f64;
        let p1typ0tx = p1t.y as f64 * p0t.x as f64;
        e2 = (p1typ0tx - p1txp0ty) as f32;
    }

    if (e0 < 0.0 || e1 < 0.0 || e2 < 0.0) && (e0 > 0.0 || e1 > 0.0 || e2 > 0.0) {
        return false;
    }
    let det = e0 + e1 + e2;
    if det == 0.0 {
        return false;
    }

    p0t.z *= sz;
    p1t.z *= sz;
    p2t.z *= sz;
    let t_scaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
    if det < 0.0 && (t_scaled >= 0.0 || t_scaled < max_t * det) {
        false;
    }
    if det > 0.0 && (t_scaled <= 0.0 || t_scaled > max_t * det) {
        return false;
    }

    let inv_det = 1.0 / det;
    let t = t_scaled * inv_det;

    let max_zt = p0t.z.abs().max(p1t.z.abs()).max(p2t.z.abs());
    let delta_z = f32::numeric_gamma(3.0) * max_zt;

    let max_xt = p0t.x.abs().max(p1t.x.abs()).max(p2t.x.abs());
    let max_yt = p0t.y.abs().max(p1t.y.abs()).max(p2t.y.abs());

    let delta_x = f32::numeric_gamma(5.0) * (max_xt + max_zt);
    let delta_y = f32::numeric_gamma(5.0) * (max_yt + max_zt);

    let delta_e =
        2.0 * (f32::numeric_gamma(2.0) * max_xt * max_yt + delta_y * max_xt + delta_x * max_yt);

    let max_e = e0.abs().max(e1.abs()).max(e2.abs());
    let delta_t = 3.0
        * (f32::numeric_gamma(3.0) * max_e * max_zt + delta_e * max_zt + delta_z * max_e)
        * inv_det.abs();

    if t <= delta_t {
        return false;
    }

    return true;
}

pub fn ray_triangle_intersection<S: Copy>(
    ro: Point3f32<S>,
    rd: Vector3f32<S>,
    p0: Point3f32<S>,
    p1: Point3f32<S>,
    p2: Point3f32<S>,
    max_t: f32,
) -> Option<RayTriangleIntersection<S>> {
    fn permute<S: Copy>(v: Vector3f32<S>, kz: usize) -> Vector3f32<S> {
        match kz {
            0 => Vector3f32::new(v.y, v.z, v.x),
            1 => Vector3f32::new(v.z, v.x, v.y),
            _ => Vector3f32::new(v.x, v.y, v.z),
        }
    }
    fn difference_of_products(a: f32, b: f32, c: f32, d: f32) -> f32 {
        let cd = c * d;
        let dop = a.mul_add(b, -cd);
        let e = (-c).mul_add(d, cd);
        dop + e
    }

    let mut p0t = p0 - ro;
    let mut p1t = p1 - ro;
    let mut p2t = p2 - ro;

    let kz = rd.abs().max_axis();
    let d = permute(rd, kz);
    p0t = permute(p0t, kz);
    p1t = permute(p1t, kz);
    p2t = permute(p2t, kz);

    let sx = -d.x / d.z;
    let sy = -d.y / d.z;
    let sz = 1.0 / d.z;

    p0t.x += sx * p0t.z;
    p0t.y += sy * p0t.z;
    p1t.x += sx * p1t.z;
    p1t.y += sy * p1t.z;
    p2t.x += sx * p2t.z;
    p2t.y += sy * p2t.z;

    let mut e0 = difference_of_products(p1t.x, p2t.y, p1t.y, p2t.x);
    let mut e1 = difference_of_products(p2t.x, p0t.y, p2t.y, p0t.x);
    let mut e2 = difference_of_products(p0t.x, p1t.y, p0t.y, p1t.x);

    if e0 == 0.0 || e1 == 0.0 || e2 == 0.0 {
        let p2txp1ty = p2t.x as f64 * p1t.y as f64;
        let p2typ1tx = p2t.y as f64 * p1t.x as f64;
        e0 = (p2typ1tx - p2txp1ty) as f32;
        let p0txp2ty = p0t.x as f64 * p2t.y as f64;
        let p0typ2tx = p0t.y as f64 * p2t.x as f64;
        e1 = (p0typ2tx - p0txp2ty) as f32;
        let p1txp0ty = p1t.x as f64 * p0t.y as f64;
        let p1typ0tx = p1t.y as f64 * p0t.x as f64;
        e2 = (p1typ0tx - p1txp0ty) as f32;
    }

    if (e0 < 0.0 || e1 < 0.0 || e2 < 0.0) && (e0 > 0.0 || e1 > 0.0 || e2 > 0.0) {
        return None;
    }
    let det = e0 + e1 + e2;
    if det == 0.0 {
        return None;
    }

    p0t.z *= sz;
    p1t.z *= sz;
    p2t.z *= sz;
    let t_scaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
    if det < 0.0 && (t_scaled >= 0.0 || t_scaled < max_t * det) {
        return None;
    }
    if det > 0.0 && (t_scaled <= 0.0 || t_scaled > max_t * det) {
        return None;
    }

    let inv_det = 1.0 / det;
    let t = t_scaled * inv_det;

    let max_zt = p0t.z.abs().max(p1t.z.abs()).max(p2t.z.abs());
    let delta_z = f32::numeric_gamma(3.0) * max_zt;

    let max_xt = p0t.x.abs().max(p1t.x.abs()).max(p2t.x.abs());
    let max_yt = p0t.y.abs().max(p1t.y.abs()).max(p2t.y.abs());

    let delta_x = f32::numeric_gamma(5.0) * (max_xt + max_zt);
    let delta_y = f32::numeric_gamma(5.0) * (max_yt + max_zt);

    let delta_e =
        2.0 * (f32::numeric_gamma(2.0) * max_xt * max_yt + delta_y * max_xt + delta_x * max_yt);

    let max_e = e0.abs().max(e1.abs()).max(e2.abs());
    let delta_t = 3.0
        * (f32::numeric_gamma(3.0) * max_e * max_zt + delta_e * max_zt + delta_z * max_e)
        * inv_det.abs();

    if t <= delta_t {
        return None;
    }

    return Some(RayTriangleIntersection {
        t,
        b: [e0 * inv_det, e1 * inv_det, e2 * inv_det],
        _marker: std::marker::PhantomData {},
    });
}
