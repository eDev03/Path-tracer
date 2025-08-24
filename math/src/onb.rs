use crate::*;

#[derive(Debug, Clone, Copy)]
pub struct ONB3f32<From, To> {
    pub x: Vector3f32<From>,
    pub y: Vector3f32<From>,
    pub z: Vector3f32<From>,
    pub _marker: std::marker::PhantomData<To>,
}

impl<From: Copy, To: Copy> ONB3f32<From, To> {
    pub fn init_z(z: Vector3f32<From>) -> Self {
        let sign = 1f32.copysign(z.z);
        let a = -1f32 / (sign + z.z);
        let b = z.x * z.y * a;
        let x = Vector3f32::new(b, sign + z.y * z.y * a, -z.y);
        let y = Vector3f32::new(1.0 + sign * z.x * z.x * a, sign * b, -sign * z.x);
        Self {
            x,
            y,
            z,
            _marker: std::marker::PhantomData {},
        }
    }
    pub fn apply_vector(&self, v: Vector3f32<From>) -> Vector3f32<To> {
        Vector3f32::new(self.x.dot(v), self.y.dot(v), self.z.dot(v))
    }
    pub fn apply_vector_inverse(&self, v: Vector3f32<To>) -> Vector3f32<From> {
        self.x * v.x + self.y * v.y + self.z * v.z
    }
    pub fn apply_point(&self, p: Point3f32<From>) -> Point3f32<To> {
        let v = p.to_vector();
        Point3f32::new(self.x.dot(v), self.y.dot(v), self.z.dot(v))
    }
    pub fn apply_point_inverse(&self, p: Point3f32<To>) -> Point3f32<From> {
        (self.x * p.x + self.y * p.y + self.z * p.z).to_point()
    }
}
