use crate::vector::*;

macro_rules! create_bounds_and_impl_common {
    ($bounds2:ident,$point2:ident,$vec2:ident,$bounds3:ident,$point3:ident,$vec3:ident,$t:ident) => {
        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $bounds2<S> {
            pub min: $point2<S>,
            pub max: $point2<S>,
        }

        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $bounds3<S> {
            pub min: $point3<S>,
            pub max: $point3<S>,
        }
        impl<S: Copy> $bounds2<S> {
            pub fn new(a: $point2<S>, b: $point2<S>) -> Self {
                Self {
                    min: a.min_by_component(b),
                    max: a.max_by_component(b),
                }
            }
            pub fn around(b: $point2<S>, extent: $vec2<S>) -> Self {
                let half = extent / 2 as $t;
                let min = b - half;
                let max = b + half;
                Self { min, max }
            }
            pub fn unit() -> Self {
                Self {
                    min: $point2::splat(0 as $t),
                    max: $point2::splat(1 as $t),
                }
            }
            pub fn contains_inclusive(&self, p: $point2<S>) -> bool {
                (self.min.x <= p.x && p.x <= self.max.x) && (self.min.y <= p.y && p.y <= self.max.y)
            }
            pub fn centroid(&self) -> $point2<S> {
                (self.min + self.max.to_vector()) / 2 as $t
            }
            pub fn clip(&self, p: $point2<S>) -> $point2<S> {
                p.clamp(self.min, self.max)
            }
            pub fn clamp(&self, min: $point2<S>, max: $point2<S>) -> Self {
                let min = min.clamp(self.min, self.max);
                let max = max.clamp(self.min, self.max);
                Self { min, max }
            }
            pub fn extent(&self) -> $vec2<S> {
                self.max - self.min
            }
            pub fn area(&self) -> $t {
                let e = self.extent();
                e.x * e.y
            }
            pub fn offset(&self, p: $point2<S>) -> $point2<S> {
                let e = self.extent();
                let o = p - self.min;
                return (o / e).to_point();
            }
            pub fn lerp(&self, v: $vec2<S>) -> $point2<S> {
                self.min + v * self.extent()
            }
            pub fn overlap(&self, other: Self) -> $t {
                let min = self.min.max_by_component(other.min);
                let max = self.max.min_by_component(other.max);
                let extent = max - min;
                if extent.x <= 0 as $t || extent.y <= 0 as $t {
                    return 0 as $t;
                }
                Self { min, max }.area()
            }
        }
        impl<S: Copy> $bounds3<S> {
            pub fn new(a: $point3<S>, b: $point3<S>) -> Self {
                Self {
                    min: a.min_by_component(b),
                    max: a.max_by_component(b),
                }
            }
            pub fn around(b: $point3<S>, extent: $vec3<S>) -> Self {
                let half = extent / 2 as $t;
                let min = b - half;
                let max = b + half;
                Self { min, max }
            }
            pub fn contains_inclusive(&self, p: $point3<S>) -> bool {
                (self.min.x <= p.x && p.x <= self.max.x)
                    && (self.min.y <= p.y && p.y <= self.max.y)
                    && (self.min.z <= p.z && p.z <= self.max.z)
            }
            pub fn centroid(&self) -> $point3<S> {
                (self.min + self.max.to_vector()) / 2 as $t
            }
            pub fn clip(&self, p: $point3<S>) -> $point3<S> {
                p.clamp(self.min, self.max)
            }
            pub fn clamp(&self, min: $point3<S>, max: $point3<S>) -> Self {
                let min = min.clamp(self.min, self.max);
                let max = max.clamp(self.min, self.max);
                Self { min, max }
            }
            pub fn extent(&self) -> $vec3<S> {
                self.max - self.min
            }
            pub fn volume(&self) -> $t {
                let e = self.extent();
                e.x * e.y * e.z
            }
            pub fn surface_area(&self) -> $t {
                let e = self.extent();
                2 as $t * (e.x * e.y + e.y * e.z + e.x * e.z)
            }
            pub fn offset(&self, p: $point3<S>) -> $point3<S> {
                let e = self.extent();
                let o = p - self.min;
                return (o / e).to_point();
            }
            pub fn lerp(&self, v: $vec3<S>) -> $point3<S> {
                self.min + v * self.extent()
            }
            pub fn overlap(&self, other: Self) -> $t {
                let min = self.min.max_by_component(other.min);
                let max = self.max.min_by_component(other.max);
                let extent = max - min;
                if extent.x <= 0 as $t || extent.y <= 0 as $t || extent.z <= 0 as $t {
                    return 0 as $t;
                }
                Self { min, max }.volume()
            }
        }

        impl<S: Copy> std::ops::BitOr<Self> for $bounds3<S> {
            type Output = Self;
            fn bitor(self, other: Self) -> Self {
                Self {
                    min: self.min.min_by_component(other.min),
                    max: self.max.max_by_component(other.max),
                }
            }
        }
        impl<S: Copy> std::ops::BitOr<$point3<S>> for $bounds3<S> {
            type Output = Self;
            fn bitor(self, other: $point3<S>) -> Self {
                Self {
                    min: self.min.min_by_component(other),
                    max: self.max.max_by_component(other),
                }
            }
        }
        impl<S: Copy> std::ops::BitOrAssign<Self> for $bounds3<S> {
            fn bitor_assign(&mut self, other: Self) {
                *self = *self | other
            }
        }
        impl<S: Copy> std::ops::BitOrAssign<$point3<S>> for $bounds3<S> {
            fn bitor_assign(&mut self, other: $point3<S>) {
                *self = *self | other
            }
        }
    };
}

macro_rules! impl_float {
    ($bounds2:ident,$point2:ident,$vec2:ident,$bounds3:ident,$point3:ident,$vec3:ident,$t:ident) => {
        impl<S: Copy> $bounds3<S> {
            pub fn empty() -> Self {
                Self {
                    min: $point3::splat(f32::INFINITY),
                    max: $point3::splat(f32::NEG_INFINITY),
                }
            }
        }
    };
}

create_bounds_and_impl_common!(
    Bounds2f32, Point2f32, Vector2f32, Bounds3f32, Point3f32, Vector3f32, f32
);
impl_float!(
    Bounds2f32, Point2f32, Vector2f32, Bounds3f32, Point3f32, Vector3f32, f32
);
