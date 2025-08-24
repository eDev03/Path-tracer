macro_rules! create_structs_and_impl_common {
    (
        $Vector2:ident,$Vector3:ident,$Vector4:ident,$Point2:ident,$Point3:ident,$Point4:ident,$T:ident
    ) => {
        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $Vector2<S> {
            pub x: $T,
            pub y: $T,
            _marker: std::marker::PhantomData<S>,
        }
        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $Vector3<S> {
            pub x: $T,
            pub y: $T,
            pub z: $T,
            _marker: std::marker::PhantomData<S>,
        }
        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $Vector4<S> {
            pub x: $T,
            pub y: $T,
            pub z: $T,
            pub w: $T,
            _marker: std::marker::PhantomData<S>,
        }
        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $Point2<S> {
            pub x: $T,
            pub y: $T,
            _marker: std::marker::PhantomData<S>,
        }
        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $Point3<S> {
            pub x: $T,
            pub y: $T,
            pub z: $T,
            _marker: std::marker::PhantomData<S>,
        }
        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $Point4<S> {
            pub x: $T,
            pub y: $T,
            pub z: $T,
            pub w: $T,
            _marker: std::marker::PhantomData<S>,
        }
        impl<S: Copy> $Vector2<S> {
            pub const ZERO: Self = Self::splat(0 as $T);
            pub const ONE: Self = Self::splat(1 as $T);
            pub const fn new(x: $T, y: $T) -> Self {
                Self {
                    x,
                    y,
                    _marker: std::marker::PhantomData {},
                }
            }
            pub const fn splat(x: $T) -> Self {
                Self::new(x, x)
            }
            pub fn abs(self) -> Self {
                Self::new(self.x.abs(), self.y.abs())
            }
            pub fn clamp(self, low: Self, high: Self) -> Self {
                Self::new(self.x.clamp(low.x, high.x), self.y.clamp(low.y, high.y))
            }
            pub fn min_component(self) -> $T {
                self.x.min(self.y)
            }
            pub fn max_component(self) -> $T {
                self.x.max(self.y)
            }
            pub fn min_axis(self) -> usize {
                if self.x < self.y { 0 } else { 1 }
            }
            pub fn max_axis(self) -> usize {
                if self.x > self.y { 0 } else { 1 }
            }
            pub fn min_by_component(self, other: Self) -> Self {
                Self::new(self.x.min(other.x), self.y.min(other.y))
            }
            pub fn max_by_component(self, other: Self) -> Self {
                Self::new(self.x.max(other.x), self.y.max(other.y))
            }
            pub fn extend_z(self, z: $T) -> $Vector3<S> {
                $Vector3::new(self.x, self.y, z)
            }
            pub fn dot(self, other: Self) -> $T {
                self.x * other.x + self.y * other.y
            }
            pub fn mag_sq(self) -> $T {
                self.dot(self)
            }
            pub fn array(self) -> [$T; 2] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref(&self) -> &[$T; 2] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref_mut(&mut self) -> &mut [$T; 2] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn to_point(self) -> $Point2<S> {
                unsafe { std::mem::transmute(self) }
            }
        }
        impl<S: Copy> $Vector3<S> {
            pub const ZERO: Self = Self::splat(0 as $T);
            pub const ONE: Self = Self::splat(1 as $T);
            pub const fn new(x: $T, y: $T, z: $T) -> Self {
                Self {
                    x,
                    y,
                    z,
                    _marker: std::marker::PhantomData {},
                }
            }
            pub const fn splat(x: $T) -> Self {
                Self::new(x, x, x)
            }
            pub fn abs(self) -> Self {
                Self::new(self.x.abs(), self.y.abs(), self.z.abs())
            }
            pub fn clamp(self, low: Self, high: Self) -> Self {
                Self::new(
                    self.x.clamp(low.x, high.x),
                    self.y.clamp(low.y, high.y),
                    self.z.clamp(low.z, high.z),
                )
            }
            pub fn min_component(self) -> $T {
                self.x.min(self.y).min(self.z)
            }
            pub fn max_component(self) -> $T {
                self.x.max(self.y).max(self.z)
            }
            pub fn min_axis(self) -> usize {
                let mut axis = 0;
                let mut min = self.x;
                if self.y < min {
                    min = self.y;
                    axis = 1;
                }
                if self.z < min {
                    axis = 2;
                }
                axis
            }
            pub fn max_axis(self) -> usize {
                let mut axis = 0;
                let mut max = self.x;
                if self.y > max {
                    max = self.y;
                    axis = 1;
                }
                if self.z > max {
                    axis = 2;
                }
                axis
            }
            pub fn min_by_component(self, other: Self) -> Self {
                Self::new(
                    self.x.min(other.x),
                    self.y.min(other.y),
                    self.z.min(other.z),
                )
            }
            pub fn max_by_component(self, other: Self) -> Self {
                Self::new(
                    self.x.max(other.x),
                    self.y.max(other.y),
                    self.z.max(other.z),
                )
            }
            pub fn extend_w(self, w: $T) -> $Vector4<S> {
                $Vector4::new(self.x, self.y, self.z, w)
            }
            pub fn cross(self, other: Self) -> Self {
                Self::new(
                    self.y * other.z - self.z * other.y,
                    self.z * other.x - self.x * other.z,
                    self.x * other.y - self.y * other.x,
                )
            }
            pub fn dot(self, other: Self) -> $T {
                self.x * other.x + self.y * other.y + self.z * other.z
            }
            pub fn mag_sq(self) -> $T {
                self.dot(self)
            }
            pub fn array(self) -> [$T; 3] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref(&self) -> &[$T; 3] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref_mut(&mut self) -> &mut [$T; 3] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn to_point(self) -> $Point3<S> {
                unsafe { std::mem::transmute(self) }
            }
        }
        impl<S: Copy> $Vector4<S> {
            pub const ZERO: Self = Self::splat(0 as $T);
            pub const ONE: Self = Self::splat(1 as $T);
            pub const fn new(x: $T, y: $T, z: $T, w: $T) -> Self {
                Self {
                    x,
                    y,
                    z,
                    w,
                    _marker: std::marker::PhantomData {},
                }
            }
            pub const fn splat(x: $T) -> Self {
                Self::new(x, x, x, x)
            }
            pub fn abs(self) -> Self {
                Self::new(self.x.abs(), self.y.abs(), self.z.abs(), self.w.abs())
            }
            pub fn clamp(self, low: Self, high: Self) -> Self {
                Self::new(
                    self.x.clamp(low.x, high.x),
                    self.y.clamp(low.y, high.y),
                    self.z.clamp(low.z, high.z),
                    self.w.clamp(low.w, high.w),
                )
            }
            pub fn min_component(self) -> $T {
                (self.x.min(self.y)).min(self.z.min(self.w))
            }
            pub fn max_component(self) -> $T {
                (self.x.max(self.y)).max(self.z.max(self.w))
            }
            pub fn min_axis(self) -> usize {
                let mut axis = 0;
                let mut min = self.x;
                if self.y < min {
                    min = self.y;
                    axis = 1;
                }
                if self.z < min {
                    min = self.z;
                    axis = 2;
                }
                if self.w < min {
                    axis = 3;
                }
                axis
            }
            pub fn max_axis(self) -> usize {
                let mut axis = 0;
                let mut max = self.x;
                if self.y > max {
                    max = self.y;
                    axis = 1;
                }
                if self.z > max {
                    max = self.z;
                    axis = 2;
                }
                if self.w > max {
                    axis = 3;
                }
                axis
            }
            pub fn min_by_component(self, other: Self) -> Self {
                Self::new(
                    self.x.min(other.x),
                    self.y.min(other.y),
                    self.z.min(other.z),
                    self.w.min(other.w),
                )
            }
            pub fn max_by_component(self, other: Self) -> Self {
                Self::new(
                    self.x.max(other.x),
                    self.y.max(other.y),
                    self.z.max(other.z),
                    self.w.max(other.w),
                )
            }
            pub fn xyz(self) -> $Vector3<S> {
                $Vector3::new(self.x, self.y, self.z)
            }
            pub fn dot(self, other: Self) -> $T {
                self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
            }
            pub fn mag_sq(self) -> $T {
                self.dot(self)
            }
            pub fn array(self) -> [$T; 4] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref(&self) -> &[$T; 4] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref_mut(&mut self) -> &mut [$T; 4] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn to_point(self) -> $Point4<S> {
                unsafe { std::mem::transmute(self) }
            }
        }
        impl<S: Copy> $Point2<S> {
            pub const ZERO: Self = Self::splat(0 as $T);
            pub const ONE: Self = Self::splat(1 as $T);
            pub const fn new(x: $T, y: $T) -> Self {
                Self {
                    x,
                    y,
                    _marker: std::marker::PhantomData {},
                }
            }
            pub const fn splat(x: $T) -> Self {
                Self::new(x, x)
            }
            pub fn abs(self) -> Self {
                Self::new(self.x.abs(), self.y.abs())
            }
            pub fn clamp(self, low: Self, high: Self) -> Self {
                Self::new(self.x.clamp(low.x, high.x), self.y.clamp(low.y, high.y))
            }
            pub fn min_component(self) -> $T {
                self.x.min(self.y)
            }
            pub fn max_component(self) -> $T {
                self.x.max(self.y)
            }
            pub fn min_by_component(self, other: Self) -> Self {
                Self::new(self.x.min(other.x), self.y.min(other.y))
            }
            pub fn max_by_component(self, other: Self) -> Self {
                Self::new(self.x.max(other.x), self.y.max(other.y))
            }
            pub fn extend_z(self, z: $T) -> $Point3<S> {
                $Point3::new(self.x, self.y, z)
            }
            pub fn dot(self, other: Self) -> $T {
                self.x * other.x + self.y * other.y
            }
            pub fn dist_sq(self, other: Self) -> $T {
                let v = self - other;
                v.dot(v)
            }
            pub fn array(self) -> [$T; 2] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref(&self) -> &[$T; 2] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref_mut(&mut self) -> &mut [$T; 2] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn to_vector(self) -> $Vector2<S> {
                unsafe { std::mem::transmute(self) }
            }
        }
        impl<S: Copy> $Point3<S> {
            pub const ZERO: Self = Self::splat(0 as $T);
            pub const ONE: Self = Self::splat(1 as $T);
            pub const fn new(x: $T, y: $T, z: $T) -> Self {
                Self {
                    x,
                    y,
                    z,
                    _marker: std::marker::PhantomData {},
                }
            }
            pub const fn splat(x: $T) -> Self {
                Self::new(x, x, x)
            }
            pub fn abs(self) -> Self {
                Self::new(self.x.abs(), self.y.abs(), self.z.abs())
            }
            pub fn clamp(self, low: Self, high: Self) -> Self {
                Self::new(
                    self.x.clamp(low.x, high.x),
                    self.y.clamp(low.y, high.y),
                    self.z.clamp(low.z, high.z),
                )
            }
            pub fn min_component(self) -> $T {
                self.x.min(self.y).min(self.z)
            }
            pub fn max_component(self) -> $T {
                self.x.max(self.y).max(self.z)
            }
            pub fn min_by_component(self, other: Self) -> Self {
                Self::new(
                    self.x.min(other.x),
                    self.y.min(other.y),
                    self.z.min(other.z),
                )
            }
            pub fn max_by_component(self, other: Self) -> Self {
                Self::new(
                    self.x.max(other.x),
                    self.y.max(other.y),
                    self.z.max(other.z),
                )
            }
            pub fn extend_w(self, w: $T) -> $Point4<S> {
                $Point4::new(self.x, self.y, self.z, w)
            }
            pub fn dot(self, other: Self) -> $T {
                self.x * other.x + self.y * other.y + self.z * other.z
            }
            pub fn dist_sq(self, other: Self) -> $T {
                let v = self - other;
                v.dot(v)
            }
            pub fn array(self) -> [$T; 3] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref(&self) -> &[$T; 3] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref_mut(&mut self) -> &mut [$T; 3] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn to_vector(self) -> $Vector3<S> {
                unsafe { std::mem::transmute(self) }
            }
        }
        impl<S: Copy> $Point4<S> {
            pub const ZERO: Self = Self::splat(0 as $T);
            pub const ONE: Self = Self::splat(1 as $T);
            pub const fn new(x: $T, y: $T, z: $T, w: $T) -> Self {
                Self {
                    x,
                    y,
                    z,
                    w,
                    _marker: std::marker::PhantomData {},
                }
            }
            pub const fn splat(x: $T) -> Self {
                Self::new(x, x, x, x)
            }
            pub fn abs(self) -> Self {
                Self::new(self.x.abs(), self.y.abs(), self.z.abs(), self.w.abs())
            }
            pub fn clamp(self, low: Self, high: Self) -> Self {
                Self::new(
                    self.x.clamp(low.x, high.x),
                    self.y.clamp(low.y, high.y),
                    self.z.clamp(low.z, high.z),
                    self.w.clamp(low.w, high.w),
                )
            }
            pub fn min_component(self) -> $T {
                (self.x.min(self.y)).min(self.z.min(self.w))
            }
            pub fn max_component(self) -> $T {
                (self.x.max(self.y)).max(self.z.max(self.w))
            }
            pub fn min_by_component(self, other: Self) -> Self {
                Self::new(
                    self.x.min(other.x),
                    self.y.min(other.y),
                    self.z.min(other.z),
                    self.w.min(other.w),
                )
            }
            pub fn max_by_component(self, other: Self) -> Self {
                Self::new(
                    self.x.max(other.x),
                    self.y.max(other.y),
                    self.z.max(other.z),
                    self.w.max(other.w),
                )
            }
            pub fn xyz(self) -> $Point3<S> {
                $Point3::new(self.x, self.y, self.z)
            }
            pub fn dot(self, other: Self) -> $T {
                self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
            }
            pub fn dist_sq(self, other: Self) -> $T {
                let v = self - other;
                v.dot(v)
            }
            pub fn array(self) -> [$T; 4] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref(&self) -> &[$T; 4] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn array_ref_mut(&mut self) -> &mut [$T; 4] {
                unsafe { std::mem::transmute(self) }
            }
            pub fn to_vector(self) -> $Vector4<S> {
                unsafe { std::mem::transmute(self) }
            }
        }
        impl<S: Copy> From<[$T; 2]> for $Vector2<S> {
            fn from([x, y]: [$T; 2]) -> Self {
                Self::new(x, y)
            }
        }
        impl<S: Copy> From<[$T; 3]> for $Vector3<S> {
            fn from([x, y, z]: [$T; 3]) -> Self {
                Self::new(x, y, z)
            }
        }
        impl<S: Copy> From<[$T; 4]> for $Vector4<S> {
            fn from([x, y, z, w]: [$T; 4]) -> Self {
                Self::new(x, y, z, w)
            }
        }
        impl<S: Copy> From<[$T; 2]> for $Point2<S> {
            fn from(array: [$T; 2]) -> Self {
                unsafe { std::mem::transmute(array) }
            }
        }
        impl<S: Copy> From<[$T; 3]> for $Point3<S> {
            fn from(array: [$T; 3]) -> Self {
                unsafe { std::mem::transmute(array) }
            }
        }
        impl<S: Copy> From<[$T; 4]> for $Point4<S> {
            fn from(array: [$T; 4]) -> Self {
                unsafe { std::mem::transmute(array) }
            }
        }

        impl<S: Copy> std::ops::Add<Self> for $Vector2<S> {
            type Output = Self;
            fn add(self, other: Self) -> Self::Output {
                Self::new(self.x + other.x, self.y + other.y)
            }
        }
        impl<S: Copy> std::ops::Add<Self> for $Vector3<S> {
            type Output = Self;
            fn add(self, other: Self) -> Self::Output {
                Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
            }
        }
        impl<S: Copy> std::ops::Add<Self> for $Vector4<S> {
            type Output = Self;
            fn add(self, other: Self) -> Self::Output {
                Self::new(
                    self.x + other.x,
                    self.y + other.y,
                    self.z + other.z,
                    self.w + other.w,
                )
            }
        }

        impl<S: Copy> std::ops::AddAssign<Self> for $Vector2<S> {
            fn add_assign(&mut self, other: Self) {
                *self = *self + other
            }
        }
        impl<S: Copy> std::ops::AddAssign<Self> for $Vector3<S> {
            fn add_assign(&mut self, other: Self) {
                *self = *self + other
            }
        }
        impl<S: Copy> std::ops::AddAssign<Self> for $Vector4<S> {
            fn add_assign(&mut self, other: Self) {
                *self = *self + other
            }
        }

        impl<S: Copy> std::ops::Sub<Self> for $Vector2<S> {
            type Output = Self;
            fn sub(self, other: Self) -> Self::Output {
                Self::new(self.x - other.x, self.y - other.y)
            }
        }
        impl<S: Copy> std::ops::Sub<Self> for $Vector3<S> {
            type Output = Self;
            fn sub(self, other: Self) -> Self::Output {
                Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
            }
        }
        impl<S: Copy> std::ops::Sub<Self> for $Vector4<S> {
            type Output = Self;
            fn sub(self, other: Self) -> Self::Output {
                Self::new(
                    self.x - other.x,
                    self.y - other.y,
                    self.z - other.z,
                    self.w - other.w,
                )
            }
        }

        impl<S: Copy> std::ops::SubAssign<Self> for $Vector2<S> {
            fn sub_assign(&mut self, other: Self) {
                *self = *self - other
            }
        }
        impl<S: Copy> std::ops::SubAssign<Self> for $Vector3<S> {
            fn sub_assign(&mut self, other: Self) {
                *self = *self - other
            }
        }
        impl<S: Copy> std::ops::SubAssign<Self> for $Vector4<S> {
            fn sub_assign(&mut self, other: Self) {
                *self = *self - other
            }
        }

        impl<S: Copy> std::ops::Mul<Self> for $Vector2<S> {
            type Output = Self;
            fn mul(self, other: Self) -> Self::Output {
                Self::new(self.x * other.x, self.y * other.y)
            }
        }
        impl<S: Copy> std::ops::Mul<Self> for $Vector3<S> {
            type Output = Self;
            fn mul(self, other: Self) -> Self::Output {
                Self::new(self.x * other.x, self.y * other.y, self.z * other.z)
            }
        }
        impl<S: Copy> std::ops::Mul<Self> for $Vector4<S> {
            type Output = Self;
            fn mul(self, other: Self) -> Self::Output {
                Self::new(
                    self.x * other.x,
                    self.y * other.y,
                    self.z * other.z,
                    self.w * other.w,
                )
            }
        }

        impl<S: Copy> std::ops::MulAssign<Self> for $Vector2<S> {
            fn mul_assign(&mut self, other: Self) {
                *self = *self * other
            }
        }
        impl<S: Copy> std::ops::MulAssign<Self> for $Vector3<S> {
            fn mul_assign(&mut self, other: Self) {
                *self = *self * other
            }
        }
        impl<S: Copy> std::ops::MulAssign<Self> for $Vector4<S> {
            fn mul_assign(&mut self, other: Self) {
                *self = *self * other
            }
        }

        impl<S: Copy> std::ops::Div<Self> for $Vector2<S> {
            type Output = Self;
            fn div(self, other: Self) -> Self::Output {
                Self::new(self.x / other.x, self.y / other.y)
            }
        }
        impl<S: Copy> std::ops::Div<Self> for $Vector3<S> {
            type Output = Self;
            fn div(self, other: Self) -> Self::Output {
                Self::new(self.x / other.x, self.y / other.y, self.z / other.z)
            }
        }
        impl<S: Copy> std::ops::Div<Self> for $Vector4<S> {
            type Output = Self;
            fn div(self, other: Self) -> Self::Output {
                Self::new(
                    self.x / other.x,
                    self.y / other.y,
                    self.z / other.z,
                    self.w / other.w,
                )
            }
        }

        impl<S: Copy> std::ops::DivAssign<Self> for $Vector2<S> {
            fn div_assign(&mut self, other: Self) {
                *self = *self / other
            }
        }
        impl<S: Copy> std::ops::DivAssign<Self> for $Vector3<S> {
            fn div_assign(&mut self, other: Self) {
                *self = *self / other
            }
        }
        impl<S: Copy> std::ops::DivAssign<Self> for $Vector4<S> {
            fn div_assign(&mut self, other: Self) {
                *self = *self / other
            }
        }

        impl<S: Copy> std::ops::Mul<$T> for $Vector2<S> {
            type Output = Self;
            fn mul(self, other: $T) -> Self::Output {
                Self::new(self.x * other, self.y * other)
            }
        }
        impl<S: Copy> std::ops::Mul<$T> for $Vector3<S> {
            type Output = Self;
            fn mul(self, other: $T) -> Self::Output {
                Self::new(self.x * other, self.y * other, self.z * other)
            }
        }
        impl<S: Copy> std::ops::Mul<$T> for $Vector4<S> {
            type Output = Self;
            fn mul(self, other: $T) -> Self::Output {
                Self::new(
                    self.x * other,
                    self.y * other,
                    self.z * other,
                    self.w * other,
                )
            }
        }

        impl<S: Copy> std::ops::MulAssign<$T> for $Vector2<S> {
            fn mul_assign(&mut self, other: $T) {
                *self = *self * other
            }
        }
        impl<S: Copy> std::ops::MulAssign<$T> for $Vector3<S> {
            fn mul_assign(&mut self, other: $T) {
                *self = *self * other
            }
        }
        impl<S: Copy> std::ops::MulAssign<$T> for $Vector4<S> {
            fn mul_assign(&mut self, other: $T) {
                *self = *self * other
            }
        }

        impl<S: Copy> std::ops::Mul<$Vector2<S>> for $T {
            type Output = $Vector2<S>;
            fn mul(self, other: $Vector2<S>) -> Self::Output {
                other * self
            }
        }
        impl<S: Copy> std::ops::Mul<$Vector3<S>> for $T {
            type Output = $Vector3<S>;
            fn mul(self, other: $Vector3<S>) -> Self::Output {
                other * self
            }
        }
        impl<S: Copy> std::ops::Mul<$Vector4<S>> for $T {
            type Output = $Vector4<S>;
            fn mul(self, other: $Vector4<S>) -> Self::Output {
                other * self
            }
        }

        impl<S: Copy> std::ops::Div<$T> for $Vector2<S> {
            type Output = Self;
            fn div(self, other: $T) -> Self::Output {
                Self::new(self.x / other, self.y / other)
            }
        }
        impl<S: Copy> std::ops::Div<$T> for $Vector3<S> {
            type Output = Self;
            fn div(self, other: $T) -> Self::Output {
                Self::new(self.x / other, self.y / other, self.z / other)
            }
        }
        impl<S: Copy> std::ops::Div<$T> for $Vector4<S> {
            type Output = Self;
            fn div(self, other: $T) -> Self::Output {
                Self::new(
                    self.x / other,
                    self.y / other,
                    self.z / other,
                    self.w / other,
                )
            }
        }

        impl<S: Copy> std::ops::DivAssign<$T> for $Vector2<S> {
            fn div_assign(&mut self, other: $T) {
                *self = *self / other
            }
        }
        impl<S: Copy> std::ops::DivAssign<$T> for $Vector3<S> {
            fn div_assign(&mut self, other: $T) {
                *self = *self / other
            }
        }
        impl<S: Copy> std::ops::DivAssign<$T> for $Vector4<S> {
            fn div_assign(&mut self, other: $T) {
                *self = *self / other
            }
        }

        impl<S: Copy> std::ops::Neg for $Vector2<S> {
            type Output = Self;
            fn neg(self) -> Self {
                -1 as $T * self
            }
        }
        impl<S: Copy> std::ops::Neg for $Vector3<S> {
            type Output = Self;
            fn neg(self) -> Self {
                -1 as $T * self
            }
        }
        impl<S: Copy> std::ops::Neg for $Vector4<S> {
            type Output = Self;
            fn neg(self) -> Self {
                -1 as $T * self
            }
        }

        impl<S: Copy> std::ops::Index<usize> for $Vector2<S> {
            type Output = $T;
            fn index(&self, index: usize) -> &$T {
                self.array_ref().index(index)
            }
        }
        impl<S: Copy> std::ops::IndexMut<usize> for $Vector2<S> {
            fn index_mut(&mut self, index: usize) -> &mut $T {
                self.array_ref_mut().index_mut(index)
            }
        }
        impl<S: Copy> std::ops::Index<usize> for $Vector3<S> {
            type Output = $T;
            fn index(&self, index: usize) -> &$T {
                self.array_ref().index(index)
            }
        }
        impl<S: Copy> std::ops::IndexMut<usize> for $Vector3<S> {
            fn index_mut(&mut self, index: usize) -> &mut $T {
                self.array_ref_mut().index_mut(index)
            }
        }
        impl<S: Copy> std::ops::Index<usize> for $Vector4<S> {
            type Output = $T;
            fn index(&self, index: usize) -> &$T {
                self.array_ref().index(index)
            }
        }
        impl<S: Copy> std::ops::IndexMut<usize> for $Vector4<S> {
            fn index_mut(&mut self, index: usize) -> &mut $T {
                self.array_ref_mut().index_mut(index)
            }
        }

        impl<S: Copy> std::ops::Add<$Vector2<S>> for $Point2<S> {
            type Output = Self;
            fn add(self, other: $Vector2<S>) -> Self::Output {
                (self.to_vector() + other).to_point()
            }
        }

        impl<S: Copy> std::ops::Sub<$Vector2<S>> for $Point2<S> {
            type Output = Self;
            fn sub(self, other: $Vector2<S>) -> Self::Output {
                (self.to_vector() - other).to_point()
            }
        }

        impl<S: Copy> std::ops::Sub<$Point2<S>> for $Point2<S> {
            type Output = $Vector2<S>;
            fn sub(self, other: Self) -> Self::Output {
                self.to_vector() - other.to_vector()
            }
        }

        impl<S: Copy> std::ops::Add<$Vector3<S>> for $Point3<S> {
            type Output = Self;
            fn add(self, other: $Vector3<S>) -> Self::Output {
                (self.to_vector() + other).to_point()
            }
        }

        impl<S: Copy> std::ops::Sub<$Vector3<S>> for $Point3<S> {
            type Output = Self;
            fn sub(self, other: $Vector3<S>) -> Self::Output {
                (self.to_vector() - other).to_point()
            }
        }

        impl<S: Copy> std::ops::Sub<$Point3<S>> for $Point3<S> {
            type Output = $Vector3<S>;
            fn sub(self, other: Self) -> Self::Output {
                self.to_vector() - other.to_vector()
            }
        }

        impl<S: Copy> std::ops::Add<$Vector4<S>> for $Point4<S> {
            type Output = Self;
            fn add(self, other: $Vector4<S>) -> Self::Output {
                (self.to_vector() + other).to_point()
            }
        }

        impl<S: Copy> std::ops::Sub<$Vector4<S>> for $Point4<S> {
            type Output = Self;
            fn sub(self, other: $Vector4<S>) -> Self::Output {
                (self.to_vector() - other).to_point()
            }
        }

        impl<S: Copy> std::ops::Sub<$Point4<S>> for $Point4<S> {
            type Output = $Vector4<S>;
            fn sub(self, other: Self) -> Self::Output {
                self.to_vector() - other.to_vector()
            }
        }

        impl<S: Copy> std::ops::Mul<$T> for $Point2<S> {
            type Output = $Point2<S>;
            fn mul(self, other: $T) -> Self::Output {
                Self::new(self.x * other, self.y * other)
            }
        }
        impl<S: Copy> std::ops::Mul<$T> for $Point3<S> {
            type Output = $Point3<S>;
            fn mul(self, other: $T) -> Self::Output {
                Self::new(self.x * other, self.y * other, self.z * other)
            }
        }
        impl<S: Copy> std::ops::Mul<$T> for $Point4<S> {
            type Output = $Point4<S>;
            fn mul(self, other: $T) -> Self::Output {
                Self::new(
                    self.x * other,
                    self.y * other,
                    self.z * other,
                    self.w * other,
                )
            }
        }

        impl<S: Copy> std::ops::AddAssign<$Vector2<S>> for $Point2<S> {
            fn add_assign(&mut self, other: $Vector2<S>) {
                *self = *self + other;
            }
        }
        impl<S: Copy> std::ops::AddAssign<$Vector3<S>> for $Point3<S> {
            fn add_assign(&mut self, other: $Vector3<S>) {
                *self = *self + other;
            }
        }
        impl<S: Copy> std::ops::AddAssign<$Vector4<S>> for $Point4<S> {
            fn add_assign(&mut self, other: $Vector4<S>) {
                *self = *self + other;
            }
        }

        impl<S: Copy> std::ops::MulAssign<$T> for $Point2<S> {
            fn mul_assign(&mut self, other: $T) {
                *self = *self * other;
            }
        }
        impl<S: Copy> std::ops::MulAssign<$T> for $Point3<S> {
            fn mul_assign(&mut self, other: $T) {
                *self = *self * other;
            }
        }
        impl<S: Copy> std::ops::MulAssign<$T> for $Point4<S> {
            fn mul_assign(&mut self, other: $T) {
                *self = *self * other;
            }
        }

        impl<S: Copy> std::ops::Div<$T> for $Point2<S> {
            type Output = $Point2<S>;
            fn div(self, other: $T) -> Self::Output {
                Self::new(self.x / other, self.y / other)
            }
        }
        impl<S: Copy> std::ops::Div<$T> for $Point3<S> {
            type Output = $Point3<S>;
            fn div(self, other: $T) -> Self::Output {
                Self::new(self.x / other, self.y / other, self.z / other)
            }
        }
        impl<S: Copy> std::ops::Div<$T> for $Point4<S> {
            type Output = $Point4<S>;
            fn div(self, other: $T) -> Self::Output {
                Self::new(
                    self.x / other,
                    self.y / other,
                    self.z / other,
                    self.w / other,
                )
            }
        }
        impl<S: Copy> std::ops::Mul<$Point2<S>> for $T {
            type Output = $Point2<S>;
            fn mul(self, other: $Point2<S>) -> Self::Output {
                other * self
            }
        }
        impl<S: Copy> std::ops::Mul<$Point3<S>> for $T {
            type Output = $Point3<S>;
            fn mul(self, other: $Point3<S>) -> Self::Output {
                other * self
            }
        }
        impl<S: Copy> std::ops::Mul<$Point4<S>> for $T {
            type Output = $Point4<S>;
            fn mul(self, other: $Point4<S>) -> Self::Output {
                other * self
            }
        }

        impl<S: Copy> std::ops::Index<usize> for $Point2<S> {
            type Output = $T;
            fn index(&self, index: usize) -> &$T {
                self.array_ref().index(index)
            }
        }
        impl<S: Copy> std::ops::IndexMut<usize> for $Point2<S> {
            fn index_mut(&mut self, index: usize) -> &mut $T {
                self.array_ref_mut().index_mut(index)
            }
        }
        impl<S: Copy> std::ops::Index<usize> for $Point3<S> {
            type Output = $T;
            fn index(&self, index: usize) -> &$T {
                self.array_ref().index(index)
            }
        }
        impl<S: Copy> std::ops::IndexMut<usize> for $Point3<S> {
            fn index_mut(&mut self, index: usize) -> &mut $T {
                self.array_ref_mut().index_mut(index)
            }
        }
        impl<S: Copy> std::ops::Index<usize> for $Point4<S> {
            type Output = $T;
            fn index(&self, index: usize) -> &$T {
                self.array_ref().index(index)
            }
        }
        impl<S: Copy> std::ops::IndexMut<usize> for $Point4<S> {
            fn index_mut(&mut self, index: usize) -> &mut $T {
                self.array_ref_mut().index_mut(index)
            }
        }
    };
}

macro_rules! impl_float {
    (
        $Vector2:ident,$Vector3:ident,$Vector4:ident,$Point2:ident,$Point3:ident,$Point4:ident,$T:ident
    ) => {
        impl<S: Copy> $Vector2<S> {
            pub fn mag(self) -> $T {
                self.mag_sq().sqrt()
            }
            pub fn normalized(self) -> Self {
                self * self.mag().recip()
            }
        }
        impl<S: Copy> $Vector3<S> {
            pub fn mag(self) -> $T {
                self.mag_sq().sqrt()
            }
            pub fn normalized(self) -> Self {
                self * self.mag().recip()
            }
        }
        impl<S: Copy> $Vector4<S> {
            pub fn mag(self) -> $T {
                self.mag_sq().sqrt()
            }
            pub fn normalized(self) -> Self {
                self * self.mag().recip()
            }
        }

        impl<S: Copy> $Point2<S> {
            pub fn dist(self, other: Self) -> $T {
                self.dist_sq(other).sqrt()
            }
        }
        impl<S: Copy> $Point3<S> {
            pub fn dist(self, other: Self) -> $T {
                self.dist_sq(other).sqrt()
            }
        }
        impl<S: Copy> $Point4<S> {
            pub fn dist(self, other: Self) -> $T {
                self.dist_sq(other).sqrt()
            }
        }

        impl<S: Copy> std::ops::Mul<$Vector2<S>> for $Point2<S> {
            type Output = Self;
            fn mul(self, rhs: $Vector2<S>) -> Self {
                (self.to_vector() * rhs).to_point()
            }
        }
        impl<S: Copy> std::ops::Mul<$Vector3<S>> for $Point3<S> {
            type Output = Self;
            fn mul(self, rhs: $Vector3<S>) -> Self {
                (self.to_vector() * rhs).to_point()
            }
        }
        impl<S: Copy> std::ops::Mul<$Vector4<S>> for $Point4<S> {
            type Output = Self;
            fn mul(self, rhs: $Vector4<S>) -> Self {
                (self.to_vector() * rhs).to_point()
            }
        }

        impl<S: Copy> std::ops::Mul<$Point2<S>> for $Vector2<S> {
            type Output = $Point2<S>;
            fn mul(self, rhs: $Point2<S>) -> $Point2<S> {
                (self * rhs.to_vector()).to_point()
            }
        }
        impl<S: Copy> std::ops::Mul<$Point3<S>> for $Vector3<S> {
            type Output = $Point3<S>;
            fn mul(self, rhs: $Point3<S>) -> $Point3<S> {
                (self * rhs.to_vector()).to_point()
            }
        }
        impl<S: Copy> std::ops::Mul<$Point4<S>> for $Vector4<S> {
            type Output = $Point4<S>;
            fn mul(self, rhs: $Point4<S>) -> $Point4<S> {
                (self * rhs.to_vector()).to_point()
            }
        }
    };
}

macro_rules! impl_from_le_bytes {
    (
        $Vector2:ident,$Vector3:ident,$Vector4:ident,$Point2:ident,$Point3:ident,$Point4:ident,$T:ident
    ) => {
        impl<S: Copy> $Vector2<S> {
            pub fn from_le_bytes([a, b, c, d, e, f, g, h]: [u8; { 2 * size_of::<$T>() }]) -> Self {
                [
                    f32::from_le_bytes([a, b, c, d]),
                    f32::from_le_bytes([e, f, g, h]),
                ]
                .into()
            }
        }
        impl<S: Copy> $Vector3<S> {
            pub fn from_le_bytes(
                [a, b, c, d, e, f, g, h, i, j, k, l]: [u8; { 3 * size_of::<$T>() }],
            ) -> Self {
                [
                    f32::from_le_bytes([a, b, c, d]),
                    f32::from_le_bytes([e, f, g, h]),
                    f32::from_le_bytes([i, j, k, l]),
                ]
                .into()
            }
        }
        impl<S: Copy> $Vector4<S> {
            pub fn from_le_bytes(
                [a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p]: [u8; { 4 * size_of::<$T>() }],
            ) -> Self {
                [
                    f32::from_le_bytes([a, b, c, d]),
                    f32::from_le_bytes([e, f, g, h]),
                    f32::from_le_bytes([i, j, k, l]),
                    f32::from_le_bytes([m, n, o, p]),
                ]
                .into()
            }
        }
        impl<S: Copy> $Point2<S> {
            pub fn from_le_bytes([a, b, c, d, e, f, g, h]: [u8; { 2 * size_of::<$T>() }]) -> Self {
                [
                    f32::from_le_bytes([a, b, c, d]),
                    f32::from_le_bytes([e, f, g, h]),
                ]
                .into()
            }
        }
        impl<S: Copy> $Point3<S> {
            pub fn from_le_bytes(
                [a, b, c, d, e, f, g, h, i, j, k, l]: [u8; { 3 * size_of::<$T>() }],
            ) -> Self {
                [
                    f32::from_le_bytes([a, b, c, d]),
                    f32::from_le_bytes([e, f, g, h]),
                    f32::from_le_bytes([i, j, k, l]),
                ]
                .into()
            }
        }
        impl<S: Copy> $Point4<S> {
            pub fn from_le_bytes(
                [a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p]: [u8; { 4 * size_of::<$T>() }],
            ) -> Self {
                [
                    f32::from_le_bytes([a, b, c, d]),
                    f32::from_le_bytes([e, f, g, h]),
                    f32::from_le_bytes([i, j, k, l]),
                    f32::from_le_bytes([m, n, o, p]),
                ]
                .into()
            }
        }
    };
}

macro_rules! impl_as_f64 {
    (
        $Vector2:ident,$Vector3:ident,$Vector4:ident,$Point2:ident,$Point3:ident,$Point4:ident
    ) => {
        impl<S: Copy> $Vector2<S> {
            pub fn as_f64(self) -> Vector2f64<S> {
                [self.x as f64, self.y as f64].into()
            }
        }
        impl<S: Copy> $Vector3<S> {
            pub fn as_f64(self) -> Vector3f64<S> {
                [self.x as f64, self.y as f64, self.z as f64].into()
            }
        }
        impl<S: Copy> $Vector4<S> {
            pub fn as_f64(self) -> Vector4f64<S> {
                [self.x as f64, self.y as f64, self.z as f64, self.w as f64].into()
            }
        }
        impl<S: Copy> $Point2<S> {
            pub fn as_f64(self) -> Point2f64<S> {
                [self.x as f64, self.y as f64].into()
            }
        }
        impl<S: Copy> $Point3<S> {
            pub fn as_f64(self) -> Point3f64<S> {
                [self.x as f64, self.y as f64, self.z as f64].into()
            }
        }
        impl<S: Copy> $Point4<S> {
            pub fn as_f64(self) -> Point4f64<S> {
                [self.x as f64, self.y as f64, self.z as f64, self.w as f64].into()
            }
        }
    };
}

macro_rules! impl_as_f32 {
    (
        $Vector2:ident,$Vector3:ident,$Vector4:ident,$Point2:ident,$Point3:ident,$Point4:ident
    ) => {
        impl<S: Copy> $Vector2<S> {
            pub fn as_f32(self) -> Vector2f32<S> {
                [self.x as f32, self.y as f32].into()
            }
        }
        impl<S: Copy> $Vector3<S> {
            pub fn as_f32(self) -> Vector3f32<S> {
                [self.x as f32, self.y as f32, self.z as f32].into()
            }
        }
        impl<S: Copy> $Vector4<S> {
            pub fn as_f32(self) -> Vector4f32<S> {
                [self.x as f32, self.y as f32, self.z as f32, self.w as f32].into()
            }
        }
        impl<S: Copy> $Point2<S> {
            pub fn as_f32(self) -> Point2f32<S> {
                [self.x as f32, self.y as f32].into()
            }
        }
        impl<S: Copy> $Point3<S> {
            pub fn as_f32(self) -> Point3f32<S> {
                [self.x as f32, self.y as f32, self.z as f32].into()
            }
        }
        impl<S: Copy> $Point4<S> {
            pub fn as_f32(self) -> Point4f32<S> {
                [self.x as f32, self.y as f32, self.z as f32, self.w as f32].into()
            }
        }
    };
}

create_structs_and_impl_common!(
    Vector2f32, Vector3f32, Vector4f32, Point2f32, Point3f32, Point4f32, f32
);
create_structs_and_impl_common!(
    Vector2f64, Vector3f64, Vector4f64, Point2f64, Point3f64, Point4f64, f64
);

impl_from_le_bytes!(
    Vector2f32, Vector3f32, Vector4f32, Point2f32, Point3f32, Point4f32, f32
);

impl_float!(
    Vector2f32, Vector3f32, Vector4f32, Point2f32, Point3f32, Point4f32, f32
);
impl_float!(
    Vector2f64, Vector3f64, Vector4f64, Point2f64, Point3f64, Point4f64, f64
);

impl_as_f64!(
    Vector2f32, Vector3f32, Vector4f32, Point2f32, Point3f32, Point4f32
);

impl_as_f32!(
    Vector2f64, Vector3f64, Vector4f64, Point2f64, Point3f64, Point4f64
);
