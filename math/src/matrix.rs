use crate::vector::*;

macro_rules! create_structs_anf_impl_common_mat4 {
    ($Matrix4:ident,$Vector4:ident,$Vector3:ident,$Point3:ident,$T:ident) => {
        #[repr(C)]
        #[derive(Clone, Copy, PartialEq, PartialOrd, Default, Debug)]
        pub struct $Matrix4<From, To> {
            pub cols: [$Vector4<From>; 4],
            _marker: std::marker::PhantomData<To>,
        }
        impl<From: Copy, To: Copy> std::ops::Mul<$Vector4<From>> for $Matrix4<From, To> {
            type Output = $Vector4<To>;
            fn mul(self, rhs: $Vector4<From>) -> $Vector4<To> {
                let t = self.transposed();
                $Vector4::new(
                    t.cols[0].dot(rhs),
                    t.cols[1].dot(rhs),
                    t.cols[2].dot(rhs),
                    t.cols[3].dot(rhs),
                )
            }
        }
        impl<From: Copy, To: Copy> std::ops::Mul<$Vector3<From>> for $Matrix4<From, To> {
            type Output = $Vector3<To>;
            fn mul(self, rhs: $Vector3<From>) -> $Vector3<To> {
                let t = self.transposed();
                $Vector4::new(
                    t.cols[0].dot(rhs.extend_w(0.0)),
                    t.cols[1].dot(rhs.extend_w(0.0)),
                    t.cols[2].dot(rhs.extend_w(0.0)),
                    t.cols[3].dot(rhs.extend_w(0.0)),
                )
                .xyz()
            }
        }
        impl<From: Copy, To: Copy> std::ops::Mul<$Point3<From>> for $Matrix4<From, To> {
            type Output = $Point3<To>;
            fn mul(self, rhs: $Point3<From>) -> $Point3<To> {
                let t = self.transposed();
                $Vector4::new(
                    t.cols[0].dot(rhs.extend_w(1.0).to_vector()),
                    t.cols[1].dot(rhs.extend_w(1.0).to_vector()),
                    t.cols[2].dot(rhs.extend_w(1.0).to_vector()),
                    t.cols[3].dot(rhs.extend_w(1.0).to_vector()),
                )
                .xyz()
                .to_point()
            }
        }

        impl<From: Copy, To: Copy> std::ops::Mul<Self> for $Matrix4<From, To> {
            type Output = Self;
            #[inline]
            fn mul(self, rhs: Self) -> Self {
                let sa = self.cols[0];
                let sb = self.cols[1];
                let sc = self.cols[2];
                let sd = self.cols[3];
                let oa = rhs.cols[0];
                let ob = rhs.cols[1];
                let oc = rhs.cols[2];
                let od = rhs.cols[3];
                Self::from_cols([
                    $Vector4::new(
                        (sa.x * oa.x) + (sb.x * oa.y) + (sc.x * oa.z) + (sd.x * oa.w),
                        (sa.y * oa.x) + (sb.y * oa.y) + (sc.y * oa.z) + (sd.y * oa.w),
                        (sa.z * oa.x) + (sb.z * oa.y) + (sc.z * oa.z) + (sd.z * oa.w),
                        (sa.w * oa.x) + (sb.w * oa.y) + (sc.w * oa.z) + (sd.w * oa.w),
                    ),
                    $Vector4::new(
                        (sa.x * ob.x) + (sb.x * ob.y) + (sc.x * ob.z) + (sd.x * ob.w),
                        (sa.y * ob.x) + (sb.y * ob.y) + (sc.y * ob.z) + (sd.y * ob.w),
                        (sa.z * ob.x) + (sb.z * ob.y) + (sc.z * ob.z) + (sd.z * ob.w),
                        (sa.w * ob.x) + (sb.w * ob.y) + (sc.w * ob.z) + (sd.w * ob.w),
                    ),
                    $Vector4::new(
                        (sa.x * oc.x) + (sb.x * oc.y) + (sc.x * oc.z) + (sd.x * oc.w),
                        (sa.y * oc.x) + (sb.y * oc.y) + (sc.y * oc.z) + (sd.y * oc.w),
                        (sa.z * oc.x) + (sb.z * oc.y) + (sc.z * oc.z) + (sd.z * oc.w),
                        (sa.w * oc.x) + (sb.w * oc.y) + (sc.w * oc.z) + (sd.w * oc.w),
                    ),
                    $Vector4::new(
                        (sa.x * od.x) + (sb.x * od.y) + (sc.x * od.z) + (sd.x * od.w),
                        (sa.y * od.x) + (sb.y * od.y) + (sc.y * od.z) + (sd.y * od.w),
                        (sa.z * od.x) + (sb.z * od.y) + (sc.z * od.z) + (sd.z * od.w),
                        (sa.w * od.x) + (sb.w * od.y) + (sc.w * od.z) + (sd.w * od.w),
                    ),
                ])
            }
        }

        impl<From: Copy, To: Copy> $Matrix4<From, To> {
            pub fn change_spaces<NewFrom: Copy, NewTo: Copy>(self) -> $Matrix4<NewFrom, NewTo> {
                $Matrix4::from_cols([
                    self.cols[0].array().into(),
                    self.cols[1].array().into(),
                    self.cols[2].array().into(),
                    self.cols[3].array().into(),
                ])
            }
            pub fn identity() -> Self {
                Self::from_cols([
                    [1 as $T, 0 as $T, 0 as $T, 0 as $T].into(),
                    [0 as $T, 1 as $T, 0 as $T, 0 as $T].into(),
                    [0 as $T, 0 as $T, 1 as $T, 0 as $T].into(),
                    [0 as $T, 0 as $T, 0 as $T, 1 as $T].into(),
                ])
            }
            pub fn adjugate(self) -> Self {
                let [
                    [m00, m01, m02, m03],
                    [m10, m11, m12, m13],
                    [m20, m21, m22, m23],
                    [m30, m31, m32, m33],
                ] = self.as_arrays();

                let coef00 = (m22 * m33) - (m32 * m23);
                let coef02 = (m12 * m33) - (m32 * m13);
                let coef03 = (m12 * m23) - (m22 * m13);

                let coef04 = (m21 * m33) - (m31 * m23);
                let coef06 = (m11 * m33) - (m31 * m13);
                let coef07 = (m11 * m23) - (m21 * m13);

                let coef08 = (m21 * m32) - (m31 * m22);
                let coef10 = (m11 * m32) - (m31 * m12);
                let coef11 = (m11 * m22) - (m21 * m12);

                let coef12 = (m20 * m33) - (m30 * m23);
                let coef14 = (m10 * m33) - (m30 * m13);
                let coef15 = (m10 * m23) - (m20 * m13);

                let coef16 = (m20 * m32) - (m30 * m22);
                let coef18 = (m10 * m32) - (m30 * m12);
                let coef19 = (m10 * m22) - (m20 * m12);

                let coef20 = (m20 * m31) - (m30 * m21);
                let coef22 = (m10 * m31) - (m30 * m11);
                let coef23 = (m10 * m21) - (m20 * m11);

                let fac0 = $Vector4::new(coef00, coef00, coef02, coef03);
                let fac1 = $Vector4::new(coef04, coef04, coef06, coef07);
                let fac2 = $Vector4::new(coef08, coef08, coef10, coef11);
                let fac3 = $Vector4::new(coef12, coef12, coef14, coef15);
                let fac4 = $Vector4::new(coef16, coef16, coef18, coef19);
                let fac5 = $Vector4::new(coef20, coef20, coef22, coef23);

                let vec0 = $Vector4::new(m10, m00, m00, m00);
                let vec1 = $Vector4::new(m11, m01, m01, m01);
                let vec2 = $Vector4::new(m12, m02, m02, m02);
                let vec3 = $Vector4::new(m13, m03, m03, m03);

                let inv0 = (vec1 * fac0) - (vec2 * fac1) + (vec3 * fac2);
                let inv1 = (vec0 * fac0) - (vec2 * fac3) + (vec3 * fac4);
                let inv2 = (vec0 * fac1) - (vec1 * fac3) + (vec3 * fac5);
                let inv3 = (vec0 * fac2) - (vec1 * fac4) + (vec2 * fac5);

                let sign_a = $Vector4::new(1.0, -1.0, 1.0, -1.0);
                let sign_b = $Vector4::new(-1.0, 1.0, -1.0, 1.0);

                Self {
                    cols: [inv0 * sign_a, inv1 * sign_b, inv2 * sign_a, inv3 * sign_b],
                    _marker: Default::default(),
                }
            }
            pub fn inverse(self) -> $Matrix4<To, From> {
                let ad = self.adjugate();
                let row0 = $Vector4::new(ad.cols[0].x, ad.cols[1].x, ad.cols[2].x, ad.cols[3].x);
                let dot0 = self.cols[0] * row0;
                let dot1 = dot0.x + dot0.y + dot0.z + dot0.w;
                let recip = 1.0 / dot1;
                $Matrix4::from_cols([
                    (ad.cols[0] * recip).array().into(),
                    (ad.cols[1] * recip).array().into(),
                    (ad.cols[2] * recip).array().into(),
                    (ad.cols[3] * recip).array().into(),
                ])
            }
            pub fn from_rotation(axis: $Vector3<From>, angle: f32) -> Self {
                let (sin, cos) = angle.sin_cos();
                let mul = 1 as $T - cos;

                let x_sin = axis.x * sin;
                let y_sin = axis.y * sin;
                let z_sin = axis.z * sin;

                let xy_mul = axis.x * axis.y * mul;
                let xz_mul = axis.x * axis.z * mul;
                let yz_mul = axis.y * axis.z * mul;

                let m00 = (axis.x * axis.x).mul_add(mul, cos);
                let m10 = xy_mul + z_sin;
                let m20 = xz_mul - y_sin;
                let m01 = xy_mul - z_sin;
                let m11 = (axis.y * axis.y).mul_add(mul, cos);
                let m21 = yz_mul + x_sin;
                let m02 = xz_mul + y_sin;
                let m12 = yz_mul - x_sin;
                let m22 = (axis.z * axis.z).mul_add(mul, cos);
                Self::from_cols([
                    $Vector4::new(m00, m10, m20, 0 as $T),
                    $Vector4::new(m01, m11, m21, 0 as $T),
                    $Vector4::new(m02, m12, m22, 0 as $T),
                    $Vector4::new(0 as $T, 0 as $T, 0 as $T, 1 as $T),
                ])
            }
            pub fn from_translation(v: $Vector3<To>) -> Self {
                Self::from_cols([
                    $Vector4::new(1 as $T, 0 as $T, 0 as $T, 0 as $T),
                    $Vector4::new(0 as $T, 1 as $T, 0 as $T, 0 as $T),
                    $Vector4::new(0 as $T, 0 as $T, 1 as $T, 0 as $T),
                    $Vector4::new(v.x, v.y, v.z, 1 as $T),
                ])
            }
            pub fn from_scale(v: $Vector3<To>) -> Self {
                Self::from_cols([
                    $Vector4::new(v.x, 0 as $T, 0 as $T, 0 as $T),
                    $Vector4::new(0 as $T, v.y, 0 as $T, 0 as $T),
                    $Vector4::new(0 as $T, 0 as $T, v.z, 0 as $T),
                    $Vector4::new(0 as $T, 0 as $T, 0 as $T, 1 as $T),
                ])
            }
            pub fn from_cols(cols: [$Vector4<From>; 4]) -> Self {
                Self {
                    cols,
                    _marker: Default::default(),
                }
            }
            pub fn from_cols_components(cols: [$T; 16]) -> Self {
                Self::from_cols([
                    [cols[0], cols[1], cols[2], cols[3]].into(),
                    [cols[4], cols[5], cols[6], cols[7]].into(),
                    [cols[8], cols[9], cols[10], cols[11]].into(),
                    [cols[12], cols[13], cols[14], cols[15]].into(),
                ])
            }
            pub fn from_rows(rows: [$Vector4<From>; 4]) -> Self {
                Self {
                    cols: rows,
                    _marker: Default::default(),
                }
                .transposed()
            }
            pub fn from_rows_components(rows: [$T; 16]) -> Self {
                Self::from_rows([
                    [rows[0], rows[1], rows[2], rows[3]].into(),
                    [rows[4], rows[5], rows[6], rows[7]].into(),
                    [rows[8], rows[9], rows[10], rows[11]].into(),
                    [rows[12], rows[13], rows[14], rows[15]].into(),
                ])
            }
            pub fn as_arrays(self) -> [[$T; 4]; 4] {
                let Self {
                    cols:
                        [
                            $Vector4 {
                                x: m00,
                                y: m01,
                                z: m02,
                                w: m03,
                                ..
                            },
                            $Vector4 {
                                x: m10,
                                y: m11,
                                z: m12,
                                w: m13,
                                ..
                            },
                            $Vector4 {
                                x: m20,
                                y: m21,
                                z: m22,
                                w: m23,
                                ..
                            },
                            $Vector4 {
                                x: m30,
                                y: m31,
                                z: m32,
                                w: m33,
                                ..
                            },
                        ],
                    ..
                } = self;
                [
                    [m00, m01, m02, m03],
                    [m10, m11, m12, m13],
                    [m20, m21, m22, m23],
                    [m30, m31, m32, m33],
                ]
            }
            pub fn transposed(self) -> Self {
                let [
                    [m00, m01, m02, m03],
                    [m10, m11, m12, m13],
                    [m20, m21, m22, m23],
                    [m30, m31, m32, m33],
                ] = self.as_arrays();
                Self::from_cols([
                    $Vector4::new(m00, m10, m20, m30),
                    $Vector4::new(m01, m11, m21, m31),
                    $Vector4::new(m02, m12, m22, m32),
                    $Vector4::new(m03, m13, m23, m33),
                ])
            }
        }
    };
}

create_structs_anf_impl_common_mat4!(Matrix4f32, Vector4f32, Vector3f32, Point3f32, f32);
