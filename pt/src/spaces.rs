use math::*;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Worldspace;
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Objectspace;

pub trait Localspace {}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Rasterspace;
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Cameraspace;
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Shadingspace;
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Lightspace;

impl Localspace for Cameraspace {}
impl Localspace for Shadingspace {}
impl Localspace for Lightspace {}

pub type Point2 = Point2f32<()>;
pub type Point3 = Point3f32<()>;
pub type Point4 = Point4f32<()>;
pub type Vec2 = Vector2f32<()>;
pub type Vec3 = Vector3f32<()>;
pub type Vec4 = Vector4f32<()>;

pub type Point2World = Point2f32<Worldspace>;
pub type Point3World = Point3f32<Worldspace>;
pub type Point4World = Point4f32<Worldspace>;
pub type Vec2World = Vector2f32<Worldspace>;
pub type Vec3World = Vector3f32<Worldspace>;
pub type Vec4World = Vector4f32<Worldspace>;

pub type Point2Object = Point2f32<Objectspace>;
pub type Point3Object = Point3f32<Objectspace>;
pub type PointObjectd = Point4f32<Objectspace>;
pub type Vec2Object = Vector2f32<Objectspace>;
pub type Vec3Object = Vector3f32<Objectspace>;
pub type Vec4Object = Vector4f32<Objectspace>;

pub type Point2Camera = Point2f32<Cameraspace>;
pub type Point3Camera = Point3f32<Cameraspace>;
pub type Point4Camera = Point4f32<Cameraspace>;
pub type Vec2Camera = Vector2f32<Cameraspace>;
pub type Vec3Camera = Vector3f32<Cameraspace>;
pub type Vec4Camera = Vector4f32<Cameraspace>;

pub type Point2Raster = Point2f32<Rasterspace>;
pub type Point3Raster = Point3f32<Rasterspace>;
pub type Point4Raster = Point4f32<Rasterspace>;
pub type Vec2Raster = Vector2f32<Rasterspace>;
pub type Vec3Raster = Vector3f32<Rasterspace>;
pub type Vec4Raster = Vector4f32<Rasterspace>;

pub type Point2Shading = Point2f32<Shadingspace>;
pub type Point3Shading = Point3f32<Shadingspace>;
pub type Point4Shading = Point4f32<Shadingspace>;
pub type Vec2Shading = Vector2f32<Shadingspace>;
pub type Vec3Shading = Vector3f32<Shadingspace>;
pub type Vec4Shading = Vector4f32<Shadingspace>;

pub type Point2Light = Point2f32<Lightspace>;
pub type Point3Light = Point3f32<Lightspace>;
pub type Point4Light = Point4f32<Lightspace>;
pub type Vec2Light = Vector2f32<Lightspace>;
pub type Vec3Light = Vector3f32<Lightspace>;
pub type Vec4Light = Vector4f32<Lightspace>;

pub type Bounds3World = Bounds3f32<Worldspace>;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct RGBspace;
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct XYZspace;
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Yxyspace;

pub type RGBf32 = Vector3f32<RGBspace>;
pub type XYZf32 = Vector3f32<XYZspace>;
pub type RGBf64 = Vector3f64<RGBspace>;
pub type XYZf64 = Vector3f64<XYZspace>;
pub type Yxyf32 = Vector3f32<Yxyspace>;
pub type Yxyf64 = Vector3f64<Yxyspace>;
