use crate::bsdfs::*;
use crate::bvh::{self, Bvh, BvhIntersection, CompactTriangle};
use crate::geom::*;
use crate::io_bridge;
use crate::lighting::*;
use crate::spaces::*;
use crate::spectrum::*;

use math::*;

pub struct Scene {
    bvh: Bvh,
    envmap: Option<Envmap>,
    normals: Box<[Box<[Vec3World]>]>,
    material_indices: Box<[Option<usize>]>,
    materials: Box<[Material]>,
    emissions: Box<[Spectrum]>,
    emissive_triangles: Box<[CompactTriangle]>,
}

pub enum Material {
    Lambertian {
        albedo: Spectrum,
    },
    Dielectric {
        eta: Spectrum,
    },
    Conductor {
        roughness: f32,
        eta: Spectrum,
        k: Spectrum,
    },
}

#[derive(Debug)]
pub struct SceneInteraction {
    pub bvh_isect: BvhIntersection,
    pub shading_frame: ONB3f32<Worldspace, Shadingspace>,
    pub geometric_normal: Vec3World,
    pub bounds: Bounds3World,
}

impl Default for SceneInteraction {
    fn default() -> Self {
        Self {
            bvh_isect: BvhIntersection {
                tri: Default::default(),
                isect: Default::default(),
            },
            shading_frame: ONB3f32 {
                x: Vector3f32::ZERO,
                y: Vector3f32::ZERO,
                z: Vector3f32::ZERO,
                _marker: std::marker::PhantomData {},
            },
            bounds: Bounds3f32 {
                min: Point3f32::ZERO,
                max: Point3f32::ZERO,
            },
            geometric_normal: Default::default(),
        }
    }
}

impl SceneInteraction {
    pub fn p(&self) -> Point3World {
        self.bounds.centroid()
    }
    pub fn ng(&self) -> Vec3World {
        self.geometric_normal
    }
    pub fn ns(&self) -> Vec3World {
        self.shading_frame.z
    }
    pub fn spawn_ray(&self, d: Vec3World) -> Ray {
        let o = offset_ray_origin(self.bounds, self.ng(), d);
        Ray { o, d }
    }

    pub fn bsdf_eval_ctx(&self) -> BSDFEvalCtx {
        BSDFEvalCtx {
            geometric_normal: self.ng(),
            shading_frame: self.shading_frame,
        }
    }
}

impl Scene {
    pub fn new(
        meshes: Box<[io_bridge::Mesh]>,
        materials: Box<[io_bridge::Material]>,
        envmap: Option<Envmap>,
    ) -> Self {
        let mut emissions = Vec::new();

        let mut material_indices = vec![None; meshes.len()];
        let mut mesh_normals = material_indices
            .iter()
            .map(|_| [].into_iter().collect())
            .collect::<Vec<_>>();

        let mut emissive_bvh_meshes = Vec::new();
        let mut non_emissive_bvh_meshes = Vec::new();

        let mut emissive_triangles = Vec::new();
        let mut non_emissive_triangles = Vec::new();

        let n_emissive_meshes = meshes
            .iter()
            .filter(|mesh| mesh.emission != RGBf32::ZERO)
            .count();

        meshes.into_iter().enumerate().for_each(
            |(
                _mesh_index,
                io_bridge::Mesh {
                    points,
                    triangles,
                    material_index,
                    emission: emission_rgb,
                    normals,
                },
            )| {
                if emission_rgb == RGBf32::ZERO {
                    let mesh_index = n_emissive_meshes + non_emissive_bvh_meshes.len();
                    for (triangle_index, _tri) in triangles.iter().enumerate() {
                        non_emissive_triangles.push(CompactTriangle {
                            mesh_index: mesh_index as u32,
                            triangle_index: triangle_index as u32,
                        });
                    }
                    material_indices[mesh_index] = material_index;
                    mesh_normals[mesh_index] = normals;
                    non_emissive_bvh_meshes.push(bvh::Mesh::new(points, triangles));
                } else {
                    let mesh_index = emissive_bvh_meshes.len();
                    for (triangle_index, _tri) in triangles.iter().enumerate() {
                        emissive_triangles.push(CompactTriangle {
                            mesh_index: mesh_index as u32,
                            triangle_index: triangle_index as u32,
                        });
                    }
                    let emission = Spectrum::tristimulus_illuminant(emission_rgb);
                    emissions.push(emission);
                    material_indices[mesh_index] = material_index;
                    mesh_normals[mesh_index] = normals;
                    emissive_bvh_meshes.push(bvh::Mesh::new(points, triangles));
                }
            },
        );
        let materials = materials
            .iter()
            .map(|m| match *m {
                io_bridge::Material::Lambertian { albedo } => Material::Lambertian {
                    albedo: Spectrum::new(albedo, false),
                },
                io_bridge::Material::Dielectric { eta } => Material::Dielectric {
                    eta: Spectrum::new(eta, false),
                },
                io_bridge::Material::Conductor { roughness, eta, k } => Material::Conductor {
                    roughness,
                    eta: Spectrum::new(eta, false),
                    k: Spectrum::new(k, false),
                },
            })
            .collect();

        let bvh_meshes = emissive_bvh_meshes
            .into_iter()
            .chain(non_emissive_bvh_meshes)
            .collect();

        let s = Self {
            bvh: Bvh::new(bvh_meshes),
            envmap,
            normals: mesh_normals.into_boxed_slice(),
            material_indices: material_indices.into_boxed_slice(),
            emissions: emissions.into_boxed_slice(),
            emissive_triangles: emissive_triangles.into_boxed_slice(),
            materials,
        };
        s
    }
    pub fn intersect(&self, ray: Ray) -> Option<SceneInteraction> {
        self.intersect_t(ray, f32::INFINITY)
    }

    pub fn intersect_before(&self, ray: Ray, p: Point3World) -> Option<SceneInteraction> {
        self.intersect_t(ray, ray.o.dist(p) - 0.0001)
    }

    pub fn intersect_t(&self, ray: Ray, t: f32) -> Option<SceneInteraction> {
        self.bvh.intersect(ray, t).map(|bvh_isect| {
            let [p0, p1, p2] = self.tri_points(bvh_isect.tri);
            let b = bvh_isect.isect.b;
            let p = triangle_point(p0, p1, p2, b);
            let abs_sum = p0.abs().to_vector() * b[0]
                + p1.abs().to_vector() * b[1]
                + p2.abs().to_vector() * b[2];
            let err = f32::numeric_gamma(7.0) * abs_sum;
            let ng = triangle_normal(p0, p1, p2);
            let ns = if let Some([n0, n1, n2]) = self.tri_normals(bvh_isect.tri) {
                ((n0 * b[0]) + (n1 * b[1]) + (n2 * b[2])).normalized()
            } else {
                ng
            };
            SceneInteraction {
                bvh_isect,
                shading_frame: ONB3f32::init_z(ns),
                bounds: Bounds3f32 {
                    min: p - err,
                    max: p + err,
                },
                geometric_normal: ng,
            }
        })
    }

    pub fn occluded_interaction(&self, from: &SceneInteraction, check: Point3World) -> bool {
        let w = (check - from.p()).normalized();
        let p = offset_ray_origin(from.bounds, from.ng(), w);
        self.occluded(p, check)
    }
    pub fn occluded_le_sample(&self, from: &LeSample, check: Point3World) -> bool {
        let w = (check - from.p()).normalized();
        let p = offset_ray_origin(from.bounds, from.n(), w);
        self.occluded(p, check)
    }
    pub fn occluded(&self, from_p: Point3World, check: Point3World) -> bool {
        let ray = Ray {
            o: from_p,
            d: check - from_p,
        };
        self.bvh.hit(ray, 0.9999)
    }
    pub fn compute_bsdf(
        &self,
        interaction: &SceneInteraction,
        swl: &SampledWavelengths,
    ) -> Option<BSDF> {
        self.material_indices[interaction.bvh_isect.tri.mesh_index as usize].map(|material_index| {
            match &self.materials[material_index] {
                Material::Lambertian { albedo } => BSDF::Lambertian {
                    albedo: albedo.eval(swl.lambdas),
                },
                Material::Dielectric { eta } => BSDF::Dielectric {
                    eta: eta.eval(swl.lambdas),
                },
                Material::Conductor { roughness, eta, k } => BSDF::Conductor {
                    roughness: *roughness,
                    eta: eta.eval(swl.lambdas),
                    k: k.eval(swl.lambdas),
                },
            }
        })
    }
    pub fn choose_light(&self, u1d: f32) -> Option<(AreaLight, f32)> {
        if self.emissive_triangles.is_empty() {
            return None;
        }
        let index = (self.emissive_triangles.len() as f32 * u1d) as usize;
        let tri = self.emissive_triangles[index];
        let [p0, p1, p2] = self.tri_points(tri);
        Some((
            AreaLight::new(p0, p1, p2, &self.emissions[tri.mesh_index as usize]),
            1.0 / self.emissive_triangles.len() as f32,
        ))
    }
    pub fn get_light(&self, interaction: &SceneInteraction) -> Option<AreaLight<'_>> {
        let tri = interaction.bvh_isect.tri;
        if tri.mesh_index as usize >= self.emissions.len() {
            return None;
        }
        let [p0, p1, p2] = self.tri_points(tri);
        let emission = &self.emissions[tri.mesh_index as usize];
        Some(AreaLight::new(p0, p1, p2, emission))
    }

    pub fn tri_points(&self, tri: bvh::CompactTriangle) -> [Point3World; 3] {
        let mesh = &self.bvh.meshes[tri.mesh_index as usize];
        let [i, j, k] = mesh.triangles[tri.triangle_index as usize];
        [
            mesh.points[i as usize],
            mesh.points[j as usize],
            mesh.points[k as usize],
        ]
    }
    pub fn tri_normals(&self, tri: bvh::CompactTriangle) -> Option<[Vec3World; 3]> {
        let normals = &self.normals[tri.mesh_index as usize];
        if normals.is_empty() {
            return None;
        }
        let mesh = &self.bvh.meshes[tri.mesh_index as usize];
        let [i, j, k] = mesh.triangles[tri.triangle_index as usize];
        Some([
            normals[i as usize],
            normals[j as usize],
            normals[k as usize],
        ])
    }
    pub fn eval_envmap(&self, dir: Vec3World, swl: &SampledWavelengths) -> SampledSpectrum {
        let Some(envmap) = &self.envmap else {
            return SampledSpectrum::ZERO;
        };
        envmap.eval(dir, swl)
    }
    pub fn sample_le(
        &self,
        swl: &SampledWavelengths,
        u1d: f32,
        u2d0: [f32; 2],
        u2d1: [f32; 2],
    ) -> Option<(LeSample, f32)> {
        let Some((light, pmf)) = self.choose_light(u1d) else {
            return None;
        };
        let Some(sample) = light.sample_le(swl, u2d0, u2d1) else {
            return None;
        };
        Some((sample, pmf))
    }
    pub fn sample_li(
        &self,
        from_p: Point3World,
        swl: &SampledWavelengths,
        u1d: f32,
        u2d: [f32; 2],
    ) -> Option<(LiSample, f32)> {
        let Some((light, pmf)) = self.choose_light(u1d) else {
            return None;
        };
        let Some(sample) = light.sample_li_solid_angle(from_p, swl, u2d) else {
            return None;
        };
        Some((sample, pmf))
    }

    pub fn bounds(&self) -> Bounds3f32<Worldspace> {
        self.bvh.bounds()
    }
}
