use crate::{
    geom::{Ray, RayTriangleIntersection, ray_triangle_intersection},
    spaces::*,
};
use arrayvec::ArrayVec;
use math::*;

pub struct Mesh {
    pub points: Box<[Point3World]>,
    pub triangles: Box<[[u32; 3]]>,
}
pub struct Bvh {
    pub meshes: Box<[Mesh]>,
    pub compact_triangles: Box<[CompactTriangle]>,
    nodes: Box<[BvhNode]>,
}

#[derive(Debug, Clone, Copy, Default, Hash, PartialEq, PartialOrd)]
pub struct CompactTriangle {
    pub mesh_index: u32,
    pub triangle_index: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct BvhIntersection {
    pub tri: CompactTriangle,
    pub isect: RayTriangleIntersection<Worldspace>,
}

#[repr(align(32))]
struct BvhNode {
    bounds: Bounds3World,
    offset: u32,
    n_tris: u16,
    split_axis: u8,
}

impl Mesh {
    pub fn new(points: Box<[Point3World]>, triangles: Box<[[u32; 3]]>) -> Self {
        Self { points, triangles }
    }
}

impl Bvh {
    pub fn bounds(&self) -> Bounds3f32<Worldspace> {
        self.nodes[0].bounds
    }
    pub fn new(meshes: Box<[Mesh]>) -> Self {
        let mut original_compact_triangles = vec![];
        let mut build_triangles = vec![];
        for (mesh_index, mesh) in meshes.iter().enumerate() {
            for (triangle_index, tri) in mesh.triangles.iter().enumerate() {
                let &[i, j, k] = tri;
                let [p0, p1, p2] = [
                    mesh.points[i as usize],
                    mesh.points[k as usize],
                    mesh.points[j as usize],
                ];
                build_triangles.push(BuildTriangle {
                    bounds: Bounds3World::new(p0, p1) | p2,
                    index: original_compact_triangles.len(),
                });
                original_compact_triangles.push(CompactTriangle {
                    mesh_index: mesh_index as u32,
                    triangle_index: triangle_index as u32,
                });
            }
        }
        let mut compact_triangles = Vec::with_capacity(original_compact_triangles.len());
        let mut n_nodes = 0;
        let root = build_recursive(
            &original_compact_triangles,
            &mut compact_triangles,
            &mut build_triangles,
            &mut n_nodes,
        );
        let mut nodes = Vec::with_capacity(n_nodes);
        flatten(&mut nodes, root);
        // dbg!(nodes[0].bounds);
        Self {
            meshes,
            compact_triangles: compact_triangles.into_boxed_slice(),
            nodes: nodes.into_boxed_slice(),
        }
    }
    pub fn intersect(&self, ray: Ray, ray_end_t: f32) -> Option<BvhIntersection> {
        let mut isect = RayTriangleIntersection::<Worldspace>::default();
        isect.t = ray_end_t;
        let mut tri = CompactTriangle {
            mesh_index: 0,
            triangle_index: 0,
        };

        let mut node_stack = ArrayVec::<usize, 64>::new();
        node_stack.push(0);
        let inv_dir = Vec3World::ONE / ray.d;
        let dir_is_neg = [inv_dir.x < 0.0, inv_dir.y < 0.0, inv_dir.z < 0.0];
        while let Some(node_index) = node_stack.pop() {
            let node = &self.nodes[node_index];
            if ray_bounds_hit(
                [node.bounds.min, node.bounds.max],
                ray.o,
                inv_dir,
                dir_is_neg,
                ray_end_t,
            ) {
                if node.n_tris > 0 {
                    for &check_tri in self.compact_triangles[node.offset as usize..]
                        [..node.n_tris as usize]
                        .iter()
                    {
                        let mesh = &self.meshes[check_tri.mesh_index as usize];
                        let [i, j, k] = mesh.triangles[check_tri.triangle_index as usize];
                        let [p0, p1, p2] = [
                            mesh.points[i as usize],
                            mesh.points[j as usize],
                            mesh.points[k as usize],
                        ];
                        if let Some(new_isect) =
                            ray_triangle_intersection(ray.o, ray.d, p0, p1, p2, isect.t)
                        {
                            isect = new_isect;
                            tri = check_tri;
                        }
                    }
                } else {
                    if dir_is_neg[node.split_axis as usize] {
                        node_stack.push(node_index + 1);
                        node_stack.push(node.offset as usize);
                    } else {
                        node_stack.push(node.offset as usize);
                        node_stack.push(node_index + 1);
                    }
                }
            }
        }
        if isect.t != ray_end_t {
            Some(BvhIntersection { isect, tri })
        } else {
            None
        }
    }
    pub fn hit(&self, ray: Ray, max_t: f32) -> bool {
        self.intersect(ray, max_t).is_some()
    }
}

const N_SAH_BUCKETS: usize = 12;
#[derive(Clone, Copy)]
struct SAHBucket {
    bounds: Bounds3World,
    count: f32,
}
fn bucket_index(centroid_bounds: Bounds3World, p: Point3World, axis: usize) -> usize {
    (N_SAH_BUCKETS - 1).min((N_SAH_BUCKETS as f32 * centroid_bounds.offset(p)[axis]) as usize)
}

enum BuildNode {
    Leaf {
        bounds: Bounds3World,
        start: usize,
        end: usize,
    },
    Branch {
        bounds: Bounds3World,
        left: Box<Self>,
        right: Box<Self>,
        split_axis: usize,
    },
}

#[derive(Debug, Clone)]
struct BuildTriangle {
    bounds: Bounds3World,
    index: usize,
}
fn build_recursive(
    original_compact_triangles: &[CompactTriangle],
    compact_triangles: &mut Vec<CompactTriangle>,
    build_triangles: &mut [BuildTriangle],
    n_nodes: &mut usize,
) -> Box<BuildNode> {
    *n_nodes += 1;
    let bounds = build_triangles
        .iter()
        .fold(Bounds3World::empty(), |acc, tri| acc | tri.bounds);
    let mut create_leaf = |build_triangles: &mut [BuildTriangle]| {
        let start = compact_triangles.len();
        compact_triangles.extend(
            build_triangles
                .iter()
                .map(|bt| original_compact_triangles[bt.index]),
        );
        let end = compact_triangles.len();
        return Box::new(BuildNode::Leaf { bounds, start, end });
    };
    if build_triangles.len() == 1 || bounds.surface_area() == 0.0 {
        return create_leaf(build_triangles);
    }
    let centroid_bounds = build_triangles
        .iter()
        .fold(Bounds3World::empty(), |acc, tri| {
            acc | tri.bounds.centroid()
        });
    let split_axis = centroid_bounds.extent().max_axis();

    let mut buckets = [SAHBucket {
        bounds: Bounds3World::empty(),
        count: 0.0,
    }; N_SAH_BUCKETS];

    for tri in build_triangles.iter() {
        let index = bucket_index(centroid_bounds, tri.bounds.centroid(), split_axis);
        buckets[index].bounds |= tri.bounds;
        buckets[index].count += 1.0;
    }

    let mut costs = [0.0; N_SAH_BUCKETS - 1];
    {
        let mut bounds_below = Bounds3f32::empty();
        let mut costs_below = 0.0;
        (0..N_SAH_BUCKETS - 1).for_each(|i| {
            bounds_below |= buckets[i].bounds;
            costs_below += buckets[i].count * bounds_below.surface_area();
            costs[i] += costs_below;
        });
    }
    {
        let mut bounds_above = Bounds3f32::empty();
        let mut costs_above = 0.0;
        (1..N_SAH_BUCKETS).rev().for_each(|i| {
            bounds_above |= buckets[i].bounds;
            costs_above += buckets[i].count * bounds_above.surface_area();
            costs[i - 1] += costs_above;
        });
    }

    let mut min_index = 0;
    let mut min_cost = f32::INFINITY;
    for (idx, &cost) in costs.iter().enumerate() {
        if cost < min_cost {
            min_index = idx;
            min_cost = cost;
        }
    }

    let leaf_cost = build_triangles.len() as f32;
    let split_cost = 0.5 + min_cost / bounds.surface_area();

    if leaf_cost < split_cost {
        return create_leaf(build_triangles);
    }

    let mid = itertools::partition(build_triangles.iter_mut(), |bt| {
        bucket_index(centroid_bounds, bt.bounds.centroid(), split_axis) <= min_index
    });
    if mid == 0 || mid == build_triangles.len() - 1 {
        return create_leaf(build_triangles);
    }

    let (left, right) = build_triangles.split_at_mut(mid);
    let left = build_recursive(original_compact_triangles, compact_triangles, left, n_nodes);
    let right = build_recursive(
        original_compact_triangles,
        compact_triangles,
        right,
        n_nodes,
    );
    return Box::new(BuildNode::Branch {
        bounds,
        left,
        right,
        split_axis,
    });
}

fn flatten(nodes: &mut Vec<BvhNode>, root: Box<BuildNode>) -> usize {
    let index = nodes.len();
    nodes.push(BvhNode {
        bounds: Bounds3World::empty(),
        offset: 0,
        n_tris: 0,
        split_axis: 0,
    });
    match *root {
        BuildNode::Leaf { bounds, start, end } => {
            nodes[index].bounds = bounds;
            nodes[index].offset = start as u32;
            nodes[index].n_tris = (end - start) as u16;
        }
        BuildNode::Branch {
            bounds,
            left,
            right,
            split_axis,
        } => {
            nodes[index].bounds = bounds;
            nodes[index].split_axis = split_axis as u8;
            flatten(nodes, left);
            nodes[index].offset = flatten(nodes, right) as u32;
        }
    }
    index
}

fn ray_bounds_hit(
    bounds: [Point3World; 2],
    ray_origin: Point3World,
    inv_dir: Vec3World,
    dir_is_neg: [bool; 3],
    ray_end_t: f32,
) -> bool {
    let mut tmin = 0f32;
    let mut tmax = ray_end_t;
    for i in 0..3 {
        let ti_min = inv_dir[i] * (bounds[dir_is_neg[i] as usize][i] - ray_origin[i]);
        let ti_max = (1.0 + 2.0 * f32::numeric_gamma(3.0))
            * inv_dir[i]
            * (bounds[!dir_is_neg[i] as usize][i] - ray_origin[i]);
        // if ti_min > tmax || tmin > ti_max {
        //     return false;
        // }
        tmin = tmin.max(ti_min);
        tmax = tmax.min(ti_max);
    }
    (0.0 <= tmin) && (tmin <= tmax)
}
