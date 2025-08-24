use std::sync::Mutex;

use super::{Filter, dir_to_square, dtree::*};
use crate::{sampling::Sampler, spaces::*};
use math::*;

pub struct STree {
    pub nodes: Vec<STreeNode>,
    pub bounds: Bounds3World,
    pub filter: Filter,
}
pub struct STreeNode {
    pub sampler: DTree<SamplingState>,
    pub collector: Mutex<DTree<CollectingState>>,
    pub left: usize,
    pub right: usize,
    pub split_axis: usize,
    pub depth: usize,
}
impl STree {
    pub fn new(
        bounds: Bounds3f32<Worldspace>,
        spatial_filter: Filter,
        directional_filter: Filter,
    ) -> Self {
        Self {
            nodes: vec![STreeNode::new_root(directional_filter)],
            bounds: Bounds3World::around(
                bounds.centroid(),
                Vec3World::splat(bounds.extent().max_component()),
            ),
            filter: spatial_filter,
        }
    }
    pub fn sampler(&self, p: Point3f32<Worldspace>) -> &DTree<SamplingState> {
        let (index, _bounds) = self.leaf(p);
        &self.nodes[index].sampler
    }
    pub fn len(&self) -> usize {
        self.nodes.len()
    }
    pub fn new_iteration(
        &mut self,
        directional_threshold: f32,
        spatial_threshold: f32,
        iteration: usize,
    ) {
        self.subdivide_nodes(spatial_threshold, iteration);
        for node in self.nodes.iter_mut() {
            if node.is_leaf() {
                node.subdivide_dtree(directional_threshold);
            }
        }
    }
    pub fn subdivide_nodes(&mut self, spatial_threshold_base: f32, iteration: usize) {
        fn spatial_threshold(base: f32, _depth: usize, iteration: usize) -> f32 {
            // let i = iteration.min(depth) as i32;
            let i = iteration as i32;
            base * 2f32.powi(i).sqrt()
        }

        let mut i = 0;
        while i < self.nodes.len() {
            let lidx = self.nodes.len();
            let ridx = self.nodes.len() + 1;
            let node = &mut self.nodes[i];
            if node.is_leaf()
                && node.collector.lock().unwrap().statistical_weight
                    > spatial_threshold(spatial_threshold_base, node.depth, iteration)
            {
                let mut left = STreeNode {
                    sampler: node.sampler.clone(),
                    collector: Mutex::new(node.collector.lock().unwrap().clone()),
                    left: 0,
                    right: 0,
                    split_axis: (node.split_axis + 1) % 3,
                    depth: node.depth + 1,
                };
                let mut right = STreeNode {
                    sampler: std::mem::replace(&mut node.sampler, DTree::no_alloc()),
                    collector: std::mem::replace(
                        &mut node.collector,
                        Mutex::new(DTree::no_alloc()),
                    ),
                    left: 0,
                    right: 0,
                    split_axis: (node.split_axis + 1) % 3,
                    depth: node.depth + 1,
                };
                left.halve_statistical_weight();
                right.halve_statistical_weight();
                assert!(node.sampler.is_empty());
                assert!(node.collector.lock().unwrap().is_empty());
                node.left = lidx;
                node.right = ridx;
                self.nodes.push(left);
                self.nodes.push(right);
            }
            i += 1;
        }
    }
    pub fn record(&self, p: Point3World, wi: Vec3World, radiance: f32, sampler: &mut Sampler) {
        if !(radiance.is_finite()) {
            return;
        };
        if radiance == 0.0 {
            return;
        }
        let p3 = p;
        let p2 = dir_to_square(wi);
        match self.filter {
            Filter::Nearest => self.record_nearest(p3, p2, radiance, sampler),
            Filter::Box => self.record_box(p3, p2, radiance, sampler),
            Filter::Stochastic => self.record_stochastic(p3, p2, radiance, sampler),
        }
    }
    fn record_nearest(
        &self,
        p3: Point3World,
        p2: Point2Directional,
        radiance: f32,
        sampler: &mut Sampler,
    ) {
        let (node_index, _bounds) = self.leaf(p3);
        self.nodes[node_index].record(p2, radiance, 1.0, sampler);
    }
    fn record_box(
        &self,
        p3: Point3World,
        p2: Point2Directional,
        radiance: f32,
        sampler: &mut Sampler,
    ) {
        fn record_recursive(
            stree: &STree,
            node_bounds: Bounds3World,
            sample_footprint: Bounds3World,
            p2: Point2Directional,
            radiance: f32,
            node_index: usize,
            sampler: &mut Sampler,
        ) {
            for i in 0..2 {
                let node = &stree.nodes[node_index];
                let mid = node_bounds.centroid()[node.split_axis];
                let mut min = node_bounds.min;
                let mut max = node_bounds.max;
                if i == 0 {
                    max[node.split_axis] = mid;
                } else {
                    min[node.split_axis] = mid;
                }
                let chlid_bounds = Bounds3World { min, max };
                let w = sample_footprint.overlap(chlid_bounds) / sample_footprint.volume();
                if w == 0.0 {
                    continue;
                }
                if node.is_leaf() {
                    node.record(p2, radiance * w, w, sampler);
                } else {
                    record_recursive(
                        stree,
                        chlid_bounds,
                        sample_footprint,
                        p2,
                        radiance,
                        node.child_node(i),
                        sampler,
                    );
                }
            }
        }
        let (_index, leaf_bounds) = self.leaf(p3);
        let sample_footprint =
            Bounds3World::around(p3, leaf_bounds.extent()).clamp(self.bounds.min, self.bounds.max);
        record_recursive(
            self,
            self.bounds,
            sample_footprint,
            p2,
            radiance,
            0,
            sampler,
        );
    }
    fn record_stochastic(
        &self,
        p3: Point3World,
        p2: Point2Directional,
        radiance: f32,
        sampler: &mut Sampler,
    ) {
        let (_p3_node_index, p3_node_bounds) = self.leaf(p3);
        let jitter_bounds = Bounds3World::around(p3, p3_node_bounds.extent());
        let p3 = self.bounds.clip(jitter_bounds.lerp(Vec3World::new(
            sampler.sample1d(),
            sampler.sample1d(),
            sampler.sample1d(),
        )));
        self.record_nearest(p3, p2, radiance, sampler);
    }
    fn leaf(&self, p: Point3World) -> (usize, Bounds3World) {
        let mut bounds = self.bounds;
        let mut node_index = 0;
        loop {
            let node = &self.nodes[node_index];
            if node.is_leaf() {
                return (node_index, bounds);
            } else {
                let (child_index, child_bounds) = child_index(p, bounds, node.split_axis);
                bounds = child_bounds;
                node_index = node.child_node(child_index);
                assert!(node_index != 0);
            }
        }
    }
}

fn child_index(p: Point3World, node_bounds: Bounds3World, axis: usize) -> (usize, Bounds3World) {
    let mid = node_bounds.centroid();
    let mut child_bounds = node_bounds;
    if p[axis] < mid[axis] {
        child_bounds.max[axis] = mid[axis];
        return (0, child_bounds);
    } else {
        child_bounds.min[axis] = mid[axis];
        return (1, child_bounds);
    }
}

impl STreeNode {
    fn new_root(directional_filter: Filter) -> Self {
        Self {
            sampler: DTree::new_root(directional_filter),
            collector: Mutex::new(DTree::new_root(directional_filter)),
            left: 0,
            right: 0,
            split_axis: 0,
            depth: 0,
        }
    }
    fn halve_statistical_weight(&mut self) {
        self.collector.lock().unwrap().statistical_weight /= 2.0;
    }
    fn is_leaf(&self) -> bool {
        self.left == 0
    }
    fn record(&self, p: Point2f32<Directionalspace>, radiance: f32, w: f32, sampler: &mut Sampler) {
        assert!(self.is_leaf());
        self.collector
            .lock()
            .unwrap()
            .record(p, radiance, w, sampler);
    }
    fn subdivide_dtree(&mut self, directional_threshold: f32) {
        let mut collector = self.collector.lock().unwrap();
        collector.build();
        collector.emit_into(&mut self.sampler);
        collector.subdivide_and_clear(directional_threshold);
    }
    fn child_node(&self, index: usize) -> usize {
        if index == 0 { self.left } else { self.right }
    }
}
