use super::Filter;
use crate::sampling::{self, Sampler};
use math::*;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Directionalspace;
pub type Point2Directional = Point2f32<Directionalspace>;
pub type Bounds2Directional = Bounds2f32<Directionalspace>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct LocalIndex(u8);
impl LocalIndex {
    fn all() -> [Self; 4] {
        [0, 1, 2, 3].map(Self)
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct GlobalIndex(u16);

pub struct SamplingState;
pub struct CollectingState;
pub struct DTree<State> {
    pub statistical_weight: f32,
    pub filter: Filter,
    nodes: NodeList,
    _marker: std::marker::PhantomData<State>,
}
#[derive(Default, Debug, Clone, Copy)]
pub struct DTreeNode {
    radiance: [f32; 4],
    children: [GlobalIndex; 4],
}

impl<State> Clone for DTree<State> {
    fn clone(&self) -> Self {
        Self {
            nodes: self.nodes.clone(),
            statistical_weight: self.statistical_weight,
            filter: self.filter,
            _marker: self._marker,
        }
    }
}

impl<State> DTree<State> {
    pub fn new_root(filter: Filter) -> Self {
        Self {
            nodes: NodeList {
                nodes: vec![DTreeNode::default()],
                free_slots: vec![],
            },
            statistical_weight: 0.0,
            _marker: Default::default(),
            filter,
        }
    }
    pub fn no_alloc() -> Self {
        Self {
            nodes: NodeList {
                nodes: vec![],
                free_slots: vec![],
            },
            statistical_weight: 0.0,
            _marker: Default::default(),
            filter: Filter::Nearest,
        }
    }
    pub fn sum(&self) -> f32 {
        self.nodes[GlobalIndex(0)].sum()
    }
    pub fn mean(&self) -> f32 {
        if self.statistical_weight == 0.0 {
            0.0
        } else {
            self.sum() / (FOUR_PI * self.statistical_weight)
        }
    }
    pub fn is_empty(&self) -> bool {
        self.nodes.nodes.is_empty()
    }
    pub fn leaf(&self, p: Point2Directional) -> (GlobalIndex, Bounds2Directional) {
        let mut bounds = Bounds2Directional::unit();
        let mut node_index = GlobalIndex(0);
        loop {
            let node = &self.nodes[node_index];
            let (child, child_bounds) = child_index(p, bounds);
            if node.is_leaf(child) {
                return (node.child_node(child), child_bounds);
            } else {
                node_index = node.child_node(child);
                bounds = child_bounds;
            }
        }
    }
}

impl DTree<SamplingState> {
    pub fn sample(&self, sampler: &mut Sampler) -> Point2Directional {
        if self.mean() == 0.0 {
            return sampler.sample2d().into();
        }
        fn sample_recursive<State>(
            dtree: &DTree<State>,
            node: GlobalIndex,
            bounds: Bounds2Directional,
            sampler: &mut Sampler,
        ) -> Point2Directional {
            let node = &dtree.nodes[node];
            let child = LocalIndex(sampling::sample_discrete(
                sampler.sample1d(),
                &[
                    node.child_radiance(LocalIndex(0)),
                    node.child_radiance(LocalIndex(1)),
                    node.child_radiance(LocalIndex(2)),
                    node.child_radiance(LocalIndex(3)),
                ],
            ) as u8);

            let min = bounds.min;
            let mid = bounds.centroid();
            let max = bounds.max;
            let child_bounds = [
                Bounds2Directional {
                    min: Point2f32::new(min.x, min.y),
                    max: Point2f32::new(mid.x, mid.y),
                },
                Bounds2Directional {
                    min: Point2f32::new(mid.x, min.y),
                    max: Point2f32::new(max.x, mid.y),
                },
                Bounds2Directional {
                    min: Point2f32::new(min.x, mid.y),
                    max: Point2f32::new(mid.x, max.y),
                },
                Bounds2Directional {
                    min: Point2f32::new(mid.x, mid.y),
                    max: Point2f32::new(max.x, max.y),
                },
            ][child.0 as usize];

            if node.is_leaf(child) {
                let u2d = sampler.sample2d();
                let sample = child_bounds.lerp(u2d.into());
                sample.min_by_component(child_bounds.max - Vector2f32::splat(f32::EPSILON))
            } else {
                sample_recursive(dtree, node.child_node(child), child_bounds, sampler)
            }
        }
        let sample = sample_recursive(self, GlobalIndex(0), Bounds2Directional::unit(), sampler)
            .clamp(Point2f32::ZERO, Point2f32::ONE);
        // assert!(self.pdf(sample) != 0.0, "{:?}", sample);
        sample
    }

    pub fn pdf(&self, p: Point2Directional) -> f32 {
        if self.mean() == 0.0 {
            return INV_FOUR_PI;
        }
        fn pdf_recursive<State>(
            dtree: &DTree<State>,
            p: Point2Directional,
            bounds: Bounds2Directional,
            node: GlobalIndex,
        ) -> f32 {
            let node = &dtree.nodes[node];
            let (child, child_bounds) = child_index(p, bounds);

            if node.child_radiance(child) == 0.0 {
                return 0.0;
            }
            let scale = 4.0 * node.child_radiance(child) / node.sum();
            if node.is_leaf(child) {
                scale
            } else {
                scale * pdf_recursive(dtree, p, child_bounds, node.child_node(child))
            }
        }
        return pdf_recursive(self, p, Bounds2Directional::unit(), GlobalIndex(0)) * INV_FOUR_PI;
    }
}

impl DTree<CollectingState> {
    // progagetes radiance values up the tree
    // must be called in between recording and sampling
    pub fn build(&mut self) {
        fn build_recursive(dtree: &mut DTree<CollectingState>, node: GlobalIndex) {
            for child in LocalIndex::all() {
                if dtree.nodes[node].is_leaf(child) {
                    continue;
                }
                let child_node = dtree.nodes[node].child_node(child);
                build_recursive(dtree, child_node);
                let child_sum = dtree.nodes[child_node].sum();
                dtree.nodes[node].set_sum(child, child_sum);
            }
        }
        build_recursive(self, GlobalIndex(0));
    }
    // values must be propagated back up the tree before it can be used for sampling
    pub fn record(&mut self, p: Point2Directional, radiance: f32, w: f32, sampler: &mut Sampler) {
        self.statistical_weight += w;
        match self.filter {
            Filter::Nearest => self.record_nearest(p, radiance),
            Filter::Box => self.record_box(p, radiance),
            Filter::Stochastic => self.record_stochastic(p, radiance, sampler),
        }
    }
    fn record_nearest(&mut self, p: Point2Directional, radiance: f32) {
        let mut bounds = Bounds2Directional::unit();
        let mut node_index = GlobalIndex(0);
        loop {
            let node = &mut self.nodes[node_index];
            let (child, child_bounds) = child_index(p, bounds);
            if node.is_leaf(child) {
                node.add_sum(child, radiance);
                return;
            } else {
                node_index = node.child_node(child);
                bounds = child_bounds;
            }
        }
    }
    fn record_stochastic(
        &mut self,
        p: Point2f32<Directionalspace>,
        radiance: f32,
        sampler: &mut Sampler,
    ) {
        let (_p_node_index, p_node_bounds) = self.leaf(p);
        let jitter_bounds = Bounds2Directional::around(p, p_node_bounds.extent());
        let p = Bounds2Directional::unit()
            .clip(jitter_bounds.lerp(Vector2f32::new(sampler.sample1d(), sampler.sample1d())));
        self.record_nearest(p, radiance);
    }
    fn record_box(&mut self, p: Point2Directional, radiance: f32) {
        fn record_recursive(
            dtree: &mut DTree<CollectingState>,
            node_bounds: Bounds2Directional,
            sample_footprint: Bounds2Directional,
            radiance: f32,
            node_index: GlobalIndex,
        ) {
            for child in LocalIndex::all() {
                let node = &mut dtree.nodes[node_index];
                let min = node_bounds.min;
                let mid = node_bounds.centroid();
                let max = node_bounds.max;
                let child_bounds = [
                    Bounds2Directional {
                        min: Point2f32::new(min.x, min.y),
                        max: Point2f32::new(mid.x, mid.y),
                    },
                    Bounds2Directional {
                        min: Point2f32::new(mid.x, min.y),
                        max: Point2f32::new(max.x, mid.y),
                    },
                    Bounds2Directional {
                        min: Point2f32::new(min.x, mid.y),
                        max: Point2f32::new(mid.x, max.y),
                    },
                    Bounds2Directional {
                        min: Point2f32::new(mid.x, mid.y),
                        max: Point2f32::new(max.x, max.y),
                    },
                ][child.0 as usize];
                let w = sample_footprint.overlap(child_bounds) / sample_footprint.area();
                if w == 0.0 {
                    continue;
                }
                if node.is_leaf(child) {
                    node.add_sum(child, radiance * w);
                } else {
                    let child_node = node.child_node(child);
                    record_recursive(dtree, child_bounds, sample_footprint, radiance, child_node);
                }
            }
        }
        let (_index, leaf_bounds) = self.leaf(p);
        let sample_footprint = Bounds2Directional::around(p, leaf_bounds.extent())
            .clamp(Point2Directional::ZERO, Point2Directional::ONE);

        record_recursive(
            self,
            Bounds2Directional::unit(),
            sample_footprint,
            radiance,
            GlobalIndex(0),
        );
    }

    pub fn subdivide_and_clear(&mut self, subdivision_threshold: f32) {
        let mut node_stack = vec![(GlobalIndex(0), 1)];
        let total = self.sum();
        while let Some((node_index, depth)) = node_stack.pop() {
            let node = self.nodes[node_index];
            for child in LocalIndex::all() {
                let frac = if total > 0.0 {
                    node.child_radiance(child) / total
                } else {
                    0.25f32.powi(depth)
                };

                if frac < subdivision_threshold {
                    self.nodes.free_recursive(node.child_node(child));
                    self.nodes[node_index].set_child(child, GlobalIndex(0));
                    continue;
                }
                if node.is_leaf(child) {
                    let new_branch = self.nodes.add(DTreeNode {
                        radiance: [node.child_radiance(child) / 4.0; 4],
                        ..Default::default()
                    });
                    self.nodes[node_index].set_child(child, new_branch);
                    node_stack.push((new_branch, depth + 1));
                } else {
                    node_stack.push((node.child_node(child), depth + 1));
                }
            }
        }
        self.statistical_weight = 0.0;
        for node in self.nodes.nodes.iter_mut() {
            node.radiance = [0.0; 4];
        }
    }
    pub fn emit_into(&self, sampler: &mut DTree<SamplingState>) {
        sampler.statistical_weight = self.statistical_weight;
        sampler.nodes.replace_by(&self.nodes);
    }
}

fn child_index(
    p: Point2Directional,
    bounds: Bounds2Directional,
) -> (LocalIndex, Bounds2Directional) {
    let mid = bounds.centroid();
    let mut min = bounds.min;
    let mut max = bounds.max;
    let mut res = 0;
    if p.x < mid.x {
        max.x = mid.x;
    } else {
        min.x = mid.x;
        res |= 0b01;
    }
    if p.y < mid.y {
        max.y = mid.y;
    } else {
        min.y = mid.y;
        res |= 0b10;
    }
    (LocalIndex(res), Bounds2f32 { min, max })
}

impl DTreeNode {
    fn add_sum(&mut self, child: LocalIndex, val: f32) {
        self.radiance[child.0 as usize] += val;
    }
    fn set_sum(&mut self, child: LocalIndex, val: f32) {
        self.radiance[child.0 as usize] = val;
    }
    fn sum(&self) -> f32 {
        self.radiance.iter().sum()
    }
    fn child_radiance(&self, child: LocalIndex) -> f32 {
        self.radiance[child.0 as usize]
    }
    fn set_child(&mut self, child: LocalIndex, node: GlobalIndex) {
        self.children[child.0 as usize] = node
    }
    fn child_node(&self, child: LocalIndex) -> GlobalIndex {
        self.children[child.0 as usize]
    }
    fn is_leaf(&self, child: LocalIndex) -> bool {
        self.child_node(child).0 == 0
    }
}

struct NodeList {
    nodes: Vec<DTreeNode>,
    free_slots: Vec<GlobalIndex>,
}

impl NodeList {
    pub fn replace_by(&mut self, other: &Self) {
        self.nodes.clear();
        self.free_slots.clear();
        self.nodes.extend(&other.nodes);
        self.free_slots.extend(&other.free_slots);
    }
    pub fn add(&mut self, node: DTreeNode) -> GlobalIndex {
        if let Some(index) = self.free_slots.pop() {
            self[index] = node;
            return index;
        }
        let index = GlobalIndex(self.nodes.len() as u16);
        self.nodes.push(node);
        return index;
    }
    pub fn free_recursive(&mut self, node: GlobalIndex) {
        if node.0 == 0 {
            return;
        }
        self.free_slots.push(node);
        let node = self[node];
        for child in LocalIndex::all() {
            if !node.is_leaf(child) {
                self.free_recursive(node.child_node(child));
            }
        }
    }

    pub fn clone(&self) -> NodeList {
        Self {
            nodes: self.nodes.clone(),
            free_slots: self.free_slots.clone(),
        }
    }
}

impl std::ops::Index<GlobalIndex> for NodeList {
    type Output = DTreeNode;
    fn index(&self, index: GlobalIndex) -> &Self::Output {
        self.nodes.index(index.0 as usize)
    }
}
impl std::ops::IndexMut<GlobalIndex> for NodeList {
    fn index_mut(&mut self, index: GlobalIndex) -> &mut Self::Output {
        self.nodes.index_mut(index.0 as usize)
    }
}
