//! A bounding-volume hierarchy over boxes (triangles or points), built and walked with explicit
//! stacks so its depth never touches the call stack.

use crate::{Bounds3, Vec3};

/// Items per leaf.
const LEAF: usize = 4;

#[derive(Clone, Copy)]
struct Node {
    bounds: Bounds3,
    /// A leaf (`count > 0`) holds `items[first..first + count]`; an inner node has the
    /// children `first` and `right`.
    first: u32,
    count: u32,
    right: u32,
}

pub(crate) struct Bvh {
    nodes: Vec<Node>,
    items: Vec<u32>,
}

fn axis(v: Vec3, a: usize) -> f64 {
    match a {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

impl Bvh {
    /// A hierarchy over `boxes`; item `i` is `boxes[i]`.
    pub(crate) fn new(boxes: &[Bounds3]) -> Bvh {
        let mut items: Vec<u32> = (0..boxes.len()).filter_map(|i| u32::try_from(i).ok()).collect();
        let bx = |i: u32| boxes.get(i as usize).copied().unwrap_or(Bounds3::EMPTY);
        let mut nodes: Vec<Node> = Vec::with_capacity(2 * boxes.len() / LEAF + 1);
        if items.is_empty() {
            return Bvh { nodes, items };
        }
        nodes.push(Node { bounds: Bounds3::EMPTY, first: 0, count: 0, right: 0 });
        // (node, start, end) still to build.
        let mut stack = vec![(0usize, 0usize, items.len())];
        while let Some((node, start, end)) = stack.pop() {
            let Some(slice) = items.get_mut(start..end) else { continue };
            let bounds = slice.iter().fold(Bounds3::EMPTY, |b, i| b.union(&bx(*i)));
            let centers = Bounds3::from_points(slice.iter().map(|i| bx(*i).center()));
            let size = centers.size();
            let a = if size.x >= size.y && size.x >= size.z {
                0
            } else if size.y >= size.z {
                1
            } else {
                2
            };
            if slice.len() <= LEAF || axis(size, a) <= 0.0 {
                if let Some(n) = nodes.get_mut(node) {
                    *n = Node { bounds, first: start as u32, count: (end - start) as u32, right: 0 };
                }
                continue;
            }
            let mid = slice.len() / 2;
            slice.select_nth_unstable_by(mid, |p, q| axis(bx(*p).center(), a).total_cmp(&axis(bx(*q).center(), a)));
            let left = nodes.len();
            nodes.push(Node { bounds: Bounds3::EMPTY, first: 0, count: 0, right: 0 });
            nodes.push(Node { bounds: Bounds3::EMPTY, first: 0, count: 0, right: 0 });
            if let Some(n) = nodes.get_mut(node) {
                *n = Node { bounds, first: left as u32, count: 0, right: (left + 1) as u32 };
            }
            stack.push((left, start, start + mid));
            stack.push((left + 1, start + mid, end));
        }
        Bvh { nodes, items }
    }

    /// Visit every item whose box `hit` accepts, pruning subtrees whose box it rejects.
    fn walk(&self, hit: impl Fn(&Bounds3) -> bool, mut visit: impl FnMut(u32)) {
        let mut stack = vec![0usize];
        while let Some(i) = stack.pop() {
            let Some(n) = self.nodes.get(i) else { continue };
            if !hit(&n.bounds) {
                continue;
            }
            if n.count > 0 {
                let range = n.first as usize..(n.first + n.count) as usize;
                self.items.get(range).into_iter().flatten().for_each(|it| visit(*it));
            } else {
                stack.push(n.first as usize);
                stack.push(n.right as usize);
            }
        }
    }

    /// Items whose boxes overlap `b`.
    pub(crate) fn overlapping(&self, b: &Bounds3, out: &mut Vec<u32>) {
        out.clear();
        self.walk(|n| n.intersects(b), |i| out.push(i));
    }

    /// Items whose boxes the ray `origin + t·dir`, `t` in `[t0, t1]`, passes through.
    pub(crate) fn along(&self, origin: Vec3, dir: Vec3, t0: f64, t1: f64, visit: impl FnMut(u32)) {
        let inv = Vec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
        let hit = |b: &Bounds3| {
            let (mut lo, mut hi) = (t0, t1);
            for a in 0..3 {
                let (o, d) = (axis(origin, a), axis(inv, a));
                let (mut ta, mut tb) = ((axis(b.min, a) - o) * d, (axis(b.max, a) - o) * d);
                if ta.is_nan() || tb.is_nan() {
                    // Parallel to this slab: inside it or not at all.
                    if o < axis(b.min, a) || o > axis(b.max, a) {
                        return false;
                    }
                    continue;
                }
                if ta > tb {
                    std::mem::swap(&mut ta, &mut tb);
                }
                lo = lo.max(ta);
                hi = hi.min(tb);
                if lo > hi {
                    return false;
                }
            }
            true
        };
        self.walk(hit, visit);
    }
}
