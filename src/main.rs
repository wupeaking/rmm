use rtree_rs::{RTree, Rect};

fn main() {
    let mut tr = RTree::new();
    tr.insert(Rect::new_point([-112.0078, 33.4373]), "PHX");

    println!("Hello, world!");
}
