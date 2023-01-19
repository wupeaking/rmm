use crate::algorithm;
use crate::graph::Edge;
use std::cell::RefCell;
use std::rc::Rc;

/**
 * 定义mm需要的一些结构体
 * */

// 定义每次坐标点可能位于某个edge的候选
#[derive(Clone)]
pub struct Candidate {
    pub edge: Edge,
    // gps点到edge的距离
    pub distance: f64,
    pub offset: f64,                     // 偏移量
    pub closest_point: algorithm::Point, // 最近点
    pub ep: f64,                         // 计算出的输出概率
    pub dummy_node_index: usize,         // 该点加入路网图形成的虚拟节点的索引
}

pub struct Layer {
    pub candidate: Candidate,
    prev_layer: RefCell<Option<Rc<Layer>>>,
    cumulative_prob: f64, // 累积概率
    tp: f64,              // 转移概率
}

pub type Layers = Vec<Layer>;

pub struct TrajInfo {
    pub point: (f64, f64),
    pub time_stamp: u64,
}

pub type Trajectory = Vec<TrajInfo>;
