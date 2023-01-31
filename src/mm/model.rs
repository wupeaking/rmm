use super::traj::TrajInfo;
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
    pub dummy_node_id: String,           // 该点加入路网图形成的虚拟节点的索引
    pub ori_traj_point: TrajInfo,
}

pub struct Layer {
    pub candidate: Option<Candidate>,
    pub prev_layer: RefCell<Option<Rc<RefCell<Layer>>>>,
    pub cumulative_prob: f64, // 累积概率
    pub tp: f64,              // 转移概率
}

pub type Layers = Vec<Rc<RefCell<Layer>>>;
pub type LayerLists = Vec<Layers>;

pub struct MMResult {
    pub o_path: Vec<String>,
    pub matched_candidates: Vec<Option<Candidate>>,
}

#[derive(Clone)]
pub struct Config {
    pub gps_err: f64,
    pub radius: f64,
    pub knn: u16,
    pub v_max: f64, // 最大速度
    pub factor: f64,
    pub reverse_tolerance: f64, // 反向公差
    pub road_netwok_path: String,
}
