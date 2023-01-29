use super::model::{Candidate, Config, Layer, LayerLists, Layers, MMResult};
use super::TrajInfo;
use super::Trajectory;
use crate::algorithm;
use crate::graph::{network, Edge, EdgeType, RoadGraph};
use anyhow;
use geojson;
use log::{debug, error, info, warn};
use rtree_rs::{RTree, Rect};
use std::cell::RefCell;
use std::{fs::File, io::BufReader, rc::Rc};

pub struct MMatch {
    road_graph: RoadGraph,
    road_rtree: RTree<2, f64, usize>,
}

// from network file to map matching
impl TryFrom<String> for MMatch {
    type Error = anyhow::Error;
    fn try_from(value: String) -> std::result::Result<Self, Self::Error> {
        debug!("loading netwok from : {}", value);
        let file = File::open(value.clone())?;
        debug!("open file {} success", value);
        let reader = BufReader::new(file);
        debug!("load geojson file {} ", value);
        let geojson = geojson::GeoJson::from_reader(reader)?;
        let network = network::Network::try_from(geojson)?;
        info!("load road network success file: {} ", value);
        let road_graph = RoadGraph::new(network);
        debug!("build road graph success");
        let mut road_rtree = RTree::new();
        for (index, edge) in road_graph.network.edges.iter().enumerate() {
            let points = edge.get_geom_rect()?;
            let rect = Rect::new([points.0 .0, points.0 .1], [points.1 .0, points.1 .1]);
            road_rtree.insert(rect, index);
        }
        debug!("build road rtree success");
        info!("construct map matching success... ");
        Ok(MMatch {
            road_graph,
            road_rtree,
        })
    }
}

impl MMatch {
    // 查询bbox内的edge 返回edge索引
    pub fn query_bbox(&self, bbox: &Rect<2, f64>) -> Vec<usize> {
        let mut result = Vec::new();
        for item in self.road_rtree.search(*bbox) {
            result.push(*item.data);
        }
        result
    }

    // 查询候选者
    pub fn query_candidate(
        &self,
        traj_point: &TrajInfo,
        radius: f64,
        knn: u16,
        gps_err: f64,
        prev_candidate: Option<Rc<Candidate>>,
    ) -> Vec<Candidate> {
        // 构建bbox
        let bbox = Rect::new(
            [traj_point.point.0 - radius, traj_point.point.1 - radius],
            [traj_point.point.0 + radius, traj_point.point.1 + radius],
        );
        let edge_indices = self.query_bbox(&bbox);
        let mut candidates = Vec::new();
        let mut prev_edge_id = "".to_string();
        if prev_candidate.is_some() {
            let p = prev_candidate.unwrap().clone();
            prev_edge_id = p.edge.get_edge_id().clone();
        }
        let mut exit_candidate: Option<Candidate> = None;

        for (_, edge_index) in edge_indices.iter().enumerate() {
            let edge = self.road_graph.network.find_edge_by_index(*edge_index);

            match edge {
                Some(edge) => {
                    let line: algorithm::Line = edge.get_geometry().clone().try_into().unwrap();
                    let (distance, offset, close_point) =
                        algorithm::linear_reference_distance(traj_point.point, &line.0);
                    if distance > radius {
                        continue;
                    }
                    if offset > edge.get_length() {
                        continue;
                    }
                    let candidate = Candidate {
                        edge: edge,
                        distance: distance,
                        offset: offset,
                        closest_point: close_point,
                        dummy_node_index: 0,
                        ep: self.calc_ep(distance, gps_err),
                        ori_traj_point: traj_point.clone(),
                    };
                    if prev_edge_id == candidate.edge.get_edge_id() {
                        exit_candidate = Some(candidate.clone());
                    }
                    candidates.push(candidate);
                }
                None => {
                    error!("edge not found by index {}", edge_index);
                    continue;
                }
            }
        }
        candidates.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        if candidates.len() > knn as usize {
            candidates.truncate(knn as usize);
        }
        if exit_candidate.is_none() {
            return candidates;
        }
        let candidate = exit_candidate.unwrap();
        let exit_candiate_edge_id = candidate.edge.get_edge_id();
        for (_, candidate) in candidates.iter().enumerate() {
            if candidate.edge.get_edge_id() == exit_candiate_edge_id {
                return candidates;
            }
        }
        candidates.push(candidate);

        candidates
    }

    // calc ep
    fn calc_ep(&self, dist: f64, gps_err: f64) -> f64 {
        let a = dist / gps_err;
        let v = f64::exp(-0.5 * a * a);
        if v < f64::MIN_POSITIVE {
            return f64::MIN_POSITIVE;
        }
        v
    }

    fn calc_tp(&self, gps_dist: f64, candidate_dist: f64) -> f64 {
        if gps_dist > candidate_dist {
            return 1.0 as f64;
        }
        gps_dist / candidate_dist
    }

    fn max_prob_candidate(layers: &Layers) -> Option<Candidate> {
        if layers.is_empty() {
            return None;
        }
        let mut max_prob = 0.0;
        let mut max_prob_candidate: Option<Candidate> = None;
        for layer in layers {
            if layer.borrow().candidate.is_none() {
                continue;
            }
            if layer.borrow().cumulative_prob > max_prob {
                max_prob = layer.borrow().cumulative_prob;
                max_prob_candidate = Some(layer.borrow().candidate.as_ref().unwrap().clone());
            }
        }
        max_prob_candidate
    }
}

impl MMatch {
    pub fn match_traj(&mut self, traj: &Trajectory, cfg: &Config) -> anyhow::Result<MMResult> {
        if traj.len() == 0 {
            return Err(anyhow::anyhow!("trajectory is empty"));
        }
        let mut layer_lists = LayerLists::new();

        let condicates = self.query_candidate(&traj[0], cfg.radius, cfg.knn, cfg.gps_err, None);
        let mut cur_layers = Layers::new();
        // 如果没有候选者 添加一个空的候选者

        for mut candidate in condicates {
            // 开始构建虚拟图
            let from_node_index = self
                .road_graph
                .network
                .find_node_by_id(&candidate.edge.get_from_node());
            if from_node_index.is_none() {
                error!(
                    "from node not found by id {}",
                    candidate.edge.get_from_node()
                );
                continue;
            }
            let to_node_index = self
                .road_graph
                .network
                .find_node_by_id(&candidate.edge.get_to_node());
            if to_node_index.is_none() {
                error!("to node not found by id {}", candidate.edge.get_to_node());
                continue;
            }
            // 构建虚拟的node 并添加到图中
            // a--->b     a-->cs--->b
            let dummy_node_id = format!(
                "{:.16}-{:.16}-{:.16}-{:.16}",
                candidate.closest_point.0,
                candidate.closest_point.1,
                traj[0].point.0,
                traj[0].point.1
            );
            // self.road_graph.network.add_node(dummy_node_id.clone())?;
            // 生成一个虚拟边
            use uuid::Uuid;
            let edge = Edge::new(
                Uuid::new_v4().to_string(),
                candidate.edge.get_from_node().clone(),
                dummy_node_id.clone(),
                candidate.offset,
                candidate.offset,
                EdgeType::Dummy,
                "dummy_edge".to_string(),
                candidate.edge.get_geometry().clone(),
            );
            self.road_graph.update_graph(edge)?;
            // 生成cs---->b 这个虚拟边
            let edge = Edge::new(
                Uuid::new_v4().to_string(),
                dummy_node_id.clone(),
                candidate.edge.get_to_node().clone(),
                candidate.edge.get_length() - candidate.offset,
                candidate.edge.get_length() - candidate.offset,
                EdgeType::Dummy,
                "dummy_edge".to_string(),
                candidate.edge.get_geometry().clone(),
            );
            self.road_graph.update_graph(edge)?;
            // 计算输出概率
            candidate.ep = self.calc_ep(candidate.distance, cfg.gps_err);
            let ep = candidate.ep;
            cur_layers.push(Rc::new(RefCell::new(Layer {
                candidate: Some(candidate),
                prev_layer: RefCell::new(None),
                cumulative_prob: ep.ln(),
                tp: 0.0,
            })));
        }
        layer_lists.push(cur_layers);

        for (index, trj) in (&traj[1..]).iter().enumerate() {
            let mut cur_layers = Layers::new();
            let prev_max_condate = MMatch::max_prob_candidate(&layer_lists.last().unwrap());
            let prev_candiate = if prev_max_condate.is_none() {
                None
            } else {
                Some(Rc::new(prev_max_condate.unwrap()))
            };
            let condicates =
                self.query_candidate(trj, cfg.radius, cfg.knn, cfg.gps_err, prev_candiate);
            if condicates.is_empty() {
                warn!("no candidate found in {} gps point", index + 1);
                layer_lists.push(cur_layers);
                continue;
            }
            for cs in condicates {
                cur_layers.push(Rc::new(RefCell::new(Layer {
                    candidate: Some(cs),
                    prev_layer: RefCell::new(None),
                    cumulative_prob: f64::MIN,
                    tp: 0.0,
                })));
            }
            let prev_layers = layer_lists.last().unwrap();
            for prev_layer in prev_layers {
                for cur_layer in cur_layers.iter_mut() {
                    // let cur_condidate = cur_layer.borrow().candidate.as_ref().unwrap();
                    let cur = cur_layer.borrow();
                    let cur_condidate = cur.candidate.as_ref().unwrap();
                    // 更新虚拟图
                    let from_node_index = self
                        .road_graph
                        .network
                        .find_node_by_id(&cur_condidate.edge.get_from_node());
                    if from_node_index.is_none() {
                        error!(
                            "from node not found by id {}",
                            cur_condidate.edge.get_from_node()
                        );
                        continue;
                    }
                    let to_node_index = self
                        .road_graph
                        .network
                        .find_node_by_id(&cur_condidate.edge.get_to_node());
                    if to_node_index.is_none() {
                        error!(
                            "to node not found by id {}",
                            cur_condidate.edge.get_to_node()
                        );
                        continue;
                    }
                    // 构建虚拟的node 并添加到图中
                    // a--->b     a-->cs--->b
                    let dummy_node_id = format!(
                        "{:.16}-{:.16}-{:.16}-{:.16}",
                        cur_condidate.closest_point.0,
                        cur_condidate.closest_point.1,
                        trj.point.0,
                        trj.point.1
                    );
                    // 生成一个虚拟边
                    use uuid::Uuid;
                    let edge = Edge::new(
                        Uuid::new_v4().to_string(),
                        cur_condidate.edge.get_from_node().clone(),
                        dummy_node_id.clone(),
                        cur_condidate.offset,
                        cur_condidate.offset,
                        EdgeType::Dummy,
                        "dummy_edge".to_string(),
                        cur_condidate.edge.get_geometry().clone(),
                    );
                    self.road_graph.update_graph(edge)?;
                    // 生成cs---->b 这个虚拟边
                    let edge = Edge::new(
                        Uuid::new_v4().to_string(),
                        dummy_node_id.clone(),
                        cur_condidate.edge.get_to_node().clone(),
                        cur_condidate.edge.get_length() - cur_condidate.offset,
                        cur_condidate.edge.get_length() - cur_condidate.offset,
                        EdgeType::Dummy,
                        "dummy_edge".to_string(),
                        cur_condidate.edge.get_geometry().clone(),
                    );
                    self.road_graph.update_graph(edge)?;

                    // 此时还需要判断前一层的候选者是否和当前候选者在同一个边上 如果在同一个边上 还需要构建两者都联通关系 否着路径查询时 两者关联不上
                    let prev_candidate_edge_id = if prev_layer.borrow().candidate.is_some() {
                        prev_layer
                            .borrow()
                            .candidate
                            .as_ref()
                            .unwrap()
                            .edge
                            .get_edge_id()
                    } else {
                        "".to_string()
                    };
                    if prev_candidate_edge_id == cur_condidate.edge.get_edge_id() {
                        let mut edge = Edge::new(
                            Uuid::new_v4().to_string(),
                            prev_layer
                                .borrow()
                                .candidate
                                .as_ref()
                                .unwrap()
                                .edge
                                .get_from_node()
                                .clone(),
                            dummy_node_id.clone(),
                            cur_condidate.offset
                                - prev_layer.borrow().candidate.as_ref().unwrap().offset,
                            cur_condidate.offset
                                - prev_layer.borrow().candidate.as_ref().unwrap().offset,
                            EdgeType::Dummy,
                            "dummy_edge".to_string(),
                            prev_layer
                                .borrow()
                                .candidate
                                .as_ref()
                                .unwrap()
                                .edge
                                .get_geometry()
                                .clone(),
                        );
                        let mut updata_graph_ok = false;
                        if edge.get_length() > 0.0 {
                            updata_graph_ok = true;
                        } else if prev_layer.borrow().candidate.as_ref().unwrap().offset
                            - cur_condidate.offset
                            < cfg.reverse_tolerance
                        {
                            edge.set_ori_length(0.0000000001);
                            updata_graph_ok = true;
                        }
                        if updata_graph_ok {
                            self.road_graph.update_graph(edge)?;
                        }
                    }

                    // 计算前一个GPS 点和当前gps点之间的距离
                    let gps_distance = algorithm::eu_distance(
                        &prev_layer
                            .borrow()
                            .candidate
                            .as_ref()
                            .unwrap()
                            .ori_traj_point
                            .point,
                        &cur_condidate.ori_traj_point.point,
                    );
                    // 给出两个轨迹点最大距离限制
                    let max_gps_distance = if trj.time_stamp == 0 || traj[index - 1].time_stamp == 0
                    {
                        gps_distance * cfg.factor * 4.0
                    } else {
                        let time_diff = trj.time_stamp - traj[index - 1].time_stamp;
                        cfg.v_max * cfg.factor * time_diff as f64
                    };
                    // 查询最短路径
                    let prev = prev_layer.borrow();
                    let prev_candidate = prev.candidate.as_ref().unwrap();
                    // let prev_candidate = prev_layer.borrow().candidate.unwrap();
                    let prev_node_id = format!(
                        "{:.16}-{:.16}-{:.16}-{:.16}",
                        prev_candidate.closest_point.0,
                        prev_candidate.closest_point.1,
                        prev_candidate.ori_traj_point.point.0,
                        prev_candidate.ori_traj_point.point.1
                    );
                    let prev_node_index = self.road_graph.network.find_node_by_id(&prev_node_id);
                    if prev_node_index.is_none() {
                        error!("prev_node_index {} is none", prev_node_id);
                        continue;
                    }
                    let cur_node_index = self.road_graph.network.find_node_by_id(&dummy_node_id);
                    if cur_node_index.is_none() {
                        error!("cur_node_index {} is none", dummy_node_id);
                        continue;
                    }

                    let mut candidate_distance = self
                        .road_graph
                        .short_path(prev_node_index.unwrap(), cur_node_index.unwrap())
                        .unwrap_or(f64::MAX);
                    if candidate_distance > max_gps_distance {
                        candidate_distance = f64::MAX;
                    }
                    let tp = self.calc_tp(gps_distance, candidate_distance);
                    // 累积概率
                    let cumu_prob =
                        prev_layer.borrow().cumulative_prob + cur_condidate.ep.ln() + tp.ln();
                    // 如果该累及概率大于目前的累积概率 则更新当前层

                    // let lay = prev_layer.as_ref().as_ptr();
                    if cumu_prob > cur_layer.borrow().cumulative_prob {
                        cur_layer.borrow_mut().cumulative_prob = cumu_prob;
                        cur_layer.borrow_mut().tp = tp;
                        *cur_layer.borrow_mut().prev_layer.borrow_mut() = Some(prev_layer.clone());
                    }
                }
            }
            layer_lists.push(cur_layers);

            // todo:: 打印当前最大累积概率
        }

        // 回溯
        Ok(self.back_tracking(&layer_lists))
    }

    // 回溯
    fn back_tracking(&self, layer_lists: &LayerLists) -> MMResult {
        let mut result = MMResult {
            o_path: vec![],
            matched_candidates: vec![],
        };

        let mut prev_layer: Option<Rc<RefCell<Layer>>> = None;
        let last_layers = &layer_lists[layer_lists.len() - 1];
        if last_layers.is_empty() {
            prev_layer = None;
            result.o_path.push("".to_string());
            result.matched_candidates.push(None);
        } else {
            let max_layer = last_layers.iter().max_by(|a, b| {
                a.borrow()
                    .cumulative_prob
                    .partial_cmp(&b.borrow().cumulative_prob)
                    .unwrap()
            });
            let max_layer = max_layer.unwrap().as_ref().borrow();
            result
                .o_path
                .push(max_layer.candidate.as_ref().unwrap().edge.get_edge_id());
            result
                .matched_candidates
                .push(Some(max_layer.candidate.as_ref().unwrap().clone()));
            prev_layer = if max_layer.prev_layer.borrow().is_none() {
                None
            } else {
                let prev = max_layer.prev_layer.borrow();
                Some(prev.as_ref().unwrap().clone())
            };
        }
        for i in (0..layer_lists.len() - 1).rev() {
            let cur_layer = prev_layer.as_ref();
            if cur_layer.is_none() {
                let layers = &layer_lists[i];
                let max_layer = layers.iter().max_by(|a, b| {
                    a.borrow()
                        .cumulative_prob
                        .partial_cmp(&b.borrow().cumulative_prob)
                        .unwrap()
                });
                let max_layer = max_layer.unwrap().as_ref().borrow();
                result
                    .o_path
                    .push(max_layer.candidate.as_ref().unwrap().edge.get_edge_id());
                result
                    .matched_candidates
                    .push(Some(max_layer.candidate.as_ref().unwrap().clone()));
                prev_layer = if max_layer.prev_layer.borrow().is_none() {
                    None
                } else {
                    let prev = max_layer.prev_layer.borrow();
                    Some(prev.as_ref().unwrap().clone())
                };
                continue;
            }
            let cur = cur_layer.unwrap().clone();
            let edge_id = cur.borrow().candidate.as_ref().unwrap().edge.get_edge_id();
            let can = cur.as_ref().borrow().candidate.as_ref().unwrap().clone();
            result.o_path.push(edge_id);
            result.matched_candidates.push(Some(can));
            prev_layer = if cur.as_ref().borrow().prev_layer.borrow().is_none() {
                None
            } else {
                let cur_ref = cur.as_ref().borrow();
                let prev = cur_ref.prev_layer.borrow();
                Some(prev.as_ref().unwrap().clone())
            };
        }

        result.o_path.reverse();
        result.matched_candidates.reverse();
        result
    }
}
