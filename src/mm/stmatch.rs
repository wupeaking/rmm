use super::model::{Candidate, Config, Layer, LayerLists, Layers, MMResult, Trajectory};
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
        point: algorithm::Point,
        radius: f64,
        knn: u16,
        gps_err: f64,
        prev_candidate: Option<Rc<Candidate>>,
    ) -> Vec<Candidate> {
        // 构建bbox
        let bbox = Rect::new(
            [point.0 - radius, point.1 - radius],
            [point.0 + radius, point.1 + radius],
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
                        algorithm::linear_reference_distance(point, &line.0);
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

    fn calc_tp(gps_dist: f64, candidate_dist: f64) -> f64 {
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
            if layer.candidate.is_none() {
                continue;
            }
            if layer.cumulative_prob > max_prob {
                max_prob = layer.cumulative_prob;
                max_prob_candidate = Some(layer.candidate.as_ref().unwrap().clone());
            }
        }
        max_prob_candidate
    }
}

impl MMatch {
    fn match_traj(&mut self, traj: &Trajectory, cfg: &Config) -> anyhow::Result<MMResult> {
        if traj.len() == 0 {
            return Err(anyhow::anyhow!("trajectory is empty"));
        }
        let mut layer_lists = LayerLists::new();

        let condicates =
            self.query_candidate(traj[0].point, cfg.radius, cfg.knn, cfg.gps_err, None);
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
            cur_layers.push(Layer {
                candidate: Some(candidate),
                prev_layer: RefCell::new(None),
                cumulative_prob: ep.ln(),
                tp: 0.0,
            });
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
                self.query_candidate(trj.point, cfg.radius, cfg.knn, cfg.gps_err, prev_candiate);
            if condicates.is_empty() {
                warn!("no candidate found in {} gps point", index + 1);
                layer_lists.push(cur_layers);
                continue;
            }
            for cs in condicates {
                cur_layers.push(Layer {
                    candidate: Some(cs),
                    prev_layer: RefCell::new(None),
                    cumulative_prob: f64::MIN,
                    tp: 0.0,
                })
            }
            let prev_layers = layer_lists.last().unwrap();
            for prev_layer in prev_layers {
                for cur_layer in cur_layers.iter_mut() {
                    let cur_condidate = cur_layer.candidate.as_ref().unwrap();
                }
            }
        }

        Ok(MMResult {
            o_path: vec![],
            matched_candidates: vec![],
        })
    }
}
