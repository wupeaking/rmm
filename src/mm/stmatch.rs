use super::model::Candidate;
use crate::algorithm;
use crate::graph::network;
use crate::graph::RoadGraph;
use anyhow;
use geojson;
use log::{debug, error, info};
use rtree_rs::{RTree, Rect};
use std::borrow::Borrow;
use std::clone;
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
}
