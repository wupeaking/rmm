use crate::algorithm;
use anyhow::Result;
use geojson::{Geometry, Value};
use std::collections::HashMap;
/**
 * @file network.rs
 * 定义路网信息结构体
 * 从文件中读取路网信息, 并存储在内存中。
 */
pub struct Network {
    edges_index: HashMap<String, usize>, // edge的id对应的索引
    nodes_index: HashMap<String, usize>, // node的id对应的索引
    pub edges: Vec<Edge>,                // 所有的edge
    pub nodes: Vec<String>,              // 所有的node
}

#[derive(Clone)]
pub enum EdgeType {
    Real,  // 真实的edge
    Dummy, // 虚拟的edge
}

#[derive(Clone)]
pub struct Edge {
    id: String,          // edge的id
    from: String,        // edge的起点
    to: String,          // edge的终点
    length: f64,         // edge的长度 米制单位
    ori_length: f64,     // edge的原始长度
    edge_type: EdgeType, // edge的类型
    name: String,        // edge的名字
    geometry: Geometry,  // edge的几何信息
}

impl Edge {
    pub fn new(
        id: String,
        from: String,
        to: String,
        length: f64,
        ori_length: f64,
        edge_type: EdgeType,
        name: String,
        geometry: Geometry,
    ) -> Self {
        Edge {
            id,
            from,
            to,
            length,
            ori_length,
            edge_type,
            name,
            geometry,
        }
    }
}

impl Edge {
    pub fn get_from_node(&self) -> String {
        self.from.clone()
    }

    pub fn get_edge_id(&self) -> String {
        self.id.clone()
    }

    pub fn get_to_node(&self) -> String {
        self.to.clone()
    }

    pub fn get_length(&self) -> f64 {
        self.length
    }

    pub fn get_geometry(&self) -> &geojson::Geometry {
        &self.geometry
    }

    // reurn minx, miny, maxx, maxy
    pub fn get_geom_rect(&self) -> Result<(algorithm::Point, algorithm::Point)> {
        let mut min_lng = 180 as f64;
        let mut min_lat = 90 as f64;
        let mut max_lng = -180 as f64;
        let mut max_lat = -90 as f64;
        match &self.geometry.value {
            Value::LineString(line_string) => {
                for point in line_string {
                    if point[0] < min_lng {
                        min_lng = point[0];
                    }
                    if point[0] > max_lng {
                        max_lng = point[0];
                    }
                    if point[1] < min_lat {
                        min_lat = point[1];
                    }
                    if point[1] > max_lat {
                        max_lat = point[1];
                    }
                }
                Ok((
                    algorithm::Point(min_lng, min_lat),
                    algorithm::Point(max_lng, max_lat),
                ))
            }
            _ => Err(anyhow::anyhow!("geometry is not linestring")),
        }
    }
}

impl TryFrom<geojson::GeoJson> for Network {
    // use anyhow::Error;
    type Error = anyhow::Error;
    fn try_from(value: geojson::GeoJson) -> std::result::Result<Self, Self::Error> {
        let mut edges_index = HashMap::new();
        let mut nodes_index = HashMap::new();
        let mut edges = Vec::new();
        let mut nodes = Vec::new();
        match value {
            geojson::GeoJson::FeatureCollection(feature_collection) => {
                for feature in feature_collection.features {
                    let geometry = feature.geometry.unwrap();
                    let properties = feature.properties.unwrap();
                    let from = properties["from_node_id"].as_i64().unwrap().to_string();
                    let to = properties["to_node_id"].as_i64().unwrap().to_string();
                    let ori_length = properties["length"].as_f64().unwrap();
                    let name = match properties.get("name") {
                        Some(v) => {
                            if v.is_string() {
                                v.as_str().unwrap().to_string()
                            } else {
                                "".to_string()
                            }
                        }
                        None => "".to_string(),
                    };
                    let edge_type = EdgeType::Real;
                    let length = algorithm::linestring_distance(&geometry)?;
                    let id = properties["edge_id"].as_i64().unwrap().to_string();
                    let edge = Edge {
                        id,
                        from,
                        to,
                        length,
                        ori_length,
                        edge_type,
                        name,
                        geometry,
                    };
                    if !nodes_index.contains_key(&edge.from) {
                        nodes.push(edge.from.clone());
                        nodes_index.insert(edge.from.clone(), nodes.len() - 1);
                    }
                    if !nodes_index.contains_key(&edge.to) {
                        nodes.push(edge.to.clone());
                        nodes_index.insert(edge.to.clone(), nodes.len() - 1);
                    }
                    if !edges_index.contains_key(&edge.id) {
                        let key = edge.id.clone();
                        edges.push(edge);
                        edges_index.insert(key, edges.len() - 1);
                    }
                }
            }
            _ => {
                return Err(anyhow::anyhow!("not a feature collection"));
            }
        }
        Ok(Network {
            edges_index,
            nodes_index,
            edges,
            nodes,
        })
    }
}

impl Network {
    pub fn find_edge_by_index(&self, index: usize) -> Option<Edge> {
        if index < self.edges.len() {
            Some(self.edges[index].clone())
        } else {
            None
        }
    }

    pub fn find_edge_by_id(&self, id: &str) -> Option<Edge> {
        if let Some(index) = self.edges_index.get(id) {
            Some(self.edges[*index].clone())
        } else {
            None
        }
    }

    pub fn find_node_by_index(&self, index: usize) -> Option<String> {
        if index < self.nodes.len() {
            Some(self.nodes[index].clone())
        } else {
            None
        }
    }

    pub fn find_node_by_id(&self, id: &str) -> Option<usize> {
        if let Some(index) = self.nodes_index.get(id) {
            Some(*index)
        } else {
            None
        }
    }

    /// 添加node
    pub fn add_node(&mut self, node: String) -> Result<usize> {
        if !self.nodes_index.contains_key(&node) {
            self.nodes.push(node.clone());
            self.nodes_index.insert(node.clone(), self.nodes.len() - 1);
            Ok(self.nodes.len() - 1)
        } else {
            Err(anyhow::anyhow!("node already exists"))
        }
    }

    /// 添加edge
    pub fn add_edge(&mut self, edge: Edge) -> Result<usize> {
        if !self.edges_index.contains_key(&edge.id) {
            self.edges.push(edge.clone());
            self.edges_index
                .insert(edge.id.clone(), self.edges.len() - 1);
            Ok(self.edges.len() - 1)
        } else {
            Err(anyhow::anyhow!("edge already exists"))
        }
    }
}
