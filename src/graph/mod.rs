pub mod network;
pub use network::*;
pub mod graph;
pub use graph::*;

// test
#[cfg(test)]
mod test {
    use super::graph;
    use super::network;
    use crate::utils::log;
    use petgraph::graph::NodeIndex;
    use std::fs::File;
    use std::io::BufReader;

    // 读取geojson文件
    #[test]
    fn test_road_graph_path() {
        log::log_init();
        let file = File::open("edges_shanghai.json").unwrap();
        let reader = BufReader::new(file);
        let geojson = geojson::GeoJson::from_reader(reader).unwrap();
        let network = network::Network::try_from(geojson).unwrap();
        let graph = graph::RoadGraph::new(network);
        let from = graph.network.find_node_by_id("31000028835").unwrap();
        let to = graph.network.find_node_by_id("31000028718").unwrap();
        let result = graph
            .short_path(NodeIndex::new(from), NodeIndex::new(to))
            .unwrap();
        assert_eq!(result, 2630.016424987122);
    }
}
