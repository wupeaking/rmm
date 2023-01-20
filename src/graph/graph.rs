use super::network::Network;
use crate::graph::Edge;
use anyhow::Result;
use log::{debug, error, info};
use petgraph::graph::{Graph, NodeIndex};
use petgraph::Directed;
pub struct RoadGraph {
    pub network: Network,
    graph: Graph<f64, f64, Directed, usize>,
}

impl RoadGraph {
    pub fn new(network: Network) -> Self {
        info!("start construct graph...");
        let node_size = network.nodes.len();
        let edge_size = network.edges.len();
        debug!("node size: {}, edge size: {}", node_size, edge_size);
        let mut gh = RoadGraph {
            network,
            graph: Graph::with_capacity(node_size, edge_size),
        };
        // 添加node
        for (_, _) in gh.network.nodes.iter().enumerate() {
            // let Some(node) = gh.network.find_node_by_index(index);
            gh.graph.add_node(1.0);
        }

        // 添加edge
        for (index, _) in gh.network.edges.iter().enumerate() {
            let edge = gh.network.find_edge_by_index(index).unwrap();
            let from = gh.network.find_node_by_id(&edge.get_from_node()).unwrap();
            let to = gh.network.find_node_by_id(&edge.get_to_node()).unwrap();
            gh.graph
                .add_edge(NodeIndex::new(from), NodeIndex::new(to), edge.get_length());
        }
        info!("finish construct origin road network graph...");
        gh
    }

    pub fn update_graph(&mut self, edge: Edge) -> Result<()> {
        // 检查from_node 是否存在
        let from_node_index: NodeIndex<usize>;
        if let Some(from) = self.network.find_node_by_id(&edge.get_from_node()) {
            from_node_index = NodeIndex::new(from);
        } else {
            from_node_index = self.graph.add_node(1.0);
            self.network.add_node(edge.get_from_node().clone())?;
        }
        // 进行一次校验
        if self.network.nodes.len() != self.graph.node_count() {
            error!("node size not match");
            return Err(anyhow::anyhow!("node size not match"));
        }
        // 检查to_node 是否存在
        let to_node_index: NodeIndex<usize>;
        if let Some(to) = self.network.find_node_by_id(&edge.get_to_node()) {
            to_node_index = NodeIndex::new(to);
        } else {
            to_node_index = self.graph.add_node(1.0);
            self.network.add_node(edge.get_to_node().clone())?;
        }
        self.graph
            .add_edge(from_node_index, to_node_index, edge.get_length());
        Ok(())
    }

    pub fn short_path(&self, from: usize, to: usize) -> Result<f64> {
        use petgraph::algo::dijkstra;
        let result = dijkstra(
            &self.graph,
            NodeIndex::new(from),
            Some(NodeIndex::new(to)),
            |e| *e.weight(),
        );
        for (node, weight) in &result {
            debug!("node: {:?}, weight: {:?}", node, weight);
        }
        if !result.contains_key(&NodeIndex::new(to)) {
            return Err(anyhow::anyhow!("no path"));
        }
        Ok(result[&NodeIndex::new(to)])
    }

    pub fn short_k_path(
        &self,
        from: NodeIndex<usize>,
        to: NodeIndex<usize>,
        k: usize,
    ) -> Result<f64> {
        use petgraph::algo::k_shortest_path;
        let result = k_shortest_path(&self.graph, from, Some(to), k, |e| *e.weight());
        for (node, weight) in &result {
            debug!("node: {:?}, weight: {:?}", node, weight);
        }
        if !result.contains_key(&to) {
            return Err(anyhow::anyhow!("no path"));
        }
        Ok(result[&to])
    }
}

// test
#[cfg(test)]
mod test {}
