use anyhow::Result;
use geo::algorithm::haversine_distance::HaversineDistance;
use geojson::{Geometry, Value};

/// 计算linestring的半正弦距离
pub fn linestring_distance(geometry: &Geometry) -> Result<f64> {
    match &geometry.value {
        Value::LineString(line_string) => {
            let mut distance = 0.0;
            let line1 = &line_string[0..line_string.len() - 1];
            let line2 = &line_string[1..line_string.len()];
            for (p1, p2) in line1.iter().zip(line2.iter()) {
                use geo::Point;
                distance += Point::new(p1[0], p1[1]).haversine_distance(&Point::new(p2[0], p2[1]));
            }
            Ok(distance)
        }
        _ => Err(anyhow::anyhow!("geometry is not linestring")),
    }
}
