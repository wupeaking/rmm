use anyhow::Result;
use geo::algorithm::haversine_distance::HaversineDistance;
use geojson::{Geometry, Value};

#[derive(Clone, Copy)]
pub struct Point(pub f64, pub f64);
pub struct Line(pub Vec<[f64; 2]>);

impl TryInto<Line> for geojson::Geometry {
    type Error = anyhow::Error;
    fn try_into(self) -> Result<Line> {
        match self.value {
            Value::LineString(line_string) => {
                let mut line = Vec::new();
                for point in line_string {
                    line.push([point[0], point[1]]);
                }
                Ok(Line(line))
            }
            _ => Err(anyhow::anyhow!("geometry is not linestring")),
        }
    }
}

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

// 点到直线的最短距离
// 返回结果 最短距离, 偏移距离, 偏移点
// (min_distance, offset, offset_point)
pub fn linear_reference_distance(point: Point, line: &[[f64; 2]]) -> (f64, f64, Point) {
    let mut min_distance = f64::MAX;
    let mut length_parsed = 0 as f64;
    let mut final_offset = f64::MAX;
    let mut final_offset_point = Point(0.0, 0.0);
    let line1 = &line[0..line.len() - 1];
    let line2 = &line[1..line.len()];
    for (p1, p2) in line1.iter().zip(line2.iter()) {
        let (distance, offfset, offset_point) =
            point_to_line_distance(point, Point(p1[0], p1[1]), Point(p2[0], p2[1]));
        if distance < min_distance {
            min_distance = distance;
            final_offset = length_parsed + offfset;
            final_offset_point = offset_point;
        }
        length_parsed += ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt();
    }
    (min_distance, final_offset, final_offset_point)
}

// 返回值 (distance, offfset, close_point)
fn point_to_line_distance(point: Point, start: Point, end: Point) -> (f64, f64, Point) {
    let length = ((start.0 - end.0).powi(2) + (start.1 - end.1).powi(2)).sqrt();
    if length == 0.0 {
        return (
            ((point.0 - start.0).powi(2) + (point.1 - start.1).powi(2)).sqrt(),
            0.0,
            start,
        );
    }
    // 利用向量的点积计算点到直线的距离
    let vec_start_point = (point.0 - start.0, point.1 - start.1);
    let vec_start_end = (end.0 - start.0, end.1 - start.1);
    let dot = vec_start_point.0 * vec_start_end.0 + vec_start_point.1 * vec_start_end.1;
    let radio = dot / length * length;
    let radio = if radio > 1 as f64 {
        1.0
    } else if radio < 0 as f64 {
        0.0
    } else {
        radio
    };
    // 投影点坐标
    let offset_point = Point(
        start.0 + radio * vec_start_end.0,
        start.1 + radio * vec_start_end.1,
    );
    // 偏移距离
    let offset = ((start.0 - offset_point.0).powi(2) + (start.1 - offset_point.1).powi(2)).sqrt();
    // 最短距离
    let min_distance =
        ((point.0 - offset_point.0).powi(2) + (point.1 - offset_point.1).powi(2)).sqrt();
    (min_distance, offset, offset_point)
}
