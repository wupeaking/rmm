use crate::algorithm;
use anyhow;
use geojson;
use wkt;

#[derive(Debug, Clone)]
pub struct TrajInfo {
    pub point: algorithm::Point,
    pub time_stamp: u64,
}

pub type Trajectory = Vec<TrajInfo>;

pub struct MutileTrajectory {
    pub trajs: Vec<Trajectory>,
}

// 实现轨迹转换trait
impl TryFrom<geojson::GeoJson> for MutileTrajectory {
    type Error = anyhow::Error;
    fn try_from(value: geojson::GeoJson) -> Result<Self, Self::Error> {
        let mut trajs = MutileTrajectory { trajs: Vec::new() };
        match value {
            geojson::GeoJson::FeatureCollection(fc) => {
                for feature in fc.features {
                    match feature.geometry.unwrap().value {
                        geojson::Value::LineString(line) => {
                            let mut traj = Trajectory::new();
                            for (i, point) in line.into_iter().enumerate() {
                                let time_stamp = i as u64;
                                traj.push(TrajInfo {
                                    point: algorithm::Point(point[0], point[1]),
                                    time_stamp,
                                });
                            }
                            trajs.trajs.push(traj);
                        }
                        geojson::Value::MultiPoint(multi_points) => {
                            let mut traj = Trajectory::new();
                            for (i, point) in multi_points.into_iter().enumerate() {
                                let time_stamp = i as u64;
                                traj.push(TrajInfo {
                                    point: algorithm::Point(point[0], point[1]),
                                    time_stamp,
                                });
                            }
                            trajs.trajs.push(traj);
                        }
                        _ => {
                            return Err(anyhow::anyhow!(
                                "geometry only support Point, LineString, MultiPoint"
                            ));
                        }
                    }
                }
            }
            geojson::GeoJson::Feature(feature) => match feature.geometry.unwrap().value {
                geojson::Value::LineString(line) => {
                    let mut traj = Trajectory::new();
                    for (i, point) in line.into_iter().enumerate() {
                        let time_stamp = i as u64;
                        traj.push(TrajInfo {
                            point: algorithm::Point(point[0], point[1]),
                            time_stamp,
                        });
                    }
                    trajs.trajs.push(traj);
                }
                geojson::Value::MultiPoint(multi_points) => {
                    let mut traj = Trajectory::new();
                    for (i, point) in multi_points.into_iter().enumerate() {
                        let time_stamp = i as u64;
                        traj.push(TrajInfo {
                            point: algorithm::Point(point[0], point[1]),
                            time_stamp,
                        });
                    }
                    trajs.trajs.push(traj);
                }
                _ => {
                    return Err(anyhow::anyhow!(
                        "geometry only support Point, LineString, MultiPoint"
                    ));
                }
            },
            geojson::GeoJson::Geometry(geom) => match geom.value {
                geojson::Value::LineString(line) => {
                    let mut traj = Trajectory::new();
                    for (i, point) in line.into_iter().enumerate() {
                        let time_stamp = i as u64;
                        traj.push(TrajInfo {
                            point: algorithm::Point(point[0], point[1]),
                            time_stamp,
                        });
                    }
                    trajs.trajs.push(traj);
                }
                geojson::Value::MultiPoint(multi_points) => {
                    let mut traj = Trajectory::new();
                    for (i, point) in multi_points.into_iter().enumerate() {
                        let time_stamp = i as u64;
                        traj.push(TrajInfo {
                            point: algorithm::Point(point[0], point[1]),
                            time_stamp,
                        });
                    }
                    trajs.trajs.push(traj);
                }
                _ => {
                    return Err(anyhow::anyhow!(
                        "geometry only support Point, LineString, MultiPoint"
                    ));
                }
            },
        }
        Ok(trajs)
    }
}

impl TryFrom<wkt::Wkt<f64>> for MutileTrajectory {
    type Error = anyhow::Error;
    fn try_from(value: wkt::Wkt<f64>) -> Result<Self, Self::Error> {
        let mut trajs = MutileTrajectory { trajs: Vec::new() };
        match value.item {
            wkt::Geometry::LineString(line) => {
                let mut traj = Trajectory::new();
                for (i, point) in line.0.into_iter().enumerate() {
                    let time_stamp = i as u64;
                    traj.push(TrajInfo {
                        point: algorithm::Point(point.x, point.y),
                        time_stamp,
                    });
                }
                trajs.trajs.push(traj);
            }
            wkt::Geometry::MultiPoint(multi_points) => {
                let mut traj = Trajectory::new();
                for (i, point) in multi_points.0.into_iter().enumerate() {
                    let time_stamp = i as u64;
                    traj.push(TrajInfo {
                        point: algorithm::Point(
                            point.0.as_ref().unwrap().x,
                            point.0.as_ref().unwrap().y,
                        ),
                        time_stamp,
                    });
                }
                trajs.trajs.push(traj);
            }
            _ => {
                return Err(anyhow::anyhow!("wkt only support LineString"));
            }
        }
        Ok(trajs)
    }
}
