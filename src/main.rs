use clap::Parser;
use log::{debug, info};
use rmm::mm::model;
use rmm::mm::stmatch;
use rmm::mm::traj;
use rmm::utils;
use std::fs;
use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use wkt;
#[derive(Parser, Debug)]
#[command(name = "RMM")]
#[command(author = "pengxin.wu <wupeaking@gmail.com>")]
#[command(version = "0.1")]
#[command(about = "fast map matching using rust", long_about = None)]
struct Args {
    /// gps tolerance scope
    #[arg(long, default_value_t = 0.0001)]
    gps_err: f64,
    /// find candidate points in radius
    #[arg(long, default_value_t = 0.01)]
    radius: f64,
    /// selct k nearest candidate
    #[arg(long, default_value_t = 4)]
    knn: u16,
    /// max speed
    #[arg(long, default_value_t = 30.0)]
    max_speed: f64,
    /// factor for speed
    #[arg(long, default_value_t = 4.0)]
    factor: f64,
    /// reverse tolerance
    #[arg(long, default_value_t = 4.0)]
    reverse_tolerance: f64,
    /// road network path
    #[arg(short, long, value_name = "ROAD_NETWORK_FILE")]
    network_file: String,

    /// input  gps traj input
    #[arg(value_name = "GPS_TRAJ_FILE")]
    input_file: PathBuf,
}

fn main() {
    utils::log::log_init();
    let args = Args::parse();
    debug!("{:?}", args);

    info!("try constarct map matching network from geojson file");
    let mut map_match = stmatch::MMatch::try_from(args.network_file.clone())
        .expect("constarct map matching network failed: ");

    // 读取gps轨迹
    // let file = File::open(args.input_file).unwrap();
    // 检查文件后缀
    let ext = args.input_file.extension().unwrap();

    let mutile_gps_trajs = if ext == "geojson" {
        info!("read geojson file {} ", args.input_file.display());
        let file = File::open(args.input_file).unwrap();
        let reader = BufReader::new(file);
        let geojson = geojson::GeoJson::from_reader(reader).unwrap();
        let gps_trajs = traj::MutileTrajectory::try_from(geojson).expect("read gps traj failed: ");
        gps_trajs
    } else if ext == "wkt" {
        info!("read wkt file {} ", args.input_file.display());
        let content = fs::read_to_string(args.input_file).expect("read wkt file failed: ");
        // let wkt: wkt::Wkt<f64> = wkt::Wkt::from_str(&content).unwrap();
        let wkt: wkt::Wkt<f64> = content.as_str().parse().expect("read gps traj failed: ");
        let gps_trajs: traj::MutileTrajectory = wkt.try_into().expect("read gps traj failed: ");
        gps_trajs
    } else {
        panic!("gps file format not support");
    };

    // 配置
    let config = model::Config {
        gps_err: args.gps_err,
        radius: args.radius,
        v_max: args.max_speed,
        factor: args.factor,
        reverse_tolerance: args.reverse_tolerance,
        knn: args.knn,
        road_netwok_path: args.network_file.clone(),
    };

    for gps_traj in mutile_gps_trajs.trajs.iter() {
        let mm_result = map_match
            .match_traj(gps_traj, &config)
            .expect("msg match failed: ");
        info!("match edges: {:?}", mm_result.o_path);
    }
}
