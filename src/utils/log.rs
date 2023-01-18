pub fn log_init() {
    use std::io::Write;
    // env_logger 通过环境变量 RUST_LOG 控制日志输出
    std::env::set_var("RUST_LOG", "debug");
    env_logger::builder()
        .target(env_logger::Target::Stdout)
        .format(|buf, record| writeln!(buf, "{}: {}", record.level(), record.args()))
        .init();
    // log::set_max_level(log::LevelFilter::Debug);
}
