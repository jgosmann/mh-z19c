use co2_metrics_exporter::mh_z19c::{self, MhZ19C};
use prometheus::{self, Encoder};
use protobuf;
use rppal::uart::{Parity, Uart};
use std::sync::Arc;
use tide;
use tokio::sync::{watch, Notify};
use tokio::task;

async fn co2_sensing_worker(
    start_measurement: Arc<Notify>,
    value_sender: watch::Sender<u16>,
) -> Result<(), mh_z19c::Error<rppal::uart::Error>> {
    let mut co2sensor = MhZ19C::new(
        Uart::with_path("/dev/ttyAMA0", 9600, Parity::None, 8, 1)
            .map_err(mh_z19c::Error::UartError)?,
    );
    loop {
        start_measurement.notified().await;
        loop {
            match co2sensor.read_co2_ppm() {
                Ok(value) => {
                    match value_sender.send(value) {
                        Ok(_) => break,
                        Err(_) => (), // FIXME is this desired?
                    }
                }
                Err(nb::Error::Other(err)) => return Err(err),
                Err(nb::Error::WouldBlock) => (),
            }
        }
    }
}

async fn serve_metrics(req: tide::Request<(Arc<Notify>, watch::Receiver<u16>)>) -> tide::Result {
    req.state().0.notify_one();
    match req.state().1.clone().changed().await {
        Ok(()) => {
            let mut metric_family = prometheus::proto::MetricFamily::new();
            metric_family.set_name("co2_ppm".into());
            metric_family.set_help("CO2 concnentration [ppm]".into());
            metric_family.set_field_type(prometheus::proto::MetricType::GAUGE);
            let mut gauge = prometheus::proto::Gauge::new();
            gauge.set_value(*req.state().1.borrow() as f64);
            let mut metric = prometheus::proto::Metric::new();
            metric.set_gauge(gauge);
            metric_family.set_metric(protobuf::RepeatedField::from_slice(&[metric]));

            let mut buffer = vec![];
            let encoder = prometheus::TextEncoder::new();
            encoder.encode(&[metric_family], &mut buffer)?;
            Ok(format!("{}", String::from_utf8(buffer)?).into())
        }
        Err(err) => Err(tide::Error::from_str(
            tide::StatusCode::InternalServerError,
            err,
        )),
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (tx, rx) = watch::channel(0u16);
    let start_measurement = Arc::new(Notify::new());
    let start_measurement2 = start_measurement.clone();

    task::spawn(async move { co2_sensing_worker(start_measurement2, tx).await });

    let mut app = tide::with_state((start_measurement, rx));
    app.at("/metrics").get(serve_metrics);
    println!("Spawning server ...");
    app.listen("0.0.0.0:9119").await?;
    println!("Shutdown.");

    Ok(())
}
