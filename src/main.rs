use co2_metrics_exporter::mh_z19c::{rpi::Uart, MhZ19C};
use rppal::uart::{Parity, Uart as UartDevice};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Opening device");
    let mut co2sensor = MhZ19C::new(Uart::new(
        UartDevice::with_path("/dev/ttyAMA0", 9600, Parity::None, 8, 1)?,
        Some(Duration::from_secs(2)),
    )?);
    println!("CO2 concentration: {}", co2sensor.read_co2_ppm()?);
    Ok(())
}
