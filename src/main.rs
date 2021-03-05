use co2_metrics_exporter::mh_z19c::MhZ19C;
use nb::block;
use rppal::uart::{Parity, Uart};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Opening device");
    let mut co2sensor = MhZ19C::new(Uart::with_path("/dev/ttyAMA0", 9600, Parity::None, 8, 1)?);
    println!("CO2 concentration: {}", block!(co2sensor.read_co2_ppm())?);
    Ok(())
}
