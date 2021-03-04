use rppal::uart::{Parity, Uart};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Opening device");
    let mut uart = Uart::with_path("/dev/ttyAMA0", 9600, Parity::None, 8, 1)?;
    println!("Writing command");
    uart.write(&[0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79])?;
    println!("Waiting for drain");
    uart.drain()?;
    let mut buffer = [0x00; 9];
    println!("Set read mode");
    uart.set_read_mode(9, Duration::default())?;
    println!("Reading");
    uart.read(&mut buffer)?;
    println!("{:x?}", buffer);
    println!("Calculating checksum");
    let checksum = buffer
        .iter()
        .take(7)
        .fold(0xff, |acc: u8, &x: &u8| acc.overflowing_sub(x).0);
    let is_checksum_valid: bool = checksum == buffer[8];
    println!("checksum: {:x}", checksum);
    println!("start byte: {:x}", buffer[0]);
    println!("command byte: {:x}", buffer[1]);
    println!("checksum valid: {}", is_checksum_valid);
    println!(
        "CO2 concentration: {}",
        (buffer[2] as u16) << 8 | buffer[3] as u16
    );

    Ok(())
}
