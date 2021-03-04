use super::Uart as UartInterface;
use rppal::uart::Uart as RPiUart;
use std::time::{Duration, Instant};

pub struct Uart {
    uart: RPiUart,
    read_timeout: Option<Duration>,
}

impl Uart {
    pub fn new(
        mut uart: RPiUart,
        read_timeout: Option<Duration>,
    ) -> Result<Self, rppal::uart::Error> {
        uart.set_read_mode(0, read_timeout.unwrap_or_default())?;
        Ok(Self { uart, read_timeout })
    }
}

impl UartInterface for Uart {
    type Error = rppal::uart::Error;

    fn read_blocking(&mut self, buffer: &mut [u8], len: usize) -> Result<(), Self::Error> {
        let timeout_instant = self.read_timeout.map(|t| Instant::now() + t);
        let mut bytes_read = 0;
        while bytes_read < len {
            bytes_read += self.uart.read(&mut buffer[bytes_read..])?;

            if let Some(timeout_instant) = timeout_instant {
                if Instant::now() > timeout_instant {
                    return Err(rppal::uart::Error::Io(std::io::Error::new(
                        std::io::ErrorKind::TimedOut,
                        "Time out while waiting for data on UART.",
                    )));
                }
            }
        }
        Ok(())
    }

    fn write_blocking(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.uart.write(buffer)?;
        self.uart.drain()?;
        Ok(())
    }
}
