use std::convert::TryInto;

pub mod rpi;

pub trait Uart {
    type Error;

    fn read_blocking(
        &mut self,
        buffer: &mut [u8],
        len: usize,
    ) -> std::result::Result<(), Self::Error>;
    fn write_blocking(&mut self, buffer: &[u8]) -> std::result::Result<(), Self::Error>;
}

pub struct MhZ19C<T: Uart> {
    uart: T,
}

impl<T: Uart> MhZ19C<T> {
    pub fn new(uart: T) -> Self {
        Self { uart }
    }

    pub fn read_co2_ppm(&mut self) -> Result<u16, T::Error> {
        let mut buf = [0u8; 9];
        Self::frame_command(Command::ReadCo2, &mut buf);
        self.uart.write_blocking(&buf).map_err(Error::UartError)?;
        self.uart
            .read_blocking(&mut buf, 9)
            .map_err(Error::UartError)?;
        let buf = buf;
        let data = Self::unpack_return_frame(Command::ReadCo2, &buf)?;
        Ok(u16::from_be_bytes(data[..2].try_into().unwrap()))
    }

    fn frame_command(command: Command, buf: &mut [u8; 9]) {
        buf[0] = 0xff;
        buf[1] = 0x01;
        command.serialize(&mut buf[2..8]);
        buf[8] = Self::checksum(&buf[1..8]);
    }

    fn unpack_return_frame<'a>(command: Command, buf: &'a [u8; 9]) -> Result<&'a [u8], T::Error> {
        if buf[0] != 0xff {
            return Err(Error::StartByteExpected { got: buf[0] });
        }
        if Self::checksum(&buf[1..8]) != buf[8] {
            println!("buf {:x?} {:x?}", Self::checksum(&buf[1..8]), buf[8]);
            return Err(Error::InvalidChecksum);
        }
        if buf[1] != command.op_code() {
            return Err(Error::OpCodeMismatch {
                expected: command.op_code(),
                got: buf[1],
            });
        }
        return Ok(&buf[2..8]);
    }

    fn checksum(buf: &[u8]) -> u8 {
        buf.iter()
            .fold(0x00, |acc: u8, &x: &u8| acc.overflowing_sub(x).0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Command {
    ReadCo2,
    SetSelfCalibrate(bool),
}

impl Command {
    fn op_code(&self) -> u8 {
        match self {
            Self::ReadCo2 => 0x86,
            Self::SetSelfCalibrate(_) => 0x79,
        }
    }
    fn serialize_args(&self, buf: &mut [u8]) {
        buf.copy_from_slice(&match self {
            Self::SetSelfCalibrate(true) => [0xa0, 0x00, 0x00, 0x00, 0x00],
            Self::SetSelfCalibrate(false) => [0x00; 5],
            _ => [0x00; 5],
        })
    }
    fn serialize(&self, buf: &mut [u8]) {
        buf[0] = self.op_code();
        self.serialize_args(&mut buf[1..]);
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error<T> {
    StartByteExpected { got: u8 },
    OpCodeMismatch { expected: u8, got: u8 },
    InvalidChecksum,
    UartError(T),
}

impl<T: std::fmt::Display> std::fmt::Display for Error<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::StartByteExpected { got } => {
                write!(f, "Expected start byte 0xff, but got 0x{:x}.", got)
            }
            Self::OpCodeMismatch { expected, got } => write!(
                f,
                "Expected response for op code 0x{:x}, but got op code 0x{:x}.",
                expected, got
            ),
            Self::InvalidChecksum => write!(f, "Checksum mismatch."),
            Self::UartError(err) => write!(f, "UART communication error: {}", err),
        }
    }
}

impl<T: std::fmt::Debug + std::fmt::Display> std::error::Error for Error<T> {}

type Result<T, U> = std::result::Result<T, Error<U>>;

#[cfg(test)]
mod tests {
    use super::*;

    struct MockUart {
        return_start_byte: u8,
        return_checksum: u8,
        uart_error: bool,
    }

    impl Default for MockUart {
        fn default() -> Self {
            Self {
                return_start_byte: 0xff,
                return_checksum: 0x43,
                uart_error: false,
            }
        }
    }

    impl Uart for MockUart {
        type Error = String;

        fn read_blocking(
            &mut self,
            buffer: &mut [u8],
            len: usize,
        ) -> std::result::Result<(), Self::Error> {
            if self.uart_error {
                Err("Mocked UART error.".into())
            } else if len > 9 {
                Err("Tried to read more than 9 bytes.".into())
            } else {
                buffer.copy_from_slice(
                    &[
                        self.return_start_byte,
                        0x86,
                        0x03,
                        0x20,
                        0x12,
                        0x34,
                        0x56,
                        0x78,
                        self.return_checksum,
                    ][..len],
                );
                Ok(())
            }
        }

        fn write_blocking(&mut self, buffer: &[u8]) -> std::result::Result<(), Self::Error> {
            if self.uart_error {
                Err("Mocked UART error.".into())
            } else if buffer == [0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79] {
                Ok(())
            } else {
                Err(format!(
                    "Did not write expected read command, but {:x?}.",
                    buffer
                ))
            }
        }
    }

    #[test]
    fn test_read_co2() {
        let mut co2sensor = MhZ19C::new(MockUart::default());
        assert_eq!(co2sensor.read_co2_ppm(), Ok(800));
    }

    #[test]
    fn test_read_co2_uart_error() {
        let mut uart = MockUart::default();
        uart.uart_error = true;
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            co2sensor.read_co2_ppm(),
            Err(Error::UartError("Mocked UART error.".into()))
        );
    }

    #[test]
    fn test_read_co2_invalid_start_byte() {
        let mut uart = MockUart::default();
        uart.return_start_byte = 0x00;
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            co2sensor.read_co2_ppm(),
            Err(Error::StartByteExpected { got: 0x00 })
        );
    }

    #[test]
    fn test_read_co2_invalid_checksum() {
        let mut uart = MockUart::default();
        uart.return_checksum = 0x00;
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(co2sensor.read_co2_ppm(), Err(Error::InvalidChecksum));
    }
}
