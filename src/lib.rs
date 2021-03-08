#[macro_use]
extern crate lazy_static;

use crate::command::Command;
use crate::nb_comm::{NbFuture, WriteAndReadResponse};
use core::convert::TryInto;
use embedded_hal::serial::{Read, Write};

pub mod command;
mod nb_comm;

enum MhZ19CState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    Idle(Option<U>),
    ReadCo2(WriteAndReadResponse<'a, U, E>),
}

lazy_static! {
    static ref READ_CO2: [u8; 9] = frame_command(Command::ReadCo2);
}

fn frame_command(command: Command) -> [u8; 9] {
    let mut buf = [0xff, 0x01, 0, 0, 0, 0, 0, 0, 0];
    buf[2..8].copy_from_slice(&command.serialize());
    buf[8] = checksum(&buf[1..8]);
    buf
}

fn checksum(buf: &[u8]) -> u8 {
    buf.iter()
        .fold(0x00, |acc: u8, &x: &u8| acc.overflowing_sub(x).0)
}

pub struct MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    state: MhZ19CState<'a, U, E>,
}

impl<'a, U, E> MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    pub fn new(uart: U) -> Self {
        Self {
            state: MhZ19CState::Idle(Some(uart)),
        }
    }

    pub fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>> {
        if let MhZ19CState::Idle(uart) = &mut self.state {
            self.state = MhZ19CState::ReadCo2(WriteAndReadResponse::new(
                uart.take().unwrap(),
                &*READ_CO2,
                9,
            ));
        }
        if let MhZ19CState::ReadCo2(future) = &mut self.state {
            match future.poll() {
                Ok((uart, buf)) => {
                    self.state = MhZ19CState::Idle(Some(uart));
                    let data = Self::unpack_return_frame(Command::ReadCo2, &buf)
                        .map_err(nb::Error::Other)?;
                    Ok(u16::from_be_bytes(data[..2].try_into().unwrap()))
                }
                Err(err) => Err(err.map(Error::UartError)),
            }
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn unpack_return_frame<'b>(command: Command, buf: &'b [u8]) -> Result<&'b [u8], E> {
        if buf[0] != 0xff {
            return Err(Error::StartByteExpected { got: buf[0] });
        }
        if checksum(&buf[1..8]) != buf[8] {
            println!("buf {:x?} {:x?}", checksum(&buf[1..8]), buf[8]);
            return Err(Error::InvalidChecksum);
        }
        if buf[1] != command.op_code() {
            return Err(Error::OpCodeMismatch {
                expected: command.op_code(),
                got: buf[1],
            });
        }
        Ok(&buf[2..8])
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

    use nb::block;
    use std::collections::VecDeque;

    struct MockUart {
        response: VecDeque<u8>,
        write_buffer: Vec<u8>,
        return_start_byte: u8,
        return_checksum: u8,
    }

    impl Default for MockUart {
        fn default() -> Self {
            Self {
                response: VecDeque::new(),
                write_buffer: vec![],
                return_start_byte: 0xff,
                return_checksum: 0x43,
            }
        }
    }

    impl MockUart {
        fn with_read_co2_response(mut self) -> Self {
            self.response = VecDeque::from(vec![
                self.return_start_byte,
                0x86,
                0x03,
                0x20,
                0x12,
                0x34,
                0x56,
                0x78,
                self.return_checksum,
            ]);
            self
        }
        fn with_return_start_byte(mut self, return_start_byte: u8) -> Self {
            self.return_start_byte = return_start_byte;
            self
        }
        fn with_return_checksum(mut self, return_checksum: u8) -> Self {
            self.return_checksum = return_checksum;
            self
        }
    }

    impl Read<u8> for MockUart {
        type Error = String;

        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            match self.response.pop_front() {
                Some(c) => Ok(c),
                None => Err(nb::Error::Other("No more data.".into())),
            }
        }
    }

    impl Write<u8> for MockUart {
        type Error = String;

        fn write(&mut self, c: u8) -> nb::Result<(), Self::Error> {
            self.write_buffer.push(c);
            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn test_read_co2() {
        let mut co2sensor = MhZ19C::new(MockUart::default().with_read_co2_response());
        assert_eq!(block!(co2sensor.read_co2_ppm()), Ok(800));
    }

    #[test]
    fn test_read_co2_uart_error() {
        let mut co2sensor = MhZ19C::new(MockUart::default());
        assert_eq!(
            block!(co2sensor.read_co2_ppm()),
            Err(Error::UartError("No more data.".into()))
        );
    }

    #[test]
    fn test_read_co2_invalid_start_byte() {
        let uart = MockUart::default()
            .with_return_start_byte(0x00)
            .with_read_co2_response();
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            block!(co2sensor.read_co2_ppm()),
            Err(Error::StartByteExpected { got: 0x00 })
        );
    }

    #[test]
    fn test_read_co2_invalid_checksum() {
        let uart = MockUart::default()
            .with_return_checksum(0x00)
            .with_read_co2_response();
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            block!(co2sensor.read_co2_ppm()),
            Err(Error::InvalidChecksum)
        );
    }
}
