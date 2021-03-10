#![no_std]

#[macro_use]
extern crate lazy_static;

use crate::command::Command;
use crate::frame::Frame;
use crate::nb_comm::{NbFuture, WriteAndReadResponse};
use core::convert::TryInto;
use embedded_hal::serial::{Read, Write};

pub mod command;
pub mod frame;
mod nb_comm;

enum MhZ19CState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    Idle(Option<(U, [u8; 9])>),
    ReadCo2(WriteAndReadResponse<'a, U, E, [u8; 9]>),
}

lazy_static! {
    static ref READ_CO2: Frame = Command::ReadCo2.into();
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
            state: MhZ19CState::Idle(Some((uart, [0u8; 9]))),
        }
    }

    pub fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>> {
        if let MhZ19CState::Idle(uart) = &mut self.state {
            let (uart, buf) = uart.take().unwrap();
            self.state =
                MhZ19CState::ReadCo2(WriteAndReadResponse::new(uart, &*READ_CO2.as_ref(), buf, 9));
        }
        if let MhZ19CState::ReadCo2(future) = &mut self.state {
            match future.poll() {
                Ok((uart, buf)) => {
                    let frame = Frame::new(buf);
                    let data = Self::unpack_return_frame(Command::ReadCo2, &frame)
                        .map_err(nb::Error::Other)?;
                    self.state = MhZ19CState::Idle(Some((uart, buf)));
                    Ok(u16::from_be_bytes(data[..2].try_into().unwrap()))
                }
                Err(err) => Err(err.map(Error::UartError)),
            }
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn unpack_return_frame<'b>(command: Command, frame: &'b Frame) -> Result<&'b [u8], E> {
        frame.validate().map_err(Error::FrameError)?;
        if !frame.is_response() {
            Err(Error::NotAResponse)
        } else if frame.op_code() != command.op_code() {
            return Err(Error::OpCodeMismatch {
                expected: command.op_code(),
                got: frame.op_code(),
            });
        } else {
            Ok(frame.data())
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error<T> {
    FrameError(frame::Error),
    NotAResponse,
    OpCodeMismatch { expected: u8, got: u8 },
    UartError(T),
}

impl<T: core::fmt::Display> core::fmt::Display for Error<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::FrameError(err) => write!(f, "Frame error: {}", err),
            Self::NotAResponse => write!(f, "Expected response, but got command."),
            Self::OpCodeMismatch { expected, got } => write!(
                f,
                "Expected response for op code 0x{:x}, but got op code 0x{:x}.",
                expected, got
            ),
            Self::UartError(err) => write!(f, "UART communication error: {}", err),
        }
    }
}

type Result<T, U> = core::result::Result<T, Error<U>>;

#[cfg(test)]
#[macro_use]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;

    use nb::block;
    use std::collections::VecDeque;
    use std::string::String;
    use std::vec::Vec;

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
            Err(Error::FrameError(frame::Error::InvalidStartByte(0x00)))
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
            Err(Error::FrameError(frame::Error::InvalidChecksum))
        );
    }
}
