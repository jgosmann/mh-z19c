#[macro_use]
extern crate lazy_static;

use core::convert::TryInto;
use embedded_hal::serial::{Read, Write};

struct WriteAll<'a, W, E>
where
    W: Write<u8, Error = E>,
{
    uart: Option<W>,
    buf: &'a [u8],
    bytes_written: usize,
}

impl<'a, W, E> WriteAll<'a, W, E>
where
    W: Write<u8, Error = E>,
{
    fn new(uart: W, buf: &'a [u8]) -> Self {
        Self {
            uart: Some(uart),
            buf,
            bytes_written: 0,
        }
    }
}

impl<'a, W, E> WriteAll<'a, W, E>
where
    W: Write<u8, Error = E>,
{
    fn poll(&mut self) -> nb::Result<W, E> {
        loop {
            let uart = self.uart.as_mut().unwrap();
            let bytes_written = self.buf[self.bytes_written];
            match uart.write(bytes_written) {
                Ok(()) => {
                    self.bytes_written += 1;
                    if self.bytes_written >= self.buf.len() {
                        return Ok(self.uart.take().unwrap());
                    }
                }
                Err(err) => return Err(err),
            }
        }
    }
}

struct ReadMultiple<R, E>
where
    R: Read<u8, Error = E>,
{
    uart: Option<R>,
    buf: Option<Vec<u8>>,
}

impl<R, E> ReadMultiple<R, E>
where
    R: Read<u8, Error = E>,
{
    fn new(uart: R, read_len: usize) -> Self {
        Self {
            uart: Some(uart),
            buf: Some(Vec::with_capacity(read_len)),
        }
    }
}

impl<'a, R, E> ReadMultiple<R, E>
where
    R: Read<u8, Error = E>,
{
    fn poll(&mut self) -> nb::Result<(R, Vec<u8>), E> {
        loop {
            let uart = self.uart.as_mut().unwrap();
            let buf = self.buf.as_mut().unwrap();
            match uart.read() {
                Ok(c) => {
                    buf.push(c);
                    if buf.len() >= buf.capacity() {
                        return Ok((self.uart.take().unwrap(), self.buf.take().unwrap()));
                    }
                }
                Err(err) => return Err(err),
            }
        }
    }
}

enum WriteAndReadResponseState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    Write(WriteAll<'a, U, E>),
    Read(ReadMultiple<U, E>),
}

struct WriteAndReadResponse<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    state: WriteAndReadResponseState<'a, U, E>,
}

impl<'a, U, E> WriteAndReadResponse<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    fn new(uart: U, buf: &'a [u8]) -> Self {
        Self {
            state: WriteAndReadResponseState::Write(WriteAll::new(uart, buf)),
        }
    }
}

impl<'a, U, E> WriteAndReadResponse<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    fn poll(&mut self) -> nb::Result<(U, Vec<u8>), E> {
        match &mut self.state {
            WriteAndReadResponseState::Write(future) => match future.poll() {
                Ok(uart) => {
                    self.state = WriteAndReadResponseState::Read(ReadMultiple::new(uart, 9));
                    Err(nb::Error::WouldBlock)
                }
                Err(err) => Err(err),
            },
            WriteAndReadResponseState::Read(future) => future.poll(),
        }
    }
}

enum MhZ19CState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    Idle(Option<U>),
    ReadCo2(WriteAndReadResponse<'a, U, E>),
}

lazy_static! {
    static ref READ_CO2: [u8; 9] = {
        let mut buf = [0x00; 9];
        frame_command(Command::ReadCo2, &mut buf);
        buf
    };
}

fn frame_command(command: Command, buf: &mut [u8; 9]) {
    buf[0] = 0xff;
    buf[1] = 0x01;
    command.serialize(&mut buf[2..8]);
    buf[8] = checksum(&buf[1..8]);
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
            self.state =
                MhZ19CState::ReadCo2(WriteAndReadResponse::new(uart.take().unwrap(), &*READ_CO2));
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
