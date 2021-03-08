//! Non-blocking communication state machines for the MH-Z19C UART interface.

use embedded_hal::serial::{Read, Write};

/// Trait for a future or state machine used for non-blocking communication.
///
/// These state machines depend on being actively polled (similar to Rust
/// futures). To cancel progress, just stop polling. However, this might leave
/// you within an undefined state of the UART communication protocol.
///
/// The future must not be polled after it has returned an
/// [`core::result::Result::Ok`] result.
pub trait NbFuture<R, E> {
    fn poll(&mut self) -> nb::Result<R, E>;
}

/// Write all bytes within a buffer.
pub struct WriteAll<'a, W, E>
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
    /// Create future to write all bytes in `buf` to `uart`.
    ///
    /// The ownership of the `uart` will be returned from [`Self::poll`] on
    /// completion.
    pub fn new(uart: W, buf: &'a [u8]) -> Self {
        Self {
            uart: Some(uart),
            buf,
            bytes_written: 0,
        }
    }
}

impl<'a, W, E> NbFuture<W, E> for WriteAll<'a, W, E>
where
    W: Write<u8, Error = E>,
{
    fn poll(&mut self) -> nb::Result<W, E> {
        let uart = self.uart.as_mut().unwrap();
        loop {
            let c = self.buf[self.bytes_written];
            match uart.write(c) {
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

/// Read multiple bytes.
pub struct ReadMultiple<R, E>
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
    /// Create future to read `read_len` bytes from `uart`.
    ///
    /// The ownership of the `uart` will be returned together with the bytes
    /// read from [`Self::poll`] on completion.
    pub fn new(uart: R, read_len: usize) -> Self {
        Self {
            uart: Some(uart),
            buf: Some(Vec::with_capacity(read_len)),
        }
    }
}

impl<'a, R, E> NbFuture<(R, Vec<u8>), E> for ReadMultiple<R, E>
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

/// Write all bytes within a buffer and read a fixed length response afterwards.
pub struct WriteAndReadResponse<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    state: WriteAndReadResponseState<'a, U, E>,
    response_len: usize,
}

enum WriteAndReadResponseState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    Write(WriteAll<'a, U, E>),
    Read(ReadMultiple<U, E>),
}

impl<'a, U, E> WriteAndReadResponse<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    /// Create future to write `buf` bytes to `uart` and read `read_len` bytes
    /// afterwards.
    ///
    /// The ownership of the `uart` will be returned together with the bytes
    /// read from [`Self::poll`] on completion.
    pub fn new(uart: U, buf: &'a [u8], response_len: usize) -> Self {
        Self {
            state: WriteAndReadResponseState::Write(WriteAll::new(uart, buf)),
            response_len,
        }
    }
}

impl<'a, U, E> NbFuture<(U, Vec<u8>), E> for WriteAndReadResponse<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    fn poll(&mut self) -> nb::Result<(U, Vec<u8>), E> {
        match &mut self.state {
            WriteAndReadResponseState::Write(future) => match future.poll() {
                Ok(uart) => {
                    self.state =
                        WriteAndReadResponseState::Read(ReadMultiple::new(uart, self.response_len));
                    Err(nb::Error::WouldBlock)
                }
                Err(err) => Err(err),
            },
            WriteAndReadResponseState::Read(future) => future.poll(),
        }
    }
}
