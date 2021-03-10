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
///
/// * `R`: OK result type
/// * `E`: error type
pub trait NbFuture<R, E> {
    fn poll(&mut self) -> nb::Result<R, E>;
}

/// Write all bytes within a buffer.
///
/// * `'a`: buffer lifetime
/// * `W`: Concrete [`embedded_hal::serial::Write`] type with error type `E`
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
///
/// * `R`: Concrete [`embedded_hal::serial::Read`] type with error type `E`
/// * `B`: Type of buffer to write to
pub struct ReadMultiple<R, E, B>
where
    R: Read<u8, Error = E>,
    B: AsMut<[u8]>,
{
    uart: Option<R>,
    buf: Option<B>,
    bytes_read: usize,
    read_len: usize,
}

impl<R, E, B> ReadMultiple<R, E, B>
where
    R: Read<u8, Error = E>,
    B: AsMut<[u8]>,
{
    /// Create future to read `read_len` bytes from `uart`.
    ///
    /// The ownership of the `uart` will be returned together with the bytes
    /// read from [`Self::poll`] on completion.
    pub fn new(uart: R, buf: B, read_len: usize) -> Self {
        Self {
            uart: Some(uart),
            buf: Some(buf),
            bytes_read: 0,
            read_len,
        }
    }
}

impl<'a, R, E, B> NbFuture<(R, B), E> for ReadMultiple<R, E, B>
where
    R: Read<u8, Error = E>,
    B: AsMut<[u8]>,
{
    fn poll(&mut self) -> nb::Result<(R, B), E> {
        loop {
            let uart = self.uart.as_mut().unwrap();
            let buf = self.buf.as_mut().unwrap().as_mut();
            match uart.read() {
                Ok(c) => {
                    buf[self.bytes_read] = c;
                    self.bytes_read += 1;
                    if self.bytes_read >= self.read_len {
                        return Ok((self.uart.take().unwrap(), self.buf.take().unwrap()));
                    }
                }
                Err(err) => return Err(err),
            }
        }
    }
}

/// Write all bytes within a buffer and read a fixed length response afterwards.
///
/// * `'a`: buffer lifetime
/// * `U`: Concrete [`embedded_hal::serial::Read`] and
///   [`embedded_hal::serial::Write`] type with error type `E`
/// * `B`: Type of buffer to write to
pub struct WriteAndReadResponse<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    state: Option<WriteAndReadResponseState<'a, U, E, B>>,
}

enum WriteAndReadResponseState<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    Write {
        future: WriteAll<'a, U, E>,
        read_buf: B,
        response_len: usize,
    },
    Read {
        future: ReadMultiple<U, E, B>,
    },
    Completed {
        uart: U,
        read_buf: B,
    },
}

impl<'a, U, E, B> WriteAndReadResponseState<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    fn advance(self) -> nb::Result<Self, E> {
        use WriteAndReadResponseState::*;
        match self {
            Write {
                mut future,
                read_buf,
                response_len,
            } => {
                let uart = future.poll()?;
                Ok(Read {
                    future: ReadMultiple::new(uart, read_buf, response_len),
                })
            }
            Read { mut future } => {
                let (uart, read_buf) = future.poll()?;
                Ok(Completed { uart, read_buf })
            }
            other => Ok(other),
        }
    }
}

impl<'a, U, E, B> WriteAndReadResponse<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    /// Create future to write `buf` bytes to `uart` and read `read_len` bytes
    /// afterwards.
    ///
    /// The ownership of the `uart` will be returned together with the bytes
    /// read from [`Self::poll`] on completion.
    pub fn new(uart: U, write_buf: &'a [u8], read_buf: B, response_len: usize) -> Self {
        Self {
            state: Some(WriteAndReadResponseState::Write {
                future: WriteAll::new(uart, write_buf),
                read_buf,
                response_len,
            }),
        }
    }
}

impl<'a, U, E, B> NbFuture<(U, B), E> for WriteAndReadResponse<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    fn poll(&mut self) -> nb::Result<(U, B), E> {
        loop {
            use WriteAndReadResponseState::*;
            let new_state = self.state.take().unwrap().advance()?;
            if let Completed { uart, read_buf } = new_state {
                return Ok((uart, read_buf));
            }
            self.state = Some(new_state);
        }
    }
}
