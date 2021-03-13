use embedded_hal::serial::{Read, Write};
use std::collections::VecDeque;
use std::string::String;
use std::vec::Vec;

#[derive(Debug)]
pub struct SerialMock {
    read_return_values: VecDeque<nb::Result<u8, String>>,
    write_return_values: VecDeque<nb::Result<(), String>>,
    pub write_buf: Vec<u8>,
    pub flushed_up_to: usize,
}

impl SerialMock {
    pub fn new(
        read_return_values: Vec<nb::Result<u8, String>>,
        write_return_values: Vec<nb::Result<(), String>>,
    ) -> Self {
        Self {
            read_return_values: VecDeque::from(read_return_values),
            write_return_values: VecDeque::from(write_return_values),
            write_buf: vec![],
            flushed_up_to: 0,
        }
    }
}

impl Read<u8> for SerialMock {
    type Error = String;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_return_values
            .pop_front()
            .unwrap_or(Err(nb::Error::WouldBlock))
    }
}

impl Write<u8> for SerialMock {
    type Error = String;

    fn write(&mut self, c: u8) -> nb::Result<(), Self::Error> {
        if let Some(return_value) = self.write_return_values.pop_front() {
            if return_value.is_ok() {
                self.write_buf.push(c);
            }
            return_value
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flushed_up_to = self.write_buf.len();
        Ok(())
    }
}
