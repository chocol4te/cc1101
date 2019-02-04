//! Low level unrestricted access to the CC1101 radio chip.
use hal::blocking::spi::{Transfer, Write};
use hal::digital::OutputPin;

#[macro_use]
mod macros;
mod access;
mod traits;

pub mod registers;
pub mod types;

use self::registers::*;

pub struct Cc1101<SPI, CS> {
    pub(crate) spi: SPI,
    pub(crate) cs: CS,
    //    gdo0: GDO0,
    //    gdo2: GDO2,
    pub buf: [u8; 64], // 64 byte fixed buffer to allow for transmission of arbitrary slices/arrays
}

impl<SPI, CS, E> Cc1101<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Result<Self, E> {
        let cc1101 = Cc1101 {
            spi: spi,
            cs: cs,
            buf: [0u8; 64],
        };
        Ok(cc1101)
    }

    pub fn read_register<R>(&mut self, reg: R) -> Result<u8, E>
    where
        R: Into<Register>,
    {
        self.cs.set_low();
        let mut buffer = [reg.into().raddr(), 0u8];
        self.spi.transfer(&mut buffer)?;
        self.cs.set_high();
        Ok(buffer[1])
    }

    pub fn read_fifo(&mut self, addr: &mut u8, len: &mut u8, buf: &mut [u8]) -> Result<(), E> {
        let mut buffer = [Command::FIFO.addr() | 0xC0, 0, 0];

        self.cs.set_low();
        self.spi.transfer(&mut buffer)?;
        self.spi.transfer(buf)?;
        self.cs.set_high();

        *len = buffer[1];
        *addr = buffer[2];

        Ok(())
    }

    pub fn write_strobe(&mut self, com: Command) -> Result<(), E> {
        self.cs.set_low();
        self.spi.write(&[com.addr()])?;
        self.cs.set_high();
        Ok(())
    }

    pub fn write_register<R>(&mut self, reg: R, byte: u8) -> Result<(), E>
    where
        R: Into<Register>,
    {
        self.cs.set_low();
        self.spi.write(&mut [reg.into().waddr(), byte])?;
        self.cs.set_high();
        Ok(())
    }

    pub fn modify_register<R, F>(&mut self, reg: R, f: F) -> Result<(), E>
    where
        R: Into<Register> + Copy,
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_register(reg)?;
        self.write_register(reg, f(r))?;
        Ok(())
    }
}
