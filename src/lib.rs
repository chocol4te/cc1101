#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::spi::{Transfer, Write};
use hal::digital::OutputPin;

const FXOSC: u64 = 26_000_000;

#[macro_use]
pub mod lowlevel;
mod rssi;

use lowlevel::registers::*;
use lowlevel::types::*;
use rssi::rssi_to_dbm;

/// CC1101 errors.
#[derive(Debug)]
pub enum Error<E> {
    /// The RX FIFO buffer overflowed, too small buffer for configured packet length.
    RxOverflow,
    /// Corrupt packet received with invalid CRC.
    CrcMismatch,
    /// Platform-dependent SPI-errors, such as IO errors.
    Spi(E),
    /// The supplied payload was larger than the maximum allowed packet length
    PayloadOversize,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Spi(e)
    }
}

/// High level API for interacting with the CC1101 radio chip.
pub struct Cc1101<SPI, CS>(lowlevel::Cc1101<SPI, CS>);

impl<SPI, CS, E> Cc1101<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Result<Self, Error<E>> {
        Ok(Cc1101(lowlevel::Cc1101::new(spi, cs)?))
    }

    pub fn set_frequency(&mut self, hz: u64) -> Result<(), Error<E>> {
        let freq = hz * 1u64.rotate_left(16) / FXOSC;
        self.0.write_register(Config::FREQ2, ((freq >> 16) & 0xff) as u8)?;
        self.0.write_register(Config::FREQ1, ((freq >> 8) & 0xff) as u8)?;
        self.0.write_register(Config::FREQ0, (freq & 0xff) as u8)?;
        Ok(())
    }

    pub fn get_hw_info(&mut self) -> Result<(u8, u8), Error<E>> {
        let partnum = self.0.read_register(Status::PARTNUM)?;
        let version = self.0.read_register(Status::VERSION)?;
        Ok((partnum, version))
    }

    /// Received Signal Strength Indicator is an estimate of the signal power level in the chosen channel.
    pub fn get_rssi_dbm(&mut self) -> Result<i16, Error<E>> {
        Ok(rssi_to_dbm(self.0.read_register(Status::RSSI)?))
    }

    /// The Link Quality Indicator metric of the current quality of the received signal.
    pub fn get_lqi(&mut self) -> Result<u8, Error<E>> {
        let lqi = self.0.read_register(Status::LQI)?;
        Ok(lqi & !(1u8 << 7))
    }

    /// Configure the sync word to use, and at what level it should be verified.
    pub fn set_sync_mode(&mut self, sync_mode: SyncMode) -> Result<(), Error<E>> {
        let reset: u16 = (SYNC1::default().bits() as u16) << 8 | (SYNC0::default().bits() as u16);

        let (mode, word) = match sync_mode {
            SyncMode::Disabled => (SyncCheck::DISABLED, reset),
            SyncMode::MatchPartial(word) => (SyncCheck::CHECK_15_16, word),
            SyncMode::MatchPartialRepeated(word) => (SyncCheck::CHECK_30_32, word),
            SyncMode::MatchFull(word) => (SyncCheck::CHECK_16_16, word),
        };
        self.0.modify_register(Config::MDMCFG2, |r| {
            MDMCFG2(r).modify().sync_mode(mode.value()).bits()
        })?;
        self.0.write_register(Config::SYNC1, ((word >> 8) & 0xff) as u8)?;
        self.0.write_register(Config::SYNC0, (word & 0xff) as u8)?;
        Ok(())
    }

    /// Configure signal modulation.
    pub fn set_modulation(&mut self, format: Modulation) -> Result<(), Error<E>> {
        use lowlevel::types::ModFormat as MF;

        let value = match format {
            Modulation::BinaryFrequencyShiftKeying => MF::MOD_2FSK,
            Modulation::GaussianFrequencyShiftKeying => MF::MOD_GFSK,
            Modulation::OnOffKeying => MF::MOD_ASK_OOK,
            Modulation::FourFrequencyShiftKeying => MF::MOD_4FSK,
            Modulation::MinimumShiftKeying => MF::MOD_MSK,
        };
        self.0.modify_register(Config::MDMCFG2, |r| {
            MDMCFG2(r).modify().mod_format(value.value()).bits()
        })?;
        Ok(())
    }

    /// Configure device address, and address filtering.
    pub fn set_address_filter(&mut self, filter: AddressFilter) -> Result<(), Error<E>> {
        use lowlevel::types::AddressCheck as AC;

        let (mode, addr) = match filter {
            AddressFilter::Disabled => (AC::DISABLED, ADDR::default().bits()),
            AddressFilter::Device(addr) => (AC::SELF, addr),
            AddressFilter::DeviceLowBroadcast(addr) => (AC::SELF_LOW_BROADCAST, addr),
            AddressFilter::DeviceHighLowBroadcast(addr) => (AC::SELF_HIGH_LOW_BROADCAST, addr),
        };
        self.0.modify_register(Config::PKTCTRL1, |r| {
            PKTCTRL1(r).modify().adr_chk(mode.value()).bits()
        })?;
        self.0.write_register(Config::ADDR, addr)?;
        Ok(())
    }

    /// Configure packet mode, and length.
    pub fn set_packet_length(&mut self, length: PacketLength) -> Result<(), Error<E>> {
        use lowlevel::types::LengthConfig as LC;

        let (format, pktlen) = match length {
            PacketLength::Fixed(limit) => (LC::FIXED, limit),
            PacketLength::Variable(max_limit) => (LC::VARIABLE, max_limit),
            PacketLength::Infinite => (LC::INFINITE, PKTLEN::default().bits()),
        };
        self.0.modify_register(Config::PKTCTRL0, |r| {
            PKTCTRL0(r).modify().length_config(format.value()).bits()
        })?;
        self.0.write_register(Config::PKTLEN, pktlen)?;
        Ok(())
    }

    /// Set radio in Receive/Transmit/Idle mode.
    pub fn set_radio_mode(&mut self, radio_mode: RadioMode) -> Result<(), Error<E>> {
        let target = match radio_mode {
            RadioMode::Receive => {
                self.set_radio_mode(RadioMode::Idle)?;
                self.0.write_strobe(Command::SRX)?;
                MachineState::RX
            }
            RadioMode::Transmit => {
                self.set_radio_mode(RadioMode::Idle)?;
                self.0.write_strobe(Command::STX)?;
                MachineState::TX
            }
            RadioMode::Idle => {
                self.0.write_strobe(Command::SIDLE)?;
                MachineState::IDLE
            }
        };
        self.await_machine_state(target)
    }

    /// Configure some default settings, to be removed in the future.
    #[cfg_attr(rustfmt, rustfmt_skip)]
    pub fn set_defaults(&mut self) -> Result<(), Error<E>> {
        self.0.write_strobe(Command::SRES)?;

        self.0.write_register(Config::PKTCTRL0, PKTCTRL0::default()
            .white_data(0).bits()
        )?;

        self.0.write_register(Config::FSCTRL1, FSCTRL1::default()
            .freq_if(0x08).bits() // f_if = (f_osc / 2^10) * FREQ_IF
        )?;

        self.0.write_register(Config::MDMCFG4, MDMCFG4::default()
            .chanbw_e(0x03) // bw_chan = f_osc / (8 * (4 + chanbw_m) * 2^chanbw_e
            .chanbw_m(0x00)
            .drate_e(0x0A).bits()
        )?;

        self.0.write_register(Config::MDMCFG3, MDMCFG3::default()
            .drate_m(0x83).bits() // r_data = (((256 + drate_m) * 2^drate_e) / 2**38) * f_osc
        )?;

        self.0.write_register(Config::MDMCFG2, MDMCFG2::default()
            .dem_dcfilt_off(1).bits()
        )?;

        self.0.write_register(Config::DEVIATN, DEVIATN::default()
            .deviation_e(0x03)
            .deviation_m(0x05).bits()
        )?;

        self.0.write_register(Config::MCSM0, MCSM0::default()
            .fs_autocal(AutoCalibration::FROM_IDLE.value()).bits()
        )?;

        self.0.write_register(Config::AGCCTRL2, AGCCTRL2::default()
            .max_lna_gain(0x04).bits()
        )?;

        Ok(())
    }

    fn await_machine_state(&mut self, target: MachineState) -> Result<(), Error<E>> {
        loop {
            let marcstate = MARCSTATE(self.0.read_register(Status::MARCSTATE)?);
            if target.value() == marcstate.marc_state() {
                break;
            }
        }
        Ok(())
    }

    fn rx_bytes_available(&mut self) -> Result<u8, Error<E>> {
        let mut last = 0;

        loop {
            let rxbytes = RXBYTES(self.0.read_register(Status::RXBYTES)?);
            if rxbytes.rxfifo_overflow() == 1 {
                return Err(Error::RxOverflow);
            }

            let nbytes = rxbytes.num_rxbytes();
            if nbytes > 0 && nbytes == last {
                break;
            }

            last = nbytes;
        }
        Ok(last)
    }

    // Should also be able to configure MCSM1.RXOFF_MODE to declare what state
    // to enter after fully receiving a packet.
    // Possible targets: IDLE, FSTON, TX, RX
    pub fn receive(&mut self, addr: &mut u8, buf: &mut [u8]) -> Result<u8, Error<E>> {
        match self.rx_bytes_available() {
            Ok(_nbytes) => {
                let mut length = 0u8;
                self.0.read_fifo(addr, &mut length, buf)?;
                let lqi = self.0.read_register(Status::LQI)?;
                self.await_machine_state(MachineState::IDLE)?;
                self.0.write_strobe(Command::SFRX)?;
                if (lqi >> 7) != 1 {
                    Err(Error::CrcMismatch)
                } else {
                    Ok(length)
                }
            }
            Err(err) => {
                self.0.write_strobe(Command::SFRX)?;
                Err(err)
            }
        }
    }

    /// ?
    pub fn set_500k_preset(&mut self) -> Result<(), Error<E>> {
        self.0.cs.set_low();

        self.0.spi.write(&[0x40 | 0x40,
            0x07, // IOCFG2        GDO2 Output Pin Configuration
            0x2E, // IOCFG1        GDO1 Output Pin Configuration
            0x80, // IOCFG0        GDO0 Output Pin Configuration
            0x07, // FIFOTHR       RX FIFO and TX FIFO Thresholds
            0x57, // SYNC1         Sync Word, High Byte
            0x43, // SYNC0         Sync Word, Low Byte
            0x3E, // PKTLEN        Packet Length
            0x0E, // PKTCTRL1      Packet Automation Control
            0x45, // PKTCTRL0      Packet Automation Control
            0xFF, // ADDR          Device Address
            0x00, // CHANNR        Channel Number
            0x0C, // FSCTRL1       Frequency Synthesizer Control
            0x00, // FSCTRL0       Frequency Synthesizer Control
            0x21, // FREQ2         Frequency Control Word, High Byte
            0x65, // FREQ1         Frequency Control Word, Middle Byte
            0x6A, // FREQ0         Frequency Control Word, Low Byte
            0x0E, // MDMCFG4       Modem Configuration
            0x3B, // MDMCFG3       Modem Configuration
            0x73, // MDMCFG2       Modem Configuration
            0xA0, // MDMCFG1       Modem Configuration
            0xF8, // MDMCFG0       Modem Configuration
            0x00, // DEVIATN       Modem Deviation Setting
            0x07, // MCSM2         Main Radio Control State Machine Configuration
            0x0C, // MCSM1         Main Radio Control State Machine Configuration
            0x18, // MCSM0         Main Radio Control State Machine Configuration
            0x1D, // FOCCFG        Frequency Offset Compensation Configuration
            0x1C, // BSCFG         Bit Synchronization Configuration
            0xC7, // AGCCTRL2      AGC Control
            0x40, // AGCCTRL1      AGC Control
            0xB2, // AGCCTRL0      AGC Control
            0x02, // WOREVT1       High Byte Event0 Timeout
            0x26, // WOREVT0       Low Byte Event0 Timeout
            0x09, // WORCTRL       Wake On Radio Control
            0xB6, // FREND1        Front End RX Configuration
            0x17, // FREND0        Front End TX Configuration
            0xEA, // FSCAL3        Frequency Synthesizer Calibration
            0x0A, // FSCAL2        Frequency Synthesizer Calibration
            0x00, // FSCAL1        Frequency Synthesizer Calibration
            0x19, // FSCAL0        Frequency Synthesizer Calibration
            0x41, // RCCTRL1       RC Oscillator Configuration
            0x00, // RCCTRL0       RC Oscillator Configuration
            0x59, // FSTEST        Frequency Synthesizer Calibration Control,
            0x7F, // PTEST         Production Test
            0x3F, // AGCTEST       AGC Test
            0x81, // TEST2         Various Test Settings
            0x3F, // TEST1         Various Test Settings
            0x0B  // TEST0         Various Test Settings
        ])?;

        self.0.spi.write(&[0x7E | 0x40,
        0x17, 0x1D, 0x26, 0x69, 0x51, 0x86, 0xCC, 0xC3])?;

        self.0.write_register(Config::FREQ2, 0x0C)?;
        self.0.write_register(Config::FREQ1, 0x1D)?;
        self.0.write_register(Config::FREQ0, 0x89)?;

        // Power level
        self.0.spi.write(&[0x22 | 0x00, 0x07])?;

        self.0.cs.set_high();

        Ok(())
    }

    /// ?
    pub fn transmit(&mut self, buf: &mut [u8]) -> Result<(), Error<E>> {
        let len = buf.len();
        if len > 61 {
            return Err(Error::PayloadOversize);
        }

        self.0.buf[0] = 0x7F | 0x40;
        self.0.buf[1] = len as u8 + 1;
        self.0.buf[2] = 0x02;
        for i in 0..len {
            self.0.buf[3+i] = buf[i];
        }

        self.0.cs.set_low();

        self.0.spi.write(&self.0.buf[..len+3])?;

        self.0.spi.write(&[0x35])?;
        self.await_machine_state(MachineState::IDLE)?;

        self.0.cs.set_high();

        Ok(())
    }
}

/// Modulation format configuration.
pub enum Modulation {
    /// 2-FSK.
    BinaryFrequencyShiftKeying,
    /// GFSK.
    GaussianFrequencyShiftKeying,
    /// ASK / OOK.
    OnOffKeying,
    /// 4-FSK.
    FourFrequencyShiftKeying,
    /// MSK.
    MinimumShiftKeying,
}

/// Packet length configuration.
pub enum PacketLength {
    /// Set packet length to a fixed value.
    Fixed(u8),
    /// Set upper bound of variable packet length.
    Variable(u8),
    /// Infinite packet length, streaming mode.
    Infinite,
}

/// Address check configuration.
pub enum AddressFilter {
    /// No address check.
    Disabled,
    /// Address check, no broadcast.
    Device(u8),
    /// Address check and 0 (0x00) broadcast.
    DeviceLowBroadcast(u8),
    /// Address check and 0 (0x00) and 255 (0xFF) broadcast.
    DeviceHighLowBroadcast(u8),
}

/// Radio operational mode.
pub enum RadioMode {
    Receive,
    Transmit,
    Idle,
}

/// Sync word configuration.
pub enum SyncMode {
    /// No sync word.
    Disabled,
    /// Match 15 of 16 bits of given sync word.
    MatchPartial(u16),
    /// Match 30 of 32 bits of a repetition of given sync word.
    MatchPartialRepeated(u16),
    /// Match 16 of 16 bits of given sync word.
    MatchFull(u16),
}
