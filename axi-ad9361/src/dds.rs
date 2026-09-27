//! DDS tone generators of the DAC.
//!
//! Every DAC channel (one I or Q lane of a TX channel) has two DDS generators, and the core adds
//! their outputs together. A generator is a 16 bit phase accumulator that steps once per sample,
//! so a tone frequency is a multiple of `sample_rate / 65536` (469 Hz at 30.72 MSPS). The sample
//! rate here is the TX sample rate of the transceiver, not the interface clock the core runs on.
//!
//! The DAC plays the tones while its data source is
//! [`DataSource::InternalTone`](crate::regs::dac::regs::DataSource::InternalTone).
//!
//! ```no_run
//! use axi_ad9361::dds::{DdsScale, IqPair};
//! use fugit::HertzU32;
//! # fn example(dac: &mut axi_ad9361::dac::Dac) {
//! // 1 MHz above the LO on the first TX channel, at half of full scale
//! let actual = dac
//!     .set_tone(IqPair::First, HertzU32::MHz(1), DdsScale::from_fraction(0.5), HertzU32::Hz(30_720_000))
//!     .unwrap();
//! assert_eq!(actual, HertzU32::MHz(1));
//! # }
//! ```

use arbitrary_int::u4;
use fugit::HertzU32;

use crate::dac::Dac;
use crate::regs::dac::regs::Control1;

/// Bits of the DDS phase accumulator.
const PHASE_BITS: u32 = 16;
/// Register value for a scale of 1.0 (the field is 1.1.14 fixed point).
const SCALE_ONE: u16 = 0x4000;

/// Value out of range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfRange;

/// Amplitude of a DDS generator, as a fraction of the DAC's full scale.
///
/// Both generators of a lane are added together, so their scales should add up to 1.0 at most or
/// the output clips. [`Dac::set_tone`] only uses one of them.
///
/// ```
/// use axi_ad9361::dds::DdsScale;
///
/// const HALF: DdsScale = DdsScale::from_fraction(0.5);
/// assert!((DdsScale::from_dbfs(-6).unwrap().fraction() - 0.501).abs() < 0.001);
/// assert!(DdsScale::try_from_fraction(1.5).is_err());
/// # let _ = HALF;
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct DdsScale(u16);

impl DdsScale {
    /// Silent.
    pub const ZERO: Self = Self(0);
    /// Full scale of the DAC.
    pub const FULL: Self = Self(SCALE_ONE);

    /// Fraction of full scale, 0.0 to 1.0, rounded to steps of 1/16384.
    pub const fn try_from_fraction(fraction: f32) -> Result<Self, OutOfRange> {
        // written so NaN fails too
        if !(fraction >= 0.0 && fraction <= 1.0) {
            return Err(OutOfRange);
        }
        Ok(Self((fraction * SCALE_ONE as f32 + 0.5) as u16))
    }

    /// Like [`Self::try_from_fraction`]. Out of range in a `const` is a compile error.
    pub const fn from_fraction(fraction: f32) -> Self {
        match Self::try_from_fraction(fraction) {
            Ok(scale) => scale,
            Err(_) => panic!("DDS scale out of range, has to be 0.0 to 1.0"),
        }
    }

    /// Relative to full scale in dB, 0 or below. -6 dBFS is about half the amplitude.
    pub const fn from_dbfs(dbfs: i8) -> Result<Self, OutOfRange> {
        if dbfs > 0 {
            return Err(OutOfRange);
        }
        // 10^(-1/20), one dB down in amplitude. core has no powf
        const ONE_DB_DOWN: f32 = 0.891_250_94;
        let mut fraction = 1.0f32;
        let mut db = 0;
        while db > dbfs {
            fraction *= ONE_DB_DOWN;
            db -= 1;
        }
        Self::try_from_fraction(fraction)
    }

    /// Fraction of full scale.
    pub const fn fraction(self) -> f32 {
        self.0 as f32 / SCALE_ONE as f32
    }

    /// Register value, 1.1.14 fixed point with the sign bit clear.
    pub const fn raw(self) -> u16 {
        self.0
    }
}

/// Starting phase of a DDS generator. Resolution is 360/65536 degrees.
///
/// Only the phase between generators matters, like the 90 degrees between I and Q that
/// [`Dac::set_tone`] uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DdsPhase(u16);

impl DdsPhase {
    /// Zero degrees.
    pub const ZERO: Self = Self(0);

    /// Whole degrees, wraps at 360.
    pub const fn from_degrees(degrees: u16) -> Self {
        Self::from_millidegrees(degrees as u32 * 1000)
    }

    /// Thousandths of a degree, wraps at 360 degrees.
    pub const fn from_millidegrees(millidegrees: u32) -> Self {
        let millidegrees = (millidegrees % 360_000) as u64;
        let word = (millidegrees * (1 << PHASE_BITS) + 360_000 / 2) / 360_000;
        Self(word as u16)
    }

    /// Register value, one full turn is 65536.
    pub const fn raw(self) -> u16 {
        self.0
    }
}

/// Everything about one DDS generator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DdsTone {
    /// Below half the sample rate, rounded to the frequency grid.
    pub frequency: HertzU32,
    /// Starting phase.
    pub phase: DdsPhase,
    /// Amplitude.
    pub scale: DdsScale,
}

impl DdsTone {
    /// Generator off.
    pub const OFF: Self = Self {
        frequency: HertzU32::Hz(0),
        phase: DdsPhase::ZERO,
        scale: DdsScale::ZERO,
    };
}

/// A pair of DAC channels carrying the I and Q of one TX channel: channels 0/1 or 2/3.
///
/// With the AD9361 in 2R2T, `First` is TX1 and `Second` is TX2. In 1R1T only `First` is used,
/// for whichever TX channel is enabled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IqPair {
    /// DAC channels 0 and 1
    First,
    /// DAC channels 2 and 3
    Second,
}

/// I or Q lane of an [`IqPair`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IqLane {
    /// In phase, the even DAC channel
    I,
    /// Quadrature, the odd DAC channel
    Q,
}

/// One of the two DDS generators of a DAC channel. The core adds both outputs together.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DdsGenerator {
    /// `CHAN_CNTRL_1`/`_2`
    First,
    /// `CHAN_CNTRL_3`/`_4`
    Second,
}

/// The tone is half the sample rate or more.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AboveNyquist;

impl core::fmt::Display for AboveNyquist {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("the tone is above half the sample rate")
    }
}

impl core::error::Error for AboveNyquist {}

/// Phase increment for `frequency` at `sample_rate`, and the frequency it really gives. Unlike
/// the ADI drivers, the lowest bit isn't forced on, so a frequency on the grid comes out exact.
fn phase_increment(
    frequency: HertzU32,
    sample_rate: HertzU32,
) -> Result<(u16, HertzU32), AboveNyquist> {
    let (frequency, sample_rate) = (frequency.to_raw() as u64, sample_rate.to_raw() as u64);
    if frequency * 2 >= sample_rate {
        return Err(AboveNyquist);
    }
    let increment = (((frequency << PHASE_BITS) + sample_rate / 2) / sample_rate) as u16;
    let actual = (increment as u64 * sample_rate + (1 << (PHASE_BITS - 1))) >> PHASE_BITS;
    Ok((increment, HertzU32::from_raw(actual as u32)))
}

impl Dac {
    /// Sets one DDS generator, and returns the frequency it really runs at. `sample_rate` is
    /// the transceiver's TX sample rate. Takes effect immediately.
    ///
    /// A lane on its own is a real signal. For a complex tone at LO + f, put f on both lanes of
    /// a pair with I 90 degrees ahead of Q ([`Self::set_tone`] does that); with Q 90 degrees
    /// ahead it comes out at LO - f instead.
    pub fn set_dds(
        &mut self,
        pair: IqPair,
        lane: IqLane,
        generator: DdsGenerator,
        tone: DdsTone,
        sample_rate: HertzU32,
    ) -> Result<HertzU32, AboveNyquist> {
        let (increment, actual) = phase_increment(tone.frequency, sample_rate)?;
        let channel = match pair {
            IqPair::First => 0,
            IqPair::Second => 2,
        } + match lane {
            IqLane::I => 0,
            IqLane::Q => 1,
        };

        self.regs().write_control1(Control1::ZERO);
        let mut channel = self.channel_mut(u4::new(channel));
        let scale = tone.scale.raw() as u32;
        let phase_and_increment = (tone.phase.raw() as u32) << 16 | increment as u32;
        match generator {
            DdsGenerator::First => {
                channel.write_control1(scale);
                channel.write_control2(phase_and_increment);
            }
            DdsGenerator::Second => {
                channel.write_control3(scale);
                channel.write_control4(phase_and_increment);
            }
        }
        self.synchronize();
        Ok(actual)
    }

    /// Puts a continuous tone `frequency` above the LO on `pair`, with nothing else on it:
    /// I 90 degrees ahead of Q on the first generators, the second generators off. Returns the
    /// frequency the tone really has.
    ///
    /// `sample_rate` is the transceiver's TX sample rate, the tone moves with it. Takes effect
    /// immediately and keeps playing on its own while the DAC's source is the DDS.
    pub fn set_tone(
        &mut self,
        pair: IqPair,
        frequency: HertzU32,
        scale: DdsScale,
        sample_rate: HertzU32,
    ) -> Result<HertzU32, AboveNyquist> {
        // before writing anything
        let (_, actual) = phase_increment(frequency, sample_rate)?;
        for (lane, degrees) in [(IqLane::I, 90), (IqLane::Q, 0)] {
            let phase = DdsPhase::from_degrees(degrees);
            let tone = DdsTone { frequency, phase, scale };
            self.set_dds(pair, lane, DdsGenerator::First, tone, sample_rate)?;
            self.set_dds(pair, lane, DdsGenerator::Second, DdsTone::OFF, sample_rate)?;
        }
        Ok(actual)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn increment_uses_the_sample_rate() {
        let (increment, actual) =
            phase_increment(HertzU32::Hz(2_400_000), HertzU32::Hz(30_720_000)).unwrap();
        assert_eq!(increment, 5120);
        assert_eq!(actual, HertzU32::Hz(2_400_000));
    }

    #[test]
    fn nyquist_is_rejected() {
        let fs = HertzU32::Hz(30_720_000);
        assert_eq!(phase_increment(HertzU32::Hz(15_360_000), fs), Err(AboveNyquist));
        assert!(phase_increment(HertzU32::Hz(15_359_000), fs).is_ok());
    }

    #[test]
    fn scale_conversions() {
        assert_eq!(DdsScale::FULL.raw(), 0x4000);
        assert_eq!(DdsScale::from_fraction(0.5).raw(), 0x2000);
        assert_eq!(DdsScale::from_dbfs(0).unwrap(), DdsScale::FULL);
        // -20 dB is a tenth
        assert_eq!(DdsScale::from_dbfs(-20).unwrap().raw(), 1638);
        assert!(DdsScale::from_dbfs(1).is_err());
        assert!(DdsScale::try_from_fraction(f32::NAN).is_err());
        assert!(DdsScale::try_from_fraction(-0.1).is_err());
    }

    #[test]
    fn phase_conversions() {
        assert_eq!(DdsPhase::from_degrees(90).raw(), 0x4000);
        assert_eq!(DdsPhase::from_degrees(450), DdsPhase::from_degrees(90));
        assert_eq!(DdsPhase::from_millidegrees(180_000).raw(), 0x8000);
    }

    #[test]
    fn set_tone_writes_both_lanes() {
        let memory = std::boxed::Box::leak(std::vec![0u32; 0x4000].into_boxed_slice());
        let mut dac = Dac::new_no_init(memory.as_mut_ptr() as usize);
        let actual = dac
            .set_tone(
                IqPair::Second,
                HertzU32::Hz(2_400_000),
                DdsScale::from_fraction(0.5),
                HertzU32::Hz(30_720_000),
            )
            .unwrap();
        assert_eq!(actual, HertzU32::Hz(2_400_000));

        // TX2 is DAC channels 2 (I) and 3 (Q). The low 16 bits of control2/4 are the phase
        // increment, the high ones the phase
        let i = dac.channel_mut(u4::new(2));
        assert_eq!(i.read_control2(), 0x4000 << 16 | 5120);
        assert_eq!(i.read_control1(), 0x2000);
        assert_eq!(i.read_control3(), 0, "second generator off");
        let q = dac.channel_mut(u4::new(3));
        assert_eq!(q.read_control2(), 5120);
        assert_eq!(q.read_control1(), 0x2000);
        assert_eq!(dac.channel_mut(u4::new(0)).read_control1(), 0, "TX1 untouched");
    }
}
