//! # ADI AXI DMAC driver
//!
//! This is a native Rust driver for the
//! [ADI AXI DMAC IP core](https://analogdevicesinc.github.io/hdl/library/axi_dmac/index.html),
//! used for example for the DMAs of the AXI AD9361 IP core. It is not the AMD AXI DMA IP core,
//! there is the `axi-dma` crate for that one.
//!
//! The capabilities of a core are probed once. The result is an [RxDmac] (device to memory) or
//! a [TxDmac] (memory to device, repeating in hardware), so the transfer functions only exist
//! for the direction the core has. A transfer borrows its buffer until it is finished or
//! dropped, and dropping it stops the core.
//!
//! Only transfers that the core handles in one burst are supported. Completion is polled, the
//! interrupts of the core stay masked. The driver does not do any cache maintenance, so the
//! buffers have to be in uncached memory. The addresses of the buffers are used as 32 bit
//! values.
//!
//! # Features
//!
//! - `defmt` implements `defmt::Format` for this crate's register and error types.
#![no_std]
#![deny(missing_docs)]

use core::marker::PhantomData;
use core::mem::size_of_val;

/// Raw register definitions.
pub mod regs;

use regs::fields::{Control, Flags, Irq, StartTransfer};

/// Errors of the DMAC drivers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Neither the source nor the destination of the core is memory mapped.
    #[error("neither the source nor the destination is memory mapped")]
    UnsupportedInterfaces,
    /// The core does not go in the direction of the driver type, or can not repeat transfers in
    /// hardware.
    #[error("the core does not have the direction or the mode of this driver")]
    WrongCore,
    /// The transfer is empty or longer than the core supports.
    #[error("the transfer is empty or too long")]
    InvalidLength,
    /// The address is not aligned to the width of the interface.
    #[error("the address is not aligned to the interface width")]
    Misaligned,
    /// The transfer queue of the core is full.
    #[error("the transfer queue is full")]
    QueueFull,
}

/// What both directions have in common.
struct Core {
    regs: regs::MmioRegisters<'static>,
    /// Longest transfer in bytes minus one.
    max_length: u32,
    /// Bytes per beat of the interface to the memory.
    width: u32,
}

/// The capabilities of a core.
struct Probe {
    regs: regs::MmioRegisters<'static>,
    hw_cyclic: bool,
    max_length: u32,
    dest_mem_mapped: bool,
    src_mem_mapped: bool,
    width_dest: u32,
    width_src: u32,
}

impl Probe {
    /// Detects the capabilities of the core, like `axi_dmac_init()`.
    fn new(mut regs: regs::MmioRegisters<'static>) -> Self {
        // Check if a hardware cyclic transfer is possible, then restore the flags
        let initial_flags = regs.read_flags();
        regs.write_flags(Flags::ZERO.with_cyclic(true));
        let hw_cyclic = regs.read_flags().raw_value() == Flags::ZERO.with_cyclic(true).raw_value();
        regs.write_flags(initial_flags);

        // The maximum transfer length is what sticks when writing all ones
        regs.write_x_length(u32::MAX);
        let max_length = regs.read_x_length();

        // A memory-mapped interface has address bits that stick
        regs.write_dest_address(u32::MAX);
        let dest_mem_mapped = regs.read_dest_address() != 0;
        regs.write_src_address(u32::MAX);
        let src_mem_mapped = regs.read_src_address() != 0;

        let interface = regs.read_interface_description();
        Self {
            hw_cyclic,
            max_length,
            dest_mem_mapped,
            src_mem_mapped,
            width_dest: 1 << interface.bytes_per_beat_dest_log2().value(),
            width_src: 1 << interface.bytes_per_beat_src_log2().value(),
            regs,
        }
    }
}

impl Core {
    /// Submits a transfer of `size` bytes to or from the memory at `address`, like
    /// `axi_dmac_transfer_start()`.
    fn submit(
        &mut self,
        address: usize,
        size: usize,
        to_memory: bool,
        cyclic: bool,
    ) -> Result<(), Error> {
        // The interfaces are 32 bit
        let (address, size) = (address as u32, size as u32);
        if size == 0 || size - 1 > self.max_length {
            return Err(Error::InvalidLength);
        }
        if address % self.width != 0 {
            return Err(Error::Misaligned);
        }

        // Clear the cyclic flag, and set it for a cyclic transfer
        self.regs.write_flags(Flags::ZERO.with_cyclic(cyclic));

        // Enable the core if it isn't enabled yet
        if !self.regs.read_control().enable() {
            self.regs.write_control(Control::ZERO);
            self.regs.write_control(Control::ZERO.with_enable(true));
            // Keep the interrupts masked: completion is polled through the raw source register
            self.regs.write_irq_mask(
                Irq::ZERO
                    .with_start_of_transfer(true)
                    .with_end_of_transfer(true),
            );
        }

        if self.regs.read_start_transfer().submit() {
            return Err(Error::QueueFull);
        }
        if to_memory {
            self.regs.write_dest_address(address);
            self.regs.write_dest_stride(0);
        } else {
            self.regs.write_src_address(address);
            self.regs.write_src_stride(0);
        }
        self.regs.write_x_length(size - 1);
        self.regs.write_y_length(0);
        self.regs
            .write_start_transfer(StartTransfer::ZERO.with_submit(true));
        Ok(())
    }

    /// Disables the core, which stops a running transfer.
    fn disable(&mut self) {
        self.regs.write_control(Control::ZERO);
    }
}

/// A core that transfers data from a device into memory (like the ADC DMA).
pub struct RxDmac {
    core: Core,
}

impl RxDmac {
    /// Detects the capabilities of the core at `base_addr`. Fails if the core doesn't send data
    /// from a device to memory.
    ///
    /// # Safety
    ///
    /// - The `base_addr` must be a valid memory-mapped register address of an AXI DMAC core.
    /// - Dereferencing an invalid or misaligned address results in **undefined behavior**.
    /// - The caller must ensure that no other code concurrently modifies the same peripheral
    ///   registers in an unsynchronized manner to prevent data races.
    /// - This function does not enforce uniqueness of driver instances. Creating multiple
    ///   instances with the same `base_addr` can lead to unintended behavior if not externally
    ///   synchronized.
    /// - The driver performs **volatile** reads and writes to the provided address.
    pub unsafe fn new(base_addr: usize) -> Result<Self, Error> {
        Self::from_registers(unsafe { regs::Registers::new_mmio_at(base_addr) })
    }

    fn from_registers(regs: regs::MmioRegisters<'static>) -> Result<Self, Error> {
        let probe = Probe::new(regs);
        match (probe.dest_mem_mapped, probe.src_mem_mapped) {
            (true, false) => Ok(Self {
                core: Core {
                    regs: probe.regs,
                    max_length: probe.max_length,
                    width: probe.width_dest,
                },
            }),
            (false, false) => Err(Error::UnsupportedInterfaces),
            _ => Err(Error::WrongCore),
        }
    }

    /// The longest transfer in bytes.
    pub fn max_transfer_bytes(&self) -> u64 {
        self.core.max_length as u64 + 1
    }

    /// Starts a transfer that fills `buffer` from the device. The buffer is borrowed until the
    /// transfer is finished or dropped.
    pub fn start<'a, T: Copy>(&'a mut self, buffer: &'a mut [T]) -> Result<RxTransfer<'a>, Error> {
        self.core.submit(
            buffer.as_mut_ptr() as usize,
            size_of_val(buffer),
            true,
            false,
        )?;
        Ok(RxTransfer {
            dmac: self,
            finished: false,
            _buffer: PhantomData,
        })
    }
}

/// A running transfer from a device into a buffer. Dropping it before it finished stops the
/// core.
pub struct RxTransfer<'a> {
    dmac: &'a mut RxDmac,
    finished: bool,
    _buffer: PhantomData<&'a mut [u8]>,
}

impl<'a> RxTransfer<'a> {
    /// Whether the core reported the start and the end of the transfer.
    pub fn is_finished(&mut self) -> bool {
        let source = self.dmac.core.regs.read_irq_source();
        source.start_of_transfer() && source.end_of_transfer()
    }

    /// Ends the transfer if it is finished, which completes the buffer. Otherwise the transfer
    /// is given back to try again.
    pub fn try_finish(mut self) -> Result<(), Self> {
        if !self.is_finished() {
            return Err(self);
        }
        let source = self.dmac.core.regs.read_irq_source();
        self.dmac.core.regs.write_irq_pending(source);
        self.finished = true;
        Ok(())
    }
}

impl Drop for RxTransfer<'_> {
    fn drop(&mut self) {
        if !self.finished {
            self.dmac.core.disable();
        }
    }
}

/// A core that transfers data from memory to a device, repeating it in hardware (like the DAC
/// DMA).
pub struct TxDmac {
    core: Core,
}

impl TxDmac {
    /// Detects the capabilities of the core at `base_addr`. Fails if the core doesn't send data
    /// from memory to a device, or can't repeat transfers in hardware.
    ///
    /// # Safety
    ///
    /// - The `base_addr` must be a valid memory-mapped register address of an AXI DMAC core.
    /// - Dereferencing an invalid or misaligned address results in **undefined behavior**.
    /// - The caller must ensure that no other code concurrently modifies the same peripheral
    ///   registers in an unsynchronized manner to prevent data races.
    /// - This function does not enforce uniqueness of driver instances. Creating multiple
    ///   instances with the same `base_addr` can lead to unintended behavior if not externally
    ///   synchronized.
    /// - The driver performs **volatile** reads and writes to the provided address.
    pub unsafe fn new(base_addr: usize) -> Result<Self, Error> {
        Self::from_registers(unsafe { regs::Registers::new_mmio_at(base_addr) })
    }

    fn from_registers(regs: regs::MmioRegisters<'static>) -> Result<Self, Error> {
        let probe = Probe::new(regs);
        match (probe.dest_mem_mapped, probe.src_mem_mapped) {
            (false, true) if probe.hw_cyclic => Ok(Self {
                core: Core {
                    regs: probe.regs,
                    max_length: probe.max_length,
                    width: probe.width_src,
                },
            }),
            (false, false) => Err(Error::UnsupportedInterfaces),
            _ => Err(Error::WrongCore),
        }
    }

    /// The longest transfer in bytes.
    pub fn max_transfer_bytes(&self) -> u64 {
        self.core.max_length as u64 + 1
    }

    /// Starts sending `buffer` to the device over and over. The buffer is borrowed until the
    /// transfer is stopped or dropped.
    pub fn start_cyclic<'a, T: Copy>(
        &'a mut self,
        buffer: &'a [T],
    ) -> Result<CyclicTransfer<'a>, Error> {
        self.core
            .submit(buffer.as_ptr() as usize, size_of_val(buffer), false, true)?;
        Ok(CyclicTransfer {
            dmac: self,
            _buffer: PhantomData,
        })
    }
}

/// A running cyclic transfer to a device. Dropping it stops the core.
pub struct CyclicTransfer<'a> {
    dmac: &'a mut TxDmac,
    _buffer: PhantomData<&'a [u8]>,
}

impl CyclicTransfer<'_> {
    /// Stops the transfer, which releases the buffer.
    pub fn stop(self) {}
}

impl Drop for CyclicTransfer<'_> {
    fn drop(&mut self) {
        self.dmac.core.disable();
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::{boxed::Box, vec};

    use super::regs::fields::InterfaceDescription;
    use super::*;

    /// A buffer aligned for the 8 byte interfaces of the test cores.
    #[repr(align(16))]
    struct Buffer([u32; 64]);

    /// Two register handles to the same memory: one for the driver under test, and one for the
    /// test, which reads what the driver did and plays the part of the core.
    fn registers() -> (regs::MmioRegisters<'static>, regs::MmioRegisters<'static>) {
        let memory: &'static mut [u32] = Box::leak(vec![0u32; 0x110].into_boxed_slice());
        let base = memory.as_mut_ptr() as usize;
        // Safety: the memory is big enough for the register block and nothing else uses it
        unsafe {
            (
                regs::Registers::new_mmio_at(base),
                regs::Registers::new_mmio_at(base),
            )
        }
    }

    /// The core reports the interrupt sources `source`.
    fn report(hardware: &regs::MmioRegisters<'static>, source: Irq) {
        // Safety: the pointer is valid for the register, which the driver only reads
        unsafe { hardware.pointer_to_irq_source().write_volatile(source) }
    }

    fn core(driver: regs::MmioRegisters<'static>, max_length: u32, width: u32) -> Core {
        Core {
            regs: driver,
            max_length,
            width,
        }
    }

    #[test]
    fn interface_description_gives_the_widths() {
        let (driver, hardware) = registers();
        // 8 bytes per beat at the destination, 4 at the source
        unsafe {
            hardware
                .pointer_to_interface_description()
                .write_volatile(InterfaceDescription::new_with_raw_value((2 << 8) | 3));
        }
        let probe = Probe::new(driver);
        assert_eq!((probe.width_dest, probe.width_src), (8, 4));
    }

    #[test]
    fn transfer_is_submitted_to_the_memory_side() {
        let (driver, hardware) = registers();
        let mut core = core(driver, 0xFFFF, 8);
        core.submit(0x1000_0000, 4096, true, false).unwrap();
        assert_eq!(hardware.read_dest_address(), 0x1000_0000);
        assert_eq!(hardware.read_dest_stride(), 0);
        assert_eq!(hardware.read_x_length(), 4095);
        assert_eq!(hardware.read_y_length(), 0);
        assert!(hardware.read_start_transfer().submit());
        // The core is enabled with the interrupts masked, and the transfer is not cyclic
        assert!(hardware.read_control().enable());
        assert!(hardware.read_irq_mask().start_of_transfer());
        assert!(hardware.read_irq_mask().end_of_transfer());
        assert!(!hardware.read_flags().cyclic());
    }

    #[test]
    fn cyclic_transfer_sets_the_flag_and_the_source_address() {
        let (driver, hardware) = registers();
        let mut core = core(driver, 0xFFFF, 8);
        core.submit(0x2000_0000, 1024, false, true).unwrap();
        assert_eq!(hardware.read_src_address(), 0x2000_0000);
        assert_eq!(hardware.read_x_length(), 1023);
        assert!(hardware.read_flags().cyclic());
    }

    #[test]
    fn bad_transfers_are_rejected_before_touching_the_queue() {
        let (driver, hardware) = registers();
        let mut core = core(driver, 0xFF, 8);
        assert_eq!(
            core.submit(0x1000, 0, true, false),
            Err(Error::InvalidLength)
        );
        assert_eq!(
            core.submit(0x1000, 0x101, true, false),
            Err(Error::InvalidLength)
        );
        assert_eq!(core.submit(0x1004, 64, true, false), Err(Error::Misaligned));
        assert!(!hardware.read_start_transfer().submit());
    }

    #[test]
    fn a_full_queue_is_reported() {
        let (driver, _hardware) = registers();
        let mut core = core(driver, 0xFFFF, 8);
        core.submit(0x1000, 64, true, false).unwrap();
        // The submit bit reads back as set: the queue is full
        assert_eq!(core.submit(0x1000, 64, true, false), Err(Error::QueueFull));
    }

    #[test]
    fn rx_transfer_finishes_when_start_and_end_are_reported() {
        let (driver, hardware) = registers();
        let mut rx = RxDmac {
            core: core(driver, 0xFFFF, 8),
        };
        let mut buffer = Buffer([0; 64]);
        let transfer = rx.start(&mut buffer.0).unwrap();

        // Only the start was reported
        report(&hardware, Irq::ZERO.with_start_of_transfer(true));
        let transfer = transfer.try_finish().unwrap_err();

        report(
            &hardware,
            Irq::ZERO
                .with_start_of_transfer(true)
                .with_end_of_transfer(true),
        );
        assert!(transfer.try_finish().is_ok());
        // The reported bits were cleared, and the core is left enabled
        let pending = hardware.read_irq_pending();
        assert!(pending.start_of_transfer() && pending.end_of_transfer());
        assert!(hardware.read_control().enable());
    }

    #[test]
    fn dropping_a_transfer_stops_the_core() {
        let (driver, hardware) = registers();
        let mut rx = RxDmac {
            core: core(driver, 0xFFFF, 8),
        };
        let mut buffer = Buffer([0; 64]);
        drop(rx.start(&mut buffer.0).unwrap());
        assert!(!hardware.read_control().enable());
    }
}
