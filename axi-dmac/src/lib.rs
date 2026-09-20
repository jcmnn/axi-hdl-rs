//! # ADI AXI DMAC driver
//!
//! This is a native Rust driver for the
//! [ADI AXI DMAC IP core](https://analogdevicesinc.github.io/hdl/library/axi_dmac/index.html),
//! used for example for the DMAs of the AXI AD9361 IP core. It is not the AMD AXI DMA IP core,
//! there is the `axi-dma` crate for that one.
//!
//! The capabilities of a core are probed once. The result is an [RxDmac] (device to memory) or
//! a [TxDmac] (memory to device), so the transfer functions only exist for the direction the
//! core has. A transfer borrows its buffer until it is finished or dropped, and dropping it
//! stops the core.
//!
//! Only transfers that the core handles in one burst are supported. There are two ways to use
//! a core:
//!
//! - Blocking, with the transfer objects returned by [RxDmac::start], [TxDmac::start] and
//!   [TxDmac::start_cyclic]. [RxTransfer::poll] checks for completion without blocking and
//!   [RxTransfer::wait] blocks until the transfer is done. The interrupts of the core stay
//!   masked. [TxDmac::start_cyclic] repeats a transfer in hardware until it is stopped, if the
//!   core supports that.
//! - Asynchronous, with [RxDmacAsync::read] and [TxDmacAsync::write], which are driven by the
//!   end of transfer interrupt of the core. Your interrupt handler has to call
//!   [RxDmacAsync::on_interrupt] or [TxDmacAsync::on_interrupt]. A driver is turned into its
//!   asynchronous variant with [RxDmac::into_async] or [TxDmac::into_async].
//!
//! ## Cache maintenance
//!
//! The driver does not do any cache maintenance, so the buffers have to be in uncached memory,
//! or you have to clean the cache lines of a buffer before a transfer from it, and invalidate
//! the cache lines of a buffer after a transfer to it. The addresses of the buffers are used as
//! 32 bit values.
//!
//! # Features
//!
//! - `1-waker`, `2-wakers`, `4-wakers`, `8-wakers`, `16-wakers`, `32-wakers` select
//!   [NUM_WAKERS], the size of the global waker table backing the asynchronous drivers. Only one
//!   of these can be active at a time. Each asynchronous driver (each `waker_index` passed to
//!   [RxDmac::into_async] or [TxDmac::into_async]) needs its own slot, so this bounds how many
//!   asynchronous DMACs can exist across all cores. `1-waker` is the default. This can not be
//!   changed at runtime, so you have to pick a build-time upper bound.
//! - `portable-atomic` switches every atomic type this crate uses from `core::sync::atomic` to
//!   the [`portable-atomic` crate](https://docs.rs/portable-atomic)'s equivalents. Enable this
//!   if your target does not provide native atomics.
//! - `defmt` implements `defmt::Format` for this crate's register and error types.
#![no_std]
#![deny(missing_docs)]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::size_of_val;
use core::sync::atomic::Ordering;
use core::task::Poll;

#[cfg(not(feature = "portable-atomic"))]
use core::sync::atomic::AtomicBool;
use embassy_sync::waitqueue::AtomicWaker;
#[cfg(feature = "portable-atomic")]
use portable_atomic::AtomicBool;

/// Raw register definitions.
pub mod regs;

use regs::fields::{Control, Flags, Irq, StartTransfer};

/// 1 waker (default).
#[cfg(feature = "1-waker")]
pub const NUM_WAKERS: usize = 1;
/// 2 wakers
#[cfg(feature = "2-wakers")]
pub const NUM_WAKERS: usize = 2;
/// 4 wakers
#[cfg(feature = "4-wakers")]
pub const NUM_WAKERS: usize = 4;
/// 8 wakers
#[cfg(feature = "8-wakers")]
pub const NUM_WAKERS: usize = 8;
/// 16 wakers
#[cfg(feature = "16-wakers")]
pub const NUM_WAKERS: usize = 16;
/// 32 wakers
#[cfg(feature = "32-wakers")]
pub const NUM_WAKERS: usize = 32;

static WAKERS: [AtomicWaker; NUM_WAKERS] = [const { AtomicWaker::new() }; NUM_WAKERS];
/// Set by the interrupt handler when the end of the transfer of a slot was reported.
static TRANSFER_DONE: [AtomicBool; NUM_WAKERS] = [const { AtomicBool::new(false) }; NUM_WAKERS];
/// Global ownership table for waker slots, shared by every asynchronous driver, since `WAKERS`
/// and `TRANSFER_DONE` are global too. Claimed atomically via [claim_waker].
static WAKER_TAKEN: [AtomicBool; NUM_WAKERS] = [const { AtomicBool::new(false) }; NUM_WAKERS];

/// Errors of the DMAC drivers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Neither the source nor the destination of the core is memory mapped.
    #[error("neither the source nor the destination is memory mapped")]
    UnsupportedInterfaces,
    /// The core does not go in the direction of the driver type.
    #[error("the core does not have the direction of this driver")]
    WrongCore,
    /// The core can not repeat transfers in hardware.
    #[error("the core does not support cyclic transfers")]
    CyclicUnsupported,
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

/// Error returned when claiming a waker slot for an asynchronous driver fails.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClaimWakerError {
    /// `waker_index` is out of range for [NUM_WAKERS].
    #[error("invalid waker slot index: {0}")]
    InvalidWakerIndex(usize),
    /// `waker_index` was already claimed by another asynchronous driver.
    #[error("waker slot index {0} is already in use by another asynchronous driver")]
    WakerIndexInUse(usize),
}

/// Atomically claims `waker_index` in the global [WAKER_TAKEN] table. There is no matching
/// release: a claim lasts for the program's lifetime.
fn claim_waker(waker_index: usize) -> Result<(), ClaimWakerError> {
    if waker_index >= NUM_WAKERS {
        return Err(ClaimWakerError::InvalidWakerIndex(waker_index));
    }
    WAKER_TAKEN[waker_index]
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .map_err(|_| ClaimWakerError::WakerIndexInUse(waker_index))?;
    Ok(())
}

/// What both directions have in common.
struct Core {
    regs: regs::MmioRegisters<'static>,
    /// Longest transfer in bytes minus one.
    max_length: u32,
    /// Bytes per beat of the interface to the memory.
    width: u32,
    /// The interrupts to mask when the core is enabled. A set bit masks the interrupt.
    irq_mask: Irq,
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

/// Both interrupt bits.
const IRQ_ALL: Irq = Irq::ZERO
    .with_start_of_transfer(true)
    .with_end_of_transfer(true);

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
            self.regs.write_irq_mask(self.irq_mask);
        }

        if self.regs.read_start_transfer().submit() {
            return Err(Error::QueueFull);
        }
        // Forget what earlier transfers reported
        self.regs.write_irq_pending(IRQ_ALL);
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

/// A submitted transfer that ends by itself. Dropping it before it is done stops the core.
struct Running<'a> {
    core: &'a mut Core,
    finished: bool,
}

impl<'a> Running<'a> {
    fn new(core: &'a mut Core) -> Self {
        Self {
            core,
            finished: false,
        }
    }

    /// Whether the core reported the start and the end of the transfer.
    fn poll(&mut self) -> bool {
        let source = self.core.regs.read_irq_source();
        source.start_of_transfer() && source.end_of_transfer()
    }

    fn wait(mut self) {
        while !self.poll() {}
        self.complete();
    }

    /// Ends the transfer, which leaves the core enabled.
    fn complete(mut self) {
        self.core.regs.write_irq_pending(IRQ_ALL);
        self.finished = true;
    }
}

impl Drop for Running<'_> {
    fn drop(&mut self) {
        if self.finished {
            return;
        }
        if self.poll() {
            // The transfer is done, which leaves the core enabled
            self.core.regs.write_irq_pending(IRQ_ALL);
        } else {
            self.core.disable();
        }
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
                    irq_mask: IRQ_ALL,
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
            running: Running::new(&mut self.core),
            _buffer: PhantomData,
        })
    }

    /// Turns the driver into one that awaits the transfers, driven by the end of transfer
    /// interrupt. The interrupt handler has to call [RxDmacAsync::on_interrupt] with the token
    /// of the returned driver. `waker_index` selects the slot of the global waker table, see
    /// [NUM_WAKERS].
    pub fn into_async(mut self, waker_index: usize) -> Result<RxDmacAsync, ClaimWakerError> {
        let token = claim_async(&mut self.core, waker_index)?;
        Ok(RxDmacAsync { dmac: self, token })
    }
}

/// Claims `waker_index` and unmasks the end of transfer interrupt of the core.
fn claim_async(core: &mut Core, waker_index: usize) -> Result<DmacToken, ClaimWakerError> {
    claim_waker(waker_index)?;
    // Only the end of the transfer interrupts, the start is still latched in the source bits
    core.irq_mask = Irq::ZERO.with_start_of_transfer(true);
    core.regs.write_irq_mask(core.irq_mask);
    Ok(DmacToken {
        // SAFETY: Only converted to primitive address
        base_addr: unsafe { core.regs.ptr() } as usize,
        waker_index,
    })
}

/// A running transfer from a device into a buffer. Dropping it before it finished stops the
/// core.
pub struct RxTransfer<'a> {
    running: Running<'a>,
    _buffer: PhantomData<&'a mut [u8]>,
}

impl RxTransfer<'_> {
    /// Non-blocking check for completion. Returns `true` once the core reported the start and
    /// the end of the transfer, which means the buffer is complete.
    #[inline]
    pub fn poll(&mut self) -> bool {
        self.running.poll()
    }

    /// Blocks until the transfer is done, which completes the buffer.
    pub fn wait(self) {
        self.running.wait();
    }
}

/// A core that transfers data from memory to a device (like the DAC DMA).
pub struct TxDmac {
    core: Core,
    hw_cyclic: bool,
}

impl TxDmac {
    /// Detects the capabilities of the core at `base_addr`. Fails if the core doesn't send data
    /// from memory to a device.
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
            (false, true) => Ok(Self {
                core: Core {
                    regs: probe.regs,
                    max_length: probe.max_length,
                    width: probe.width_src,
                    irq_mask: IRQ_ALL,
                },
                hw_cyclic: probe.hw_cyclic,
            }),
            (false, false) => Err(Error::UnsupportedInterfaces),
            _ => Err(Error::WrongCore),
        }
    }

    /// The longest transfer in bytes.
    pub fn max_transfer_bytes(&self) -> u64 {
        self.core.max_length as u64 + 1
    }

    /// Whether the core can repeat transfers in hardware.
    pub fn supports_cyclic(&self) -> bool {
        self.hw_cyclic
    }

    /// Starts sending `buffer` to the device once. The buffer is borrowed until the transfer is
    /// finished or dropped.
    pub fn start<'a, T: Copy>(&'a mut self, buffer: &'a [T]) -> Result<TxTransfer<'a>, Error> {
        self.core
            .submit(buffer.as_ptr() as usize, size_of_val(buffer), false, false)?;
        Ok(TxTransfer {
            running: Running::new(&mut self.core),
            _buffer: PhantomData,
        })
    }

    /// Starts sending `buffer` to the device over and over. The buffer is borrowed until the
    /// transfer is stopped or dropped. Fails if the core can not repeat transfers in hardware.
    pub fn start_cyclic<'a, T: Copy>(
        &'a mut self,
        buffer: &'a [T],
    ) -> Result<CyclicTransfer<'a>, Error> {
        if !self.hw_cyclic {
            return Err(Error::CyclicUnsupported);
        }
        self.core
            .submit(buffer.as_ptr() as usize, size_of_val(buffer), false, true)?;
        Ok(CyclicTransfer {
            core: &mut self.core,
            _buffer: PhantomData,
        })
    }

    /// Turns the driver into one that awaits the transfers, driven by the end of transfer
    /// interrupt. The interrupt handler has to call [TxDmacAsync::on_interrupt] with the token
    /// of the returned driver. `waker_index` selects the slot of the global waker table, see
    /// [NUM_WAKERS].
    pub fn into_async(mut self, waker_index: usize) -> Result<TxDmacAsync, ClaimWakerError> {
        let token = claim_async(&mut self.core, waker_index)?;
        Ok(TxDmacAsync { dmac: self, token })
    }
}

/// A running transfer from a buffer to a device. Dropping it before it finished stops the core.
pub struct TxTransfer<'a> {
    running: Running<'a>,
    _buffer: PhantomData<&'a [u8]>,
}

impl TxTransfer<'_> {
    /// Non-blocking check for completion. Returns `true` once the core reported the start and
    /// the end of the transfer.
    #[inline]
    pub fn poll(&mut self) -> bool {
        self.running.poll()
    }

    /// Blocks until the transfer is done.
    pub fn wait(self) {
        self.running.wait();
    }
}

/// A running cyclic transfer to a device. Dropping it stops the core.
pub struct CyclicTransfer<'a> {
    core: &'a mut Core,
    _buffer: PhantomData<&'a [u8]>,
}

impl CyclicTransfer<'_> {
    /// Stops the transfer, which releases the buffer.
    pub fn stop(self) {}
}

impl Drop for CyclicTransfer<'_> {
    fn drop(&mut self) {
        self.core.disable();
    }
}

/// Identifies the core of an asynchronous driver, for use in an interrupt handler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmacToken {
    base_addr: usize,
    waker_index: usize,
}

impl DmacToken {
    /// The base address of the core.
    #[inline]
    pub fn base_addr(&self) -> usize {
        self.base_addr
    }

    /// The waker slot the driver was constructed with.
    #[inline]
    pub fn waker_index(&self) -> usize {
        self.waker_index
    }

    /// Constructs a token from a raw base address and waker index, e.g. for an interrupt handler
    /// that only has these two values available from static configuration.
    ///
    /// # Safety
    ///
    /// The caller must ensure `base_addr` is the real base address of the core whose interrupt
    /// is being serviced, and that `waker_index` matches the slot originally passed to
    /// [RxDmac::into_async] or [TxDmac::into_async].
    #[inline]
    pub unsafe fn steal(waker_index: usize, base_addr: usize) -> Self {
        Self {
            base_addr,
            waker_index,
        }
    }
}

/// Services the end of transfer interrupt of the core identified by `token`. Returns whether
/// the end of a transfer was reported.
///
/// # Safety
///
/// `token` must identify an asynchronous driver, see [DmacToken::steal].
unsafe fn service_interrupt(token: &DmacToken) -> bool {
    let mut regs = unsafe { regs::Registers::new_mmio_at(token.base_addr) };
    if !regs.read_irq_pending().end_of_transfer() {
        return false;
    }
    regs.write_irq_pending(Irq::ZERO.with_end_of_transfer(true));
    TRANSFER_DONE[token.waker_index].store(true, Ordering::Release);
    WAKERS[token.waker_index].wake();
    true
}

/// Waits for the interrupt handler to report the end of a transfer of the slot `waker_index`.
async fn transfer_done(waker_index: usize) {
    poll_fn(|cx| {
        WAKERS[waker_index].register(cx.waker());
        if TRANSFER_DONE[waker_index].load(Ordering::Acquire) {
            return Poll::Ready(());
        }
        Poll::Pending
    })
    .await;
}

/// An [RxDmac] whose transfers are awaited, driven by the end of transfer interrupt.
pub struct RxDmacAsync {
    dmac: RxDmac,
    token: DmacToken,
}

impl RxDmacAsync {
    /// The token identifying this driver's core, fixed for the driver's whole lifetime. Retrieve
    /// it once, right after construction, to hand to your interrupt handler.
    #[inline]
    pub fn token(&self) -> DmacToken {
        self.token
    }

    /// The longest transfer in bytes.
    pub fn max_transfer_bytes(&self) -> u64 {
        self.dmac.max_transfer_bytes()
    }

    /// Fills `buffer` from the device and awaits the end of the transfer, driven by
    /// [Self::on_interrupt]. Dropping the future stops the core.
    pub async fn read<T: Copy>(&mut self, buffer: &mut [T]) -> Result<(), Error> {
        TRANSFER_DONE[self.token.waker_index].store(false, Ordering::Release);
        let transfer = self.dmac.start(buffer)?;
        transfer_done(self.token.waker_index).await;
        transfer.running.complete();
        Ok(())
    }

    /// Services the end of transfer interrupt of the core identified by `token`. Call this from
    /// your interrupt handler. Returns whether the end of a transfer was reported.
    ///
    /// # Safety
    ///
    /// `token` must have been returned by [Self::token], or constructed via [DmacToken::steal]
    /// to match, and identify a core that is used by an [RxDmacAsync].
    pub unsafe fn on_interrupt(token: &DmacToken) -> bool {
        unsafe { service_interrupt(token) }
    }
}

/// A [TxDmac] whose transfers are awaited, driven by the end of transfer interrupt.
pub struct TxDmacAsync {
    dmac: TxDmac,
    token: DmacToken,
}

impl TxDmacAsync {
    /// The token identifying this driver's core, fixed for the driver's whole lifetime. Retrieve
    /// it once, right after construction, to hand to your interrupt handler.
    #[inline]
    pub fn token(&self) -> DmacToken {
        self.token
    }

    /// The longest transfer in bytes.
    pub fn max_transfer_bytes(&self) -> u64 {
        self.dmac.max_transfer_bytes()
    }

    /// Sends `buffer` to the device and awaits the end of the transfer, driven by
    /// [Self::on_interrupt]. Dropping the future stops the core.
    pub async fn write<T: Copy>(&mut self, buffer: &[T]) -> Result<(), Error> {
        TRANSFER_DONE[self.token.waker_index].store(false, Ordering::Release);
        let transfer = self.dmac.start(buffer)?;
        transfer_done(self.token.waker_index).await;
        transfer.running.complete();
        Ok(())
    }

    /// Services the end of transfer interrupt of the core identified by `token`. Call this from
    /// your interrupt handler. Returns whether the end of a transfer was reported.
    ///
    /// # Safety
    ///
    /// `token` must have been returned by [Self::token], or constructed via [DmacToken::steal]
    /// to match, and identify a core that is used by a [TxDmacAsync].
    pub unsafe fn on_interrupt(token: &DmacToken) -> bool {
        unsafe { service_interrupt(token) }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Waker};
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
            irq_mask: IRQ_ALL,
        }
    }

    fn rx(driver: regs::MmioRegisters<'static>) -> RxDmac {
        RxDmac {
            core: core(driver, 0xFFFF, 8),
        }
    }

    fn tx(driver: regs::MmioRegisters<'static>, hw_cyclic: bool) -> TxDmac {
        TxDmac {
            core: core(driver, 0xFFFF, 8),
            hw_cyclic,
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
    fn starting_a_transfer_forgets_the_reports_of_the_last_one() {
        let (driver, hardware) = registers();
        let mut rx = rx(driver);
        let mut buffer = Buffer([0; 64]);
        drop(rx.start(&mut buffer.0).unwrap());
        // The interrupt sources are cleared with a write of ones to the pending register
        let pending = hardware.read_irq_pending();
        assert!(pending.start_of_transfer() && pending.end_of_transfer());
    }

    #[test]
    fn rx_transfer_polls_until_start_and_end_are_reported() {
        let (driver, hardware) = registers();
        let mut rx = rx(driver);
        let mut buffer = Buffer([0; 64]);
        let mut transfer = rx.start(&mut buffer.0).unwrap();
        assert!(!transfer.poll());

        // Only the start was reported
        report(&hardware, Irq::ZERO.with_start_of_transfer(true));
        assert!(!transfer.poll());

        report(&hardware, IRQ_ALL);
        assert!(transfer.poll());
        // Dropping a transfer that is done leaves the core enabled
        drop(transfer);
        assert!(hardware.read_control().enable());
    }

    #[test]
    fn waiting_ends_the_transfer_and_leaves_the_core_enabled() {
        let (driver, hardware) = registers();
        let mut rx = rx(driver);
        let mut buffer = Buffer([0; 64]);
        let transfer = rx.start(&mut buffer.0).unwrap();
        report(&hardware, IRQ_ALL);
        transfer.wait();
        assert!(hardware.read_control().enable());
    }

    #[test]
    fn tx_transfer_is_read_from_the_memory() {
        let (driver, hardware) = registers();
        let mut tx = tx(driver, false);
        let buffer = Buffer([0; 64]);
        let mut transfer = tx.start(&buffer.0).unwrap();
        assert_eq!(
            hardware.read_src_address(),
            buffer.0.as_ptr() as usize as u32
        );
        assert_eq!(hardware.read_x_length(), 4 * 64 - 1);
        assert!(!hardware.read_flags().cyclic());
        report(&hardware, IRQ_ALL);
        assert!(transfer.poll());
        transfer.wait();
    }

    #[test]
    fn cyclic_transfers_need_hardware_support() {
        let (driver, hardware) = registers();
        let mut tx = tx(driver, false);
        let buffer = Buffer([0; 64]);
        assert!(!tx.supports_cyclic());
        assert!(matches!(
            tx.start_cyclic(&buffer.0),
            Err(Error::CyclicUnsupported)
        ));
        assert!(!hardware.read_start_transfer().submit());
    }

    #[test]
    fn dropping_a_transfer_stops_the_core() {
        let (driver, hardware) = registers();
        let mut rx = rx(driver);
        let mut buffer = Buffer([0; 64]);
        drop(rx.start(&mut buffer.0).unwrap());
        assert!(!hardware.read_control().enable());
    }

    #[test]
    fn stopping_a_cyclic_transfer_stops_the_core() {
        let (driver, hardware) = registers();
        let mut tx = tx(driver, true);
        let buffer = Buffer([0; 64]);
        let transfer = tx.start_cyclic(&buffer.0).unwrap();
        assert!(hardware.read_flags().cyclic());
        transfer.stop();
        assert!(!hardware.read_control().enable());
    }

    #[test]
    fn waker_slots_can_only_be_claimed_once() {
        // The tests share the global table, so this uses the last slot
        let index = NUM_WAKERS - 1;
        assert_eq!(
            claim_waker(NUM_WAKERS),
            Err(ClaimWakerError::InvalidWakerIndex(NUM_WAKERS))
        );
        // Slot 0 is claimed by the asynchronous test, which is the only slot with 1 waker
        if index != 0 {
            assert_eq!(claim_waker(index), Ok(()));
            assert_eq!(
                claim_waker(index),
                Err(ClaimWakerError::WakerIndexInUse(index))
            );
        }
    }

    #[test]
    fn async_read_is_driven_by_the_interrupt() {
        let (driver, mut hardware) = registers();
        let mut rx = rx(driver).into_async(0).unwrap();
        // The end of transfer interrupt is unmasked, the start is not
        assert!(hardware.read_irq_mask().start_of_transfer());
        assert!(!hardware.read_irq_mask().end_of_transfer());

        let token = rx.token();
        assert_eq!(token.waker_index(), 0);
        let mut buffer = Buffer([0; 64]);
        let mut cx = Context::from_waker(Waker::noop());
        {
            let mut read = pin!(rx.read(&mut buffer.0));
            assert!(read.as_mut().poll(&mut cx).is_pending());
            assert!(hardware.read_control().enable());

            // No end of transfer, no interrupt. (The register model keeps the ones the driver
            // wrote to clear the sources, the core would have cleared them.)
            hardware.write_irq_pending(Irq::ZERO);
            assert!(!unsafe { RxDmacAsync::on_interrupt(&token) });
            assert!(read.as_mut().poll(&mut cx).is_pending());

            // The interrupt handler sees the end of transfer, which wakes and finishes the read
            hardware.write_irq_pending(Irq::ZERO.with_end_of_transfer(true));
            assert!(unsafe { RxDmacAsync::on_interrupt(&token) });
            assert_eq!(read.as_mut().poll(&mut cx), core::task::Poll::Ready(Ok(())));
        }
        // The core is left enabled after a finished transfer
        assert!(hardware.read_control().enable());

        // Dropping a read that is still running stops the core. (The core took the submitted
        // transfer from the queue.)
        hardware.write_start_transfer(StartTransfer::ZERO);
        {
            let mut read = pin!(rx.read(&mut buffer.0));
            assert!(read.as_mut().poll(&mut cx).is_pending());
            assert!(hardware.read_control().enable());
        }
        assert!(!hardware.read_control().enable());
    }
}
