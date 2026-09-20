//! # Register definitions.

use fields::{Control, Flags, InterfaceDescription, Irq, StartTransfer};

/// Register fields.
pub mod fields {
    use arbitrary_int::u4;

    /// Interface description, `AXI_DMAC_REG_INTERFACE_DESC`.
    #[bitbybit::bitfield(u32, debug, default = 0x0, defmt_bitfields(feature = "defmt"))]
    pub struct InterfaceDescription {
        /// Log2 of the bytes per beat of the source interface.
        #[bits(8..=11, r)]
        bytes_per_beat_src_log2: u4,
        /// Log2 of the bytes per beat of the destination interface.
        #[bits(0..=3, r)]
        bytes_per_beat_dest_log2: u4,
    }

    /// Interrupt bits of the mask, pending and source registers.
    #[bitbybit::bitfield(u32, debug, default = 0x0, defmt_bitfields(feature = "defmt"))]
    pub struct Irq {
        /// End of transfer.
        #[bit(1, rw)]
        end_of_transfer: bool,
        /// Start of transfer.
        #[bit(0, rw)]
        start_of_transfer: bool,
    }

    /// Control register, `AXI_DMAC_REG_CTRL`.
    #[bitbybit::bitfield(u32, debug, default = 0x0, defmt_bitfields(feature = "defmt"))]
    pub struct Control {
        /// Pauses the core.
        #[bit(1, rw)]
        pause: bool,
        /// Enables the core. Clearing it stops a running transfer.
        #[bit(0, rw)]
        enable: bool,
    }

    /// Transfer submission register, `AXI_DMAC_REG_START_TRANSFER`.
    #[bitbybit::bitfield(u32, debug, default = 0x0, defmt_bitfields(feature = "defmt"))]
    pub struct StartTransfer {
        /// Reads 1 while the transfer queue is full. Writing 1 submits the transfer.
        #[bit(0, rw)]
        submit: bool,
    }

    /// Transfer flags, `AXI_DMAC_REG_FLAGS`.
    #[bitbybit::bitfield(u32, debug, default = 0x0, defmt_bitfields(feature = "defmt"))]
    pub struct Flags {
        /// The transfer repeats until the core is disabled. Only cores which support this in
        /// hardware keep the bit.
        #[bit(0, rw)]
        cyclic: bool,
    }
}

/// Register block.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Core version.
    #[mmio(PureRead)]
    version: u32,
    /// Peripheral ID.
    #[mmio(PureRead)]
    peripheral_id: u32,
    /// Scratch register.
    scratch: u32,
    /// Identification, reads `DMAC` as ASCII.
    #[mmio(PureRead)]
    identification: u32,
    /// Interface description: the widths of the source and destination interfaces.
    #[mmio(PureRead)]
    interface_description: InterfaceDescription,

    _gap0: [u32; 0x1B],

    /// Interrupt mask. A set bit masks the interrupt.
    irq_mask: Irq,
    /// Pending interrupts (the unmasked sources). Writing 1 clears the source bit.
    irq_pending: Irq,
    /// Interrupt sources, latched regardless of the mask.
    #[mmio(PureRead)]
    irq_source: Irq,

    _gap1: [u32; 0xDD],

    /// Control register.
    control: Control,
    /// ID of the last submitted transfer.
    #[mmio(PureRead)]
    transfer_id: u32,
    /// Transfer submission register.
    start_transfer: StartTransfer,
    /// Flags of the next transfer.
    flags: Flags,
    /// Destination address of the next transfer.
    dest_address: u32,
    /// Source address of the next transfer.
    src_address: u32,
    /// Bytes per row of the next transfer, minus one.
    x_length: u32,
    /// Rows of the next transfer, minus one.
    y_length: u32,
    /// Destination stride between two rows.
    dest_stride: u32,
    /// Source stride between two rows.
    src_stride: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x428);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, irq_mask), 0x80);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, irq_source), 0x88);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, control), 0x400);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, start_transfer), 0x408);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, flags), 0x40C);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, dest_address), 0x410);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, x_length), 0x418);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, src_stride), 0x424);
