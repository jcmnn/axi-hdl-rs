[![Crates.io](https://img.shields.io/crates/v/axi-dmac)](https://crates.io/crates/axi-dmac)
[![docs.rs](https://img.shields.io/docsrs/axi-dmac)](https://docs.rs/axi-dmac)
[![ci](https://github.com/us-irs/axi-hdl-rs/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/us-irs/axi-hdl-rs/actions/workflows/ci.yml)

AXI DMAC driver
========

This is a native Rust driver for the Analog Devices
[AXI DMAC IP core](https://analogdevicesinc.github.io/hdl/library/axi_dmac/index.html), used for
example for the DMAs of the AXI AD9361 IP core. It is not the AMD AXI DMA IP core, see the
`axi-dma` crate for that one.

# Core features

- The capabilities of a core are probed once. The result is a driver for the device to memory
  direction or one for the memory to device direction, so the transfer functions only exist for
  the direction the core has.
- Transfers borrow their buffer until they are finished or dropped. Dropping a transfer stops the
  core.
- Cyclic transfers in hardware for the memory to device direction.
- Polled completion, the interrupts of the core stay masked.

# Features

- `defmt` implements `defmt::Format` for this crate's register and error types.
