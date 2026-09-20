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
- Blocking API: `poll` checks for completion without blocking and `wait` blocks until the
  transfer is done.
- Asynchronous API, driven by the end of transfer interrupt of the core.
- Cyclic transfers in hardware for the memory to device direction.

# Features

If the asynchronous support is used, the number of statically provided wakers can be configured
using the following features:

- `1-waker`, which is the default
- `2-wakers`
- `4-wakers`
- `8-wakers`
- `16-wakers`
- `32-wakers`

The number of required wakers is the number of asynchronous drivers, so a design with the DMAC of
the ADC and the DAC of an AXI AD9361 core needs at least 2, which could be the `2-wakers`
feature.

Additionally:

- `portable-atomic` uses the [`portable-atomic`](https://docs.rs/portable-atomic) crate for the
  atomic operations, for targets without native atomics.
- `defmt` implements `defmt::Format` for this crate's register and error types.
