//! Shared library for the `esp32-s31-spi-display-to-ethernet` firmware.
//!
//! The binary in `src/bin/main.rs` is a thin bootstrap that:
//!   * brings up the chip (clocks, PSRAM heap, esp-rtos scheduler on core 0),
//!   * boots core 1 to run the Ethernet transport, and
//!   * runs the SPI capture monitor (`monitor_task`) on core 0.
//!
//! All of the real logic lives in the modules below so it can be reused and
//! tested independently as the project grows.

#![no_std]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

extern crate alloc;

pub mod capture;
pub mod framebuffer;
pub mod monitor;

pub use capture::{
    CMD_CASET, CMD_MEMWR, CMD_RASET, Capture, DMA_ALIGN, FRAME_BYTES, FRAME_H, FRAME_W, HEAD_LEN,
    RX_SIZE, State,
};
pub use framebuffer::allocate_framebuffer;

/// A raw pointer to a completed framebuffer, wrapped so it is `Send`.
///
/// A bare `*const u8` is `!Send`, which would stop it crossing the core boundary
/// inside a `Signal`. This newtype is `Copy` (the pointer is just an integer) and
/// asserts `Send`, so the transport core can safely receive it. The pointed-to
/// memory is a PSRAM framebuffer that lives for the whole program, so the
/// pointer never dangles while it is valid.
#[derive(Clone, Copy)]
pub struct FramebufferPtr(pub *const u8);

// SAFETY: the pointer always refers to a PSRAM framebuffer that outlives the
// program, so sending it to another core is sound.
unsafe impl Send for FramebufferPtr {}

/// Cross-core handoff type: core 1 signals a completed [`FramebufferPtr`],
/// core 0 awaits it. `CriticalSectionRawMutex` makes the single-slot signal
/// core-atomic (latest-wins backpressure).
pub type TransportSignal = Signal<CriticalSectionRawMutex, FramebufferPtr>;
