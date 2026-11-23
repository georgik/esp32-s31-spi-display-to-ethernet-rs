//! PSRAM framebuffer allocation.

use alloc::boxed::Box;
use alloc::vec;

use esp_hal::dma::aligned::DmaAlignedMut;

use crate::capture::DMA_ALIGN;

/// Allocate a 64-byte-aligned framebuffer of `size` bytes from the PSRAM heap.
///
/// Extra bytes let us align the start of the framebuffer. The allocation is
/// leaked (via [`Box::leak`]) and valid for the lifetime of the firmware, which
/// is what lets us hand a raw `*const u8` pointer to the other core.
pub fn allocate_framebuffer(size: usize) -> DmaAlignedMut<'static, [u8]> {
    let allocation: &'static mut [u8] =
        Box::leak(vec![0u8; size + DMA_ALIGN - 1].into_boxed_slice());

    let offset = allocation.as_ptr().align_offset(DMA_ALIGN);
    assert!(offset < DMA_ALIGN);

    let pixels = &mut allocation[offset..offset + size];
    assert_eq!(pixels.as_ptr() as usize % DMA_ALIGN, 0);
    assert_eq!(pixels.len() % DMA_ALIGN, 0);

    DmaAlignedMut::new(pixels).expect("Framebuffer is not DMA-aligned")
}
