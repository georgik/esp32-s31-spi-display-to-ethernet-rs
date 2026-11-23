# Dual-core design

The firmware is a two-core design. The split exists so the microsecond-fast
SPI capture hot path can **never** be stalled by the frame-encoding network
transport.

## Core assignment

| Core | Task | Pinned function | Responsibility |
|------|------|-----------------|----------------|
| 0 (ProCpu) | Capture monitor | `monitor_task` | DMA re-arm loop + GC9A01 byte-stream parser |
| 1 (AppCpu) | Transport | `ethernet_task` | Gigabit Ethernet bring-up + HTTP web server |

Core 0 is the *pinned* core: it boots first, brings up PSRAM, then boots core 1.
Everything core 0 does on the hot path is non-blocking and never touches the
network or UART.

## Why split the two cores

The capture hot path is:

```
DMA read completes  ->  re-arm the NEXT buffer  ->  feed bytes to the parser
```

The critical step is re-arming the next DMA buffer. If anything between a read
completing and the next `spi.read()` takes longer than a 512-byte read at the
40 MHz wire rate (a few microseconds), the master's following CS cycles are
missed — the stream desyncs into the `0x2a` + idle-zeros artifact.

If capture and transport shared a core, the transport's allocations, JPEG
encoding, and TCP writes would preempt the re-arm and fragment the stream.
Separating them makes the `gaps=0` state durable: core 0 re-arms in microseconds
and only ever yields when it hands a completed frame to core 1.

## Double buffering and ping-pong RX

### Framebuffers

Two PSRAM framebuffers (`FRAME_BYTES = 240*240*2 = 115200` bytes each) are
alternated by `Capture`. On `MEMWR` the parser swaps `current` and clears the new
buffer; on frame completion the just-finished frame stays put for core 1 to read
while core 0 starts the next frame in the other buffer.

### RX DMA buffers

Two internal-RAM RX DMA buffers form a ping-pong pair. Core 0 arms the **next**
buffer *before* it runs the state machine on the just-received segment, so the
gap between consecutive reads is a single re-arm (microseconds) instead of a
full process-then-re-arm round trip. This is the whole point of the dual-core
split on the capture side: the re-arm is the critical path, and it is kept as
short as possible.

## Cross-core handoff

Handoff is a single-slot `Signal<FramebufferPtr>` (`TransportSignal`), created
with a `StaticCell` exactly like the `embassy_multicore` example's `LED_CTRL`
signal.

- Core 0: after flushing the framebuffer to PSRAM, `transport.signal(...)`.
- Core 1: `transport.wait().await` to receive the pointer, then reads it.

The pointer is wrapped in a `FramebufferPtr` newtype so it is `Send` (a bare
`*const u8` is `!Send` and cannot cross the core boundary inside the signal).

## Backpressure: latest-wins

If core 0 produces frames faster than core 1 can serve them, the next
`signal()` **overwrites** the previous pending pointer. The older frame is
dropped — by design, not a bug. The transport core trades old frames for new
ones rather than blocking capture (which would let the DMA re-arm gaps grow and
fragment the stream). The `CriticalSectionRawMutex` makes the single-slot
signal core-atomic.

## See also

- `docs/esp32s31-psram-cache-mmu.md` — the cache/MMU bring-up and the coherency
  handshake that makes the handoff pointer valid.
- `docs/wiring.md` — the SPI wiring between master and slave.
