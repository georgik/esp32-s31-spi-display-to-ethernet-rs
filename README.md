# esp32-s31-spi-display-to-ethernet

Bare-metal Rust firmware for the **ESP32-S31** that listens on an SPI display
bus and re-emits the captured framebuffer over Ethernet.

The S31 runs as an **SPI slave listener** to a display master (an
ESP32-C3 LCD kit driving a GC9A01 panel). It reconstructs the 240x240 RGB565
framebuffer from the raw command/pixel byte stream and streams each completed
frame to a web browser as a live JPEG. There is no host dump to slow down the
capture: the frame rate is driven entirely by the master.

```
   ESP32-C3 LCD kit (master)        ESP32-S31 (slave, this firmware)
   ---------------------------------  ------------------------------------
   GC9A01 display  <---SPI2/Mode0----|  captures the display byte stream
   (GC9A01 240x240 RGB565)           |  rebuilds the framebuffer in PSRAM
                                     |
   Gigabit Ethernet (RGMII/YT8531)   |  serves each frame as a live JPEG
   browser  --->  HTTP GET /frame -->|  http://<dhcp-ip>/  (status page)
```

## Architecture

The firmware is a two-core design split.

- **Core 0 (ProCpu, pinned)** runs `monitor_task`: the pure DMA re-arm loop
  plus the GC9A01 byte-stream parser. It never logs on the hot path and never
  blocks on the network, so capture gaps stay at 0.
- **Core 1 (AppCpu)** runs the transport: it brings up Gigabit Ethernet, awaits
  the completed framebuffer pointer handed over via a cross-core `Signal`, and
  serves it over HTTP.

The split decouples the microsecond-fast DMA re-arm (core 0) from the
frame-encoding network transport (core 1). If core 0 produces frames faster than
core 1 can serve them, the signal overwrites (latest-wins backpressure) and
older frames are dropped — by design, not a bug.

### How it works

1. The master clocks a GC9A01 stream: `CASET (0x2a)` + 4 args, `RASET (0x2b)` +
   4 args, then `MEMWR (0x2c)` + 115200 pixels.
2. `Capture` (in `src/capture.rs`) reconstructs that stream purely from the
   command *bytes* on the wire. It does not sample the master's DC line, which
   is unreliable to sample at 40 MHz, so it is immune to DMA/CS boundary
   misalignment.
3. Pixels are collected into a double-buffered PSRAM framebuffer. On frame
   completion core 0 flushes and invalidates its L1 DCache for the window, then
   signals the framebuffer pointer to core 1.
4. Core 1 drops its stale cache lines, expands each RGB565 pixel to RGB888,
   encodes a baseline JPEG (quality 80), and sends it as the response to
   `GET /frame`.

## Hardware

- **Slave board:** ESP32-S31-Function-CoreBoard-1 (in-package PSRAM, Gigabit
  Ethernet via EMAC/RGMII to a Motorcomm YT8531 PHY on MDIO address 0).
- **Master:** an ESP32-C3 LCD kit driving a GC9A01 240x240 display as the SPI
  master (Mode 0, 40 MHz).
- **Network:** Gigabit Ethernet through a standard cable to your LAN/DHCP
  server.

### Wiring (slave side)

The S31 is a listener: it only receives and emits nothing on the bus, so the
original display may stay connected in parallel or be removed.

| Signal     | Master (C3 LCD kit) | Slave (S31) | Notes                          |
|------------|---------------------|-------------|--------------------------------|
| CS         | FSPICS0 (GPIO7)     | GPIO42      | clean GPIO block, GPIO matrix  |
| SCK        | FSPICLK (GPIO1)     | GPIO43      |                                |
| MOSI       | FSPID (GPIO0)       | GPIO44      |                                |
| MISO/FSPIQ | —                   | GPIO55      | 4-wire display, RX-only, unused|
| DC         | GPIO2               | (not wired) | parser keys off command bytes  |

The capture uses the clean general-purpose block GPIO42-45 (never used by
flash/PSRAM/UART/boot on this board), routed through the GPIO matrix. Common
ground between the two boards is required.

## Build and flash

### Requirements

- The ESP Rust toolchain (stable) with the `riscv32imafc-unknown-none-elf`
  target and `rust-src`.
- `espflash` (used to flash and to read the serial console).

### Build

```bash
cargo build --release
```

### Flash

```bash
cargo run --release
```

Flash over the USB-UART port with espflash (not a JTAG probe).

### Monitor

Open the serial console to see logs:

```bash
espflash monitor
```

## Using it

1. Connect the S31 to your LAN with a  Ethernet cable and flash the
   firmware.
2. Watch the serial console for the DHCP lease, e.g.
   `[ethernet] ETH up: http://192.168.1.42/ ...`.
3. Open that address in a browser. `/` shows a status page; the embedded image
   requests `/frame` on demand, so you see the live display.

## Feature flags

Flags are selected with `--features`. There are no default flags; each is opt-in.

| Feature      | Default | Effect                                                                        |
|--------------|---------|-------------------------------------------------------------------------------|
| `diag`       | no      | Non-blocking per-read capture summary on the rare heartbeat.                  |
| `diag-hex`   | no      | Heavy, blocking raw-hex dump of the first reads (off: it fragments capture).  |
| `capture-log`| no      | Verbose per-command `debug!` from the byte scanner.                           |

Keep `diag-hex` and `capture-log` off in production: blocking UART logging on
the capture core stalls the DMA re-arm and fragments the stream.

## Project layout

- `src/bin/main.rs` — firmware entry point: two-core bootstrap, the Gigabit
  Ethernet web server (`ethernet_task`, `web_task`, `serve_frame`), and the
  core-1 PSRAM region-protection (PMA/PMP) and cache bring-up.
- `src/capture.rs` — the GC9A01 byte-stream parser and double-buffered
  framebuffer storage.
- `src/monitor.rs` — the core-0 capture monitor (DMA re-arm loop + parser).
- `src/framebuffer.rs` — PSRAM framebuffer allocation.
- `src/lib.rs` — shared library and cross-core handoff types.

## Testing

The GC9A01 byte-stream parser is verified by a host-side reproduction tool
(`tools/parser_sync.rs`) that mirrors `capture::Capture::feed` verbatim and
feeds realistic streams at many offsets and chunk boundaries, checking that the
parser resynchronises on each frame:

```bash
rustc -O tools/parser_sync.rs -o /tmp/p && /tmp/p
```

## Dependencies

- `esp-hal` (with the `esp32s31` + `unstable` features) — the low-level driver.
  Released `esp-hal` lacks the RGMII EMAC driver this firmware needs, so the
  whole esp-rs stack is pinned to one exact git revision of the official
  `esp-rs/esp-hal` repository (see `[patch.crates-io]` in `Cargo.toml`).
- `esp-rtos` — the two-core scheduler and core-1 bootstrap.
- `embassy-net` — TCP stack + DHCP.
- `jpeg-encoder` — on-device baseline JPEG encoding (pure-Rust, `no_std`).
- `esp32-s31-coreboard` — the board-support crate (RGMII pin map, PHY reset,
  MAC derivation), pinned to an exact git revision of its upstream repository.

## References

Published documentation lives in `docs/`:

- `docs/wiring.md` — master/slave pinout and the DC-line note.
- `docs/esp32s31-psram-cache-mmu.md` — PSRAM cache/MMU bring-up and the
  cross-core coherence handshake.
- `docs/dual-core-design.md` — the two-core split.

