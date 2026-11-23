# Wiring (ESP32-S31 SPI slave)

The S31 runs as an **SPI slave listener** to a display master. It only receives
and emits nothing on the bus, so the original display may stay connected in
parallel or be removed.

## Boards

- **Slave:** ESP32-S31-Function-CoreBoard-1 — in-package PSRAM, Gigabit Ethernet
  via EMAC/RGMII to a Motorcomm YT8531 PHY on MDIO address 0.
- **Master:** an ESP32-C3 LCD kit driving a GC9A01 240x240 display as the SPI
  master (Mode 0, 40 MHz).

```
   ESP32-C3 LCD kit (master)        ESP32-S31 (slave, this firmware)
   ---------------------------------  ------------------------------------
   GC9A01 display  <---SPI2/Mode0----|  captures the display byte stream
   (GC9A01 240x240 RGB565)           |  rebuilds the framebuffer in PSRAM
                                     |
   Gigabit Ethernet (RGMII/YT8531)  |  serves each frame as a live JPEG
   browser  --->  HTTP GET /frame -->|  http://<dhcp-ip>/  (status page)
```

## Pinout (slave side)

The capture uses the clean general-purpose block **GPIO42-45** (never used by
flash/PSRAM/UART/boot on this board), routed through the GPIO matrix. Common
ground between the two boards is required.

| Signal    | Master (C3 LCD kit) | Slave (S31) | Notes                          |
|-----------|---------------------|-------------|--------------------------------|
| CS        | FSPICS0 (GPIO7)     | GPIO42      | clean GPIO block, GPIO matrix  |
| SCK       | FSPICLK (GPIO1)     | GPIO43      |                                |
| MOSI      | FSPID (GPIO0)       | GPIO44      |                                |
| MISO/FSPIQ| —                   | GPIO55      | 4-wire display, RX-only, unused|
| DC        | GPIO2               | *(not wired)*| parser keys off command bytes  |

The master's `MISO/FSPIQ` line (GPIO55) is left unused: the display is
RX-only from the slave's point of view.

## The DC-line note

The master toggles its **DC** (data/command) line to distinguish commands
(DC=LOW) from args/pixels (DC=HIGH). The original capture approach sampled DC
right after each transfer to classify the transaction.

**This firmware does not wire DC to a slave input, and the parser does not use
it.** Sampling DC is unreliable at 40 MHz: it is sampled *after* the transfer
completes, by which time the master has usually already advanced to the next
phase, so a single per-read sample is unreliable.

Instead [`Capture`](../src/capture.rs) keys off the **command bytes** on the
wire. It scans the received byte stream for the well-known GC9A01 command
sequence emitted by mipidsi's `fill_contiguous`:

```text
0x2a CASET + 4 args  ->  0x2b RASET + 4 args  ->  0x2c MEMWR + 115200px
```

Because it keys off the command bytes rather than DC, the parser is **immune to
the misalignment between DMA read boundaries and the master's SPI
transactions** — reads can start mid-frame, mid-command, or on a 512-byte CS
cycle boundary, and the parser still resynchronizes correctly on the next
`0x2a`/`0x2b`/`0x2c`.

Only CASET/RASET/MEMWR are tracked; gamma (0xE0/0xE1) and MAC (0x36) are sent
once during display init, before the first frame, so they are not tracked.

## See also

- `docs/dual-core-design.md` — the two-core split.
- `docs/esp32s31-psram-cache-mmu.md` — PSRAM cache/MMU bring-up.
