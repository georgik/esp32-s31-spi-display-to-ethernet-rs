//! GC9A01 byte-stream parser + double-buffered framebuffer storage.
//!
//! [`Capture`] owns two PSRAM framebuffers and reconstructs the master's
//! GC9A01 display stream purely from the *bytes* on the wire — it does not trust
//! the master's DC line (see [`State`]), which is unreliable to sample at 40 MHz.

use core::cmp::min;

use esp_hal::dma::aligned::DmaAlignedMut;
use log::{debug, info};

/// Display simulated by the master (GC9A01, 240x240, RGB565).
pub const FRAME_W: usize = 240;
pub const FRAME_H: usize = 240;
/// Full frame in bytes (240 * 240 * 2 RGB565).
pub const FRAME_BYTES: usize = FRAME_W * FRAME_H * 2;
/// DMA alignment required for DMA-capable buffers.
pub const DMA_ALIGN: usize = 64;

/// Size of the SPI slave RX DMA buffer (internal RAM).
///
/// The master batches pixels into 512-byte chunks (one per CS cycle). We ask for
/// more than one chunk per read so the DMA can absorb a whole burst of CS
/// cycles before the transfer completes, which reduces how often we have to
/// re-arm. See [`crate::monitor`] for why re-arm latency is the critical path.
pub const RX_SIZE: usize = 8192;

// --- GC9A01 display controller commands ---
pub const CMD_CASET: u8 = 0x2A; // Column Address Set
pub const CMD_RASET: u8 = 0x2B; // Row Address Set
pub const CMD_MEMWR: u8 = 0x2C; // Memory Write
// Note: the parser only tracks CASET/RASET/MEMWR (the per-frame sequence
// mipidsi's fill_contiguous emits). Gamma (0xE0/0xE1) and MAC (0x36) are sent
// once during display init, before the first frame, so we don't track them.

/// Size of the raw-stream head buffer kept for the full command-structure dump.
pub const HEAD_LEN: usize = 512;

/// Byte-stream parser state.
///
/// The parser no longer trusts the master's DC line (it is sampled *after* the
/// transfer completes, by which time the master has usually already advanced to
/// the next phase, so a single per-read sample is unreliable at 40 MHz).
/// Instead it scans the received byte stream for the well-known GC9A01 command
/// sequence emitted by mipidsi `fill_contiguous`:
///
/// ```text
/// 0x2a CASET + 4 args  ->  0x2b RASET + 4 args  ->  0x2c MEMWR + 115200px
/// ```
///
/// Because it keys off the *command bytes* rather than DC, it is immune to the
/// misalignment between DMA read boundaries and the master's SPI transactions.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum State {
    /// Scanning the byte stream for the next command byte (0x2a / 0x2b / 0x2c).
    SeekCmd,
    /// Collecting the argument bytes that follow a command.
    Args,
    /// Accumulating pixel bytes into the current framebuffer (after MEMWR).
    Collect,
}

/// State machine that reconstructs the GC9A01 stream received from the master.
///
/// The master toggles its DC line to distinguish commands (DC=LOW) from
/// args/pixels (DC=HIGH). Each SPI transaction on the slave corresponds to
/// exactly one master CS cycle, during which DC stays constant, so sampling
/// DC right after a transfer completes tells us what the transaction was.
pub struct Capture {
    /// Two PSRAM framebuffers, written alternately (double buffering).
    fbs: [DmaAlignedMut<'static, [u8]>; 2],
    /// Index of the framebuffer currently being written.
    current: usize,
    /// Number of fully captured frames.
    pub(crate) frames: usize,
    /// Number of commands seen.
    cmds: u32,
    state: State,
    /// Command whose arguments we are currently collecting (0x2a/0x2b).
    pending_cmd: Option<u8>,
    /// How many more argument bytes are expected for the current command.
    arg_remaining: usize,
    /// Accumulated argument bytes (max 16, e.g. gamma / window coords).
    arg_acc: [u8; 16],
    arg_acc_len: usize,
    /// How many pixel bytes have been collected for the current frame.
    pub(crate) collected: usize,
    /// Number of read cycles that completed with no bytes captured (bus idle /
    /// missed transactions). Used as a diagnostic to spot capture gaps.
    pub(crate) gaps: usize,
    /// Last 25% bucket logged, to avoid spamming progress lines.
    last_quarter: usize,
    /// Transactions seen since the last frame boundary (diagnostics).
    /// Reset on CASET so per-frame counters restart each frame.
    pub(crate) txns_since_frame: usize,
    /// How many command-like bytes (0x2b/0x2c) were seen while in SeekCmd
    /// but ignored because the parser was out of sync (diagnostics).
    missed_cmds: usize,
    /// Parsed window from CASET/RASET (for logging).
    x0: u16,
    x1: u16,
    y0: u16,
    y1: u16,
    /// --- Diagnostics (non-blocking receive forensics) ---------------------
    /// Head (first 16 B) of each received read, kept in a rotating 4-slot
    /// ring buffer so we can see whether reads land aligned to the master's
    /// 512-B CS cycles or are fragmented (core-1 falling behind). Never logs
    /// on its own — only dumped rarely via [`Capture::format_head_hist`].
    head_hist: [u8; 4 * 16],
    head_hist_n: usize,
}

impl Capture {
    pub fn new(fb0: DmaAlignedMut<'static, [u8]>, fb1: DmaAlignedMut<'static, [u8]>) -> Self {
        let mut this = Self {
            fbs: [fb0, fb1],
            current: 0,
            frames: 0,
            cmds: 0,
            state: State::SeekCmd,
            pending_cmd: None,
            arg_remaining: 0,
            arg_acc: [0u8; 16],
            arg_acc_len: 0,
            collected: 0,
            gaps: 0,
            last_quarter: 0,
            txns_since_frame: 0,
            missed_cmds: 0,
            x0: 0,
            x1: 0,
            y0: 0,
            y1: 0,
            head_hist: [0u8; 4 * 16],
            head_hist_n: 0,
        };
        // Seed both framebuffers with a known value, via the normal cacheable
        // PSRAM pointer.
        for fb in this.fbs.iter_mut() {
            fb.fill(0);
        }
        this
    }

    /// Feed a freshly received SPI segment into the byte-stream parser.
    ///
    /// Returns `Some(fb_ptr)` when a full frame was just collected, with a raw
    /// pointer to the completed framebuffer for the transport layer to dump.
    /// Returns `None` otherwise.
    pub fn feed(&mut self, data: &[u8]) -> Option<*const u8> {
        // --- Non-blocking receive forensics ---------------------------------
        // Record the head of this read into the rotating ring buffer. This is
        // just a few SRAM writes (no UART), so it adds negligible hot-path
        // overhead and lets us see the real read pattern when dumped in the
        // rare [beat] below.
        {
            let n = self.head_hist_n % 4;
            let k = min(data.len(), 16);
            self.head_hist[n * 16..n * 16 + k].copy_from_slice(&data[..k]);
            self.head_hist_n += 1;
        }

        let mut completed: Option<*const u8> = None;

        for &b in data {
            match self.state {
                State::Collect => {
                    // Collect pixel bytes into the active framebuffer until the
                    // full frame (FRAME_BYTES) has arrived. Written through the
                    // cacheable PSRAM pointer, the same pointer the transport
                    // reads from, so both cores agree on the same bytes.
                    if self.collected < FRAME_BYTES {
                        let dst = &mut *self.fbs[self.current];
                        dst[self.collected] = b;
                        self.collected += 1;

                        if self.collected == 2 {
                            // Log the very first pixel for content sanity.
                            // Gated behind `capture-log`: on the byte-scanner hot
                            // path, blocking on UART here would stall the re-arm.
                            #[cfg(feature = "capture-log")]
                            {
                                let c = u16::from_le_bytes([dst[0], dst[1]]);
                                debug!("[capture] first px = 0x{:04x}", c);
                            }
                        }

                        let quarter = (self.collected / (FRAME_BYTES / 4)).min(4);
                        if self.last_quarter != quarter {
                            self.last_quarter = quarter;
                            // Gated behind `capture-log`: hot-path, blocks the
                            // capture core on UART mid-capture.
                            #[cfg(feature = "capture-log")]
                            debug!(
                                "[capture] pixels {}% ({} / {}) gaps={}",
                                quarter * 25,
                                self.collected,
                                FRAME_BYTES,
                                self.gaps
                            );
                        }

                        if self.collected >= FRAME_BYTES {
                            completed = Some(self.on_frame_complete());
                        }
                    }
                }
                State::Args => {
                    // Accumulate argument bytes (they may span reads).
                    if self.arg_acc_len < self.arg_acc.len() {
                        self.arg_acc[self.arg_acc_len] = b;
                        self.arg_acc_len += 1;
                    }
                    self.arg_remaining -= 1;
                    if self.arg_remaining == 0 {
                        // Args complete: parse the window, then resume seeking.
                        match self.pending_cmd {
                            Some(CMD_CASET) if self.arg_acc_len >= 4 => {
                                self.x0 = u16::from_be_bytes([self.arg_acc[0], self.arg_acc[1]]);
                                self.x1 = u16::from_be_bytes([self.arg_acc[2], self.arg_acc[3]]);
                            }
                            Some(CMD_RASET) if self.arg_acc_len >= 4 => {
                                self.y0 = u16::from_be_bytes([self.arg_acc[0], self.arg_acc[1]]);
                                self.y1 = u16::from_be_bytes([self.arg_acc[2], self.arg_acc[3]]);
                            }
                            _ => {}
                        }
                        self.pending_cmd = None;
                        self.arg_acc_len = 0;
                        self.state = State::SeekCmd;
                    }
                }
                State::SeekCmd => {
                    match b {
                        CMD_CASET => {
                            self.cmds += 1;
                            self.pending_cmd = Some(b);
                            self.arg_remaining = 4;
                            self.arg_acc_len = 0;
                            self.state = State::Args;
                            // Hot-path: gated behind `capture-log` so a clean
                            // production capture never blocks the core on UART
                            // right after catching CASET.
                            #[cfg(feature = "capture-log")]
                            debug!("[framestart] cmd=CASET");
                        }
                        CMD_RASET => {
                            self.cmds += 1;
                            self.pending_cmd = Some(b);
                            self.arg_remaining = 4;
                            self.arg_acc_len = 0;
                            self.state = State::Args;
                            #[cfg(feature = "capture-log")]
                            debug!("[framestart] cmd=RASET");
                        }
                        CMD_MEMWR => {
                            // Start of a new frame: swap to the other buffer,
                            // clear it, and collect pixels into it. The swap here
                            // is what makes the two framebuffers alternate.
                            self.cmds += 1;
                            self.current ^= 1;
                            self.fbs[self.current].fill(0);
                            self.collected = 0;
                            self.last_quarter = 0;
                            self.gaps = 0;
                            self.state = State::Collect;
                            #[cfg(feature = "capture-log")]
                            debug!(
                                "[capture] MEMWR -> collect window {}x{} @({},{})",
                                self.x1.saturating_sub(self.x0),
                                self.y1.saturating_sub(self.y0),
                                self.x0,
                                self.y0
                            );
                        }
                        _ => {
                            // Unknown byte while seeking a command. If it looks
                            // like a command we care about, the parser is out of
                            // sync — log the first few so we can see why.
                            if (b == 0x2B || b == 0x2C) && self.missed_cmds < 4 {
                                self.missed_cmds += 1;
                                #[cfg(feature = "capture-log")]
                                debug!(
                                    "[framestart] MISSED 0x{:02x} while SeekCmd (out of sync)",
                                    b
                                );
                            }
                        }
                    }
                }
            }
        }

        completed
    }

    /// Called when a full frame (FRAME_BYTES pixels) has been collected.
    ///
    /// Runs the cheap, rare diagnostics (first-16-pixels + checksum), bumps the
    /// frame counter, and re-syncs the parser for the next frame. Returns a raw
    /// pointer to the completed framebuffer for the transport layer to read.
    ///
    /// **Important:** this deliberately does *not* swap `self.current`. The
    /// MEMWR handler swaps to the next buffer, so leaving `current` untouched
    /// here is what lets the just-completed frame stay in place for core 1 to
    /// read while core 0 begins the next frame in the *other* buffer.
    fn on_frame_complete(&mut self) -> *const u8 {
        // Raw pointer to the just-completed frame. The framebuffer lives in
        // PSRAM for the whole program, so this outlives the borrow below.
        let fb_ptr: *const u8 = self.fbs[self.current].as_ptr();

        // Rare diagnostics, kept off the per-transaction hot path (they run once
        // per frame, ~20 Hz, never mid-capture). Read back through the normal
        // cacheable PSRAM pointer — the same pointer core 0 wrote through, so the
        // two cores agree on the same physical bytes.
        let sum = {
            let fb: &[u8] = &self.fbs[self.current];

            let mut hexbuf = [0u8; 128];
            let hn = hex_to_buf(fb, &mut hexbuf);
            debug!(
                "[capture] frame #{} first16px: {}",
                self.frames,
                core::str::from_utf8(&hexbuf[..hn]).unwrap_or("")
            );

            checksum(fb)
        };

        self.frames += 1;
        debug!(
            "[capture] frame #{} done txns={} gaps={}",
            self.frames, self.txns_since_frame, self.gaps
        );
        self.txns_since_frame = 0;

        let w = self.x1.saturating_sub(self.x0) as usize;
        let h = self.y1.saturating_sub(self.y0) as usize;
        info!(
            "[capture] frame #{} complete checksum={:#010x} window={}x{} bytes={}",
            self.frames, sum, w, h, FRAME_BYTES
        );

        // Re-sync the parser for the next frame. `current` is intentionally left
        // alone so the completed frame stays visible to the transport core.
        self.state = State::SeekCmd;
        self.pending_cmd = None;
        self.arg_remaining = 0;
        self.collected = 0;
        self.missed_cmds = 0;

        fb_ptr
    }
}

/// Format the rotating head-history ring buffer into `buf` as a pipe-separated
/// list of hex bytes, one group per received read, e.g.
/// `"2a 00 00 00 00 .. | 2b 00 00 00 00 .. | ..."`. Returns the length written.
/// Groups are capped at 4 so the beat line stays short.
pub fn format_head_hist(cap: &Capture, buf: &mut [u8]) -> usize {
    const HEX: [u8; 16] = *b"0123456789abcdef";
    let mut i = 0usize;
    // At most 4 groups so the beat line stays short. Every write is bounded by
    // buf.len(): the S31 *panics* on any out-of-bounds slice write, and four
    // groups of 16 hex pairs (47 bytes each, plus '|' separators = 191 bytes)
    // can exceed a small buffer, so we must never assume the caller's buffer is
    // big enough.
    let groups = cap.head_hist_n.min(4);
    for r in 0..groups {
        if r > 0 && i < buf.len() {
            buf[i] = b'|';
            i += 1;
        }
        for j in 0..16 {
            // Two hex digits per byte: bail out if both slots are not free.
            if i + 1 >= buf.len() {
                return i;
            }
            let b = cap.head_hist[r * 16 + j];
            buf[i] = HEX[(b >> 4) as usize];
            i += 1;
            buf[i] = HEX[(b & 0x0f) as usize];
            i += 1;
            if j < 15 && i + 1 < buf.len() {
                buf[i] = b' ';
                i += 1;
            }
        }
    }
    i
}

/// Simple additive checksum over the framebuffer, used to detect frame changes.
pub fn checksum(data: &[u8]) -> u32 {
    let mut sum: u32 = 0;
    for &b in data {
        sum = sum.wrapping_add(b as u32);
    }
    sum
}

/// Format up to `n` bytes of `data` into `buf` as a space-separated, lowercase
/// hex string (e.g. `"11 22 33"`). Returns the number of characters written.
/// No heap / no formatting deps — just a stack buffer.
pub fn hex_to_buf(data: &[u8], buf: &mut [u8]) -> usize {
    const HEX: [u8; 16] = *b"0123456789abcdef";
    let n = min(data.len(), 32); // first 16 pixels == 32 bytes
    let mut i = 0usize;
    for (k, &b) in data.iter().take(n).enumerate() {
        if k > 0 {
            buf[i] = b' ';
            i += 1;
        }
        buf[i] = HEX[(b >> 4) as usize];
        buf[i + 1] = HEX[(b & 0x0f) as usize];
        i += 2;
    }
    i
}
