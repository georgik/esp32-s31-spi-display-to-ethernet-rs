//! Core 0 (ProCpu) capture monitor.
//!
//! [`monitor_task`] is the only thing that touches the SPI peripheral, the DMA
//! channels, and the framebuffers. It runs the ping-pong RX loop and the
//! [`crate::capture::Capture`] state machine. It never logs on the hot path and
//! never blocks on UART — the moment a full frame is collected it hands a
//! [`crate::FramebufferPtr`] to the transport core (core 1) via the `transport`
//! signal, then immediately starts the next frame in the other buffer.
//!
//! **Cross-core handoff:** core 0 writes the framebuffer through its per-core
//! L1 DCache and, on frame completion, flushes it to PSRAM then drops its cache
//! lines (see [`cache_writeback_fb`] / [`cache_invalidate_fb`]). Core 1 reads
//! the same PSRAM window through its own cache and drops those lines first
//! (see [`crate::serve_frame`]). The two halves of this handshake are
//! serialized by the `transport` Signal, so no lock is needed.

use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{Blocking, dma::DmaRxBuf, spi::slave::dma::SpiDma};

use crate::FramebufferPtr;
use crate::capture::{Capture, FRAME_BYTES, RX_SIZE};

/// The transport handoff signal (core 0 signals a completed framebuffer,
/// core 1 awaits it). Latest-wins backpressure (see [`crate::monitor`]).
type TransportSignal = Signal<CriticalSectionRawMutex, FramebufferPtr>;

/// L1 DCache partition selector for the shared data cache. This is the same
/// constant `esp_hal::soc` uses internally (`CACHE_MAP_L1_DCACHE = 1 << 4`);
/// the `soc` module is private, so we reference the value directly and reach
/// the ROM cache helpers via `unsafe extern "C"`, exactly as `main.rs` does for
/// `Cache_Invalidate_Addr` in `map_psram_for_current_core`.
const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;

/// Flush core-0's dirty PSRAM framebuffer cache lines to PSRAM (write-back).
///
/// The framebuffer is written by core 0 through its **non-coherent** per-core
/// L1 DCache. Without this the fresh pixels only ever sit in core 0's write-back
/// cache and never reach PSRAM, so core 1 reads its own stale copy (the
/// `d0ba dd0a` column artifact). This is the writer half of the cross-core
/// coherency handshake.
#[inline(always)]
unsafe fn cache_writeback_fb(addr: usize, size: usize) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_WriteBack_Addr(CACHE_MAP_L1_DCACHE, addr as u32, size as u32);
    }
}

/// Drop core-0's cached PSRAM framebuffer lines so the next read refills them
/// from PSRAM (which [`cache_writeback_fb`] above now holds the fresh pixels in).
#[inline(always)]
unsafe fn cache_invalidate_fb(addr: usize, size: usize) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE, addr as u32, size as u32);
    }
}

/// The fixed 8-byte block that shows up as the "purple column" noise:
/// `d0ba dd0a 0000 0000`. Half a 64-byte DCache line; used by the per-frame
/// RX-stream summary to decide whether the captured stream is corrupted.
const NOISE_BLOCK: [u8; 8] = [0xd0, 0xba, 0xdd, 0x0a, 0x00, 0x00, 0x00, 0x00];

/// Non-blocking capture diagnostic: record the head (first 16 bytes) of the
/// first few reads so we can see the frame structure at capture start
/// (aligned to the `0x2a` CASET header, or starting mid-frame in pixel data).
/// Logs exactly once. Kept tiny so it never disturbs the re-arm timing.
// Head (first bytes) of the first few reads, enough to see the command header
// and the pixel reads that follow MEMWR. Logged exactly once, off the hot path.
const DIAG_N_READS: usize = 24;
const DIAG_HEAD_W: usize = 16;

/// Lightweight per-read capture diagnostic (opt-in via the `diag` feature).
///
/// For each RX read we record a compact *summary* instead of the raw bytes:
/// the number of non-zero bytes in the scanned prefix, plus the byte offsets
/// and identity of any GC9A01 command bytes (`0x2a` CASET / `0x2b` RASET /
/// `0x2c` MEMWR) seen there. This is decisive for telling the two failure
/// modes apart:
///   * **Real capture**  -> reads starting at a frame boundary show the
///     command header (`0:2a 5:2b 10:2c`) and a high non-zero count; mid-frame
///     reads show many non-zero bytes and no commands.
///   * **Broken capture** (the `2a` + idle-zeros symptom) -> the first read
///     shows `0:2a` with a non-zero count of ~1 and no `0x2b`/`0x2c`, and the
///     following reads are all zeros (non-zero count 0, no commands).
///
/// We only scan the first `DIAG_SCAN` bytes of each read: the whole command
/// header (11 bytes) fits well inside it, and the "leading byte then zeros"
/// symptom is fully visible there, so the per-read cost stays sub-microsecond
/// and never perturbs the re-arm timing we are trying to measure.
#[cfg(feature = "diag")]
#[derive(Clone, Copy)]
struct ReadSummary {
    /// `number_of_received_bytes()` for the read.
    received: u16,
    /// Count of non-zero bytes within the scanned prefix.
    nz: u16,
    /// Byte offsets (into the read) of each command byte found, in order.
    offs: [u16; DIAG_CMD_MAX],
    /// The command bytes themselves, parallel to `offs`.
    bytes: [u8; DIAG_CMD_MAX],
    /// Number of valid entries stored in `offs`/`bytes`.
    ncmd: u8,
}

#[cfg(feature = "diag")]
const DIAG_SCAN: usize = 64; // bytes of each read scanned for the summary
#[cfg(feature = "diag")]
const DIAG_CMD_MAX: usize = 8; // max command bytes remembered per read
#[cfg(feature = "diag")]
const DIAG_RING: usize = 512; // summaries retained for the rolling dump

/// The capture monitor. Runs on core 1.
///
/// `spi`, `rx_bufs`, and `capture` are all owned by this task (moved in), which
/// is what lets the DMA `read()` take the driver by value each cycle.
#[task]
pub async fn monitor_task(
    mut spi: SpiDma<'static, Blocking>,
    mut rx_bufs: [Option<DmaRxBuf>; 2],
    mut capture: Capture,
    transport: &'static TransportSignal,
) {
    log::info!(
        "[monitor] enter on core={}",
        esp_hal::system::Cpu::current() as usize
    );

    // Index of the buffer currently armed/capturing.
    let mut captured_idx: usize = 0;

    // Per-frame RX-stream summary: tally how many 8-byte blocks are the fixed
    // `d0ba dd0a` noise, all-zero, or "real". Accumulated across the reads that
    // make up one frame, dumped and reset when the parser completes a frame.
    let mut rx_noise: u32 = 0;
    let mut rx_zero: u32 = 0;
    let mut rx_real: u32 = 0;
    let mut rx_sum: u32 = 0;

    // Continuous receive loop. Each SPI master transaction (one CS cycle)
    // completes one DMA read. We re-arm the alternate buffer before processing
    // so capture never stalls waiting on serial logging or the PSRAM copy.
    let mut transfer = loop {
        match spi.read(RX_SIZE, rx_bufs[0].take().expect("buf 0 present")) {
            Ok(t) => break t,
            Err((err, returned_spi, returned_buf)) => {
                spi = returned_spi;
                rx_bufs[0] = Some(returned_buf);
                log::warn!("[spi] initial read error: {:?}", err);
                Timer::after(Duration::from_millis(1)).await;
            }
        }
    };
    log::info!("[monitor] initial read armed");

    // Monotonic clock used to measure inter-transaction deltas for diagnostics.
    let mut last = Instant::now();

    // --- Non-blocking capture diagnostic (core 1 only) ----------------------
    // Head (first bytes) of each of the first few reads, plus bookkeeping, so we
    // can see whether capture starts aligned to the GC9A01 CASET header
    // (`0x2a`) or mid-frame inside pixel data. Logged exactly once, off the
    // critical re-arm path.
    let mut diag_heads: [u8; DIAG_N_READS * DIAG_HEAD_W] = [0u8; DIAG_N_READS * DIAG_HEAD_W];
    let mut diag_count: u32 = 0;
    let mut diag_logged = false;
    // One-time full-frame read-summary dump (first frame only, to avoid
    // desyncing the capture with a long UART burst). See [`dump_read_ring`].
    #[cfg(feature = "diag")]
    let mut frame_dumped = false;

    // Lightweight per-read capture-summary ring (opt-in via the `diag`
    // feature). See [`ReadSummary`] for what each entry records and why it is
    // decisive for the `2a` + idle-zeros diagnosis.
    #[cfg(feature = "diag")]
    let mut sum_ring: [ReadSummary; DIAG_RING] = core::array::from_fn(|_| ReadSummary {
        received: 0,
        nz: 0,
        offs: [0; DIAG_CMD_MAX],
        bytes: [0; DIAG_CMD_MAX],
        ncmd: 0,
    });
    #[cfg(feature = "diag")]
    let mut sum_head: usize = 0;

    loop {
        // The hot path must never block on UART: any logging between a read
        // completing and the next buffer being re-armed would stall the re-arm
        // longer than a 512-B read and drop the master's following reads.
        let (returned_spi, captured) = transfer.wait();
        spi = returned_spi;
        let prev_idx = captured_idx; // the buffer that just completed

        let received = captured.number_of_received_bytes().min(captured.capacity());

        // --- Non-blocking capture diagnostic --------------------------------
        // Record the head of the first few reads so we can see the frame
        // structure at capture start (aligned to the 0x2a CASET command, or
        // starting mid-frame in pixel data). Buffers only; logs exactly once,
        // off the critical re-arm path.
        {
            let mut count = diag_count;
            if received > 0 && (count as usize) < DIAG_N_READS {
                let k = core::cmp::min(received, DIAG_HEAD_W);
                let base = count as usize * DIAG_HEAD_W;
                diag_heads[base..base + k].copy_from_slice(&captured.as_slice()[..k]);
                count += 1;
                diag_count = count;
            }
            if !diag_logged && count as usize >= DIAG_N_READS {
                diag_log_first_reads(&diag_heads);
                diag_logged = true;
            }
        }

        // Re-arm the OTHER buffer FIRST (the critical, minimal-gap step).
        let other = 1 - prev_idx;
        loop {
            let next_buf = rx_bufs[other].take().expect("other buffer free");
            match spi.read(RX_SIZE, next_buf) {
                Ok(t) => {
                    transfer = t;
                    break;
                }
                Err((err, returned_spi, returned_buf)) => {
                    spi = returned_spi;
                    rx_bufs[other] = Some(returned_buf);
                    log::warn!("[spi] re-arm error: {:?}", err);
                    Timer::after(Duration::from_millis(1)).await;
                }
            }
        }
        captured_idx = other;

        // --- Diagnostics (kept minimal — off the critical path) ---
        capture.txns_since_frame += 1;
        let now = Instant::now();
        let dt_us = now.duration_since(last).as_micros();
        last = now;
        // Rare heartbeat (every 1024 transactions). This is the ONLY logging on
        // the hot path, and it is a single short line, so it must not stall the
        // loop enough to drop SPI transactions. A long beat line would block the
        // capture core on UART for several ms — at the 40 MHz wire rate that is
        // dozens of missed CS cycles, exactly the "0x2a then idle-zeros" desync
        // we are fighting — so it is kept short and emitted rarely.
        if capture.txns_since_frame & 0x3f == 0 {
            log::debug!(
                "[beat] txns={} gaps={} dt_last={}us received={} collected={} frames={}",
                capture.txns_since_frame,
                capture.gaps,
                dt_us,
                received,
                capture.collected,
                capture.frames
            );
            // Read-summary ring. DUMPED RARE (every 1024 txns) on purpose: it
            // blocks on UART, so emitting it on the per-read hot path would stall
            // the re-arm and fragment the capture. The one-time full-frame dump at
            // frame completion is the main tool.
            #[cfg(feature = "diag")]
            if capture.txns_since_frame & 0x3ff == 0 {
                // The 8 most-recent reads before this beat.
                dump_read_ring(&sum_ring, (sum_head + DIAG_RING - 8) % DIAG_RING, 8);
            }
        }

        if received == 0 {
            // A read cycle completed with no bytes captured — the master was
            // idle or we missed a transaction. Count it as a gap.
            capture.gaps += 1;
        } else {
            // --- Diagnostic: tally this read's RX-stream composition --------
            // Classify each 8-byte block of the received bytes as the fixed
            // `d0ba dd0a 0000 0000` noise, all-zero, or "real". Accumulated
            // per frame and dumped at completion (see above).
            // `as_chunks` is unavailable here: `received` is not a multiple of
            // 8, and the trailing bytes are tallied separately below.
            #[allow(clippy::chunks_exact_to_as_chunks)]
            for chunk in captured.as_slice()[..received].chunks_exact(8) {
                if chunk == NOISE_BLOCK {
                    rx_noise += 1;
                } else if chunk.iter().all(|&b| b == 0) {
                    rx_zero += 1;
                } else {
                    rx_real += 1;
                }
                rx_sum = rx_sum.wrapping_add(
                    chunk
                        .iter()
                        .map(|&b| b as u32)
                        .fold(0u32, u32::wrapping_add),
                );
            }
            // Any trailing <8 bytes count as "real" so nothing is mis-tallied.
            if received % 8 != 0 {
                rx_real += 1;
            }
            // --- HEAVY raw-hex forensics (opt-in via `diag-hex`, OFF by
            // default) ------------------------------------------------------
            // This blocks the capture core on UART long enough to fragment the
            // stream, so it is a diagnostic confound and stays OFF. The
            // non-blocking `ReadSummary` ring (plain `diag`) is the real tool.
            #[cfg(feature = "diag-hex")]
            {
                static DIAG_N: core::sync::atomic::AtomicU32 =
                    core::sync::atomic::AtomicU32::new(0);
                let _dn = DIAG_N.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if _dn < 48 {
                    let k = core::cmp::min(received, 24);
                    let raw = &captured.as_slice()[..received];
                    let mut s = [0u8; 24 * 3];
                    for (i, &b) in raw.iter().take(k).enumerate() {
                        s[i * 3] = b' ';
                        s[i * 3 + 1] = b'0' + (b >> 4);
                        s[i * 3 + 2] = b'0' + (b & 0x0f);
                    }
                    log::debug!(
                        "[diag] read#{} recv={} first[{}]={}",
                        _dn,
                        received,
                        k,
                        core::str::from_utf8(&s[..k * 3]).unwrap_or("")
                    );
                }
            }
            let data = &captured.as_slice()[..received];
            // --- Lightweight per-read capture-summary diagnostic (opt-in) -----
            // Record a compact summary of this read (non-zero count + command
            // byte offsets within the scanned prefix) into the rolling ring.
            // Pure integer work over a bounded prefix — no UART — so it adds
            // sub-microsecond cost and never perturbs the re-arm timing.
            #[cfg(feature = "diag")]
            {
                let raw = &captured.as_slice()[..received];
                let scan = core::cmp::min(received, DIAG_SCAN);
                let mut s = ReadSummary {
                    received: received as u16,
                    nz: 0,
                    offs: [0; DIAG_CMD_MAX],
                    bytes: [0; DIAG_CMD_MAX],
                    ncmd: 0,
                };
                for (i, &b) in raw.iter().take(scan).enumerate() {
                    if b != 0 {
                        s.nz += 1;
                    }
                    if (b == 0x2a || b == 0x2b || b == 0x2c) && (s.ncmd as usize) < DIAG_CMD_MAX {
                        s.offs[s.ncmd as usize] = i as u16;
                        s.bytes[s.ncmd as usize] = b;
                        s.ncmd += 1;
                    }
                }
                sum_ring[sum_head] = s;
                sum_head = (sum_head + 1) % DIAG_RING;
            }
            // Feed the raw bytes into the byte-stream parser. On frame
            // completion this hands the framebuffer pointer to the transport
            // core (core 1). The re-arm above already started the next capture,
            // so the transport dump overlaps with the next frame's capture.
            //
            // The framebuffer is written through its cacheable PSRAM pointer;
            // both cores touch the same window, so the completed frame is exactly
            // what core 0 captured.
            if let Some(fb_ptr) = capture.feed(data) {
                // --- Diagnostic: dump the accumulated RX-stream summary -----
                // rx_noise vs the framebuffer checksum localizes the artifact:
                //   * rx_noise high  -> corruption on the SPI/DMA RX path
                //   * rx_noise ~0     -> the framebuffer write/coherence path
                // Format the framebuffer address as a plain integer (the format
                // macro does not accept a raw pointer presentation here).
                let fb_addr = fb_ptr as usize;
                log::debug!(
                    "[rxsummary] frame={} rx_noise={} rx_zero={} rx_real={} rx_sum=0x{:08x} fb=0x{:x}",
                    capture.frames,
                    rx_noise,
                    rx_zero,
                    rx_real,
                    rx_sum,
                    fb_addr
                );
                rx_noise = 0;
                rx_zero = 0;
                rx_real = 0;
                rx_sum = 0;

                // --- Cross-core L1 DCache coherency (writer half) ---
                //
                // Core 1 reads the same PSRAM window through its own cache, so
                // core 0 must flush its dirty lines to PSRAM and drop its stale
                // cache lines so any subsequent read refills from PSRAM. Core 1
                // does the matching `Cache_Invalidate_Addr` in `serve_frame`
                // before it reads. The two are serialized by the Signal
                // handshake (core 0 flushes before signalling; core 1
                // invalidates after receiving), so no lock is needed.
                let fb_addr = fb_ptr as usize;
                unsafe {
                    cache_writeback_fb(fb_addr, FRAME_BYTES);
                    cache_invalidate_fb(fb_addr, FRAME_BYTES);
                }
                // Post-flush readback (diagnostic): re-read the first 16 px now
                // that the framebuffer is flushed+invalidated. If clean here while
                // the pre-flush value was `d0ba`, the cache flush fixed it.
                {
                    let fb = unsafe { core::slice::from_raw_parts(fb_ptr, FRAME_BYTES) };
                    // hex_to_buf emits 2 hex digits + a leading space per byte, so
                    // 32 input bytes need up to 95 bytes of output — use 128 like
                    // on_frame_complete does (a 32-byte buffer would panic).
                    let mut hb = [0u8; 128];
                    let hn = crate::capture::hex_to_buf(&fb[..32.min(fb.len())], &mut hb);
                    log::debug!(
                        "[coherency] post-flush first16px: {} fb=0x{:x}",
                        core::str::from_utf8(&hb[..hn]).unwrap_or(""),
                        fb_addr
                    );
                }

                // One-time full-frame read-summary dump (first frame only, to
                // avoid a long UART burst desyncing capture).
                #[cfg(feature = "diag")]
                if !frame_dumped {
                    frame_dumped = true;
                    log::info!("===== FULL-FRAME READ SUMMARY (first frame) =====");
                    // Ring holds reads 0..head for the first frame; dump the
                    // first 96 so the command header + pixel start are visible.
                    dump_read_ring(&sum_ring, 0, 96);
                }
                // Latest-wins backpressure: if core 1 is still dumping the
                // previous frame, this overwrites the signal and the older one
                // is dropped. That is the designed backpressure — the transport
                // core trades old frames for new ones rather than stalling
                // capture (which would let DMA re-arm gaps grow).
                transport.signal(FramebufferPtr(fb_ptr));
            }
        }

        // Return this buffer to its slot so it can be armed next cycle.
        rx_bufs[prev_idx] = Some(captured);
    }
}

/// Log the last [`DIAG_RING`] read summaries (opt-in via the `diag` feature).
///
/// Each line shows how many non-zero bytes the read held and where (at which
/// byte offset) any GC9A01 command bytes were found within the scanned prefix:
///   * a healthy capture shows the command header at the start of a frame
///     (`cmd=o0:C o5:R o10:M`) with a high non-zero count, then pixel-only
///     reads (`cmd=`) with still many non-zero bytes;
///   * the `2a` + idle-zeros symptom shows a lone `o0:C` with non-zero count
///     ~1, followed by reads that are all zeros (`recv=512 nz=1 cmd=o0:C` then
///     `recv=512 nz=0 cmd=`).
///
/// Runs only on the rare beat, so it never perturbs the re-arm timing.
#[cfg(feature = "diag")]
fn dump_read_ring(ring: &[ReadSummary; DIAG_RING], start: usize, max: usize) {
    for r in 0..max {
        let s = &ring[(start + r) % DIAG_RING];
        // Build a compact " o<off>:<mnemonic>" token list for the commands.
        let mut ctok = [0u8; DIAG_CMD_MAX * 6];
        let mut ci = 0usize;
        for c in 0..s.ncmd as usize {
            // " o<off>:<M>" needs at most 6 bytes.
            if ci + 6 > ctok.len() {
                break;
            }
            ctok[ci] = b' ';
            ctok[ci + 1] = b'o';
            // Decimal offset (up to 3 digits within our 64-byte prefix).
            let off = s.offs[c] as u32;
            let mut tmp = [0u8; 3];
            let mut k = 0usize;
            if off == 0 {
                tmp[k] = b'0';
                k += 1;
            }
            let mut v = off;
            while v > 0 && k < 3 {
                tmp[k] = b'0' + (v % 10) as u8;
                k += 1;
                v /= 10;
            }
            if ci + k <= ctok.len() {
                ctok[ci + 2..ci + 2 + k].copy_from_slice(&tmp[..k]);
                ci += 2 + k;
            }
            ctok[ci] = b':';
            ctok[ci + 1] = match s.bytes[c] {
                0x2a => b'C', // CASET
                0x2b => b'R', // RASET
                0x2c => b'M', // MEMWR
                _ => b'?',
            };
            ci += 2;
        }
        // Skip slots not yet written (received == 0) so the early boot phase
        // does not print spurious all-zero lines.
        if s.received == 0 {
            continue;
        }
        log::debug!(
            "[diag] recv={} nz={} cmd={}",
            s.received,
            s.nz,
            core::str::from_utf8(&ctok[..ci]).unwrap_or("")
        );
    }
}

/// Hex-encode the heads of the first few captured reads and log them exactly
/// once. Used to confirm whether capture starts aligned to the GC9A01 CASET
/// header (`0x2a`) or mid-frame inside pixel data. Runs once, off the hot path.
fn diag_log_first_reads(heads: &[u8]) {
    const HEX: [u8; 16] = *b"0123456789abcdef";
    let mut buf = [0u8; DIAG_N_READS * DIAG_HEAD_W * 2 + DIAG_N_READS];
    let mut i = 0usize;
    for r in 0..DIAG_N_READS {
        if r > 0 && i < buf.len() {
            buf[i] = b'|';
            i += 1;
        }
        for j in 0..DIAG_HEAD_W {
            if i + 1 >= buf.len() {
                break;
            }
            let b = heads[r * DIAG_HEAD_W + j];
            buf[i] = HEX[(b >> 4) as usize];
            i += 1;
            buf[i] = HEX[(b & 0x0f) as usize];
            i += 1;
        }
    }
    log::info!(
        "[diag] first{} reads head: {}",
        DIAG_N_READS,
        core::str::from_utf8(&buf[..i]).unwrap_or("")
    );
}
