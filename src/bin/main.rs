//! SPI-slave capture firmware for the ESP32-S31, split across two cores.
//!
//! * **Core 0 (ProCpu, pinned)** runs [`crate::monitor::monitor_task`]: the pure
//!   DMA re-arm loop plus the GC9A01 byte-stream parser. It never logs on the
//!   hot path and never blocks on the network, so capture gaps stay at 0.
//! * **Core 1 (AppCpu)** runs the transport: it brings up Gigabit Ethernet,
//!   awaits the completed framebuffer pointer handed over via
//!   [`TransportSignal`], and serves it over HTTP — `GET /frame` returns the
//!   live framebuffer as a JPEG, `GET /` a status page. See
//!   [`ethernet_task`] / [`web_task`] / [`serve_frame`].
//!
//! Decoupling the µs-fast DMA re-arm (core 0) from the frame-encoding network
//! transport (core 1) makes the `gaps=0` state durable; the Ethernet transport
//! also means capture no longer blocks on a slow serial dump.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::future::pending;
use core::sync::atomic::{AtomicUsize, Ordering};

extern crate alloc;

use embassy_executor::Spawner;
use embassy_net::{
    Config as NetConfig, Runner, Stack as NetStack, StackResources,
    tcp::{Error as TcpError, TcpSocket},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Duration;
use embedded_io_async::Write;
use esp_hal::{
    clock::CpuClock,
    dma::DmaRxBuf,
    psram::Psram,
    spi::{Mode, slave::Spi},
    system::Stack,
    timer::timg::TimerGroup,
};
use esp_println::logger::init_logger_from_env;
use esp_rtos::embassy::Executor;
use esp32_s31_coreboard::{ethernet, rgmii_pins};
use log::{info, warn};
use static_cell::StaticCell;

use esp32_s31_spi_display_to_ethernet::capture::{Capture, RX_SIZE};
use esp32_s31_spi_display_to_ethernet::framebuffer::allocate_framebuffer;
use esp32_s31_spi_display_to_ethernet::monitor::monitor_task;
use esp32_s31_spi_display_to_ethernet::{FRAME_BYTES, FRAME_H, FRAME_W, FramebufferPtr};

esp_bootloader_esp_idf::esp_app_desc!();

/// Size of the PSRAM, detected by core 0 and shared with core 1.
///
/// Core 1 needs this to map the `0x5000_0000` window into its own per-core
/// DCACHE (see [`map_psram_for_current_core`]). Core 0 fills it in right after
/// it brings up PSRAM, before it ever boots the second core.
static PSRAM_SIZE: AtomicUsize = AtomicUsize::new(0);

/// The S31 serves PSRAM through the SPI flash cache at this virtual origin.
const PSRAM_EXTMEM_ORIGIN: u32 = 0x5000_0000;

/// PSRAM MMU page size for the S31 (SOC: `mmu.page_size = 0x10000`).
///
/// This is the page granularity core 0's `oct_hex::mmu_map_psram` uses when it
/// maps the whole PSRAM heap at boot. Kept only as documentation: the shared
/// MMU is mapped once by core 0 and must NOT be reprogrammed from core 1.
#[allow(dead_code)]
const PSRAM_MMU_PAGE_SIZE: usize = 0x10000;

/// The cross-core handoff signal (core 0 signals a completed framebuffer,
/// core 1 awaits it). Latest-wins backpressure (see [`crate::monitor::monitor_task`]).
type TransportSignal = Signal<CriticalSectionRawMutex, FramebufferPtr>;

#[allow(clippy::large_stack_frames, reason = "larger locals are fine here")]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // Log over the USB-UART serial port (esp-println) instead of RTT/JTAG, so
    // output is visible on a plain serial monitor.
    init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialise PSRAM and expose it as the heap so the framebuffers live in
    // external memory (115200 bytes each would not fit in internal RAM).
    //
    // Default timing is used: pinning an explicit speed that the live MPLM does
    // not divide cleanly makes `init_psram` return false, leaving the heap at
    // size 0 and panicking the allocator on boot. We build the `Psram`
    // explicitly only so we can read back the *detected* size via
    // [`Psram::raw_parts`], which core 1 needs to map the same window.
    let psram = Psram::new(peripherals.PSRAM, Default::default());
    PSRAM_SIZE.store(psram.raw_parts().1, Ordering::Relaxed);
    esp_alloc::psram_allocator!(&psram);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    // Core 0 must start its scheduler first, before starting the second core.
    esp_rtos::start(timg0.timer0);

    // The transport tasks run on core 1, so core 0's own spawner is unused.
    let _ = spawner;

    // Keep the CPU awake while the SPI slave is receiving.
    let _wake_lock = esp_hal::rtc_cntl::WakeLock::new();

    info!(
        "[init] frame={} bytes, rx={}, two-core Ethernet transport",
        FRAME_BYTES, RX_SIZE
    );

    // --- SPI2 slave configuration ---
    // Wired to the master (esp32-c3-lcdkit) header J2 -> ESP32-S31 Function
    // board J2 "function part" (GPIOs 42-49, 35-39, 1-3). We use the clean,
    // general-purpose block GPIO42-45 (never used by flash/PSRAM/UART/boot on
    // this board), routed through the GPIO matrix:
    //   CS  = FSPICS0  (GPIO7  master) -> GPIO42
    //   SCK = FSPICLK  (GPIO1  master) -> GPIO43
    //   MOSI= FSPID    (GPIO0  master) -> GPIO44
    //   MISO/FSPIQ = GPIO55, left unused.
    let spi = Spi::new(peripherals.SPI2, Mode::_0)
        .with_sck(peripherals.GPIO43)
        .with_mosi(peripherals.GPIO44)
        .with_cs(peripherals.GPIO42)
        .with_dma(peripherals.DMA_AXI_CH0);

    // NOTE: the master's DC line (GPIO45) is intentionally NOT wired to a slave
    // input here. The parser in `capture::Capture` keys off the command *bytes*
    // (0x2a/0x2b/0x2c), not DC, so it is immune to DMA/CS misalignment and needs
    // no DC sample.

    // --- Framebuffers in PSRAM (double buffered) ---
    let fb0 = allocate_framebuffer(FRAME_BYTES);
    let fb1 = allocate_framebuffer(FRAME_BYTES);
    let capture = Capture::new(fb0, fb1);

    // Two RX DMA buffers in internal RAM, used as a ping-pong pair. We arm the
    // *next* buffer *before* running the state machine on the just-received
    // segment, so the gap between consecutive reads is a single re-arm
    // (microseconds) instead of a full process+re-arm round trip. See
    // `docs/dual-core-design.md` for why this gap is the critical path.
    let rx_bufs: [Option<DmaRxBuf>; 2] = [
        Some(esp_hal::dma_rx_buffer!(RX_SIZE).expect("RX DMA buffer 0")),
        Some(esp_hal::dma_rx_buffer!(RX_SIZE).expect("RX DMA buffer 1")),
    ];

    // Cross-core handoff: core 0 signals a completed framebuffer pointer, core 1
    // awaits it. Created via a StaticCell exactly like the
    // `embassy_multicore` example's `LED_CTRL` signal.
    static TRANSPORT: StaticCell<TransportSignal> = StaticCell::new();
    let transport = &*TRANSPORT.init(Signal::new());

    // Boot core 1 (the Ethernet transport). The thread-mode executor on core 1
    // needs the esp-rtos scheduler, which `start_second_core` pins to it; we
    // mirror the `embassy_multicore` example by building a StaticCell<Executor>
    // inside the closure. The whole embassy-net stack runs on this one core
    // (its shared state is a `RefCell`), so the async call graph stacks up:
    // `web_task` holds a `TcpSocket` future, `handle` buffers the request,
    // `serve_frame` drives the `jpeg-encoder` call chain. 32 KB leaves ample
    // headroom (the old 8 KB stack overflowed on the first `/frame` request).
    static CORE1_STACK: StaticCell<Stack<32768>> = StaticCell::new();
    let core1_stack = CORE1_STACK.init(Stack::new());

    // Move the Ethernet (core 1) peripherals out of `peripherals`. The
    // capture-side fields (SPI2, PSRAM, TIMG0 + their GPIOs) were moved above,
    // so we move only what core 1's transport needs. `Peripherals::take()` ran
    // once in `init`, so each field is moved individually (the whole struct is
    // only partially owned here).
    let rgmii = rgmii_pins!(peripherals);
    let eth = peripherals.ETH;
    let eth_reset = peripherals.GPIO7;

    esp_rtos::start_second_core(peripherals.CPU_CTRL, core1_stack, move || {
        // Map PSRAM into THIS core's per-core cache before touching any
        // framebuffer. The S31 serves PSRAM through the SPI flash cache at
        // `0x5000_0000`; the MEMSPI2 MMU that maps that window is shared
        // between the cores, but the DCACHE is per-core, so core 1 must
        // invalidate its own cache for the window or every store/load to a
        // PSRAM framebuffer faults with "Store/AMO access fault".
        //
        // We deliberately do NOT call `Psram::new` here: that re-runs the
        // full PSRAM controller bring-up (`reset_psram_mspi`, clock re-lock,
        // re-detect), which resets the controller core 0 already configured
        // and returns false — leaving our cache unmapped and disturbing
        // core 0. Instead we only program the shared MMU + invalidate our
        // own cache, using the size core 0 detected.
        // Program this core's per-core PMA/PMP so the 0x5000_0000 PSRAM
        // window is granted, BEFORE enabling the cache on it. esp_rtos
        // starts core 1 via start_second_core, which never runs the
        // bootloader's bootloader_init_mem() /
        // esp_cpu_configure_region_protection(), so the ROM default leaves
        // PMA#9 (PSRAM) disabled and the first cached store to a framebuffer
        // would fault with "Store/AMO access fault".
        //
        // configure_region_protection only *enables* the two PSRAM entries
        // (PMA#9 + PMP#10) and never resets the PMA table, so flash PMA#7
        // keeps the ICache served and the handler cannot double-fault.
        configure_region_protection();
        map_psram_for_current_core(PSRAM_SIZE.load(Ordering::Relaxed));
        info!("[core1] PSRAM cache mapped, starting Ethernet transport");

        static EX: StaticCell<Executor> = StaticCell::new();
        let executor = EX.init(Executor::new());
        executor.run(|spawner| {
            spawner.spawn(
                ethernet_task(spawner, transport, eth, eth_reset, rgmii)
                    .expect("spawn ethernet_task"),
            );
        });
    });

    // --- Core 0 capture monitor ---
    // Core 0 (its PSRAM DCACHE enabled by the bootloader) runs the DMA re-arm
    // + GC9A01 parser loop directly, so it can write the framebuffers. The
    // Ethernet transport lives on core 1, decoupled from capture. If core 0
    // produces frames faster than we can serve them, the Signal overwrites
    // (latest-wins) and older frames are dropped — the designed backpressure.
    info!("[monitor] core 0 spawning capture monitor_task");
    spawner.spawn(monitor_task(spi, rx_bufs, capture, transport).expect("spawn monitor_task"));
    // Idle forever, yielding to the executor so monitor_task runs on core 0.
    // Frames are served over Ethernet by core 1 via ethernet_task. (A `loop`
    // with no `break` diverges, satisfying the `-> !` return type; each
    // `pending().await` yields so the executor can poll monitor_task.)
    loop {
        pending::<()>().await;
    }
}

// ===========================================================================
// Core 1 transport: Gigabit Ethernet web server
// ===========================================================================

/// Number of socket slots: DHCP + DNS + a couple for the application.
const SOCKETS: usize = 6;

/// Drop a client that sends nothing for this long.
const CLIENT_TIMEOUT: Duration = Duration::from_secs(10);

/// L1 DCache partition selector for the shared data cache. This is the same
/// constant `esp_hal::soc` uses internally (`CACHE_MAP_L1_DCACHE = 1 << 4`);
/// the `soc` module is private, so we reference the value directly and reach
/// the ROM cache helpers via `unsafe extern "C"`, exactly as
/// `map_psram_for_current_core` does.
const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;

/// The static status page served on `GET /`. The `<img>` at `/frame` makes the
/// browser request the live framebuffer whenever it wants a fresh image.
const INDEX_HTML: &str = r#"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-S31 SPI interceptor — Ethernet</title>
<style>
  body { font-family: system-ui, sans-serif; background: #111; color: #eee;
         display: flex; flex-direction: column; align-items: center; margin: 2rem; }
  h1 { font-weight: 500; font-size: 1.4rem; }
  code { background: #222; padding: .1rem .3rem; border-radius: 4px; }
  #fb { width: min(90vw, 360px); height: auto; border: 3px solid #333;
        background: #000; image-rendering: pixelated; }
  .hint { color: #888; font-size: .9rem; margin-top: 1rem; max-width: 40rem;
          text-align: center; line-height: 1.5; }
</style>
</head>
<body>
<h1>ESP32-S31 SPI interceptor (Ethernet)</h1>
<img id="fb" alt="live framebuffer"
     src="/frame?_=document.lastModification">
<p class="hint">
  This page shows the live SPI-captured framebuffer streamed over Gigabit
  Ethernet. The browser re-requests <code>/frame</code> whenever it wants a
  fresh image (the master drives the frame rate, not us).
</p>
</body>
</html>"#;

/// Core 1 Ethernet bring-up + task bootstrap.
///
/// Runs on core 1 (inside the esp-rtos executor started in the
/// `start_second_core` closure). Brings up the YT8531 PHY, starts the
/// embassy-net DHCP stack, spawns the packet runner and the web server, then
/// idles. The whole stack runs on this one core: embassy-net's shared state is
/// a `RefCell`, so the runner and the socket code must not run on different
/// cores simultaneously.
#[embassy_executor::task]
#[allow(clippy::large_stack_frames, reason = "the ETH driver state is large")]
async fn ethernet_task(
    spawner: Spawner,
    transport: &'static TransportSignal,
    eth: esp_hal::peripherals::ETH<'static>,
    reset: esp_hal::peripherals::GPIO7<'static>,
    pins: ethernet::RgmiiPins<'static>,
) -> ! {
    info!("[ethernet] core 1 bringing up Gigabit Ethernet");
    let eth = ethernet::new(eth, reset, pins)
        .await
        .expect("Ethernet init (is the YT8531 PHY answering on MDIO?)");

    // Seed the stack from the per-chip MAC (unique per board; a fixed-per-boot
    // value is fine for DHCP). Generated here rather than on core 0 because
    // core 1 cannot `Peripherals::take()` the RNG that core 0 already owns.
    let mac = ethernet::mac_address();
    let seed = u64::from_le_bytes([mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], 0x31, 0x31]);

    static RESOURCES: StaticCell<StackResources<SOCKETS>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        eth,
        NetConfig::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );

    // The runner must be running for link/DHCP to progress.
    spawner.spawn(net_task(runner).expect("spawn net_task"));

    info!("[ethernet] waiting for cable link...");
    stack.wait_link_up().await;
    info!("[ethernet] waiting for DHCP...");
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        // `config.address` is an `Ipv4Cidr`; `.address` is the `Ipv4Addr`.
        info!(
            "[ethernet] ETH up: http://{}/ (gateway {:?}, DNS {:?})",
            config.address.address(),
            config.gateway,
            config.dns_servers
        );
    } else {
        warn!("[ethernet] DHCP did not hand out an address yet");
    }

    spawner.spawn(web_task(transport, stack).expect("spawn web_task"));

    // Idle forever, yielding so the spawned tasks run.
    loop {
        pending::<()>().await;
    }
}

/// embassy-net packet runner: feeds the driver and polls the stack.
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, ethernet::Eth<'static>>) -> ! {
    runner.run().await
}

/// Accepts connections on port 80 of `stack` and answers them, one at a time.
#[allow(
    clippy::large_stack_frames,
    reason = "socket buffers live in the static future"
)]
#[embassy_executor::task(pool_size = 2)]
async fn web_task(transport: &'static TransportSignal, stack: NetStack<'static>) -> ! {
    let mut rx = [0u8; 1024];
    let mut tx = [0u8; 2048];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
        socket.set_timeout(Some(CLIENT_TIMEOUT));
        if let Err(e) = socket.accept(80).await {
            warn!("accept failed: {e:?}");
            continue;
        }
        if let Err(e) = handle(&mut socket, transport).await {
            warn!("connection error: {e:?}");
        }
        // Send what is left, then close.
        socket.close();
        let _ = socket.flush().await;
    }
}

/// Reads one request and writes the response.
#[allow(clippy::large_stack_frames, reason = "1 KiB request header buffer")]
async fn handle(
    socket: &mut TcpSocket<'_>,
    transport: &'static TransportSignal,
) -> Result<(), TcpError> {
    let mut head = [0u8; 1024];
    let mut len = 0;
    while len < head.len() && !header_complete(&head[..len]) {
        match socket.read(&mut head[len..]).await? {
            0 => return Ok(()), // closed before a full request
            n => len += n,
        }
    }

    // Only serve real GET requests; anything else still gets a clean 404-ish
    // response so the browser does not hang.
    let req = parse_request(&head[..len]);
    let is_get = req.as_ref().map(|r| r.method == "GET").unwrap_or(false);
    if !is_get {
        return respond(socket, "405 Method Not Allowed", "text/plain", b"only GET").await;
    }

    // Route on the path (the part of the target before any `?`). `/` serves the
    // static page; `/frame` serves the live framebuffer (Phase 2).
    let path = req
        .map(|r| r.target.split('?').next().unwrap_or("/"))
        .unwrap_or("/");
    match path {
        "/" => {
            respond(
                socket,
                "200 OK",
                "text/html; charset=utf-8",
                INDEX_HTML.as_bytes(),
            )
            .await
        }
        "/frame" => serve_frame(socket, transport).await,
        _ => respond(socket, "404 Not Found", "text/plain", b"not found").await,
    }
}

/// Writes a response with the standard header (connection closes after it).
async fn respond(
    socket: &mut TcpSocket<'_>,
    status: &str,
    content_type: &str,
    body: &[u8],
) -> Result<(), TcpError> {
    let mut buf = [0u8; 192];
    let header = response_header(&mut buf, status, content_type, body.len())
        .expect("192 bytes hold this header");
    socket.write_all(header).await?;
    socket.write_all(body).await
}

/// Serves the most recent completed framebuffer as a baseline JPEG.
///
/// Waits for the latest frame handed over via `transport`, drops core 1's stale
/// cache lines over the window, expands each RGB565 pixel to 8-bit RGB into a
/// scratch PSRAM `Vec`, and encodes a baseline JPEG (quality 80). JPEG is used
/// instead of PPM because browsers render it natively while PPM does not.
#[allow(clippy::large_stack_frames, reason = "the jpeg-encoder state is large")]
async fn serve_frame(
    socket: &mut TcpSocket<'_>,
    transport: &'static TransportSignal,
) -> Result<(), TcpError> {
    // Wait for the most recent completed frame.
    let fb_ptr = transport.wait().await;

    // Reader half of the cross-core L1 DCache coherency handshake. Core 0
    // flushed the fresh pixels into PSRAM before signalling (see
    // `monitor_task`), but core 1 still holds its own *stale* copy of the
    // framebuffer cache lines. Drop them here so the read refills from PSRAM,
    // which now holds the real pixels. Serialized against core 0's flush by the
    // Signal handshake, so no SYNC_LOCK guard is needed.
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE, fb_ptr.0 as u32, FRAME_BYTES as u32);
    }

    // The framebuffer lives in PSRAM for the whole program, so this raw pointer
    // is valid. Reconstruct the slice.
    let fb = unsafe { core::slice::from_raw_parts(fb_ptr.0, FRAME_BYTES) };

    // The JPEG encoder wants 8-bit RGB, but the framebuffer is RGB565 (2
    // bytes/pixel). Expand each pixel to RGB888 into a scratch Vec. This is a
    // one-per-frame cost (the client requests `/frame` on demand), so a fresh
    // PSRAM-backed Vec is fine.
    let mut rgb = alloc::vec::Vec::with_capacity(FRAME_W * FRAME_H * 3);
    for chunk in fb.as_chunks::<2>().0 {
        let p = u16::from_le_bytes([chunk[0], chunk[1]]);
        let (r, g, b) = rgb565_to_rgb(p);
        rgb.push(r);
        rgb.push(g);
        rgb.push(b);
    }

    // Encode the framebuffer as a baseline JPEG into a fresh Vec. `encode`
    // consumes the encoder by value, releasing its borrow on `out`, so the
    // encoded bytes are readable afterwards. Quality 80 is a good
    // size/quality trade-off for a live view.
    let mut out: alloc::vec::Vec<u8> = alloc::vec::Vec::new();
    jpeg_encoder::Encoder::new(&mut out, 80)
        .encode(
            &rgb,
            FRAME_W as u16,
            FRAME_H as u16,
            jpeg_encoder::ColorType::Rgb,
        )
        .expect("JPEG encode");
    let len = out.len();

    // Write the HTTP response header, then the JPEG body.
    let mut hdr_buf = [0u8; 192];
    let header =
        response_header(&mut hdr_buf, "200 OK", "image/jpeg", len).expect("JPEG response header");
    socket.write_all(header).await?;
    socket.write_all(&out[..len]).await
}

/// Expand one RGB565 pixel (already little-endian, as clocked in from the
/// master) to 8-bit RGB.
#[inline]
fn rgb565_to_rgb(p: u16) -> (u8, u8, u8) {
    let r = (p & 0xF800) >> 11; // 5 bits
    let g = (p & 0x07E0) >> 5; // 6 bits
    let b = p & 0x001F; // 5 bits
    // Replicate the high bits to widen 5/6/5 -> 8/8/8.
    let r = ((r << 3) | (r >> 2)) as u8;
    let g = ((g << 2) | (g >> 4)) as u8;
    let b = ((b << 3) | (b >> 2)) as u8;
    (r, g, b)
}

// ---------------------------------------------------------------------------
// Minimal HTTP/1.1 request parsing (copied from the board crate's `http.rs`,
// which is pure-Rust and unit-tested on a PC). Only what this server needs.
// ---------------------------------------------------------------------------

/// The parts of a request line (`GET /frame?x=1 HTTP/1.1`).
struct Request<'a> {
    method: &'a str,
    target: &'a str,
}

/// Parses the request line at the start of `head`. `None` if incomplete/malformed.
fn parse_request(head: &[u8]) -> Option<Request<'_>> {
    let end = head.windows(2).position(|w| w == b"\r\n")?;
    let line = core::str::from_utf8(&head[..end]).ok()?;
    let mut parts = line.split(' ');
    let method = parts.next()?.trim();
    let target = parts.next()?.trim();
    if parts.next()?.trim_start().starts_with("HTTP/1.") {
        Some(Request { method, target })
    } else {
        None
    }
}

/// Whether `head` holds the complete request header (ends with a blank line).
fn header_complete(head: &[u8]) -> bool {
    head.windows(4).any(|w| w == b"\r\n\r\n")
}

/// Formats the response header into `buf` and returns the written bytes.
fn response_header<'a>(
    buf: &'a mut [u8],
    status: &str,
    content_type: &str,
    body_len: usize,
) -> Option<&'a [u8]> {
    struct Cursor<'b> {
        buf: &'b mut [u8],
        len: usize,
    }
    impl core::fmt::Write for Cursor<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let end = self.len + s.len();
            let dst = self.buf.get_mut(self.len..end).ok_or(core::fmt::Error)?;
            dst.copy_from_slice(s.as_bytes());
            self.len = end;
            Ok(())
        }
    }
    let mut c = Cursor { buf, len: 0 };
    let len = {
        // Scope the mutable borrow of `buf` so we can return a slice of it.
        match core::fmt::write(
            &mut c,
            format_args!(
                "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\n\
             Content-Length: {body_len}\r\nCache-Control: no-store\r\n\
             Connection: close\r\n\r\n"
            ),
        ) {
            Ok(()) => c.len,
            Err(_) => return None,
        }
    };
    Some(&buf[..len])
}

// ===========================================================================
// Core-1 region protection (PMA + PMP)
// ===========================================================================
//
// `esp_rtos::start_second_core` boots core 1 without the bootloader's
// `esp_cpu_configure_region_protection()`, so core 1 inherits the ROM default
// PMA/PMP in which the `0x5000_0000` PSRAM window (PMA#9) is disabled. A cached
// store to a framebuffer then faults with "Store/AMO access fault" (RISC-V
// exception 7). We enable only those two PSRAM entries and deliberately do NOT
// reset the whole PMA table: a full reset would transiently clear PMA#7 (flash,
// which serves the live ICache) and double-fault the exception handler. Every
// other entry is left exactly as the ROM set it.
//
//   * PMA#9  = `PMA_TOR | PMA_RWX` with limit `SOC_EXTRAM_HIGH` -> grants the
//     `[SOC_EXTRAM_LOW, SOC_EXTRAM_HIGH)` window and is cacheable, so the
//     DCACHE enabled by [`map_psram_for_current_core`] can serve it.
//   * PMP#10 = `PMP_NAPOT | PMP_RWX` over the same window -> the matching grant.

// RISC-V CSR addresses (components/riscv/include/riscv/csr.h).
// PMPCFG0 packs four PMP config bytes (entries 0..=3); the bank for entry E is
// PMPCFG0 + E/4.
const CSR_PMPCFG0: u32 = 0x3A0; // config byte region for PMP entries 0..=15
const CSR_PMPADDR0: u32 = 0x3B0; // pmpaddr0, then + entry index
const CSR_PMACFG0: u32 = 0xBC0; // pmacfg0, then + entry index (full 32-bit entry)
const CSR_PMAADDR0: u32 = 0xBD0; // pmaaddr0, then + entry index

// PMP field bits (privileged architecture spec).
const PMP_R: u32 = 0x01;
const PMP_W: u32 = 0x02;
const PMP_X: u32 = 0x04;
const PMP_L: u32 = 0x80;
const PMP_NAPOT: u32 = 0x18;
const PMP_SHIFT: u32 = 2;

// PMA field bits (components/riscv/include/riscv/csr.h).
const PMA_EN: u32 = 1 << 0;
const PMA_X: u32 = 1 << 2;
const PMA_W: u32 = 1 << 3;
const PMA_R: u32 = 1 << 4;
const PMA_L: u32 = 1 << 29;
const PMA_TOR: u32 = 0x4000_0000;
const PMA_SHIFT: u32 = 2;

// Bounds of the PSRAM window granted by PMA#9 / PMP#10 (soc.h).
const SOC_EXTRAM_LOW: u32 = 0x5000_0000;
const SOC_EXTRAM_HIGH: u32 = 0x5400_0000;

/// Encode a NAPOT region `[start, end)` exactly like esp-idf's
/// `PMPADDR_NAPOT(START, END)` macro: `START | ((END - START - 1) >> 1)`.
#[inline(always)]
const fn pmpaddr_napot(start: u32, end: u32) -> u32 {
    start | (((end - start) - 1) >> 1)
}

// --- Raw CSR access ----------------------------------------------------------
//
// A RISC-V `csrr`/`csrw` takes the CSR number as a 12-bit *immediate*, so the
// address must be a compile-time constant. Each primitive takes the entry index
// as a const parameter and passes the computed address to the assembler via the
// `addr = const (BASE + ENTRY)` operand form, which the inline-asm parser
// accepts. The `asm!` call needs its own `unsafe` block even though the wrapper
// is already `unsafe fn` (inline assembly is always unchecked).

/// Read PMPADDR[`ENTRY`] (`CSR_PMPADDR0 + ENTRY`).
#[inline(always)]
unsafe fn pmpaddr_read<const ENTRY: u32>() -> u32 {
    let mut v: u32 = 0;
    unsafe {
        core::arch::asm!(
            "csrr {v}, {addr}",
            v = out(reg) v,
            addr = const (CSR_PMPADDR0 + ENTRY),
            options(nostack)
        );
    }
    v
}

/// Write PMPADDR[`ENTRY`] (`CSR_PMPADDR0 + ENTRY`).
#[inline(always)]
unsafe fn pmpaddr_write<const ENTRY: u32>(val: u32) {
    unsafe {
        core::arch::asm!(
            "csrw {addr}, {v}",
            addr = const (CSR_PMPADDR0 + ENTRY),
            v = in(reg) val,
            options(nostack)
        );
    }
}

/// Read the PMP config byte region containing entry `ENTRY`
/// (`CSR_PMPCFG0 + ENTRY / 4`); the return value is the full 32-bit register.
#[inline(always)]
unsafe fn pmpcfg_read<const ENTRY: u32>() -> u32 {
    let mut v: u32 = 0;
    unsafe {
        core::arch::asm!(
            "csrr {v}, {addr}",
            v = out(reg) v,
            addr = const (CSR_PMPCFG0 + ENTRY / 4),
            options(nostack)
        );
    }
    v
}

/// Write the PMP config byte region containing entry `ENTRY`
/// (`CSR_PMPCFG0 + ENTRY / 4`).
#[inline(always)]
unsafe fn pmpcfg_write<const ENTRY: u32>(val: u32) {
    unsafe {
        core::arch::asm!(
            "csrw {addr}, {v}",
            addr = const (CSR_PMPCFG0 + ENTRY / 4),
            v = in(reg) val,
            options(nostack)
        );
    }
}

/// Write PMACFG[`ENTRY`] (`CSR_PMACFG0 + ENTRY`).
#[inline(always)]
unsafe fn pma_cfg_write<const ENTRY: u32>(val: u32) {
    unsafe {
        core::arch::asm!(
            "csrw {addr}, {v}",
            addr = const (CSR_PMACFG0 + ENTRY),
            v = in(reg) val,
            options(nostack)
        );
    }
}

/// Write PMAADDR[`ENTRY`] (`CSR_PMAADDR0 + ENTRY`).
#[inline(always)]
unsafe fn pma_addr_write<const ENTRY: u32>(val: u32) {
    unsafe {
        core::arch::asm!(
            "csrw {addr}, {v}",
            addr = const (CSR_PMAADDR0 + ENTRY),
            v = in(reg) val,
            options(nostack)
        );
    }
}

/// Read PMAADDR[`ENTRY`] (`CSR_PMAADDR0 + ENTRY`).
#[inline(always)]
unsafe fn pmaaddr_read<const ENTRY: u32>() -> u32 {
    let mut v: u32 = 0;
    unsafe {
        core::arch::asm!(
            "csrr {v}, {addr}",
            v = out(reg) v,
            addr = const (CSR_PMAADDR0 + ENTRY),
            options(nostack)
        );
    }
    v
}

/// Read PMACFG[`ENTRY`] (`CSR_PMACFG0 + ENTRY`).
#[inline(always)]
unsafe fn pmacfg_read<const ENTRY: u32>() -> u32 {
    let mut v: u32 = 0;
    unsafe {
        core::arch::asm!(
            "csrr {v}, {addr}",
            v = out(reg) v,
            addr = const (CSR_PMACFG0 + ENTRY),
            options(nostack)
        );
    }
    v
}

/// Grant the PSRAM window to core 1 (PMA#9 + PMP#10).
///
/// Called from the core-1 bootstrap before the cache is enabled and before any
/// PSRAM access, matching esp-idf's "region protection before cache" order.
pub fn configure_region_protection() {
    // Snapshot the ROM default so the log shows what we enable. PMA#9 is the
    // External-PSRAM entry; PMP#10 lives in PMPCFG0 bank 0x3A2 (byte index
    // 10 % 4 == 2).
    unsafe {
        info!(
            "[core1] regprot: ROM default  PMA9 addr=0x{:08x} cfg=0x{:08x} | PMP10 addr=0x{:08x} cfg=0x{:02x}",
            pmaaddr_read::<9>(),
            pmacfg_read::<9>(),
            pmpaddr_read::<10>(),
            (pmpcfg_read::<8>() >> ((10 % 4) * 8)) & 0xFF,
        );
    }

    // PMA#9: PSRAM TOR window `[.., SOC_EXTRAM_HIGH)`, cacheable RWX and locked.
    const PMA_RWX: u32 = PMA_L | PMA_EN | PMA_R | PMA_W | PMA_X;
    unsafe {
        pma_addr_write::<9>(SOC_EXTRAM_HIGH >> PMA_SHIFT);
        pma_cfg_write::<9>(PMA_TOR | PMA_RWX);
    }

    // PMP#10: PSRAM NAPOT window `[SOC_EXTRAM_LOW, SOC_EXTRAM_HIGH)`, RWX locked.
    unsafe {
        let napt = pmpaddr_napot(SOC_EXTRAM_LOW, SOC_EXTRAM_HIGH);
        pmpaddr_write::<10>(napt >> PMP_SHIFT);
        let mask = 0xFFu32 << ((10 % 4) * 8);
        let byte = PMP_NAPOT | PMP_L | PMP_R | PMP_W | PMP_X; // 0x9F
        pmpcfg_write::<10>((pmpcfg_read::<10>() & !mask) | (byte << ((10 % 4) * 8)));
    }

    // Confirm the grant took effect.
    unsafe {
        info!(
            "[core1] regprot: new state    PMA9 addr=0x{:08x} cfg=0x{:08x} | PMP10 addr=0x{:08x} cfg=0x{:02x}",
            pmaaddr_read::<9>(),
            pmacfg_read::<9>(),
            pmpaddr_read::<10>(),
            (pmpcfg_read::<8>() >> ((10 % 4) * 8)) & 0xFF,
        );
    }
}

/// Map the PSRAM window into the *current* core's cache without touching the
/// PSRAM controller.
///
/// This mirrors what `esp_hal`'s `oct_hex::mmu_map_psram` does at the end of
/// PSRAM bring-up, but only the parts that are safe to run a second time and on
/// a different core:
///
/// 1. Program the shared MEMSPI2 MMU entries (virtual page N -> physical page
///    N, PSRAM, valid). These registers are shared across both cores, so
///    re-writing them is harmless and makes this call self-contained rather
///    than relying on core 0 to have done it.
/// 2. Invalidate this core's DCACHE for the whole window, which is the actual
///    per-core operation that makes the `0x5000_0000` region cacheable. Without
///    it, CPU stores/loads to PSRAM fault with "Store/AMO access fault".
///
/// We never call `reset_psram_mspi` / clock / PHY bring-up here — that would
/// clobber the controller the other core already configured.
///
/// # Safety
///
/// Writes MMU and cache registers directly. Must only be called once, and only
/// while no other core is mid-cache-refill on the same window (i.e. at core-1
/// boot, before capture begins).
fn map_psram_for_current_core(size: usize) {
    unsafe extern "C" {
        /// ROM helper (S31): invalidate `size` bytes starting at `addr` in the
        /// given cache partition.
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
        /// ROM helper (S31): enable core-*N*'s L1 ICache (per-core).
        fn Cache_Enable_L1_CORE0_ICache(autoload: u32);
        fn Cache_Enable_L1_CORE1_ICache(autoload: u32);
        /// ROM helper (S31): enable the shared L1 DCache.
        fn Cache_Enable_L1_DCache(autoload: u32);
    }

    // CACHE_MAP_L1_DCACHE on the S31 is `1 << 4`.
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
    // cache_ll_l1_enable_icache/dcache autoload flag (CACHE_LL_CACHE_AUTOLOAD).
    const CACHE_LL_CACHE_AUTOLOAD: u32 = 1 << 0;

    // L1 cache control registers (DR_REG_CACHE_BASE). The SHUT bits gate whether
    // each bus is connected to the cache: a cleared SHUT bit means the bus is
    // cacheable. See `components/soc/esp32s31/register/soc/cache_reg.h`.
    const DR_REG_CACHE_BASE: u32 = 0x2C00_0000;
    const CACHE_L1_ICACHE_CTRL_REG: u32 = DR_REG_CACHE_BASE;
    const CACHE_L1_DCACHE_CTRL_REG: u32 = DR_REG_CACHE_BASE + 0x4;
    // SHUT_IBUS1 = bit1 of the ICache ctrl reg: core-1's instruction bus.
    const SHUT_IBUS1: u32 = 1 << 1;
    // SHUT_DBUS0 = bit0 of the DCache ctrl reg: the shared data bus.
    const SHUT_DBUS0: u32 = 1 << 0;

    // We deliberately do NOT reprogram the shared MEMSPI2 MMU here. On the S31
    // the PSRAM MMU is shared by both cores and is already mapped for the whole
    // heap by core 0 during `Psram::new` -> `oct_hex::mmu_map_psram` (which runs
    // before `start_second_core`). Rewriting it from core 1 while core 0 is
    // capturing corrupts core 0's framebuffer page mapping and yields the
    // `0x0addbad0` bus-default pattern. Core 1 only enables its own caches over
    // the already-valid shared mapping. See `docs/esp32s31-psram-cache-mmu.md`.

    // esp-rtos boots core 1 without esp-idf's `do_multicore_settings()`, so
    // core 1 starts with its L1 caches disabled and a store to the PSRAM window
    // faults with "Store/AMO access fault". We enable the caches in the same
    // order esp-idf uses: clear the bus SHUT bits first, then enable DCache
    // before ICache (see `docs/esp32s31-psram-cache-mmu.md` §3).
    //
    // (1) Unmask both buses.
    unsafe {
        let ic = core::ptr::read_volatile(CACHE_L1_ICACHE_CTRL_REG as *const u32);
        core::ptr::write_volatile(CACHE_L1_ICACHE_CTRL_REG as *mut u32, ic & !SHUT_IBUS1);
        let dc = core::ptr::read_volatile(CACHE_L1_DCACHE_CTRL_REG as *const u32);
        core::ptr::write_volatile(CACHE_L1_DCACHE_CTRL_REG as *mut u32, dc & !SHUT_DBUS0);
    }

    // (2) Invalidate any stale lines, then enable the cache: DCache first,
    // then ICache (matching cache_ll_enable_cache(CACHE_TYPE_ALL)).
    unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE, PSRAM_EXTMEM_ORIGIN, size as u32);
        Cache_Enable_L1_DCache(CACHE_LL_CACHE_AUTOLOAD);
        Cache_Enable_L1_CORE1_ICache(CACHE_LL_CACHE_AUTOLOAD);
        Cache_Enable_L1_CORE0_ICache(CACHE_LL_CACHE_AUTOLOAD);
    }

    // Confirmation (read-only): the shared MMU page-0 mapping core 0 set up
    // during `Psram::new` must still be valid after core 1 enables its caches.
    // `valid` + `access_spiram` set means the framebuffer window is mapped.
    unsafe {
        esp_hal::peripherals::MEMSPI2::regs()
            .mmu_item_index()
            .write(|w| w.mmu_item_index().bits(0));
        let c = esp_hal::peripherals::MEMSPI2::regs()
            .mmu_item_content()
            .read();
        log::debug!(
            "[core1] mmu page0: paddr=0x{:04x} access_spiram={} valid={} (shared mapping from core 0, left intact)",
            c.paddr().bits(),
            c.access_spiram().bit(),
            c.valid().bit(),
        );
    }

    // Diagnostics: read back the SHUT bits so the log confirms the enable
    // actually took effect on THIS core (a cleared SHUT bit == bus cacheable).
    unsafe {
        let ic = core::ptr::read_volatile(CACHE_L1_ICACHE_CTRL_REG as *const u32);
        let dc = core::ptr::read_volatile(CACHE_L1_DCACHE_CTRL_REG as *const u32);
        info!(
            "[core1] cache: ICTRL=0x{:08x} (IBUS1_shut={}) DCTRL=0x{:08x} (DBUS0_shut={})",
            ic,
            (ic & SHUT_IBUS1) != 0,
            dc,
            (dc & SHUT_DBUS0) != 0
        );
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("[panic] {}", info);
    loop {
        core::hint::spin_loop();
    }
}
