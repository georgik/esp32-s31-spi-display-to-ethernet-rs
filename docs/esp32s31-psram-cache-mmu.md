# ESP32-S31 PSRAM cache + MMU

The ESP32-S31 serves external PSRAM through the **SPI flash cache** at the
virtual origin `0x5000_0000`. The framebuffer lives in PSRAM (115200 bytes each
would not fit in internal RAM), so every store/load to it goes through this
window. Two independent mechanisms are involved:

1. The **MEMSPI2 MMU** maps virtual pages to physical PSRAM pages. It is
   **shared** between both cores.
2. The **L1 DCache** is **per-core**. Each core must enable and invalidate its
   own cache over the window before it can safely touch PSRAM.

Getting these right is what makes the two-core design work; getting them wrong
produces the `d0ba dd0a 0000 0000` column artifact and "Store/AMO access fault"
exceptions.

## The shared MMU (core 0 maps once)

`esp_hal` maps the whole PSRAM heap at the end of PSRAM bring-up via
`oct_hex::mmu_map_psram`: each MEMSPI2 MMU entry maps virtual page *N* to
physical page *N* in PSRAM, marked valid + `access_spiram`.

Core 0 does this during `Psram::new` → `oct_hex::mmu_map_psram`, which runs
**before** `start_second_core`. Because the MMU is shared, core 1 must **not**
reprogram it: rewriting it from core 1 while core 0 is mid-capture corrupts core
0's framebuffer page mapping and yields the `0x0addbad0` bus-default pattern.

Core 1 only reads the shared page-0 mapping (a read-only confirmation in
`map_psram_for_current_core`) and leaves it intact.

## The per-core DCache (core 1 must enable)

`esp_rtos::start_second_core` boots core 1 **without** esp-idf's
`do_multicore_settings()` / `bsp_start_cpu1()`, so core 1 starts with its L1
caches **disabled**. A cached store/load to the `0x5000_0000` window then faults
with "Store/AMO access fault" (RISC-V exception 7).

`map_psram_for_current_core` brings up core 1's cache in the same order
esp-idf uses:

1. **Unmask both buses.** Clear the SHUT bits in the L1 cache control registers
   (`CACHE_L1_ICACHE_CTRL_REG` bit `SHUT_IBUS1`, `CACHE_L1_DCACHE_CTRL_REG` bit
   `SHUT_DBUS0`). A cleared SHUT bit means the bus is cacheable.
2. **Invalidate** any stale lines over the window, then enable **DCache first,
   then ICache** (matching `cache_ll_enable_cache(CACHE_TYPE_ALL)`).

We deliberately do **not** call `Psram::new` from core 1: that re-runs the full
PSRAM controller bring-up (`reset_psram_mspi`, clock re-lock, re-detect), which
resets the controller core 0 already configured and returns `false`, leaving the
cache unmapped and disturbing core 0. Core 1 only programs the cache over the
already-valid shared mapping.

## Region protection (PMA#9 + PMP#10)

`start_second_core` also boots core 1 **without** the bootloader's
`esp_cpu_configure_region_protection()`, so core 1 inherits the ROM default
PMA/PMP in which the PSRAM window (**PMA#9**) is disabled. A cached store to a
framebuffer then faults with "Store/AMO access fault".

`configure_region_protection()` enables only those two PSRAM entries and
**deliberately does not reset the whole PMA table**: a full reset would
transiently clear PMA#7 (flash, which serves the live ICache) and double-fault
the exception handler. Every other entry is left exactly as the ROM set it.

| Entry | Type | Encoding | Effect |
|-------|------|----------|--------|
| PMA#9  | TOR  | `PMA_EN \| PMA_R \| PMA_W \| PMA_X \| PMA_L`, limit `SOC_EXTRAM_HIGH` | grants `[SOC_EXTRAM_LOW, SOC_EXTRAM_HIGH)`, cacheable |
| PMP#10 | NAPOT| `PMP_NAPOT \| PMP_R \| PMP_W \| PMP_X \| PMP_L` | the matching grant over the same window |

`SOC_EXTRAM_LOW = 0x5000_0000`, `SOC_EXTRAM_HIGH = 0x5400_0000` (a 64 MiB
window). The NAPOT address is encoded like esp-idf's
`PMPADDR_NAPOT(START, END)`: `START | ((END - START - 1) >> 1)`, then shifted
right by 2.

Region protection is programmed **before** the cache is enabled — esp-idf's
"protection before cache" order — so the handler cannot double-fault.

## Cross-core coherency handshake

The S31's per-core L1 DCache is **non-coherent**: the two cores do not
automatically keep their cache lines in sync over shared PSRAM. The firmware
performs an explicit handshake, serialized by the `transport` Signal (core 0
flushes before signalling; core 1 invalidates after receiving), so no lock is
needed.

| Phase | Core 0 (writer) | Core 1 (reader) |
|-------|-----------------|-----------------|
| Frame completes | writes pixels into PSRAM via cacheable pointer | — |
| Handoff | `signal(FramebufferPtr)` | `wait()` returns pointer |
| Release | `Cache_WriteBack_Addr` (flush dirty lines to PSRAM) | — |
| Release | `Cache_Invalidate_Addr` (drop its stale lines) | — |
| Consume | — | `Cache_Invalidate_Addr` (drop its stale lines, refill from PSRAM) |

Without the flush, the fresh pixels only ever sit in core 0's write-back cache
and never reach PSRAM, so core 1 reads its own stale copy — the `d0ba dd0a`
column artifact. Without the invalidates on both sides, each core keeps serving
its own stale line.

The cache partition selector for the shared data cache is
`CACHE_MAP_L1_DCACHE = 1 << 4`. The `esp_hal::soc` module is private, so the
firmware references the value directly and reaches the ROM cache helpers via
`unsafe extern "C"`, exactly as both `map_psram_for_current_core` and the
monitor's `cache_writeback_fb` / `cache_invalidate_fb` do.

### Register addresses

| CSR | Address | Purpose |
|-----|---------|---------|
| `CSR_PMPCFG0` | `0x3A0` | PMP config bytes (entries 0..=15, 4 per register) |
| `CSR_PMPADDR0` | `0x3B0` | PMPADDR entries |
| `CSR_PMACFG0` | `0xBC0` | PMA config (full 32-bit entries) |
| `CSR_PMAADDR0` | `0xBD0` | PMAADDR entries |
| `DR_REG_CACHE_BASE` | `0x2C00_0000` | L1 cache control registers |

The RISC-V `csrr`/`csrw` instruction takes the CSR number as a 12-bit immediate,
so the address must be a compile-time constant. The raw CSR primitives take the
entry index as a const parameter and pass the computed address via the
`addr = const (BASE + ENTRY)` inline-asm operand form.

## See also

- `docs/dual-core-design.md` — the two-core split and the handoff signal.
- `docs/wiring.md` — the SPI wiring.
