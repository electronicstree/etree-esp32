// ============================================================================
//  etree-esp32.exe - runs ESP32 firmware in QEMU (lcgamboa fork, PICSimLab
//  build of libqemu-xtensa.dll) and keeps it in lockstep with a simulator
//  that talks to it through esp32link.h.
//
//  SPDX-License-Identifier: GPL-2.0-or-later
//
//  Usage (started by the simulator, not by hand):
//    etree-esp32.exe --link <tag> --parent <pid> --flash <merged.bin>
//                    [--slice-us 20] [--shift 2] [--wifi] [--wifi-port 8080]
//                    [--espnow 1234] [--mac 24:0A:C4:12:34:56]
//
//  Folder layout:
//    etree-esp32.exe + libglib-2.0-0.dll, libgcrypt-20.dll, libgpg-error-0.dll,
//    libintl-8.dll, libiconv-2.dll, libpcre2-8-0.dll, libwinpthread-1.dll, zlib1.dll
//    qemu\libqemu-xtensa.dll
//    qemu\fw\esp32-v3-rom.bin, qemu\fw\esp32-v3-rom-app.bin
//
//  Threads:
//    main thread : qemu_init + qemu_main_loop (must be the same thread).
//                  The lockstep timer runs here, holding the QEMU big lock,
//                  so while it waits for the simulator the guest is frozen.
//    vCPU thread : created by QEMU. GPIO callbacks arrive here.
// ============================================================================
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <algorithm>
#include <deque>

#include "esp32link.h"
#include "qemu_esp32_api.h"
#include "bt.h"

// ---------------------------------------------------------------------------
//  globals
// ---------------------------------------------------------------------------
static E32Shared*  g_sh = nullptr;
static HANDLE      g_grantEvt = nullptr;
static HANDLE      g_reachedEvt = nullptr;
static HANDLE      g_parent = nullptr;
static CRITICAL_SECTION g_pushLock;      // vCPU thread and main thread both push to toModel

static int64_t     g_sliceNs = 20000;
static QemuTimerStorage g_timer;
static short       g_pinmap[E32LINK_MAX_PINS];
static uint32_t*   g_uartBaud[3] = { nullptr, nullptr, nullptr };
static bool        g_enHeld = false;   // EN pin low: chip held in reset
struct PwmOut { bool on; uint32_t duty1000; uint32_t freq; };
static PwmOut      g_pwm[E32LINK_MAX_PINS];   // what each board pin outputs from the LEDC
struct BusPins { uint32_t b, c; };
static BusPins     g_busPins[6];              // last routing sent to the model, per bus
static bool        g_busPinsSent[6];
static int         g_rmtPin[8];               // board pin of RMT channel 0..7 output (0 = none)
static int         g_dac[2] = { -1, -1 };     // DAC1 / DAC2 value sent to the model, -1 = off
static volatile int8_t g_outEcho[E32LINK_MAX_PINS];   // output level to copy into the pin's input, -1 = none
static std::string g_flashCopy;
static bool        g_btOn = false;             // Bluetooth: controller API replaced in the firmware (bt.cpp)
static std::string g_efuseCopy;                  // per-part efuse file with the MAC (--mac), deleted at exit

// QEMU entry points
static qemu_init_fn                     q_init;
static qemu_main_loop_fn                q_main_loop;
static qemu_cleanup_fn                  q_cleanup;
static qemu_register_callbacks_fn       q_register_callbacks;
static qemu_set_pin_fn                  q_set_pin;
typedef void (*qemu_set_irq_fn)(void* irq, int level);
static qemu_set_irq_fn                  q_set_irq;
static qemu_set_apin_fn                 q_set_apin;
static qemu_uart_receive_fn             q_uart_receive;
static qemu_get_internals_fn            q_get_internals;
static timer_init_full_fn               q_timer_init_full;
static timer_mod_ns_fn                  q_timer_mod_ns;
static qemu_clock_get_ns_fn             q_clock_get_ns;
static vm_start_fn                      q_vm_start;
static void* (*q_get_cpu)(int) = nullptr;                    // qemu_get_cpu: CPUState* (XtensaCPU)
static void  (*q_runstall)(void* env, bool stall) = nullptr; // xtensa_runstall(CPUXtensaState*, bool)
static void  (*q_cpu_reset_interrupt)(void* cpu, int mask) = nullptr;
static qemu_system_reset_request_fn     q_reset_request;
static get_system_memory_fn             q_get_system_memory;
static memory_region_init_io_fn         q_mr_init_io;
static memory_region_add_subregion_overlap_fn q_mr_add_overlap;
static qemu_register_reset_fn           q_register_reset;

// ---------------------------------------------------------------------------
//  helpers
// ---------------------------------------------------------------------------
static void elog(const char* fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    fprintf(stderr, "[etree-esp32] %s\n", buf);
    fflush(stderr);
}

static void fail(const char* fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    fprintf(stderr, "[etree-esp32] ERROR: %s\n", buf);
    fflush(stderr);
    if (g_sh)
    {
        strncpy_s(g_sh->errorText, sizeof(g_sh->errorText), buf, _TRUNCATE);
        InterlockedExchange(&g_sh->engineState, E32_STATE_FAILED);
        if (g_reachedEvt) SetEvent(g_reachedEvt);
    }
    if (!g_flashCopy.empty()) DeleteFileA(g_flashCopy.c_str());
    if (!g_efuseCopy.empty()) DeleteFileA(g_efuseCopy.c_str());
    ExitProcess(2);
}

static std::string exeDir()
{
    char p[MAX_PATH];
    DWORD n = GetModuleFileNameA(nullptr, p, MAX_PATH);
    std::string s(p, n);
    size_t k = s.find_last_of("\\/");
    return k == std::string::npos ? std::string(".") : s.substr(0, k);
}

static int64_t now_ns()
{
    return q_clock_get_ns(QEMU_CLOCK_VIRTUAL_);
}

// Push an event to the model. Waits if the ring is full (the model drains
// the ring while it waits for us, so this can't dead-lock).
// toModelAt: the event happens at engine time ns (MCPWM edges computed for a past moment).
static void toModelAt(int64_t ns, uint16_t type, uint16_t a, uint32_t b, uint32_t c = 0, uint32_t res = 0)
{
    E32Event e;
    memset(&e, 0, sizeof(e));
    e.ns = ns;
    e.type = type;
    e.a = a;
    e.b = b;
    e.c = c;
    e.reserved = res;
    EnterCriticalSection(&g_pushLock);
    while (!e32_push(&g_sh->toModel, e))
    {
        if (g_sh->quit) break;
        SetEvent(g_reachedEvt);   // wake the model so it drains
        Sleep(0);
    }
    LeaveCriticalSection(&g_pushLock);
}

static void toModel(uint16_t type, uint16_t a, uint32_t b, uint32_t c = 0)
{
    toModelAt(now_ns(), type, a, b, c);
}

static void quitNow()
{
    InterlockedExchange(&g_sh->engineState, E32_STATE_EXITED);
    if (!g_flashCopy.empty()) DeleteFileA(g_flashCopy.c_str());
    if (!g_efuseCopy.empty()) DeleteFileA(g_efuseCopy.c_str());
    ExitProcess(0);   // QEMU state is thrown away on purpose: nothing to save
}

// ---------------------------------------------------------------------------
//  callbacks from QEMU
// ---------------------------------------------------------------------------
static void spiLog(const char* item);

static void cb_write_pin(int pin, int value)
{
    if (pin <= 0 || pin >= E32LINK_MAX_PINS) return;
    toModel(E32_EV_PIN_OUT, (uint16_t)pin, value ? 1u : 0u);
    // the pad's input buffer sees what the pin drives: digitalRead() of an output pin returns
    // its level, as on the real chip (the model sends no input level for output pins).
    // Set at the next slice: qemu_picsimlab_set_pin can't be called inside this register write.
    g_outEcho[pin] = (int8_t)(value ? 1 : 0);
    char item[16];
    sprintf_s(item, " [p%d=%d]", pin, value ? 1 : 0);
    spiLog(item);
}

static void updatePwm();
static void updateRouting();
static void updateDac();

static void pcntRoute();
static void mcRoute();
static void mcRun(int u, int64_t to);
static void spisRoute();
static void cb_dir_pin(int pin, int isOutput)
{
    if (pin == -1) { updatePwm(); updateRouting(); pcntRoute(); mcRoute(); spisRoute(); return; }   // sync: GPIO matrix / IO MUX / LEDC changed
    if (pin <= 0 || pin >= E32LINK_MAX_PINS) return;
    toModel(E32_EV_PIN_DIR, (uint16_t)pin, isOutput ? 1u : 0u);
}

static void cb_rmt_event(const uint8_t ch, const uint32_t conf0, const uint32_t item);   // routing section
static void rmtRxReset();
static void pcntReset();
static void mcpwmReset();
static void i2csReset();
static void spisReset();
static void canReset();
static void i2sReset();
static void i2sRun(int p, int64_t to);
static void updateI2s();

// ---------------------------------------------------------------------------
//  Real UART baud rate.
//  QEMU computes baud = 80 MHz * 16 / CLKDIV, as if the APB bus always ran
//  from the PLL. The ROM (and anything before the clock switch) runs the APB
//  from the 40 MHz crystal, so the real rate is half of QEMU's number: the ROM
//  boot text is really 115200, QEMU says 230400. We read the clock source
//  from the RTC_CNTL state and correct it.
//  Offsets are from the DWARF info of this exact DLL build (QEMU 9.2.2,
//  PICSimLab 0.9.3). They are checked once at start-up; if they look wrong
//  we fall back to QEMU's number.
// ---------------------------------------------------------------------------
static const uint8_t* g_soc = nullptr;           // Esp32SocState*
enum
{
    OFS_STRAP_MODE    = 0xb2d0,   // what get_internals(0) points to
    OFS_RTC_XTAL_APB  = 0xc2fc,   // rtc_cntl.xtal_apb_freq
    OFS_RTC_PLL_APB   = 0xc300,   // rtc_cntl.pll_apb_freq
    OFS_RTC_SOC_CLK   = 0xc304,   // rtc_cntl.soc_clk: 0 XTAL, 1 PLL, 2 8M, 3 APLL
    OFS_UART0         = 0x9c80,   // uart[0]
    UART_STRIDE       = 0x600,
    OFS_UART_CONF0    = 0x53c     // bit 27 TICK_REF_ALWAYS_ON: 1 = APB clock, 0 = REF_TICK
};

static void initClockProbe()
{
    const uint8_t* soc = (const uint8_t*)q_get_internals(QI_GPIO_STRAP_MODE) - OFS_STRAP_MODE;
    uint32_t xtal = *(const uint32_t*)(soc + OFS_RTC_XTAL_APB);
    uint32_t pll = *(const uint32_t*)(soc + OFS_RTC_PLL_APB);
    const uint32_t* b0 = (const uint32_t*)(soc + OFS_UART0 + 0x4e4);
    if (xtal >= 10000000 && xtal <= 50000000 && pll >= 40000000 && pll <= 160000000 && b0 == g_uartBaud[0])
    {
        g_soc = soc;
        elog("clock probe ok: XTAL APB %u Hz, PLL APB %u Hz", xtal, pll);
    }
    else
        elog("WARNING: clock probe failed (xtal %u, pll %u) - UART baud may be wrong during boot", xtal, pll);
}

// Snap to a standard rate if one is within 3 %.
static uint32_t nearestStd(uint32_t b, double* relErr)
{
    static const uint32_t STD[] = { 300, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600,
        74880, 76800, 115200, 128000, 230400, 250000, 256000, 460800, 500000, 921600, 1000000,
        1500000, 2000000 };
    uint32_t best = b;
    double bestErr = 1e9;
    for (uint32_t s : STD)
    {
        double e = (double)b / (double)s - 1.0;
        if (e < 0) e = -e;
        if (e < bestErr) { bestErr = e; best = s; }
    }
    *relErr = bestErr;
    return best;
}

static uint32_t realBaud(int uart)
{
    uint32_t q = g_uartBaud[uart] ? *g_uartBaud[uart] : 115200;
    if (q == 0) q = 115200;
    uint32_t out = q;
    uint32_t conf0 = 0, sel = 1;
    if (g_soc)
    {
        conf0 = *(const uint32_t*)(g_soc + OFS_UART0 + uart * UART_STRIDE + OFS_UART_CONF0);
        sel = *(const uint32_t*)(g_soc + OFS_RTC_SOC_CLK);
        if (conf0 & (1u << 27))                       // APB clock (not REF_TICK)
        {
            if (sel == 1)                             // PLL: APB = pll_apb_freq
            {
                uint32_t pll = *(const uint32_t*)(g_soc + OFS_RTC_PLL_APB);
                out = (uint32_t)((uint64_t)q * pll / 80000000u);
            }
            else                                      // crystal clock
            {
                // The ROM inside QEMU believes the crystal is 26 MHz, a real
                // board detects 40 MHz. Try both and keep the one that gives a
                // standard baud rate.
                uint32_t xtal = *(const uint32_t*)(g_soc + OFS_RTC_XTAL_APB);
                uint32_t cand[2] = { xtal, 26000000u };
                double bestErr = 1e9;
                for (int k = 0; k < 2; k++)
                {
                    uint32_t b = (uint32_t)((uint64_t)q * cand[k] / 80000000u);
                    double err;
                    uint32_t snapped = nearestStd(b, &err);
                    if (err < bestErr) { bestErr = err; out = (err < 0.03) ? snapped : b; }
                }
            }
        }
    }
    if (out == 0) out = q;

    static uint32_t lastQ[3], lastOut[3];
    if (q != lastQ[uart] || out != lastOut[uart])
    {
        lastQ[uart] = q;
        lastOut[uart] = out;
        elog("UART%d baud: qemu %u, clock %s, %s -> %u", uart, q,
             sel == 1 ? "PLL" : (sel == 0 ? "XTAL" : "other"),
             (conf0 & (1u << 27)) ? "APB" : "REF_TICK", out);
    }
    return out;
}

static int uartTxPin(int uart);   // routing section

static void cb_uart_tx(uint8_t uart, uint8_t byte)
{
    if (uart > 2) return;
    E32Event e;
    memset(&e, 0, sizeof(e));
    e.ns = now_ns();
    e.type = E32_EV_UART_TX;
    e.a = uart;
    e.b = byte;
    e.c = realBaud(uart);
    e.reserved = (uint32_t)uartTxPin(uart);      // 0 = UART0 default (TX0)
    EnterCriticalSection(&g_pushLock);
    while (!e32_push(&g_sh->toModel, e))
    {
        if (g_sh->quit) break;
        SetEvent(g_reachedEvt);
        Sleep(0);
    }
    LeaveCriticalSection(&g_pushLock);
}


// ===========================================================================
//  Pins: pull-up / pull-down, RTC IO registers, reset
// ===========================================================================
//  Where the ESP32 keeps the pull resistors:
//   - digital pads : IO_MUX register of the pad, FUN_WPU bit 8, FUN_WPD bit 7.
//                    QEMU stores these (iomux.muxgpios[gpio]).
//   - RTC pads     : (GPIO 0 2 4 12-15 25-27 32-39) ESP-IDF sets the pulls of
//                    these pads in the RTC IO block. QEMU does not emulate that
//                    block ("unimplemented device"), so the engine maps its
//                    own small register block over it and reads RUE/RDE there.
// ---------------------------------------------------------------------------
static const uint32_t* g_iomux = nullptr;          // muxgpios[40], by GPIO number
static uint32_t        g_rtcio[0x400 / 4];          // RTC IO registers (0x3FF48400)
static uint8_t         g_lastPull[E32LINK_MAX_PINS];
static QemuMemoryRegionOps     g_rtcioOps;
static QemuMemoryRegionStorage g_rtcioMr[2];
static bool            g_heldAnnounced = false;

struct RtcPad { int gpio; uint32_t reg; int rue; int rde; };
static const RtcPad RTC_PADS[] = {
    {  4, 0x94, 27, 28 },   // TOUCH_PAD0
    {  0, 0x98, 27, 28 },   // TOUCH_PAD1
    {  2, 0x9C, 27, 28 },   // TOUCH_PAD2
    { 15, 0xA0, 27, 28 },   // TOUCH_PAD3
    { 13, 0xA4, 27, 28 },   // TOUCH_PAD4
    { 12, 0xA8, 27, 28 },   // TOUCH_PAD5
    { 14, 0xAC, 27, 28 },   // TOUCH_PAD6
    { 27, 0xB0, 27, 28 },   // TOUCH_PAD7
    { 25, 0x84, 27, 28 },   // PAD_DAC1
    { 26, 0x88, 27, 28 },   // PAD_DAC2
    { 32, 0x8C, 22, 23 },   // XTAL_32K_PAD, X32P (GPIO32: RUE bit 22, RDE bit 23)
    { 33, 0x8C, 27, 28 },   // XTAL_32K_PAD, X32N (GPIO33: RUE bit 27, RDE bit 28)
};

// Power-on pulls of the strapping pins (datasheet): GPIO0 and GPIO15 up,
// GPIO2, GPIO4 and GPIO12 down. GPIO5 (also up) is a digital pad: not modelled.
static void rtcioResetDefaults()
{
    memset(g_rtcio, 0, sizeof(g_rtcio));
    for (const RtcPad& p : RTC_PADS)
    {
        if (p.gpio == 0 || p.gpio == 15) g_rtcio[p.reg / 4] |= 1u << p.rue;
        if (p.gpio == 2 || p.gpio == 4 || p.gpio == 12) g_rtcio[p.reg / 4] |= 1u << p.rde;
    }
}

static uint64_t rtcio_read(void*, uint64_t addr, unsigned)
{
    return addr < sizeof(g_rtcio) ? g_rtcio[addr / 4] : 0;
}

static void rtcio_write(void*, uint64_t addr, uint64_t data, unsigned)
{
    if (addr >= sizeof(g_rtcio)) return;
    uint32_t idx = (uint32_t)(addr / 4);
    uint32_t v = (uint32_t)data;
    // W1TS / W1TC companions of RTC_GPIO_OUT (0x00) and ENABLE (0x0C)
    if (addr == 0x04) { g_rtcio[0] |= v; return; }
    if (addr == 0x08) { g_rtcio[0] &= ~v; return; }
    if (addr == 0x10) { g_rtcio[3] |= v; return; }
    if (addr == 0x14) { g_rtcio[3] &= ~v; return; }
    g_rtcio[idx] = v;
    static int logged = 0;
    if (addr >= 0x84 && addr <= 0xB8 && logged < 40)   // pad registers: log the first few (debug)
    {
        logged++;
        elog("RTC IO write 0x%02X = 0x%08X", (unsigned)addr, v);
    }
    if (addr == 0x84 || addr == 0x88) updateDac();      // PAD_DAC1 / PAD_DAC2: dacWrite
}

static void installRtcio()
{
    memset(&g_rtcioOps, 0, sizeof(g_rtcioOps));
    g_rtcioOps.read = rtcio_read;
    g_rtcioOps.write = rtcio_write;
    g_rtcioOps.endianness = 2;                 // little endian
    g_rtcioOps.valid_min_access_size = 1;
    g_rtcioOps.valid_max_access_size = 4;
    g_rtcioOps.impl_min_access_size = 4;
    g_rtcioOps.impl_max_access_size = 4;
    rtcioResetDefaults();

    // The ESP32 sees its peripherals at 0x3FF4xxxx and again at 0x6000xxxx.
    void* sys = q_get_system_memory();
    const uint64_t base[2] = { 0x3FF48400ull, 0x60008400ull };
    for (int k = 0; k < 2; k++)
    {
        memset(&g_rtcioMr[k], 0, sizeof(g_rtcioMr[k]));
        q_mr_init_io(&g_rtcioMr[k], nullptr, &g_rtcioOps, nullptr, k ? "esp32link.rtcio.alias" : "esp32link.rtcio", 0x400);
        q_mr_add_overlap(sys, base[k], &g_rtcioMr[k], 10);   // above QEMU's "unimplemented" block
    }
    elog("RTC IO pull-up/pull-down registers installed");
}

// ---------------------------------------------------------------------------
//  GPIO32-39 outputs. QEMU's GPIO block keeps gpio_out1 but has no ENABLE1
//  register at all (Esp32GpioState: gpio_enable at +0x474, then gpio_pin[]),
//  and write_pin fires only for GPIO0-31. So the engine maps its own copy of
//  OUT1 (0x10, W1TS 0x14, W1TC 0x18) and ENABLE1 (0x2C, W1TS 0x30, W1TC 0x34)
//  over the GPIO block (0x3FF44000) and sends direction and level of
//  GPIO32/33 itself, at the moment of the write (34-39 are input only).
// ---------------------------------------------------------------------------
static uint32_t        g_out1 = 0, g_en1 = 0;       // GPIO32-39: bit 0 = GPIO32
static uint32_t        g_sentOut1 = 0, g_sentEn1 = 0;
static QemuMemoryRegionOps     g_gpio1Ops;
static QemuMemoryRegionStorage g_gpio1Mr[4];
static int             g_gpioToBoard[40];

static void updateGpio1()
{
    for (int g = 32; g <= 33; g++)
    {
        int pin = g_gpioToBoard[g];
        if (!pin) continue;
        uint32_t m = 1u << (g - 32);
        bool en = (g_en1 & m) != 0, wasEn = (g_sentEn1 & m) != 0;
        bool lv = (g_out1 & m) != 0, wasLv = (g_sentOut1 & m) != 0;
        if (en != wasEn) toModel(E32_EV_PIN_DIR, (uint16_t)pin, en ? 1u : 0u);
        if (en && (lv != wasLv || !wasEn)) toModel(E32_EV_PIN_OUT, (uint16_t)pin, lv ? 1u : 0u);
    }
    g_sentEn1 = g_en1;
    g_sentOut1 = g_out1;
}

static uint64_t gpio1_read(void* opaque, uint64_t addr, unsigned)
{
    if (addr != 0) return 0;                       // W1TS / W1TC read as 0
    return opaque ? g_en1 : g_out1;
}

static void gpio1_write(void* opaque, uint64_t addr, uint64_t data, unsigned)
{
    uint32_t& r = opaque ? g_en1 : g_out1;
    uint32_t v = (uint32_t)data & 0xFF;
    if (addr == 0) r = v;
    else if (addr == 4) r |= v;
    else if (addr == 8) r &= ~v;
    else return;
    updateGpio1();
}

static void installGpio1()
{
    memset(&g_gpio1Ops, 0, sizeof(g_gpio1Ops));
    g_gpio1Ops.read = gpio1_read;
    g_gpio1Ops.write = gpio1_write;
    g_gpio1Ops.endianness = 2;
    g_gpio1Ops.valid_min_access_size = 1;
    g_gpio1Ops.valid_max_access_size = 4;
    g_gpio1Ops.impl_min_access_size = 4;
    g_gpio1Ops.impl_max_access_size = 4;

    void* sys = q_get_system_memory();
    const uint64_t base[2] = { 0x3FF44000ull, 0x60004000ull };
    for (int k = 0; k < 4; k++)
    {
        bool en = (k & 1) != 0;                       // opaque: nullptr = OUT1, non-null = ENABLE1
        memset(&g_gpio1Mr[k], 0, sizeof(g_gpio1Mr[k]));
        q_mr_init_io(&g_gpio1Mr[k], nullptr, &g_gpio1Ops, en ? (void*)1 : nullptr,
                     en ? "esp32link.gpio.enable1" : "esp32link.gpio.out1", 12);
        q_mr_add_overlap(sys, base[k >> 1] + (en ? 0x2C : 0x10), &g_gpio1Mr[k], 10);   // above QEMU's GPIO block
    }
    elog("GPIO32-39 OUT1/ENABLE1 registers installed");
}

// ---------------------------------------------------------------------------
//  Touch sensor (touchRead). QEMU's sens block keeps no touch registers
//  (Esp32SensState: touch_sensor[10] + ADC values only); it reads back
//  1500 - touch_sensor[i] + rand()%20 and never sets TOUCH_MEAS_DONE, which
//  the IDF touch driver polls. So the engine owns SENS 0x58..0x8C
//  (SAR_TOUCH_CTRL1, THRES1-5, OUT1-5, CTRL2, 0x88, ENABLE):
//   - ENABLE bits 0-9 (TOUCH_PAD_WORKEN)  -> E32_EV_TOUCH on / off to the model
//   - OUT1-5: one count per slot (even slot = high 16 bits) from the pin
//     voltage the model reports (E32_EV_TOUCH_V)
//   - CTRL2 always reads with MEAS_DONE (bit 10) set
//  Register slot -> GPIO: slots 8 / 9 are T9 / T8 (the IDF swaps them).
// ---------------------------------------------------------------------------
static const int TOUCH_GPIO[10] = { 4, 0, 2, 15, 13, 12, 14, 27, 32, 33 };
enum { SENS_BLK = 0x58, SENS_CTRL1 = 0x58, SENS_OUT1 = 0x70, SENS_OUT5 = 0x80, SENS_CTRL2 = 0x84, SENS_ENABLE = 0x8C };
static uint32_t        g_sens[0x38 / 4];            // SENS 0x58..0x8F
static uint32_t        g_touchOn = 0;               // slots the model was told are on
static int             g_touchRatio[10];            // from the model: 1000 = not touched, 0 = pin at GND
static uint32_t        g_touchTrig = 0;             // pads that woke the chip (read in CTRL2 MEAS_EN until cleared)
static QemuMemoryRegionOps     g_sensOps;
static QemuMemoryRegionStorage g_sensMr[2];

static void touchReset()
{
    memset(g_sens, 0, sizeof(g_sens));
    g_touchOn = 0;
    g_touchTrig = 0;
    for (int k = 0; k < 10; k++) g_touchRatio[k] = 1000;
}

// Count of one slot, like a real board: ~75 untouched and ~10 with the pin at
// GND for the classic measure time of 0x1000 cycles (0.5 ms, Arduino core 2.x);
// it grows with TOUCH_MEAS_DELAY (core 3.x measures 5 ms: ~730 untouched).
static uint32_t touchCount(int slot)
{
    uint32_t meas = g_sens[(SENS_CTRL1 - SENS_BLK) / 4] & 0xFFFF;
    if (!meas) meas = 0x1000;
    double r = g_touchRatio[slot] / 1000.0;
    double c = 75.0 * meas / 4096.0 * (0.13 + 0.87 * r);
    return c > 65535.0 ? 65535u : (uint32_t)(c + 0.5);
}

// Touch wake-up from deep sleep: pads (register slots) whose count is past their threshold.
// CTRL1 bit 24 out_sel: 0 below / 1 above the threshold; bit 25 out_1en: 1 = SET1 alone
// (ENABLE bits 20-29), 0 = SET1 and SET2 (bits 10-19). THRES1-5: even slot high 16 bits.
static uint8_t g_touchPast[10];                    // slices in a row a pad has been past its threshold

static uint32_t touchThreshold(int k)
{
    uint32_t thr = g_sens[(0x5C - SENS_BLK) / 4 + k / 2];
    return (k & 1) ? thr & 0xFFFF : thr >> 16;
}

// every slice while sleeping: a pad counts after 3 slices in a row past its threshold
// (the model reports a pad right after touch is switched on again, before the pin has settled)
static uint32_t touchWakeMask()
{
    uint32_t ctrl1 = g_sens[(SENS_CTRL1 - SENS_BLK) / 4], en = g_sens[(SENS_ENABLE - SENS_BLK) / 4];
    uint32_t work = en & 0x3FF, set2 = (en >> 10) & 0x3FF, set1 = (en >> 20) & 0x3FF, touched = 0;
    for (int k = 0; k < 10; k++)
    {
        uint32_t c = touchCount(k), thr = touchThreshold(k);
        bool past = ((work >> k) & 1) && ((ctrl1 >> 24) & 1 ? c > thr : c < thr);
        g_touchPast[k] = past ? (uint8_t)std::min(g_touchPast[k] + 1, 100) : 0;
        if (g_touchPast[k] >= 3) touched |= 1u << k;
    }
    if ((ctrl1 >> 25) & 1) return touched & set1;
    return (touched & set1) && (touched & set2) ? touched & (set1 | set2) : 0;
}

static void touchLog(const char* when)
{
    uint32_t en = g_sens[(SENS_ENABLE - SENS_BLK) / 4];
    char line[400];
    int n = 0;
    for (int k = 0; k < 10; k++)
        if ((en >> k) & 1)
            n += sprintf_s(line + n, sizeof(line) - n, " T%d(GPIO%d) value %d count %u threshold %u%s;", k == 8 ? 9 : k == 9 ? 8 : k,
                           TOUCH_GPIO[k], g_touchRatio[k], touchCount(k), touchThreshold(k), ((en >> 20) >> k) & 1 ? " SET1" : "");
    elog("touch %s:%s", when, n ? line : " no pads");
}

static void updateTouch()
{
    uint32_t want = g_sens[(SENS_ENABLE - SENS_BLK) / 4] & 0x3FF;
    if (want == g_touchOn) return;
    EnterCriticalSection(&g_pushLock);
    for (int k = 0; k < 10; k++)
    {
        uint32_t m = 1u << k;
        if ((want & m) == (g_touchOn & m)) continue;
        int pin = g_gpioToBoard[TOUCH_GPIO[k]];
        if (!pin) continue;
        toModel(E32_EV_TOUCH, (uint16_t)pin, (want & m) ? 1u : 0u);
        static int logged = 0;
        if (logged < 30)
        {
            logged++;
            elog("touch %s on GPIO%d (slot %d)", (want & m) ? "on" : "off", TOUCH_GPIO[k], k);
        }
    }
    g_touchOn = want;
    LeaveCriticalSection(&g_pushLock);
}

static uint64_t sens_read(void*, uint64_t addr, unsigned)
{
    uint32_t reg = SENS_BLK + (uint32_t)(addr & ~3ull);
    if (reg >= SENS_OUT1 && reg <= SENS_OUT5)
    {
        int k = (int)(reg - SENS_OUT1) / 4;
        return ((uint64_t)touchCount(2 * k) << 16) | touchCount(2 * k + 1);
    }
    uint32_t v = g_sens[(reg - SENS_BLK) / 4];
    if (reg == SENS_CTRL2) v = (v & ~0x3FFu) | (1u << 10) | (g_touchTrig ? g_touchTrig : g_touchOn);   // done; MEAS_EN = wake-up pads / pads in use
    return v;
}

static void sens_write(void*, uint64_t addr, uint64_t data, unsigned)
{
    uint32_t reg = SENS_BLK + (uint32_t)(addr & ~3ull);
    if (reg >= SENS_OUT1 && reg <= SENS_OUT5) return;     // read only
    g_sens[(reg - SENS_BLK) / 4] = (uint32_t)data;
    if (reg == SENS_CTRL2 && (data & (1u << 30))) { g_touchTrig = 0; g_sens[(reg - SENS_BLK) / 4] &= ~(1u << 30); }   // MEAS_EN_CLR
    if (reg == SENS_ENABLE) updateTouch();
}

static void installSens()
{
    memset(&g_sensOps, 0, sizeof(g_sensOps));
    g_sensOps.read = sens_read;
    g_sensOps.write = sens_write;
    g_sensOps.endianness = 2;
    g_sensOps.valid_min_access_size = 1;
    g_sensOps.valid_max_access_size = 4;
    g_sensOps.impl_min_access_size = 4;
    g_sensOps.impl_max_access_size = 4;
    touchReset();

    void* sys = q_get_system_memory();
    const uint64_t base[2] = { 0x3FF48800ull, 0x60008800ull };
    for (int k = 0; k < 2; k++)
    {
        memset(&g_sensMr[k], 0, sizeof(g_sensMr[k]));
        q_mr_init_io(&g_sensMr[k], nullptr, &g_sensOps, nullptr, k ? "esp32link.touch.alias" : "esp32link.touch",
                     sizeof(g_sens));
        q_mr_add_overlap(sys, base[k] + SENS_BLK, &g_sensMr[k], 10);   // above QEMU's sens block
    }
    elog("touch sensor registers installed");
}

static int gpioPull(int gpio)
{
    bool up = false, down = false;
    if (g_iomux && gpio >= 0 && gpio < 40)
    {
        uint32_t m = g_iomux[gpio];
        up = (m & (1u << 8)) != 0;
        down = (m & (1u << 7)) != 0;
    }
    for (const RtcPad& p : RTC_PADS)
        if (p.gpio == gpio)
        {
            uint32_t r = g_rtcio[p.reg / 4];
            if (r & (1u << p.rue)) up = true;
            if (r & (1u << p.rde)) down = true;
        }
    return up ? 1 : (down ? 2 : 0);
}

// Called once per slice (main loop thread): report pull changes to the model.
static void updatePulls()
{
    int n = g_pinmap[0];
    for (int i = 1; i <= n && i < E32LINK_MAX_PINS; i++)
    {
        if (g_pinmap[i] < 0) continue;
        uint8_t p = (uint8_t)gpioPull(g_pinmap[i]);
        if (p != g_lastPull[i])
        {
            g_lastPull[i] = p;
            toModel(E32_EV_PULL, (uint16_t)i, p);
        }
    }
}

// QEMU reset hook (every system reset: EN, watchdog, esp_restart, ...).
// QEMU's GPIO block keeps its output registers over a reset; a real chip
// does not, so clear them here.
enum { OFS_GPIO = 0xae80, OFS_GPIO_OUT = 0x448, OFS_GPIO_OUT1 = 0x44c, OFS_GPIO_ENABLE = 0x474 };
//
// QEMU's ESP32 machine uses one "system reset" for two things: a real chip
// reset, and a restart of only one CPU core (ESP-IDF restarts core 1 at every
// boot). soc.requested_reset says which: bit 0 / bit 1 = that CPU only,
// bit 2 = the digital peripherals too (= chip reset). The machine clears the
// field during its own reset, which may run before or after this hook. So we
// also keep a marker in the IO MUX register of GPIO20 (no such pad on the
// WROOM module, the firmware never uses it): only a chip reset clears it.
enum { OFS_REQUESTED_RESET = 0x1c380, MUX_MARK_GPIO = 20, MUX_MARK = 0x80000800u };
static uint32_t* g_iomuxW = nullptr;

static void plantResetMarker()
{
    if (g_iomuxW) g_iomuxW[MUX_MARK_GPIO] = MUX_MARK;
}

// ---------------------------------------------------------------------------
//  ESP32 clock from the PC (--clock local | utc). The ESP-IDF keeps the "boot time"
//  (time() at power-on, us since 1970) in RTC_CNTL_STORE2 / STORE3
//  (RTC_BOOT_TIME_LOW_REG / HIGH_REG; QEMU: rtc_cntl.scratch_reg[2..3]):
//  time() = boot time + RTC time since power-on. Written before the firmware runs
//  (and after a reset that cleared it), so time() starts at the PC time and then
//  runs with the simulation time. Off: the firmware starts at 1970, like a real chip.
// ---------------------------------------------------------------------------
enum { OFS_RTC_CNTL = 0xbe80, OFS_RTC_SCRATCH = 0x4b4 };
static int     g_clockMode = 0;                // 0 off, 1 PC local time, 2 PC UTC
static int64_t g_clockStartUs = 0;             // PC time at the start of the simulation

static void setBootTime()
{
    if (!g_clockMode || !g_soc) return;
    uint32_t* scratch = (uint32_t*)(g_soc + OFS_RTC_CNTL + OFS_RTC_SCRATCH);
    if (scratch[2] || scratch[3]) return;      // already set (the firmware's own, or still ours)
    scratch[2] = (uint32_t)g_clockStartUs;
    scratch[3] = (uint32_t)((uint64_t)g_clockStartUs >> 32);
}

// ---------------------------------------------------------------------------
//  Sleep. QEMU's RTC controller has no sleep at all: esp_deep_sleep_start()
//  never woke up. The engine keeps the sleep registers of RTC_CNTL itself
//  (QEMU only handles 0x00, 0x0C-0x14, 0x34, 0x4C-0x58, 0x70, 0xAC-0xBC):
//    0x04 / 0x08 SLP_TIMER0/1  wake-up time in RTC slow clock ticks (48 bit)
//    0x18 STATE0               bit 31 SLEEP_EN starts the sleep
//    0x34 RESET_STATE          read: QEMU's reset causes, but 5 (DEEPSLEEP_RESET)
//                              after a deep-sleep wake-up
//    0x38 WAKEUP_STATE         bits 11-21 wake-up enable, bits 0-10 wake-up cause
//    0x3C-0x48 INT_ENA/RAW/ST/CLR  RAW bit 0 SLP_WAKEUP (light sleep done)
//    0x60 EXT_WAKEUP_CONF      bit 30 ext0 level, bit 31 ext1 mode (1 any high)
//    0x64 SLP_REJECT_CONF, 0x80-0x8C power (0x84 bit 31 DG_WRAP_PD_EN = deep)
//    0xCC EXT_WAKEUP1          bits 0-17 ext1 RTC pins, bit 18 clear status
//    0xD0 EXT_WAKEUP1_STATUS   pins that woke the chip (ext1)
//    0x100 / 0x104 RTC_MEM_CONF / CRC_RES  RTC memory CRC: always finished
//  Deep sleep: the chip is held in reset (like EN low: pins float, no CPU
//  time); every slice the wake-up sources are checked: timer (QEMU's RTC time
//  = (virtual ns - time base) x slow clock / 1e9), ext0 (one RTC pin at a level,
//  select in RTC IO 0xBC bits 27-31), ext1 (any high / all low). Wake-up =
//  chip reset with reason DEEPSLEEP_RESET; RTC memory stays (RTC_DATA_ATTR).
//  Light sleep: the CPU polls INT_RAW; at the wake-up SLP_WAKEUP is set. GPIO wake-up (bit 2,
//  light sleep only): a pin with GPIO_PINn wakeup_enable at its level (int type 4 low / 5 high).
//  Touch (trigger bit 8): pads past their threshold (touchWakeMask); the touch settings
//  and the model's touch pads stay over the deep-sleep resets (SENS is RTC).
// ---------------------------------------------------------------------------
enum { OFS_RTCC_SLOWCLK_FREQ = 0x494, OFS_RTCC_TIME_BASE = 0x498, OFS_RTCC_RESET_CAUSE = 0x4d4 };   // Esp32RtcCntlState
static const int RTCIO_GPIO[18] = { 36, 37, 38, 39, 34, 35, 25, 26, 33, 32, 4, 0, 2, 15, 13, 12, 14, 27 };
static uint32_t        g_rtcc[0x108 / 4];          // our RTC_CNTL registers (by offset / 4)
static volatile bool   g_deepSleep = false, g_lightSleep = false;
static bool            g_wakeResetPending = false;  // the next reset is a deep-sleep wake-up
static bool            g_engineChipReset = false;   // the engine asked for the next reset: a chip reset
static int             g_sleepStep = 0;             // deep sleep: 0 reset not yet done, 1 reset requested, 2 CPUs stalled
static int             g_sleepSlices = 0;           // slices since the CPUs were stalled (pins settle first)
static bool            g_cpuStalled[2] = {};
enum { OFS_XTENSA_ENV = 0x2800, OFS_ENV_RUNSTALL = 0x1150, OFS_ENV_TIME_BASE = 0x12b0 };   // XtensaCPU: env after
                                                                                          // CPUState; env fields (DWARF)
static int64_t g_stallNs[2];                        // virtual time when the core was stalled

// RUNSTALL a CPU core: it stops (no instructions, no time cost; with icount the clock jumps ahead)
static void stallCpu(int i, bool stall)
{
    if (g_cpuStalled[i] == stall) return;
    uint8_t* cpu = (uint8_t*)q_get_cpu(i);
    if (!cpu) return;
    q_runstall(cpu + OFS_XTENSA_ENV, stall);
    if (stall) g_stallNs[i] = now_ns();
    else {
        // the stall also leaves CPU_INTERRUPT_HALT (0x20) pending: the core would halt again at once
        q_cpu_reset_interrupt(cpu, 0x20);
        // A real core's clock is off in light sleep: CCOUNT stops. QEMU computes CCOUNT from the virtual
        // clock (ccount_base + ticks(now - time_base)), so it jumped by the sleep time; more than 2^31
        // cycles (8.9 s at 240 MHz) and the FreeRTOS tick (signed CCOMPARE - CCOUNT) waited for the
        // 32-bit wrap: delay() after a 10 s light sleep took ~8 s too long. Move time_base on instead.
        *(uint64_t*)(cpu + OFS_XTENSA_ENV + OFS_ENV_TIME_BASE) += (uint64_t)(now_ns() - g_stallNs[i]);
    }
    g_cpuStalled[i] = stall;
}

// both cores stalled: nothing runs, the engine moves the clock on itself (QEMU's own idle
// "warp" moves it slowly)
static bool allCoresStalled()
{
    for (int i = 0; i < 2; i++) {
        uint8_t* cpu = (uint8_t*)q_get_cpu(i);
        if (cpu && !cpu[OFS_XTENSA_ENV + OFS_ENV_RUNSTALL]) return false;
    }
    return true;
}
static bool            g_reportDeepSleep = false;   // RESET_STATE reads DEEPSLEEP_RESET (until the next reset)
static int8_t          g_inLevel[E32LINK_MAX_PINS]; // input level of each board pin from the model, -1 = unknown
static uint32_t        g_crcRead = 0;
static int64_t         g_timerWakeNs = 0;           // timer wake-up in virtual ns (QEMU's RTC restarts at every
                                                    // reset while the chip is held, so it is computed once)
static int64_t         g_rtcBase = 0;               // RTC time base before the sleep: restored after the wake-up
static bool            g_restoreRtcBase = false;
static QemuMemoryRegionOps     g_rtccOps;
static QemuMemoryRegionStorage g_rtccMr[6];

// UART wake-up (light sleep), see "UART wake-up" below
static void     uartWakeStart(uint32_t ena);
static uint32_t uartWakeCheck();
static void     uartWakeStop();
static void     uartWakeReset();

static uint64_t rtcTicksNow()
{
    const uint8_t* rc = g_soc + OFS_RTC_CNTL;
    uint32_t freq = *(const uint32_t*)(rc + OFS_RTCC_SLOWCLK_FREQ);
    int64_t base = *(const int64_t*)(rc + OFS_RTCC_TIME_BASE);
    int64_t ns = now_ns() - base;
    return ns > 0 ? (uint64_t)((double)ns * freq / 1e9) : 0;
}

static int rtcPinLevel(int ch)
{
    if (ch < 0 || ch >= 18) return -1;
    int pin = g_gpioToBoard[RTCIO_GPIO[ch]];
    return pin ? g_inLevel[pin] : -1;
}

// wake-up sources that are true now (trigger bits: 0 ext0, 1 ext1, 3 timer)
static uint32_t sleepWakeCheck()
{
    uint32_t ena = (g_rtcc[0x38 / 4] >> 11) & 0x7FF, cause = 0;
    if ((ena & 0x008) && now_ns() >= g_timerWakeNs) cause |= 0x008;
    if ((ena & 0x004) && g_lightSleep && g_soc) {         // GPIO (light sleep only): GPIO_PINn bit 10 wakeup_enable,
        const uint32_t* pinReg = (const uint32_t*)(g_soc + OFS_GPIO + 0x478);   // bits 7-9: 4 low / 5 high level
        for (int g = 0; g < 40; g++) {
            uint32_t r = pinReg[g], type = (r >> 7) & 7;
            int b = g_gpioToBoard[g];
            if (!((r >> 10) & 1) || !b) continue;
            if ((type == 4 && g_inLevel[b] == 0) || (type == 5 && g_inLevel[b] == 1)) { cause |= 0x004; break; }
        }
    }
    if ((ena & 0x0C0) && g_lightSleep) cause |= uartWakeCheck() & ena;   // UART0 / UART1 (light sleep only)
    if (g_deepSleep && g_sleepSlices < 6) return cause;  // pins: the model has released them, wait for the pulls
    if (ena & 0x100) {                                    // touch (the model measures the pads again)
        uint32_t m = touchWakeMask();
        if (m) { cause |= 0x100; g_touchTrig = m; touchLog("wake-up"); }
    }
    if (ena & 0x001) {
        int ch = (int)((g_rtcio[0xBC / 4] >> 27) & 0x1F);
        if (rtcPinLevel(ch) == (int)((g_rtcc[0x60 / 4] >> 30) & 1)) cause |= 0x001;
    }
    if (ena & 0x002) {
        uint32_t mask = g_rtcc[0xCC / 4] & 0x3FFFF, high = 0;
        bool allLow = mask != 0;
        for (int ch = 0; ch < 18; ch++) {
            if (!(mask & (1u << ch))) continue;
            int lv = rtcPinLevel(ch);
            if (lv == 1) high |= 1u << ch;
            if (lv != 0) allLow = false;
        }
        bool anyHigh = (g_rtcc[0x60 / 4] >> 31) & 1;
        if (anyHigh ? high != 0 : allLow) {
            cause |= 0x002;
            g_rtcc[0xD0 / 4] = anyHigh ? high : mask;
        }
    }
    return cause;
}

static void enterSleep()
{
    bool deep = (g_rtcc[0x84 / 4] >> 31) & 1;             // DG_WRAP_PD_EN: the digital core powers down
    uint32_t ena = (g_rtcc[0x38 / 4] >> 11) & 0x7FF;
    g_rtcc[0x38 / 4] &= ~0x7FFu;                          // no cause yet
    if (deep) { g_deepSleep = true; g_sleepStep = 0; g_sleepSlices = 0; }
    else { g_lightSleep = true; g_rtcc[0x40 / 4] &= ~3u; }
    double secs = 0;
    if (ena & 8) {
        uint64_t target = g_rtcc[0x04 / 4] | ((uint64_t)(g_rtcc[0x08 / 4] & 0xFFFF) << 32), now = rtcTicksNow();
        uint32_t freq = *(const uint32_t*)(g_soc + OFS_RTC_CNTL + OFS_RTCC_SLOWCLK_FREQ);
        secs = target > now && freq ? (double)(target - now) / freq : 0;
        g_timerWakeNs = now_ns() + (int64_t)(secs * 1e9);
    }
    g_rtcBase = *(const int64_t*)(g_soc + OFS_RTC_CNTL + OFS_RTCC_TIME_BASE);
    uint32_t known = deep ? 0x10Bu : 0x1CFu;              // GPIO and UART wake-up: light sleep only
    elog("%s sleep: wake-up on%s%s%s%s%s%s%s%s (%.3f s)", deep ? "deep" : "light", (ena & 8) ? " timer" : "", (ena & 1) ? " ext0" : "",
         (ena & 2) ? " ext1" : "", (ena & 0x100) ? " touch" : "", (!deep && (ena & 4)) ? " gpio" : "",
         (!deep && (ena & 0x40)) ? " uart0" : "", (!deep && (ena & 0x80)) ? " uart1" : "", (ena & ~known) ? " (other sources: not simulated)" : "", secs);
    if (!(ena & known)) elog("WARNING: no simulated wake-up source - the chip sleeps until the simulation stops");
    if (ena & 0x100) { memset(g_touchPast, 0, sizeof(g_touchPast)); touchLog("at sleep start"); }
    if (!deep && (ena & 0xC0)) uartWakeStart(ena);
}

// every slice: wake up when a source is true
static void updateSleep()
{
    if (g_restoreRtcBase && !g_deepSleep) {
        g_restoreRtcBase = false;
        *(int64_t*)((uint8_t*)g_soc + OFS_RTC_CNTL + OFS_RTCC_TIME_BASE) = g_rtcBase;
    }
    if (!g_deepSleep && !g_lightSleep) return;
    if (g_deepSleep && g_enHeld) {                        // EN low ends the sleep: a normal reset follows
        g_deepSleep = false;
        stallCpu(0, false); stallCpu(1, false);
        return;
    }
    if (g_deepSleep) {
        // one chip reset (peripherals off, the model releases the pins), then both cores stall
        if (g_sleepStep == 0) { g_sleepStep = 1; g_engineChipReset = true; q_reset_request(SHUTDOWN_CAUSE_GUEST_RESET_); return; }
        if (g_sleepStep == 1) { g_sleepStep = 2; stallCpu(0, true); stallCpu(1, true); }
        g_sleepSlices++;
        updatePulls();                                    // RTC pulls stay on in deep sleep
        updateTouch();                                    // ... and the touch pads (switched on again after the reset)
    }
    else {                                                // light sleep: the CPU waits (no busy polling); the IDF
        stallCpu(0, true);                                // parks the other core in a spin loop (IPC ISR) for the
        stallCpu(1, true);                                // sleep: stall it too, so nothing runs (fast slices)
    }
    uint32_t cause = sleepWakeCheck();
    if (!cause) return;
    g_rtcc[0x38 / 4] = (g_rtcc[0x38 / 4] & ~0x7FFu) | cause;
    elog("wake-up (cause 0x%03X)", cause);
    if (g_lightSleep) {
        g_lightSleep = false;
        uartWakeStop();
        g_rtcc[0x40 / 4] |= 1;                            // SLP_WAKEUP: the CPU goes on
        stallCpu(0, false);
        stallCpu(1, false);                               // back to its spin loop until core 0 lets it go
        return;
    }
    g_deepSleep = false;
    stallCpu(0, false);                                   // core 1: the reset / boot stalls it until the app starts it
    g_cpuStalled[1] = false;
    g_wakeResetPending = true;
    g_engineChipReset = true;
    q_reset_request(SHUTDOWN_CAUSE_GUEST_RESET_);
}

// chip reset (from onSystemReset): a deep-sleep wake-up keeps the RTC registers and
// reports DEEPSLEEP_RESET; any other reset clears them
static void sleepOnReset()
{
    uartWakeReset();                                      // the model drops its edge pins at a reset
    if (g_deepSleep) return;                              // held in sleep: nothing changes
    g_reportDeepSleep = g_wakeResetPending;
    if (g_wakeResetPending) g_restoreRtcBase = true;      // the RTC timer ran on through the sleep
    else memset(g_rtcc, 0, sizeof(g_rtcc));
    g_wakeResetPending = false;
    g_lightSleep = false;
    g_rtcc[0x18 / 4] &= ~0x80000000u;
}

static uint64_t rtcc_read(void* opaque, uint64_t addr, unsigned)
{
    uint32_t ofs = (uint32_t)(uintptr_t)opaque + (uint32_t)addr;
    switch (ofs) {
    case 0x34: {                                          // RESET_STATE: PRO cause bits 0-5, APP 6-11
        if (g_reportDeepSleep) return 5u | (5u << 6);
        const uint8_t* rc = g_soc + OFS_RTC_CNTL;
        return (*(const uint32_t*)(rc + OFS_RTCC_RESET_CAUSE) & 0x3F) | ((*(const uint32_t*)(rc + OFS_RTCC_RESET_CAUSE + 4) & 0x3F) << 6);
    }
    case 0x44: return g_rtcc[0x40 / 4] & g_rtcc[0x3C / 4];  // INT_ST
    case 0x100: return g_rtcc[0x100 / 4] | 0x80000000u;     // RTC_MEM_CONF: CRC always finished
    case 0x104: return 0xC0DE0000u + (g_crcRead++);         // CRC_RES: never the stored one -> no wake stub, normal boot
    default: return ofs < sizeof(g_rtcc) ? g_rtcc[ofs / 4] : 0;
    }
}

static void rtcc_write(void* opaque, uint64_t addr, uint64_t data, unsigned)
{
    uint32_t ofs = (uint32_t)(uintptr_t)opaque + (uint32_t)addr, v = (uint32_t)data;
    switch (ofs) {
    case 0x34: return;                                    // reset causes: read only
    case 0x38: g_rtcc[0x38 / 4] = (g_rtcc[0x38 / 4] & 0x7FFu) | (v & ~0x7FFu); return;   // cause bits read only
    case 0x48: g_rtcc[0x40 / 4] &= ~v; return;            // INT_CLR
    case 0xCC:
        if (v & (1u << 18)) g_rtcc[0xD0 / 4] = 0;         // clear ext1 status
        g_rtcc[0xCC / 4] = v & ~(1u << 18);
        return;
    case 0xD0: return;
    case 0x18:
        g_rtcc[0x18 / 4] = v & ~0x80000000u;
        if (v & 0x80000000u) enterSleep();                // SLEEP_EN
        return;
    default:
        if (ofs < sizeof(g_rtcc)) g_rtcc[ofs / 4] = v;
        return;
    }
}

static void installSleep()
{
    memset(&g_rtccOps, 0, sizeof(g_rtccOps));
    g_rtccOps.read = rtcc_read;
    g_rtccOps.write = rtcc_write;
    g_rtccOps.endianness = 2;
    g_rtccOps.valid_min_access_size = 1;
    g_rtccOps.valid_max_access_size = 4;
    g_rtccOps.impl_min_access_size = 4;
    g_rtccOps.impl_max_access_size = 4;
    memset(g_rtcc, 0, sizeof(g_rtcc));
    memset(g_inLevel, -1, sizeof(g_inLevel));
    if (!g_soc) { elog("sleep registers not installed (no SoC state)"); return; }
    // RTC_CNTL blocks the engine owns (offset, size); the opaque value is the offset
    static const struct { uint32_t ofs, size; } BLK[6] = {
        { 0x04, 0x08 }, { 0x18, 0x04 }, { 0x34, 0x18 }, { 0x60, 0x08 }, { 0x80, 0x10 }, { 0xCC, 0x08 } };
    void* sys = q_get_system_memory();
    for (int k = 0; k < 6; k++) {
        memset(&g_rtccMr[k], 0, sizeof(g_rtccMr[k]));
        q_mr_init_io(&g_rtccMr[k], nullptr, &g_rtccOps, (void*)(uintptr_t)BLK[k].ofs, "esp32link.rtccntl", BLK[k].size);
        q_mr_add_overlap(sys, 0x3FF48000ull + BLK[k].ofs, &g_rtccMr[k], 10);
    }
    static QemuMemoryRegionStorage crcMr;
    memset(&crcMr, 0, sizeof(crcMr));
    q_mr_init_io(&crcMr, nullptr, &g_rtccOps, (void*)(uintptr_t)0x100, "esp32link.rtcmemcrc", 0x08);
    q_mr_add_overlap(sys, 0x3FF48100ull, &crcMr, 10);
    elog("sleep registers installed (deep / light sleep: timer, ext0, ext1)");
}

static void onSystemReset(void*)
{
    bool chip = true;
    if (g_soc && g_iomuxW)
    {
        uint32_t rr = *(const uint32_t*)(g_soc + OFS_REQUESTED_RESET);
        if (rr != 0) chip = (rr & 4u) != 0;                     // we run before the machine's reset
        else chip = g_iomuxW[MUX_MARK_GPIO] != MUX_MARK;        // we run after it: was the marker wiped?
        plantResetMarker();
    }
    if (g_engineChipReset) { g_engineChipReset = false; chip = true; }   // our own request (deep-sleep wake-up)
    if (!chip) return;                                          // only one CPU core restarted
    if (g_deepSleep && g_heldAnnounced) return;                 // deep sleep: the hold reset of every slice

    if (g_soc)
    {
        uint8_t* gpio = (uint8_t*)g_soc + OFS_GPIO;
        *(uint32_t*)(gpio + OFS_GPIO_OUT) = 0;
        *(uint32_t*)(gpio + OFS_GPIO_OUT1) = 0;
        *(uint32_t*)(gpio + OFS_GPIO_ENABLE) = 0;
    }
    g_out1 = g_en1 = g_sentOut1 = g_sentEn1 = 0;   // our GPIO32-39 registers (the model releases the pins)
    if (g_deepSleep || g_wakeResetPending)
    {                                              // touch is RTC: settings and wake-up status kept; the model
        g_touchOn = 0;                             // gets the pads again and reports fresh values (an old
        for (int k = 0; k < 10; k++) g_touchRatio[k] = 1000;   // "pressed" value would be read at boot)
    }
    else
        touchReset();                              // ... and touch
    rmtRxReset();                                  // ... and RMT receive (the model drops its edge pins)
    pcntReset();                                   // ... and the pulse counter
    mcpwmReset();                                  // ... and the motor PWM
    i2csReset();                                   // ... and the I2C slave (the model forgets it)
    spisReset();                                   // ... and the SPI slave
    canReset();                                    // ... and TWAI (the model releases the CAN pins)
    i2sReset();                                    // ... and I2S (the model releases the I2S pins)
    if (g_btOn) btReset();                         // ... and the Bluetooth controller
    if (!g_deepSleep && !g_wakeResetPending)
        rtcioResetDefaults();                      // RTC IO is in the RTC domain: kept over a deep sleep
    sleepOnReset();                                // deep sleep: wake-up reason / RTC registers
    memset(g_lastPull, 0, sizeof(g_lastPull));   // the model forgets its pulls on reset too
    memset(g_pwm, 0, sizeof(g_pwm));             // ... and its PWM outputs
    memset(g_busPinsSent, 0, sizeof(g_busPinsSent));  // ... and its bus pins
    g_dac[0] = g_dac[1] = -1;                    // ... and its DAC outputs
    memset(g_rmtPin, 0, sizeof(g_rmtPin));       // ... and its RMT pins (the model released them all)
    if (g_enHeld || g_deepSleep)
    {
        if (g_heldAnnounced) return;            // held (EN low / deep sleep): we reset every slice, say it once
        g_heldAnnounced = true;
        toModel(E32_EV_RESET, 0, 0);
    }
    else
    {
        g_heldAnnounced = false;
        toModel(E32_EV_RESET, 0, 1);
        uint32_t cause = g_reportDeepSleep ? 5 : g_soc ? *(const uint32_t*)(g_soc + 0xc354) : 0;   // rtc_cntl.reset_cause[0]
        elog("chip reset (reset cause %u)", cause);
    }
}


// ===========================================================================
//  LEDC PWM
// ===========================================================================
//  QEMU's LEDC does not make a waveform; it keeps per timer the frequency
//  (Hz) and per channel the duty (percent). A pin outputs a channel when its
//  GPIO matrix output select is an LEDC signal: 71..78 = high-speed channel
//  0..7, 79..86 = low-speed channel 0..7 (our channels 8..15).
//  Channel -> timer: CONF0 bits 0-1 (high-speed: timers 0-3, low-speed 4-7).
//  CONF0 bit 2 = output enabled, bit 3 = idle level. OUT_SEL bit 9 = invert.
//  We send E32_EV_PWM to the model whenever what a pin outputs changes.
// ---------------------------------------------------------------------------
static const uint32_t* g_outSel = nullptr;      // gpio_out_sel[40]
static const uint32_t* g_ledcConf0 = nullptr;   // channel_conf0_reg[16]
static const uint32_t* g_ledcFreq = nullptr;    // freq[8]
static const float*    g_ledcDuty = nullptr;    // duty[16], percent
static void updatePwm()
{
    if (!g_outSel || !g_ledcConf0 || !g_ledcFreq || !g_ledcDuty) return;
    EnterCriticalSection(&g_pushLock);           // vCPU thread (sync) and main thread (slices)
    int n = g_pinmap[0];
    for (int i = 1; i <= n && i < E32LINK_MAX_PINS; i++)
    {
        int g = g_pinmap[i];
        if (g < 0 || g > 39) continue;
        uint32_t sel = g_outSel[g];
        uint32_t sig = sel & 0x1FF;
        PwmOut st = { false, 0, 0 };
        if (sig >= 71 && sig <= 86)
        {
            int ch = (int)sig - 71;                              // 0..15
            uint32_t conf = g_ledcConf0[ch];
            int timer = (ch < 8 ? 0 : 4) + (int)(conf & 3);
            double duty;
            if (conf & (1u << 2)) duty = g_ledcDuty[ch];         // output enabled
            else duty = (conf & (1u << 3)) ? 100.0 : 0.0;        // idle level
            if (sel & (1u << 9)) duty = 100.0 - duty;            // inverted in the GPIO matrix
            if (duty < 0) duty = 0;
            if (duty > 100) duty = 100;
            st.on = true;
            st.duty1000 = (uint32_t)(duty * 1000.0 + 0.5);
            st.freq = g_ledcFreq[timer];
        }
        PwmOut& old = g_pwm[i];
        if (st.on != old.on || (st.on && (st.duty1000 != old.duty1000 || st.freq != old.freq)))
        {
            old = st;
            if (st.on) toModel(E32_EV_PWM, (uint16_t)i, st.duty1000, st.freq);
            else toModel(E32_EV_PWM, (uint16_t)i, 0xFFFFFFFFu, 0);
            static int logged = 0;
            if (logged < 30)
            {
                logged++;
                if (st.on) elog("PWM on GPIO%d: %u Hz, duty %.3f %%", g, st.freq, st.duty1000 / 1000.0);
                else elog("PWM off on GPIO%d", g);
            }
        }
    }
    LeaveCriticalSection(&g_pushLock);
}


// ===========================================================================
//  Signal routing: which board pin carries a peripheral signal
// ===========================================================================
//  A pad is either in GPIO-matrix mode (IO MUX function field == 2), then
//  gpio_out_sel[gpio] holds the output signal number, or it uses a direct
//  IO MUX function (fast "native" pins, e.g. VSPI on 18/19/23/5).
//  Inputs: gpio_in_sel[signal] bit 7 = via matrix, bits 0-5 = GPIO.
//  Signal numbers (ESP32 gpio_sig_map.h): U0 14, U1 17, U2 198,
//  I2C0 SCL 29 / SDA 30, I2C1 SCL 95 / SDA 96,
//  HSPI CLK 8 Q 9 D 10 CS0 11 CS1 61 CS2 62, VSPI CLK 63 Q 64 D 65 CS0 68 CS1 69 CS2 70.
// ---------------------------------------------------------------------------
static const uint32_t* g_inSel = nullptr;       // gpio_in_sel[256]

// IO MUX "native" functions. out = 1: the pin outputs the signal (TXD, CLK,
// MOSI, CS), out = 0: the pin feeds the signal in (RXD, MISO). U2TXD and U2RXD
// are both signal 198, so the direction matters.
struct NativeFn { int gpio, mux, sig, out; };
static const NativeFn NATIVE[] = {
    {  1, 0,  14, 1 }, {  3, 0,  14, 0 },                     // U0TXD / U0RXD
    { 17, 4, 198, 1 }, { 16, 4, 198, 0 },                     // U2TXD / U2RXD
    { 18, 1,  63, 1 }, { 19, 1,  64, 0 }, { 23, 1,  65, 1 }, {  5, 1,  68, 1 },   // VSPI CLK Q D CS0
    { 14, 1,   8, 1 }, { 12, 1,   9, 0 }, { 13, 1,  10, 1 }, { 15, 1,  11, 1 },   // HSPI CLK Q D CS0
};

static int muxFn(int gpio) { return g_iomux ? (int)((g_iomux[gpio] >> 12) & 7) : 2; }

// signal a pin outputs through its IO MUX function, -1 = none
static int nativeOutSig(int gpio)
{
    int m = muxFn(gpio);
    for (const NativeFn& n : NATIVE) if (n.out && n.gpio == gpio && n.mux == m) return n.sig;
    return -1;
}

// board pin that outputs 'sig', 0 = none
static int outPinFor(int sig)
{
    if (!g_outSel) return 0;
    for (int g = 0; g < 40; g++)
    {
        int b = g_gpioToBoard[g];
        if (!b) continue;
        if (muxFn(g) == 2) { if ((int)(g_outSel[g] & 0x1FF) == sig) return b; }
        else if (nativeOutSig(g) == sig) return b;
    }
    return 0;
}

// board pin that feeds input signal 'sig', 0 = none
static int inPinFor(int sig)
{
    if (!g_inSel) return 0;
    uint32_t v = g_inSel[sig];
    if (v & 0x80) { int g = (int)(v & 0x3F); return g < 40 ? g_gpioToBoard[g] : 0; }
    for (const NativeFn& n : NATIVE)
        if (!n.out && n.sig == sig && muxFn(n.gpio) == n.mux && g_gpioToBoard[n.gpio]) return g_gpioToBoard[n.gpio];
    return 0;
}

static int uartTxPin(int uart)
{
    if (uart == 1) return outPinFor(17);
    if (uart == 2) return outPinFor(198);
    return 0;
}


static uint32_t pack4(int p0, int p1, int p2, int p3)
{
    return (uint32_t)(p0 & 0xFF) | ((uint32_t)(p1 & 0xFF) << 8) | ((uint32_t)(p2 & 0xFF) << 16) | ((uint32_t)(p3 & 0xFF) << 24);
}

static bool spisActive(int h);

// Called at every sync pulse, every slice and before every bus request.
static void updateRouting()
{
    if (!g_outSel || !g_inSel) return;
    EnterCriticalSection(&g_pushLock);
    BusPins now[6];
    now[0].b = pack4(outPinFor(29), outPinFor(30), 0, 0);  now[0].c = 0;          // I2C0 scl, sda
    now[1].b = pack4(outPinFor(95), outPinFor(96), 0, 0);  now[1].c = 0;          // I2C1
    now[2].b = pack4(outPinFor(8), outPinFor(10), inPinFor(9), 0);                // HSPI clk, mosi, miso
    now[2].c = pack4(outPinFor(11), outPinFor(61), outPinFor(62), 0);
    now[3].b = pack4(outPinFor(63), outPinFor(65), inPinFor(64), 0);              // VSPI
    now[3].c = pack4(outPinFor(68), outPinFor(69), outPinFor(70), 0);
    for (int h = 0; h < 2; h++)                                                   // SPI slave: the pins are
        if (spisActive(h)) now[2 + h].b = now[2 + h].c = 0;                       // not a master bus
    now[4].b = pack4(outPinFor(17), inPinFor(17), 0, 0);   now[4].c = realBaud(1); // UART1 tx, rx
    now[5].b = pack4(outPinFor(198), inPinFor(198), 0, 0); now[5].c = realBaud(2); // UART2
    for (int k = 0; k < 6; k++)
    {
        if (g_busPinsSent[k] && now[k].b == g_busPins[k].b && now[k].c == g_busPins[k].c) continue;
        bool wasEmpty = !g_busPinsSent[k] || g_busPins[k].b == 0;
        g_busPins[k] = now[k];
        g_busPinsSent[k] = true;
        if (wasEmpty && now[k].b == 0) continue;           // nothing routed, nothing to say
        toModel(E32_EV_BUSPINS, (uint16_t)k, now[k].b, now[k].c);
        static const char* NAMES[6] = { "I2C0", "I2C1", "HSPI", "VSPI", "UART1", "UART2" };
        elog("%s pins (board pins): %u %u %u %u  extra 0x%08X", NAMES[k],
             now[k].b & 0xFF, (now[k].b >> 8) & 0xFF, (now[k].b >> 16) & 0xFF, now[k].b >> 24, now[k].c);
    }
    // RMT channel 0..7 output = GPIO matrix signal 87..94. A pin no longer routed to its channel
    // goes back to a plain GPIO in the model (tick 0 = released).
    for (int ch = 0; ch < 8; ch++)
    {
        int p = outPinFor(87 + ch);
        if (p == g_rmtPin[ch]) continue;
        if (g_rmtPin[ch]) toModel(E32_EV_RMT, (uint16_t)g_rmtPin[ch], 0, 0);
        g_rmtPin[ch] = p;
        elog("RMT channel %d pin (board pin): %d", ch, p);
    }
    LeaveCriticalSection(&g_pushLock);
}

// ---------------------------------------------------------------------------
//  RMT: QEMU calls rmt_event once per 32-bit RMT item when a channel sends
//  (all items at once, no timing). The model plays them out on the pin with
//  the tick length computed here: divider (conf0 bits 0-7, 0 = 256) of APB
//  (conf1 bit 17 REF_ALWAYS_ON = 1) or REF_TICK 1 MHz. Idle level: conf1
//  bit 18 IDLE_OUT_LV when bit 19 IDLE_OUT_EN is set, else low.
// ---------------------------------------------------------------------------
enum { OFS_RMT = 0x14090, OFS_RMT_CONF1 = 0x478 };   // soc.rmt (Esp32RmtState), conf1[8]

// tick length of an RMT channel in ps: divider of APB (conf1 bit 17) or REF_TICK
static uint32_t rmtTickPs(int ch, uint32_t conf0)
{
    uint32_t conf1 = g_soc ? *(const uint32_t*)(g_soc + OFS_RMT + OFS_RMT_CONF1 + 4 * ch) : (1u << 17);
    double apb = 80e6;
    if (g_soc)
    {
        uint32_t sel = *(const uint32_t*)(g_soc + OFS_RTC_SOC_CLK);
        apb = sel == 1 ? *(const uint32_t*)(g_soc + OFS_RTC_PLL_APB) : *(const uint32_t*)(g_soc + OFS_RTC_XTAL_APB);
    }
    uint32_t div = conf0 & 0xFF;
    if (!div) div = 256;
    double clk = (conf1 & (1u << 17)) ? apb : 1e6;
    return (uint32_t)(div * 1e12 / clk + 0.5);
}

static void cb_rmt_event(const uint8_t ch, const uint32_t conf0, const uint32_t item)
{
    if (ch > 7) return;
    int pin = outPinFor(87 + ch);
    if (!pin) return;
    uint32_t conf1 = g_soc ? *(const uint32_t*)(g_soc + OFS_RMT + OFS_RMT_CONF1 + 4 * ch) : (1u << 17);
    uint32_t div = conf0 & 0xFF;
    if (!div) div = 256;
    uint32_t tickPs = rmtTickPs(ch, conf0);
    uint32_t idle = (conf1 & (1u << 19)) ? ((conf1 >> 18) & 1u) : 0u;

    static int logged;
    if (logged < 3) { logged++; elog("RMT ch%d item 0x%08X: div %u, %s, tick %u ps, idle %u", ch, item, div,
                                    (conf1 & (1u << 17)) ? "APB" : "REF_TICK", tickPs, idle); }
    // items per frame (a frame = items with less than 1 ms between them), first 12 frames
    static int64_t lastNs[8];
    static uint32_t count[8];
    static int frames;
    int64_t t = now_ns();
    if (count[ch] && t - lastNs[ch] > 1000000)
    {
        if (frames < 12) { frames++; elog("RMT ch%d frame: %u items", ch, count[ch]); }
        count[ch] = 0;
    }
    count[ch]++;
    lastNs[ch] = t;
    E32Event e;
    memset(&e, 0, sizeof(e));
    e.ns = now_ns();
    e.type = E32_EV_RMT;
    e.a = (uint16_t)pin;
    e.b = item;
    e.c = tickPs;
    e.reserved = idle;
    EnterCriticalSection(&g_pushLock);
    while (!e32_push(&g_sh->toModel, e))
    {
        if (g_sh->quit) break;
        SetEvent(g_reachedEvt);
        Sleep(0);
    }
    LeaveCriticalSection(&g_pushLock);
}

// ---------------------------------------------------------------------------
//  RMT receive. QEMU's RMT has no RX at all (rx_en is only stored, the
//  channel status registers read 0), so the engine does it:
//   - rx_en (conf1 bit 1) on -> input pin = GPIO matrix signal 83 + ch;
//     the model is asked for the exact time of every edge on that pin
//   - edges -> pulses (level, duration in ticks), two pulses per 32-bit item
//   - no edge for longer than idle_thres (conf0 bits 8-23) ticks -> frame end:
//     items into QEMU's RMT RAM (ch * 64 ..), an end pulse of duration 0,
//     status_ch[ch] = ch * 64 + items (the driver reads this, bits 0-9),
//     INT_RAW rx_end bit (ch * 3 + 1) and the RMT interrupt
//   Edges come from the model up to 2 slices late, so a frame only ends when
//   Proteus time (grant - 2 slices) is past the idle threshold.
// ---------------------------------------------------------------------------
enum { OFS_RMT_CONF0 = 0x458, OFS_RMT_INT_RAW = 0x498, OFS_RMT_INT_EN = 0x49C, OFS_RMT_DATA = 0x4CC, OFS_RMT_IRQ = 0x440 };

struct RmtRx
{
    int      pin = 0;          // board pin the model reports edges for (0 = none)
    bool     on = false;       // rx_en
    bool     active = false;   // a frame is being received
    int      level = 0;
    int64_t  lastNs = 0;
    std::vector<uint16_t> pulses;
};
static RmtRx    g_rmtRx[8];
static uint32_t g_rmtStatus[8];
static QemuMemoryRegionOps     g_rmtStOps;
static QemuMemoryRegionStorage g_rmtStMr[2];

static uint32_t& rmtReg(uint32_t ofs) { return *(uint32_t*)(g_soc + OFS_RMT + ofs); }

static uint64_t rmtst_read(void*, uint64_t addr, unsigned) { return g_rmtStatus[(addr >> 2) & 7]; }
static void rmtst_write(void*, uint64_t, uint64_t, unsigned) {}

// Exact edges of a pin (E32_EV_EDGE) are asked from the model with E32_EV_RMT_RXPIN.
// RMT receive and PCNT may watch the same pin: count the users.
static uint8_t g_edgeWatch[E32LINK_MAX_PINS];

static void watchPin(int pin, bool on)
{
    if (pin <= 0 || pin >= E32LINK_MAX_PINS) return;
    if (on && g_edgeWatch[pin]++ == 0) toModel(E32_EV_RMT_RXPIN, (uint16_t)pin, 1);
    if (!on && g_edgeWatch[pin] && --g_edgeWatch[pin] == 0) toModel(E32_EV_RMT_RXPIN, (uint16_t)pin, 0);
}

static void rmtRxReset()
{
    for (int ch = 0; ch < 8; ch++) { g_rmtRx[ch] = RmtRx(); g_rmtStatus[ch] = (uint32_t)ch * 64; }
    memset(g_edgeWatch, 0, sizeof(g_edgeWatch));   // the model drops all edge pins at a reset (RMT and PCNT)
}

// ---------------------------------------------------------------------------
//  UART wake-up from light sleep (trigger bits 6 UART0 / 7 UART1; UART2 cannot wake the chip).
//  Real chip: it wakes when RXD had (UART_SLEEP_CONF active_threshold + 2) rising edges
//  (IDF uart_ll_set_wakeup_edge_thrd); the characters up to the one that woke it are lost.
//  QEMU keeps SLEEP_CONF (0x38) in its reg[] (ESPUARTState +0x51c): the threshold is read there.
//  The edges are counted from the received bytes (the model decodes RX on any pin): an 8N1 frame
//  is 0, d0..d7, 1, so its rising edges follow from the byte. Bytes received in the sleep are
//  lost, up to and including the one that reaches the threshold (the wake-up comes at the end
//  of that character instead of at its edge: < 0.1 ms later at 115200).
// ---------------------------------------------------------------------------
enum { OFS_UART_SLEEP_CONF = 0x51c + 0x38 };

struct UartWake
{
    bool armed = false;               // light sleep with this wake-up: counting
    bool reached = false;             // enough edges: wake up
    int  edges = 0;                   // rising edges since the sleep start
};
static UartWake g_uartWake[2];

static int uartWakeThreshold(int u)
{
    uint32_t v = *(const uint32_t*)(g_soc + OFS_UART0 + u * UART_STRIDE + OFS_UART_SLEEP_CONF);
    return (int)(v & 0x3FF) + 2;
}

static void uartWakeStart(uint32_t ena)
{
    if (!g_soc) return;
    for (int u = 0; u < 2; u++)
    {
        if (!(ena & (0x40u << u))) continue;
        g_uartWake[u] = UartWake();
        g_uartWake[u].armed = true;
        elog("UART%d wake-up: after %d rising edges on RX", u, uartWakeThreshold(u));
    }
}

// rising edges of one 8N1 frame: start 0, data bits LSB first, stop 1
static int frameRisingEdges(uint8_t b)
{
    int n = 0, prev = 0;
    for (int k = 0; k <= 8; k++)
    {
        int bit = k < 8 ? (b >> k) & 1 : 1;
        if (bit && !prev) n++;
        prev = bit;
    }
    return n;
}

// a received byte: true = lost (received while the chip slept)
static bool uartWakeDrop(int u, uint8_t b)
{
    if (u > 1) return false;
    UartWake& W = g_uartWake[u];
    if (!W.armed || W.reached) return false;
    W.edges += frameRisingEdges(b);
    if (W.edges >= uartWakeThreshold(u)) W.reached = true;
    static int logged;
    if (logged < 20) { logged++; elog("UART%d RX byte 0x%02X lost (received in light sleep)", u, b); }
    return true;
}

// wake-up causes (bits 6 / 7) of the UARTs that saw enough edges
static uint32_t uartWakeCheck()
{
    uint32_t cause = 0;
    for (int u = 0; u < 2; u++)
        if (g_uartWake[u].armed && g_uartWake[u].reached) cause |= 0x40u << u;
    return cause;
}

// light sleep over (any cause): bytes are received again
static void uartWakeStop()
{
    for (int u = 0; u < 2; u++)
    {
        UartWake& W = g_uartWake[u];
        if (!W.armed) continue;
        W.armed = false;
        elog("UART%d wake-up: %d rising edges in the sleep", u, W.edges);
    }
}

static void uartWakeReset()
{
    for (int u = 0; u < 2; u++) g_uartWake[u] = UartWake();
}

static void installRmtRx()
{
    memset(&g_rmtStOps, 0, sizeof(g_rmtStOps));
    g_rmtStOps.read = rmtst_read;
    g_rmtStOps.write = rmtst_write;
    g_rmtStOps.endianness = 2;
    g_rmtStOps.valid_min_access_size = 1;
    g_rmtStOps.valid_max_access_size = 4;
    g_rmtStOps.impl_min_access_size = 4;
    g_rmtStOps.impl_max_access_size = 4;
    rmtRxReset();
    void* sys = q_get_system_memory();
    const uint64_t base[2] = { 0x3FF56060ull, 0x60016060ull };      // RMT status_ch[0..7]
    for (int k = 0; k < 2; k++)
    {
        memset(&g_rmtStMr[k], 0, sizeof(g_rmtStMr[k]));
        q_mr_init_io(&g_rmtStMr[k], nullptr, &g_rmtStOps, nullptr, k ? "esp32link.rmtst.alias" : "esp32link.rmtst", 0x20);
        q_mr_add_overlap(sys, base[k], &g_rmtStMr[k], 10);
    }
    elog("RMT receive status registers installed");
}

// an edge on a board pin at engine time ns (from the model)
static void rmtRxEdge(int pin, int level, int64_t ns)
{
    if (!g_soc) return;
    for (int ch = 0; ch < 8; ch++)
    {
        RmtRx& R = g_rmtRx[ch];
        if (!R.on || R.pin != pin) continue;
        if (R.active)
        {
            if (level == R.level) continue;
            double tick = rmtTickPs(ch, rmtReg(OFS_RMT_CONF0 + 4 * ch)) / 1000.0;
            int64_t d = (int64_t)((double)(ns - R.lastNs) / tick + 0.5);
            if (d < 1) d = 1;
            if (d > 32767) d = 32767;
            R.pulses.push_back((uint16_t)(d | (R.level ? 0x8000 : 0)));
        }
        R.active = true;
        R.level = level;
        R.lastNs = ns;
    }
}

static void rmtRxFinish(int ch)
{
    RmtRx& R = g_rmtRx[ch];
    R.pulses.push_back((uint16_t)(R.level ? 0x8000 : 0));          // end marker: duration 0
    uint32_t conf0 = rmtReg(OFS_RMT_CONF0 + 4 * ch);
    uint32_t blocks = (conf0 >> 24) & 0xF;
    if (!blocks) blocks = 1;
    size_t maxItems = (size_t)blocks * 64;
    if (ch * 64 + maxItems > 512) maxItems = 512 - (size_t)ch * 64;
    size_t items = (R.pulses.size() + 1) / 2;
    if (items > maxItems) items = maxItems;
    uint32_t* mem = (uint32_t*)(g_soc + OFS_RMT + OFS_RMT_DATA);
    for (size_t k = 0; k < items; k++)
    {
        uint32_t lo = R.pulses[2 * k];
        uint32_t hi = 2 * k + 1 < R.pulses.size() ? R.pulses[2 * k + 1] : 0;
        mem[ch * 64 + k] = lo | (hi << 16);
    }
    g_rmtStatus[ch] = (uint32_t)(ch * 64 + items);
    static int logged;
    if (logged < 10) { logged++; elog("RMT RX ch%d frame: %u pulses, %u items", ch, (unsigned)R.pulses.size(), (unsigned)items); }
    R.active = false;
    R.pulses.clear();
    rmtReg(OFS_RMT_INT_RAW) |= 1u << (ch * 3 + 1);                    // rx_end
    if (rmtReg(OFS_RMT_INT_RAW) & rmtReg(OFS_RMT_INT_EN))
        q_set_irq(*(void**)(g_soc + OFS_RMT + OFS_RMT_IRQ), 1);
}

// every slice: follow rx_en / routing, end frames after the idle threshold
static void updateRmtRx()
{
    if (!g_soc || !g_inSel) return;
    const int64_t safeNs = e32_load64(&g_sh->grantNs) - 2 * g_sliceNs;    // Proteus has reported all edges before this
    for (int ch = 0; ch < 8; ch++)
    {
        RmtRx& R = g_rmtRx[ch];
        bool en = (rmtReg(OFS_RMT_CONF1 + 4 * ch) & 2u) != 0;
        int pin = en ? inPinFor(83 + ch) : 0;
        if (en != R.on || pin != R.pin)
        {
            if (R.pin && R.pin != pin) watchPin(R.pin, false);
            if (pin && pin != R.pin) watchPin(pin, true);
            if (en && !R.on) g_rmtStatus[ch] = (uint32_t)ch * 64;      // new receive: write pointer back to the start
            if (en != R.on || pin != R.pin) elog("RMT RX ch%d %s, pin (board pin) %d", ch, en ? "on" : "off", pin);
            R.on = en;
            R.pin = pin;
            R.active = false;
            R.pulses.clear();
        }
        if (!R.active) continue;
        uint32_t conf0 = rmtReg(OFS_RMT_CONF0 + 4 * ch);
        double idleNs = (double)((conf0 >> 8) & 0xFFFF) * rmtTickPs(ch, conf0) / 1000.0;
        if ((double)(safeNs - R.lastNs) > idleNs) rmtRxFinish(ch);
    }
}

// ---------------------------------------------------------------------------
//  CAN (TWAI). QEMU has the controller: esp32_twai is an SJA1000 core
//  (can_sja1000.c) on a QEMU "can-bus" object. It sends and receives whole
//  frames on that bus, never bits on a pin. PICSimLab's SoC never creates it,
//  so the engine does (installCan): QEMU gets "-object can-bus,id=canbus0",
//  the TWAI is linked to it, realized and mapped, and the engine joins the
//  bus as a second client:
//   - frame sent by the firmware -> E32_EV_CAN_TX; the model makes the bits on
//     the TX pin (GPIO matrix signal 123 TWAI_TX)
//   - frame the model read from the RX pin (signal 94 TWAI_RX) -> E32_EV_CAN_RX
//     -> can_bus_client_send -> the controller (its acceptance filter applies)
//  Two things QEMU's SJA1000 lacks, done with own registers (priority 10):
//   - BTR0 / BTR1 (0x18 / 0x1C) are not stored at all: the driver writes them
//     field by field (read-modify-write), and the bit time comes from them
//   - CMR (0x04) only knows "transmit" (bit 0). "Self reception" (bit 4, what
//     twai_transmit sends for a self_rx frame) becomes transmit, and the frame
//     is looped back to the controller like on the real chip.
//   - CMR "release receive buffer" skips too few bytes after a remote frame
//     (QEMU stores its DLC data bytes anyway); the engine skips the rest.
//   - QEMU completes a transmit at once. The engine keeps the transmit buffer
//     busy until the model reports the bus result (E32_EV_CAN_DONE): sent ->
//     TCS + TI; arbitration lost -> ALI + ALC (the model sends it again);
//     error -> BEI + ECC, TEC / REC as CAN counts them, error warning /
//     passive (EI, EPI), bus off (TEC > 255: reset mode) and its recovery
//     (128 x 11 bits after the firmware leaves reset mode). Own registers
//     ALC, ECC, EWL, RXERR, TXERR (0x2C..0x3C).
// ---------------------------------------------------------------------------
enum { OFS_TWAI = 0x96f0, OFS_TWAI_IOMEM = 0x330, OFS_TWAI_OPS = 0x440, OFS_TWAI_SJA = 0x490, OFS_TWAI_CANBUS = 0x588,
       OFS_INTMATRIX = 0x8d70, ETS_TWAI_INTR_SOURCE = 45 };
enum { SJA_MODE = 0x00, SJA_STATUS = 0x01, SJA_INT = 0x02, SJA_INT_EN = 0x03, SJA_RXMSG_CNT = 0x04, SJA_RXBUF_START = 0x05,
       SJA_RX_BUFF = 0x1c, SJA_RX_CNT = 0x60, SJA_IRQ = 0x90 };   // CanSJA1000State
enum { TWAI_SIG_RX = 94, TWAI_SIG_TX = 123 };

struct QemuCanFrame { uint32_t can_id; uint8_t can_dlc; uint8_t flags; uint8_t pad[2]; uint8_t data[64]; };   // qemu_can_frame (0x48)
struct QemuCanClientInfo { bool (*can_receive)(void*); intptr_t (*receive)(void*, const QemuCanFrame*, size_t); };
typedef int (*can_bus_insert_client_fn)(void* bus, void* client);
typedef intptr_t (*can_bus_client_send_fn)(void* client, const QemuCanFrame* frames, size_t count);
typedef void (*can_sja_mem_write_fn)(void* sja, uint64_t addr, uint64_t val, unsigned size);
typedef void* (*object_resolve_path_fn)(const char* path, bool* ambiguous);

static can_bus_client_send_fn  q_can_send;
static can_sja_mem_write_fn    q_sja_write;
static bool                    g_canOk = false;
alignas(8) static uint8_t      g_canClient[0x50];      // CanBusClientState: info, bus, ...
static QemuCanClientInfo       g_canInfo;
static uint32_t                g_btr[2];               // BTR0, BTR1
static bool                    g_canSelf = false;      // inside a self-reception transmit
static bool                    g_canHaveLast = false;
static QemuCanFrame            g_canLast;              // the self-reception frame, looped back when it is sent
// transmit buffer: QEMU says "sent" at once; the engine keeps it busy until the model reports the bus result
static bool                    g_txPending = false, g_txSingle = false, g_txSelf = false;
static uint8_t                 g_txSeq = 0;
// error counters and status (QEMU has none): ALC, ECC, EWL, RXERR, TXERR registers
static uint32_t                g_alc, g_ecc, g_ewl = 96, g_rec, g_tec;
static bool                    g_busOff = false, g_recovering = false, g_passive = false;
static int64_t                 g_recoverMidNs, g_recoverEndNs;
struct CanConf { int tx, rx, mode, state; uint32_t bitPs, sp; };
static CanConf                 g_canSent;              // last E32_EV_CAN_CONF (all 0 = TWAI off)
static QemuMemoryRegionOps     g_twaiOps;
static QemuMemoryRegionStorage g_twaiMr[3];

static uint8_t* twaiSja() { return (uint8_t*)g_soc + OFS_TWAI + OFS_TWAI_SJA; }

static void canPush(uint16_t type, uint16_t a, uint32_t b, uint32_t c = 0, uint32_t r = 0)
{
    E32Event e;
    memset(&e, 0, sizeof(e));
    e.ns = now_ns();
    e.type = type;
    e.a = a;
    e.b = b;
    e.c = c;
    e.reserved = r;
    EnterCriticalSection(&g_pushLock);
    while (!e32_push(&g_sh->toModel, e)) { if (g_sh->quit) break; SetEvent(g_reachedEvt); Sleep(0); }
    LeaveCriticalSection(&g_pushLock);
}

// interrupt line = pending & enabled interrupts (as QEMU's SJA1000 does it)
static void twaiIrq()
{
    uint8_t* s = twaiSja();
    q_set_irq(*(void**)(s + SJA_IRQ), (s[SJA_INT] & s[SJA_INT_EN]) ? 1 : 0);
}

static void twaiInt(uint8_t bits)
{
    twaiSja()[SJA_INT] |= bits;
    twaiIrq();
}

// bit time in ps: tq = 2 * (BRP + 1) APB clocks (x2 with BRP_DIV, interrupt enable bit 4),
// bit = 1 + (TSEG1 + 1) + (TSEG2 + 1) tq; sample point after 1 + TSEG1 + 1 tq
static uint32_t canBitPs(uint32_t* sp)
{
    double apb = 80e6;
    uint32_t sel = *(const uint32_t*)(g_soc + OFS_RTC_SOC_CLK);
    if (sel == 1) apb = *(const uint32_t*)(g_soc + OFS_RTC_PLL_APB);
    uint32_t brp = 2 * ((g_btr[0] & 0x3F) + 1);
    if (twaiSja()[SJA_INT_EN] & 0x10) brp *= 2;
    uint32_t tseg1 = (g_btr[1] & 0xF) + 1, tseg2 = ((g_btr[1] >> 4) & 7) + 1;
    uint32_t tq = 1 + tseg1 + tseg2;
    *sp = 1000 * (1 + tseg1) / tq;
    return (uint32_t)(brp * tq * 1e12 / apb + 0.5);
}

// the transmit buffer is free again: TBS, sent -> TCS (transmission complete), TI interrupt
static void canTxFinish(bool sent, bool interrupt)
{
    uint8_t* s = twaiSja();
    g_txPending = false;
    s[SJA_STATUS] = (uint8_t)((s[SJA_STATUS] & ~0x28) | 0x04 | (sent ? 0x08 : 0));
    if (interrupt) twaiInt(0x02);
}

// TEC / REC changed: error warning (ES), error passive, bus off; EI / EPI interrupts on a change
static void canErrorState()
{
    uint8_t* s = twaiSja();
    uint8_t ints = 0;
    if (!g_busOff && g_tec >= 256)
    {
        // bus off: the controller stops (reset mode), the transmission is cancelled
        g_busOff = true;
        g_tec = 127;
        g_rec = 0;
        s[SJA_MODE] |= 0x01;
        if (g_txPending) { canTxFinish(false, false); canPush(E32_EV_CAN_ABORT, 0, 0); }
        elog("TWAI: bus off");
    }
    bool es = g_tec >= g_ewl || g_rec >= g_ewl;                     // bus off: TEC 127 counts down -> ES 0 = recovery runs
    bool passive = g_busOff ? g_passive : (g_tec >= 128 || g_rec >= 128);   // EPI only between passive and active
    uint8_t st = (uint8_t)((s[SJA_STATUS] & 0x3F) | (es ? 0x40 : 0) | (g_busOff ? 0x80 : 0));
    if ((st ^ s[SJA_STATUS]) & 0xC0) ints |= 0x04;                 // EI: ES or BS changed
    if (passive != g_passive) ints |= 0x20;                         // EPI
    g_passive = passive;
    s[SJA_STATUS] = st;
    if (ints) twaiInt(ints);
}

// pins, mode, error state and bit time -> the model when they change; bus-off recovery.
// Every slice, and before a frame.
static void updateCan()
{
    if (!g_canOk) return;
    uint8_t* s = twaiSja();
    if (g_busOff && !g_recovering && !(s[SJA_MODE] & 0x01))
    {
        // the firmware left reset mode while bus off: recovery = 128 x 11 recessive bits
        uint32_t sp = 0;
        int64_t bitNs = canBitPs(&sp) / 1000;
        g_recovering = true;
        g_tec = 127;
        g_recoverMidNs = now_ns() + 64 * 11 * bitNs;
        g_recoverEndNs = now_ns() + 128 * 11 * bitNs;
        elog("TWAI: bus-off recovery started");
    }
    if (g_recovering && g_tec >= g_ewl && now_ns() >= g_recoverMidNs)
    {
        g_tec = g_ewl ? g_ewl - 1 : 0;                              // TEC counts down: below the warning limit
        canErrorState();
    }
    if (g_recovering && now_ns() >= g_recoverEndNs)
    {
        g_recovering = g_busOff = false;
        g_tec = g_rec = 0;
        canErrorState();
        elog("TWAI: bus-off recovery complete");
    }
    if ((s[SJA_MODE] & 0x01) && g_txPending)                        // reset mode cancels a transmission
    {
        canTxFinish(false, false);
        canPush(E32_EV_CAN_ABORT, 0, 0);
    }

    CanConf now = {};
    now.tx = outPinFor(TWAI_SIG_TX);
    now.rx = inPinFor(TWAI_SIG_RX);
    if (now.tx || now.rx)
    {
        now.mode = s[SJA_MODE] & 0x1F;
        now.state = g_busOff ? 2 : (g_passive ? 1 : 0);
        now.bitPs = canBitPs(&now.sp);
    }
    if (memcmp(&now, &g_canSent, sizeof(now)) == 0) return;
    g_canSent = now;
    canPush(E32_EV_CAN_CONF, (uint16_t)now.tx, (uint32_t)(now.rx & 0xFF) | ((uint32_t)now.mode << 8) | ((uint32_t)now.state << 16),
            now.bitPs, now.sp);
    static int logged;
    if (logged < 20) { logged++; elog("TWAI: TX pin %d, RX pin %d, mode 0x%02X, state %d, %.0f bit/s, sample point %u.%u %%",
                                      now.tx, now.rx, now.mode, now.state, now.bitPs ? 1e12 / now.bitPs : 0.0, now.sp / 10, now.sp % 10); }
}

static bool can_client_can_receive(void*) { return true; }

// the controller sent a frame (vCPU thread, inside the CMR write)
static intptr_t can_client_receive(void*, const QemuCanFrame* f, size_t n)
{
    updateCan();
    for (size_t k = 0; k < n; k++)
    {
        uint32_t dlc = f[k].can_dlc > 8 ? 8 : f[k].can_dlc;
        canPush(E32_EV_CAN_TX, (uint16_t)(dlc | (g_txSingle ? 0x10 : 0) | ((uint32_t)g_txSeq << 8)),
                f[k].can_id & 0xDFFFFFFFu,                      // EFF + RTR flags, id (no error flag)
                *(const uint32_t*)f[k].data, *(const uint32_t*)(f[k].data + 4));
        static int logged;
        if (logged < 10) { logged++; elog("TWAI TX frame id 0x%X%s%s dlc %u", f[k].can_id & 0x1FFFFFFFu,
                                          (f[k].can_id & 0x80000000u) ? " (ext)" : "", (f[k].can_id & 0x40000000u) ? " RTR" : "", dlc); }
        if (g_canSelf) { g_canLast = f[k]; g_canHaveLast = true; }
    }
    return (intptr_t)n;
}

// frame read from the RX pin by the model (main loop thread)
static void canReceive(const E32Event& e)
{
    if (!g_canOk) return;
    QemuCanFrame f;
    memset(&f, 0, sizeof(f));
    f.can_id = e.b & 0xDFFFFFFFu;
    f.can_dlc = (uint8_t)((e.a & 0x0F) > 8 ? 8 : (e.a & 0x0F));
    memcpy(f.data, &e.c, 4);
    memcpy(f.data + 4, &e.reserved, 4);
    q_can_send(g_canClient, &f, 1);
    if (g_rec > 127) g_rec = 120;                                   // received OK: REC goes down
    else if (g_rec) g_rec--;
    canErrorState();
    static int logged;
    if (logged < 10) { logged++; elog("TWAI RX frame id 0x%X%s%s dlc %u -> controller", f.can_id & 0x1FFFFFFFu,
                                      (f.can_id & 0x80000000u) ? " (ext)" : "", (f.can_id & 0x40000000u) ? " RTR" : "", f.can_dlc); }
}

// bus result from the model (main loop thread)
static void canDone(const E32Event& e)
{
    if (!g_canOk) return;
    const uint32_t kind = e.a, seq = e.b & 0xFF, info = (e.b >> 8) & 0xFF;
    const bool mine = g_txPending && seq == g_txSeq;
    static int logged;
    if (kind && logged < 20) { logged++; elog("TWAI bus: %s (code 0x%02X), TEC %u, REC %u", kind == 1 ? "arbitration lost" :
                                              kind == 2 ? "send error" : kind == 3 ? "receive error" : "frame given up", info, g_tec, g_rec); }
    switch (kind)
    {
    case 0:                                                         // sent, ACK seen
        if (!mine) break;
        canTxFinish(true, true);
        if (g_tec) g_tec--;
        canErrorState();
        if (g_txSelf && g_canHaveLast) q_can_send(g_canClient, &g_canLast, 1);   // self reception
        g_txSelf = g_canHaveLast = false;
        break;
    case 1:                                                         // arbitration lost: the model sends it again later
        if (!mine) break;
        g_alc = info;
        twaiInt(0x40);                                              // ALI
        break;
    case 2:                                                         // error while sending
        g_ecc = info;
        twaiInt(0x80);                                              // BEI
        if (!((info & 0x1F) == 0x19 && g_passive)) g_tec += 8;      // an ACK error does not count while error passive
        canErrorState();
        break;
    case 3:                                                         // error while receiving
        g_ecc = info;
        twaiInt(0x80);
        g_rec += 1;
        canErrorState();
        break;
    case 4:                                                         // single shot: not sent
        if (mine) canTxFinish(false, true);
        break;
    }
}

// own registers (opaque): 0 CMR 0x04, 1 BTR0 / BTR1 0x18 / 0x1C, 2 ALC ECC EWL RXERR TXERR 0x2C..0x3C
static uint64_t twai_read(void* opaque, uint64_t addr, unsigned)
{
    switch ((intptr_t)opaque)
    {
    case 1: return g_btr[(addr >> 2) & 1];
    case 2:
        switch (addr >> 2)
        {
        case 0: return g_alc;
        case 1: return g_ecc;
        case 2: return g_ewl;
        case 3: return g_rec > 255 ? 255 : g_rec;
        default: return g_tec > 255 ? 255 : g_tec;
        }
    default: return 0;                                              // CMR is write only
    }
}

static void twai_write(void* opaque, uint64_t addr, uint64_t data, unsigned)
{
    const uint32_t d = (uint32_t)data & 0xFF;
    if ((intptr_t)opaque == 1) { g_btr[(addr >> 2) & 1] = d; return; }
    if ((intptr_t)opaque == 2)
    {
        switch (addr >> 2)
        {
        case 2: g_ewl = d; break;
        case 3: g_rec = d; break;
        case 4: g_tec = d; break;
        default: break;                                             // ALC / ECC: read only
        }
        return;
    }

    // CMR: bit 0 transmit, 1 abort, 2 release receive buffer, 3 clear overrun, 4 self reception
    uint8_t* sja = twaiSja();
    uint32_t v = d;
    const bool self = (v & 0x10) != 0;
    const bool tr = (v & 0x01) || self;
    if (tr && (g_txPending || (sja[SJA_MODE] & 0x01))) v &= ~0x13u;   // buffer busy / reset mode: no transmit
    else if (tr)
    {
        g_txSeq++;
        g_txSingle = (v & 0x02) != 0;                               // transmit + abort = single shot
        g_txSelf = self;
        g_canSelf = self;
        g_canHaveLast = false;
        v = (v & ~0x12u) | 0x01u;
    }
    else if ((v & 0x02) && g_txPending)                             // abort: frames not on the bus yet are dropped
    {
        canTxFinish(false, true);
        canPush(E32_EV_CAN_ABORT, 0, 0);
    }
    v &= ~0x02u;
    // QEMU bug: a received remote frame is stored with DLC data bytes (3 + DLC, 5 + DLC extended),
    // but "release receive buffer" (bit 2) skips only 3 / 5 bytes for it. Every later frame is then
    // read from the wrong place, and the counters go wrong until the firmware hangs. Skip the rest.
    uint32_t extra = 0;
    if ((v & 0x04) && sja[SJA_RXMSG_CNT])
    {
        uint8_t info = sja[SJA_RX_BUFF + sja[SJA_RXBUF_START]];
        if (info & 0x40) extra = (info & 0x0F) > 8 ? 8 : (info & 0x0F);
    }
    q_sja_write(sja, 1, v, 1);
    if (extra)
    {
        sja[SJA_RXBUF_START] = (uint8_t)((sja[SJA_RXBUF_START] + extra) % 64);
        *(uint32_t*)(sja + SJA_RX_CNT) -= extra;
    }
    g_canSelf = false;
    if (v & 0x01)
    {
        // QEMU sent the frame (now with the model) and said "complete": the buffer stays busy
        // (TBS 0, TCS 0, TS 1, no TI) until the model reports the result (canDone)
        g_txPending = true;
        sja[SJA_STATUS] = (uint8_t)((sja[SJA_STATUS] & ~0x0C) | 0x20);
        sja[SJA_INT] &= ~0x02;
        twaiIrq();
    }
}

static void canReset()
{
    g_btr[0] = g_btr[1] = 0;
    memset(&g_canSent, 0, sizeof(g_canSent));           // the model released the CAN pins
    g_canSelf = g_canHaveLast = false;
    g_txPending = g_txSingle = g_txSelf = false;
    g_alc = g_ecc = g_rec = g_tec = 0;
    g_ewl = 96;
    g_busOff = g_recovering = g_passive = false;
}

// ===========================================================================
//  PCNT (pulse counter). Not in QEMU at all: the engine is the whole block,
//  registers at 0x3FF57000 (and 0x60017000), interrupt = matrix source 48.
//    0x00 + 12u CONF0  filter thres 0-9 (APB cycles), filter en 10, event enables
//                      11 zero, 12 h_lim, 13 l_lim, 14 thres0, 15 thres1;
//                      channel c (bit 16 + 8c): neg mode, pos mode, hctrl mode, lctrl mode
//                      (2 bits each; edge: 1 +1, 2 -1, else hold; level: 0 keep, 1 invert, else hold)
//    0x04 + 12u CONF1  thres0 0-15, thres1 16-31      0x08 + 12u CONF2  h_lim 0-15, l_lim 16-31
//    0x60 + 4u  CNT    16-bit count                    0x80 / 84 / 88 / 8C INT_RAW / ST / ENA / CLR (bit = unit)
//    0x90 + 4u  STATUS bits 0-1 zero mode, 2 thres1, 3 thres0, 4 l_lim, 5 h_lim, 6 zero
//    0xB0 CTRL  unit u: bit 2u reset (count 0 while set), bit 2u+1 pause    0xFC DATE
//  Inputs: GPIO matrix in signals (sig ch0, sig ch1, ctrl ch0, ctrl ch1) 39-42 + 4u for units 0-4,
//  71-74 + 4(u-5) for units 5-7. A routed pin is watched: the model sends every edge with its
//  exact time (E32_EV_EDGE). Edges are counted at the slice boundary once Proteus has
//  reported everything before them (grant - 2 slices), so the count is <= 2 slices late.
//  Glitch filter: a level must stay for thres x 12.5 ns, shorter pulses are dropped.
//  Counting at a limit: event + count back to 0 (like the chip).
// ===========================================================================
enum { PCNT_SRC = 48 };
struct PcntIn { int pin = 0; int fixed = 0; bool inv = false; int64_t sinceNs = 0; };   // pin 0: constant 'fixed'
struct PcntUnit
{
    int16_t cnt = 0;
    PcntIn src[4];                  // 0 sig ch0, 1 ctrl ch0, 2 sig ch1, 3 ctrl ch1
    uint8_t lvl[4] = {};            // filtered input levels
    bool pendOn[4] = {};            // a change waiting for the filter
    uint8_t pendLvl[4] = {};
    int64_t pendNs[4] = {};
    uint32_t status = 0;
};
struct PcntEdge { int pin, level; int64_t ns; };
static uint32_t              g_pcnt[0x100 / 4];
static PcntUnit              g_pcntU[8];
static std::vector<PcntEdge> g_pcntEdges;
static int8_t                g_pcntPinLvl[E32LINK_MAX_PINS];   // pin level as far as edges are counted
static void*                 g_pcntIrq = nullptr;
static QemuMemoryRegionOps     g_pcntOps;
static QemuMemoryRegionStorage g_pcntMr[2];

static int pcntSig(int u, int k)     // GPIO matrix input signal of unit u, input k
{
    static const int ORDER[4] = { 0, 2, 1, 3 };   // signal order: sig ch0, sig ch1, ctrl ch0, ctrl ch1
    return (u < 5 ? 39 + 4 * u : 71 + 4 * (u - 5)) + ORDER[k];
}

static void pcntUpdateIrq()
{
    if (g_pcntIrq) q_set_irq(g_pcntIrq, (g_pcnt[0x80 / 4] & g_pcnt[0x88 / 4]) ? 1 : 0);
}

static void pcntReset()
{
    memset(g_pcnt, 0, sizeof(g_pcnt));
    for (int u = 0; u < 8; u++) { g_pcnt[3 * u] = 0x3C10; g_pcntU[u] = PcntUnit(); }
    g_pcnt[0xB0 / 4] = 0x5555;                         // every counter held at 0
    g_pcntEdges.clear();
    memset(g_pcntPinLvl, -1, sizeof(g_pcntPinLvl));   // watched pins: the model forgets them at a reset
    pcntUpdateIrq();
}

// one count step (+1 / -1) of unit u: limits, thresholds, zero, interrupt
static void pcntStep(int u, int dir)
{
    PcntUnit& U = g_pcntU[u];
    uint32_t conf0 = g_pcnt[3 * u], conf1 = g_pcnt[3 * u + 1], conf2 = g_pcnt[3 * u + 2];
    int16_t hl = (int16_t)(conf2 & 0xFFFF), ll = (int16_t)(conf2 >> 16);
    int16_t t0 = (int16_t)(conf1 & 0xFFFF), t1 = (int16_t)(conf1 >> 16);
    int c = U.cnt + dir;
    uint32_t ev = 0;                                   // status bits 2-6
    if (dir > 0 && hl > 0 && c >= hl) { if (conf0 & (1u << 12)) ev |= 1u << 5; c = 0; }
    else if (dir < 0 && ll < 0 && c <= ll) { if (conf0 & (1u << 13)) ev |= 1u << 4; c = 0; }
    else
    {
        if (c == 0 && (conf0 & (1u << 11))) ev |= 1u << 6;
        if (c == t0 && (conf0 & (1u << 14))) ev |= 1u << 3;
        if (c == t1 && (conf0 & (1u << 15))) ev |= 1u << 2;
    }
    if (c < -32768) c += 65536;                        // 16-bit counter
    if (c > 32767) c -= 65536;
    int16_t before = U.cnt;
    U.cnt = (int16_t)c;
    if (!ev) return;
    uint32_t& raw = g_pcnt[0x80 / 4];
    if (!(raw & (1u << u))) U.status = 0;             // interrupt was cleared: a new event set
    U.status |= ev;
    if (ev & (1u << 6)) U.status = (U.status & ~3u) | (before > 0 ? 0u : 1u);   // +N -> 0 / -N -> 0
    raw |= 1u << u;
    pcntUpdateIrq();
}

// a filtered input of unit u changes to 'level'
static void pcntApply(int u, int k, int level)
{
    PcntUnit& U = g_pcntU[u];
    U.lvl[k] = (uint8_t)level;
    if (k & 1) return;                                 // control inputs only change the mode
    int ch = k / 2;
    uint32_t m = g_pcnt[3 * u] >> (16 + 8 * ch);
    int edge = level ? (m >> 2) & 3 : m & 3;            // pos / neg mode
    int dir = edge == 1 ? 1 : edge == 2 ? -1 : 0;
    int ctrl = U.lvl[k + 1] ? (m >> 4) & 3 : (m >> 6) & 3;   // hctrl / lctrl mode
    if (ctrl == 1) dir = -dir;
    else if (ctrl >= 2) dir = 0;
    uint32_t ctl = g_pcnt[0xB0 / 4];
    if (dir && !(ctl & (3u << (2 * u)))) pcntStep(u, dir);   // not reset, not paused
}

static int64_t pcntFilterNs(int u)
{
    uint32_t conf0 = g_pcnt[3 * u];
    return (conf0 & (1u << 10)) ? (int64_t)(conf0 & 0x3FF) * 25 / 2 : 0;   // APB 80 MHz: 12.5 ns
}

// changes that passed the filter before 'ns', in time order
static void pcntCommit(int u, int64_t ns)
{
    PcntUnit& U = g_pcntU[u];
    int64_t w = pcntFilterNs(u);
    for (;;)
    {
        int best = -1;
        for (int k = 0; k < 4; k++)
            if (U.pendOn[k] && U.pendNs[k] + w <= ns && (best < 0 || U.pendNs[k] < U.pendNs[best])) best = k;
        if (best < 0) return;
        U.pendOn[best] = false;
        if (U.pendLvl[best] != U.lvl[best]) pcntApply(u, best, U.pendLvl[best]);
    }
}

// raw input change of unit u at time ns
static void pcntInput(int u, int k, int level, int64_t ns)
{
    PcntUnit& U = g_pcntU[u];
    pcntCommit(u, ns);
    int64_t w = pcntFilterNs(u);
    if (U.pendOn[k] && ns - U.pendNs[k] < w) { U.pendOn[k] = false; return; }   // pulse shorter than the filter
    if (w == 0) { if (level != U.lvl[k]) pcntApply(u, k, level); return; }
    U.pendOn[k] = true;
    U.pendLvl[k] = (uint8_t)level;
    U.pendNs[k] = ns;
}

static int pcntInLevel(const PcntIn& in)
{
    int v = in.pin ? (g_pcntPinLvl[in.pin] > 0 ? 1 : 0) : in.fixed;
    return in.inv ? !v : v;
}

static void pcntEdge(int pin, int level, int64_t ns)
{
    if (pin > 0 && pin < E32LINK_MAX_PINS) g_pcntEdges.push_back({ pin, level, ns });
}

// follow the GPIO matrix (every slice and at once when the firmware changes it,
// so the model watches a new pin before the first pulse)
static void pcntRoute()
{
    if (!g_inSel) return;
    for (int u = 0; u < 8; u++)
        for (int k = 0; k < 4; k++)
        {
            uint32_t v = g_inSel[pcntSig(u, k)];
            PcntIn in;
            if (v & 0x80)
            {
                int g = (int)(v & 0x3F);
                if (g < 40) in.pin = g_gpioToBoard[g];
                else in.fixed = g == 0x38 ? 1 : 0;     // 0x38 constant high, 0x30 constant low
                in.inv = (v & 0x40) != 0;
            }
            PcntIn& old = g_pcntU[u].src[k];
            if (in.pin == old.pin && in.fixed == old.fixed && in.inv == old.inv) continue;
            if (old.pin) watchPin(old.pin, false);
            if (in.pin)
            {
                watchPin(in.pin, true);
                if (g_pcntPinLvl[in.pin] < 0) g_pcntPinLvl[in.pin] = g_inLevel[in.pin] > 0 ? 1 : 0;
                elog("PCNT unit %d %s ch%d: board pin %d%s", u, (k & 1) ? "ctrl" : "signal", k / 2, in.pin, in.inv ? " (inverted)" : "");
            }
            in.sinceNs = now_ns();
            old = in;
            g_pcntU[u].lvl[k] = (uint8_t)pcntInLevel(in);   // new source: no edge counted
            g_pcntU[u].pendOn[k] = false;
        }
}

// every slice: count the edges Proteus has fully reported
static void updatePcnt()
{
    pcntRoute();

    const int64_t safeNs = e32_load64(&g_sh->grantNs) - 2 * g_sliceNs;   // Proteus has reported all edges before this
    std::stable_sort(g_pcntEdges.begin(), g_pcntEdges.end(), [](const PcntEdge& a, const PcntEdge& b) { return a.ns < b.ns; });
    size_t n = 0;
    for (; n < g_pcntEdges.size() && g_pcntEdges[n].ns <= safeNs; n++)
    {
        const PcntEdge& e = g_pcntEdges[n];
        if (g_pcntPinLvl[e.pin] == e.level) continue;
        g_pcntPinLvl[e.pin] = (int8_t)e.level;
        for (int u = 0; u < 8; u++)
            for (int k = 0; k < 4; k++)
                if (g_pcntU[u].src[k].pin == e.pin && e.ns >= g_pcntU[u].src[k].sinceNs)
                    pcntInput(u, k, pcntInLevel(g_pcntU[u].src[k]), e.ns);
    }
    g_pcntEdges.erase(g_pcntEdges.begin(), g_pcntEdges.begin() + n);
    for (int u = 0; u < 8; u++) pcntCommit(u, safeNs);
}

static uint64_t pcnt_read(void*, uint64_t addr, unsigned)
{
    uint32_t a = (uint32_t)addr;
    if (a >= 0x60 && a < 0x80) return (uint16_t)g_pcntU[(a - 0x60) / 4].cnt;
    if (a >= 0x90 && a < 0xB0) { const PcntUnit& U = g_pcntU[(a - 0x90) / 4]; return U.status; }
    if (a == 0x84) return g_pcnt[0x80 / 4] & g_pcnt[0x88 / 4];
    if (a == 0x8C) return 0;
    if (a == 0xFC) return 0x14122600;
    return a < sizeof(g_pcnt) ? g_pcnt[a / 4] : 0;
}

static void pcnt_write(void*, uint64_t addr, uint64_t data, unsigned)
{
    uint32_t a = (uint32_t)addr, v = (uint32_t)data;
    if (a < 0x60) { g_pcnt[a / 4] = v; return; }
    switch (a)
    {
    case 0x88: g_pcnt[0x88 / 4] = v & 0xFF; pcntUpdateIrq(); return;
    case 0x8C: g_pcnt[0x80 / 4] &= ~v; pcntUpdateIrq(); return;
    case 0xB0:
        g_pcnt[0xB0 / 4] = v;
        for (int u = 0; u < 8; u++)
            if (v & (1u << (2 * u))) g_pcntU[u].cnt = 0;    // counter reset
        return;
    default: return;                                        // counts, status, raw: read only
    }
}

static void installPcnt(HMODULE q)
{
    typedef void* (*qdev_get_gpio_in_fn)(void* dev, int n);
    qdev_get_gpio_in_fn gpioIn = (qdev_get_gpio_in_fn)GetProcAddress(q, "qdev_get_gpio_in");
    memset(&g_pcntOps, 0, sizeof(g_pcntOps));
    g_pcntOps.read = pcnt_read;
    g_pcntOps.write = pcnt_write;
    g_pcntOps.endianness = 2;
    g_pcntOps.valid_min_access_size = 1;
    g_pcntOps.valid_max_access_size = 4;
    g_pcntOps.impl_min_access_size = 4;
    g_pcntOps.impl_max_access_size = 4;
    pcntReset();
    if (gpioIn && g_soc) g_pcntIrq = gpioIn((uint8_t*)g_soc + OFS_INTMATRIX, PCNT_SRC);
    void* sys = q_get_system_memory();
    const uint64_t base[2] = { 0x3FF57000ull, 0x60017000ull };
    for (int k = 0; k < 2; k++)
    {
        memset(&g_pcntMr[k], 0, sizeof(g_pcntMr[k]));
        q_mr_init_io(&g_pcntMr[k], nullptr, &g_pcntOps, nullptr, k ? "esp32link.pcnt.alias" : "esp32link.pcnt", 0x100);
        q_mr_add_overlap(sys, base[k], &g_pcntMr[k], 10);
    }
    elog("PCNT installed (8 units, interrupt %s)", g_pcntIrq ? "on" : "MISSING");
}

// ===========================================================================
//  MCPWM (motor control PWM). Not in QEMU at all: the engine is the whole
//  block. Unit 0 at 0x3FF5E000, unit 1 at 0x3FF6C000 (also 0x6001E000 /
//  0x6002C000), interrupt matrix sources 39 / 40. Register map (mcpwm_struct.h):
//    0x000 CLK_CFG prescale (PWM clock = 160 MHz / (p + 1))
//    0x004 + 0x10 t  timer t: CFG0 prescale 0-7, period 8-23, period update 24-25;
//                    CFG1 start 0-2 (0 stop at TEZ, 1 stop at TEP, 2 run, 3/4 one period),
//                    mode 3-4 (0 freeze, 1 up, 2 down, 3 up-down); SYNC; STATUS value 0-15, dir 16
//    0x038 operator -> timer (2 bits each)
//    0x03C + 0x38 o  operator o: +00 compare update, +04 compare A, +08 compare B, +0C gen cfg,
//                    +10 force, +14 / +18 generator A / B actions (2 bits per event: 1 low,
//                    2 high, 3 toggle; UTEZ UTEP UTEA UTEB UT0 UT1 then the same counting down),
//                    +1C dead time cfg, +20 FED, +24 RED, +28 carrier, +2C.. fault handler
//    0x0E4 fault, 0x0E8 capture timer cfg (bit 0 on), 0x0EC phase, 0x0F0 + 4n capture n cfg
//          (en 0, mode 1-2 = neg / pos edge, prescale 3-10, invert 11, sw 12), 0x0FC + 4n value,
//          0x108 status (bit n: last capture on a falling edge)
//    0x110 / 114 / 118 / 11C INT_ENA / RAW / ST / CLR: 0-2 timer stop, 3-5 TEZ, 6-8 TEP,
//          15-17 TEA, 18-20 TEB, 27-29 capture
//  Outputs: the edges of every output are computed up to "now" at each slice boundary and
//  before every register write, and sent as PIN_OUT events with their exact times (the model
//  plays each event at its time). Inputs (capture): watched pins, exact edge times, counted
//  at the slice boundary up to grant - 2 slices. Not simulated: carrier, fault, sync.
// ===========================================================================
enum {
    MC_TCFG0 = 0x04 / 4, MC_TIMERSEL = 0x38 / 4, MC_OP = 0x3C / 4, MC_OPSZ = 0x38 / 4,
    MC_CAPT = 0xE8 / 4, MC_CAPPHASE = 0xEC / 4, MC_CAPCFG = 0xF0 / 4, MC_CAPVAL = 0xFC / 4, MC_CAPST = 0x108 / 4,
    MC_ENA = 0x110 / 4, MC_RAW = 0x114 / 4, MC_ST = 0x118 / 4, MC_CLR = 0x11C / 4, MC_VER = 0x124 / 4, MC_NREG = 0x128 / 4
};
enum { MC_TEZ, MC_TEP, MC_TEA, MC_TEB };            // generator event types (bit pairs 0-3)
struct McTimer
{
    bool run = false;
    bool stopAtEnd = false;
    int period = 255;               // P in use
    int pendPeriod = -1;            // waits for the period end
    int64_t epochNs = 0;            // time of tick 0
    double tick = 0;                // ns per timer tick (at the epoch)
    int64_t anchor = 0;             // tick of the current period start
    int holdValue = 0;              // value while stopped
};
struct McStage { int in = 0, level = 0; bool pend = false; int64_t pendNs = 0; };
struct McOp
{
    int cmp[2] = {};
    bool cmpPend[2] = {};
    int gen[2] = {};
    McStage red, fed;
    int out[2] = { -1, -1 };
    int pin[2] = {};
};
struct McCap { int pin = 0; bool inv = false; uint32_t count = 0; int64_t sinceNs = 0; };
struct McCapVal { uint32_t value; int falling; };
struct McUnit
{
    uint32_t r[MC_NREG];
    McTimer t[3];
    McOp op[3];
    McCap cap[3];
    int64_t capStartNs = 0;
    int64_t doneNs = 0;             // outputs computed up to here
    void* irq = nullptr;
    std::vector<PcntEdge> edges;    // capture input edges
    std::deque<McCapVal> capQ[3];   // captures waiting for the firmware (one per interrupt)
    uint32_t pend[32] = {};         // interrupt events waiting (several in one slice)
    bool warned = false;
};
static McUnit g_mc[2];
static QemuMemoryRegionOps     g_mcOps;
static QemuMemoryRegionStorage g_mcMr[4];

static int mcOpReg(int o, int k) { return MC_OP + o * MC_OPSZ + k; }
static int mcTimerOf(McUnit& M, int o) { int t = (M.r[MC_TIMERSEL] >> (2 * o)) & 3; return t < 3 ? t : 0; }
static double mcTickNs(McUnit& M, int t)
{
    double pwmClk = 160e6 / ((M.r[0] & 0xFF) + 1);
    return ((M.r[MC_TCFG0 + 4 * t] & 0xFF) + 1) * 1e9 / pwmClk;
}
static int64_t mcAt(const McTimer& T, double ticks) { return T.epochNs + (int64_t)llround(ticks * T.tick); }

static void mcUpdateIrq(McUnit& M)
{
    if (M.irq) q_set_irq(M.irq, (M.r[MC_RAW] & M.r[MC_ENA]) ? 1 : 0);
}

// An interrupt event. Events computed together at a slice boundary are handed to the
// firmware one by one: each INT_CLR of the bit raises it again while more are waiting.
static void mcRaise(McUnit& M, int bit)
{
    if ((M.r[MC_ENA] >> bit) & 1) M.pend[bit]++;
    M.r[MC_RAW] |= 1u << bit;
}

static void mcLoadCapture(McUnit& M, int n)
{
    McCapVal c = M.capQ[n].front();
    M.capQ[n].pop_front();
    M.r[MC_CAPVAL + n] = c.value;
    M.r[MC_CAPST] = (M.r[MC_CAPST] & ~(1u << n)) | ((uint32_t)c.falling << n);
}

static void mcPushCapture(McUnit& M, int n, McCapVal c)
{
    if (!((M.r[MC_ENA] >> (27 + n)) & 1)) { M.capQ[n].clear(); M.capQ[n].push_back(c); mcLoadCapture(M, n); }   // polled: newest
    else if (M.capQ[n].size() < 4096) M.capQ[n].push_back(c);
    mcRaise(M, 27 + n);
}

static void mcClear(McUnit& M, uint32_t mask)
{
    for (int bit = 0; bit < 32; bit++)
    {
        if (!((mask >> bit) & 1)) continue;
        M.r[MC_RAW] &= ~(1u << bit);
        if (bit >= 27 && bit < 30 && !M.capQ[bit - 27].empty()) mcLoadCapture(M, bit - 27);   // the ISR reads it now
        if (M.pend[bit] && --M.pend[bit]) M.r[MC_RAW] |= 1u << bit;
    }
    mcUpdateIrq(M);
}

// ---- output path: generator -> dead time -> pin ----
static int mcGenEff(McUnit& M, int o, int g)          // generator level after a continuous force
{
    int f = (M.r[mcOpReg(o, 4)] >> (6 + 2 * g)) & 3;
    return f == 1 ? 0 : f == 2 ? 1 : M.op[o].gen[g];
}

static void mcSetOut(int u, int o, int64_t t)
{
    McUnit& M = g_mc[u];
    McOp& O = M.op[o];
    uint32_t dt = M.r[mcOpReg(o, 7)];
    int redOut = O.red.level ^ (int)((dt >> 13) & 1), fedOut = O.fed.level ^ (int)((dt >> 14) & 1);
    int a = (dt >> 15) & 1 ? mcGenEff(M, o, 0) : redOut;
    int b = (dt >> 16) & 1 ? mcGenEff(M, o, 1) : fedOut;
    int lv[2] = { (dt >> 9) & 1 ? b : a, (dt >> 10) & 1 ? a : b };
    for (int g = 0; g < 2; g++)
    {
        if (lv[g] == O.out[g]) continue;
        O.out[g] = lv[g];
        if (O.pin[g]) toModelAt(t, E32_EV_PIN_OUT, (uint16_t)O.pin[g], (uint32_t)lv[g]);
    }
}

static double mcDtTick(McUnit& M, int o)
{
    return (M.r[mcOpReg(o, 7)] >> 17) & 1 ? mcTickNs(M, mcTimerOf(M, o)) : 1e9 / (160e6 / ((M.r[0] & 0xFF) + 1));
}

// delayed changes that are due by 'upto', in time order
static void mcFlush(int u, int o, int64_t upto)
{
    McOp& O = g_mc[u].op[o];
    for (;;)
    {
        McStage* s = nullptr;
        if (O.red.pend && O.red.pendNs <= upto) s = &O.red;
        if (O.fed.pend && O.fed.pendNs <= upto && (!s || O.fed.pendNs < s->pendNs)) s = &O.fed;
        if (!s) return;
        s->pend = false;
        s->level = !s->level;
        mcSetOut(u, o, s->pendNs);
    }
}

// RED delays rising edges, FED delays falling edges; a delayed edge is dropped when the input goes back
static void mcStage(McStage& s, int in, bool rising, int64_t t, int64_t delay)
{
    if (in == s.in) return;
    s.in = in;
    int slow = rising ? 1 : 0;                         // the level that comes late
    if (in == slow)
    {
        if (s.level == slow || s.pend) return;
        if (delay <= 0) s.level = slow;
        else { s.pend = true; s.pendNs = t + delay; }
    }
    else { s.pend = false; s.level = in; }
}

static void mcGenChanged(int u, int o, int64_t t)
{
    McUnit& M = g_mc[u];
    McOp& O = M.op[o];
    mcFlush(u, o, t);
    uint32_t dt = M.r[mcOpReg(o, 7)];
    double dtick = mcDtTick(M, o);
    int a = mcGenEff(M, o, 0), b = mcGenEff(M, o, 1);
    // delay = (register + 1) ticks (the IDF writes ticks - 1)
    mcStage(O.red, (dt >> 11) & 1 ? b : a, true, t, (int64_t)llround(((M.r[mcOpReg(o, 9)] & 0xFFFF) + 1) * dtick));
    mcStage(O.fed, (dt >> 12) & 1 ? b : a, false, t, (int64_t)llround(((M.r[mcOpReg(o, 8)] & 0xFFFF) + 1) * dtick));
    mcSetOut(u, o, t);
}

static void mcGenEvent(int u, int o, int type, int down, int64_t t)
{
    McUnit& M = g_mc[u];
    mcFlush(u, o, t);                                     // delayed edges before this event: with the old levels
    bool changed = false;
    bool upDown = ((M.r[MC_TCFG0 + 4 * mcTimerOf(M, o) + 1] >> 3) & 3) == 3;
    for (int g = 0; g < 2; g++)
    {
        uint32_t act = M.r[mcOpReg(o, 5 + g)];
        int a = (act >> (2 * (type + (down ? 6 : 0)))) & 3;
        if (!a && upDown && (type == MC_TEZ || type == MC_TEP)) a = (act >> (2 * (type + (down ? 0 : 6)))) & 3;   // either direction
        if (!a) continue;
        int lv = a == 1 ? 0 : a == 2 ? 1 : !M.op[o].gen[g];
        if (lv != M.op[o].gen[g]) { M.op[o].gen[g] = lv; changed = true; }
    }
    if (changed) mcGenChanged(u, o, t);
}

// ---- timers ----
struct McEv { int off, prio, op, type, down; };

static void mcPeriodEnd(McUnit& M, int t)
{
    McTimer& T = M.t[t];
    if (T.pendPeriod >= 0) { T.period = T.pendPeriod; T.pendPeriod = -1; }
    for (int o = 0; o < 3; o++)
        if (mcTimerOf(M, o) == t)
            for (int g = 0; g < 2; g++)
                if (M.op[o].cmpPend[g]) { M.op[o].cmp[g] = M.r[mcOpReg(o, 1 + g)] & 0xFFFF; M.op[o].cmpPend[g] = false; }
}

static void mcTimerRun(int u, int t, int64_t from, int64_t to)
{
    McUnit& M = g_mc[u];
    McTimer& T = M.t[t];
    int mode = (M.r[MC_TCFG0 + 4 * t + 1] >> 3) & 3;
    if (!T.run || mode == 0) return;
    for (int guard = 0; guard < 1000000; guard++)
    {
        int P = T.period > 0 ? T.period : 1;
        int L = mode == 3 ? 2 * P : P + 1;
        std::vector<McEv> ev;
        ev.push_back({ mode == 2 ? P : 0, 0, -1, MC_TEZ, mode == 2 });
        ev.push_back({ mode == 2 ? 0 : P, 1, -1, MC_TEP, mode != 1 });
        for (int o = 0; o < 3; o++)
        {
            if (mcTimerOf(M, o) != t) continue;
            ev.push_back({ mode == 2 ? P : 0, 0, o, MC_TEZ, mode == 2 });
            ev.push_back({ mode == 2 ? 0 : P, 1, o, MC_TEP, mode != 1 });
            for (int g = 0; g < 2; g++)
            {
                int c = M.op[o].cmp[g], type = g ? MC_TEB : MC_TEA, prio = g ? 2 : 3;
                if (c > P) continue;
                if (mode == 1) ev.push_back({ c, prio, o, type, 0 });
                else if (mode == 2) ev.push_back({ P - c, prio, o, type, 1 });
                else
                {
                    ev.push_back({ c, prio, o, type, 0 });
                    if (c > 0 && c < P) ev.push_back({ 2 * P - c, prio, o, type, 1 });
                }
            }
        }
        std::sort(ev.begin(), ev.end(), [](const McEv& a, const McEv& b) { return a.off != b.off ? a.off < b.off : a.prio < b.prio; });
        for (const McEv& e : ev)
        {
            int64_t te = mcAt(T, (double)(T.anchor + e.off));
            if (te <= from || te > to) continue;
            if (e.op < 0) { mcRaise(M, (e.type == MC_TEZ ? 3 : 6) + t); continue; }
            if (e.type == MC_TEA || e.type == MC_TEB) mcRaise(M, (e.type == MC_TEA ? 15 : 18) + e.op);
            mcGenEvent(u, e.op, e.type, e.down, te);
        }
        int64_t end = mcAt(T, (double)(T.anchor + L));
        if (end > to) return;
        T.anchor += L;
        mcPeriodEnd(M, t);
        if (T.stopAtEnd)
        {
            T.run = false;
            T.stopAtEnd = false;
            T.holdValue = mode == 2 ? T.period : 0;
            mcRaise(M, t);                                   // timer stop
            return;
        }
    }
}

// all outputs of unit u up to 'to'
static void mcRun(int u, int64_t to)
{
    McUnit& M = g_mc[u];
    if (to <= M.doneNs) return;
    int64_t from = M.doneNs;
    for (int t = 0; t < 3; t++) mcTimerRun(u, t, from, to);
    for (int o = 0; o < 3; o++) mcFlush(u, o, to);
    M.doneNs = to;
    mcUpdateIrq(M);
}

// new tick length (prescale change): keep the period start, new tick from there
static void mcReEpoch(McUnit& M, int t, int64_t now)
{
    McTimer& T = M.t[t];
    if (T.run) T.epochNs = mcAt(T, (double)T.anchor);
    else T.epochNs = now;
    T.anchor = 0;
    T.tick = mcTickNs(M, t);
}

static int mcTimerValue(McUnit& M, int t, int64_t now, int* down)
{
    McTimer& T = M.t[t];
    int mode = (M.r[MC_TCFG0 + 4 * t + 1] >> 3) & 3;
    *down = mode == 2;
    if (!T.run || mode == 0 || T.tick <= 0) return T.holdValue;
    int P = T.period > 0 ? T.period : 1;
    int64_t off = (int64_t)((now - mcAt(T, (double)T.anchor)) / T.tick);
    if (off < 0) off = 0;
    if (mode == 1) return (int)(off % (P + 1));
    if (mode == 2) return P - (int)(off % (P + 1));
    off %= 2 * P;
    *down = off >= P;
    return off < P ? (int)off : 2 * P - (int)off;
}

// ---- routing (outputs every slice + GPIO matrix sync; capture inputs) ----
static void mcRoute()
{
    if (!g_outSel || !g_inSel) return;
    int64_t now = now_ns();
    for (int u = 0; u < 2; u++)
    {
        McUnit& M = g_mc[u];
        for (int o = 0; o < 3; o++)
            for (int g = 0; g < 2; g++)
            {
                int pin = outPinFor((u ? 108 : 32) + 2 * o + g);
                McOp& O = M.op[o];
                if (pin == O.pin[g]) continue;
                O.pin[g] = pin;
                if (pin)
                {
                    elog("MCPWM%d operator %d output %c: board pin %d", u, o, g ? 'B' : 'A', pin);
                    if (O.out[g] < 0) { mcSetOut(u, o, now); }
                    else toModelAt(now, E32_EV_PIN_OUT, (uint16_t)pin, (uint32_t)O.out[g]);
                }
            }
        for (int n = 0; n < 3; n++)
        {
            uint32_t v = g_inSel[(u ? 112 : 109) + n];
            int pin = (v & 0x80) && (v & 0x3F) < 40 ? g_gpioToBoard[v & 0x3F] : 0;
            McCap& C = M.cap[n];
            C.inv = (v & 0x40) != 0;
            if (pin == C.pin) continue;
            if (C.pin) watchPin(C.pin, false);
            if (pin) { watchPin(pin, true); elog("MCPWM%d capture %d: board pin %d", u, n, pin); }
            C.pin = pin;
            C.sinceNs = now;
        }
    }
}

static void mcpwmEdge(int pin, int level, int64_t ns)
{
    for (int u = 0; u < 2; u++)
        for (int n = 0; n < 3; n++)
            if (g_mc[u].cap[n].pin == pin) { g_mc[u].edges.push_back({ pin, level, ns }); break; }
}

static void mcCapture(McUnit& M, int n, int falling, int64_t t)
{
    uint32_t cfg = M.r[MC_CAPCFG + n];
    if (!(cfg & 1) || !(M.r[MC_CAPT] & 1)) return;
    int mode = (cfg >> 1) & 3;                            // bit 0: falling edge, bit 1: rising edge
    if (!(falling ? (mode & 1) : (mode & 2))) return;
    McCap& C = M.cap[n];
    if (++C.count % (((cfg >> 3) & 0xFF) + 1)) return;     // prescale
    mcPushCapture(M, n, { (uint32_t)((t - M.capStartNs) * 2 / 25) + M.r[MC_CAPPHASE], falling });   // APB 80 MHz
}

static void updateMcpwm()
{
    mcRoute();
    int64_t now = now_ns();
    const int64_t safeNs = e32_load64(&g_sh->grantNs) - 2 * g_sliceNs;
    for (int u = 0; u < 2; u++)
    {
        McUnit& M = g_mc[u];
        std::stable_sort(M.edges.begin(), M.edges.end(), [](const PcntEdge& a, const PcntEdge& b) { return a.ns < b.ns; });
        size_t k = 0;
        for (; k < M.edges.size() && M.edges[k].ns <= safeNs; k++)
            for (int n = 0; n < 3; n++)
            {
                McCap& C = M.cap[n];
                if (C.pin != M.edges[k].pin || M.edges[k].ns < C.sinceNs) continue;
                int lv = M.edges[k].level ^ (C.inv ? 1 : 0);
                mcCapture(M, n, !lv, M.edges[k].ns);
            }
        M.edges.erase(M.edges.begin(), M.edges.begin() + k);
        mcRun(u, now);
        if (!M.warned)
        {
            bool carrier = false, fault = (M.r[0xE4 / 4] & 7) != 0, sync = false;
            for (int o = 0; o < 3; o++) carrier |= (M.r[mcOpReg(o, 10)] & 1) != 0;
            for (int t = 0; t < 3; t++) sync |= (M.r[MC_TCFG0 + 4 * t + 2] & 1) != 0;
            if (carrier || fault || sync)
            {
                M.warned = true;
                elog("MCPWM%d: %s%s%snot simulated", u, carrier ? "carrier " : "", fault ? "fault inputs " : "", sync ? "timer sync " : "");
            }
        }
    }
}

static void mcpwmReset()
{
    for (int u = 0; u < 2; u++)
    {
        McUnit& M = g_mc[u];
        void* irq = M.irq;
        M = McUnit();
        M.irq = irq;
        memset(M.r, 0, sizeof(M.r));
        for (int t = 0; t < 3; t++) M.r[MC_TCFG0 + 4 * t] = 0xFFu << 8;     // period 255
        for (int o = 0; o < 3; o++) M.r[mcOpReg(o, 7)] = (1u << 15) | (1u << 16);   // dead time bypassed
        M.r[MC_VER] = 34632240;
        M.doneNs = now_ns();
        mcUpdateIrq(M);
    }
}

static uint64_t mcpwm_read(void* opaque, uint64_t addr, unsigned)
{
    McUnit& M = g_mc[(int)(uintptr_t)opaque];
    int i = (int)(addr / 4);
    if (i >= MC_NREG) return 0;
    if (i >= MC_TCFG0 && i < MC_TCFG0 + 12 && (i - MC_TCFG0) % 4 == 3)   // timer status
    {
        int down = 0, v = mcTimerValue(M, (i - MC_TCFG0) / 4, now_ns(), &down);
        return (uint32_t)v | ((uint32_t)down << 16);
    }
    if (i == MC_ST) return M.r[MC_RAW] & M.r[MC_ENA];
    if (i == MC_CLR) return 0;
    return M.r[i];
}

static void mcpwm_write(void* opaque, uint64_t addr, uint64_t data, unsigned)
{
    int u = (int)(uintptr_t)opaque;
    McUnit& M = g_mc[u];
    int i = (int)(addr / 4);
    uint32_t v = (uint32_t)data;
    if (i >= MC_NREG) return;
    int64_t now = now_ns();
    mcRun(u, now);                                        // everything before this write with the old values
    uint32_t old = M.r[i];
    if (i == MC_CLR) { mcClear(M, v); return; }
    if (i == MC_RAW || i == MC_ST || i == MC_VER || i == MC_CAPST || (i >= MC_CAPVAL && i < MC_CAPVAL + 3)) return;
    M.r[i] = v;
    if (i == 0)                                           // PWM clock prescale: every timer
    {
        for (int t = 0; t < 3; t++) mcReEpoch(M, t, now);
        return;
    }
    if (i >= MC_TCFG0 && i < MC_TCFG0 + 12)
    {
        int t = (i - MC_TCFG0) / 4, k = (i - MC_TCFG0) % 4;
        McTimer& T = M.t[t];
        if (k == 0)
        {
            if ((v & 0xFF) != (old & 0xFF)) mcReEpoch(M, t, now);
            int P = (v >> 8) & 0xFFFF;
            if (((v >> 24) & 3) == 0 || !T.run) { T.period = P; T.pendPeriod = -1; }
            else T.pendPeriod = P;
        }
        else if (k == 1)
        {
            int start = v & 7;
            if (start == 2 || start == 3 || start == 4)
            {
                if (!T.run)
                {
                    T.run = true;
                    T.epochNs = now;
                    T.anchor = 0;
                    T.tick = mcTickNs(M, t);
                    if (T.pendPeriod >= 0) { T.period = T.pendPeriod; T.pendPeriod = -1; }
                    mcTimerRun(u, t, now - 1, now);       // the start event (TEZ) at 'now'
                }
                T.stopAtEnd = start != 2;
            }
            else if (T.run) T.stopAtEnd = true;           // 0 / 1: stop at the end of this period
        }
        return;
    }
    if (i >= MC_OP && i < MC_OP + 3 * MC_OPSZ)
    {
        int o = (i - MC_OP) / MC_OPSZ, k = (i - MC_OP) % MC_OPSZ;
        McOp& O = M.op[o];
        if (k == 1 || k == 2)                             // compare A / B
        {
            int g = k - 1;
            int method = (M.r[mcOpReg(o, 0)] >> (4 * g)) & 0xF;
            if (method == 0 || !M.t[mcTimerOf(M, o)].run) { O.cmp[g] = v & 0xFFFF; O.cmpPend[g] = false; }
            else O.cmpPend[g] = true;
        }
        else if (k == 4)                                  // force
        {
            mcFlush(u, o, now);
            if ((v ^ old) & (1u << 10)) { int m = (v >> 11) & 3; if (m) O.gen[0] = m == 2; }
            if ((v ^ old) & (1u << 13)) { int m = (v >> 14) & 3; if (m) O.gen[1] = m == 2; }
            mcGenChanged(u, o, now);
        }
        else if (k == 5 || k == 6 || k == 7 || k == 8 || k == 9) mcGenChanged(u, o, now);
        return;
    }
    if (i == MC_TIMERSEL) return;
    if (i == MC_CAPT)
    {
        if ((v & 1) && !(old & 1)) M.capStartNs = now;
        return;
    }
    if (i >= MC_CAPCFG && i < MC_CAPCFG + 3)
    {
        int n = i - MC_CAPCFG;
        if (v & (1u << 12))                               // software capture
        {
            mcPushCapture(M, n, { (uint32_t)((now - M.capStartNs) * 2 / 25) + M.r[MC_CAPPHASE], (int)((M.r[MC_CAPST] >> n) & 1) });
            M.r[i] &= ~(1u << 12);
        }
        if ((v ^ old) & 1) M.cap[n].count = 0;
        mcUpdateIrq(M);
        return;
    }
    if (i == MC_ENA) mcUpdateIrq(M);
}

static void installMcpwm(HMODULE q)
{
    typedef void* (*qdev_get_gpio_in_fn)(void* dev, int n);
    qdev_get_gpio_in_fn gpioIn = (qdev_get_gpio_in_fn)GetProcAddress(q, "qdev_get_gpio_in");
    memset(&g_mcOps, 0, sizeof(g_mcOps));
    g_mcOps.read = mcpwm_read;
    g_mcOps.write = mcpwm_write;
    g_mcOps.endianness = 2;
    g_mcOps.valid_min_access_size = 1;
    g_mcOps.valid_max_access_size = 4;
    g_mcOps.impl_min_access_size = 4;
    g_mcOps.impl_max_access_size = 4;
    for (int u = 0; u < 2; u++)
        if (gpioIn && g_soc) g_mc[u].irq = gpioIn((uint8_t*)g_soc + OFS_INTMATRIX, 39 + u);
    mcpwmReset();
    void* sys = q_get_system_memory();
    const uint64_t base[4] = { 0x3FF5E000ull, 0x3FF6C000ull, 0x6001E000ull, 0x6002C000ull };
    for (int k = 0; k < 4; k++)
    {
        memset(&g_mcMr[k], 0, sizeof(g_mcMr[k]));
        q_mr_init_io(&g_mcMr[k], nullptr, &g_mcOps, (void*)(uintptr_t)(k & 1), k < 2 ? "esp32link.mcpwm" : "esp32link.mcpwm.alias", 0x128);
        q_mr_add_overlap(sys, base[k], &g_mcMr[k], 10);
    }
    elog("MCPWM installed (2 units, interrupts %s)", g_mc[0].irq && g_mc[1].irq ? "on" : "MISSING");
}

static void installCan(HMODULE q)
{
    typedef bool (*mapped_fn)(void* mr);
    typedef bool (*set_link_fn)(void* obj, const char* name, void* value, void** errp);
    typedef bool (*sysbus_realize_fn)(void* dev, void** errp);
    typedef void (*init_child_fn)(void* parent, const char* name, void* child, size_t size, const char* type);
    typedef void* (*qdev_get_gpio_in_fn)(void* dev, int n);
    typedef void (*sysbus_connect_irq_fn)(void* dev, int n, void* irq);
    can_bus_insert_client_fn insert = (can_bus_insert_client_fn)GetProcAddress(q, "can_bus_insert_client");
    object_resolve_path_fn resolve = (object_resolve_path_fn)GetProcAddress(q, "object_resolve_path");
    mapped_fn mapped = (mapped_fn)GetProcAddress(q, "memory_region_is_mapped");
    set_link_fn setLink = (set_link_fn)GetProcAddress(q, "object_property_set_link");
    sysbus_realize_fn realize = (sysbus_realize_fn)GetProcAddress(q, "sysbus_realize");
    init_child_fn initChild = (init_child_fn)GetProcAddress(q, "object_initialize_child_internal");
    qdev_get_gpio_in_fn gpioIn = (qdev_get_gpio_in_fn)GetProcAddress(q, "qdev_get_gpio_in");
    sysbus_connect_irq_fn connectIrq = (sysbus_connect_irq_fn)GetProcAddress(q, "sysbus_connect_irq");
    q_can_send = (can_bus_client_send_fn)GetProcAddress(q, "can_bus_client_send");
    q_sja_write = (can_sja_mem_write_fn)GetProcAddress(q, "can_sja_mem_write");
    if (!insert || !resolve || !mapped || !setLink || !realize || !initChild || !gpioIn || !connectIrq || !q_can_send || !q_sja_write || !g_soc)
    {
        elog("TWAI: QEMU functions missing - CAN off");
        return;
    }
    uint8_t* twai = (uint8_t*)g_soc + OFS_TWAI;
    bool amb = false;
    void* bus = resolve("/objects/canbus0", &amb);
    if (!bus) { elog("TWAI: canbus0 not found - CAN off"); return; }

    // PICSimLab's SoC has room for the TWAI but never creates it (the guest gets a bus error
    // at 0x3FF6B000). Do what Espressif's SoC does: create it in that room, canbus link,
    // realize, registers at 0x3FF6B000, interrupt = interrupt matrix source 45.
    if (!*(void**)(twai + OFS_TWAI_OPS))
    {
        const uint8_t* p = twai;
        for (int i = 0; i < 0x590; i++) if (p[i]) { elog("TWAI: SoC memory not empty (other QEMU build) - CAN off"); return; }
        initChild((void*)g_soc, "esp32link-twai", twai, 0x590, "esp32.twai");
        setLink(twai, "canbus", bus, nullptr);
        void* err = nullptr;
        if (*(void**)(twai + OFS_TWAI_CANBUS) != bus || !realize(twai, &err))
        {
            elog("TWAI: cannot start the controller - CAN off");
            return;
        }
        q_mr_add_overlap(q_get_system_memory(), 0x3FF6B000ull, (QemuMemoryRegionStorage*)(twai + OFS_TWAI_IOMEM), 0);
        connectIrq(twai, 0, gpioIn((uint8_t*)g_soc + OFS_INTMATRIX, ETS_TWAI_INTR_SOURCE));
        elog("TWAI: controller created, registers at 0x3FF6B000, interrupt source %d", ETS_TWAI_INTR_SOURCE);
    }
    else if (!mapped(twai + OFS_TWAI_IOMEM) || *(void**)(twai + OFS_TWAI_CANBUS) != bus)
    {
        elog("TWAI: controller set up differently (other QEMU build) - CAN off");
        return;
    }

    g_canInfo.can_receive = can_client_can_receive;
    g_canInfo.receive = can_client_receive;
    memset(g_canClient, 0, sizeof(g_canClient));
    *(QemuCanClientInfo**)g_canClient = &g_canInfo;
    if (insert(bus, g_canClient) < 0) { elog("TWAI: cannot join canbus0 - CAN off"); return; }

    memset(&g_twaiOps, 0, sizeof(g_twaiOps));
    g_twaiOps.read = twai_read;
    g_twaiOps.write = twai_write;
    g_twaiOps.endianness = 2;
    g_twaiOps.valid_min_access_size = 1;
    g_twaiOps.valid_max_access_size = 4;
    g_twaiOps.impl_min_access_size = 4;
    g_twaiOps.impl_max_access_size = 4;
    void* sys = q_get_system_memory();
    memset(g_twaiMr, 0, sizeof(g_twaiMr));
    q_mr_init_io(&g_twaiMr[0], nullptr, &g_twaiOps, (void*)0, "esp32link.twai.cmr", 4);
    q_mr_add_overlap(sys, 0x3FF6B004ull, &g_twaiMr[0], 10);
    q_mr_init_io(&g_twaiMr[1], nullptr, &g_twaiOps, (void*)1, "esp32link.twai.btr", 8);
    q_mr_add_overlap(sys, 0x3FF6B018ull, &g_twaiMr[1], 10);
    q_mr_init_io(&g_twaiMr[2], nullptr, &g_twaiOps, (void*)2, "esp32link.twai.err", 0x14);
    q_mr_add_overlap(sys, 0x3FF6B02Cull, &g_twaiMr[2], 10);
    canReset();
    g_canOk = true;
    elog("TWAI (CAN) connected: engine joined canbus0, CMR / BTR / error registers installed");
}

// ===========================================================================
//  DAC (dacWrite): DAC1 on GPIO25, DAC2 on GPIO26
// ===========================================================================
//  The DAC lives in the RTC IO block (our own register copy, see above):
//  RTC_IO_PAD_DAC1 0x84 / PAD_DAC2 0x88, bit 18 XPD_DAC = DAC powered,
//  bits 19-26 = 8-bit value. Output = VDD3P3_RTC * value / 255.
//  While the DAC is on it owns the pad; the GPIO / LEDC output does not reach it.
//  Called on every write to those registers (vCPU thread) and every slice.
// ---------------------------------------------------------------------------
static void updateDac()
{
    static const int GPIO[2] = { 25, 26 };
    static const uint32_t REG[2] = { 0x84, 0x88 };
    EnterCriticalSection(&g_pushLock);
    for (int k = 0; k < 2; k++)
    {
        int pin = g_gpioToBoard[GPIO[k]];
        if (!pin) continue;
        uint32_t r = g_rtcio[REG[k] / 4];
        int now = (r & (1u << 18)) ? (int)((r >> 19) & 0xFF) : -1;
        if (now == g_dac[k]) continue;
        bool wasOn = g_dac[k] >= 0;
        g_dac[k] = now;
        if (now >= 0) toModel(E32_EV_DAC, (uint16_t)pin, (uint32_t)now);
        else
        {
            toModel(E32_EV_DAC, (uint16_t)pin, 0xFFFFFFFFu);
            g_pwm[pin].on = false;                       // send the LEDC state again if it has one
        }
        static int logged = 0;
        if (logged < 30 || wasOn != (now >= 0))
        {
            logged++;
            if (now >= 0) elog("DAC%d on GPIO%d: %d/255 (%.3f V at 3.3 V)", k + 1, GPIO[k], now, now * 3.3 / 255.0);
            else elog("DAC%d off on GPIO%d", k + 1, GPIO[k]);
        }
    }
    LeaveCriticalSection(&g_pushLock);
}

// ===========================================================================
//  Bus requests: the firmware waits while the model clocks the bits in
//  Proteus time and reads the answer (ACK, data) back from the wires.
//  Runs on the vCPU thread with the QEMU big lock held: the guest is frozen.
// ===========================================================================
// ---------------------------------------------------------------------------
//  Bus time. While the firmware waits for an I2C / SPI step, Proteus clocks the
//  bits; QEMU's clock stands still. Afterwards the clock is moved forward by the
//  time the step took in Proteus (icount: virtual time = qemu_icount_bias +
//  instructions x 2^shift ns), so millis(), delay() and timeouts include the bus
//  time, as on the real chip. Written like QEMU does: spin lock + seqlock.
// ---------------------------------------------------------------------------
enum { OFS_TS_SEQLOCK = 0x10, OFS_TS_SPIN = 0x14, OFS_TS_ICOUNT_BIAS = 0x28 };   // TimersState (DWARF)
static uint8_t* g_timersState = nullptr;

static void advanceVirtualClock(int64_t ns)
{
    if (!g_timersState || ns <= 0) return;
    volatile LONG* spin = (volatile LONG*)(g_timersState + OFS_TS_SPIN);
    volatile LONG* seq = (volatile LONG*)(g_timersState + OFS_TS_SEQLOCK);
    while (InterlockedCompareExchange(spin, 1, 0) != 0) YieldProcessor();
    InterlockedIncrement(seq);                    // odd: readers retry
    *(volatile int64_t*)(g_timersState + OFS_TS_ICOUNT_BIAS) += ns;
    InterlockedIncrement(seq);                    // even again
    InterlockedExchange(spin, 0);
}

static uint32_t busCall(uint16_t type, uint16_t bus, uint32_t b, uint32_t c)
{
    updateRouting();
    LONG seq = InterlockedIncrement(&g_sh->busReqSeq);
    E32Event e;
    memset(&e, 0, sizeof(e));
    e.ns = now_ns();
    e.type = type;
    e.a = bus;
    e.b = b;
    e.c = c;
    e.reserved = (uint32_t)seq;
    EnterCriticalSection(&g_pushLock);
    while (!e32_push(&g_sh->toModel, e))
    {
        if (g_sh->quit) quitNow();
        SetEvent(g_reachedEvt);
        Sleep(0);
    }
    LeaveCriticalSection(&g_pushLock);
    SetEvent(g_reachedEvt);                      // wake the model: it stops its slice and does the bus step

    HANDLE h[2] = { g_grantEvt, g_parent };
    for (;;)
    {
        if (g_sh->quit) quitNow();
        if (InterlockedCompareExchange(&g_sh->busRespSeq, 0, 0) == seq) break;
        DWORD w = WaitForMultipleObjects(2, h, FALSE, 200);
        if (w == WAIT_OBJECT_0 + 1) quitNow();
    }
    advanceVirtualClock(g_sh->busElapsedNs);      // the firmware's clock catches up with Proteus
    return (uint32_t)g_sh->busResp;
}

// I2C SCL frequency from the controller: 80 MHz / (high + low period)
enum { OFS_I2C0 = 0xeee0, I2C_STRIDE = 0x500, OFS_I2C_HIGH = 0x49c, OFS_I2C_LOW = 0x4a0 };
static uint32_t i2cHz(int bus)
{
    if (!g_soc || bus > 1) return 100000;
    const uint8_t* st = g_soc + OFS_I2C0 + bus * I2C_STRIDE;
    uint32_t hi = *(const uint32_t*)(st + OFS_I2C_HIGH) & 0x3FFF;
    uint32_t lo = *(const uint32_t*)(st + OFS_I2C_LOW) & 0x3FFF;
    if (hi + lo < 20) return 100000;
    uint32_t f = 80000000u / (hi + lo);
    return f > 1000000 ? 1000000 : f;
}

// picsimlab_i2c bridge: event = op | data << 8 (see esp32link.h), addr = 7-bit address
static int cb_i2c_event(const uint8_t bus, const uint8_t addr, const uint16_t event)
{
    int op = event & 0xFF;
    uint32_t data = event >> 8;
    switch (op)
    {
    case 0:   // START + read
        return (int)busCall(E32_EV_I2C, bus, 0u | ((uint32_t)addr << 8), i2cHz(bus));
    case 1:   // START + write
    case 2:   // START + write (async)
        return (int)busCall(E32_EV_I2C, bus, 1u | ((uint32_t)addr << 8), i2cHz(bus));
    case 3:   // STOP
        busCall(E32_EV_I2C, bus, 3u, i2cHz(bus));
        return 0;
    case 4:   // master NACK
        busCall(E32_EV_I2C, bus, 4u, i2cHz(bus));
        return 0;
    case 5:   // write byte -> 0 ACK, 1 NACK
        return (int)busCall(E32_EV_I2C, bus, 5u | (data << 8), i2cHz(bus));
    case 6:   // read byte
        return (int)(busCall(E32_EV_I2C, bus, 6u, i2cHz(bus)) & 0xFF);
    default:
        return 0;
    }
}

// ===========================================================================
//  I2C slave (Wire.begin(address)). QEMU's esp32_i2c is master only (no slave
//  address register). The engine puts its own block over both controllers
//  (0x3FF53000 / 0x3FF67000 and the FIFO windows 0x60013000 / 0x60027000):
//  in master mode every access goes straight to QEMU's own handlers; in slave
//  mode (CTR bit 4 ms_mode = 0 and an address in SLAVE_ADDR 0x10) the engine
//  answers STATUS 0x08, FIFO_CONF 0x18, FIFO data 0x1C, INT_RAW / CLR / ENA /
//  ST 0x20-0x2C and drives the interrupt itself. The model is the slave on the
//  pins (link version 12): E32_EV_I2CS_CONF / _TX to it, E32_EV_I2CS back.
//  Status: bit 1 slave_rw, 4 bus_busy, 5 slave_addressed, 6 byte_trans, 8-13 RX
//  count, 18-23 TX count, 24-26 scl_main_state_last (6 after a master read:
//  Arduino then calls onRequest for the NEXT read, as on a real ESP32).
//  Interrupts: 0 rx_fifo_full (count >= threshold), 1 tx_fifo_empty (count <=
//  threshold), 2 rx overflow, 7 trans_complete (STOP and repeated START).
// ===========================================================================
enum { OFS_I2C_IOMEM = 0x330, OFS_I2C_IRQ = 0x440, OFS_I2C_CTR = 0x484, OFS_MR_OPS = 0x50, OFS_MR_OPAQUE = 0x58, OFS_MR_SIZE = 0x70 };
struct I2cSlave
{
    std::deque<uint8_t> rx, tx;
    uint32_t raw = 0, ena = 0;
    bool rw = false, addressed = false, busy = false, byteTrans = false;
    int lastState = 0;
};
static I2cSlave                  g_i2cs[2];
static uint32_t                  g_i2cCtr[2], g_i2cSlaveAddr[2], g_i2cFifoConf[2], g_i2csSent[2];
static const QemuMemoryRegionOps* g_i2cOrig[2];
static void*                     g_i2cOrigOpaque[2];
static QemuMemoryRegionOps       g_i2cOps;
static QemuMemoryRegionStorage   g_i2cMr[4];

static uint8_t* i2cState(int b) { return (uint8_t*)g_soc + OFS_I2C0 + b * I2C_STRIDE; }
static bool i2csActive(int b) { return !(g_i2cCtr[b] & 0x10) && (g_i2cSlaveAddr[b] & 0x7FFF) != 0; }

static void i2csIrq(int b)
{
    if (i2csActive(b)) q_set_irq(*(void**)(i2cState(b) + OFS_I2C_IRQ), (g_i2cs[b].raw & g_i2cs[b].ena) ? 1 : 0);
}

// tell the model when the slave starts / stops / changes its address
static void i2csConf(int b)
{
    uint32_t want = i2csActive(b) ? 0x80000000u | (g_i2cSlaveAddr[b] & 0x7F) : 0;
    if (want == g_i2csSent[b]) return;
    g_i2csSent[b] = want;
    g_i2cs[b] = I2cSlave();
    toModel(E32_EV_I2CS_CONF, (uint16_t)b, want);
    if (want) elog("I2C%d slave on, address 0x%02X%s", b, want & 0x7F, (g_i2cSlaveAddr[b] >> 31) ? " (10-bit address: not simulated)" : "");
    else elog("I2C%d slave off", b);
}

static void i2csEvent(int b, int op, int data)
{
    if (b > 1 || !i2csActive(b)) return;
    I2cSlave& S = g_i2cs[b];
    uint32_t rxThr = g_i2cFifoConf[b] & 0x1F, txThr = (g_i2cFifoConf[b] >> 5) & 0x1F;
    switch (op)
    {
    case 0:                                               // START / repeated START for us
        if (S.busy) S.raw |= 1u << 7;                     // repeated start ends the first part
        S.busy = S.addressed = true;
        S.rw = data != 0;
        S.lastState = 0;
        break;
    case 1:                                               // byte from the master
        if (S.rx.size() < 32) S.rx.push_back((uint8_t)data);
        else S.raw |= 1u << 2;
        S.byteTrans = true;
        if (S.rx.size() >= rxThr) S.raw |= 1u;
        break;
    case 2:                                               // byte sent to the master
        if (!S.tx.empty()) S.tx.pop_front();
        S.byteTrans = true;
        if (S.tx.size() <= txThr) S.raw |= 1u << 1;
        break;
    case 3:                                               // STOP
        if (S.busy) S.raw |= 1u << 7;
        S.lastState = S.rw ? 6 : 0;
        S.busy = false;
        break;
    }
    i2csIrq(b);
}

static uint64_t i2c_read(void* opaque, uint64_t addr, unsigned size)
{
    int b = (int)(uintptr_t)opaque & 1;
    if (!i2csActive(b)) return g_i2cOrig[b]->read(g_i2cOrigOpaque[b], addr, size);
    I2cSlave& S = g_i2cs[b];
    switch (addr)
    {
    case 0x04: return g_i2cCtr[b];
    case 0x08:
        return ((uint32_t)S.rw << 1) | ((uint32_t)S.busy << 4) | ((uint32_t)S.addressed << 5) | ((uint32_t)S.byteTrans << 6) |
               ((uint32_t)S.rx.size() << 8) | ((uint32_t)S.tx.size() << 18) | ((uint32_t)S.lastState << 24);
    case 0x10: return g_i2cSlaveAddr[b];
    case 0x18: return g_i2cFifoConf[b];
    case 0x1C:
    {
        uint8_t v = 0;
        if (!S.rx.empty()) { v = S.rx.front(); S.rx.pop_front(); }
        return v;
    }
    case 0x20: return S.raw;
    case 0x28: return S.ena;
    case 0x2C: return S.raw & S.ena;
    default: return g_i2cOrig[b]->read(g_i2cOrigOpaque[b], addr, size);
    }
}

static void i2c_write(void* opaque, uint64_t addr, uint64_t data, unsigned size)
{
    int b = (int)(uintptr_t)opaque & 1;
    uint32_t v = (uint32_t)data;
    if (addr == 0x18) g_i2cFifoConf[b] = v & ~(3u << 12);
    if (addr == 0x04 || addr == 0x10)                     // mode / address: QEMU keeps its own copy too
    {
        (addr == 0x04 ? g_i2cCtr : g_i2cSlaveAddr)[b] = v;
        g_i2cOrig[b]->write(g_i2cOrigOpaque[b], addr, data, size);
        i2csConf(b);
        return;
    }
    if (!i2csActive(b)) { g_i2cOrig[b]->write(g_i2cOrigOpaque[b], addr, data, size); return; }
    I2cSlave& S = g_i2cs[b];
    switch (addr)
    {
    case 0x18:
        if (v & (1u << 12)) S.rx.clear();
        if (v & (1u << 13)) { S.tx.clear(); toModel(E32_EV_I2CS_TX, (uint16_t)b, 0x100); }
        break;
    case 0x1C:
        if (S.tx.size() < 32) { S.tx.push_back((uint8_t)v); toModel(E32_EV_I2CS_TX, (uint16_t)b, v & 0xFF); }
        break;
    case 0x24: S.raw &= ~v; break;
    case 0x28: S.ena = v; break;
    default: g_i2cOrig[b]->write(g_i2cOrigOpaque[b], addr, data, size); break;
    }
    i2csIrq(b);
}

static void i2csReset()
{
    for (int b = 0; b < 2; b++)
    {
        g_i2cs[b] = I2cSlave();
        g_i2cCtr[b] = g_soc ? *(const uint32_t*)(i2cState(b) + OFS_I2C_CTR) : 0;
        g_i2cSlaveAddr[b] = 0;
        g_i2cFifoConf[b] = 0;
        g_i2csSent[b] = 0;                                 // the model forgets the slave at a reset
    }
}

static void installI2cSlave()
{
    if (!g_soc) return;
    memset(&g_i2cOps, 0, sizeof(g_i2cOps));
    g_i2cOps.read = i2c_read;
    g_i2cOps.write = i2c_write;
    g_i2cOps.endianness = 2;
    g_i2cOps.valid_min_access_size = 1;
    g_i2cOps.valid_max_access_size = 4;
    g_i2cOps.impl_min_access_size = 4;
    g_i2cOps.impl_max_access_size = 4;
    uint64_t size = 0;
    for (int b = 0; b < 2; b++)
    {
        uint8_t* mr = i2cState(b) + OFS_I2C_IOMEM;
        g_i2cOrig[b] = *(const QemuMemoryRegionOps**)(mr + OFS_MR_OPS);
        g_i2cOrigOpaque[b] = *(void**)(mr + OFS_MR_OPAQUE);
        size = *(const uint64_t*)(mr + OFS_MR_SIZE);
        if (!g_i2cOrig[b] || !g_i2cOrig[b]->read || !g_i2cOrig[b]->write || size < 0x100 || size > 0x10000)
        {
            elog("I2C slave: QEMU's I2C block looks different (other QEMU build) - I2C slave off");
            return;
        }
    }
    i2csReset();
    void* sys = q_get_system_memory();
    const uint64_t base[4] = { 0x3FF53000ull, 0x3FF67000ull, 0x60013000ull, 0x60027000ull };
    for (int k = 0; k < 4; k++)
    {
        memset(&g_i2cMr[k], 0, sizeof(g_i2cMr[k]));
        q_mr_init_io(&g_i2cMr[k], nullptr, &g_i2cOps, (void*)(uintptr_t)(k & 1), k < 2 ? "esp32link.i2c" : "esp32link.i2c.fifo", size);
        q_mr_add_overlap(sys, base[k], &g_i2cMr[k], 10);
    }
    elog("I2C slave support installed (QEMU I2C block 0x%llX bytes)", (unsigned long long)size);
}

// SPI mode from the controller (spi[2] = HSPI = bridge id 0, spi[3] = VSPI = id 1)
enum { OFS_SPI0 = 0xda20, SPI_STRIDE = 0x530, OFS_SPI_CTRL = 0x4b0, OFS_SPI_USER = 0x4c0, OFS_SPI_PIN = 0x4d4 };
static uint32_t spiMode(int bus)
{
    if (!g_soc || bus > 1) return 0;
    const uint8_t* st = g_soc + OFS_SPI0 + (bus + 2) * SPI_STRIDE;
    uint32_t ctrl = *(const uint32_t*)(st + OFS_SPI_CTRL);
    uint32_t user = *(const uint32_t*)(st + OFS_SPI_USER);
    uint32_t pin = *(const uint32_t*)(st + OFS_SPI_PIN);
    uint32_t cpol = (pin >> 29) & 1;              // CK_IDLE_EDGE
    uint32_t outEdge = (user >> 7) & 1;           // CK_OUT_EDGE
    uint32_t cpha = cpol ^ outEdge;
    uint32_t lsb = (ctrl >> 26) & 1;              // WR_BIT_ORDER
    return cpol | (cpha << 1) | (lsb << 2);
}

// picsimlab_spi bridge: event low byte 0 = data (byte in bits 8-15), 1 = CS change
// The first 3000 SPI bytes go to the log as out/in pairs, with the GPIO writes between them
// (e.g. "IO5=0": a chip select driven by digitalWrite), to see what a SPI device answers.
static void spiLog(const char* item)
{
    static int count;
    static char line[160];
    static size_t n;
    if (count >= 3000) return;
    count++;
    size_t k = strlen(item);
    if (n + k >= sizeof(line) - 1) { elog("SPI out/in:%s", line); n = 0; }
    memcpy(line + n, item, k + 1);
    n += k;
    if (count == 3000) { elog("SPI out/in:%s", line); n = 0; }
}

static uint8_t cb_spi_event(const uint8_t bus, const uint16_t event)
{
    int op = event & 0xFF;
    uint32_t data = event >> 8;
    if (op == 1)
    {
        updateRouting();
        toModel(E32_EV_SPI, bus, 1u | (data << 8), spiMode(bus));   // CS: no answer needed
        return 0;
    }
    uint8_t in = (uint8_t)busCall(E32_EV_SPI, bus, 0u | ((data & 0xFF) << 8), spiMode(bus));
    char item[16];
    sprintf_s(item, " %02X/%02X", (unsigned)(data & 0xFF), in);
    spiLog(item);
    return in;
}

// ===========================================================================
//  SPI slave (ESP-IDF driver/spi_slave.h) on HSPI (SPI2, 0x3FF64000) and VSPI
//  (SPI3, 0x3FF65000). QEMU's esp32_spi is master only. Like the I2C slave,
//  the engine puts its own block over both: master mode goes to QEMU's own
//  handlers; slave mode (SLAVE 0x38 bit 30) is answered by the engine:
//    0x00 CMD (bit 18 usr: the firmware arms a transaction), 0x38 SLAVE (bit 4
//    trans_done, bit 9 trans_inten -> interrupt), 0x48 / 0x4C TX / RX bit length - 1,
//    0x64 received bits - 1, 0x80-0xBC data_buf (no DMA), 0x100-0x11C DMA
//    (OUT / IN link: descriptor address & 0xFFFFF + 0x3FF00000, bit 29 start).
//  DMA descriptors (lldesc_t): word 0 size 0-11, length 12-23, eof 30, owner 31;
//  word 1 buffer; word 2 next. At the arm the engine reads the MISO bytes from
//  the OUT chain (or data_buf) and sends them to the model with the SPI mode
//  (from the clock-edge / delay settings spi_ll_slave_set_mode() writes) and
//  bit order; at CS high the model sends the MOSI bytes back: into the IN
//  chain (length, owner = CPU, eof) or data_buf, then trans_done + interrupt.
// ===========================================================================
enum { OFS_SPI_IOMEM = 0x330, OFS_SPI_IRQ = 0x440 };
struct SpiSlave
{
    uint32_t r[0x400 / 4] = {};    // our copy of the registers
    bool armed = false;
    bool outStart = false, inStart = false;   // DMA links started for this transaction
    std::vector<uint8_t> rx;
    uint32_t sentConf = 0, sentPins = 0, loggedPins = 0;
};
static SpiSlave                  g_spis[2];
static const QemuMemoryRegionOps* g_spiOrig[2];
static void*                     g_spiOrigOpaque[2];
static QemuMemoryRegionOps       g_spisOps;
static QemuMemoryRegionStorage   g_spisMr[4];
static void (*q_phys_rw)(uint64_t addr, void* buf, uint64_t len, bool isWrite) = nullptr;   // cpu_physical_memory_rw

static uint8_t* spiState(int h) { return (uint8_t*)g_soc + OFS_SPI0 + (h + 2) * SPI_STRIDE; }
static bool spisActive(int h) { return (g_spis[h].r[0x38 / 4] >> 30) & 1; }

static void spisIrq(int h)
{
    uint32_t sl = g_spis[h].r[0x38 / 4];
    if (spisActive(h)) q_set_irq(*(void**)(spiState(h) + OFS_SPI_IRQ), ((sl >> 4) & 1) && ((sl >> 9) & 1) ? 1 : 0);
}

// SPI mode from what spi_ll_slave_set_mode() wrote (it uses other edge settings with DMA)
static int spisMode(int h)
{
    const uint32_t* r = g_spis[h].r;
    int idle = (r[0x34 / 4] >> 29) & 1, iEdge = (r[0x1C / 4] >> 6) & 1, mosiNum = (r[0x14 / 4] >> 23) & 7;
    if (mosiNum == 3) return idle ? 2 : 0;                  // DMA timing of modes 0 / 2
    if (idle && !iEdge) return 0;
    if (idle && iEdge) return 1;
    if (!idle && iEdge) return 2;
    return 3;
}

// board pins of the slave: GPIO matrix or the IO MUX pads (function 1)
static int spisPinIn(int sig, int gpioNative)
{
    uint32_t v = g_inSel ? g_inSel[sig] : 0;
    if (v & 0x80) { int g = (int)(v & 0x3F); return g < 40 ? g_gpioToBoard[g] : 0; }
    return muxFn(gpioNative) == 1 ? g_gpioToBoard[gpioNative] : 0;
}
static int spisPinOut(int sig, int gpioNative)
{
    if (muxFn(gpioNative) == 1) return g_gpioToBoard[gpioNative];
    for (int g = 0; g < 40; g++)
        if (g_gpioToBoard[g] && muxFn(g) == 2 && (int)(g_outSel[g] & 0x1FF) == sig) return g_gpioToBoard[g];
    return 0;
}

// every slice and at GPIO matrix changes: tell the model the slave and its pins
static void spisRoute()
{
    if (!g_inSel || !g_outSel) return;
    for (int h = 0; h < 2; h++)
    {
        SpiSlave& S = g_spis[h];
        uint32_t on = spisActive(h) ? 1 : 0, pins = 0;
        if (on)
            pins = h == 0 ? pack4(spisPinIn(8, 14), spisPinIn(10, 13), spisPinOut(9, 12), spisPinIn(11, 15))
                          : pack4(spisPinIn(63, 18), spisPinIn(65, 23), spisPinOut(64, 19), spisPinIn(68, 5));
        if (on == S.sentConf && pins == S.sentPins) continue;
        bool logIt = on != S.sentConf || ((pins & 0xFF) && (pins & 0xFF00) && (pins & 0xFF0000) && (pins >> 24) && pins != S.loggedPins);
        S.sentConf = on;
        S.sentPins = pins;
        toModel(E32_EV_SPIS_CONF, (uint16_t)h, on, pins);
        if (!logIt) continue;                           // (CS is tied high for a moment between DMA transactions)
        if (on) S.loggedPins = pins;
        elog("%s slave %s, pins (board pins) CLK %u MOSI %u MISO %u CS %u", h ? "VSPI" : "HSPI", on ? "on" : "off",
             pins & 0xFF, (pins >> 8) & 0xFF, (pins >> 16) & 0xFF, pins >> 24);
    }
}

static void physRead(uint32_t addr, void* buf, uint32_t len) { if (q_phys_rw) q_phys_rw(addr, buf, len, false); }
static void physWrite(uint32_t addr, const void* buf, uint32_t len) { if (q_phys_rw) q_phys_rw(addr, (void*)buf, len, true); }

// the firmware set CMD.usr: collect the MISO bytes and send them to the model
static void spisArm(int h)
{
    SpiSlave& S = g_spis[h];
    uint32_t txBits = (S.r[0x48 / 4] & 0xFFFFFF) + 1;
    size_t want = (txBits + 7) / 8;
    std::vector<uint8_t> tx;
    if (S.outStart)
    {
        uint32_t d = 0x3FF00000u | (S.r[0x104 / 4] & 0xFFFFF);
        for (int guard = 0; d && guard < 256 && tx.size() < want; guard++)
        {
            uint32_t desc[3];
            physRead(d, desc, sizeof(desc));
            uint32_t len = (desc[0] >> 12) & 0xFFF;
            std::vector<uint8_t> chunk(len);
            if (len) physRead(desc[1], chunk.data(), len);
            tx.insert(tx.end(), chunk.begin(), chunk.end());
            if ((desc[0] >> 30) & 1) break;                // eof
            d = desc[2];
        }
    }
    else
        for (size_t i = 0; i < 64; i++) tx.push_back((uint8_t)(S.r[0x80 / 4 + i / 4] >> (8 * (i % 4))));
    if (tx.size() > want) tx.resize(want);
    int mode = spisMode(h);
    uint32_t txLsb = (S.r[0x08 / 4] >> 26) & 1, rxLsb = (S.r[0x08 / 4] >> 25) & 1;
    toModel(E32_EV_SPIS_TX, (uint16_t)h, 0x100u | ((uint32_t)mode << 9) | (txLsb << 11) | (rxLsb << 12));
    for (uint8_t b : tx) toModel(E32_EV_SPIS_TX, (uint16_t)h, b);
    S.armed = true;
    S.rx.clear();
    static int logged;
    if (logged < 5) { logged++; elog("%s slave: transaction armed, mode %d, %u TX bytes%s, RX %u bits%s", h ? "VSPI" : "HSPI", mode,
                                     (unsigned)tx.size(), S.outStart ? " (DMA)" : "", (S.r[0x4C / 4] & 0xFFFFFF) + 1, S.inStart ? " (DMA)" : ""); }
}

// CS went high: received bytes into the IN chain / data_buf, transaction done
static void spisEnd(int h, uint32_t bits)
{
    SpiSlave& S = g_spis[h];
    if (!S.armed) return;                                 // nothing armed: the bytes are lost (as on the chip)
    uint32_t rxBits = (S.r[0x4C / 4] & 0xFFFFFF) + 1;
    size_t n = S.rx.size();
    if (n > (rxBits + 7) / 8) n = (rxBits + 7) / 8;
    if (S.inStart)
    {
        uint32_t d = 0x3FF00000u | (S.r[0x108 / 4] & 0xFFFFF);
        size_t done = 0;
        for (int guard = 0; d && guard < 256; guard++)
        {
            uint32_t desc[3];
            physRead(d, desc, sizeof(desc));
            uint32_t size = desc[0] & 0xFFF, len = (uint32_t)std::min<size_t>(size, n - done);
            if (len) physWrite(desc[1], S.rx.data() + done, len);
            done += len;
            bool last = done >= n || !desc[2];
            desc[0] = (desc[0] & ~(0xFFFu << 12) & ~(1u << 31) & ~(1u << 30)) | (len << 12) | (last ? 1u << 30 : 0);
            physWrite(d, desc, 4);
            if (last) break;
            d = desc[2];
        }
        S.r[0x114 / 4] |= (1u << 5) | (1u << 3);          // in_suc_eof, in_done
    }
    else
        for (size_t i = 0; i < 64; i++)
        {
            uint32_t& w = S.r[0x80 / 4 + i / 4];
            uint8_t b = i < n ? S.rx[i] : (uint8_t)(w >> (8 * (i % 4)));
            w = (w & ~(0xFFu << (8 * (i % 4)))) | ((uint32_t)b << (8 * (i % 4)));
        }
    if (bits > rxBits) bits = rxBits;
    S.r[0x64 / 4] = bits ? bits - 1 : 0;                  // the chip reports bits - 1 (the HAL adds 1 back)
    S.r[0x00 / 4] &= ~(1u << 18);                         // usr done
    S.r[0x38 / 4] |= 1u << 4;                             // trans_done
    S.armed = S.outStart = S.inStart = false;
    spisIrq(h);
}

static void spisEvent(int h, int op, int data, uint32_t bits)
{
    if (h > 1 || !spisActive(h)) return;
    if (op == 1) { if (g_spis[h].rx.size() < 65536) g_spis[h].rx.push_back((uint8_t)data); }
    else if (op == 3) spisEnd(h, bits);
}

static uint64_t spis_read(void* opaque, uint64_t addr, unsigned size)
{
    int h = (int)(uintptr_t)opaque & 1;
    if (!spisActive(h) || addr >= 0x400) return g_spiOrig[h]->read(g_spiOrigOpaque[h], addr, size);
    return g_spis[h].r[addr / 4];
}

static void spis_write(void* opaque, uint64_t addr, uint64_t data, unsigned size)
{
    int h = (int)(uintptr_t)opaque & 1;
    uint32_t v = (uint32_t)data;
    if (addr >= 0x400) { g_spiOrig[h]->write(g_spiOrigOpaque[h], addr, data, size); return; }
    SpiSlave& S = g_spis[h];
    bool was = spisActive(h);
    S.r[addr / 4] = v;
    bool now = spisActive(h);
    if (!now)                                            // master (or just left slave mode): QEMU's job
    {
        g_spiOrig[h]->write(g_spiOrigOpaque[h], addr, data, size);
        if (was) spisRoute();
        return;
    }
    if (!was) { S.armed = S.outStart = S.inStart = false; spisRoute(); }
    switch (addr)
    {
    case 0x00:                                           // CMD: usr = start a transaction
        if (v & (1u << 18)) spisArm(h);
        return;
    case 0x38:
        if (v & (1u << 31)) S.armed = false;             // sync reset
        spisIrq(h);
        return;
    case 0x104: if (v & (1u << 29)) S.outStart = true; return;
    case 0x108: if (v & (1u << 29)) S.inStart = true; return;
    case 0x11C: S.r[0x114 / 4] &= ~v; return;
    default:
        if (addr >= 0x80 && addr < 0xC0) return;         // data_buf: ours in slave mode
        if (addr >= 0x100 && addr < 0x120) return;       // DMA: ours in slave mode
        g_spiOrig[h]->write(g_spiOrigOpaque[h], addr, data, size);   // timing / pin settings: keep QEMU in step
        return;
    }
}

static void spisReset()
{
    for (int h = 0; h < 2; h++) g_spis[h] = SpiSlave();  // the model forgets the slave at a reset
}

static void installSpiSlave(HMODULE q)
{
    q_phys_rw = (void (*)(uint64_t, void*, uint64_t, bool))GetProcAddress(q, "cpu_physical_memory_rw");
    if (!g_soc || !q_phys_rw) { elog("SPI slave: QEMU functions missing - SPI slave off"); return; }
    memset(&g_spisOps, 0, sizeof(g_spisOps));
    g_spisOps.read = spis_read;
    g_spisOps.write = spis_write;
    g_spisOps.endianness = 2;
    g_spisOps.valid_min_access_size = 1;
    g_spisOps.valid_max_access_size = 4;
    g_spisOps.impl_min_access_size = 4;
    g_spisOps.impl_max_access_size = 4;
    uint64_t size = 0;
    for (int h = 0; h < 2; h++)
    {
        uint8_t* mr = spiState(h) + OFS_SPI_IOMEM;
        g_spiOrig[h] = *(const QemuMemoryRegionOps**)(mr + OFS_MR_OPS);
        g_spiOrigOpaque[h] = *(void**)(mr + OFS_MR_OPAQUE);
        size = *(const uint64_t*)(mr + OFS_MR_SIZE);
        if (!g_spiOrig[h] || !g_spiOrig[h]->read || !g_spiOrig[h]->write || size < 0x400 || size > 0x10000)
        {
            elog("SPI slave: QEMU's SPI block looks different (other QEMU build) - SPI slave off");
            return;
        }
    }
    spisReset();
    void* sys = q_get_system_memory();
    const uint64_t base[4] = { 0x3FF64000ull, 0x3FF65000ull, 0x60024000ull, 0x60025000ull };
    for (int k = 0; k < 4; k++)
    {
        memset(&g_spisMr[k], 0, sizeof(g_spisMr[k]));
        q_mr_init_io(&g_spisMr[k], nullptr, &g_spisOps, (void*)(uintptr_t)(k & 1), k < 2 ? "esp32link.spi" : "esp32link.spi.alias", size);
        q_mr_add_overlap(sys, base[k], &g_spisMr[k], 10);
    }
    elog("SPI slave support installed (QEMU SPI block 0x%llX bytes)", (unsigned long long)size);
}

// QEMU puts a TMP105 temperature sensor at 0x48 on I2C0. Real parts (LM75,
// PCF8591, ADS1115) use 0x48 too, so move it to 0x7F (I2CSlave.address, +160).
typedef void* (*object_resolve_path_type_fn)(const char* path, const char* type, bool* ambiguous);
// ===========================================================================
//  I2S (link version 14). Not in QEMU at all: the engine is the whole block.
//  I2S0 at 0x3FF4F000, I2S1 at 0x3FF6D000 (also 0x6000F000 / 0x6002D000), 0x100 bytes
//  (i2s_struct.h), interrupt matrix sources 32 / 33. TX and RX as master (the ESP32 makes
//  BCK / WS); in full duplex RX uses the TX clock (the IDF shares BCK / WS).
//    0x08 CONF: 0 tx_reset, 1 rx_reset, 4 tx_start, 5 rx_start, 10 / 11 tx / rx_msb_shift
//         (Philips), 12 / 13 tx / rx_short_sync (PCM), 17 rx_msb_right
//    0x0C-0x18 INT_RAW / ST / ENA / CLR: 8 in_done, 9 in_suc_eof, 11 out_done, 12 out_eof,
//         15 in_dscr_empty, 16 out_total_eof
//    0x20 FIFO_CONF: 12 dscr_en, 13-15 tx_fifo_mod / 16-18 rx_fifo_mod (0 16-bit stereo,
//         1 16-bit mono, 2 32-bit stereo, 3 32-bit mono); 0x24 RXEOF_NUM (words); 0x28 single
//         data; 0x2C CONF_CHAN: 0-2 tx_chan_mod, 3-4 rx_chan_mod
//    0x30 / 0x34 OUT_LINK / IN_LINK: 0-19 descriptor address, 28 stop, 29 start;
//         0x38 OUT_EOF_DES_ADDR, 0x3C IN_EOF_DES_ADDR, 0x40 OUT_EOF_BFR_DES_ADDR;
//         0x60 LC_CONF: 0 in_rst, 1 out_rst, 6 out_auto_wrback
//    0xAC CLKM_CONF: N 0-7, b 8-13, a 14-19, clka_en 21 (APLL); 0xB0 SAMPLE_RATE_CONF:
//         tx_bck_div_num 0-5, rx_bck_div_num 6-11, tx_bits_mod 12-17, rx_bits_mod 18-23
//  Clock: MCLK = 160 MHz / (N + b / a), BCK = MCLK / bck_div, a frame = 2 x bits_mod bits
//  (the IDF default: MCLK = 256 x rate). APLL is not simulated (160 MHz). The clock runs while
//  tx_start or rx_start is set; with tx_start it uses the TX fields and pins, RX alone the
//  RX fields and the RX clock pins (I2S0I / I2S1I BCK / WS out).
//  TX DMA: OUT_LINK start loads the descriptor chain (lldesc: size 0-11, length 12-23, eof 30,
//  owner 31; buffer; next). At the end of a descriptor: out_done, owner cleared (auto write
//  back) and with eof: out_eof + OUT_EOF_DES_ADDR; next = 0: out_total_eof, DMA stops (the
//  clock runs on with zeros while tx_start is set, like the chip's empty FIFO).
//  RX DMA: IN_LINK start; every received frame (E32_EV_I2S_RX from the model: DIN read at the
//  BCK rising edges) is written into the current descriptor's buffer; when it is full or
//  RXEOF_NUM words have arrived: length + eof + owner = CPU written back, in_done and
//  in_suc_eof + IN_EOF_DES_ADDR; next = 0: in_dscr_empty, RX DMA stops.
//  The IDF ISRs clear the status first and then read the EOF_DES_ADDR register, so
//  descriptors that ended in one slice are handed over one per interrupt: each INT_CLR
//  raises the bit again and the NEXT clear shows the next address.
//  Frames: exact start times anchor + k x frame, computed up to "now" at every register write
//  and at the slice boundary (before 'reached', like MCPWM); each frame goes to the model
//  (E32_EV_I2S_DATA), which plays BCK / WS / DOUT on the pins and reads DIN.
// ===========================================================================
enum {
    I2S_CONF = 0x08 / 4, I2S_INT_RAW = 0x0C / 4, I2S_INT_ST = 0x10 / 4, I2S_INT_ENA = 0x14 / 4, I2S_INT_CLR = 0x18 / 4,
    I2S_FIFO_CONF = 0x20 / 4, I2S_RXEOF_NUM = 0x24 / 4, I2S_SINGLE = 0x28 / 4, I2S_CHAN = 0x2C / 4,
    I2S_OUT_LINK = 0x30 / 4, I2S_IN_LINK = 0x34 / 4, I2S_OUT_EOF_DES = 0x38 / 4, I2S_IN_EOF_DES = 0x3C / 4,
    I2S_OUT_EOF_BFR = 0x40 / 4, I2S_INLINK_DSCR = 0x48 / 4, I2S_OUTLINK_DSCR = 0x54 / 4, I2S_LC_CONF = 0x60 / 4,
    I2S_CONF2 = 0xA8 / 4, I2S_CLKM = 0xAC / 4, I2S_RATE = 0xB0 / 4, I2S_PDM = 0xB4 / 4, I2S_STATE = 0xBC / 4,
    I2S_DATE = 0xFC / 4, I2S_NREG = 0x100 / 4
};
enum { I2S_IN_DONE = 8, I2S_IN_SUC_EOF = 9, I2S_OUT_DONE = 11, I2S_OUT_EOF = 12, I2S_IN_EMPTY = 15, I2S_OUT_TEOF = 16, I2S_SRC0 = 32 };

// descriptors handed to the firmware one per interrupt (see above)
struct I2sEofQ
{
    std::deque<uint32_t> q;         // finished descriptors not shown yet
    bool loadOnClear = false;       // the next clear shows q.front()
};

struct I2sPort
{
    uint32_t r[I2S_NREG];
    void* irq = nullptr;
    bool running = false;           // tx_start or rx_start: the clock runs, frames go out
    bool tx = false, rx = false;    // which side runs the clock (tx wins)
    int64_t anchorNs = 0;           // start of frame 0
    double frameNs = 0;
    int64_t frame = 0;              // next frame
    uint32_t bitPs = 0;             // BCK period
    uint32_t fmt = 0;               // bits | 0x100 Philips | 0x200 short sync | 0x400 RX on
    // TX DMA
    bool dma = false;
    uint32_t desc = 0, descW0 = 0, descNext = 0;
    std::vector<uint8_t> buf;
    size_t pos = 0;
    I2sEofQ outQ;
    // RX DMA
    bool inDma = false;
    uint32_t inDesc = 0, inW0 = 0, inBuf = 0, inNext = 0;
    uint32_t inPos = 0, inSize = 0, inSinceEof = 0;
    I2sEofQ inQ;
    uint32_t pinsSent = 0, bitPsSent = 0, fmtSent = 0;
    bool confSent = false;
    bool warned = false, rxLost = false;
};
static I2sPort                 g_i2s[2];
static QemuMemoryRegionOps     g_i2sOps;
static QemuMemoryRegionStorage g_i2sMr[4];

static void i2sUpdateIrq(I2sPort& P)
{
    if (P.irq) q_set_irq(P.irq, (P.r[I2S_INT_RAW] & P.r[I2S_INT_ENA]) ? 1 : 0);
}

static void i2sRaise(I2sPort& P, int bit) { P.r[I2S_INT_RAW] |= 1u << bit; }

// a finished descriptor for the firmware: shown at once, or queued while the bit is still set
static void i2sEofPush(I2sPort& P, I2sEofQ& Q, int bit, int desReg, uint32_t desc)
{
    if (!((P.r[I2S_INT_RAW] >> bit) & 1) && Q.q.empty())
    {
        if (desReg == I2S_OUT_EOF_DES) P.r[I2S_OUT_EOF_BFR] = P.r[I2S_OUT_EOF_DES];
        P.r[desReg] = desc;
        i2sRaise(P, bit);
    }
    else if (Q.q.size() < 4096) Q.q.push_back(desc);
}

// INT_CLR of that bit: the next queued descriptor (clear first, read after - the IDF order)
static void i2sEofClear(I2sPort& P, I2sEofQ& Q, int bit, int desReg)
{
    if (Q.loadOnClear && !Q.q.empty())
    {
        if (desReg == I2S_OUT_EOF_DES) P.r[I2S_OUT_EOF_BFR] = P.r[I2S_OUT_EOF_DES];
        P.r[desReg] = Q.q.front();
        Q.q.pop_front();
    }
    Q.loadOnClear = false;
    if (!Q.q.empty()) { i2sRaise(P, bit); Q.loadOnClear = true; }
}

// clock and format from the registers (TX fields while TX runs, else RX)
static void i2sTiming(I2sPort& P)
{
    uint32_t c = P.r[I2S_CLKM];
    double n = (double)(c & 0xFF);
    if (n == 0) n = 256;
    uint32_t b = (c >> 8) & 0x3F, a = (c >> 14) & 0x3F;
    double div = n + (a ? (double)b / a : 0.0);
    bool useTx = P.tx || !P.rx;
    uint32_t bck = useTx ? (P.r[I2S_RATE] & 0x3F) : ((P.r[I2S_RATE] >> 6) & 0x3F);
    if (bck == 0) bck = 64;
    double bckHz = 160e6 / div / bck;
    int bits = (int)((P.r[I2S_RATE] >> (useTx ? 12 : 18)) & 0x3F);
    if (!bits) bits = 16;
    uint32_t conf = P.r[I2S_CONF];
    P.frameNs = 2.0 * bits * 1e9 / bckHz;
    P.bitPs = (uint32_t)llround(1e12 / bckHz);
    P.fmt = (uint32_t)bits | (((conf >> (useTx ? 10 : 11)) & 1) << 8) | (((conf >> (useTx ? 12 : 13)) & 1) << 9) | (P.rx ? 0x400u : 0u);
}

// ---- TX DMA ----
// the descriptor at 'addr' becomes the current one (its data is read now)
static bool i2sLoad(I2sPort& P, uint32_t addr)
{
    P.desc = addr;
    P.buf.clear();
    P.pos = 0;
    if (!addr || (addr & 3)) { P.dma = false; P.desc = 0; return false; }
    uint32_t w[3];
    physRead(addr, w, sizeof(w));
    P.descW0 = w[0];
    P.descNext = w[2];
    uint32_t len = (w[0] >> 12) & 0xFFF;
    P.buf.resize(len);
    if (len) physRead(w[1], P.buf.data(), len);
    P.r[I2S_OUTLINK_DSCR] = addr;
    return true;
}

// the current descriptor is played out: write back, interrupts, on to the next one
static void i2sFinish(I2sPort& P)
{
    if ((P.r[I2S_LC_CONF] >> 6) & 1)                       // auto write back: owner = CPU
    {
        uint32_t w0 = P.descW0 & ~0x80000000u;
        physWrite(P.desc, &w0, 4);
    }
    i2sRaise(P, I2S_OUT_DONE);
    if ((P.descW0 >> 30) & 1) i2sEofPush(P, P.outQ, I2S_OUT_EOF, I2S_OUT_EOF_DES, P.desc);   // the driver gets it back
    if (!P.descNext) { i2sRaise(P, I2S_OUT_TEOF); P.dma = false; P.desc = 0; P.buf.clear(); P.pos = 0; return; }
    i2sLoad(P, P.descNext);
}

// n bytes of the TX stream (zeros when the DMA has nothing)
static void i2sTake(I2sPort& P, uint8_t* out, int n)
{
    for (int k = 0; k < n; k++)
    {
        for (int guard = 0; P.dma && P.pos >= P.buf.size() && guard < 64; guard++) i2sFinish(P);   // empty descriptors
        if (!P.dma || P.pos >= P.buf.size()) { memset(out + k, 0, (size_t)(n - k)); return; }
        out[k] = P.buf[P.pos++];
        if (P.pos >= P.buf.size()) i2sFinish(P);
    }
}

// left / right of the next TX frame, left-aligned to 32 bits (zeros while only RX runs)
static void i2sFrame(I2sPort& P, uint32_t* l, uint32_t* r)
{
    *l = *r = 0;
    if (!P.tx) return;
    int mode = (int)((P.r[I2S_FIFO_CONF] >> 13) & 7);
    uint8_t d[8];
    uint32_t a = 0, b = 0;
    switch (mode)
    {
    case 1:  i2sTake(P, d, 2); a = b = (uint32_t)(d[0] | d[1] << 8) << 16; break;                       // 16-bit mono
    case 2:  i2sTake(P, d, 8); memcpy(&a, d, 4); memcpy(&b, d + 4, 4); break;                          // 32-bit stereo
    case 3:  i2sTake(P, d, 4); memcpy(&a, d, 4); b = a; break;                                         // 32-bit mono
    default: i2sTake(P, d, 4); a = (uint32_t)(d[0] | d[1] << 8) << 16; b = (uint32_t)(d[2] | d[3] << 8) << 16; break;   // 16-bit stereo: L, R
    }
    int chan = (int)(P.r[I2S_CHAN] & 7);                   // mono variants: one side holds the single data
    uint32_t single = P.r[I2S_SINGLE];
    if (mode & 1)
    {
        if (chan == 3) a = single;                         // data on the right only
        else if (chan == 4) b = single;                    // data on the left only
    }
    *l = a;
    *r = b;
}

// ---- RX DMA ----
static bool i2sInLoad(I2sPort& P, uint32_t addr)
{
    P.inDesc = addr;
    P.inPos = 0;
    if (!addr || (addr & 3)) { P.inDma = false; P.inDesc = 0; return false; }
    uint32_t w[3];
    physRead(addr, w, sizeof(w));
    P.inW0 = w[0];
    P.inBuf = w[1];
    P.inNext = w[2];
    P.inSize = w[0] & 0xFFF;
    if (!P.inSize) P.inSize = 4096;
    P.r[I2S_INLINK_DSCR] = addr;
    return true;
}

// the current RX descriptor is done: length, eof, owner = CPU, interrupts, the next one
static void i2sInFinish(I2sPort& P, bool eof)
{
    uint32_t w0 = (P.inW0 & 0x00000FFFu) | ((P.inPos & 0xFFF) << 12) | (eof ? 0x40000000u : 0u);   // owner 0 = CPU
    physWrite(P.inDesc, &w0, 4);
    i2sRaise(P, I2S_IN_DONE);
    if (eof) i2sEofPush(P, P.inQ, I2S_IN_SUC_EOF, I2S_IN_EOF_DES, P.inDesc);
    if (!P.inNext) { i2sRaise(P, I2S_IN_EMPTY); P.inDma = false; P.inDesc = 0; return; }
    i2sInLoad(P, P.inNext);
}

// received bytes into the IN chain
static void i2sInPut(I2sPort& P, const uint8_t* d, int n)
{
    uint32_t eofBytes = P.r[I2S_RXEOF_NUM] * 4;
    for (int k = 0; k < n && P.inDma; )
    {
        uint32_t room = P.inSize - P.inPos;
        uint32_t take = (uint32_t)(n - k) < room ? (uint32_t)(n - k) : room;
        if (eofBytes && eofBytes - P.inSinceEof < take) take = eofBytes - P.inSinceEof;
        if (take) physWrite(P.inBuf + P.inPos, d + k, take);
        P.inPos += take;
        P.inSinceEof += take;
        k += (int)take;
        bool eof = eofBytes ? P.inSinceEof >= eofBytes : P.inPos >= P.inSize;
        if (eof) P.inSinceEof = 0;
        if (eof || P.inPos >= P.inSize) i2sInFinish(P, eof);
    }
}

// a frame the model read from DIN (left / right left-aligned to 32 bits)
static void i2sRxFrame(int p, uint32_t l, uint32_t r)
{
    I2sPort& P = g_i2s[p];
    if (!P.rx) return;
    if (!P.inDma)
    {
        if (!P.rxLost) { P.rxLost = true; elog("I2S%d RX: frames received while the RX DMA is not running - dropped", p); }
        return;
    }
    int mode = (int)((P.r[I2S_FIFO_CONF] >> 16) & 7);
    uint8_t d[8];
    int n = 4;
    uint32_t conf = P.r[I2S_CONF];
    bool msbRight = (conf >> 17) & 1;
    int chan = (int)((P.r[I2S_CHAN] >> 3) & 3);
    bool left = (chan == 2) == msbRight;                   // mono: which side (the IDF swaps 1 / 2 with msb_right)
    uint32_t m = left ? l : r;
    switch (mode)
    {
    case 1:  { uint16_t s = (uint16_t)(m >> 16); memcpy(d, &s, 2); n = 2; break; }                       // 16-bit mono
    case 2:  memcpy(d, &l, 4); memcpy(d + 4, &r, 4); n = 8; break;                                     // 32-bit stereo
    case 3:  memcpy(d, &m, 4); n = 4; break;                                                           // 32-bit mono
    default: { uint16_t a = (uint16_t)(l >> 16), b = (uint16_t)(r >> 16); memcpy(d, &a, 2); memcpy(d + 2, &b, 2); n = 4; break; }   // 16-bit stereo: L, R
    }
    i2sInPut(P, d, n);
    i2sUpdateIrq(P);
}

// ---- pins / clock to the model ----
static int i2sOutPin(int p, int k)        // k: 0 BCK, 1 WS, 2 DOUT (TX signals)
{
    static const int SIG[2][3] = { { 23, 25, 163 }, { 24, 26, 189 } };
    return outPinFor(SIG[p][k]);
}

// tell the model the pins / clock / format when they change
static void i2sConf(int p, int64_t at)
{
    I2sPort& P = g_i2s[p];
    uint32_t pins = 0;
    if (P.running)
    {
        static const int RXCLK[2][2] = { { 27, 28 }, { 164, 165 } };   // I2S0I / I2S1I BCK, WS (RX master clock out)
        static const int DIN[2] = { 155, 181 };                        // I2S0I_DATA_IN15 / I2S1I_DATA_IN15
        int bck = P.tx ? i2sOutPin(p, 0) : outPinFor(RXCLK[p][0]);
        int ws = P.tx ? i2sOutPin(p, 1) : outPinFor(RXCLK[p][1]);
        if (!bck) bck = i2sOutPin(p, 0);                               // full duplex: the TX clock pins
        if (!ws) ws = i2sOutPin(p, 1);
        pins = pack4(bck, ws, P.tx ? i2sOutPin(p, 2) : 0, P.rx ? inPinFor(DIN[p]) : 0);
    }
    uint32_t bitPs = pins ? P.bitPs : 0;
    if (P.confSent && pins == P.pinsSent && bitPs == P.bitPsSent && P.fmt == P.fmtSent) return;
    if (!P.confSent && !bitPs) return;
    P.pinsSent = pins;
    P.bitPsSent = bitPs;
    P.fmtSent = P.fmt;
    P.confSent = bitPs != 0;
    toModelAt(at, E32_EV_I2S_CONF, (uint16_t)p, pins, bitPs, P.fmt);
    if (bitPs)
        elog("I2S%d %s: %.1f Hz (BCK %.0f Hz), %u bits, %s%s, FIFO mode %u, board pins BCK %u WS %u DOUT %u DIN %u", p,
             P.tx && P.rx ? "TX + RX" : P.tx ? "TX" : "RX", 1e9 / P.frameNs, 1e12 / bitPs, P.fmt & 0x3F,
             (P.fmt & 0x200) ? "PCM short sync" : (P.fmt & 0x100) ? "Philips" : "MSB",
             ((P.r[I2S_CLKM] >> 21) & 1) ? " (APLL: not simulated, taken as 160 MHz)" : "",
             (P.r[I2S_FIFO_CONF] >> (P.tx ? 13 : 16)) & 7, pins & 0xFF, (pins >> 8) & 0xFF, (pins >> 16) & 0xFF, pins >> 24);
    else
        elog("I2S%d off", p);
}

// every frame that starts up to 'to'
static void i2sRun(int p, int64_t to)
{
    I2sPort& P = g_i2s[p];
    if (!P.running || P.frameNs <= 0) return;
    for (int guard = 0; guard < 1000000; guard++)
    {
        int64_t at = P.anchorNs + (int64_t)llround((double)P.frame * P.frameNs);
        if (at > to) break;
        uint32_t l, r;
        i2sFrame(P, &l, &r);
        if (P.confSent) toModelAt(at, E32_EV_I2S_DATA, (uint16_t)p, l, r);
        P.frame++;
    }
    i2sUpdateIrq(P);
}

// new clock / format / direction while running: the next frame starts with the new timing
static void i2sReEpoch(int p, int64_t now)
{
    I2sPort& P = g_i2s[p];
    uint32_t oldPs = P.bitPs, oldFmt = P.fmt;
    double oldFrame = P.frameNs;
    i2sTiming(P);
    if (!P.running || (P.bitPs == oldPs && P.fmt == oldFmt && P.frameNs == oldFrame)) return;
    int64_t next = P.anchorNs + (int64_t)llround((double)P.frame * oldFrame);
    P.anchorNs = next > now ? next : now;
    P.frame = 0;
    i2sConf(p, P.anchorNs);
}

// tx_start / rx_start changed
static void i2sStartStop(int p, int64_t now)
{
    I2sPort& P = g_i2s[p];
    bool tx = (P.r[I2S_CONF] >> 4) & 1, rx = (P.r[I2S_CONF] >> 5) & 1;
    bool was = P.running;
    P.tx = tx;
    P.rx = rx;
    P.running = tx || rx;
    if (P.running && !was)
    {
        i2sTiming(P);
        P.anchorNs = now;
        P.frame = 0;
        P.rxLost = false;
        if (!P.warned && ((tx && (P.r[I2S_PDM] & 1)) || (rx && (P.r[I2S_PDM] & 2))))   // tx_pdm_en / rx_pdm_en
        {                                                  // (CONF2 is not a reliable sign: the IDF leaves 0x38 there in std mode)
            P.warned = true;
            elog("I2S%d: PDM mode is not simulated (standard I2S only) - PDM_CONF 0x%08X", p, P.r[I2S_PDM]);
        }
        i2sConf(p, now);
    }
    else if (!P.running && was) i2sConf(p, now);
    else if (P.running) { i2sReEpoch(p, now); i2sConf(p, now); }   // the other direction joined / left
}

static void i2sDmaReset(I2sPort& P)
{
    P.dma = false;
    P.desc = 0;
    P.buf.clear();
    P.pos = 0;
    P.outQ = I2sEofQ();
}

static void i2sInReset(I2sPort& P)
{
    P.inDma = false;
    P.inDesc = 0;
    P.inPos = P.inSinceEof = 0;
    P.inQ = I2sEofQ();
}

static void i2sReset()
{
    int64_t now = now_ns();
    for (int p = 0; p < 2; p++)
    {
        I2sPort& P = g_i2s[p];
        if (P.running) { P.running = P.tx = P.rx = false; i2sConf(p, now); }   // the model releases the pins
        void* irq = P.irq;
        P = I2sPort();
        P.irq = irq;
        memset(P.r, 0, sizeof(P.r));
        P.r[I2S_CLKM] = 4;                                 // reset values (TRM)
        P.r[I2S_RATE] = (16u << 18) | (16u << 12) | (6u << 6) | 6u;
        P.r[I2S_DATE] = 0x1604201;
        i2sUpdateIrq(P);
    }
}

// slice boundary: frames up to now, pin routing
static void updateI2s()
{
    int64_t now = now_ns();
    for (int p = 0; p < 2; p++)
    {
        i2sRun(p, now);
        if (g_i2s[p].running) i2sConf(p, now);             // routing may have changed (GPIO matrix)
    }
}

static uint64_t i2s_read(void* opaque, uint64_t addr, unsigned)
{
    int p = (int)(uintptr_t)opaque;
    I2sPort& P = g_i2s[p];
    int i = (int)(addr / 4);
    if (i >= I2S_NREG) return 0;
    i2sRun(p, now_ns());
    if (i == I2S_INT_ST) return P.r[I2S_INT_RAW] & P.r[I2S_INT_ENA];
    if (i == I2S_INT_CLR) return 0;
    if (i == I2S_STATE) return (P.tx && P.dma) ? 0u : 1u;   // tx_idle
    return P.r[i];
}

static void i2s_write(void* opaque, uint64_t addr, uint64_t data, unsigned)
{
    int p = (int)(uintptr_t)opaque;
    I2sPort& P = g_i2s[p];
    int i = (int)(addr / 4);
    uint32_t v = (uint32_t)data;
    if (i >= I2S_NREG) return;
    int64_t now = now_ns();
    i2sRun(p, now);                                       // everything before this write with the old values
    uint32_t old = P.r[i];
    switch (i)
    {
    case 0: case 1:                                       // FIFO data (CPU mode, no DMA): not simulated
        if (!P.warned) { P.warned = true; elog("I2S%d: FIFO access by the CPU (no DMA) is not simulated", p); }
        return;
    case I2S_INT_CLR:
        P.r[I2S_INT_RAW] &= ~v;
        if ((v >> I2S_OUT_EOF) & 1) i2sEofClear(P, P.outQ, I2S_OUT_EOF, I2S_OUT_EOF_DES);
        if ((v >> I2S_IN_SUC_EOF) & 1) i2sEofClear(P, P.inQ, I2S_IN_SUC_EOF, I2S_IN_EOF_DES);
        i2sUpdateIrq(P);
        return;
    case I2S_INT_RAW: case I2S_INT_ST: case I2S_OUT_EOF_DES: case I2S_IN_EOF_DES: case I2S_OUT_EOF_BFR:
    case I2S_INLINK_DSCR: case I2S_OUTLINK_DSCR: case I2S_STATE: case I2S_DATE:
        return;
    case I2S_INT_ENA:
        P.r[i] = v;
        i2sUpdateIrq(P);
        return;
    case I2S_OUT_LINK:
        P.r[i] = v & 0x000FFFFFu;
        if ((v >> 28) & 1) P.dma = false;                 // stop
        if ((v >> 29) & 1)                                // start: the chain from this descriptor
        {
            i2sDmaReset(P);
            P.dma = true;
            i2sLoad(P, 0x3FF00000u | (v & 0xFFFFF));
        }
        return;
    case I2S_IN_LINK:
        P.r[i] = v & 0x000FFFFFu;
        if ((v >> 28) & 1) P.inDma = false;
        if ((v >> 29) & 1)
        {
            i2sInReset(P);
            P.inDma = true;
            i2sInLoad(P, 0x3FF00000u | (v & 0xFFFFF));
        }
        return;
    case I2S_LC_CONF:
        P.r[i] = v;
        if ((v >> 1) & 1) i2sDmaReset(P);                 // out_rst
        if (v & 1) i2sInReset(P);                         // in_rst
        return;
    case I2S_CONF:
        P.r[i] = v;
        if ((v & 3) && P.running) { P.anchorNs = now; P.frame = 0; }   // tx / rx reset: the frame starts again
        if (((v ^ old) >> 4) & 3) i2sStartStop(p, now);
        else i2sReEpoch(p, now);                          // Philips / short sync bits
        return;
    case I2S_CLKM: case I2S_RATE: case I2S_FIFO_CONF:
        P.r[i] = v;
        i2sReEpoch(p, now);
        return;
    default:
        P.r[i] = v;
        return;
    }
}

static void installI2s(HMODULE q)
{
    typedef void* (*qdev_get_gpio_in_fn)(void* dev, int n);
    qdev_get_gpio_in_fn gpioIn = (qdev_get_gpio_in_fn)GetProcAddress(q, "qdev_get_gpio_in");
    if (!q_phys_rw) q_phys_rw = (void (*)(uint64_t, void*, uint64_t, bool))GetProcAddress(q, "cpu_physical_memory_rw");
    memset(&g_i2sOps, 0, sizeof(g_i2sOps));
    g_i2sOps.read = i2s_read;
    g_i2sOps.write = i2s_write;
    g_i2sOps.endianness = 2;
    g_i2sOps.valid_min_access_size = 1;
    g_i2sOps.valid_max_access_size = 4;
    g_i2sOps.impl_min_access_size = 4;
    g_i2sOps.impl_max_access_size = 4;
    for (int p = 0; p < 2; p++)
        if (gpioIn && g_soc) g_i2s[p].irq = gpioIn((uint8_t*)g_soc + OFS_INTMATRIX, I2S_SRC0 + p);
    i2sReset();
    void* sys = q_get_system_memory();
    const uint64_t base[4] = { 0x3FF4F000ull, 0x3FF6D000ull, 0x6000F000ull, 0x6002D000ull };
    for (int k = 0; k < 4; k++)
    {
        memset(&g_i2sMr[k], 0, sizeof(g_i2sMr[k]));
        q_mr_init_io(&g_i2sMr[k], nullptr, &g_i2sOps, (void*)(uintptr_t)(k & 1), k < 2 ? "esp32link.i2s" : "esp32link.i2s.alias", 0x100);
        q_mr_add_overlap(sys, base[k], &g_i2sMr[k], 10);
    }
    elog("I2S installed (2 ports, TX + RX master, interrupts %s, DMA %s)", g_i2s[0].irq && g_i2s[1].irq ? "on" : "MISSING",
         q_phys_rw ? "on" : "MISSING");
}

static void moveTmp105(HMODULE q)
{
    object_resolve_path_type_fn resolve = (object_resolve_path_type_fn)GetProcAddress(q, "object_resolve_path_type");
    if (!resolve) return;
    bool amb = false;
    uint8_t* obj = (uint8_t*)resolve("", "tmp105", &amb);
    if (!obj) return;
    if (obj[160] == 0x48)
    {
        obj[160] = 0x7F;
        elog("built-in TMP105 moved from I2C address 0x48 to 0x7F (0x48 is free for Proteus parts)");
    }
}

// QEMU bug in esp32_rmt.c (TX in wrap mode, which the IDF RMT driver uses):
//   item = data[(ch * 64 + (sent + i) % mem) % mem]     -> every channel reads channel 0's memory
// The second "% mem" is an idiv at RVA 0x1A6BA2 of this DLL build; replace it with
// "mov edx,eax" so the index is ch * 64 + (sent + i) % mem. Patched in memory only, after
// checking the bytes; another DLL build is left alone.
static void patchRmtWrap(HMODULE q)
{
    static const uint8_t want[12] = { 0x8B,0x45,0x18, 0xC1,0xE0,0x06, 0x01,0xD0, 0x99, 0xF7,0x7D,0xD4 };
    uint8_t* p = (uint8_t*)q + 0x1A6B9A;            // mov eax,[rbp+18h]; shl eax,6; add eax,edx; cdq; idiv [rbp-2Ch]
    if (memcmp(p, want, sizeof(want)) != 0) { elog("RMT wrap patch: code differs (other QEMU build) - not patched"); return; }
    DWORD old = 0;
    if (!VirtualProtect(p + 8, 4, PAGE_EXECUTE_READWRITE, &old)) { elog("RMT wrap patch: VirtualProtect failed"); return; }
    static const uint8_t fix[4] = { 0x89,0xC2, 0x90, 0x90 };   // mov edx,eax; nop; nop
    memcpy(p + 8, fix, sizeof(fix));
    VirtualProtect(p + 8, 4, old, &old);
    FlushInstructionCache(GetCurrentProcess(), p + 8, 4);
    elog("RMT wrap patch applied (QEMU read channel 0's memory for every channel)");

    // Second bug: before each chunk QEMU waits while the channel's threshold or end bit is set
    // in INT_RAW. The IDF driver disables the threshold interrupt after the last refill and
    // never clears that raw bit, so QEMU never reaches the end marker and rmtWrite() hangs.
    // Real hardware does not wait for the CPU: make the "je" at RVA 0x1A6B25 always jump.
    static const uint8_t want2[7] = { 0x44,0x21,0xC0, 0x85,0xC0, 0x74,0x10 };   // and eax,r8d; test eax,eax; je +10
    uint8_t* p2 = (uint8_t*)q + 0x1A6B20;
    if (memcmp(p2, want2, sizeof(want2)) != 0) { elog("RMT wait patch: code differs - not patched"); return; }
    if (!VirtualProtect(p2 + 5, 1, PAGE_EXECUTE_READWRITE, &old)) { elog("RMT wait patch: VirtualProtect failed"); return; }
    p2[5] = 0xEB;                                   // jmp +10
    VirtualProtect(p2 + 5, 1, old, &old);
    FlushInstructionCache(GetCurrentProcess(), p2 + 5, 1);
    elog("RMT wait patch applied (QEMU stalled on a pending threshold bit)");
}

// Wi-Fi soft-AP (WiFi.softAP): when the ESP32 sends beacons, QEMU's esp32_wifi_ap.c plays a station
// that joins it (probe request -> probe response -> authentication -> association -> DHCP).
// Its probe request carries a DS Parameter element with QEMU's "current channel" variable, which
// is not the AP channel (it follows PHY calibration writes: 0..24 at start-up; 3 when the first
// beacon came), so the ESP32 AP ignored it ("stations: 0"). The element is optional: leave it out
// (the call that adds it, RVA 0xC3307, becomes a 5-byte NOP).
static void patchSoftApProbe(HMODULE q)
{
    static const uint8_t want[13] = { 0xBA,0x03,0x00,0x00,0x00, 0x48,0x89,0xC1, 0xE8,0xAD,0xF5,0xFF,0xFF };   // mov edx,3; mov rcx,rax; call add_ie
    uint8_t* p = (uint8_t*)q + 0xC32FF;
    if (memcmp(p, want, sizeof(want)) != 0) { elog("soft-AP patch: code differs (other QEMU build) - not patched"); return; }
    DWORD old = 0;
    if (!VirtualProtect(p + 8, 5, PAGE_EXECUTE_READWRITE, &old)) { elog("soft-AP patch: VirtualProtect failed"); return; }
    static const uint8_t nop5[5] = { 0x0F,0x1F,0x44,0x00,0x00 };
    memcpy(p + 8, nop5, sizeof(nop5));
    VirtualProtect(p + 8, 5, old, &old);
    FlushInstructionCache(GetCurrentProcess(), p + 8, 5);
    elog("soft-AP patch applied (probe request without the wrong channel)");

    // The station's DHCP messages (builder at RVA 0xC2DE3) had two bugs, so the ESP32's DHCP
    // server ignored them and the station never got an IP:
    //  - BOOTP op = 0: QEMU writes htype = 1 at [rbp-43h] and hlen = 6 at [rbp-42h] but never op.
    //    One store now writes op 1, htype 1, hlen 6, hops 0 at [rbp-44h].
    //  - chaddr = the ESP32's own MAC (macaddr +0x34C4) instead of the station's (ap_macaddr +0x34CE).
    static const uint8_t want2[8] = { 0xC6,0x45,0xBD,0x01, 0xC6,0x45,0xBE,0x06 };   // mov byte [rbp-43h],1; mov byte [rbp-42h],6
    static const uint8_t want3[7] = { 0x48,0x8D,0x90, 0xC4,0x34,0x00,0x00 };        // lea rdx,[rax+34C4h]
    uint8_t* p2 = (uint8_t*)q + 0xC2EB2;
    uint8_t* p3 = (uint8_t*)q + 0xC2ED2;
    if (memcmp(p2, want2, sizeof(want2)) != 0 || memcmp(p3, want3, sizeof(want3)) != 0)
    {
        elog("soft-AP DHCP patch: code differs - not patched");
        return;
    }
    if (!VirtualProtect(p2, 0x30, PAGE_EXECUTE_READWRITE, &old)) { elog("soft-AP DHCP patch: VirtualProtect failed"); return; }
    static const uint8_t fix2[8] = { 0xC7,0x45,0xBC, 0x01,0x01,0x06,0x00, 0x90 };   // mov dword [rbp-44h],00060101h; nop
    memcpy(p2, fix2, sizeof(fix2));
    p3[3] = 0xCE;                                                                   // lea rdx,[rax+34CEh]
    VirtualProtect(p2, 0x30, old, &old);
    FlushInstructionCache(GetCurrentProcess(), p2, 0x30);
    elog("soft-AP DHCP patch applied (BOOTP op = request, chaddr = the station)");
}

// WIFIPORT forward: 127.0.0.1:N on the PC -> port 80 of the ESP32. In station mode the ESP32 is
// slirp's guest 192.168.4.15; as soft-AP it is 192.168.4.1 (QEMU's station got 192.168.4.2 by
// DHCP, which is also slirp's own address, so the answers come back). Every slice: follow QEMU's
// Wi-Fi mode (Esp32WifiState +0x34E0: 1 = joined to an ESP32 soft-AP) and move the forward.
// Slirp*: Esp32WifiState.nic (+0x34A0) -> NICState.ncs -> NetClientState.peer (+0x20) = SlirpState,
// slirp at +0x188. libslirp: slirp_add_hostfwd / slirp_remove_hostfwd (in_addr = network order).
enum { OFS_WIFI = 0x14d90, OFS_WIFI_MODE = 0x34E0, OFS_WIFI_NIC = 0x34A0 };
static int g_wifiPort = 0;
static int (*q_slirp_add_fwd)(void* slirp, int udp, uint32_t haddr, int hport, uint32_t gaddr, int gport) = nullptr;
static int (*q_slirp_remove_fwd)(void* slirp, int udp, uint32_t haddr, int hport) = nullptr;

static void updateWifiForward()
{
    static int fwdMode = 0;                               // 0: to 192.168.4.15 (as started), 1: to 192.168.4.1
    if (!g_wifiPort || !g_soc || !q_slirp_add_fwd || !q_slirp_remove_fwd) return;
    const uint8_t* w = g_soc + OFS_WIFI;
    int mode = w[OFS_WIFI_MODE] == 1 ? 1 : 0;
    if (mode == fwdMode) return;
    const uint8_t* nic = *(const uint8_t* const*)(w + OFS_WIFI_NIC);
    const uint8_t* nc = nic ? *(const uint8_t* const*)nic : nullptr;
    const uint8_t* peer = nc ? *(const uint8_t* const*)(nc + 0x20) : nullptr;
    void* slirp = peer ? *(void* const*)(peer + 0x188) : nullptr;
    if (!slirp) return;
    const uint32_t local = 0x0100007Fu, guest = mode ? 0x0104A8C0u : 0x0F04A8C0u;   // 127.0.0.1, 192.168.4.1 / .15
    q_slirp_remove_fwd(slirp, 0, local, g_wifiPort);
    int r = q_slirp_add_fwd(slirp, 0, local, g_wifiPort, guest, 80);
    fwdMode = mode;
    elog("WIFIPORT: http://127.0.0.1:%d -> ESP32 %s port 80%s", g_wifiPort, mode ? "soft-AP 192.168.4.1" : "station 192.168.4.15",
         r < 0 ? " (FAILED)" : "");
}

// QEMU's virtual access points are a table in esp32_wifi_ap.c (RVA 0x7775E0 of this DLL build):
// 3 records of 24 bytes { const char* ssid; int channel; int rssi; uint8_t mac[6]; }, then the count.
// --wifi-ssid gives the first one ("PICSimLabWifi") the user's name, so a sketch can use its real SSID.
static std::string g_wifiSsid;

static void renameAccessPoint(HMODULE q)
{
    if (g_wifiSsid.empty()) return;
    uint8_t* tab = (uint8_t*)q + 0x7775E0;
    const char* const* names = (const char* const*)tab;
    if (*(const int32_t*)(tab + 3 * 24) != 3 || !names[0] || strcmp(names[0], "PICSimLabWifi") != 0
        || strcmp(*(const char* const*)(tab + 24), "Espressif") != 0)
    {
        elog("WIFISSID: access point table not found (other QEMU build) - name not changed");
        return;
    }
    static char ssid[33];
    strncpy_s(ssid, g_wifiSsid.c_str(), 32);
    DWORD old = 0;
    if (!VirtualProtect(tab, sizeof(char*), PAGE_READWRITE, &old)) { elog("WIFISSID: VirtualProtect failed"); return; }
    *(const char**)tab = ssid;
    VirtualProtect(tab, sizeof(char*), old, &old);
    elog("Wi-Fi access point 1 renamed: \"PICSimLabWifi\" -> \"%s\"", ssid);
}

// WIFI=NO: no access points. QEMU uses an AP only when its channel equals the radio's channel
// (beacons, answers to the station), so channel -1 hides all three: a scan is empty and WiFi.begin
// never connects. (The count nb_aps must stay: the beacon timer does "% nb_aps".)
static bool g_hideAps = false;

static void hideAccessPoints(HMODULE q)
{
    if (!g_hideAps) return;
    uint8_t* tab = (uint8_t*)q + 0x7775E0;
    if (*(const int32_t*)(tab + 3 * 24) != 3 || strcmp(*(const char* const*)(tab + 24), "Espressif") != 0)
    {
        elog("WIFI=NO: access point table not found (other QEMU build) - access points stay visible");
        return;
    }
    DWORD old = 0;
    if (!VirtualProtect(tab, 3 * 24, PAGE_READWRITE, &old)) { elog("WIFI=NO: VirtualProtect failed"); return; }
    for (int i = 0; i < 3; i++) *(int32_t*)(tab + i * 24 + 8) = -1;
    VirtualProtect(tab, 3 * 24, old, &old);
    elog("Wi-Fi: no network (WIFI=NO), access points hidden");
}

// ---------------------------------------------------------------------------
//  lockstep timer (main loop thread, QEMU big lock held)
// ---------------------------------------------------------------------------
static void applyInputs()
{
    for (int i = 1; i < E32LINK_MAX_PINS; i++)                    // output levels -> the pins' inputs
        if (g_outEcho[i] >= 0) { q_set_pin(i, g_outEcho[i]); g_outEcho[i] = -1; }
    E32Event e;
    uint8_t rx[3][256];
    int rxn[3] = { 0, 0, 0 };
    while (e32_pop(&g_sh->toEngine, e))
    {
        switch (e.type)
        {
        case E32_EV_PIN_IN:
            if (e.a > 0 && e.a < E32LINK_MAX_PINS) { q_set_pin((int)e.a, e.b ? 1 : 0); g_inLevel[e.a] = e.b ? 1 : 0; }
            break;
        case E32_EV_UART_RX:
            if (e.a < 3 && !uartWakeDrop(e.a, (uint8_t)e.b))
            {
                static int logged[3];
                if (logged[e.a] < 20) { logged[e.a]++; elog("UART%u RX byte 0x%02X -> QEMU", e.a, e.b & 0xFF); }
                rx[e.a][rxn[e.a]++] = (uint8_t)e.b;
                if (rxn[e.a] == 256) { q_uart_receive(e.a, rx[e.a], 256); rxn[e.a] = 0; }
            }
            break;
        case E32_EV_ADC:
            q_set_apin((int)e.a, (int)e.b);
            break;
        case E32_EV_TOUCH_V:
            if (e.a > 0 && e.a < E32LINK_MAX_PINS)
                for (int k = 0; k < 10; k++)
                    if (TOUCH_GPIO[k] == g_pinmap[e.a]) g_touchRatio[k] = e.b > 1000 ? 1000 : (int)e.b;
            break;
        case E32_EV_EDGE:
            if (e.a > 0 && e.a < E32LINK_MAX_PINS) { rmtRxEdge((int)e.a, e.b ? 1 : 0, e.ns); pcntEdge((int)e.a, e.b ? 1 : 0, e.ns); mcpwmEdge((int)e.a, e.b ? 1 : 0, e.ns); }
            break;
        case E32_EV_CAN_RX:
            canReceive(e);
            break;
        case E32_EV_CAN_DONE:
            canDone(e);
            break;
        case E32_EV_I2S_RX:
            if (e.a < 2) i2sRxFrame((int)e.a, e.b, e.c);
            break;
        case E32_EV_I2CS:
            i2csEvent((int)e.a, (int)(e.b & 0xFF), (int)((e.b >> 8) & 0xFF));
            break;
        case E32_EV_SPIS:
            spisEvent((int)e.a, (int)(e.b & 0xFF), (int)((e.b >> 8) & 0xFF), e.c);
            break;
        case E32_EV_EN:
            // EN low: the chip is held in reset (reset again at every slice).
            // EN high: one last reset, then it boots.
            // Both are CHIP resets: without the mark the reset hook may take them for a one-CPU restart
            // (it depends on the order of QEMU's reset handlers) and never tell the model "running
            // again" - the model then stayed held and ignored the chip (EN button, 25 Oct 2026).
            if (!e.b && !g_enHeld) { g_enHeld = true; g_engineChipReset = true; q_reset_request(SHUTDOWN_CAUSE_GUEST_RESET_); elog("EN low: held in reset"); }
            else if (e.b && g_enHeld) { g_enHeld = false; g_engineChipReset = true; q_reset_request(SHUTDOWN_CAUSE_GUEST_RESET_); elog("EN high: booting"); }
            break;
        default:
            break;
        }
    }
    for (int u = 0; u < 3; u++)
        if (rxn[u]) q_uart_receive(u, rx[u], rxn[u]);
}

static void lockstep_cb(void*)
{
    // while every core is stalled (sleep) the slices run here in a loop: returning to QEMU
    // would need a wake-up of its idle CPU thread for every slice (slow on Windows)
    for (;;)
    {
        int64_t now = now_ns();

        // MCPWM outputs up to now go out BEFORE 'reached': the model takes the events
        // when it sees 'reached', later ones would come after Proteus passed their time
        mcRun(0, now);
        mcRun(1, now);
        i2sRun(0, now);              // ... and the I2S frames
        i2sRun(1, now);

        // tell the model how far we are
        e32_store64(&g_sh->reachedNs, now);
        SetEvent(g_reachedEvt);

        // wait until the model lets us go further
        HANDLE h[2] = { g_grantEvt, g_parent };
        int64_t grant;
        for (;;)
        {
            if (g_sh->quit) quitNow();
            grant = e32_load64(&g_sh->grantNs);
            if (grant > now) break;
            DWORD w = WaitForMultipleObjects(2, h, FALSE, 200);
            if (w == WAIT_OBJECT_0 + 1) quitNow();   // the simulator process is gone
        }

        applyInputs();
        updateSleep();               // deep / light sleep: reset + stall, wake-up sources
        if (g_enHeld) q_reset_request(SHUTDOWN_CAUSE_GUEST_RESET_);
        else if (g_deepSleep) {}     // stalled: nothing runs
        else { updatePulls(); updatePwm(); updateDac(); updateRouting(); updateRmtRx(); updatePcnt(); updateMcpwm(); updateI2s(); spisRoute(); updateCan(); updateWifiForward(); }
        if (g_btOn) btSlice(now_ns());   // Bluetooth link to the other parts (also while asleep: nothing is sent then)
        plantResetMarker();          // put the marker back if a chip reset wiped it
        setBootTime();               // PC clock: again if the boot code or a reset cleared it

        // next stop: the grant, but never more than one slice at a time, so input
        // from the simulator is seen at least once per slice
        int64_t next = now + g_sliceNs;
        if (next > grant) next = grant;
        if ((g_deepSleep || g_lightSleep) && allCoresStalled())
        {
            advanceVirtualClock(next - now_ns());   // nothing runs: move the clock on ourselves
            continue;                               // ... and do the next slice at once
        }
        q_timer_mod_ns(&g_timer, next);
        return;
    }
}

// ---------------------------------------------------------------------------
//  MAC address (--mac): QEMU reads it from the efuse file (Esp32EfuseRegs, 0x7C bytes;
//  without a file every ESP32 gets 10:01:00:C4:0A:24, so two parts would clash on ESP-NOW).
//  BLK0 word 1 = MAC bytes 5..2, word 2 bits 0-15 = MAC bytes 1..0, bits 16-23 = CRC-8/Maxim
//  (polynomial 0x8C reflected, init 0) over the 6 MAC bytes in normal order (checked by the IDF).
// ---------------------------------------------------------------------------
static uint8_t macCrc8(const uint8_t* mac)
{
    uint8_t c = 0;
    for (int i = 0; i < 6; i++)
    {
        c ^= mac[i];
        for (int b = 0; b < 8; b++) c = (c & 1) ? (uint8_t)((c >> 1) ^ 0x8C) : (uint8_t)(c >> 1);
    }
    return c;
}

static std::string prepareEfuse(const std::string& macText)
{
    unsigned m[6];
    if (sscanf_s(macText.c_str(), "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6)
        fail("bad MAC address '%s' (use 24:0A:C4:12:34:56)", macText.c_str());
    uint8_t mac[6];
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
    if (mac[0] & 1) fail("MAC address '%s' is a multicast address (first byte must be even)", macText.c_str());
    uint8_t efuse[0x7C] = {};
    uint32_t w1 = (uint32_t)mac[5] | ((uint32_t)mac[4] << 8) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[2] << 24);
    uint32_t w2 = (uint32_t)mac[1] | ((uint32_t)mac[0] << 8) | ((uint32_t)macCrc8(mac) << 16);
    memcpy(efuse + 4, &w1, 4);
    memcpy(efuse + 8, &w2, 4);
    char tmp[MAX_PATH];
    GetTempPathA(MAX_PATH, tmp);
    std::string path = std::string(tmp) + "etree-esp32_efuse_" + std::to_string(GetCurrentProcessId()) + ".bin";
    FILE* f = nullptr;
    if (fopen_s(&f, path.c_str(), "wb") != 0 || !f) fail("cannot create '%s'", path.c_str());
    fwrite(efuse, 1, sizeof(efuse), f);
    fclose(f);
    elog("MAC address %02X:%02X:%02X:%02X:%02X:%02X (efuse file %s)", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], path.c_str());
    return path;
}

// ---------------------------------------------------------------------------
//  flash image: copy to a temp file padded to 2/4/8/16 MB
// ---------------------------------------------------------------------------
static std::string prepareFlash(const std::string& src)
{
    FILE* f = nullptr;
    if (fopen_s(&f, src.c_str(), "rb") != 0 || !f) fail("cannot open firmware file '%s'", src.c_str());
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0 || len > 16L * 1024 * 1024) { fclose(f); fail("firmware '%s' has a bad size (%ld bytes)", src.c_str(), len); }

    size_t size = 2u * 1024 * 1024;
    while (size < (size_t)len) size *= 2;
    std::vector<uint8_t> img(size, 0xFF);
    size_t got = fread(img.data(), 1, (size_t)len, f);
    fclose(f);
    if (got != (size_t)len) fail("cannot read firmware '%s'", src.c_str());

    // A merged image has the 2nd-stage bootloader at 0x1000, starting with 0xE9.
    if (img[0x1000] != 0xE9)
        fail("'%s' is not a merged image (no bootloader at 0x1000). In Arduino IDE use "
             "Sketch -> Export Compiled Binary and pick the *.merged.bin file", src.c_str());
    // Byte 2 of the image header is the SPI mode: 0 QIO, 1 QOUT, 2 DIO, 3 DOUT.
    if (img[0x1002] != 2)
        elog("WARNING: flash mode byte is %u, not DIO (2). If the firmware does not boot, "
             "set Tools -> Flash Mode -> DIO and export again", img[0x1002]);
    g_btOn = btPatchFirmware(img, src, elog);      // Bluetooth sketch: our controller instead of Espressif's

    char tmp[MAX_PATH], name[MAX_PATH];
    GetTempPathA(MAX_PATH, tmp);
    sprintf_s(name, "%setree-esp32_%lu.bin", tmp, GetCurrentProcessId());
    if (fopen_s(&f, name, "wb") != 0 || !f) fail("cannot write temp flash file '%s'", name);
    fwrite(img.data(), 1, img.size(), f);
    fclose(f);
    elog("flash: %s (%ld bytes) -> %s (%u MB)", src.c_str(), len, name, (unsigned)(size >> 20));
    return name;
}

// ---------------------------------------------------------------------------
//  main
// ---------------------------------------------------------------------------
template <class T> static void need(HMODULE m, T& fn, const char* name)
{
    fn = (T)GetProcAddress(m, name);
    if (!fn) fail("libqemu-xtensa.dll has no export '%s' (wrong DLL version?)", name);
}

int main(int argc, char** argv)
{
    std::string tag, flash;
    DWORD parentPid = 0;
    int shift = 2;
    bool wifi = false;
    int wifiPort = 0;
    std::string sdcard;                          // SD card image for the SD/MMC host (SD_MMC library)
    int espnowPort = 0;                          // ESP-NOW: Wi-Fi frames between all parts on this multicast port
    std::string macText;
    std::string btMacText;                       // Bluetooth base MAC (the model gives every part its own)
    int btPort = 1235;                           // Bluetooth link between the engines of one design
    bool blePc = false;                          // BLEPC: Bluetooth also through the PC's adapter
    for (int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        auto next = [&](void) -> std::string { return (i + 1 < argc) ? std::string(argv[++i]) : std::string(); };
        if (a == "--link") tag = next();
        else if (a == "--parent") parentPid = (DWORD)strtoul(next().c_str(), nullptr, 10);
        else if (a == "--flash") flash = next();
        else if (a == "--slice-us") g_sliceNs = (int64_t)(atof(next().c_str()) * 1000.0);
        else if (a == "--shift") shift = atoi(next().c_str());
        else if (a == "--wifi") wifi = true;
        else if (a == "--wifi-port") wifiPort = g_wifiPort = atoi(next().c_str());
        else if (a == "--wifi-ssid")
        {
            g_wifiSsid = next();                     // no spaces at the ends (typing slips in Proteus)
            while (!g_wifiSsid.empty() && g_wifiSsid.back() == ' ') g_wifiSsid.pop_back();
            while (!g_wifiSsid.empty() && g_wifiSsid.front() == ' ') g_wifiSsid.erase(0, 1);
        }
        else if (a == "--sdcard") sdcard = next();
        else if (a == "--espnow") espnowPort = atoi(next().c_str());
        else if (a == "--mac") macText = next();
        else if (a == "--bt-mac") btMacText = next();
        else if (a == "--bt-port") btPort = atoi(next().c_str());
        else if (a == "--ble-pc") blePc = true;
        else if (a == "--clock") { std::string m = next(); g_clockMode = m == "utc" ? 2 : m == "local" ? 1 : 0; }
    }
    if (tag.empty() || flash.empty() || !parentPid)
    {
        fprintf(stderr, "etree-esp32: started by the ESP32 simulation model, not by hand.\n"
                        "usage: etree-esp32 --link <tag> --parent <pid> --flash <merged.bin> "
                        "[--slice-us 20] [--shift 2] [--wifi]\n");
        return 1;
    }
    if (g_sliceNs < 1000) g_sliceNs = 1000;
    if (shift < 0 || shift > 10) shift = 2;
    InitializeCriticalSection(&g_pushLock);
    memset((void*)g_outEcho, -1, sizeof(g_outEcho));

    // ---- link ----
    char mapName[128], grantName[128], reachedName[128];
    e32_names(tag.c_str(), mapName, grantName, reachedName, sizeof(mapName));
    HANDLE hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, mapName);
    if (!hMap) { fprintf(stderr, "etree-esp32: link '%s' not found\n", mapName); return 1; }
    g_sh = (E32Shared*)MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(E32Shared));
    if (!g_sh) { fprintf(stderr, "etree-esp32: cannot map link\n"); return 1; }
    if (g_sh->magic != E32LINK_MAGIC || g_sh->version != E32LINK_VERSION || g_sh->structSize != sizeof(E32Shared))
    {
        fprintf(stderr, "etree-esp32: link version mismatch (model and engine are from different releases)\n");
        return 1;
    }
    g_grantEvt = OpenEventA(SYNCHRONIZE | EVENT_MODIFY_STATE, FALSE, grantName);
    g_reachedEvt = OpenEventA(SYNCHRONIZE | EVENT_MODIFY_STATE, FALSE, reachedName);
    g_parent = OpenProcess(SYNCHRONIZE, FALSE, parentPid);
    if (!g_grantEvt || !g_reachedEvt || !g_parent) fail("cannot open link events or parent process");
    g_sh->sliceNs = (uint32_t)g_sliceNs;
    memcpy(g_pinmap, (const void*)g_sh->pinmap, sizeof(g_pinmap));
    if (g_pinmap[0] <= 0 || g_pinmap[0] >= E32LINK_MAX_PINS) fail("bad pinmap from the model");

    // ---- load QEMU ----
    std::string dir = exeDir();
    SetDllDirectoryA(dir.c_str());   // glib, gcrypt, ... live next to the exe
    std::string dllPath = dir + "\\qemu\\libqemu-xtensa.dll";
    HMODULE q = LoadLibraryExA(dllPath.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
    if (!q) fail("cannot load %s (Windows error %lu). Are the 8 support DLLs next to etree-esp32.exe?",
                 dllPath.c_str(), GetLastError());
    need(q, q_init, "qemu_init");
    need(q, q_main_loop, "qemu_main_loop");
    need(q, q_cleanup, "qemu_cleanup");
    need(q, q_register_callbacks, "qemu_picsimlab_register_callbacks");
    need(q, q_set_pin, "qemu_picsimlab_set_pin");
    need(q, q_set_apin, "qemu_picsimlab_set_apin");
    need(q, q_uart_receive, "qemu_picsimlab_uart_receive");
    need(q, q_get_internals, "qemu_picsimlab_get_internals");
    need(q, q_timer_init_full, "timer_init_full");
    need(q, q_timer_mod_ns, "timer_mod_ns");
    need(q, q_clock_get_ns, "qemu_clock_get_ns");
    need(q, q_vm_start, "vm_start");
    need(q, q_get_cpu, "qemu_get_cpu");
    need(q, q_runstall, "xtensa_runstall");
    need(q, q_cpu_reset_interrupt, "cpu_reset_interrupt");
    need(q, q_reset_request, "qemu_system_reset_request");
    need(q, q_get_system_memory, "get_system_memory");
    need(q, q_mr_init_io, "memory_region_init_io");
    need(q, q_mr_add_overlap, "memory_region_add_subregion_overlap");
    need(q, q_register_reset, "qemu_register_reset");
    need(q, q_set_irq, "qemu_set_irq");
    g_timersState = (uint8_t*)GetProcAddress(q, "timers_state");
    if (!g_timersState) elog("WARNING: timers_state not exported - millis() stops during I2C / SPI steps");
    patchRmtWrap(q);
    patchSoftApProbe(q);
    q_slirp_add_fwd = (int (*)(void*, int, uint32_t, int, uint32_t, int))GetProcAddress(q, "slirp_add_hostfwd");
    q_slirp_remove_fwd = (int (*)(void*, int, uint32_t, int))GetProcAddress(q, "slirp_remove_hostfwd");
    renameAccessPoint(q);

    // ---- firmware ----
    g_flashCopy = prepareFlash(flash);
    if (!macText.empty()) g_efuseCopy = prepareEfuse(macText);

    // ---- callbacks (before qemu_init: the machine reads the pinmap while it is built) ----
    static QemuPicsimlabCallbacks cbs;
    cbs.write_pin = cb_write_pin;
    cbs.dir_pin = cb_dir_pin;
    cbs.i2c_event = cb_i2c_event;
    cbs.spi_event = cb_spi_event;
    cbs.uart_tx_event = cb_uart_tx;
    cbs.pinmap = g_pinmap;
    cbs.rmt_event = cb_rmt_event;
    q_register_callbacks(&cbs);

    // ---- QEMU command line ----
    std::string fwDir = dir + "\\qemu\\fw";
    std::string drive = "file=" + g_flashCopy + ",if=mtd,format=raw";
    std::string icount = "shift=" + std::to_string(shift) + ",align=off,sleep=off";
    std::vector<std::string> args = {
        dllPath,
        "-M", "esp32-picsimlab",
        "-L", fwDir,
        "-drive", drive,
        "-icount", icount,
        "-S",                                    // start paused; we start it after the timer exists
        "-display", "none",
        "-monitor", "none",
        "-serial", "null",                       // UART0 bytes come to us through uart_tx_event
        "-global", "driver=timer.esp32.timg,property=wdt_disable,value=true",
        "-object", "can-bus,id=canbus0",            // TWAI (CAN): controller + engine on this bus
    };
    if (espnowPort > 0 && espnowPort < 65536)
    {
        // ESP-NOW: QEMU sends every Wi-Fi frame of this ESP32 as a UDP multicast packet and gets
        // the ones of the other ESP32 parts (same port): no access point, no internet.
        if (wifi) elog("Wi-Fi: ESP-NOW mode, the internet (WIFI=YES) is off");
        args.push_back("-nic");
        // localaddr=127.0.0.1: the multicast stays on this PC (not on the local network)
        args.push_back("socket,model=esp32_wifi,mcast=230.0.0.1:" + std::to_string(espnowPort) + ",localaddr=127.0.0.1");
    }
    else if (wifi)
    {
        // slirp: DHCP + DNS + NAT to the PC's network. --wifi-port N: 127.0.0.1:N on the PC -> ESP32 port 80
        std::string nic = "user,model=esp32_wifi,net=192.168.4.0/24";
        if (wifiPort > 0 && wifiPort < 65536) nic += ",hostfwd=tcp:127.0.0.1:" + std::to_string(wifiPort) + "-:80";
        args.push_back("-nic");
        args.push_back(nic);
    }
    else
    {
        // No network, but the Wi-Fi device must exist: without it a sketch that starts Wi-Fi crashes
        // in the PHY start-up (LoadStorePIFAddrError at 0x60033c00). A hub port with nothing else on
        // the hub drops every frame; hideAccessPoints() empties the AP list (scan finds nothing).
        args.push_back("-nic");
        args.push_back("hubport,hubid=0,model=esp32_wifi");
        g_hideAps = true;
    }
    if (!sdcard.empty())
    {
        args.push_back("-drive");
        args.push_back("file=" + sdcard + ",if=sd,format=raw");
    }
    if (!g_efuseCopy.empty())
    {
        args.push_back("-drive");
        args.push_back("file=" + g_efuseCopy + ",if=none,format=raw,id=efuse");
        args.push_back("-global");
        args.push_back("driver=nvram.esp32.efuse,property=drive,value=efuse");
    }
    std::vector<char*> av;
    std::string cmdline;
    for (auto& s : args) { av.push_back(&s[0]); cmdline += s + " "; }
    av.push_back(nullptr);
    elog("qemu args: %s", cmdline.c_str());
    hideAccessPoints(q);                       // needs the Wi-Fi choice made above

    q_init((int)args.size(), av.data());       // calls exit() itself on a bad command line
    if (g_timersState && *(const int16_t*)(g_timersState + 0x1a) != shift)       // TimersState.icount_time_shift
    {
        elog("WARNING: timers_state layout differs (other QEMU build) - millis() stops during I2C / SPI steps");
        g_timersState = nullptr;
    }

    for (int u = 0; u < 3; u++) g_uartBaud[u] = q_get_internals(QI_UART0_BAUD + u);
    initClockProbe();
    g_iomux = q_get_internals(QI_IOMUX);
    g_iomuxW = (uint32_t*)g_iomux;
    g_outSel = q_get_internals(QI_GPIO_OUT_SEL);
    g_inSel = q_get_internals(QI_GPIO_IN_SEL);
    for (int g = 0; g < 40; g++) g_gpioToBoard[g] = 0;
    for (int i = 1; i <= g_pinmap[0] && i < E32LINK_MAX_PINS; i++)
        if (g_pinmap[i] >= 0 && g_pinmap[i] < 40) g_gpioToBoard[g_pinmap[i]] = i;
    moveTmp105(q);
    g_ledcConf0 = q_get_internals(QI_LEDC_CHANNEL_CONF0);
    g_ledcFreq = q_get_internals(QI_LEDC_TIMER_FREQ);
    g_ledcDuty = (const float*)q_get_internals(QI_LEDC_DUTY);
    plantResetMarker();
    installRtcio();
    installGpio1();
    installSens();
    installRmtRx();
    installCan(q);
    installPcnt(q);
    installMcpwm(q);
    installI2cSlave();
    installSpiSlave(q);
    installI2s(q);
    installSleep();
    if (g_btOn)
    {
        uint8_t mac[6] = { 0x10, 0x01, 0x00, 0xC4, 0x0A, 0x24 };   // QEMU's MAC without an efuse file
        unsigned m[6];
        const std::string& mt = !btMacText.empty() ? btMacText : macText;
        if (!mt.empty() && sscanf_s(mt.c_str(), "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6)
            for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
        BtEnv env = { elog, physRead, q_get_system_memory, q_mr_init_io, q_mr_add_overlap };
        btInstall(env, mac, btPort, blePc);
    }
    if (g_clockMode)
    {
        FILETIME ft;
        GetSystemTimeAsFileTime(&ft);
        if (g_clockMode == 1) { FILETIME lt; FileTimeToLocalFileTime(&ft, &lt); ft = lt; }
        ULARGE_INTEGER u;
        u.LowPart = ft.dwLowDateTime;
        u.HighPart = ft.dwHighDateTime;
        g_clockStartUs = (int64_t)(u.QuadPart / 10) - 11644473600000000LL;   // 1601 -> 1970
        setBootTime();
        elog("ESP32 clock starts at the PC %s time", g_clockMode == 1 ? "local" : "UTC");
    }
    q_register_reset(onSystemReset, nullptr);

    // ---- lockstep timer, then run ----
    memset(&g_timer, 0, sizeof(g_timer));
    q_timer_init_full(&g_timer, nullptr, QEMU_CLOCK_VIRTUAL_, QEMU_SCALE_NS, 0, lockstep_cb, nullptr);
    int64_t base = now_ns();
    e32_store64(&g_sh->baseNs, base);
    e32_store64(&g_sh->reachedNs, base);
    q_timer_mod_ns(&g_timer, base);            // first stop right away: wait for the first grant
    InterlockedExchange(&g_sh->engineState, E32_STATE_RUNNING);
    SetEvent(g_reachedEvt);
    elog("running, base %lld ns, slice %lld ns, icount shift %d", (long long)base, (long long)g_sliceNs, shift);

    q_vm_start();
    int status = q_main_loop();
    q_cleanup(status);

    InterlockedExchange(&g_sh->engineState, E32_STATE_EXITED);
    SetEvent(g_reachedEvt);
    DeleteFileA(g_flashCopy.c_str());
    if (!g_efuseCopy.empty()) DeleteFileA(g_efuseCopy.c_str());
    return status;
}
