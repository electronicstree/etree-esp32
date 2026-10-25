#pragma once
// ============================================================================
//  qemu_esp32_api.h - the libqemu-xtensa.dll entry points used by etree-esp32.exe
//
//  SPDX-License-Identifier: GPL-2.0-or-later
//
//  Read from the DWARF debug info inside the PICSimLab 0.9.3 (260920, win64)
//  build of libqemu-xtensa.dll: QEMU 9.2.2, machine "esp32-picsimlab".
//  The DLL is 64-bit (MinGW). Load it with LoadLibrary + GetProcAddress.
//
//  All pin indexes are BOARD PIN numbers (1..N) from the pinmap you pass,
//  NOT GPIO numbers. pinmap[0] = N, pinmap[i] = GPIO of board pin i,
//  or a negative value for a pin that is not a GPIO (VCC, GND, EN ...).
//  The DLL has room for 99 board pins (pin_irq[100]).
//
//  Facts checked in the machine code:
//   - write_pin(i, level) fires only for GPIO0..31 whose output enable is set,
//     when the level or the enable changes. GPIO32..39 never call write_pin.
//   - dir_pin(i, 1) = output enabled, dir_pin(i, 0) = input.
//     dir_pin(-1, x) is a "sync" pulse from the GPIO / IO MUX / LEDC blocks.
//   - uart_tx_event(uart, byte) fires for every byte as soon as the firmware
//     writes the TX FIFO - there is NO baud-rate timing. Pace it yourself.
//   - set_pin(i, level) drives the GPIO input of board pin i (all 40 GPIOs).
// ============================================================================
#include <stdint.h>

extern "C" {

// ---- callbacks that the DLL calls (vCPU thread or main-loop thread) ----
typedef void    (*qemu_write_pin_fn)(int boardPin, int value);
typedef void    (*qemu_dir_pin_fn)(int boardPin, int isOutput);
typedef int     (*qemu_i2c_event_fn)(uint8_t id, uint8_t addr, uint16_t event);
typedef uint8_t (*qemu_spi_event_fn)(uint8_t id, uint16_t event);
typedef void    (*qemu_uart_tx_fn)(uint8_t uart, uint8_t byte);
typedef void    (*qemu_rmt_event_fn)(uint8_t channel, uint32_t config0, uint32_t value);

// Layout checked against qemu_picsimlab_register_callbacks (offsets 0x00..0x30).
// The DLL copies the pointers. Keep pinmap alive while QEMU runs.
struct QemuPicsimlabCallbacks
{
    qemu_write_pin_fn   write_pin;      // 0x00
    qemu_dir_pin_fn     dir_pin;        // 0x08
    qemu_i2c_event_fn   i2c_event;      // 0x10
    qemu_spi_event_fn   spi_event;      // 0x18
    qemu_uart_tx_fn     uart_tx_event;  // 0x20
    const short*        pinmap;         // 0x28
    qemu_rmt_event_fn   rmt_event;      // 0x30
};

// ---- picsimlab functions ----
typedef void      (*qemu_init_fn)(int argc, char** argv);
typedef int       (*qemu_main_loop_fn)(void);                 // same thread as qemu_init
typedef void      (*qemu_cleanup_fn)(int status);
typedef void      (*qemu_register_callbacks_fn)(void* callbacks);   // BEFORE qemu_init
typedef void      (*qemu_set_pin_fn)(int boardPin, int value);
typedef void      (*qemu_set_apin_fn)(int adcChannel, int raw);     // sens.ADC_values[adcChannel]
typedef void      (*qemu_uart_receive_fn)(int uart, const uint8_t* data, int len);
typedef uint32_t* (*qemu_get_internals_fn)(int which);              // see QemuInternals
typedef int       (*qemu_flash_dump_fn)(int64_t offset, void* buf, int len);

// qemu_picsimlab_get_internals(which) returns a pointer INTO the live SoC state.
enum QemuInternals
{
    QI_GPIO_STRAP_MODE     = 0,   // uint32_t; in this build ptr[-1] = gpio_out1 (GPIO32..39), ptr[9] = gpio_enable
    QI_GPIO_IN_SEL         = 1,   // uint32_t[256] GPIO matrix input select
    QI_GPIO_OUT_SEL        = 2,   // uint32_t[40]  GPIO matrix output select
    QI_IOMUX               = 3,   // uint32_t[40]  IO MUX function per pad
    QI_LEDC_CHANNEL_CONF0  = 4,   // uint32_t[16]
    QI_LEDC_TIMER_FREQ     = 5,   // uint32_t[8]
    QI_LEDC_DUTY           = 6,   // float[16], percent
    QI_UART0_BAUD          = 7,   // uint32_t
    QI_UART1_BAUD          = 8,
    QI_UART2_BAUD          = 9
};

// ---- plain QEMU functions (also exported by this build) ----
enum QemuClockType { QEMU_CLOCK_REALTIME_ = 0, QEMU_CLOCK_VIRTUAL_ = 1, QEMU_CLOCK_HOST_ = 2, QEMU_CLOCK_VIRTUAL_RT_ = 3 };
enum { QEMU_SCALE_NS = 1 };
enum { SHUTDOWN_CAUSE_GUEST_RESET_ = 7, SHUTDOWN_CAUSE_HOST_UI_ = 5 };

// struct QEMUTimer, 48 bytes on x64. timer_new_full is NOT exported,
// so the engine owns the storage and calls timer_init_full.
struct QemuTimerStorage
{
    int64_t expire_time;
    void*   timer_list;
    void*   cb;
    void*   opaque;
    void*   next;
    int     attributes;
    int     scale;
};

typedef void    (*qemu_timer_cb)(void* opaque);
typedef void    (*timer_init_full_fn)(QemuTimerStorage* t, void* tlg /*NULL = main loop*/,
                                      int clockType, int scale, int attributes,
                                      qemu_timer_cb cb, void* opaque);
typedef void    (*timer_mod_ns_fn)(QemuTimerStorage* t, int64_t expire);
typedef int64_t (*qemu_clock_get_ns_fn)(int clockType);
typedef void    (*vm_start_fn)(void);
typedef void    (*qemu_system_reset_request_fn)(int cause);
typedef void    (*qemu_system_shutdown_request_fn)(int cause);

// ---- memory regions (used for the RTC IO register block, see main.cpp) ----
// struct MemoryRegionOps, 80 bytes on x64 (layout from the DWARF info)
struct QemuMemoryRegionOps
{
    uint64_t (*read)(void* opaque, uint64_t addr, unsigned size);                 // 0
    void     (*write)(void* opaque, uint64_t addr, uint64_t data, unsigned size); // 8
    void*    read_with_attrs;                                                     // 16
    void*    write_with_attrs;                                                    // 24
    int      endianness;            // 2 = DEVICE_LITTLE_ENDIAN                    // 32
    int      pad0;
    unsigned valid_min_access_size;                                               // 40
    unsigned valid_max_access_size;                                               // 44
    uint8_t  valid_unaligned;                                                     // 48
    uint8_t  pad1[7];
    void*    valid_accepts;                                                       // 56
    unsigned impl_min_access_size;                                                // 64
    unsigned impl_max_access_size;                                                // 68
    uint8_t  impl_unaligned;                                                      // 72
    uint8_t  pad2[7];
};
static_assert(sizeof(QemuMemoryRegionOps) == 80, "MemoryRegionOps layout");

// struct MemoryRegion is 272 bytes and holds an Int128: 16-byte alignment.
struct alignas(16) QemuMemoryRegionStorage { uint8_t bytes[272]; };

typedef void* (*get_system_memory_fn)(void);
typedef void  (*memory_region_init_io_fn)(QemuMemoryRegionStorage* mr, void* owner,
                                          const QemuMemoryRegionOps* ops, void* opaque,
                                          const char* name, uint64_t size);
typedef void  (*memory_region_add_subregion_overlap_fn)(void* parent, uint64_t offset,
                                                        QemuMemoryRegionStorage* sub, int priority);
typedef void  (*qemu_reset_handler)(void* opaque);
typedef void  (*qemu_register_reset_fn)(qemu_reset_handler fn, void* opaque);

} // extern "C"
