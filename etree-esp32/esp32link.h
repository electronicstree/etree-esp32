#pragma once
// ============================================================================
//  esp32link.h - message link between the ESP32 Proteus model (32-bit DLL)
//  and etree-esp32.exe (64-bit, runs QEMU).
//
//  SPDX-License-Identifier: MIT
//  This one header is MIT licensed (not GPL) on purpose: it is only a data
//  format, and both the private model and the open-source engine include it.
//
//  One named shared-memory block holds:
//    - a header (state, time grant, pinmap, error text)
//    - two one-way rings of fixed-size events:
//        toModel  : engine -> model  (pin output, pin direction, UART TX byte)
//        toEngine : model  -> engine (pin input, UART RX byte, ADC, reset)
//    - two auto-reset events: "grant" (model -> engine), "reached" (engine -> model)
//
//  Lockstep: all times are QEMU virtual-clock nanoseconds. The model writes
//  grantNs = "you may run until here" and sets the grant event. The engine
//  runs the firmware up to grantNs, writes reachedNs and sets the reached event.
//
//  Layout is identical in 32-bit and 64-bit builds: fixed-size types only,
//  no pointers, 8-byte packing. 64-bit values are read/written with
//  Interlocked* so a 32-bit reader never sees half a value.
// ============================================================================
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <stdint.h>

#define E32LINK_MAGIC      0x4C323345u   // "E32L"
#define E32LINK_VERSION    14u
#define E32LINK_RING_SIZE  16384u        // events per ring, power of two
#define E32LINK_MAX_PINS   100           // pinmap[0] = count, pinmap[1..99]

// Engine states
enum
{
    E32_STATE_STARTING = 0,
    E32_STATE_RUNNING  = 1,
    E32_STATE_FAILED   = 2,   // errorText says why; engine exits
    E32_STATE_EXITED   = 3
};

// Event types
enum
{
    // engine -> model
    E32_EV_PIN_OUT  = 1,      // a = board pin, b = level 0/1
    E32_EV_PIN_DIR  = 2,      // a = board pin, b = 1 output / 0 input
    E32_EV_UART_TX  = 3,      // a = uart 0..2, b = byte, c = baud rate
    E32_EV_RESET    = 4,      // the chip was reset (EN, watchdog, esp_restart):
                              //   b = 1 it runs again, b = 0 it is held in reset (EN low)
    E32_EV_PULL     = 5,      // a = board pin, b = 0 none / 1 pull-up / 2 pull-down
    E32_EV_PWM      = 6,      // a = board pin, b = duty in 1/1000 % (0..100000),
                              //   c = frequency in Hz; b = 0xFFFFFFFF: PWM off (plain GPIO again)
    E32_EV_I2C      = 7,      // a = bus 0/1, b = op | data << 8, c = SCL Hz, reserved = request seq
                              //   op: 0 START+read (data = 7-bit addr), 1 START+write, 3 STOP,
                              //   4 NACK, 5 write byte, 6 read byte. The engine WAITS for the answer
                              //   (busResp): ACK 0 / NACK 1, or the byte read.
    E32_EV_SPI      = 8,      // a = bus 0 HSPI / 1 VSPI, b = op | data << 8, c = mode
                              //   (bit0 CPOL, bit1 CPHA, bit2 LSB first), reserved = request seq.
                              //   op 0: transfer byte (engine waits, busResp = MISO byte)
                              //   op 1: CS change, data = cs | level << 2 (no answer)
    E32_EV_BUSPINS  = 9,      // a = bus (0 I2C0, 1 I2C1, 2 HSPI, 3 VSPI, 4 UART1, 5 UART2)
                              //   b = 4 board pins (byte 0..3): I2C scl,sda / SPI clk,mosi,miso / UART tx,rx
                              //   c = SPI: cs0,cs1,cs2 board pins (bytes 0..2); UART: baud rate
                              //   (0 = not connected). Sent whenever the routing changes.
    E32_EV_DAC      = 10,     // a = board pin (IO25 DAC1 / IO26 DAC2), b = 8-bit value 0..255
                              //   (output = 3V3 * b / 255); b = 0xFFFFFFFF: DAC off (plain GPIO again)
    E32_EV_TOUCH    = 11,     // a = board pin (touch pad T0..T9), b = 1 touch on / 0 off (plain GPIO again)
    E32_EV_RMT      = 12,     // a = board pin, b = RMT item (bits 0-14 duration0, 15 level0,
                              //   16-30 duration1, 31 level1; a duration 0 ends the frame),
                              //   c = tick length in ps (0: RMT released the pin, plain GPIO again),
                              //   reserved = idle level after the frame
    E32_EV_RMT_RXPIN = 13,    // a = board pin, b = 1: send E32_EV_EDGE for every level change of this
                              //   pin (RMT receive / PCNT input) / 0: stop
    E32_EV_CAN_CONF = 14,     // TWAI (CAN) setup: a = TX board pin, b = RX board pin (bits 0-7) |
                              //   controller mode << 8 (bit 0 reset, bit 1 listen only, bit 2 self test) |
                              //   error state << 16 (0 error active, 1 error passive, 2 bus off),
                              //   c = bit time in ps (0: TWAI off, pins are plain GPIOs again),
                              //   reserved = sample point in 1/1000 of a bit
    E32_EV_CAN_TX   = 15,     // CAN frame to send on the TX pin: a = DLC 0..8 | 0x10 single shot |
                              //   sequence number << 8, b = CAN id | 0x80000000 extended (29-bit) |
                              //   0x40000000 remote frame, c = data bytes 0..3 (byte 0 in bits 0-7),
                              //   reserved = data bytes 4..7. The model answers with E32_EV_CAN_DONE.
    E32_EV_CAN_ABORT = 16,    // drop the frames that have not started on the bus yet
    E32_EV_I2CS_CONF = 17,    // I2C slave: a = bus 0/1, b = 0x80000000 | 7-bit address (0 = slave off).
                              //   The bus pins come with E32_EV_BUSPINS (SCL, SDA)
    E32_EV_I2CS_TX  = 18,     // I2C slave TX FIFO: a = bus, b = byte appended, or 0x100 = FIFO emptied
    E32_EV_SPIS_CONF = 19,    // SPI slave: a = host 0 HSPI / 1 VSPI, b = 1 on / 0 off,
                              //   c = board pins: CLK, MOSI, MISO, CS (bytes 0..3)
    E32_EV_SPIS_TX  = 20,     // SPI slave transaction: a = host, b = 0x100 | SPI mode << 9 | TX LSB first
                              //   << 11 | RX LSB first << 12: a new transaction (MISO bytes follow), or
                              //   b = one MISO byte
    E32_EV_I2S_CONF = 21,     // I2S master clock: a = port 0/1, b = board pins BCK, WS, DOUT, DIN (bytes 0..3,
                              //   0 = not routed / not used), c = BCK period in ps (0: I2S off, pins are plain
                              //   GPIOs again), reserved = bits per channel (bits 0-5) | 0x100 Philips (WS
                              //   changes one BCK before the MSB) | 0x200 short frame sync (PCM) | 0x400 RX on
                              //   (the model reads DIN and answers every frame with E32_EV_I2S_RX)
    E32_EV_I2S_DATA = 22,     // one stereo frame: a = port, b = left, c = right (left-aligned to 32 bits: the
                              //   pins send the top "bits" bits, MSB first); ns = EXACT start of the frame
                              //   (frames follow each other without a gap while the clock runs)

    // model -> engine (ns is ignored: applied at the next slice boundary)
    E32_EV_PIN_IN   = 64,     // a = board pin, b = level 0/1
    E32_EV_UART_RX  = 65,     // a = uart 0..2, b = byte
    E32_EV_ADC      = 66,     // a = ADC channel, b = raw value
    E32_EV_EN       = 67,     // b = level of the EN pin (0 = hold in reset)
    E32_EV_TOUCH_V  = 68,     // a = board pin, b = touch pin voltage / (3V3 / 2) in 1/1000 (0..1000;
                              //   1000 = not touched, 0 = pin at GND)
    E32_EV_EDGE     = 69,     // a = board pin, b = new level; ns = EXACT engine time of the edge
                              //   (only for pins asked by E32_EV_RMT_RXPIN)
    E32_EV_CAN_RX   = 70,     // CAN frame read from the RX pin (sent by another node), same fields
                              //   as E32_EV_CAN_TX (a = DLC only)
    E32_EV_CAN_DONE = 71,     // CAN bus result: a = 0 frame sent (ACK seen), 1 arbitration lost,
                              //   2 error while sending, 3 error while receiving, 4 frame given up
                              //   (single shot); b = sequence number of the frame (bits 0-7) |
                              //   arbitration lost bit (a = 1) or SJA1000 error code ECC (a = 2, 3) << 8
    E32_EV_I2CS     = 72,     // I2C slave bus activity: a = bus, b = op | data << 8; op 0 START or
                              //   repeated START for our address (data = 1 master reads), 1 byte
                              //   received, 2 byte sent (taken from the TX FIFO), 3 STOP
    E32_EV_SPIS     = 73,     // SPI slave: a = host, b = op | data << 8; op 1 byte received (MOSI),
                              //   op 3 CS went high: transaction end, c = bits clocked
    E32_EV_I2S_RX   = 74      // I2S frame read from DIN: a = port, b = left, c = right (left-aligned to 32 bits),
                              //   ns = start of the frame
};

#pragma pack(push, 8)

struct E32Event
{
    int64_t  ns;              // QEMU virtual time of the event
    uint16_t type;
    uint16_t a;
    uint32_t b;
    uint32_t c;
    uint32_t reserved;        // UART_TX: TX board pin (0 = default); I2C/SPI: request seq
};                            // 24 bytes

struct E32Ring
{
    volatile LONG head;       // written by the producer only
    uint8_t       pad1[60];
    volatile LONG tail;       // written by the consumer only
    uint8_t       pad2[60];
    E32Event      ev[E32LINK_RING_SIZE];
};

struct E32Shared
{
    uint32_t magic;
    uint32_t version;
    uint32_t structSize;
    volatile LONG engineState;     // E32_STATE_*
    volatile LONG quit;            // model sets 1: engine must exit now
    uint32_t sliceNs;              // informational
    volatile LONGLONG baseNs;      // virtual time when the firmware starts (set before RUNNING)
    volatile LONGLONG grantNs;     // model -> engine
    volatile LONGLONG reachedNs;   // engine -> model
    int16_t  pinmap[E32LINK_MAX_PINS];
    char     errorText[512];
    volatile LONG busReqSeq;       // engine: last bus request (I2C / SPI) it waits for
    volatile LONG busRespSeq;      // model: set to busReqSeq when busResp is ready
    volatile LONG busResp;         // model: ACK/NACK or byte
    volatile LONG busElapsedNs;    // model: Proteus time the bus step took (set before busRespSeq);
                                   //   the engine moves the firmware's clock forward by it
    E32Ring  toModel;
    E32Ring  toEngine;
};

#pragma pack(pop)

// ---------------------------------------------------------------------------
//  small helpers (header-only)
// ---------------------------------------------------------------------------
inline LONGLONG e32_load64(volatile LONGLONG* p)
{
    return InterlockedCompareExchange64(p, 0, 0);
}

inline void e32_store64(volatile LONGLONG* p, LONGLONG v)
{
    LONGLONG old = *p;
    for (;;)
    {
        LONGLONG seen = InterlockedCompareExchange64(p, v, old);
        if (seen == old) break;
        old = seen;
    }
}

// Producer side. Returns false when the ring is full.
inline bool e32_push(E32Ring* r, const E32Event& e)
{
    LONG head = r->head;
    LONG tail = InterlockedCompareExchange(&r->tail, 0, 0);
    if ((uint32_t)(head - tail) >= E32LINK_RING_SIZE) return false;
    r->ev[(uint32_t)head & (E32LINK_RING_SIZE - 1)] = e;
    MemoryBarrier();
    InterlockedExchange(&r->head, head + 1);
    return true;
}

// Consumer side. Returns false when the ring is empty.
inline bool e32_pop(E32Ring* r, E32Event& out)
{
    LONG tail = r->tail;
    LONG head = InterlockedCompareExchange(&r->head, 0, 0);
    if (head == tail) return false;
    MemoryBarrier();
    out = r->ev[(uint32_t)tail & (E32LINK_RING_SIZE - 1)];
    MemoryBarrier();
    InterlockedExchange(&r->tail, tail + 1);
    return true;
}

// Object names. "tag" is unique per model instance, e.g. "1234_5" (pid_counter).
// Local\ names stay inside the user's session; both processes run there.
inline void e32_names(const char* tag, char* map, char* grant, char* reached, size_t n)
{
    wsprintfA(map, "Local\\esp32link_%s_map", tag);
    wsprintfA(grant, "Local\\esp32link_%s_grant", tag);
    wsprintfA(reached, "Local\\esp32link_%s_reached", tag);
    (void)n;
}
