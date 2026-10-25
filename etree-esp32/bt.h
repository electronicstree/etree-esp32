#pragma once
// bt.h - Bluetooth for the ESP32 engine: a virtual controller behind the IDF controller API (see bt.cpp)
#include <stdint.h>
#include <string>
#include <vector>
#include "qemu_esp32_api.h"

// engine services bt.cpp uses
struct BtEnv
{
    void  (*log)(const char* fmt, ...);
    void  (*physRead)(uint32_t addr, void* buf, uint32_t len);
    void* (*getSystemMemory)();
    memory_region_init_io_fn              mrInitIo;
    memory_region_add_subregion_overlap_fn mrAddOverlap;
};

// Before qemu_init: replace the controller API in the app image of the merged flash image
// (needs <firmware>.elf next to <firmware>.merged.bin). True = patched, Bluetooth runs.
bool btPatchFirmware(std::vector<uint8_t>& img, const std::string& firmwarePath, void (*log)(const char*, ...));

// After qemu_init (only when patched): register block + virtual controller. mac = the chip's base MAC,
// port = UDP port of the link between the ESP32 parts of one design (multicast on 127.0.0.1),
// pc = BLEPC: also the PC's real Bluetooth adapter (bt_pc.cpp).
void btInstall(const BtEnv& env, const uint8_t mac[6], int port, bool pc);

// Every slice (main loop, QEMU lock held): the link (send advertising, receive the other parts).
void btSlice(int64_t nowNs);

// Chip reset: controller off, nothing queued.
void btReset();
