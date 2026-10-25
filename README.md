# etree-esp32

[![Download](https://img.shields.io/badge/Download-etree--esp32%20Latest-brightgreen?logo=github&logoColor=brightgreen)](https://github.com/electronicstree/etree-esp32/releases/latest)
[![Proteus Model](https://img.shields.io/badge/Proteus%20Model-ESP32%20DevKitC-orange?logo=espressif&logoColor=orange)](https://www.electronicstree.com)
[![License](https://img.shields.io/badge/License-GPL--2.0--or--later-blue?logo=gnu&logoColor=blue)](LICENSE)
[![QEMU](https://img.shields.io/badge/QEMU-9.2.2-red?logo=qemu&logoColor=red)](https://github.com/electronicstree/qemu/tree/75bd5828e58ad4925451bf1fd28b0ff79269b927)
[![Platform](https://img.shields.io/badge/Platform-Windows%2064--bit-lightgrey)](#requirements)

**etree-esp32** runs real ESP32 firmware (for example an Arduino sketch exported as a `.merged.bin`
file) in the QEMU ESP32 emulator. It keeps the emulated chip in exact time step (lockstep) with a
circuit simulator.

The engine is the open-source half of the ESP32 model for **Proteus 8** by
[electronicstree.com](https://www.electronicstree.com). The other half, the Proteus model `ESP32.DLL`,
is free to use but closed source. The two halves talk only through a small MIT-licensed shared-memory
link (`esp32link.h`).

> You do **not** need to build this to use the ESP32 Proteus model: a ready-to-run engine is in the
> release zip (see Download at the end). This repository is for people who want to read, change or
> rebuild the engine.

---

## What it does

```
        ┌───────────────────────────┐            ┌───────────────────────────┐
        │  ESP32.DLL (32-bit)       │            │  etree-esp32.exe (64-bit) │
        │  model inside Proteus     │            │  QEMU + your firmware     │
        └─────────────┬─────────────┘            └─────────────┬─────────────┘
                      │                                        │
        ┌─────────────┴────────────────────────────────────────┴─────────────┐
        │  Shared memory link (esp32link.h)                                  │
        │  • 2 event rings: pin changes, UART bytes, bus steps ...           │
        │  • event "grant"   model -> engine: run the firmware until time T  │
        │  • event "reached" engine -> model: time T reached, events ready   │
        └────────────────────────────────────────────────────────────────────┘
```

- **Exact time.** QEMU runs with `-icount` (a fixed number of nanoseconds per instruction), so the
  firmware's clock depends only on the code, not on the PC's speed. A QEMU timer stops the guest at the
  end of every time slice (default 20 µs); the simulator then grants the next slice. Every event the
  engine reports carries its exact virtual time. `millis()` grows exactly 1000 per simulated second.
- **Pins.** Pin levels, directions, pulls, PWM, and the bus signals are sent to the simulator as events.
  Inputs from the circuit (pin levels, UART bytes, ADC voltages) reach the firmware at most 2 slices late.
- **Buses are clocked by the circuit.** For I2C and SPI the engine stops the firmware, the simulator
  clocks the real bits on its wires, and the answer (ACK, data) comes back. The bus time is added to the
  firmware's clock.
- **Safety.** If the simulator closes or crashes, the engine notices and exits.

### Peripherals

| Group | What works | Not simulated |
|---|---|---|
| Basic | GPIO, interrupts, pulls (IO MUX + RTC IO), ADC, DAC, touch, EN reset | |
| Timing / PWM | LEDC PWM, RMT TX + RX (WS2812), PCNT, MCPWM (timers, compare, generator actions, dead time, capture) | MCPWM carrier, fault and timer sync; RMT RX filter |
| Buses | UART 0-2 (real baud), I2C master + slave, SPI master + slave (HSPI / VSPI, DMA), CAN (TWAI), I2S master send + receive | I2S slave, PDM, built-in DAC / ADC mode, MCLK; Ethernet |
| Storage | SD card (SD_MMC host, raw image file) | |
| Power | deep sleep + light sleep with timer, ext0, ext1, touch, GPIO and UART wake-up | ULP co-processor |
| Wireless | Wi-Fi station + soft-AP (QEMU's slirp: DHCP, DNS, internet, port forward), ESP-NOW between engines, Bluetooth (BLE + Classic SPP, see below) | Wi-Fi with a password (WPA2) |
| Clock | optional start time from the PC clock | |

### Bluetooth

QEMU has no Bluetooth, and the ESP32 controller is a closed binary blob. The engine replaces the
**controller**: before the firmware boots, the controller API functions in the app image (found with the
`.elf` file next to the `.merged.bin`) are overwritten with small stubs (`bt_stubs.S`) that pass every HCI
packet to a virtual controller in the engine (`bt.cpp`). The firmware's Bluedroid host runs unchanged.
Engines of one design find each other over UDP multicast on 127.0.0.1 (`btlink.h`). With `--ble-pc` the
PC's real Bluetooth adapter is used through Windows WinRT (`bt_pc.cpp`).

### Things QEMU does not have, built by the engine

The PICSimLab QEMU build lacks several ESP32 blocks. The engine adds its own memory-mapped register blocks
(QEMU memory regions with a higher priority) for: RTC IO pulls, DAC, touch, GPIO32/33 output registers,
sleep (RTC_CNTL), PCNT, MCPWM, I2S, I2C / SPI slave mode, RMT RX, and it creates the TWAI (CAN) device. It
also fixes a few bugs in this exact QEMU build with small in-memory byte patches (RMT wrap mode, soft-AP
probe / DHCP). Every patch checks the original bytes first and is skipped if they do not match.

---

## Requirements

- Windows 10 / 11, **64-bit**.
- The run-time files (QEMU DLL, ROM files, 8 DLLs): take them from the release zip (see Download). If you
  build your own exe, put it into that folder in place of the one from the zip. The files are the same as
  in **PICSimLab** 0.9.3 (`picsimlab_win64`, by Luis Claudio Gamboa Lopes,
  [github.com/lcgamboa/picsimlab](https://github.com/lcgamboa/picsimlab)), folder `lib\qemu` and the
  program folder.

The engine reads struct offsets and patches bytes that belong to **this exact build** of
`libqemu-xtensa.dll` (PICSimLab 0.9.3, 260920, win64). Another build may not work.

## Build

Visual Studio 2022 or newer with the "Desktop development with C++" workload (CMake included).

The engine source is in the `etree-esp32` folder of this repository (with its `CMakeLists.txt`).

1. **File -> Open -> Folder...** and pick the `etree-esp32` folder (VS reads `CMakeLists.txt`).
2. Choose the **x64-Release** configuration.
3. **Build -> Build All**. The result is `etree-esp32\out\build\x64-Release\etree-esp32.exe`.

Or from an "x64 Native Tools" command prompt, in the repository folder:

```
cmake -S etree-esp32 -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

It must be **64-bit** (`libqemu-xtensa.dll` is 64-bit). The C runtime is linked statically, so users do
not need the VC++ redistributable. C++20 is needed for C++/WinRT (`bt_pc.cpp`).

## Command line

The simulator model starts the engine; you normally never start it by hand.

```
etree-esp32 --link <tag> --parent <pid> --flash <file.merged.bin> [options]
```

| Option | Meaning |
|---|---|
| `--link <tag>` | Name of the shared-memory link (see below). Needed. |
| `--parent <pid>` | Process id of the simulator; the engine exits when it is gone. |
| `--flash <file>` | Firmware image (`.merged.bin`, 2 / 4 / 8 / 16 MB, Flash Mode DIO). The engine works on a copy in `%TEMP%`; your file is never changed. For Bluetooth, the `.elf` of the same build must be next to it. |
| `--slice-us <n>` | Lockstep slice in µs (default 20). |
| `--shift <n>` | 2^n ns per instruction (default 2 = 4 ns, about 250 MHz). |
| `--wifi` | Wi-Fi with QEMU's open access points and NAT to the PC's network. |
| `--wifi-ssid <name>` | Name of the first access point. |
| `--wifi-port <n>` | Forward `127.0.0.1:<n>` on the PC to port 80 of the ESP32. |
| `--espnow <port>` | ESP-NOW between engines over UDP multicast on 127.0.0.1 (no access points). |
| `--mac <mac>` | Wi-Fi base MAC address (written to a temporary efuse file). |
| `--bt-mac <mac>` | Bluetooth base MAC address. |
| `--bt-port <port>` | Bluetooth link port; engines with the same port see each other. |
| `--ble-pc` | Also use the PC's Bluetooth adapter. |
| `--sdcard <image>` | Raw SD card image for the SD_MMC host (size a power of 2). |
| `--clock local\|utc` | `time()` starts at the PC's local / UTC time. |

The engine writes its log to standard output / error. The Proteus model sends it to
`%TEMP%\etree-esp32_<part id>.log`.

## Licence

- **etree-esp32** (all files except the two headers below): **GPL-2.0-or-later**, see `LICENSE`.
- `esp32link.h` and `btlink.h`: **MIT** (see the SPDX line at the top of each file).
- Not in this repository, but needed at run time:
  - `libqemu-xtensa.dll` and the ROM files: QEMU, GPL-2.0, by the QEMU project, Espressif and
    Luis Claudio Gamboa Lopes. Source: [github.com/lcgamboa/qemu](https://github.com/lcgamboa/qemu),
    branch `picsimlab-esp32`.
  - The 8 runtime DLLs: GLib, libgcrypt, libgpg-error, libintl, libiconv, PCRE2 (LGPL / BSD) and
    winpthreads, zlib (MIT / zlib licence). Their licence texts are shipped in `licenses\` of the release.

ESP32, ESP-IDF and the Arduino ESP32 core are by Espressif Systems. Proteus is a product of Labcenter
Electronics. This project is not connected with either company.

## Download

Ready-to-run engine (no build, no other downloads needed):
**[Latest release](https://github.com/electronicstree/etree-esp32/releases/latest)** -> `etree-esp32-<version>-win64.zip`

The zip holds one folder, `etree-esp32`, with everything the engine needs to run:

```
etree-esp32\
  etree-esp32.exe
  libglib-2.0-0.dll   libgcrypt-20.dll   libgpg-error-0.dll   libintl-8.dll
  libiconv-2.dll      libpcre2-8-0.dll   libwinpthread-1.dll  zlib1.dll
  qemu\libqemu-xtensa.dll
  qemu\fw\esp32-v3-rom.bin
  qemu\fw\esp32-v3-rom-app.bin
  licenses\           (licence texts)
  SOURCE.txt          (where the source code of every file is)
  README.txt
```

Proteus users: copy the whole `etree-esp32` folder into `...\Proteus 8 Professional\DATA\MODELS\`.
The ESP32 Proteus model itself (`ESP32.DLL`, the part library and the user guide) is available at
[electronicstree.com](https://www.electronicstree.com).

## Credits

- Luis Claudio Gamboa Lopes for PICSimLab and its QEMU ESP32 build.
- Espressif Systems for the QEMU ESP32 work and ESP-IDF.
- The QEMU project.
