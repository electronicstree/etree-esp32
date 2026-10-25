// bt_stubs.S - guest (Xtensa) code that replaces the ESP32 Bluetooth controller API in the firmware.
// Assembled once by hand; the bytes are in bt.cpp (BT_STUB / BT_BLOB). Rebuild after a change:
//   xtensa-esp32-elf-as bt_stubs.S -o bt_stubs.o && xtensa-esp32-elf-objdump -d bt_stubs.o
// (toolchain: %LOCALAPPDATA%\Arduino15\packages\esp32\tools\esp-x32\<ver>\bin)
//
// The engine owns a register block (BT_MBOX): writing a call id to +0 makes the engine run that call
// at once (args at +4 / +8, result at +12). Reading +16 hands over the next HCI packet from the
// virtual controller (copied to +0x100, length returned, 0 = none); +20 = the host's notify_host_recv.
        .text

// One per replaced API function, written over its first bytes. "call8 helper" is encoded by the
// engine (PC-relative). 16 bytes.
        .align 4
stub:
        entry   a1, 32
        movi.n  a10, 5                  // call id (patched)
        mov.n   a11, a2                 // arg 0
        mov.n   a12, a3                 // arg 1
        call8   stub                    // -> helper (patched)
        mov.n   a2, a10
        retw.n

// The blob: written over the body of esp_bt_controller_init (after that function's own stub).
// Literals first (l32r loads only from lower addresses); the engine fills them.
        .align 4
blob:
lit_mbox:   .word 0                     // BT_MBOX
lit_rxbuf:  .word 0                     // BT_MBOX + 0x100
lit_name:   .word 0                     // BT_MBOX + 0x40 ("vhci")
lit_task:   .word 0                     // address of rx_task
lit_create: .word 0                     // xTaskCreatePinnedToCore
lit_delay:  .word 0                     // vTaskDelay
lit_stack:  .word 4096

// helper(id, a0, a1): one engine call
        .align 4
helper:
        entry   a1, 32
        l32r    a8, lit_mbox
        s32i    a3, a8, 4
        s32i    a4, a8, 8
        memw
        s32i    a2, a8, 0
        memw
        l32i    a2, a8, 12
        retw.n

// esp_vhci_host_register_callback(cb): the engine gets cb and cb->notify_host_recv (read here: the
// struct is const, in flash (DROM), which the engine cannot read), then the RX task is created once
// (id 9 answers 1 the first time).
        .align 4
regcb:
        entry   a1, 48
        mov.n   a11, a2
        l32i.n  a12, a2, 4              // cb->notify_host_recv
        movi.n  a10, 8                  // id 8: register_callback
        call8   helper
        movi.n  a10, 9                  // id 9: is the RX task needed?
        call8   helper
        beqz.n  a10, 1f
        l32r    a10, lit_task           // xTaskCreatePinnedToCore(rx_task, "vhci", 4096, NULL, 22, NULL, 0)
        l32r    a11, lit_name
        l32r    a12, lit_stack
        movi.n  a13, 0
        movi.n  a14, 22
        movi.n  a15, 0
        movi.n  a9, 0
        s32i    a9, a1, 0               // 7th argument (core) on the stack
        l32r    a8, lit_create
        callx8  a8
1:      movi.n  a2, 0
        retw.n

// RX task: hand every HCI packet from the engine to the host; idle: wait 1 tick
        .align 4
rx_task:
        entry   a1, 32
2:      l32r    a8, lit_mbox
        memw
        l32i    a11, a8, 16             // length of the next packet (engine copies it to +0x100)
        beqz.n  a11, 3f
        l32i    a9, a8, 20              // notify_host_recv
        l32r    a10, lit_rxbuf
        callx8  a9
        j       2b
3:      movi.n  a10, 1
        l32r    a9, lit_delay
        callx8  a9
        j       2b
