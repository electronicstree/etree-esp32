#pragma once
// bt_internal.h - between the virtual controller (bt.cpp) and the PC Bluetooth bridge (bt_pc.cpp)
// Every bridge connection has a link id (like the connections between engines).
#include <stdint.h>
#include <vector>

// ---- bridge -> controller (any thread; queued, done in btSlice under the QEMU lock) ----
enum
{
    PC_CONNECT = 1,     // phone mirror: connect to the ESP32 when it advertises connectable (ESP32 = slave);
                        //   data = PC adapter address (6 bytes, HCI order)
    PC_L2CAP,           // data = one L2CAP frame (length, CID, payload) for the ESP32's host
    PC_DISCONNECT,      // the bridge ends the connection; data[0] = reason
    PC_ADV,             // a real device's advertising: addr type, addr (6, HCI order), event type, rssi, len, data
    PC_REAL_UP,         // the real device of a pending LE Create Connection is connected (ESP32 = master)
    PC_REAL_FAIL,       // ... could not be connected (the controller tries again at its next advertising)
};
void btPcPost(int type, uint32_t link, std::vector<uint8_t> data);

// ---- controller -> bridge (called with the QEMU lock held; the bridge only queues) ----
void pcOnConnected(uint32_t link);                     // PC_CONNECT done
void pcOnNotReady();                                   // PC_CONNECT: the ESP32 does not advertise connectable
void pcOnAcl(uint32_t link, uint8_t pb, const uint8_t* data, size_t n);   // ACL fragment from the ESP32's host
void pcOnDisconnected(uint32_t link, uint8_t reason);  // a bridge connection is gone (ESP32 host / reset)
void pcConnectReal(uint32_t link, uint8_t addrType, const uint8_t addr[6]);   // ESP32 connects to a real device

// start the bridge (BLEPC = YES); log = the engine's log
void btPcStart(void (*log)(const char*, ...));
