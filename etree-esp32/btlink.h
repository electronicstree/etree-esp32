#pragma once
// ============================================================================
//  btlink.h - the Bluetooth link between the parts of one Proteus design
//  (ESP32 engines, the virtual phone model)
//
//  SPDX-License-Identifier: MIT
//  This one header is MIT licensed (not GPL) on purpose: it is only a data format, and both
//  the open-source engine and private models (the virtual phone) include it.
//
//  UDP multicast BTLINK_GROUP:<BTPORT> (default 1235), sent and received only on 127.0.0.1
//  (sockets bound to 127.0.0.1: no firewall prompt). Every message:
//    "E32B" | version (1) | type (1) | sender nonce (4, random per sender; own messages come
//    back through the loopback and are skipped) | body
//  Addresses are 6 bytes in HCI order (least significant byte first); numbers little-endian.
//  A connection is known on both sides by a random 32-bit link id chosen by the initiator.
// ============================================================================
#include <stdint.h>

#define BTLINK_MAGIC    "E32B"
#define BTLINK_VERSION  1
#define BTLINK_GROUP    "230.0.0.2"
#define BTLINK_PORT     1235
#define BTLINK_HEADER   10                // magic 4 + version 1 + type 1 + nonce 4

enum BtLinkMsg
{
    BTL_ADV = 1,        // advertiser: addr type, addr, adv type (0 ADV_IND, 1 DIRECT, 2 SCAN_IND, 3 NONCONN,
                        //   4 DIRECT low duty), len, data, rsp len, rsp data (sent about every 100 ms)
    BTL_CONNECT,        // initiator: target type, target addr, initiator type, initiator addr, link (4),
                        //   interval (2), latency (2), supervision timeout (2)
    BTL_CONN_ACK,       // target (it was advertising connectable at that address): link, target type, target addr
    BTL_ACL,            // link, packet boundary flag (0 / 2 first fragment, 1 continuing), L2CAP data
    BTL_DISC,           // link, reason (HCI error code, e.g. 0x13 remote user terminated, 0x08 timeout)
    BTL_CONN_UPD,       // link, interval, latency, timeout
    BTL_DLE,            // link, tx octets (2), tx time (2)
    BTL_ENC_REQ,        // link, rand (8), ediv (2)            master's LE Start Encryption
    BTL_ENC_RSP,        // link, 1 + LTK (16) / 0              slave host's LTK reply / negative reply
    BTL_ENC_DONE,       // link                                keys matched: the slave reports Encryption Change
    // Bluetooth Classic (BR/EDR): after the connection the ACL, disconnect and encryption messages above apply
    BTL_BR_INQ,         // discoverable device (inquiry scan on), about every 300 ms: addr, class of device (3),
                        //   name len, name, EIR len, EIR data
    BTL_BR_CONNECT,     // pager (initiator): target addr, own addr, link (4), class of device (3), name len, name;
                        //   the target's host accepts -> BTL_CONN_ACK (link, 0, target addr), rejects -> BTL_DISC
};
