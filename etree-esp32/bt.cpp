// ============================================================================
//  bt.cpp - Bluetooth for the ESP32 engine (GPL, part of etree-esp32)
//
//  QEMU has no Bluetooth hardware, and the ESP32 controller is a closed blob that drives
//  undocumented radio registers (it asserted in btdm_low_power_mode_init). So the controller
//  is replaced: before the firmware boots, the controller API functions in the app image are
//  overwritten with small stubs (code: bt_stubs.S) that pass every call to the engine through
//  a register block (BT_MBOX). Bluedroid stays unchanged and talks HCI to our virtual controller.
//
//  Needs <sketch>.ino.elf next to <sketch>.ino.merged.bin (Arduino puts both in the build folder)
//  to find the functions. The image checksum and the appended SHA-256 are computed again, so the
//  bootloader accepts the patched image. Only the engine's temp copy of the flash is changed.
//
//  Register block BT_MBOX (engine's own, priority 10):
//    +0  write: call id (runs the call at once)   +4 / +8 arguments   +12 result
//    +16 read: length of the next HCI packet to the host (copied to +0x100), 0 = none
//    +20 read: the host's notify_host_recv        +0x40 "vhci" (task name)
//  Call ids: 1 controller_init, 2 deinit, 3 enable, 4 disable, 5 get_status,
//            6 vhci check_send_available, 7 vhci send_packet, 8 vhci register_callback,
//            9 "RX task needed?" (1 the first time after a reset).
// ============================================================================
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <bcrypt.h>
#include <stdio.h>
#include <string.h>
#include <deque>
#include <map>
#include "bt.h"
#include "bt_internal.h"
#include "btlink.h"
#include <mutex>

#pragma comment(lib, "bcrypt.lib")
#pragma comment(lib, "ws2_32.lib")

static const uint32_t BT_MBOX = 0x3FF71000;      // BT register space (unused: the real controller is gone)
static const uint32_t BT_MBOX_SIZE = 0x1000;
static const uint32_t BT_RXBUF = 0x100;          // offset of the packet buffer in BT_MBOX

// ---------------------------------------------------------------------------
//  guest code (bt_stubs.S, assembled with xtensa-esp32-elf-as)
// ---------------------------------------------------------------------------
// stub: entry a1,32; movi.n a10,ID; mov.n a11,a2; mov.n a12,a3; call8 helper; mov.n a2,a10; retw.n (16 bytes).
// Functions without arguments get it without the two mov.n (12 bytes: esp_bt_controller_get_status has 12).
static std::vector<uint8_t> makeStub(int id, bool args)
{
    std::vector<uint8_t> s = { 0x36, 0x41, 0x00, 0x0C, (uint8_t)(0x0A | (id << 4)) };   // entry; movi.n a10, id
    if (args) s.insert(s.end(), { 0xBD, 0x02, 0xCD, 0x03 });                           // mov.n a11, a2; mov.n a12, a3
    s.insert(s.end(), { 0x65, 0xFF, 0xFF, 0x2D, 0x0A, 0x1D, 0xF0 });                     // call8 (patched); mov.n a2, a10; retw.n
    return s;
}
static size_t stubCallPos(bool args) { return args ? 9 : 5; }

// blob: literals, helper, regcb, rx_task
static const uint8_t BT_BLOB[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x36, 0x41, 0x00, 0x81,
    0xF8, 0xFF, 0x39, 0x18, 0x49, 0x28, 0xC0, 0x20, 0x00, 0x29, 0x08, 0xC0, 0x20, 0x00, 0x28, 0x38,
    0x1D, 0xF0, 0x00, 0x00, 0x36, 0x61, 0x00, 0xBD, 0x02, 0xC8, 0x12, 0x0C, 0x8A, 0xE5, 0xFD, 0xFF,
    0x0C, 0x9A, 0xA5, 0xFD, 0xFF, 0x9C, 0x7A, 0xA1, 0xF1, 0xFF, 0xB1, 0xEF, 0xFF, 0xC1, 0xF2, 0xFF,
    0x0C, 0x0D, 0x1C, 0x6E, 0x0C, 0x0F, 0x0C, 0x09, 0x99, 0x01, 0x81, 0xED, 0xFF, 0xE0, 0x08, 0x00,
    0x0C, 0x02, 0x1D, 0xF0, 0x36, 0x41, 0x00, 0x81, 0xE6, 0xFF, 0xC0, 0x20, 0x00, 0xB8, 0x48, 0x8C,
    0x9B, 0x98, 0x58, 0xA1, 0xE4, 0xFF, 0xE0, 0x09, 0x00, 0x86, 0xFA, 0xFF, 0x0C, 0x1A, 0x91, 0xE5,
    0xFF, 0xE0, 0x09, 0x00, 0xC6, 0xF7, 0xFF,
};
enum {
    LIT_MBOX = 0x00, LIT_RXBUF = 0x04, LIT_NAME = 0x08, LIT_TASK = 0x0C, LIT_CREATE = 0x10, LIT_DELAY = 0x14,
    BLOB_HELPER = 0x1C, BLOB_REGCB = 0x34, BLOB_RXTASK = 0x64,
};

// call8 at 'pc' to 'target' (both guest addresses): op0 5, n 2, 18-bit word offset
static bool encodeCall8(uint8_t* p, uint32_t pc, uint32_t target)
{
    int64_t off = ((int64_t)target - (int64_t)((pc & ~3u) + 4)) / 4;
    if (target & 3 || off < -(1 << 17) || off >= (1 << 17)) return false;
    uint32_t w = 0x25u | (((uint32_t)off & 0x3FFFF) << 6);
    p[0] = (uint8_t)w; p[1] = (uint8_t)(w >> 8); p[2] = (uint8_t)(w >> 16);
    return true;
}

// j at 'pc' to 'target': op0 6, n 0, 18-bit byte offset from pc + 4
static bool encodeJ(uint8_t* p, uint32_t pc, uint32_t target)
{
    int64_t off = (int64_t)target - (int64_t)(pc + 4);
    if (off < -(1 << 17) || off >= (1 << 17)) return false;
    uint32_t w = 0x06u | (((uint32_t)off & 0x3FFFF) << 6);
    p[0] = (uint8_t)w; p[1] = (uint8_t)(w >> 8); p[2] = (uint8_t)(w >> 16);
    return true;
}

static void put32(uint8_t* p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// ---------------------------------------------------------------------------
//  ELF symbols (32-bit little-endian)
// ---------------------------------------------------------------------------
struct ElfSym { uint32_t addr, size; };
struct ElfFile
{
    std::vector<uint8_t> data;
    std::map<std::string, ElfSym> syms;
    struct Sec { uint32_t addr, size, off, type; };
    std::vector<Sec> secs;

    bool load(const std::string& path)
    {
        FILE* f = nullptr;
        if (fopen_s(&f, path.c_str(), "rb") != 0 || !f) return false;
        fseek(f, 0, SEEK_END);
        long n = ftell(f);
        fseek(f, 0, SEEK_SET);
        data.resize(n > 0 ? (size_t)n : 0);
        size_t got = fread(data.data(), 1, data.size(), f);
        fclose(f);
        if (got != data.size() || data.size() < 52 || memcmp(data.data(), "\x7F" "ELF\x01\x01", 6) != 0) return false;
        uint32_t shoff = get32(&data[0x20]);
        uint16_t shentsize = get16(&data[0x2E]), shnum = get16(&data[0x30]);
        if (shentsize < 40 || (uint64_t)shoff + (uint64_t)shnum * shentsize > data.size()) return false;
        for (int i = 0; i < shnum; i++)
        {
            const uint8_t* s = &data[shoff + i * shentsize];
            secs.push_back({ get32(s + 12), get32(s + 20), get32(s + 16), get32(s + 4) });
        }
        for (int i = 0; i < shnum; i++)
        {
            const uint8_t* s = &data[shoff + i * shentsize];
            if (get32(s + 4) != 2) continue;                       // SHT_SYMTAB
            uint32_t off = get32(s + 16), size = get32(s + 20), link = get32(s + 24), ent = get32(s + 36);
            if (link >= secs.size() || ent < 16 || (uint64_t)off + size > data.size()) continue;
            const Sec& str = secs[link];
            for (uint32_t k = 0; k + ent <= size; k += ent)
            {
                const uint8_t* y = &data[off + k];
                uint32_t name = get32(y);
                if (name >= str.size || (y[12] & 0xF) != 2) continue; // STT_FUNC only
                const char* nm = (const char*)&data[str.off + name];
                syms[nm] = { get32(y + 4), get32(y + 8) };
            }
        }
        return !syms.empty();
    }

    const ElfSym* find(const char* name) const
    {
        auto it = syms.find(name);
        return it == syms.end() ? nullptr : &it->second;
    }

    // bytes of the loaded section that holds 'addr' (to check the ELF belongs to the image)
    const uint8_t* bytesAt(uint32_t addr, uint32_t len) const
    {
        for (const Sec& s : secs)
            if (s.type == 1 && addr >= s.addr && addr + len <= s.addr + s.size && (uint64_t)s.off + s.size <= data.size())
                return &data[s.off + (addr - s.addr)];
        return nullptr;
    }
};

// ---------------------------------------------------------------------------
//  ESP app image: segments, checksum, SHA-256
// ---------------------------------------------------------------------------
struct AppImage
{
    std::vector<uint8_t>* img = nullptr;
    size_t base = 0;                                  // image start in the flash image
    struct Seg { uint32_t load, len; size_t data; };  // data = offset in the flash image
    std::vector<Seg> segs;
    size_t csumPos = 0;                               // checksum byte
    bool hashed = false;

    bool parse(std::vector<uint8_t>& flash, size_t at)
    {
        img = &flash;
        base = at;
        if (at + 24 > flash.size() || flash[at] != 0xE9) return false;
        int n = flash[at + 1];
        hashed = flash[at + 23] == 1;
        size_t p = at + 24;
        for (int i = 0; i < n; i++)
        {
            if (p + 8 > flash.size()) return false;
            Seg s = { get32(&flash[p]), get32(&flash[p + 4]), p + 8 };
            if (s.data + s.len > flash.size()) return false;
            segs.push_back(s);
            p = s.data + s.len;
        }
        while (((p - at) % 16) != 15) p++;
        csumPos = p;
        return p + 1 + (hashed ? 32 : 0) <= flash.size();
    }

    uint8_t checksum() const
    {
        uint8_t c = 0xEF;
        for (const Seg& s : segs)
            for (uint32_t i = 0; i < s.len; i++) c ^= (*img)[s.data + i];
        return c;
    }

    void sha256(uint8_t out[32]) const
    {
        BCryptHash(BCRYPT_SHA256_ALG_HANDLE, nullptr, 0, img->data() + base, (ULONG)(csumPos + 1 - base), out, 32);
    }

    uint8_t* at(uint32_t addr, uint32_t len)
    {
        for (const Seg& s : segs)
            if (addr >= s.load && addr + len <= s.load + s.len) return &(*img)[s.data + (addr - s.load)];
        return nullptr;
    }
};

// first app partition in the partition table at 0x8000 (entries: AA 50, type, subtype, offset, size)
static size_t appOffset(const std::vector<uint8_t>& flash)
{
    for (size_t p = 0x8000; p + 32 <= flash.size() && p < 0x9000; p += 32)
    {
        if (flash[p] != 0xAA || flash[p + 1] != 0x50) break;
        if (flash[p + 2] == 0) return get32(&flash[p + 4]);   // type 0 = app
    }
    return 0x10000;
}

// ---------------------------------------------------------------------------
//  patch the firmware
// ---------------------------------------------------------------------------
static uint32_t g_helperAddr = 0;                     // guest address of the helper (log only)

bool btPatchFirmware(std::vector<uint8_t>& flash, const std::string& firmwarePath, void (*log)(const char*, ...))
{
    // <name>.merged.bin -> <name>.elf
    std::string elfPath = firmwarePath;
    size_t m = elfPath.rfind(".merged.bin");
    if (m == std::string::npos) return false;
    elfPath = elfPath.substr(0, m) + ".elf";
    ElfFile elf;
    if (!elf.load(elfPath))
    {
        if (GetFileAttributesA(elfPath.c_str()) != INVALID_FILE_ATTRIBUTES)
            log("Bluetooth: cannot read %s - Bluetooth not simulated", elfPath.c_str());
        return false;
    }
    if (!elf.find("esp_vhci_host_send_packet") || !elf.find("esp_bluedroid_init")) return false;   // no Bluetooth in this sketch

    AppImage app;
    if (!app.parse(flash, appOffset(flash))) { log("Bluetooth: app image not understood - Bluetooth not simulated"); return false; }
    uint8_t sha[32];
    if (app.checksum() != flash[app.csumPos]) { log("Bluetooth: app image checksum mismatch - Bluetooth not simulated"); return false; }
    if (app.hashed)
    {
        app.sha256(sha);
        if (memcmp(sha, &flash[app.csumPos + 1], 32) != 0) { log("Bluetooth: app image SHA-256 mismatch - Bluetooth not simulated"); return false; }
    }

    struct Fn { const char* name; int id; bool needed, args; };
    static const Fn FNS[] = {
        { "esp_bt_controller_init", 1, true, true }, { "esp_bt_controller_deinit", 2, false, false },
        { "esp_bt_controller_enable", 3, true, true }, { "esp_bt_controller_disable", 4, false, false },
        { "esp_bt_controller_get_status", 5, true, false }, { "esp_vhci_host_check_send_available", 6, true, false },
        { "esp_vhci_host_send_packet", 7, true, true }, { "esp_vhci_host_register_callback", 8, true, true },
    };
    // every function: present, big enough, the ELF bytes match the image (the right .elf)
    const ElfSym* initSym = elf.find("esp_bt_controller_init");
    for (const Fn& f : FNS)
    {
        const ElfSym* s = elf.find(f.name);
        if (!s) { if (f.needed) { log("Bluetooth: %s not in the .elf - Bluetooth not simulated", f.name); return false; } continue; }
        uint32_t need = f.id == 1 ? 16 + (uint32_t)sizeof(BT_BLOB) : f.id == 8 ? 4 : (uint32_t)makeStub(f.id, f.args).size();
        const uint8_t* e = elf.bytesAt(s->addr, need);
        uint8_t* p = app.at(s->addr, need);
        // room = size rounded up to 4: the padding up to the next 4-byte boundary is free (the next
        // function's literal pool, which comes right before its code, is 4-byte aligned)
        if ((s->addr & 3) || (s->size && ((s->size + 3) & ~3u) < need) || !e || !p || memcmp(e, p, need) != 0)
        {
            log("Bluetooth: %s does not match the firmware (other .elf?) - Bluetooth not simulated", f.name);
            return false;
        }
    }
    const ElfSym* create = elf.find("xTaskCreatePinnedToCore");
    const ElfSym* delay = elf.find("vTaskDelay");
    if (!create || !delay) { log("Bluetooth: FreeRTOS functions not in the .elf - Bluetooth not simulated"); return false; }

    // blob after the stub of esp_bt_controller_init
    uint32_t blobAddr = initSym->addr + 16;
    uint8_t blob[sizeof(BT_BLOB)];
    memcpy(blob, BT_BLOB, sizeof(blob));
    put32(blob + LIT_MBOX, BT_MBOX);
    put32(blob + LIT_RXBUF, BT_MBOX + BT_RXBUF);
    put32(blob + LIT_NAME, BT_MBOX + 0x40);
    put32(blob + LIT_TASK, blobAddr + BLOB_RXTASK);
    put32(blob + LIT_CREATE, create->addr);
    put32(blob + LIT_DELAY, delay->addr);
    uint32_t helper = blobAddr + BLOB_HELPER;

    // all checks first, then write
    struct Write { uint8_t* p; std::vector<uint8_t> bytes; };
    std::vector<Write> writes;
    for (const Fn& f : FNS)
    {
        const ElfSym* s = elf.find(f.name);
        if (!s) continue;
        std::vector<uint8_t> code;
        if (f.id == 8)                                        // register_callback: j regcb
        {
            code.resize(3);
            if (!encodeJ(code.data(), s->addr, blobAddr + BLOB_REGCB)) { log("Bluetooth: %s too far from the stubs", f.name); return false; }
        }
        else
        {
            code = makeStub(f.id, f.args);
            size_t cp = stubCallPos(f.args);
            if (!encodeCall8(&code[cp], s->addr + (uint32_t)cp, helper)) { log("Bluetooth: %s too far from the stubs", f.name); return false; }
        }
        writes.push_back({ app.at(s->addr, (uint32_t)code.size()), code });
    }
    uint8_t* bp = app.at(blobAddr, sizeof(blob));
    if (!bp) { log("Bluetooth: no room for the stubs - Bluetooth not simulated"); return false; }
    writes.push_back({ bp, std::vector<uint8_t>(blob, blob + sizeof(blob)) });
    for (const Write& w : writes) memcpy(w.p, w.bytes.data(), w.bytes.size());

    flash[app.csumPos] = app.checksum();
    if (app.hashed) { app.sha256(sha); memcpy(&flash[app.csumPos + 1], sha, 32); }
    g_helperAddr = helper;
    log("Bluetooth: controller replaced by the engine's virtual controller (%s, stubs at 0x%08X)", elfPath.c_str(), blobAddr);
    return true;
}

// ---------------------------------------------------------------------------
//  virtual controller (HCI)
// ---------------------------------------------------------------------------
static BtEnv    g_env;
static uint8_t  g_bdaddr[6];                          // HCI order (LSB first)
static int      g_status = 0;                         // esp_bt_controller_status_t: 0 idle, 1 inited, 2 enabled
static uint32_t g_hostCb = 0, g_notifyRecv = 0;
static bool     g_taskMade = false;
static uint32_t g_arg[2], g_result;
static uint8_t  g_hostFeatures = 0;                  // ext. features page 1: bit 0 SSP, 1 LE, 2 LE + BR/EDR, 3 SC (host)
static std::deque<std::vector<uint8_t>> g_toHost;     // H4 packets for the host

// LE advertising / scanning state (from the host's commands); the link sends / receives it
struct LeAdv
{
    bool     on = false;
    uint8_t  type = 0, ownType = 0;                   // ADV_IND / DIRECT / SCAN_IND / NONCONN_IND; own address public / random
    uint16_t interval = 0x800;                        // units of 0.625 ms
    uint8_t  data[31] = {}, len = 0, rsp[31] = {}, rspLen = 0;
};
static LeAdv    g_adv;
static uint8_t  g_randAddr[6];                        // LE Set Random Address (HCI order)
static bool     g_scanOn = false, g_scanActive = false, g_scanNoDup = false;
static std::vector<std::vector<uint8_t>> g_seen;      // reported (type, address) while filtering duplicates
static int64_t  g_advNext = 0;                        // virtual time of the next advertising packet

// Bluetooth Classic: what the host set (BluetoothSerial)
static uint8_t  g_scanEnable = 0;                     // Write Scan Enable: bit 0 inquiry scan, bit 1 page scan
static uint8_t  g_cod[3] = {};                        // Class of Device
static char     g_localName[249] = "ESP32";
static std::vector<uint8_t> g_eir;                    // Extended Inquiry Response data
static int64_t  g_brNext = 0;                         // virtual time of the next BTL_BR_INQ
struct BrPending { bool on; uint32_t link; uint8_t addr[6], cod[3]; std::string name; };
static BrPending g_brPending;                          // a pager waits for our host's Accept Connection Request
static uint8_t  g_rx[BT_MBOX_SIZE - BT_RXBUF];        // packet being read by the host
static QemuMemoryRegionOps     g_ops;
static QemuMemoryRegionStorage g_mr;

static void toHost(std::vector<uint8_t> pkt) { g_toHost.push_back(std::move(pkt)); }

static void event(uint8_t code, const std::vector<uint8_t>& params)
{
    std::vector<uint8_t> p = { 0x04, code, (uint8_t)params.size() };
    p.insert(p.end(), params.begin(), params.end());
    toHost(p);
}

static void cmdComplete(uint16_t op, std::vector<uint8_t> ret)
{
    std::vector<uint8_t> p = { 1, (uint8_t)op, (uint8_t)(op >> 8) };
    p.insert(p.end(), ret.begin(), ret.end());
    event(0x0E, p);
}

static void cmdStatus(uint16_t op, uint8_t status)
{
    event(0x0F, { status, 1, (uint8_t)op, (uint8_t)(op >> 8) });
}

static void add16(std::vector<uint8_t>& v, uint16_t x) { v.push_back((uint8_t)x); v.push_back((uint8_t)(x >> 8)); }

static void aes128(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
    // LE Encrypt: key and data come MSB last (HCI order) -> AES works on MSB first
    uint8_t k[16], d[16], o[16];
    for (int i = 0; i < 16; i++) { k[i] = key[15 - i]; d[i] = in[15 - i]; }
    BCRYPT_KEY_HANDLE h = nullptr;
    ULONG n = 0;
    BCryptGenerateSymmetricKey(BCRYPT_AES_ECB_ALG_HANDLE, &h, nullptr, 0, k, 16, 0);
    BCryptEncrypt(h, d, 16, nullptr, nullptr, 0, o, 16, &n, 0);
    BCryptDestroyKey(h);
    for (int i = 0; i < 16; i++) out[i] = o[15 - i];
}

// LE connections (see "connections" at the end)
static int  connCommand(uint16_t op, const uint8_t* a, size_t alen, std::vector<uint8_t>& r);
static void aclFromHost(const uint8_t* p, size_t len);
static void connOnAdv(uint8_t addrType, const uint8_t* addr, uint8_t advType);
static void connReceive(uint8_t type, const uint8_t* b, size_t bl);
static void connReset();
static void pcSlice();                                // PC bridge requests (BLEPC)

static void hciCommand(const uint8_t* c, size_t len)
{
    if (len < 3) return;
    uint16_t op = get16(c);
    const uint8_t* a = c + 3;
    size_t alen = len - 3;
    std::vector<uint8_t> r = { 0 };                   // status OK + return parameters
    if (op == 0x0C35) return;                         // Host Number Of Completed Packets: no event (spec)
    int done = connCommand(op, a, alen, r);           // 1: Command Complete with r, 2: answered already
    if (done == 2) return;
    if (done == 1) { cmdComplete(op, r); return; }
    switch (op)
    {
    // ---- controller information (values like a real ESP32: Bluetooth 4.2, Espressif 0x02E5) ----
    case 0x1001: r.push_back(8); add16(r, 0x030E); r.push_back(8); add16(r, 0x02E5); add16(r, 0x030E); break;  // Read Local Version
    case 0x1009: r.insert(r.end(), g_bdaddr, g_bdaddr + 6); break;                                          // Read BD_ADDR
    case 0x1002: for (int i = 0; i < 64; i++) r.push_back(0xFF); break;                                     // Read Local Supported Commands
    case 0x1003: { static const uint8_t F[8] = { 0xBF, 0xEE, 0xCD, 0xFE, 0xDB, 0xFF, 0x7B, 0x87 }; r.insert(r.end(), F, F + 8); break; }
    case 0x1004:                                                                                              // Read Local Extended Features
    {
        uint8_t page = alen ? a[0] : 0;
        static const uint8_t P0[8] = { 0xBF, 0xEE, 0xCD, 0xFE, 0xDB, 0xFF, 0x7B, 0x87 };
        r.push_back(page); r.push_back(2);
        for (int i = 0; i < 8; i++) r.push_back(page == 0 ? P0[i] : page == 1 && i == 0 ? g_hostFeatures : 0);
        break;
    }
    // ---- LE advertising and scanning ----
    case 0x2005: if (alen >= 6) memcpy(g_randAddr, a, 6); break;                                           // LE Set Random Address
    case 0x2006: if (alen >= 6) { g_adv.interval = get16(a); g_adv.type = a[4]; g_adv.ownType = a[5]; } break;  // LE Set Advertising Parameters
    case 0x2008: if (alen >= 1) { g_adv.len = (uint8_t)std::min<int>(a[0], 31); memcpy(g_adv.data, a + 1, std::min<size_t>(g_adv.len, alen - 1)); } break;
    case 0x2009: if (alen >= 1) { g_adv.rspLen = (uint8_t)std::min<int>(a[0], 31); memcpy(g_adv.rsp, a + 1, std::min<size_t>(g_adv.rspLen, alen - 1)); } break;
    case 0x200A:                                                                                              // LE Set Advertising Enable
        if (alen >= 1 && (a[0] != 0) != g_adv.on)
        {
            g_adv.on = a[0] != 0;
            g_advNext = 0;
            g_env.log("Bluetooth: advertising %s", g_adv.on ? "on" : "off");
        }
        break;
    case 0x200B: if (alen >= 1) g_scanActive = a[0] == 1; break;                                            // LE Set Scan Parameters
    case 0x200C:                                                                                              // LE Set Scan Enable
        if (alen >= 2)
        {
            if ((a[0] != 0) != g_scanOn) g_env.log("Bluetooth: scanning %s (%s)", a[0] ? "on" : "off", g_scanActive ? "active" : "passive");
            g_scanOn = a[0] != 0;
            g_scanNoDup = a[1] != 0;
            g_seen.clear();
        }
        break;
    // host features (extended features page 1): Bluedroid uses BLE only when "LE Supported (Host)" is set
    case 0x0C56: if (alen) g_hostFeatures = (uint8_t)((g_hostFeatures & ~1) | (a[0] ? 1 : 0)); break;           // Write Simple Pairing Mode
    case 0x0C6D: if (alen >= 2) g_hostFeatures = (uint8_t)((g_hostFeatures & ~6) | (a[0] ? 2 : 0) | (a[1] ? 4 : 0)); break;  // Write LE Host Support
    case 0x0C7A: if (alen) g_hostFeatures = (uint8_t)((g_hostFeatures & ~8) | (a[0] ? 8 : 0)); break;           // Write Secure Connections Host Support
    case 0x1005: add16(r, 1021); r.push_back(255); add16(r, 9); add16(r, 4); break;                          // Read Buffer Size
    case 0x2002: add16(r, 251); r.push_back(10); break;                                                     // LE Read Buffer Size
    case 0x2003: { r.push_back(0x3F); for (int i = 1; i < 8; i++) r.push_back(0); break; }                   // LE Read Local Supported Features
    case 0x201C: for (int i = 0; i < 8; i++) r.push_back(0xFF); break;                                      // LE Read Supported States
    case 0x200F: r.push_back(12); break;                                                                    // LE Read White List Size
    case 0x202A: r.push_back(10); break;                                                                    // LE Read Resolving List Size
    case 0x202F: add16(r, 251); add16(r, 2120); add16(r, 251); add16(r, 2120); break;                       // LE Read Maximum Data Length
    case 0x2023: add16(r, 27); add16(r, 328); break;                                                        // LE Read Suggested Default Data Length
    case 0x2007: r.push_back(0); break;                                                                     // LE Read Advertising Channel Tx Power
    case 0x0C14: r.insert(r.end(), g_localName, g_localName + 248); break;                                  // Read Local Name
    // ---- Bluetooth Classic settings ----
    case 0x0C13: if (alen) { memset(g_localName, 0, sizeof(g_localName)); memcpy(g_localName, a, (std::min)(alen, (size_t)248)); } break;   // Write Local Name
    case 0x0C1A:                                                                                              // Write Scan Enable
        if (alen && a[0] != g_scanEnable)
        {
            g_scanEnable = a[0];
            g_env.log("Bluetooth Classic: %s%s (name %s)", g_scanEnable & 1 ? "discoverable " : "", g_scanEnable & 2 ? "connectable" : g_scanEnable ? "" : "not visible", g_localName);
        }
        break;
    case 0x0C24: if (alen >= 3) memcpy(g_cod, a, 3); break;                                                // Write Class of Device
    case 0x0C52: if (alen >= 2) g_eir.assign(a + 1, a + (std::min)(alen, (size_t)241)); break;            // Write Extended Inquiry Response
    case 0x2018: { uint8_t rnd[8]; BCryptGenRandom(nullptr, rnd, 8, BCRYPT_USE_SYSTEM_PREFERRED_RNG); r.insert(r.end(), rnd, rnd + 8); break; }  // LE Rand
    case 0x2017:                                                                                              // LE Encrypt
    {
        uint8_t out[16] = {};
        if (alen >= 32) aes128(a, a + 16, out);
        r.insert(r.end(), out, out + 16);
        break;
    }
    // ---- commands answered with Command Status (the result comes later as an event) ----
    case 0x0401: case 0x0405: case 0x0406: case 0x0409: case 0x040A: case 0x0411: case 0x0413: case 0x0419:
    case 0x041B: case 0x041C: case 0x041D: case 0x200D: case 0x2013: case 0x2016: case 0x2019: case 0x2025:
    case 0x2026: case 0x2032:
        g_env.log("Bluetooth: HCI command 0x%04X (status only, not simulated yet)", op);
        cmdStatus(op, 0);
        return;
    default:
    {
        static int logged;
        if (logged < 60) { logged++; g_env.log("Bluetooth: HCI command 0x%04X (%u bytes) -> OK", op, (unsigned)alen); }
        break;
    }
    }
    cmdComplete(op, r);
}

static void hostPacket(const uint8_t* p, size_t len)
{
    if (len < 1) return;
    if (p[0] == 0x01) hciCommand(p + 1, len - 1);
    else if (p[0] == 0x02) aclFromHost(p + 1, len - 1);
    else
    {
        static int logged;
        if (logged < 10) { logged++; g_env.log("Bluetooth: host packet type %u (%u bytes) dropped (not simulated yet)", p[0], (unsigned)len); }
    }
}

// ---------------------------------------------------------------------------
//  register block
// ---------------------------------------------------------------------------
static uint32_t engineCall(uint32_t id)
{
    switch (id)
    {
    case 1: if (g_status == 0) g_status = 1; g_env.log("Bluetooth: controller init"); return 0;
    case 2: g_status = 0; return 0;
    case 3:
        if (g_status != 1) return 0x103;              // ESP_ERR_INVALID_STATE
        g_status = 2;
        g_env.log("Bluetooth: controller enabled (mode %u)", g_arg[0]);
        return 0;
    case 4: if (g_status == 2) g_status = 1; return 0;
    case 5: return (uint32_t)g_status;
    case 6: return 1;
    case 7:
    {
        uint32_t n = g_arg[1] & 0xFFFF;
        std::vector<uint8_t> buf(n);
        if (n) g_env.physRead(g_arg[0], buf.data(), n);
        hostPacket(buf.data(), n);
        return 0;
    }
    case 8:
        g_hostCb = g_arg[0];
        g_notifyRecv = g_arg[1];                      // read by the guest (the struct is in flash)
        g_env.log("Bluetooth: host registered (notify_host_recv 0x%08X)", g_notifyRecv);
        return 0;
    case 9:
        if (g_taskMade) return 0;
        g_taskMade = true;
        return 1;
    default:
        return 0;
    }
}

static uint64_t mbox_read(void*, uint64_t addr, unsigned size)
{
    uint32_t a = (uint32_t)addr;
    if (a >= BT_RXBUF)
    {
        uint64_t v = 0;
        for (unsigned i = 0; i < size && a - BT_RXBUF + i < sizeof(g_rx); i++) v |= (uint64_t)g_rx[a - BT_RXBUF + i] << (8 * i);
        return v;
    }
    if (a >= 0x40 && a < 0x48)
    {
        static const char NAME[8] = "vhci";
        uint64_t v = 0;
        for (unsigned i = 0; i < size && a - 0x40 + i < 8; i++) v |= (uint64_t)(uint8_t)NAME[a - 0x40 + i] << (8 * i);
        return v;
    }
    switch (a)
    {
    case 12: return g_result;
    case 16:
    {
        if (g_toHost.empty() || !g_notifyRecv) return 0;
        std::vector<uint8_t> p = std::move(g_toHost.front());
        g_toHost.pop_front();
        if (getenv("BT_TRACE"))
        {
            char hex[200] = "";
            for (size_t i = 0; i < p.size() && i < 40; i++) sprintf_s(hex + i * 3, sizeof(hex) - i * 3, "%02X ", p[i]);
            g_env.log("BTTRACE to host (%u): %s", (unsigned)p.size(), hex);
        }
        size_t n = std::min(p.size(), sizeof(g_rx));
        memcpy(g_rx, p.data(), n);
        return n;
    }
    case 20: return g_notifyRecv;
    default: return 0;
    }
}

static void mbox_write(void*, uint64_t addr, uint64_t data, unsigned)
{
    switch ((uint32_t)addr)
    {
    case 4: g_arg[0] = (uint32_t)data; break;
    case 8: g_arg[1] = (uint32_t)data; break;
    case 0: g_result = engineCall((uint32_t)data); break;
    default: break;
    }
}

static void linkOpen(int port);

void btInstall(const BtEnv& env, const uint8_t mac[6], int port, bool pc)
{
    g_env = env;
    // Bluetooth address = base MAC + 2 (IDF esp_read_mac ESP_MAC_BT), HCI order LSB first
    uint8_t bt[6];
    memcpy(bt, mac, 6);
    bt[5] = (uint8_t)(bt[5] + 2);
    for (int i = 0; i < 6; i++) g_bdaddr[i] = bt[5 - i];
    memset(&g_ops, 0, sizeof(g_ops));
    g_ops.read = mbox_read;
    g_ops.write = mbox_write;
    g_ops.endianness = 2;
    g_ops.valid_min_access_size = 1;
    g_ops.valid_max_access_size = 4;
    g_ops.impl_min_access_size = 1;
    g_ops.impl_max_access_size = 4;
    memset(&g_mr, 0, sizeof(g_mr));
    env.mrInitIo(&g_mr, nullptr, &g_ops, nullptr, "esp32link.bt", BT_MBOX_SIZE);
    env.mrAddOverlap(env.getSystemMemory(), BT_MBOX, &g_mr, 10);
    btReset();
    linkOpen(port);
    if (pc) btPcStart(env.log);
    env.log("Bluetooth: virtual controller at 0x%08X, address %02X:%02X:%02X:%02X:%02X:%02X", BT_MBOX,
            bt[0], bt[1], bt[2], bt[3], bt[4], bt[5]);
}

void btReset()
{
    g_status = 0;
    g_hostCb = g_notifyRecv = 0;
    g_taskMade = false;
    g_hostFeatures = 0;
    g_toHost.clear();
    g_adv = LeAdv();
    g_scanOn = g_scanActive = g_scanNoDup = false;
    g_seen.clear();
    g_scanEnable = 0;
    memset(g_cod, 0, sizeof(g_cod));
    strcpy_s(g_localName, "ESP32");
    g_eir.clear();
    g_brPending = BrPending();
    connReset();                                      // the peers see a connection timeout
}

// ---------------------------------------------------------------------------
//  link between the ESP32 parts of one design: UDP multicast 230.0.0.2:port, only on 127.0.0.1
//  (like the ESP-NOW link). Every message: "E32B", version 1, type, sender nonce (own messages
//  come back and are skipped), then the body.
//    type 1 ADV: address type, address (HCI order), advertising type, len, data, rsp len, rsp data
// ---------------------------------------------------------------------------
static SOCKET      g_sock = INVALID_SOCKET;
static sockaddr_in g_group;
static uint32_t    g_nonce;
enum { MSG_ADV = BTL_ADV };                          // message types: btlink.h

static void linkOpen(int port)
{
    WSADATA w;
    if (WSAStartup(MAKEWORD(2, 2), &w) != 0) { g_env.log("Bluetooth link: WSAStartup failed"); return; }
    BCryptGenRandom(nullptr, (PUCHAR)&g_nonce, sizeof(g_nonce), BCRYPT_USE_SYSTEM_PREFERRED_RNG);
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) { g_env.log("Bluetooth link: no socket (%d)", WSAGetLastError()); return; }
    BOOL yes = TRUE;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
    // bound to 127.0.0.1, not INADDR_ANY: a socket that listens on all interfaces makes the Windows
    // firewall ask the user for permission (Windows delivers the multicast to a socket bound to the
    // interface address)
    in_addr lo;
    inet_pton(AF_INET, "127.0.0.1", &lo);
    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_port = htons((u_short)port);
    local.sin_addr = lo;
    ip_mreq mreq = {};
    inet_pton(AF_INET, "230.0.0.2", &mreq.imr_multiaddr);
    mreq.imr_interface = lo;
    DWORD loop = 1;
    u_long nb = 1;
    if (bind(s, (sockaddr*)&local, sizeof(local)) != 0
        || setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char*)&mreq, sizeof(mreq)) != 0
        || setsockopt(s, IPPROTO_IP, IP_MULTICAST_IF, (const char*)&lo, sizeof(lo)) != 0
        || setsockopt(s, IPPROTO_IP, IP_MULTICAST_LOOP, (const char*)&loop, sizeof(loop)) != 0
        || ioctlsocket(s, FIONBIO, &nb) != 0)
    {
        g_env.log("Bluetooth link: port %d not usable (%d) - no link to other ESP32 parts", port, WSAGetLastError());
        closesocket(s);
        return;
    }
    g_group = {};
    g_group.sin_family = AF_INET;
    g_group.sin_port = htons((u_short)port);
    g_group.sin_addr = mreq.imr_multiaddr;
    g_sock = s;
    g_env.log("Bluetooth link: UDP 230.0.0.2:%d on 127.0.0.1 (BTPORT)", port);
}

static void linkSend(uint8_t type, const std::vector<uint8_t>& body)
{
    if (g_sock == INVALID_SOCKET) return;
    std::vector<uint8_t> m = { 'E', '3', '2', 'B', 1, type };
    m.insert(m.end(), (uint8_t*)&g_nonce, (uint8_t*)&g_nonce + 4);
    m.insert(m.end(), body.begin(), body.end());
    sendto(g_sock, (const char*)m.data(), (int)m.size(), 0, (const sockaddr*)&g_group, sizeof(g_group));
}

// our advertising address (HCI order) and its type
static const uint8_t* advAddr(uint8_t& type)
{
    type = g_adv.ownType & 1;                          // 0 public, 1 random (2 / 3: resolvable, not simulated)
    return type ? g_randAddr : g_bdaddr;
}

static void sendAdv()
{
    uint8_t at;
    const uint8_t* addr = advAddr(at);
    std::vector<uint8_t> b = { at };
    b.insert(b.end(), addr, addr + 6);
    b.push_back(g_adv.type);
    b.push_back(g_adv.len);
    b.insert(b.end(), g_adv.data, g_adv.data + g_adv.len);
    b.push_back(g_adv.rspLen);
    b.insert(b.end(), g_adv.rsp, g_adv.rsp + g_adv.rspLen);
    linkSend(MSG_ADV, b);
}

// LE Advertising Report (LE Meta event 0x3E, subevent 2)
static void advReport(uint8_t evType, uint8_t addrType, const uint8_t* addr, const uint8_t* data, uint8_t len, int8_t rssi = -50)
{
    if (g_scanNoDup)
    {
        std::vector<uint8_t> key = { evType, addrType };
        key.insert(key.end(), addr, addr + 6);
        if (std::find(g_seen.begin(), g_seen.end(), key) != g_seen.end()) return;
        g_seen.push_back(key);
    }
    std::vector<uint8_t> p = { 0x02, 1, evType, addrType };
    p.insert(p.end(), addr, addr + 6);
    p.push_back(len);
    p.insert(p.end(), data, data + len);
    p.push_back((uint8_t)rssi);                       // RSSI: -50 for the parts ("close"), real for real devices
    event(0x3E, p);
}

static void linkReceive(const uint8_t* m, size_t n)
{
    if (n < 10 || memcmp(m, "E32B", 4) != 0 || m[4] != 1 || memcmp(m + 6, &g_nonce, 4) == 0) return;
    const uint8_t* b = m + 10;
    size_t bl = n - 10;
    if (m[5] != MSG_ADV) { connReceive(m[5], b, bl); return; }
    if (bl >= 9 && !g_scanOn)                         // not scanning: only for a pending connection
    {
        connOnAdv(b[0], b + 1, b[7]);
        return;
    }
    if (bl >= 9)
    {
        uint8_t at = b[0], type = b[7], len = b[8];
        if (bl < 9u + len + 1) return;
        uint8_t rspLen = b[9 + len];
        if (bl < 10u + len + rspLen) return;
        connOnAdv(at, b + 1, type);                       // a pending LE Create Connection to this device
        static const uint8_t EV[5] = { 0, 1, 2, 3, 1 };   // ADV_IND, DIRECT, SCAN_IND, NONCONN_IND, DIRECT (low duty)
        advReport(type < 5 ? EV[type] : 0, at, b + 1, b + 9, len);
        if (g_scanActive && (type == 0 || type == 2))     // active scan: the scan response too
            advReport(4, at, b + 1, b + 10 + len, rspLen);
    }
}

void btSlice(int64_t nowNs)
{
    pcSlice();
    if (g_sock == INVALID_SOCKET) return;
    if (g_adv.on && g_status == 2 && nowNs >= g_advNext)
    {
        sendAdv();
        int64_t iv = (int64_t)g_adv.interval * 625000;    // 0.625 ms units; at least 100 ms (fewer reports)
        g_advNext = nowNs + std::max<int64_t>(iv, 100000000);
    }
    if (g_status == 2 && (g_scanEnable & 1) && nowNs >= g_brNext)   // Classic: discoverable (inquiry scan)
    {
        std::vector<uint8_t> b(g_bdaddr, g_bdaddr + 6);
        b.insert(b.end(), g_cod, g_cod + 3);
        size_t nl = strnlen(g_localName, 248);
        b.push_back((uint8_t)nl);
        b.insert(b.end(), g_localName, g_localName + nl);
        b.push_back((uint8_t)g_eir.size());
        b.insert(b.end(), g_eir.begin(), g_eir.end());
        linkSend(BTL_BR_INQ, b);
        g_brNext = nowNs + 300000000;
    }
    uint8_t buf[600];
    for (;;)
    {
        int r = recv(g_sock, (char*)buf, sizeof(buf), 0);
        if (r <= 0) break;
        if (g_status == 2) linkReceive(buf, (size_t)r);
    }
}
// ---------------------------------------------------------------------------
//  LE connections between parts (phase 2b). The initiator (LE Create Connection) waits for an
//  advertising packet of its target, then sends CONNECT. The target, when it advertises
//  connectable at that address, stops advertising (like a real link layer), reports LE Connection
//  Complete as slave and answers CONN_ACK; the initiator then reports it as master. A connection
//  is known on both sides by a link id (random, from the initiator).
//    MSG_CONNECT:  target type, target addr, initiator type, initiator addr, link, interval, latency, timeout
//    MSG_CONN_ACK: link, target type, target addr
//    MSG_ACL:      link, packet boundary flag, data (one host fragment)
//    MSG_DISC:     link, reason
//    MSG_CONN_UPD: link, interval, latency, timeout
//    MSG_DLE:      link, tx octets, tx time
//    MSG_ENC_REQ:  link, rand (8), ediv (2)         (master's LE Start Encryption)
//    MSG_ENC_RSP:  link, 1 + LTK (16) / 0           (slave host's LTK reply / negative reply)
//    MSG_ENC_DONE: link                             (keys matched: the slave reports Encryption Change)
// ---------------------------------------------------------------------------
enum { MSG_CONNECT = BTL_CONNECT, MSG_CONN_ACK = BTL_CONN_ACK, MSG_ACL = BTL_ACL, MSG_DISC = BTL_DISC, MSG_CONN_UPD = BTL_CONN_UPD,
       MSG_DLE = BTL_DLE, MSG_ENC_REQ = BTL_ENC_REQ, MSG_ENC_RSP = BTL_ENC_RSP, MSG_ENC_DONE = BTL_ENC_DONE };   // btlink.h

struct Conn
{
    uint16_t handle;
    uint32_t link;
    uint8_t  role;                                    // 0 master, 1 slave
    uint8_t  peerType, peer[6];
    uint16_t interval, latency, timeout;
    bool     encrypted, encPending;               // encPending: master waits for the slave's LTK
    bool     local;                               // peer = the PC bridge in this engine (BLEPC), not another part
    bool     br;                                  // Bluetooth Classic (BR/EDR) connection
    std::string name;                             // Classic: the peer's name (Remote Name Request)
    uint8_t  ltk[16];                             // master: the key of its LE Start Encryption
};
static std::vector<Conn> g_conns;
static uint16_t g_nextHandle = 1;

struct Initiating
{
    bool     on = false, sent = false;
    uint8_t  policy = 0, peerType = 0, peer[6] = {}, ownType = 0;
    uint16_t interval = 0x18, latency = 0, timeout = 0x48;
    uint32_t link = 0;
};
static Initiating g_init;
static std::vector<std::vector<uint8_t>> g_white;     // white list: type + address

static Conn* connByHandle(uint16_t h) { for (Conn& c : g_conns) if (c.handle == h) return &c; return nullptr; }
static Conn* connByLink(uint32_t l) { for (Conn& c : g_conns) if (c.link == l) return &c; return nullptr; }
static void add32(std::vector<uint8_t>& v, uint32_t x) { for (int i = 0; i < 4; i++) v.push_back((uint8_t)(x >> (8 * i))); }

static uint16_t newHandle()
{
    for (;;)
    {
        uint16_t h = g_nextHandle;
        g_nextHandle = (uint16_t)(g_nextHandle % 0xEFF + 1);
        if (!connByHandle(h)) return h;
    }
}

static void connComplete(const Conn& c)
{
    std::vector<uint8_t> p = { 0x01, 0 };
    add16(p, c.handle);
    p.push_back(c.role);
    p.push_back(c.peerType);
    p.insert(p.end(), c.peer, c.peer + 6);
    add16(p, c.interval); add16(p, c.latency); add16(p, c.timeout);
    p.push_back(0);                                   // master clock accuracy
    event(0x3E, p);
    g_env.log("Bluetooth: connected as %s to %02X:%02X:%02X:%02X:%02X:%02X (handle %u)", c.role ? "slave" : "master",
              c.peer[5], c.peer[4], c.peer[3], c.peer[2], c.peer[1], c.peer[0], c.handle);
}

static void connGone(uint16_t handle, uint8_t reason)
{
    event(0x05, { 0, (uint8_t)handle, (uint8_t)(handle >> 8), reason });     // Disconnection Complete
    g_conns.erase(std::remove_if(g_conns.begin(), g_conns.end(), [&](const Conn& c) { return c.handle == handle; }), g_conns.end());
    g_env.log("Bluetooth: disconnected (handle %u, reason 0x%02X)", handle, reason);
}

static void dataLengthChange(uint16_t handle, uint16_t tx, uint16_t txTime, uint16_t rx, uint16_t rxTime)
{
    std::vector<uint8_t> p = { 0x07 };
    add16(p, handle); add16(p, tx); add16(p, txTime); add16(p, rx); add16(p, rxTime);
    event(0x3E, p);
}

static void connUpdateComplete(const Conn& c)
{
    std::vector<uint8_t> p = { 0x03, 0 };
    add16(p, c.handle); add16(p, c.interval); add16(p, c.latency); add16(p, c.timeout);
    event(0x3E, p);
}

// 0 = not a connection command, 1 = Command Complete with r, 2 = answered here
static int connCommand(uint16_t op, const uint8_t* a, size_t alen, std::vector<uint8_t>& r)
{
    uint16_t h = alen >= 2 ? (uint16_t)(get16(a) & 0x0FFF) : 0xFFFF;
    Conn* c = connByHandle(h);
    switch (op)
    {
    case 0x200D:                                      // LE Create Connection
        if (alen < 25) return 0;
        if (g_init.on) { cmdStatus(op, 0x0C); return 2; }   // Command Disallowed
        g_init = Initiating();
        g_init.on = true;
        g_init.policy = a[4];
        g_init.peerType = a[5];
        memcpy(g_init.peer, a + 6, 6);
        g_init.ownType = a[12];
        g_init.interval = get16(a + 13);
        g_init.latency = get16(a + 17);
        g_init.timeout = get16(a + 19);
        BCryptGenRandom(nullptr, (PUCHAR)&g_init.link, sizeof(g_init.link), BCRYPT_USE_SYSTEM_PREFERRED_RNG);
        cmdStatus(op, 0);
        if (g_init.policy) g_env.log("Bluetooth: connecting to a white-list device");
        else g_env.log("Bluetooth: connecting to %02X:%02X:%02X:%02X:%02X:%02X", a[11], a[10], a[9], a[8], a[7], a[6]);
        return 2;
    case 0x200E:                                      // LE Create Connection Cancel
        if (!g_init.on) { r[0] = 0x0C; return 1; }
        g_init.on = false;
        cmdComplete(op, { 0 });
        {
            std::vector<uint8_t> p = { 0x01, 0x02 };  // LE Connection Complete, Unknown Connection Identifier
            p.resize(19, 0);
            event(0x3E, p);
        }
        return 2;
    // ---- Bluetooth Classic connection commands ----
    case 0x0409:                                      // Accept Connection Request (addr, role)
    case 0x040A:                                      // Reject Connection Request (addr, reason)
    {
        if (alen < 7 || !g_brPending.on || memcmp(a, g_brPending.addr, 6) != 0) { cmdStatus(op, 0x02); return 2; }
        cmdStatus(op, 0);
        g_brPending.on = false;
        if (op == 0x040A)
        {
            std::vector<uint8_t> p = { a[6], 0, 0 };  // Connection Complete with the reason
            p.insert(p.end(), a, a + 6);
            p.push_back(1); p.push_back(0);
            event(0x03, p);
            std::vector<uint8_t> d;
            add32(d, g_brPending.link);
            d.push_back(a[6]);
            linkSend(MSG_DISC, d);
            return 2;
        }
        Conn n = {};
        n.handle = newHandle();
        n.link = g_brPending.link;
        n.role = a[6] == 0 ? 0 : 1;                   // 0: our host asked to become master
        n.br = true;
        memcpy(n.peer, g_brPending.addr, 6);
        n.name = g_brPending.name;
        g_conns.push_back(n);
        if (n.role == 0)                              // Role Change: we are the master now
        {
            std::vector<uint8_t> p = { 0 };
            p.insert(p.end(), a, a + 6);
            p.push_back(0);
            event(0x12, p);
        }
        std::vector<uint8_t> p = { 0, (uint8_t)n.handle, (uint8_t)(n.handle >> 8) };   // Connection Complete
        p.insert(p.end(), a, a + 6);
        p.push_back(1);                               // ACL
        p.push_back(0);                               // no encryption
        event(0x03, p);
        std::vector<uint8_t> ack;
        add32(ack, n.link);
        ack.push_back(0);
        ack.insert(ack.end(), g_bdaddr, g_bdaddr + 6);
        linkSend(MSG_CONN_ACK, ack);
        g_env.log("Bluetooth Classic: connected to %s (handle %u)", n.name.empty() ? "a device" : n.name.c_str(), n.handle);
        return 2;
    }
    case 0x0419:                                      // Remote Name Request (addr, ...)
    {
        if (alen < 6) return 0;
        cmdStatus(op, 0);
        std::string nm;
        bool known = false;
        for (Conn& x : g_conns) if (x.br && memcmp(x.peer, a, 6) == 0) { nm = x.name; known = true; }
        if (g_brPending.on && memcmp(g_brPending.addr, a, 6) == 0) { nm = g_brPending.name; known = true; }
        std::vector<uint8_t> p = { (uint8_t)(known ? 0 : 0x04) };   // 0x04 page timeout
        p.insert(p.end(), a, a + 6);
        p.insert(p.end(), nm.begin(), nm.end());
        p.resize(7 + 248, 0);
        event(0x07, p);
        return 2;
    }
    case 0x041B:                                      // Read Remote Supported Features
    case 0x041C:                                      // Read Remote Extended Features (page)
    {
        if (!c) { cmdStatus(op, 0x02); return 2; }
        cmdStatus(op, 0);
        static const uint8_t F[8] = { 0xBF, 0xEE, 0xCD, 0xFE, 0xDB, 0xFF, 0x7B, 0x87 };
        std::vector<uint8_t> p = { 0 };
        add16(p, h);
        if (op == 0x041C) { p.push_back(alen >= 3 ? a[2] : 0); p.push_back(1); }
        bool page0 = op == 0x041B || (alen >= 3 && a[2] == 0);
        for (int i = 0; i < 8; i++) p.push_back(page0 ? F[i] : 0);
        event(op == 0x041B ? 0x0B : 0x23, p);
        return 2;
    }
    case 0x040F:                                      // Change Connection Packet Type
    {
        if (!c || alen < 4) { cmdStatus(op, 0x02); return 2; }
        cmdStatus(op, 0);
        std::vector<uint8_t> p = { 0 };
        add16(p, h); add16(p, get16(a + 2));
        event(0x1D, p);
        return 2;
    }
    case 0x0803:                                      // Sniff Mode
    case 0x0804:                                      // Exit Sniff Mode
    {
        if (!c) { cmdStatus(op, 0x02); return 2; }
        cmdStatus(op, 0);
        std::vector<uint8_t> p = { 0 };
        add16(p, h);
        p.push_back(op == 0x0803 ? 2 : 0);
        add16(p, op == 0x0803 && alen >= 4 ? get16(a + 2) : 0);
        event(0x14, p);                               // Mode Change
        return 2;
    }
    case 0x041F:                                      // Read Clock Offset
    {
        if (!c) { cmdStatus(op, 0x02); return 2; }
        cmdStatus(op, 0);
        std::vector<uint8_t> p = { 0 };
        add16(p, h); add16(p, 0);
        event(0x1C, p);                               // Read Clock Offset Complete
        return 2;
    }
    case 0x080C: if (!c) r[0] = 0x02; add16(r, h); add16(r, 0x0F); return 1;   // Read Link Policy Settings
    case 0x080D: if (!c) r[0] = 0x02; add16(r, h); return 1;                  // Write Link Policy Settings
    case 0x0C36: if (!c) r[0] = 0x02; add16(r, h); add16(r, 0x7D00); return 1; // Read Link Supervision Timeout
    case 0x0C37: if (!c) r[0] = 0x02; add16(r, h); return 1;                  // Write Link Supervision Timeout
    case 0x2010: g_white.clear(); return 1;          // LE Clear White List
    case 0x2011:                                      // LE Add Device To White List
    case 0x2012:                                      // LE Remove Device From White List
        if (alen >= 7)
        {
            std::vector<uint8_t> k(a, a + 7);
            k[0] &= 1;
            g_white.erase(std::remove(g_white.begin(), g_white.end(), k), g_white.end());
            if (op == 0x2011) g_white.push_back(k);
        }
        return 1;
    case 0x0406:                                      // Disconnect
        if (!c) { cmdStatus(op, 0x02); return 2; }
        {
            cmdStatus(op, 0);
            std::vector<uint8_t> b;
            add32(b, c->link);
            b.push_back(alen >= 3 ? a[2] : 0x13);
            if (c->local) pcOnDisconnected(c->link, alen >= 3 ? a[2] : 0x13);
            else linkSend(MSG_DISC, b);
            connGone(h, 0x16);                        // Connection Terminated By Local Host
        }
        return 2;
    case 0x2016:                                      // LE Read Remote Features
        if (!c) { cmdStatus(op, 0x02); return 2; }
        cmdStatus(op, 0);
        {
            std::vector<uint8_t> p = { 0x04, 0 };
            add16(p, h);
            p.push_back(0x3F);
            for (int i = 1; i < 8; i++) p.push_back(0);
            event(0x3E, p);
        }
        return 2;
    case 0x041D:                                      // Read Remote Version Information
        if (!c) { cmdStatus(op, 0x02); return 2; }
        cmdStatus(op, 0);
        {
            std::vector<uint8_t> p = { 0 };
            add16(p, h);
            p.push_back(8); add16(p, 0x02E5); add16(p, 0x030E);
            event(0x0C, p);
        }
        return 2;
    case 0x2013:                                      // LE Connection Update
        if (!c || alen < 10) { cmdStatus(op, 0x02); return 2; }
        cmdStatus(op, 0);
        c->interval = get16(a + 4);                   // interval max
        c->latency = get16(a + 6);
        c->timeout = get16(a + 8);
        connUpdateComplete(*c);
        if (!c->local)
        {
            std::vector<uint8_t> b;
            add32(b, c->link); add16(b, c->interval); add16(b, c->latency); add16(b, c->timeout);
            linkSend(MSG_CONN_UPD, b);
        }
        return 2;
    case 0x2022:                                      // LE Set Data Length
        if (!c || alen < 6) { r[0] = 0x02; add16(r, h); return 1; }
        r.push_back((uint8_t)h); r.push_back((uint8_t)(h >> 8));
        cmdComplete(op, r);
        {
            uint16_t tx = get16(a + 2), txTime = get16(a + 4);
            dataLengthChange(h, tx, txTime, 251, 2120);
            if (c->local) return 2;
            std::vector<uint8_t> b;
            add32(b, c->link); add16(b, tx); add16(b, txTime);
            linkSend(MSG_DLE, b);
        }
        return 2;
    case 0x1405:                                      // Read RSSI
        if (!c) r[0] = 0x02;
        add16(r, h);
        r.push_back((uint8_t)(int8_t)-50);
        return 1;
    case 0x0C7B:                                      // Read Authenticated Payload Timeout
        if (!c) r[0] = 0x02;
        add16(r, h); add16(r, 3000);
        return 1;
    case 0x1408:                                      // Read Encryption Key Size
        if (!c) r[0] = 0x02;
        add16(r, h);
        r.push_back(16);
        return 1;
    case 0x201A:                                      // LE Long Term Key Request Reply (slave): to the master
    case 0x201B:                                      // ... Negative Reply
        if (!c) { r[0] = 0x02; add16(r, h); return 1; }
        {
            std::vector<uint8_t> b;
            add32(b, c->link);
            b.push_back(op == 0x201A && alen >= 18 ? 1 : 0);
            if (op == 0x201A && alen >= 18) b.insert(b.end(), a + 2, a + 18);
            linkSend(MSG_ENC_RSP, b);
        }
        add16(r, h);
        return 1;
    case 0x0C7C:                                      // Write Authenticated Payload Timeout
    case 0x2020: case 0x2021:                         // LE Remote Connection Parameter Request (Negative) Reply
        if (!c) r[0] = 0x02;
        add16(r, h);
        return 1;
    case 0x2019:                                      // LE Start Encryption (master): ask the slave for its key
        if (!c || alen < 28 || c->role != 0) { cmdStatus(op, c ? 0x0C : 0x02); return 2; }
        if (c->local) { cmdStatus(op, 0x11); g_env.log("Bluetooth: encryption with a real device is not simulated yet"); return 2; }
        cmdStatus(op, 0);
        memcpy(c->ltk, a + 12, 16);
        c->encPending = true;
        {
            std::vector<uint8_t> b;
            add32(b, c->link);
            b.insert(b.end(), a + 2, a + 12);         // rand, ediv
            linkSend(MSG_ENC_REQ, b);
        }
        return 2;
    default:
        return 0;
    }
}

// a pending LE Create Connection: our target advertises connectable -> CONNECT
static void connOnAdv(uint8_t addrType, const uint8_t* addr, uint8_t advType)
{
    if (!g_init.on || g_init.sent || g_status != 2) return;
    if (advType != 0 && advType != 1 && advType != 4) return;   // connectable advertising only
    std::vector<uint8_t> k = { (uint8_t)(addrType & 1) };
    k.insert(k.end(), addr, addr + 6);
    bool match = g_init.policy ? std::find(g_white.begin(), g_white.end(), k) != g_white.end()
                               : (addrType & 1) == (g_init.peerType & 1) && memcmp(addr, g_init.peer, 6) == 0;
    if (!match) return;
    uint8_t ot;
    const uint8_t* own = g_init.ownType & 1 ? g_randAddr : g_bdaddr;
    ot = g_init.ownType & 1;
    g_init.peerType = addrType & 1;
    memcpy(g_init.peer, addr, 6);
    std::vector<uint8_t> b = { g_init.peerType };
    b.insert(b.end(), addr, addr + 6);
    b.push_back(ot);
    b.insert(b.end(), own, own + 6);
    add32(b, g_init.link); add16(b, g_init.interval); add16(b, g_init.latency); add16(b, g_init.timeout);
    linkSend(MSG_CONNECT, b);
    g_init.sent = true;
}

static void connReceive(uint8_t type, const uint8_t* b, size_t bl)
{
    if (type == MSG_CONNECT)
    {
        if (bl < 24 || !g_adv.on || (g_adv.type != 0 && g_adv.type != 1 && g_adv.type != 4)) return;
        uint8_t at;
        const uint8_t* mine = advAddr(at);
        if ((b[0] & 1) != at || memcmp(b + 1, mine, 6) != 0) return;   // not for us
        uint32_t link = get32(b + 14);
        if (connByLink(link)) return;
        Conn c = {};
        c.handle = newHandle();
        c.link = link;
        c.role = 1;
        c.peerType = b[7] & 1;
        memcpy(c.peer, b + 8, 6);
        c.interval = get16(b + 18); c.latency = get16(b + 20); c.timeout = get16(b + 22);
        g_adv.on = false;                             // a connection ends connectable advertising
        g_conns.push_back(c);
        std::vector<uint8_t> ack;
        add32(ack, link);
        ack.push_back(at);
        ack.insert(ack.end(), mine, mine + 6);
        linkSend(MSG_CONN_ACK, ack);
        connComplete(c);
        return;
    }
    if (type == BTL_BR_CONNECT)                       // Classic paging: our host decides (Accept / Reject)
    {
        if (bl < 20 || g_status != 2 || !(g_scanEnable & 2) || memcmp(b, g_bdaddr, 6) != 0 || g_brPending.on) return;
        uint32_t link = get32(b + 12);
        if (connByLink(link)) return;
        g_brPending.on = true;
        g_brPending.link = link;
        memcpy(g_brPending.addr, b + 6, 6);
        memcpy(g_brPending.cod, b + 16, 3);
        size_t nl = bl > 19 ? (std::min)((size_t)b[19], bl - 20) : 0;
        g_brPending.name.assign((const char*)b + 20, nl);
        std::vector<uint8_t> p(b + 6, b + 12);        // Connection Request: addr, class of device, ACL
        p.insert(p.end(), b + 16, b + 19);
        p.push_back(1);
        event(0x04, p);
        g_env.log("Bluetooth Classic: connection request from %s", g_brPending.name.empty() ? "a device" : g_brPending.name.c_str());
        return;
    }
    if (bl < 4) return;
    uint32_t link = get32(b);
    if (type == MSG_CONN_ACK)
    {
        if (!g_init.on || !g_init.sent || link != g_init.link || bl < 11) return;
        g_init.on = false;
        Conn c = {};
        c.handle = newHandle();
        c.link = link;
        c.role = 0;
        c.peerType = b[4] & 1;
        memcpy(c.peer, b + 5, 6);
        c.interval = g_init.interval; c.latency = g_init.latency; c.timeout = g_init.timeout;
        g_conns.push_back(c);
        connComplete(c);
        return;
    }
    Conn* c = connByLink(link);
    if (!c) return;
    switch (type)
    {
    case MSG_ACL:
    {
        if (bl < 5) return;
        size_t n = bl - 5;
        uint8_t pb = b[4] == 1 ? 1 : 2;               // continuing / first fragment (flushable, to the host)
        std::vector<uint8_t> p = { 0x02, (uint8_t)c->handle, (uint8_t)((c->handle >> 8) | (pb << 4)), (uint8_t)n, (uint8_t)(n >> 8) };
        p.insert(p.end(), b + 5, b + bl);
        toHost(p);
        break;
    }
    case MSG_DISC:
        connGone(c->handle, bl >= 5 ? b[4] : 0x13);
        break;
    case MSG_CONN_UPD:
        if (bl < 10) return;
        c->interval = get16(b + 4); c->latency = get16(b + 6); c->timeout = get16(b + 8);
        connUpdateComplete(*c);
        break;
    case MSG_DLE:
        if (bl < 8) return;
        dataLengthChange(c->handle, 251, 2120, get16(b + 4), get16(b + 6));
        break;
    case MSG_ENC_REQ:                                 // slave: LE Long Term Key Request to our host
    {
        if (bl < 14) return;
        std::vector<uint8_t> p = { 0x05 };
        add16(p, c->handle);
        p.insert(p.end(), b + 4, b + 14);
        event(0x3E, p);
        break;
    }
    case MSG_ENC_RSP:                                 // master: the slave's key
    {
        if (!c->encPending || bl < 5) return;
        c->encPending = false;
        uint16_t h = c->handle;
        if (!b[4])                                    // negative reply: the slave has no key
        {
            event(0x08, { 0x06, (uint8_t)h, (uint8_t)(h >> 8), 0 });   // PIN or Key Missing
            g_env.log("Bluetooth: encryption refused by the peer (no key)");
            return;
        }
        if (bl < 21 || memcmp(b + 5, c->ltk, 16) != 0)   // different keys: a real link fails the MIC check
        {
            std::vector<uint8_t> d;
            add32(d, c->link);
            d.push_back(0x3D);
            linkSend(MSG_DISC, d);
            g_env.log("Bluetooth: encryption keys differ - disconnected (MIC failure)");
            connGone(h, 0x3D);
            return;
        }
        if (c->encrypted) event(0x30, { 0, (uint8_t)h, (uint8_t)(h >> 8) });   // Encryption Key Refresh Complete
        else event(0x08, { 0, (uint8_t)h, (uint8_t)(h >> 8), 1 });            // Encryption Change: on
        c->encrypted = true;
        std::vector<uint8_t> d;
        add32(d, c->link);
        linkSend(MSG_ENC_DONE, d);
        g_env.log("Bluetooth: link encrypted (handle %u)", h);
        break;
    }
    case MSG_ENC_DONE:                                // slave: the keys matched
    {
        uint16_t h = c->handle;
        if (c->encrypted) event(0x30, { 0, (uint8_t)h, (uint8_t)(h >> 8) });
        else event(0x08, { 0, (uint8_t)h, (uint8_t)(h >> 8), 1 });
        c->encrypted = true;
        g_env.log("Bluetooth: link encrypted (handle %u)", h);
        break;
    }
    default:
        break;
    }
}

// ACL from the host: to the peer, then Number Of Completed Packets (the host's flow control)
static void aclFromHost(const uint8_t* p, size_t len)
{
    if (len < 4) return;
    uint16_t hf = get16(p), n = get16(p + 2);
    uint16_t h = hf & 0x0FFF;
    if (len < 4u + n) return;
    Conn* c = connByHandle(h);
    if (c && c->local) pcOnAcl(c->link, (uint8_t)((hf >> 12) & 3), p + 4, n);
    else if (c)
    {
        std::vector<uint8_t> b;
        add32(b, c->link);
        b.push_back((uint8_t)((hf >> 12) & 3));
        b.insert(b.end(), p + 4, p + 4 + n);
        linkSend(MSG_ACL, b);
    }
    event(0x13, { 1, (uint8_t)h, (uint8_t)(h >> 8), 1, 0 });
}

// chip reset: the peers lose the connections (supervision timeout)
static void connReset()
{
    for (const Conn& c : g_conns)
    {
        if (c.local) { pcOnDisconnected(c.link, 0x08); continue; }
        std::vector<uint8_t> b;
        add32(b, c.link);
        b.push_back(0x08);                            // Connection Timeout
        linkSend(MSG_DISC, b);
    }
    g_conns.clear();
    g_init = Initiating();
    g_white.clear();
}
// ---------------------------------------------------------------------------
//  PC bridge (bt_pc.cpp, BLEPC = YES): a "local" connection whose peer is the bridge in this
//  engine. The bridge posts requests from its own threads; they are done here, every slice,
//  under the QEMU lock. The ESP32 is the slave (it advertises, the bridge connects).
// ---------------------------------------------------------------------------
struct PcReq { int type; uint32_t link; std::vector<uint8_t> data; };
static std::mutex g_pcLock;
static std::deque<PcReq> g_pcReq;

void btPcPost(int type, uint32_t link, std::vector<uint8_t> data)
{
    std::lock_guard<std::mutex> l(g_pcLock);
    g_pcReq.push_back({ type, link, std::move(data) });
}

// a real device's advertising (PC watcher): report it while scanning; a pending LE Create
// Connection to it -> the bridge connects with Windows (PC_REAL_UP / PC_REAL_FAIL come back)
static void realAdv(const std::vector<uint8_t>& d)
{
    if (d.size() < 10 || g_status != 2) return;
    uint8_t at = d[0], ev = d[7], len = d[9];
    if (d.size() < 10u + len) return;
    if (g_scanOn && (ev != 4 || g_scanActive)) advReport(ev, at, &d[1], &d[10], len, (int8_t)d[8]);
    if (!g_init.on || g_init.sent || (ev != 0 && ev != 1)) return;      // connectable advertising only
    std::vector<uint8_t> k = { (uint8_t)(at & 1) };
    k.insert(k.end(), &d[1], &d[7]);
    bool match = g_init.policy ? std::find(g_white.begin(), g_white.end(), k) != g_white.end()
                               : (at & 1) == (g_init.peerType & 1) && memcmp(&d[1], g_init.peer, 6) == 0;
    if (!match) return;
    g_init.peerType = at & 1;
    memcpy(g_init.peer, &d[1], 6);
    g_init.sent = true;
    pcConnectReal(g_init.link, g_init.peerType, g_init.peer);
}

static void pcSlice()
{
    std::deque<PcReq> q;
    {
        std::lock_guard<std::mutex> l(g_pcLock);
        q.swap(g_pcReq);
    }
    for (PcReq& r : q)
    {
        Conn* c = connByLink(r.link);
        switch (r.type)
        {
        case PC_CONNECT:                              // phone mirror: the bridge connects to the advertising ESP32
        {
            if (c) { pcOnConnected(r.link); break; }
            bool connectable = g_adv.type == 0 || g_adv.type == 1 || g_adv.type == 4;
            if (g_status != 2 || !g_adv.on || !connectable || r.data.size() < 6) { pcOnNotReady(); break; }
            Conn n = {};
            n.handle = newHandle();
            n.link = r.link;
            n.role = 1;                               // the ESP32 is the slave
            n.local = true;
            n.peerType = 0;                           // the PC adapter's public address
            memcpy(n.peer, r.data.data(), 6);
            n.interval = 0x18; n.latency = 0; n.timeout = 400;   // 30 ms, 4 s
            g_adv.on = false;
            g_conns.push_back(n);
            connComplete(n);
            pcOnConnected(r.link);
            break;
        }
        case PC_ADV:
            realAdv(r.data);
            break;
        case PC_REAL_UP:                              // the ESP32 (master) is connected to a real device
        {
            if (!g_init.on || g_init.link != r.link) { pcOnDisconnected(r.link, 0x16); break; }   // cancelled meanwhile
            g_init.on = false;
            Conn n = {};
            n.handle = newHandle();
            n.link = r.link;
            n.role = 0;
            n.local = true;
            n.peerType = g_init.peerType;
            memcpy(n.peer, g_init.peer, 6);
            n.interval = g_init.interval; n.latency = g_init.latency; n.timeout = g_init.timeout;
            g_conns.push_back(n);
            connComplete(n);
            break;
        }
        case PC_REAL_FAIL:                            // try again at the device's next advertising
            if (g_init.on && g_init.link == r.link) g_init.sent = false;
            break;
        case PC_L2CAP:                                // to the host in ACL fragments of up to 251 bytes
        {
            if (!c) break;
            size_t off = 0;
            do
            {
                size_t n = std::min<size_t>(251, r.data.size() - off);
                uint8_t pb = off == 0 ? 2 : 1;
                std::vector<uint8_t> p = { 0x02, (uint8_t)c->handle, (uint8_t)((c->handle >> 8) | (pb << 4)), (uint8_t)n, (uint8_t)(n >> 8) };
                p.insert(p.end(), r.data.begin() + off, r.data.begin() + off + n);
                toHost(p);
                off += n;
            } while (off < r.data.size());
            break;
        }
        case PC_DISCONNECT:
            if (c) connGone(c->handle, r.data.empty() ? 0x13 : r.data[0]);
            break;
        default:
            break;
        }
    }
}