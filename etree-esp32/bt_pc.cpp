// ============================================================================
//  bt_pc.cpp - the PC's real Bluetooth adapter for the simulated ESP32 (part property BLEPC = YES)
//  (GPL, part of etree-esp32; C++/WinRT, needs C++20)
//
//  Phone -> simulated ESP32 (the ESP32 sketch is a GATT server):
//    1. the bridge connects to the ESP32 through the virtual controller ("local" connection,
//       bt.cpp: PC_CONNECT when the ESP32 advertises connectable; the ESP32 is the slave)
//    2. it is a GATT client of the ESP32 (ATT over L2CAP CID 4): MTU, services, characteristics,
//       CCCD descriptors
//    3. it mirrors every service on the PC adapter (GattServiceProvider, PC = BLE peripheral) and
//       advertises them; a phone's read / write goes to the ESP32 as ATT Read / Write, a phone
//       that subscribes makes the bridge write the ESP32's CCCD, and the ESP32's notifications /
//       indications go to the subscribed phones (NotifyValueAsync)
//    4. ESP32 disconnects (sketch reset ...): the providers stop, the bridge connects again
//  Windows chooses the advertising (the PC's name, the service UUIDs); apps find the ESP32's
//  services by UUID. GAP (0x1800) and GATT (0x1801) are Windows' own and not mirrored.
//  Not done: encryption / pairing on either side (the bridge answers SMP "pairing not supported").
//
//  Threads: the controller calls pcOn...() under the QEMU lock -> only queued here. A receive
//  thread handles the ESP32's L2CAP frames; the main bridge thread connects / discovers /
//  publishes; Windows calls the GATT handlers on its thread pool (they block on the ATT answer).
// ============================================================================
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Devices.Bluetooth.Advertisement.h>
#include <map>
#include <winrt/Windows.Devices.Radios.h>
#include <stdio.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "bt_internal.h"

#pragma comment(lib, "windowsapp")

using namespace winrt;
using namespace winrt::Windows::Devices::Bluetooth;
using namespace winrt::Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace winrt::Windows::Devices::Bluetooth::Advertisement;
using namespace winrt::Windows::Devices::Radios;
using namespace winrt::Windows::Storage::Streams;

static void (*g_log)(const char*, ...) = nullptr;

// ---------------------------------------------------------------------------
//  state shared with the controller (g_m)
// ---------------------------------------------------------------------------
static std::mutex              g_m;
static std::condition_variable g_cv;
static int  g_conn = 0;                               // 0 none, 1 PC_CONNECT sent, 2 connected
static int  g_gen = 0;                                // +1 at every disconnect
static bool g_notReady = false;
static std::vector<uint8_t> g_partial;                // L2CAP frame being put together
static std::deque<std::vector<uint8_t>> g_frames;     // complete L2CAP frames from the ESP32
static bool g_attHave = false;                        // an ATT response is waiting in g_attRsp
static std::vector<uint8_t> g_attRsp;
static uint32_t g_mirrorLink = 0;                     // link id of the phone-mirror connection

// real devices (ESP32 = client), see "real devices" below
static void realAcl(uint32_t link, uint8_t pb, const uint8_t* data, size_t n);
static void realGone(uint32_t link);

void pcOnConnected(uint32_t link) { if (link != g_mirrorLink) return; std::lock_guard<std::mutex> l(g_m); g_conn = 2; g_cv.notify_all(); }
void pcOnNotReady() { std::lock_guard<std::mutex> l(g_m); g_conn = 0; g_notReady = true; g_cv.notify_all(); }

void pcOnDisconnected(uint32_t link, uint8_t)
{
    if (link != g_mirrorLink) { realGone(link); return; }
    std::lock_guard<std::mutex> l(g_m);
    g_conn = 0;
    g_gen++;
    g_partial.clear();
    g_frames.clear();
    g_cv.notify_all();
}

void pcOnAcl(uint32_t link, uint8_t pb, const uint8_t* data, size_t n)
{
    if (link != g_mirrorLink) { realAcl(link, pb, data, n); return; }
    std::lock_guard<std::mutex> l(g_m);
    if (pb != 1) g_partial.assign(data, data + n);    // first fragment
    else g_partial.insert(g_partial.end(), data, data + n);
    if (g_partial.size() >= 4 && g_partial.size() >= 4u + (g_partial[0] | (g_partial[1] << 8)))
    {
        g_frames.push_back(std::move(g_partial));
        g_partial.clear();
        g_cv.notify_all();
    }
}

static void sendL2cap(uint16_t cid, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> f = { (uint8_t)payload.size(), (uint8_t)(payload.size() >> 8), (uint8_t)cid, (uint8_t)(cid >> 8) };
    f.insert(f.end(), payload.begin(), payload.end());
    btPcPost(PC_L2CAP, g_mirrorLink, std::move(f));
}

static void add16(std::vector<uint8_t>& v, uint16_t x) { v.push_back((uint8_t)x); v.push_back((uint8_t)(x >> 8)); }
static uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// ---------------------------------------------------------------------------
//  ATT client (one request at a time, like ATT itself)
// ---------------------------------------------------------------------------
static std::mutex g_attLock;
static uint16_t   g_mtu = 23;

// send 'pdu', wait for its response (or an Error Response); false = no connection / timeout
static bool attRequest(const std::vector<uint8_t>& pdu, std::vector<uint8_t>& rsp)
{
    std::lock_guard<std::mutex> al(g_attLock);
    int gen;
    {
        std::lock_guard<std::mutex> l(g_m);
        if (g_conn != 2) return false;
        g_attHave = false;
        gen = g_gen;
    }
    sendL2cap(4, pdu);
    std::unique_lock<std::mutex> l(g_m);
    g_cv.wait_for(l, std::chrono::seconds(30), [&] { return g_attHave || g_gen != gen; });
    if (!g_attHave) return false;
    rsp = std::move(g_attRsp);
    g_attHave = false;
    return true;
}

// read a value (Read, then Read Blob while it is full); returns the ATT error code, 0 = OK
static uint8_t attRead(uint16_t h, std::vector<uint8_t>& value)
{
    std::vector<uint8_t> rsp, req = { 0x0A };
    add16(req, h);
    if (!attRequest(req, rsp) || rsp.empty()) return 0x0E;            // Unlikely Error
    if (rsp[0] == 0x01) return rsp.size() >= 5 ? rsp[4] : 0x0E;
    value.assign(rsp.begin() + 1, rsp.end());
    while (value.size() >= (size_t)(g_mtu - 1) && value.size() < 512)
    {
        std::vector<uint8_t> blob = { 0x0C };
        add16(blob, h);
        add16(blob, (uint16_t)value.size());
        if (!attRequest(blob, rsp) || rsp.empty() || rsp[0] != 0x0D) break;
        if (rsp.size() == 1) break;
        value.insert(value.end(), rsp.begin() + 1, rsp.end());
        if (rsp.size() - 1 < (size_t)(g_mtu - 1)) break;
    }
    return 0;
}

// write (Write Request with response / Write Command without); returns the ATT error code
static uint8_t attWrite(uint16_t h, const std::vector<uint8_t>& value, bool withResponse)
{
    std::vector<uint8_t> req = { (uint8_t)(withResponse ? 0x12 : 0x52) };
    add16(req, h);
    req.insert(req.end(), value.begin(), value.end());
    if (!withResponse) { sendL2cap(4, req); return 0; }
    std::vector<uint8_t> rsp;
    if (!attRequest(req, rsp) || rsp.empty()) return 0x0E;
    if (rsp[0] == 0x01) return rsp.size() >= 5 ? rsp[4] : 0x0E;
    return 0;
}

// ---------------------------------------------------------------------------
//  the ESP32's GATT database, mirrored on the PC
// ---------------------------------------------------------------------------
struct PcChar
{
    uint16_t decl = 0, value = 0, cccd = 0;
    uint8_t  props = 0;
    guid     uuid{};
    GattLocalCharacteristic local{ nullptr };
    GattLocalCharacteristic::ReadRequested_revoker rd;
    GattLocalCharacteristic::WriteRequested_revoker wr;
    GattLocalCharacteristic::SubscribedClientsChanged_revoker sub;
};
struct PcService
{
    uint16_t start = 0, end = 0;
    guid     uuid{};
    std::vector<std::shared_ptr<PcChar>> chars;
    GattServiceProvider provider{ nullptr };
};
static std::mutex g_dbLock;
static std::vector<std::shared_ptr<PcService>> g_db;

// ATT UUID (2 or 16 bytes, little-endian) -> guid
static guid toGuid(const uint8_t* p, size_t n)
{
    uint8_t be[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB };
    if (n == 2) { be[2] = p[1]; be[3] = p[0]; }
    else if (n == 16) for (int i = 0; i < 16; i++) be[i] = p[15 - i];
    guid g;
    g.Data1 = ((uint32_t)be[0] << 24) | (be[1] << 16) | (be[2] << 8) | be[3];
    g.Data2 = (uint16_t)((be[4] << 8) | be[5]);
    g.Data3 = (uint16_t)((be[6] << 8) | be[7]);
    for (int i = 0; i < 8; i++) g.Data4[i] = be[8 + i];
    return g;
}

static bool isBase16(const guid& g, uint16_t u)
{
    return g == toGuid(std::vector<uint8_t>{ (uint8_t)u, (uint8_t)(u >> 8) }.data(), 2);
}

static std::string guidText(const guid& g)
{
    char b[40];
    sprintf_s(b, "%08X-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X", g.Data1, g.Data2, g.Data3, g.Data4[0], g.Data4[1],
              g.Data4[2], g.Data4[3], g.Data4[4], g.Data4[5], g.Data4[6], g.Data4[7]);
    return b;
}

static bool discover(std::vector<std::shared_ptr<PcService>>& db)
{
    std::vector<uint8_t> rsp;
    // MTU
    {
        std::vector<uint8_t> req = { 0x02 };
        add16(req, 517);
        if (attRequest(req, rsp) && rsp.size() >= 3 && rsp[0] == 0x03) g_mtu = std::min<uint16_t>(517, std::max<uint16_t>(23, get16(&rsp[1])));
    }
    // primary services: Read By Group Type 0x2800
    for (uint32_t start = 1; start <= 0xFFFF;)
    {
        std::vector<uint8_t> req = { 0x10 };
        add16(req, (uint16_t)start); add16(req, 0xFFFF); add16(req, 0x2800);
        if (!attRequest(req, rsp)) return false;
        if (rsp.size() < 2 || rsp[0] != 0x11) break;   // Attribute Not Found: done
        size_t len = rsp[1];
        if (len < 6) break;
        uint32_t last = start;
        for (size_t o = 2; o + len <= rsp.size(); o += len)
        {
            auto s = std::make_shared<PcService>();
            s->start = get16(&rsp[o]);
            s->end = get16(&rsp[o + 2]);
            s->uuid = toGuid(&rsp[o + 4], len - 4);
            db.push_back(s);
            last = s->end;
        }
        if (last >= 0xFFFF) break;
        start = last + 1;
    }
    // characteristics: Read By Type 0x2803 in every service
    for (auto& s : db)
    {
        for (uint32_t start = s->start; start <= s->end;)
        {
            std::vector<uint8_t> req = { 0x08 };
            add16(req, (uint16_t)start); add16(req, s->end); add16(req, 0x2803);
            if (!attRequest(req, rsp)) return false;
            if (rsp.size() < 2 || rsp[0] != 0x09) break;
            size_t len = rsp[1];
            if (len < 7) break;
            uint32_t last = start;
            for (size_t o = 2; o + len <= rsp.size(); o += len)
            {
                auto c = std::make_shared<PcChar>();
                c->decl = get16(&rsp[o]);
                c->props = rsp[o + 2];
                c->value = get16(&rsp[o + 3]);
                c->uuid = toGuid(&rsp[o + 5], len - 5);
                s->chars.push_back(c);
                last = c->decl;
            }
            start = last + 1;
        }
        // descriptors: the CCCD (0x2902) of notify / indicate characteristics
        for (size_t i = 0; i < s->chars.size(); i++)
        {
            auto& c = s->chars[i];
            if (!(c->props & 0x30)) continue;
            uint16_t endH = i + 1 < s->chars.size() ? (uint16_t)(s->chars[i + 1]->decl - 1) : s->end;
            for (uint32_t start = c->value + 1; start <= endH && !c->cccd;)
            {
                std::vector<uint8_t> req = { 0x04 };
                add16(req, (uint16_t)start); add16(req, endH);
                if (!attRequest(req, rsp)) return false;
                if (rsp.size() < 2 || rsp[0] != 0x05) break;
                size_t len = rsp[1] == 1 ? 4 : 18;
                uint32_t last = start;
                for (size_t o = 2; o + len <= rsp.size(); o += len)
                {
                    uint16_t h = get16(&rsp[o]);
                    if (len == 4 && get16(&rsp[o + 2]) == 0x2902) c->cccd = h;
                    last = h;
                }
                start = last + 1;
            }
        }
    }
    return true;
}

static IBuffer toBuffer(const std::vector<uint8_t>& v)
{
    DataWriter w;
    w.WriteBytes(array_view<uint8_t const>(v.data(), v.data() + v.size()));
    return w.DetachBuffer();
}

static std::vector<uint8_t> fromBuffer(const IBuffer& b)
{
    std::vector<uint8_t> v(b ? b.Length() : 0);
    if (!v.empty()) DataReader::FromBuffer(b).ReadBytes(v);
    return v;
}

static std::atomic<int> g_logCount{ 0 };
static void logSome(const char* what, const PcChar& c, size_t n)
{
    if (g_logCount++ < 40) g_log("BLEPC: phone %s %s (%u bytes)", what, guidText(c.uuid).c_str(), (unsigned)n);
}

// mirror the services on the PC adapter and advertise them
static void publish(std::vector<std::shared_ptr<PcService>>& db)
{
    for (auto& s : db)
    {
        if (isBase16(s->uuid, 0x1800) || isBase16(s->uuid, 0x1801)) continue;   // Windows has its own GAP / GATT
        auto r = GattServiceProvider::CreateAsync(s->uuid).get();
        if (r.Error() != BluetoothError::Success)
        {
            g_log("BLEPC: service %s not created (Windows error %d)", guidText(s->uuid).c_str(), (int)r.Error());
            continue;
        }
        s->provider = r.ServiceProvider();
        for (auto& c : s->chars)
        {
            GattLocalCharacteristicParameters p;
            p.CharacteristicProperties((GattCharacteristicProperties)(c->props & 0x3E));
            p.ReadProtectionLevel(GattProtectionLevel::Plain);
            p.WriteProtectionLevel(GattProtectionLevel::Plain);
            auto cr = s->provider.Service().CreateCharacteristicAsync(c->uuid, p).get();
            if (cr.Error() != BluetoothError::Success)
            {
                g_log("BLEPC: characteristic %s not created (Windows error %d)", guidText(c->uuid).c_str(), (int)cr.Error());
                continue;
            }
            c->local = cr.Characteristic();
            std::shared_ptr<PcChar> cc = c;
            if (c->props & 0x02)
                c->rd = c->local.ReadRequested(auto_revoke, [cc](GattLocalCharacteristic const&, GattReadRequestedEventArgs const& a) {
                    auto def = a.GetDeferral();
                    try
                    {
                        auto req = a.GetRequestAsync().get();
                        if (req)
                        {
                            std::vector<uint8_t> v;
                            uint8_t err = attRead(cc->value, v);
                            if (err) req.RespondWithProtocolError(err);
                            else { req.RespondWithValue(toBuffer(v)); logSome("read", *cc, v.size()); }
                        }
                    }
                    catch (...) {}
                    def.Complete();
                });
            if (c->props & 0x0C)
                c->wr = c->local.WriteRequested(auto_revoke, [cc](GattLocalCharacteristic const&, GattWriteRequestedEventArgs const& a) {
                    auto def = a.GetDeferral();
                    try
                    {
                        auto req = a.GetRequestAsync().get();
                        if (req)
                        {
                            std::vector<uint8_t> v = fromBuffer(req.Value());
                            bool withRsp = req.Option() == GattWriteOption::WriteWithResponse;
                            uint8_t err = attWrite(cc->value, v, withRsp);
                            if (withRsp) { if (err) req.RespondWithProtocolError(err); else req.Respond(); }
                            logSome("wrote", *cc, v.size());
                        }
                    }
                    catch (...) {}
                    def.Complete();
                });
            if ((c->props & 0x30) && c->cccd)
                c->sub = c->local.SubscribedClientsChanged(auto_revoke, [cc](GattLocalCharacteristic const& l, auto const&) {
                    try
                    {
                        uint16_t want = l.SubscribedClients().Size() ? (cc->props & 0x10 ? 1 : 2) : 0;
                        std::vector<uint8_t> v;
                        add16(v, want);
                        attWrite(cc->cccd, v, true);
                        g_log("BLEPC: phone %s %s", want ? "subscribed to" : "unsubscribed from", guidText(cc->uuid).c_str());
                    }
                    catch (...) {}
                });
        }
        s->provider.AdvertisementStatusChanged([u = s->uuid](GattServiceProvider const&, GattServiceProviderAdvertisementStatusChangedEventArgs const& e) {
            static const char* NAMES[] = { "created", "stopped", "started", "aborted", "started without all service data" };
            int st = (int)e.Status();
            g_log("BLEPC: advertising of %s: %s (error %d)", guidText(u).c_str(), st >= 0 && st < 5 ? NAMES[st] : "?", (int)e.Error());
        });
        GattServiceProviderAdvertisingParameters ap;
        ap.IsConnectable(true);
        ap.IsDiscoverable(true);
        s->provider.StartAdvertising(ap);
        g_log("BLEPC: service %s on the PC adapter (%u characteristics)", guidText(s->uuid).c_str(), (unsigned)s->chars.size());
    }
}

static void unpublish(std::vector<std::shared_ptr<PcService>>& db)
{
    for (auto& s : db)
    {
        for (auto& c : s->chars) { c->rd.revoke(); c->wr.revoke(); c->sub.revoke(); }
        if (s->provider)
        {
            try { s->provider.StopAdvertising(); } catch (...) {}
            s->provider = nullptr;
        }
    }
    db.clear();
}

// ESP32 notification / indication -> the subscribed phones
static void toPhones(uint16_t h, const uint8_t* v, size_t n)
{
    std::shared_ptr<PcChar> c;
    {
        std::lock_guard<std::mutex> l(g_dbLock);
        for (auto& s : g_db)
            for (auto& x : s->chars)
                if (x->value == h) c = x;
    }
    if (!c || !c->local) return;
    try { c->local.NotifyValueAsync(toBuffer(std::vector<uint8_t>(v, v + n))); }
    catch (...) {}
}

// ---------------------------------------------------------------------------
//  receive thread: L2CAP frames from the ESP32
// ---------------------------------------------------------------------------
static void rxThread()
{
    init_apartment(apartment_type::multi_threaded);
    for (;;)
    {
        std::vector<uint8_t> f;
        {
            std::unique_lock<std::mutex> l(g_m);
            g_cv.wait(l, [] { return !g_frames.empty(); });
            f = std::move(g_frames.front());
            g_frames.pop_front();
        }
        if (f.size() < 5) continue;
        uint16_t cid = get16(&f[2]);
        const uint8_t* p = &f[4];
        size_t n = f.size() - 4;
        if (cid == 4)                                 // ATT
        {
            uint8_t op = p[0];
            if (op == 0x1B && n >= 3) toPhones(get16(p + 1), p + 3, n - 3);              // notification
            else if (op == 0x1D && n >= 3) { sendL2cap(4, { 0x1E }); toPhones(get16(p + 1), p + 3, n - 3); }   // indication
            else if (op == 0x02) { std::vector<uint8_t> r = { 0x03 }; add16(r, 517); sendL2cap(4, r); }  // MTU from the ESP32
            else if (op & 1)                          // a response: to the waiting request
            {
                std::lock_guard<std::mutex> l(g_m);
                g_attRsp.assign(p, p + n);
                g_attHave = true;
                g_cv.notify_all();
            }
            else if (!(op & 0x40))                    // a request to us (we have no database): not supported
            {
                std::vector<uint8_t> r = { 0x01, op };
                add16(r, n >= 3 ? get16(p + 1) : 0);
                r.push_back(0x06);
                sendL2cap(4, r);
            }
        }
        else if (cid == 5 && n >= 4)                  // L2CAP signaling
        {
            if (p[0] == 0x12) sendL2cap(5, { 0x13, p[1], 2, 0, 0, 0 });   // Connection Parameter Update: accepted (others: ignored)
        }
        else if (cid == 6 && n >= 1)                  // SMP: pairing is not done by the bridge
        {
            if (p[0] == 0x01 || p[0] == 0x0B) sendL2cap(6, { 0x05, 0x05 });   // Pairing Failed: Pairing Not Supported
        }
    }
}

// ---------------------------------------------------------------------------
//  main bridge thread
// ---------------------------------------------------------------------------
static void startWatcher();

static void bridgeThread()
{
    init_apartment(apartment_type::multi_threaded);
    try
    {
        BluetoothAdapter ad = BluetoothAdapter::GetDefaultAsync().get();
        if (!ad) { g_log("BLEPC: no Bluetooth adapter in this PC - real Bluetooth not available"); return; }
        if (!ad.IsLowEnergySupported()) { g_log("BLEPC: the Bluetooth adapter has no Low Energy - not available"); return; }
        bool peripheral = ad.IsPeripheralRoleSupported();
        Radio radio = ad.GetRadioAsync().get();
        bool said = false;
        while (radio && radio.State() != RadioState::On)
        {
            if (!said) { g_log("BLEPC: Bluetooth is switched off in Windows - switch it on (waiting)"); said = true; }
            Sleep(2000);
        }
        uint64_t a = ad.BluetoothAddress();
        std::vector<uint8_t> pcAddr(6);
        for (int i = 0; i < 6; i++) pcAddr[i] = (uint8_t)(a >> (8 * i));
        g_log("BLEPC: PC Bluetooth adapter %02X:%02X:%02X:%02X:%02X:%02X ready", pcAddr[5], pcAddr[4], pcAddr[3], pcAddr[2], pcAddr[1], pcAddr[0]);
        startWatcher();                               // real devices for a scanning ESP32
        if (!peripheral)
        {
            g_log("BLEPC: the Bluetooth adapter cannot be a BLE peripheral (Windows) - a phone cannot connect to the ESP32 (the ESP32 can still connect to real devices)");
            return;
        }
        std::thread(rxThread).detach();

        for (;;)
        {
            // connect to the ESP32 (when its sketch advertises)
            int gen;
            {
                std::unique_lock<std::mutex> l(g_m);
                g_conn = 1;
                g_notReady = false;
                btPcPost(PC_CONNECT, g_mirrorLink, pcAddr);
                g_cv.wait_for(l, std::chrono::seconds(5), [] { return g_conn == 2 || g_notReady; });
                if (g_conn != 2) { l.unlock(); Sleep(500); continue; }
                gen = g_gen;
            }
            g_log("BLEPC: connected to the simulated ESP32, reading its services");
            std::vector<std::shared_ptr<PcService>> db;
            if (!discover(db))
            {
                g_log("BLEPC: reading the services failed - trying again");
                btPcPost(PC_DISCONNECT, g_mirrorLink, { 0x13 });
                Sleep(1000);
                continue;
            }
            publish(db);
            {
                std::lock_guard<std::mutex> l(g_dbLock);
                g_db = db;
            }
            g_log("BLEPC: ready - a phone can connect to this PC (name: the PC's Bluetooth name)");
            {
                std::unique_lock<std::mutex> l(g_m);
                g_cv.wait(l, [&] { return g_gen != gen; });
            }
            {
                std::lock_guard<std::mutex> l(g_dbLock);
                g_db.clear();
            }
            unpublish(db);
            g_log("BLEPC: the ESP32 disconnected - services removed, waiting for it to advertise again");
        }
    }
    catch (hresult_error const& e)
    {
        g_log("BLEPC: Windows Bluetooth error 0x%08X: %ls", (unsigned)e.code(), e.message().c_str());
    }
}

void btPcStart(void (*log)(const char*, ...))
{
    g_log = log;
    g_mirrorLink = 0xC0DE0000u | (GetTickCount() & 0xFFFF);   // only has to differ from the controller's random ids
    std::thread(bridgeThread).detach();
}

// ===========================================================================
//  real devices (the ESP32 sketch is a GATT client, the device near the PC is the server)
//  Scanning: a Windows advertisement watcher; every advertisement goes to the controller
//  (PC_ADV), which makes LE Advertising Reports while the ESP32 scans. Connecting: the
//  controller calls pcConnectReal() for a pending LE Create Connection; a thread connects with
//  Windows (BluetoothLEDevice + GattSession that keeps the link), reads the device's GATT
//  database and gives it our own handle numbers (service, declaration, value, descriptors;
//  a CCCD is added when Windows does not list one), then answers the ESP32's ATT requests
//  from it: MTU, Find Information, Find By Type Value, Read By Type, Read By Group Type, Read,
//  Read Blob, Write Request / Command; a CCCD write subscribes with Windows, and the device's
//  value changes go to the ESP32 as notifications. GAP / GATT services: Windows' own, left out.
//  Not done: pairing (SMP -> "pairing not supported"), prepared writes.
// ===========================================================================
static BluetoothLEAdvertisementWatcher g_watcher{ nullptr };

static void startWatcher()
{
    g_watcher = BluetoothLEAdvertisementWatcher();
    g_watcher.ScanningMode(BluetoothLEScanningMode::Active);
    g_watcher.Received([](BluetoothLEAdvertisementWatcher const&, BluetoothLEAdvertisementReceivedEventArgs const& a) {
        try
        {
            uint64_t addr = a.BluetoothAddress();
            std::vector<uint8_t> d = { (uint8_t)(a.BluetoothAddressType() == BluetoothAddressType::Random ? 1 : 0) };
            for (int i = 0; i < 6; i++) d.push_back((uint8_t)(addr >> (8 * i)));
            int ev = (int)a.AdvertisementType();      // 0..4 like HCI; extended advertising -> 0
            d.push_back((uint8_t)(ev <= 4 ? ev : 0));
            d.push_back((uint8_t)(int8_t)std::max(-127, (int)a.RawSignalStrengthInDBm()));
            std::vector<uint8_t> ad;                  // the AD structures again (length, type, data)
            for (auto const& s : a.Advertisement().DataSections())
            {
                std::vector<uint8_t> v = fromBuffer(s.Data());
                if (ad.size() + 2 + v.size() > 31) break;
                ad.push_back((uint8_t)(v.size() + 1));
                ad.push_back(s.DataType());
                ad.insert(ad.end(), v.begin(), v.end());
            }
            d.push_back((uint8_t)ad.size());
            d.insert(d.end(), ad.begin(), ad.end());
            btPcPost(PC_ADV, 0, std::move(d));
        }
        catch (...) {}
    });
    g_watcher.Start();
    g_log("BLEPC: watching real BLE devices near the PC (a scanning ESP32 sees them)");
}

struct RAttr
{
    uint16_t h = 0, end = 0;                          // end: last handle of a service
    uint8_t  kind = 0;                                // 0 service, 1 characteristic declaration, 2 value, 3 descriptor, 4 CCCD
    std::vector<uint8_t> type, value;                 // type: ATT UUID (2 / 16 bytes); value: services / declarations
    GattCharacteristic ch{ nullptr };
    GattDescriptor desc{ nullptr };
    uint16_t valueHandle = 0;                         // CCCD: its characteristic's value handle
    bool subscribed = false;                          // CCCD: ValueChanged registered
};

struct RealConn
{
    uint32_t link = 0;
    BluetoothLEDevice dev{ nullptr };
    GattSession session{ nullptr };
    std::vector<RAttr> attrs;
    std::vector<GattCharacteristic::ValueChanged_revoker> vc;
    BluetoothLEDevice::ConnectionStatusChanged_revoker cs;
    std::atomic<uint16_t> mtu{ 23 };
    std::mutex m;
    std::condition_variable cv;
    std::deque<std::vector<uint8_t>> rx;
    std::vector<uint8_t> partial;
    bool closing = false, remoteGone = false;
};
static std::mutex g_realLock;
static std::map<uint32_t, std::shared_ptr<RealConn>> g_real;

static std::shared_ptr<RealConn> realFind(uint32_t link)
{
    std::lock_guard<std::mutex> l(g_realLock);
    auto it = g_real.find(link);
    return it == g_real.end() ? nullptr : it->second;
}

static void realAcl(uint32_t link, uint8_t pb, const uint8_t* data, size_t n)
{
    auto rc = realFind(link);
    if (!rc) return;
    std::lock_guard<std::mutex> l(rc->m);
    if (pb != 1) rc->partial.assign(data, data + n);
    else rc->partial.insert(rc->partial.end(), data, data + n);
    if (rc->partial.size() >= 4 && rc->partial.size() >= 4u + (rc->partial[0] | (rc->partial[1] << 8)))
    {
        rc->rx.push_back(std::move(rc->partial));
        rc->partial.clear();
        rc->cv.notify_all();
    }
}

static void realGone(uint32_t link)                   // the ESP32's host disconnected (or reset)
{
    auto rc = realFind(link);
    if (!rc) return;
    std::lock_guard<std::mutex> l(rc->m);
    rc->closing = true;
    rc->cv.notify_all();
}

static void realSend(uint32_t link, uint16_t cid, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> f = { (uint8_t)payload.size(), (uint8_t)(payload.size() >> 8), (uint8_t)cid, (uint8_t)(cid >> 8) };
    f.insert(f.end(), payload.begin(), payload.end());
    btPcPost(PC_L2CAP, link, std::move(f));
}

// guid -> ATT UUID: 2 bytes for Bluetooth base UUIDs, else 16 bytes (both little-endian)
static std::vector<uint8_t> attUuid(const guid& g)
{
    uint8_t be[16] = { (uint8_t)(g.Data1 >> 24), (uint8_t)(g.Data1 >> 16), (uint8_t)(g.Data1 >> 8), (uint8_t)g.Data1,
                       (uint8_t)(g.Data2 >> 8), (uint8_t)g.Data2, (uint8_t)(g.Data3 >> 8), (uint8_t)g.Data3 };
    for (int i = 0; i < 8; i++) be[8 + i] = g.Data4[i];
    static const uint8_t BASE[12] = { 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB };
    if (be[0] == 0 && be[1] == 0 && memcmp(be + 4, BASE, 12) == 0) return { be[3], be[2] };
    std::vector<uint8_t> le(16);
    for (int i = 0; i < 16; i++) le[i] = be[15 - i];
    return le;
}

static bool buildDb(RealConn& rc)
{
    auto sr = rc.dev.GetGattServicesAsync(BluetoothCacheMode::Uncached).get();
    if (sr.Status() != GattCommunicationStatus::Success) return false;
    uint16_t h = 1;
    for (auto const& svc : sr.Services())
    {
        guid u = svc.Uuid();
        if (isBase16(u, 0x1800) || isBase16(u, 0x1801)) continue;
        size_t si = rc.attrs.size();
        RAttr s;
        s.h = h++; s.kind = 0; s.type = { 0x00, 0x28 }; s.value = attUuid(u);
        rc.attrs.push_back(s);
        auto cr = svc.GetCharacteristicsAsync(BluetoothCacheMode::Uncached).get();
        if (cr.Status() == GattCommunicationStatus::Success)
            for (auto const& ch : cr.Characteristics())
            {
                uint8_t props = (uint8_t)((uint32_t)ch.CharacteristicProperties() & 0xFF);
                RAttr d;
                d.h = h++; d.kind = 1; d.type = { 0x03, 0x28 };
                uint16_t vh = h;
                d.value = { props, (uint8_t)vh, (uint8_t)(vh >> 8) };
                std::vector<uint8_t> cu = attUuid(ch.Uuid());
                d.value.insert(d.value.end(), cu.begin(), cu.end());
                rc.attrs.push_back(d);
                RAttr v;
                v.h = h++; v.kind = 2; v.type = cu; v.ch = ch;
                rc.attrs.push_back(v);
                bool hasCccd = false;
                auto dr = ch.GetDescriptorsAsync(BluetoothCacheMode::Uncached).get();
                if (dr.Status() == GattCommunicationStatus::Success)
                    for (auto const& ds : dr.Descriptors())
                    {
                        RAttr x;
                        x.h = h++; x.type = attUuid(ds.Uuid());
                        if (isBase16(ds.Uuid(), 0x2902)) { x.kind = 4; x.ch = ch; x.valueHandle = vh; hasCccd = true; }
                        else { x.kind = 3; x.desc = ds; }
                        rc.attrs.push_back(x);
                    }
                if ((props & 0x30) && !hasCccd)
                {
                    RAttr x;
                    x.h = h++; x.kind = 4; x.type = { 0x02, 0x29 }; x.ch = ch; x.valueHandle = vh;
                    rc.attrs.push_back(x);
                }
            }
        rc.attrs[si].end = (uint16_t)(h - 1);
    }
    return true;
}

static uint8_t statusToAtt(GattCommunicationStatus st, winrt::Windows::Foundation::IReference<uint8_t> const& proto)
{
    if (st == GattCommunicationStatus::Success) return 0;
    if (st == GattCommunicationStatus::ProtocolError && proto) return proto.Value();
    if (st == GattCommunicationStatus::AccessDenied) return 0x05;   // Insufficient Authentication
    return 0x0E;                                                    // Unlikely Error (unreachable ...)
}

static uint8_t readAttr(RAttr& a, std::vector<uint8_t>& v)
{
    try
    {
        switch (a.kind)
        {
        case 0: case 1: v = a.value; return 0;
        case 2: { auto r = a.ch.ReadValueAsync(BluetoothCacheMode::Uncached).get(); uint8_t e = statusToAtt(r.Status(), r.ProtocolError()); if (!e) v = fromBuffer(r.Value()); return e; }
        case 3: { auto r = a.desc.ReadValueAsync(BluetoothCacheMode::Uncached).get(); uint8_t e = statusToAtt(r.Status(), r.ProtocolError()); if (!e) v = fromBuffer(r.Value()); return e; }
        case 4: v = { (uint8_t)(a.subscribed ? 1 : 0), 0 }; return 0;
        }
    }
    catch (...) {}
    return 0x0E;
}

static uint8_t writeAttr(const std::shared_ptr<RealConn>& rc, RAttr& a, const std::vector<uint8_t>& v, bool withRsp)
{
    try
    {
        switch (a.kind)
        {
        case 2:
        {
            auto r = a.ch.WriteValueWithResultAsync(toBuffer(v), withRsp ? GattWriteOption::WriteWithResponse : GattWriteOption::WriteWithoutResponse).get();
            return statusToAtt(r.Status(), r.ProtocolError());
        }
        case 3:
        {
            auto r = a.desc.WriteValueWithResultAsync(toBuffer(v)).get();
            return statusToAtt(r.Status(), r.ProtocolError());
        }
        case 4:                                       // CCCD: subscribe / unsubscribe with Windows
        {
            uint16_t cfg = v.size() >= 2 ? get16(v.data()) : (v.empty() ? 0 : v[0]);
            auto want = cfg & 1 ? GattClientCharacteristicConfigurationDescriptorValue::Notify
                      : cfg & 2 ? GattClientCharacteristicConfigurationDescriptorValue::Indicate
                                : GattClientCharacteristicConfigurationDescriptorValue::None;
            auto r = a.ch.WriteClientCharacteristicConfigurationDescriptorWithResultAsync(want).get();
            uint8_t e = statusToAtt(r.Status(), r.ProtocolError());
            if (e) return e;
            if (cfg && !a.subscribed)
            {
                std::weak_ptr<RealConn> w = rc;
                uint32_t link = rc->link;
                uint16_t vh = a.valueHandle;
                rc->vc.push_back(a.ch.ValueChanged(auto_revoke, [w, link, vh](GattCharacteristic const&, GattValueChangedEventArgs const& e) {
                    auto r = w.lock();
                    if (!r) return;
                    std::vector<uint8_t> val = fromBuffer(e.CharacteristicValue());
                    if (val.size() > (size_t)(r->mtu - 3)) val.resize(r->mtu - 3);
                    std::vector<uint8_t> p = { 0x1B };       // Handle Value Notification (also for indications)
                    add16(p, vh);
                    p.insert(p.end(), val.begin(), val.end());
                    realSend(link, 4, p);
                }));
            }
            a.subscribed = cfg != 0;
            return 0;
        }
        default:
            return 0x03;                              // Write Not Permitted
        }
    }
    catch (...) {}
    return 0x0E;
}

static void attError(uint32_t link, uint8_t op, uint16_t h, uint8_t code)
{
    std::vector<uint8_t> r = { 0x01, op };
    add16(r, h);
    r.push_back(code);
    realSend(link, 4, r);
}

// one ATT request from the ESP32 (the client)
static void attServe(const std::shared_ptr<RealConn>& rc, const uint8_t* p, size_t n)
{
    RealConn& c = *rc;
    uint8_t op = p[0];
    uint16_t mtu = c.mtu;
    auto inRange = [&](const RAttr& a, uint16_t s, uint16_t e) { return a.h >= s && a.h <= e; };
    auto find = [&](uint16_t h) -> RAttr* { for (auto& a : c.attrs) if (a.h == h) return &a; return nullptr; };
    switch (op)
    {
    case 0x02:                                        // Exchange MTU
    {
        uint16_t cm = n >= 3 ? get16(p + 1) : 23;
        c.mtu = std::max<uint16_t>(23, std::min<uint16_t>(cm, 517));
        std::vector<uint8_t> r = { 0x03 };
        add16(r, 517);
        realSend(c.link, 4, r);
        return;
    }
    case 0x04:                                        // Find Information
    {
        if (n < 5) break;
        uint16_t s = get16(p + 1), e = get16(p + 3);
        std::vector<uint8_t> r;
        size_t tl = 0;
        for (auto& a : c.attrs)
        {
            if (!inRange(a, s, e)) continue;
            if (!tl) { tl = a.type.size(); r = { 0x05, (uint8_t)(tl == 2 ? 1 : 2) }; }
            if (a.type.size() != tl || r.size() + 2 + tl > mtu) break;
            add16(r, a.h);
            r.insert(r.end(), a.type.begin(), a.type.end());
        }
        if (!tl) { attError(c.link, op, s, 0x0A); return; }
        realSend(c.link, 4, r);
        return;
    }
    case 0x06:                                        // Find By Type Value (primary service by UUID)
    {
        if (n < 7) break;
        uint16_t s = get16(p + 1), e = get16(p + 3), type = get16(p + 5);
        std::vector<uint8_t> val(p + 7, p + n), r = { 0x07 };
        if (type == 0x2800)
            for (auto& a : c.attrs)
                if (a.kind == 0 && inRange(a, s, e) && a.value == val && r.size() + 4 <= mtu) { add16(r, a.h); add16(r, a.end); }
        if (r.size() == 1) { attError(c.link, op, s, 0x0A); return; }
        realSend(c.link, 4, r);
        return;
    }
    case 0x08:                                        // Read By Type
    {
        if (n < 7) break;
        uint16_t s = get16(p + 1), e = get16(p + 3);
        std::vector<uint8_t> type(p + 5, p + n);
        std::vector<uint8_t> r;
        if (type == std::vector<uint8_t>{ 0x03, 0x28 })   // characteristic declarations
        {
            size_t len = 0;
            for (auto& a : c.attrs)
            {
                if (a.kind != 1 || !inRange(a, s, e)) continue;
                if (!len) { len = 2 + a.value.size(); r = { 0x09, (uint8_t)len }; }
                if (2 + a.value.size() != len || r.size() + len > mtu) break;
                add16(r, a.h);
                r.insert(r.end(), a.value.begin(), a.value.end());
            }
            if (!len) { attError(c.link, op, s, 0x0A); return; }
            realSend(c.link, 4, r);
            return;
        }
        for (auto& a : c.attrs)                       // read by characteristic UUID: the first match
            if (a.kind == 2 && inRange(a, s, e) && a.type == type)
            {
                std::vector<uint8_t> v;
                uint8_t err = readAttr(a, v);
                if (err) { attError(c.link, op, a.h, err); return; }
                if (v.size() > (size_t)(mtu - 4)) v.resize(mtu - 4);
                r = { 0x09, (uint8_t)(2 + v.size()) };
                add16(r, a.h);
                r.insert(r.end(), v.begin(), v.end());
                realSend(c.link, 4, r);
                return;
            }
        attError(c.link, op, s, 0x0A);
        return;
    }
    case 0x10:                                        // Read By Group Type (primary services)
    {
        if (n < 7) break;
        uint16_t s = get16(p + 1), e = get16(p + 3);
        std::vector<uint8_t> type(p + 5, p + n);
        if (type != std::vector<uint8_t>{ 0x00, 0x28 }) { attError(c.link, op, s, 0x10); return; }   // Unsupported Group Type
        std::vector<uint8_t> r;
        size_t len = 0;
        for (auto& a : c.attrs)
        {
            if (a.kind != 0 || !inRange(a, s, e)) continue;
            if (!len) { len = 4 + a.value.size(); r = { 0x11, (uint8_t)len }; }
            if (4 + a.value.size() != len || r.size() + len > mtu) break;
            add16(r, a.h); add16(r, a.end);
            r.insert(r.end(), a.value.begin(), a.value.end());
        }
        if (!len) { attError(c.link, op, s, 0x0A); return; }
        realSend(c.link, 4, r);
        return;
    }
    case 0x0A:                                        // Read
    case 0x0C:                                        // Read Blob
    {
        if (n < 3) break;
        uint16_t h = get16(p + 1), off = op == 0x0C && n >= 5 ? get16(p + 3) : 0;
        RAttr* a = find(h);
        if (!a) { attError(c.link, op, h, 0x01); return; }   // Invalid Handle
        std::vector<uint8_t> v;
        uint8_t err = readAttr(*a, v);
        if (err) { attError(c.link, op, h, err); return; }
        if (off > v.size()) { attError(c.link, op, h, 0x07); return; }   // Invalid Offset
        std::vector<uint8_t> r = { (uint8_t)(op + 1) };
        size_t take = std::min<size_t>(v.size() - off, mtu - 1);
        r.insert(r.end(), v.begin() + off, v.begin() + off + take);
        realSend(c.link, 4, r);
        return;
    }
    case 0x12:                                        // Write Request
    case 0x52:                                        // Write Command
    {
        if (n < 3) break;
        uint16_t h = get16(p + 1);
        RAttr* a = find(h);
        std::vector<uint8_t> v(p + 3, p + n);
        if (!a) { if (op == 0x12) attError(c.link, op, h, 0x01); return; }
        uint8_t err = writeAttr(rc, *a, v, op == 0x12);
        if (op == 0x12) { if (err) attError(c.link, op, h, err); else realSend(c.link, 4, { 0x13 }); }
        return;
    }
    case 0x1E:                                        // Handle Value Confirmation
        return;
    default:
        break;
    }
    if (!(op & 0x40)) attError(c.link, op, n >= 3 ? get16(p + 1) : 0, 0x06);   // Request Not Supported
}

static void realConnThread(std::shared_ptr<RealConn> rc, uint8_t addrType, uint64_t addr)
{
    init_apartment(apartment_type::multi_threaded);
    char at[24];
    sprintf_s(at, "%02X:%02X:%02X:%02X:%02X:%02X", (unsigned)(addr >> 40) & 0xFF, (unsigned)(addr >> 32) & 0xFF,
              (unsigned)(addr >> 24) & 0xFF, (unsigned)(addr >> 16) & 0xFF, (unsigned)(addr >> 8) & 0xFF, (unsigned)addr & 0xFF);
    bool ok = false;
    try
    {
        g_log("BLEPC: the ESP32 connects to the real device %s ...", at);
        rc->dev = BluetoothLEDevice::FromBluetoothAddressAsync(addr, addrType ? BluetoothAddressType::Random : BluetoothAddressType::Public).get();
        if (rc->dev)
        {
            rc->session = GattSession::FromDeviceIdAsync(rc->dev.BluetoothDeviceId()).get();
            if (rc->session) rc->session.MaintainConnection(true);
            ok = buildDb(*rc);
        }
    }
    catch (hresult_error const& e) { g_log("BLEPC: Windows error 0x%08X: %ls", (unsigned)e.code(), e.message().c_str()); }
    if (!ok)
    {
        g_log("BLEPC: could not connect to %s (out of range, or it does not accept connections) - trying again", at);
        { std::lock_guard<std::mutex> l(g_realLock); g_real.erase(rc->link); }
        try { if (rc->session) rc->session.Close(); if (rc->dev) rc->dev.Close(); } catch (...) {}
        Sleep(1000);
        btPcPost(PC_REAL_FAIL, rc->link, {});
        return;
    }
    std::weak_ptr<RealConn> w = rc;
    rc->cs = rc->dev.ConnectionStatusChanged(auto_revoke, [w](BluetoothLEDevice const& d, auto const&) {
        if (d.ConnectionStatus() != BluetoothConnectionStatus::Disconnected) return;
        if (auto r = w.lock())
        {
            std::lock_guard<std::mutex> l(r->m);
            r->closing = true;
            r->remoteGone = true;
            r->cv.notify_all();
        }
    });
    g_log("BLEPC: connected to %s (%u attributes)", at, (unsigned)rc->attrs.size());
    btPcPost(PC_REAL_UP, rc->link, {});
    for (;;)
    {
        std::vector<uint8_t> f;
        {
            std::unique_lock<std::mutex> l(rc->m);
            rc->cv.wait(l, [&] { return !rc->rx.empty() || rc->closing; });
            if (rc->closing) break;
            f = std::move(rc->rx.front());
            rc->rx.pop_front();
        }
        if (f.size() < 5) continue;
        uint16_t cid = get16(&f[2]);
        if (cid == 4) attServe(rc, &f[4], f.size() - 4);
        else if (cid == 5 && f[4] == 0x12 && f.size() >= 6) realSend(rc->link, 5, { 0x13, f[5], 2, 0, 0, 0 });
        else if (cid == 6 && (f[4] == 0x01 || f[4] == 0x0B)) realSend(rc->link, 6, { 0x05, 0x05 });   // Pairing Not Supported
    }
    bool remote = rc->remoteGone;
    { std::lock_guard<std::mutex> l(g_realLock); g_real.erase(rc->link); }
    rc->vc.clear();
    rc->cs.revoke();
    try { if (rc->session) rc->session.Close(); rc->dev.Close(); } catch (...) {}
    if (remote) btPcPost(PC_DISCONNECT, rc->link, { 0x08 });   // the device went away: connection timeout
    g_log("BLEPC: disconnected from %s%s", at, remote ? " (the device ended it)" : "");
}

void pcConnectReal(uint32_t link, uint8_t addrType, const uint8_t addr[6])
{
    auto rc = std::make_shared<RealConn>();
    rc->link = link;
    {
        std::lock_guard<std::mutex> l(g_realLock);
        g_real[link] = rc;
    }
    uint64_t a = 0;
    for (int i = 0; i < 6; i++) a |= (uint64_t)addr[i] << (8 * i);
    std::thread(realConnThread, rc, addrType, a).detach();
}