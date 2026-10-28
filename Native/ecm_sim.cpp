// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             ___________              __                 __________                                      __                         
//                             \__    ___/___   _______/  |_  ___________  \______   \_______   ____   ______ ____   _____/  |_                       
//                               |    |_/ __ \ /  ___/\   __\/ __ \_  __ \  |     ___/\_  __ \_/ __ \ /  ___// __ \ /    \   __\                      
//                               |    |\  ___/ \___ \  |  | \  ___/|  | \/  |    |     |  | \/\  ___/ \___ \\  ___/|   |  \  |                        
//                               |____| \___  >____  > |__|  \___  >__|     |____|     |__|    \___  >____  >\___  >___|  /__|                        
//                                          \/     \/            \/                                \/     \/     \/     \/                            
//                               _________                    .__       .__  .__          __                                                          
//                              /   _____/_____   ____   ____ |__|____  |  | |__| _______/  |_                                                        
//                              \_____  \\____ \_/ __ \_/ ___\|  \__  \ |  | |  |/  ___/\   __\                                                       
//                              /        \  |_> >  ___/\  \___|  |/ __ \|  |_|  |\___ \  |  |                                                         
//                             /_______  /   __/ \___  >\___  >__(____  /____/__/____  > |__|                                                         
//                                     \/|__|        \/     \/        \/             \/                                                               
//                                _____          __                         __  .__                                                                   
//                               /  _  \  __ ___/  |_  ____   _____   _____/  |_|__|__  __ ____                                                       
//                              /  /_\  \|  |  \   __\/  _ \ /     \ /  _ \   __\  \  \/ // __ \                                                      
//                             /    |    \  |  /|  | (  <_> )  Y Y  (  <_> )  | |  |\   /\  ___/                                                      
//                             \____|__  /____/ |__|  \____/|__|_|  /\____/|__| |__| \_/  \___  >                                                     
//                                     \/                         \/                          \/                                                      
//                               _________      .__          __  .__                                                                                  
//                              /   _____/ ____ |  |  __ ___/  |_|__| ____   ____   ______                                                            
//                              \_____  \ /  _ \|  | |  |  \   __\  |/  _ \ /    \ /  ___/                                                            
//                              /        (  <_> )  |_|  |  /|  | |  (  <_> )   |  \\___ \                                                              
//                             /_______  /\____/|____/____/ |__| |__|\____/|___|  /____  >                                                           
//                                     \/                                       \/     \/                                                             
//
//
//
//                              Tester Present Specialist Automotive Solutions Open Source License (TPSAS-OSL)
//                              Version 1.0, 2025
//
//                              Copyright (c) 2025 Tester Present Specialist Automotive Solutions
//
//
//             Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
//             documentation files (the "Software"), to use, modify, and distribute the Software subject to the following conditions:
//
//          1. Attribution.
//            - The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//            - Credit must be given to "Tester Present Specialist Automotive Solutions" in any documentation, public notices, or product materials 
//              that acknowledge the use of the Software.//
//
//          2. Open Source Use Only.
//            - The Software, including any modifications or derivative works (collectively, “Derivatives”), must be distributed only as open source software.
//            - Derivatives or distributions of the Software must be licensed under this license or another license approved by the Open Source Initiative 
//              (OSI) that maintains equivalent open source freedoms.
//            - The Software and any Derivatives may not be used as a component in any closed source or proprietary software product.
//
//          3. Redistribution.
//            - Redistributions of the Software, in source or binary form, must reproduce the above attribution, this license, and the disclaimer below 
//              in the documentation and/or other materials provided with the distribution.
//            - Any modifications made to the Software must be clearly marked as such, and the modified files must carry prominent notices stating that
//              changes have been made.
//
//          4. No Warranty.
//            - THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//              FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT.
//            - IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
//              TORT, OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//          5. Termination.
//            - This license and the rights granted hereunder will terminate automatically upon any breach of the above conditions.
//            - Upon termination, you must cease all use and distribution of the Software and any Derivatives.
//
//          6. Entire Agreement.
//            - This license constitutes the entire agreement between you and Tester Present Specialist Automotive Solutions regarding the Software 
//              and supersedes any prior understandings or agreements, whether written or oral, regarding the subject matter herein.
//
//              By using, modifying, or distributing the Software, you agree to abide by all the terms and conditions of this license.
//
//
//                         
/* =============================================================================================
 *  ecm_sim.cpp  —  the simulated ADEM ECM behaviour.
 * ============================================================================================= */
#include "ecm_sim.h"
#include "../tp.adem.core/src/cat_flash_j1939.h"   // single source of truth for the CAT-ext protocol
#include <windows.h>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>

namespace tpemu {

// Well-known PGNs (SAE J1939-71 / -81).
enum : uint32_t {
    PGN_REQUEST          = 0x00EA00,
    PGN_ADDRESS_CLAIMED  = 0x00EE00,
    PGN_PROPRIETARY_A    = 0x00EF00,   // CAT proprietary traffic (project note 02)
    PGN_TP_CM            = 0x00EC00,
    PGN_TP_DT            = 0x00EB00,
    PGN_EEC1             = 0x00F004,   // 61444 electronic engine controller 1 (RPM)
    PGN_EEC2             = 0x00F003,   // 61443 accelerator pedal / load
    PGN_ET1              = 0x00FEEE,   // 65262 engine temperature 1 (coolant)
    PGN_VIN              = 0x00FEEC,   // 65260 vehicle identification
    PGN_SOFTWARE_ID      = 0x00FEDA,   // 65242 software identification
    PGN_COMPONENT_ID     = 0x00FEEB,   // 65259 component identification
    PGN_ECUID            = 0x00FDC5,   // 64965 ECU identification (part/serial/location/type + CAT ECU-info)
    PGN_FLASH_DATA       = 0x00FF50,   // transported PGN carrying flash image blocks
};

static uint32_t crc32_update(uint32_t crc, const uint8_t* p, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int)(crc & 1)));
    }
    return crc;
}

static unsigned long long now_ms() { return GetTickCount64(); }

// --- Shared cross-process fault control -----------------------------------------------------
// The emulator runs in-process in EACH client (RP1210 model), so a fault armed in one process
// (the console, via emu_set_fault) would never reach a separate flash client. We publish the
// armed fault NAME to a shared file; every client reads it when a flash begins, so console fault
// injection reaches the flash tool across process boundaries.
static std::string sharedFaultPath()
{
    char tmp[MAX_PATH] = { 0 };
    DWORD n = GetTempPathA((DWORD)sizeof(tmp), tmp);
    std::string p = (n > 0 && n < sizeof(tmp)) ? std::string(tmp) : std::string(".\\");
    return p + "tprp32_emu_fault.txt";
}
static std::string readSharedFault()
{
    FILE* f = fopen(sharedFaultPath().c_str(), "rb");
    if (!f) return std::string();
    char buf[32] = { 0 };
    size_t r = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[r < sizeof(buf) ? r : sizeof(buf) - 1] = 0;
    std::string s(buf);
    while (!s.empty()) { char c = s.back(); if (c==10||c==13||c==9||c==32) s.pop_back(); else break; }
    return s;
}
static void writeSharedFault(const char* name)
{
    FILE* f = fopen(sharedFaultPath().c_str(), "wb");
    if (!f) return;
    const char* s = (name && *name) ? name : "none";
    fwrite(s, 1, strlen(s), f);
    fclose(f);
}

void EcmSim::start()
{
    std::lock_guard<std::mutex> lk(mu_);
    outq_.clear();
    running_ = true;
    startMs_ = now_ms();
    lastClaim_ = 0;

    // Reset flash-session state.
    inFlash_ = false; flashMode_ = 0;
    curOffset_ = 0; armLen_ = 0; storedChecksum_ = 0; burnUntilMs_ = 0; lastFlashMs_ = 0;
    flashBuf_.clear(); flashCrc_ = 0xFFFFFFFFu; swPartNumber_.clear();

    // Reset access-level state: a fresh connection starts at Normal with no presented pair.
    secLevel_ = catsec::LVL_NORMAL; memPointer_ = 0; memCommand_ = 0xFF;
    secPw1_.clear(); secPw2_.clear();
    seedConfigParams();


    // Optional fault injection (env TPRP_EMU_FAULT) so the tool's recovery/verify/precondition
    // paths can be exercised against a faithful ECM.
    fault_ = Fault::None; faultArmed_ = false;
    char fb[32] = { 0 };
    if (GetEnvironmentVariableA("TPRP_EMU_FAULT", fb, (DWORD)sizeof(fb)) > 0) {
        if      (_stricmp(fb, "inappropriate") == 0) fault_ = Fault::Inappropriate;
        else if (_stricmp(fb, "badfileinfo")   == 0) fault_ = Fault::BadFileInfo;
        else if (_stricmp(fb, "invalidoffset") == 0) { fault_ = Fault::InvalidOffset; faultArmed_ = true; }
        else if (_stricmp(fb, "badchecksum")   == 0) fault_ = Fault::BadChecksum;
        else if (_stricmp(fb, "erasefail")     == 0) fault_ = Fault::EraseFail;
        else if (_stricmp(fb, "stuck")         == 0) fault_ = Fault::Stuck;
    }

    // Construct the bus (engine periodics + peripheral controllers), seed the J1939-73 diagnostic
    // state (DTCs / readiness / tests), then announce every controller on connect.
    buildBus();
    seedDiagnostics();
    outq_.push_back(addressClaimed());
    for (auto& n : nodes_) outq_.push_back(n.addressClaimed());
}

void EcmSim::stop()
{
    std::lock_guard<std::mutex> lk(mu_);
    running_ = false;
    outq_.clear();
}

void EcmSim::enqueue(const J1939Msg& m) { outq_.push_back(m); }

// ---- outgoing periodic frames -------------------------------------------------------------

static std::vector<uint8_t> ascii(const std::string& s);   // fwd (defined below)

J1939Msg EcmSim::addressClaimed() const
{
    // 8-byte NAME (little-endian bit fields). Plausible engine-controller identity for the
    // simulated unit (function 0 = engine); not a real Caterpillar NAME.
    J1939Msg m; m.pgn = PGN_ADDRESS_CLAIMED; m.priority = 6; m.src = ECM_SA; m.dst = 0xFF;
    m.data = { 0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x14 };
    return m;
}

// Build the frames of a BAM (broadcast multi-packet) transfer sourced from `sa`.
static std::vector<J1939Msg> buildBam(uint8_t sa, uint32_t pgn, const std::vector<uint8_t>& payload)
{
    std::vector<J1939Msg> out;
    // J1939-21: one BAM carries at most 255 packets x 7 = 1785 bytes. Clamp rather than emit a
    // malformed header (the packet count would otherwise wrap in the uint8_t).
    size_t n = payload.size() > 1785 ? 1785 : payload.size();
    uint16_t total = (uint16_t)n;
    uint8_t packets = (uint8_t)((total + 6) / 7);
    J1939Msg cm; cm.pgn = 0x00EC00; cm.priority = 7; cm.src = sa; cm.dst = 0xFF;
    cm.data = { 0x20, (uint8_t)(total & 0xFF), (uint8_t)(total >> 8), packets, 0xFF,
                (uint8_t)(pgn & 0xFF), (uint8_t)((pgn >> 8) & 0xFF), (uint8_t)((pgn >> 16) & 0xFF) };
    out.push_back(cm);
    for (unsigned i = 0; i < packets; ++i) {
        J1939Msg dt; dt.pgn = 0x00EB00; dt.priority = 7; dt.src = sa; dt.dst = 0xFF;
        dt.data.push_back((uint8_t)(i + 1));
        for (int j = 0; j < 7; ++j) { size_t idx = (size_t)i * 7 + j; dt.data.push_back(idx < n ? payload[idx] : 0xFF); }
        out.push_back(dt);
    }
    return out;
}

// ---- BusNode: a generic spec'd J1939 controller -------------------------------------------

BusNode::BusNode(uint8_t sa, uint8_t function, uint16_t identity, std::string label)
    : sa_(sa), label_(std::move(label))
{
    // Well-formed J1939-81 NAME (little-endian bit fields); illustrative clean-room values, not a
    // real Caterpillar NAME. Fixed-address ECU (arbitrary-address-capable bit = 0).
    name_[0] = (uint8_t)(identity & 0xFF);
    name_[1] = (uint8_t)((identity >> 8) & 0xFF);
    name_[2] = 0x00; name_[3] = 0x00; name_[4] = 0x00;
    name_[5] = function;          // Function (illustrative J1939-71 value)
    name_[6] = 0x00; name_[7] = 0x00;
}

// Full J1939-81 NAME (mobile-plant EH controllers). 64-bit little-endian NAME bit-fields:
//   Identity 0:20 (21b) | Manufacturer 21:31 (11b) | ECU Instance 32:34 (3b) | Function Instance
//   35:39 (5b) | Function 40:47 (8b) | reserved 48 | Vehicle System 49:55 (7b) | Vehicle System
//   Instance 56:59 (4b) | Industry Group 60:62 (3b) | Arbitrary-Address-Capable 63 (1b).
BusNode::BusNode(uint8_t sa, std::string label, uint8_t industryGroup, uint8_t vehicleSystem,
                 uint8_t function, uint8_t functionInstance, uint8_t ecuInstance,
                 uint16_t mfgCode, uint32_t identity)
    : sa_(sa), label_(std::move(label))
{
    identity &= 0x1FFFFFu;   // 21-bit
    mfgCode  &= 0x07FFu;     // 11-bit
    name_[0] = (uint8_t)(identity & 0xFF);
    name_[1] = (uint8_t)((identity >> 8) & 0xFF);
    name_[2] = (uint8_t)(((identity >> 16) & 0x1F) | ((mfgCode & 0x07) << 5));   // id[16:20] | mfg[0:2]
    name_[3] = (uint8_t)((mfgCode >> 3) & 0xFF);                                 // mfg[3:10]
    name_[4] = (uint8_t)((ecuInstance & 0x07) | ((functionInstance & 0x1F) << 3));
    name_[5] = function;
    name_[6] = (uint8_t)((vehicleSystem & 0x7F) << 1);                           // bit 48 reserved = 0
    name_[7] = (uint8_t)((industryGroup & 0x07) << 4);                           // VSI = 0, AAC = 0 (fixed)
}

J1939Msg BusNode::addressClaimed() const
{
    J1939Msg m; m.pgn = 0x00EE00; m.priority = 6; m.src = sa_; m.dst = 0xFF;
    m.data.assign(name_, name_ + 8);
    return m;
}

void BusNode::addPeriodic(uint32_t pgn, uint8_t priority, unsigned long long periodMs, std::vector<uint8_t> data)
{
    Periodic p; p.pgn = pgn; p.priority = priority; p.periodMs = periodMs; p.data = std::move(data); p.last = 0;
    periodics_.push_back(std::move(p));
}

void BusNode::addRequestResponse(uint32_t pgn, std::vector<uint8_t> payload, bool bam, bool destSpecificOnly)
{
    responses_.push_back(Resp{ pgn, std::move(payload), bam, destSpecificOnly });
}

void BusNode::service(unsigned long long now, std::deque<J1939Msg>& out)
{
    for (auto& p : periodics_) {
        if (now - p.last >= p.periodMs) {
            p.last = now;
            J1939Msg m; m.pgn = p.pgn; m.priority = p.priority; m.src = sa_; m.dst = 0xFF; m.data = p.data;
            out.push_back(m);
        }
    }
}

void BusNode::respond(uint32_t reqPgn, uint8_t reqDst, std::deque<J1939Msg>& out) const
{
    for (auto& r : responses_) {
        if (r.pgn != reqPgn) continue;
        if (r.destOnly && reqDst == 0xFF) continue;   // dest-specific response: never answer a global sweep
        if (r.bam) { for (auto& f : buildBam(sa_, r.pgn, r.payload)) out.push_back(f); }
        else { J1939Msg m; m.pgn = r.pgn; m.priority = 6; m.src = sa_; m.dst = 0xFF; m.data = r.payload; out.push_back(m); }
    }
}

// ---- live drivable signals ----------------------------------------------------------------

std::vector<uint8_t>* BusNode::periodicData(uint32_t pgn)
{
    for (auto& p : periodics_) if (p.pgn == pgn) return &p.data;
    return nullptr;
}
const std::vector<uint8_t>* BusNode::periodicData(uint32_t pgn) const
{
    for (auto& p : periodics_) if (p.pgn == pgn) return &p.data;
    return nullptr;
}

void BusNode::addSignal(std::string label, std::string unit, uint32_t pgn, uint8_t off, uint8_t width,
                        bool le, double scale, double offset, double minV, double maxV)
{
    signals_.push_back(LiveSignal{ std::move(label), std::move(unit), pgn, off, width, le, scale, offset, minV, maxV });
}

bool BusNode::signalInfo(int idx, std::string& label, std::string& unit, double& minV, double& maxV, double& cur) const
{
    if (idx < 0 || idx >= (int)signals_.size()) return false;
    const LiveSignal& s = signals_[idx];
    label = s.label; unit = s.unit; minV = s.minV; maxV = s.maxV;
    const std::vector<uint8_t>* d = periodicData(s.pgn);
    unsigned raw = 0;
    if (d && (size_t)s.off + s.width <= d->size()) {
        if (s.width == 2) raw = s.le ? ((unsigned)(*d)[s.off] | ((unsigned)(*d)[s.off + 1] << 8))
                                     : (((unsigned)(*d)[s.off] << 8) | (unsigned)(*d)[s.off + 1]);
        else raw = (*d)[s.off];
    }
    cur = (double)raw * s.scale + s.offset;
    return true;
}

bool BusNode::setSignal(int idx, double value)
{
    if (idx < 0 || idx >= (int)signals_.size()) return false;
    const LiveSignal& s = signals_[idx];
    std::vector<uint8_t>* d = periodicData(s.pgn);
    if (!d || (size_t)s.off + s.width > d->size()) return false;
    if (value < s.minV) value = s.minV;
    if (value > s.maxV) value = s.maxV;
    long raw = (long)llround((value - s.offset) / s.scale);
    long maxRaw = (s.width == 2) ? 0xFFFF : 0xFF;
    if (raw < 0) raw = 0;
    if (raw > maxRaw) raw = maxRaw;
    if (s.width == 2) {
        if (s.le) { (*d)[s.off] = (uint8_t)(raw & 0xFF); (*d)[s.off + 1] = (uint8_t)((raw >> 8) & 0xFF); }
        else      { (*d)[s.off] = (uint8_t)((raw >> 8) & 0xFF); (*d)[s.off + 1] = (uint8_t)(raw & 0xFF); }
    } else {
        (*d)[s.off] = (uint8_t)(raw & 0xFF);
    }
    return true;
}

// ---- build the bus: engine's own broadcasts + the peripheral controllers -------------------

void EcmSim::buildBus()
{
    nodes_.clear();
    enginePeriodics_.clear();

    // =======================================================================================
    //  ENGINE #1 (SA 0x00) — the ADEM engine controller. It ALSO implements the full CAT-ext
    //  flash-reprogramming protocol (see handleCatExt below); these broadcasts are its normal
    //  running traffic. Byte layouts/scales conform to the generic SAE J1939 broadcast
    //  reference (byte indices ZERO-BASED; multi-byte signals little-endian, LSB first).
    // =======================================================================================
    auto eng = [&](uint32_t pgn, uint8_t prio, unsigned long long per, std::vector<uint8_t> d) {
        Periodic p; p.pgn = pgn; p.priority = prio; p.periodMs = per; p.data = std::move(d); enginePeriodics_.push_back(std::move(p));
    };
    { uint16_t rpm = (uint16_t)(650.0 / 0.125);   // SPN190 engine speed 650 rpm (0.125 rpm/bit)
      eng(0x00F004, 3, 100, { 0xFF, 0x7D, 0x87,                          // EEC1: SPN512 demand torque [1]=0%, SPN513 actual [2]=+10% (raw-125 %)
                              (uint8_t)(rpm & 0xFF), (uint8_t)(rpm >> 8), //       SPN190 engine speed [3:5] LE16
                              0xFF, 0xFF, 0xFF }); }
    eng(0x00F003, 3,  50, { 0x00,0xFF,0x19,0xFF,0xFF,0xFF,0xFF,0xFF });   // EEC2:  SPN91 pedal [0]=0%, SPN92 load [2]=25% (1 %/bit)
    eng(0x00FEDF, 6, 250, { 0x7D,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // EEC3:  SPN514 nominal friction %torque [0]=0% (raw-125)
    eng(0x00FEEE, 6,1000, { 0x7E,0x50,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // ET1:   SPN110 coolant [0]=86 C, SPN174 fuel temp [1]=40 C (raw-40 C)
    eng(0x00FEEF, 6, 500, { 0xFF,0xFF,0xFF,0x46,0xFF,0xFF,0xFF,0xFF });   // EFL/P1:SPN100 engine oil pressure [3]=280 kPa (4 kPa/bit)
    eng(0x00FEF2, 6, 100, { 0x3C,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // LFE1:  SPN183 fuel rate [0:2] LE16 = 3.0 L/h (0.05 L/h/bit)
    eng(0x00FEF6, 6, 500, { 0xFF,0x00,0x50,0xFF,0xFF,0xFF,0xFF,0xFF });   // IC1:   SPN102 boost [1]=0 kPa (2 kPa/bit), SPN105 intake-man temp [2]=40 C (raw-40)
    eng(0x00FEF7, 6,1000, { 0xFF,0xFF,0xFF,0xFF,0x14,0x01,0x14,0x01 });   // VEP1:  SPN168 battery [4:6] LE16 = 13.8 V, SPN158 keyswitch [6:8] LE16 = 13.8 V (0.05 V/bit)
    eng(0x00FEBD, 6,1000, { 0x64,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // FD1:   SPN975 estimated percent fan speed [0]=40% (0.4 %/bit)
    eng(0x00FEE4, 6,1000, { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // SHUTDN:no shutdown / protection active
    // DM1 (0xFECA Active DTCs) is NOT a static periodic — it is emitted dynamically from the
    // engine's DiagState by serviceDiag() so injected/cleared faults are reflected live.

    // =======================================================================================
    //  THE REST OF THE BUS — a full heavy-vehicle + mobile-plant controller set. Every node
    //  claims its own SA (J1939-81 NAME), broadcasts its spec'd PGNs continuously on their
    //  normal periods, and answers a Component Identification (0xFEEB) request. Only the engine
    //  answers Software Identification (0xFEDA), so the tool's resolve_target still uniquely
    //  identifies the ADEM engine on the populated bus. Values are generic simulated snapshots
    //  (clean-room — no Caterpillar data); PGNs with no reference decode use proprietary-B.
    // =======================================================================================
    auto add = [&](BusNode&& n) { nodes_.push_back(std::move(n)); };

    // Standard identity responses for a mobile-plant EH controller (note 02 step 2 / note 03 grid).
    // Component ID answers a global sweep (the strongest positive ID); Software ID + ECU ID are
    // destination-specific, so a GLOBAL 0xFEDA / 0xFDC5 request still resolves uniquely to the engine
    // while each controller answers its own when addressed directly. make/model are the published
    // vendor families with 'SIM' serials (clean-room — no real serial numbers or PINs).
    auto ehIdentity = [&](BusNode& n, const char* make, const char* model,
                          const char* serial, const char* unit, const char* sw, const char* ecuPart) {
        n.addRequestResponse(PGN_COMPONENT_ID,
            ascii(std::string(make) + "*" + model + "*" + serial + "*" + unit + "*"), true);        // global
        std::vector<uint8_t> swid; swid.push_back(0x01);
        auto s = ascii(std::string(model) + " " + sw + "*"); swid.insert(swid.end(), s.begin(), s.end());
        n.addRequestResponse(PGN_SOFTWARE_ID, swid, true, /*destSpecificOnly*/ true);
        n.addRequestResponse(PGN_ECUID,
            ascii(std::string(ecuPart) + "*" + serial + "*" + make + "*"), true, /*destSpecificOnly*/ true);
    };

    // Transmission #1 (SA 0x03)
    { BusNode n(0x03, 0x03, 0x0003, "Transmission #1");
      n.addPeriodic(0x00F002, 3, 100, { 0xF0,0x00,0x00,0xFF,0xFF,0x00,0x00,0xFF });   // ETC1 selected/current gear neutral
      n.addPeriodic(0x00F005, 6, 100, { 0x7D,0xFF,0xFF,0x7D,0xFF,0xFF,0xFF,0xFF });   // ETC2 SPN524/523 neutral (raw-125)
      n.addPeriodic(0x00FEF8, 6,1000, { 0xFF,0xFF,0xFF,0xFF,0x20,0x2C,0xFF,0xFF });   // TRF1 SPN177 trans oil temp [4:6] LE16 = 80 C (0.03125 C, -273)
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*TRANS*SIM*0003*"), true);
      n.dmCapable = true;              // broadcasts DM1, answers DM services (no faults by default)
      add(std::move(n)); }

    // Brakes / ABS — system controller (SA 0x0B)
    { BusNode n(0x0B, 0x31, 0x000B, "Brakes/ABS");
      n.addPeriodic(0x00F001, 6, 100, { 0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // EBC1 brake/ABS status (not applied)
      n.addPeriodic(0x00FEBF, 6, 100, { 0x00,0x00,0x7D,0x7D,0x7D,0x7D,0x7D,0x7D });   // EBC2 SPN904 front axle [0:2]=0, SPN905-910 rel wheels [2:8]=0.0 km/h (1/16, -7.8125)
      n.addPeriodic(0x00FEF1, 6, 100, { 0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF });   // CCVS1 SPN84 wheel-based speed [1:3]=0 km/h (1/256)
      n.addPeriodic(0x00FEAE, 6,1000, { 0xFF,0xFF,0x64,0x64,0xFF,0xFF,0xFF,0xFF });   // AIR1 SPN1087/1088 service-brake air circ1/2 [2][3]=800 kPa (8 kPa/bit)
      n.addPeriodic(0x00FEFA, 6,1000, { 0xFF,0xC8,0xC8,0xFF,0xFF,0xFF,0xFF,0xFF });   // B    SPN117/118 brake primary/secondary [1][2]=800 kPa (4 kPa/bit)
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*ABS*SIM*000B*"), true);
      n.dmCapable = true;              // broadcasts DM1, answers DM services (healthy)
      add(std::move(n)); }

    // Retarder — Engine (SA 0x0F)
    { BusNode n(0x0F, 0x10, 0x000F, "Retarder (Engine)");
      n.addPeriodic(0x00F000, 3, 100, { 0xFF,0x7D,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // ERC1 SPN1715 intended retarder %torque [1]=0% (raw-125)
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*RETARD*SIM*000F*"), true);
      add(std::move(n)); }

    // Instrument Cluster #1 (SA 0x17)
    { BusNode n(0x17, 0x17, 0x0017, "Instrument Cluster");
      n.addPeriodic(0x00FEFC, 6,1000, { 0xFF,0xBB,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // DD  SPN96 fuel level 1 [1]=75% (0.4 %/bit)
      n.addPeriodic(0x00FEC1, 6,5000, { 0x80,0x1A,0x06,0x00,0xA0,0x86,0x01,0x00 });   // VDHR SPN917 total [0:4]=2000 km, SPN918 trip [4:8]=500 km (5 m/bit)
      n.addPeriodic(0x00FEE0, 6,1000, { 0xFF,0xFF,0xFF,0xFF,0x80,0x3E,0x00,0x00 });   // VD   SPN245 total vehicle distance [4:8] LE32 = 2000 km (0.125 km/bit)
      n.addRequestResponse(0x00FEC1, { 0x80,0x1A,0x06,0x00,0xA0,0x86,0x01,0x00 }, false); // VDHR on request too
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*CLUSTER*SIM*0017*"), true);
      n.dmCapable = true;
      add(std::move(n)); }

    // Body Controller (SA 0x21) — ambient / barometric
    { BusNode n(0x21, 0x21, 0x0021, "Body Controller");
      n.addPeriodic(0x00FEF5, 6,1000, { 0xCB,0xFF,0xFF,0x40,0x25,0xFF,0xFF,0xFF });   // AMB SPN108 baro [0]=101.5 kPa (0.5 kPa), SPN171 ambient [3:5]=25 C (0.03125, -273)
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*BODY*SIM*0021*"), true);
      add(std::move(n)); }

    // Implement / Hydraulics main controller (SA 0x25) — Danfoss PLUS+1 MC-class machine controller
    // broadcasting work-hydraulics telemetry. NAME: Industry Group 3 (Construction), function 23
    // (illustrative machine-controller class), manufacturer 57 (Danfoss, published SAE assignment).
    // Payloads are proprietary-B (manufacturer-defined by nature) with documented simulated values.
    { BusNode n(0x25, "Implement/Hydraulics", /*IG*/3, /*vehSys*/0, /*func*/23, /*funcInst*/0, /*ecuInst*/0, /*mfg*/57, /*id*/250001);
      // 0xFF20 pressures: pump/system [0:2] LE16 1 kPa/bit = 20000 kPa; pilot [2:4] = 3000 kPa;
      //                   load-sense [4:6] = 18000 kPa; system flow-demand [6] = 50 % (0.4 %/bit).
      n.addPeriodic(0x00FF20, 6, 100, { 0x20,0x4E,0xB8,0x0B,0x50,0x46,0x7D,0xFF });
      // 0xFF21 pump/oil: swash [0] = 60 % (0.4 %/bit); pump command [1] = 55 %; main-pump flow [2:4]
      //                  LE16 0.1 L/min/bit = 120.0 L/min; hyd-oil temp [4] = 65 C (1 C/bit, -40);
      //                  hyd-oil level [5] = 80 % (0.4 %/bit).
      n.addPeriodic(0x00FF21, 6, 500, { 0x96,0x89,0xB0,0x04,0x69,0xC8,0xFF,0xFF });
      n.addSignal("Pump pressure", "kPa",  0x00FF20, 0, 2, true,  1.0,   0.0,   0, 35000);
      n.addSignal("Pilot pressure","kPa",  0x00FF20, 2, 2, true,  1.0,   0.0,   0,  6000);
      n.addSignal("Load sense",    "kPa",  0x00FF20, 4, 2, true,  1.0,   0.0,   0, 35000);
      n.addSignal("Pump swash",    "%",    0x00FF21, 0, 1, false, 0.4,   0.0,   0,   100);
      n.addSignal("Main flow",     "L/min",0x00FF21, 2, 2, true,  0.1,   0.0,   0,   300);
      n.addSignal("Oil temp",      "C",    0x00FF21, 4, 1, false, 1.0, -40.0, -40,   150);
      n.addSignal("Oil level",     "%",    0x00FF21, 5, 1, false, 0.4,   0.0,   0,   100);
      ehIdentity(n, "DANFOSS", "PLUS1-MC050", "SIM", "0025", "APP 4.21", "11140345");
      n.dmCapable = true;              // DM1 (seeded hydraulic oil over-temp — see seedDiagnostics)
      add(std::move(n)); }

    // Cab Controller — Primary (SA 0x31) — HVAC / operator (proprietary-B)
    { BusNode n(0x31, 0x31, 0x0031, "Cab Controller");
      n.addPeriodic(0x00FF31, 6,1000, { 0x2C,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF });   // PropB simulated cab climate state
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*CAB*SIM*0031*"), true);
      add(std::move(n)); }

    // Aftertreatment / SCR — exhaust emission controller (SA 0x3D), Tier-4 ADEM
    { BusNode n(0x3D, 0x3D, 0x003D, "Aftertreatment/SCR");
      n.addPeriodic(0x00FE56, 6,1000, { 0xC8,0x6E,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // DEF tank SPN1761 [0]=80% (0.4 %/bit), temp [1]
      n.addPeriodic(0x00FF3D, 6,1000, { 0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // PropB simulated SCR dosing status
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*SCR*SIM*003D*"), true);
      n.dmCapable = true;              // broadcasts DM1 — seeded with a DEF-quality fault (see seedDiagnostics)
      add(std::move(n)); }

    // Telematics / GPS gateway (SA 0x29)
    { BusNode n(0x29, 0x1C, 0x0029, "Telematics/GPS");
      n.addPeriodic(0x00FEE6, 6,1000, { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // TD time/date (not set)
      n.addPeriodic(0x00FEF3, 6,5000, { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF });   // VP vehicle position (no GPS fix)
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*TELE*SIM*0029*"), true);
      add(std::move(n)); }

    // =======================================================================================
    //  MOBILE-PLANT ELECTRO-HYDRAULIC (EH) CONTROLLERS (program TPSAS-RES-002). A representative
    //  off-highway work-hydraulics bus: a proportional valve driver, a hydrostatic drive, an I/O
    //  expansion module, two Danfoss PVED-CC auxiliary valves and a Grayhill joystick / vehicle
    //  display controller — each claiming a proper J1939-81 NAME (industry group / function /
    //  manufacturer per note 02) so the TES discovery cascade + Topographer enumeration (notes
    //  02/03) can be exercised end-to-end on a mixed plant bus. Telemetry is proprietary-B
    //  (0xFF00..0xFFFF, manufacturer-defined by nature) with documented, simulated idle-point byte
    //  layouts; NAME manufacturer/function codes are the published SAE/vendor values where the
    //  project notes document them (Danfoss 57 / function 129; Grayhill 294 / function 135) and
    //  clearly simulated otherwise. Clean-room: generic make/model families + 'SIM' serials, no
    //  real serial numbers or PINs. Each is dmCapable, so the J1939-73 DM stack covers it for free.
    // =======================================================================================

    // Proportional valve driver (SA 0x27) — Danfoss / Eaton Pro-FX AxisPro-class section block.
    { BusNode n(0x27, "Prop. Valve Driver", /*IG*/3, 0, /*func*/128 /*illustrative valve-driver*/, 0, 0, /*mfg*/57, 270001);
      // 0xFF22 spool: measured positions s1..s4 [0:4], commanded s1..s4 [4:8] (0.4 %/bit, 50 % = centred).
      n.addPeriodic(0x00FF22, 3, 50, { 0x7D,0x7D,0x7D,0x7D,0x7D,0x7D,0x7D,0x7D });
      // 0xFF23 drive: solenoid-1 current [0:2] LE16 mA = 850; solenoid-2 [2:4] = 0; PWM duty s1 [4] = 42 %
      //               (0.4 %/bit), s2 [5] = 0; supply [6:8] LE16 0.05 V/bit = 13.8 V.
      n.addPeriodic(0x00FF23, 3, 50, { 0x52,0x03,0x00,0x00,0x69,0x00,0x14,0x01 });
      n.addSignal("Spool 1",      "%",  0x00FF22, 0, 1, false, 0.4, 0.0, 0,  100);
      n.addSignal("Spool 2",      "%",  0x00FF22, 1, 1, false, 0.4, 0.0, 0,  100);
      n.addSignal("Spool 3",      "%",  0x00FF22, 2, 1, false, 0.4, 0.0, 0,  100);
      n.addSignal("Spool 4",      "%",  0x00FF22, 3, 1, false, 0.4, 0.0, 0,  100);
      n.addSignal("Solenoid 1 I", "mA", 0x00FF23, 0, 2, true,  1.0, 0.0, 0, 2500);
      n.addSignal("Solenoid 2 I", "mA", 0x00FF23, 2, 2, true,  1.0, 0.0, 0, 2500);
      ehIdentity(n, "DANFOSS", "PROFX-AXISPRO", "SIM", "0027", "VD 2.08", "11148820");
      n.dmCapable = true;              // DM1 (seeded section-3 solenoid open circuit — seedDiagnostics)
      add(std::move(n)); }

    // Hydrostatic traction drive controller (SA 0x2B) — Sauer-Danfoss closed-loop propulsion.
    { BusNode n(0x2B, "Hydrostatic Drive", 3, 0, /*func*/25 /*illustrative propulsion*/, 0, 0, /*mfg*/57, 430001);
      // 0xFF24: ground speed [0:2] LE16 1/256 km/h = 0 (stationary); drive-pump command [2] = neutral
      //         (0.4 %/bit, 50 %); motor displacement [3] = 100 % (0.4 %/bit); charge pressure [4:6]
      //         LE16 1 kPa/bit = 2500 kPa; direction [6] = 0 (0 N / 1 fwd / 2 rev).
      n.addPeriodic(0x00FF24, 3, 100, { 0x00,0x00,0x7D,0xFA,0xC4,0x09,0x00,0xFF });
      n.addSignal("Ground speed",   "km/h", 0x00FF24, 0, 2, true,  1.0/256.0, 0.0, 0,   60);
      n.addSignal("Pump command",   "%",    0x00FF24, 2, 1, false, 0.4,       0.0, 0,  100);
      n.addSignal("Motor disp",     "%",    0x00FF24, 3, 1, false, 0.4,       0.0, 0,  100);
      n.addSignal("Charge pressure","kPa",  0x00FF24, 4, 2, true,  1.0,       0.0, 0, 5000);
      n.addSignal("Direction 0N1F2R","",    0x00FF24, 6, 1, false, 1.0,       0.0, 0,    2);
      ehIdentity(n, "SAUER-DANFOSS", "SERIES90-H1", "SIM", "002B", "HS 1.16", "11153010");
      n.dmCapable = true;
      add(std::move(n)); }

    // I/O expansion module (SA 0x2C) — ifm ecomat-class. Manufacturer code SIMULATED (not a published
    // assignment) so the discovery's 'manufacturer-code only / inferred' (amber) path is exercised.
    { BusNode n(0x2C, "I/O Module", 3, 0, /*func*/143 /*illustrative I/O*/, 0, 0, /*mfg*/0x3FF /*simulated*/, 440001);
      // 0xFF25 analog inputs: AI0..AI3 as LE16 millivolts = 2500 / 500 / 0 / 1000.
      n.addPeriodic(0x00FF25, 6, 200, { 0xC4,0x09,0xF4,0x01,0x00,0x00,0xE8,0x03 });
      // 0xFF26 digital/output: digital-input bitmap [0] = 0x05; output bitmap [1] = 0; module temp [2]
      //                        = 45 C (1 C/bit, -40); supply [3:5] LE16 0.05 V/bit = 13.9 V.
      n.addPeriodic(0x00FF26, 6, 200, { 0x05,0x00,0x55,0x16,0x01,0xFF,0xFF,0xFF });
      n.addSignal("AI0",         "mV", 0x00FF25, 0, 2, true,  1.0,   0.0,   0, 5000);
      n.addSignal("AI1",         "mV", 0x00FF25, 2, 2, true,  1.0,   0.0,   0, 5000);
      n.addSignal("AI2",         "mV", 0x00FF25, 4, 2, true,  1.0,   0.0,   0, 5000);
      n.addSignal("AI3",         "mV", 0x00FF25, 6, 2, true,  1.0,   0.0,   0, 5000);
      n.addSignal("Digital in",  "",   0x00FF26, 0, 1, false, 1.0,   0.0,   0,  255);
      n.addSignal("Module temp", "C",  0x00FF26, 2, 1, false, 1.0, -40.0, -40,  120);
      ehIdentity(n, "IFM", "ECOMAT-CR0033", "SIM", "002C", "IO 1.04", "11156620");
      n.dmCapable = true;
      add(std::move(n)); }

    // Danfoss PVED-CC auxiliary valve #1 (SA 0x80) — J1939 / ISO 11783 (note 06). Published NAME:
    // Identity 201001, manufacturer 57 (Danfoss), function 129 (auxiliary valve on tractor), vehicle
    // system 1, industry group 2 (Agricultural), function instance 0 (= node-id index, 0x80 range).
    { BusNode n(0x80, "PVED-CC Aux Valve 1", /*IG*/2, /*vehSys*/1, /*func*/129, /*funcInst*/0, 0, /*mfg*/57, 201001);
      // 0xFF80: AVC command [0] (0.4 %/bit, 50 % = no command); AVEF estimated flow [1] (50 % = 0);
      //         measured spool position [2] (50 % = centred); device state [3] = 1 (operational);
      //         supply [4:6] LE16 0.05 V/bit = 13.8 V; valve temp [6] = 50 C (1 C/bit, -40).
      n.addPeriodic(0x00FF80, 3, 50, { 0x7D,0x7D,0x7D,0x01,0x14,0x01,0x5A,0xFF });
      n.addSignal("AVC command", "%", 0x00FF80, 0, 1, false, 0.4,   0.0,   0, 100);
      n.addSignal("AVEF flow",   "%", 0x00FF80, 1, 1, false, 0.4,   0.0,   0, 100);
      n.addSignal("Spool pos",   "%", 0x00FF80, 2, 1, false, 0.4,   0.0,   0, 100);
      n.addSignal("Valve temp",  "C", 0x00FF80, 6, 1, false, 1.0, -40.0, -40, 120);
      ehIdentity(n, "DANFOSS", "PVED-CC", "SIM", "0080", "S7 2.40", "11045501");
      n.dmCapable = true;              // DM1 (seeded valve-position fault — seedDiagnostics)
      add(std::move(n)); }

    // Danfoss PVED-CC auxiliary valve #2 (SA 0x81) — same family, function instance 1 (exercises the
    // NAME function-instance discriminator: 'which of N identical functions', note 02).
    { BusNode n(0x81, "PVED-CC Aux Valve 2", /*IG*/2, /*vehSys*/1, /*func*/129, /*funcInst*/1, 0, /*mfg*/57, 201002);
      n.addPeriodic(0x00FF81, 3, 50, { 0x7D,0x7D,0x7D,0x01,0x14,0x01,0x5A,0xFF });
      n.addSignal("AVC command", "%", 0x00FF81, 0, 1, false, 0.4,   0.0,   0, 100);
      n.addSignal("AVEF flow",   "%", 0x00FF81, 1, 1, false, 0.4,   0.0,   0, 100);
      n.addSignal("Spool pos",   "%", 0x00FF81, 2, 1, false, 0.4,   0.0,   0, 100);
      n.addSignal("Valve temp",  "C", 0x00FF81, 6, 1, false, 1.0, -40.0, -40, 120);
      ehIdentity(n, "DANFOSS", "PVED-CC", "SIM", "0081", "S7 2.40", "11045501");
      n.dmCapable = true;
      add(std::move(n)); }

    // Grayhill 3JG2 joystick / vehicle display controller (SA 0xF1) — J1939 (note 07). Published:
    // manufacturer 294 (Grayhill), function 135, joystick broadcast on PGN 0xFF03. Industry group is
    // not published for this part → Construction (3) assumed for the simulation.
    { BusNode n(0xF1, "Joystick/VDC", /*IG*/3, 0, /*func*/135, 0, 0, /*mfg*/294, 241001);
      // 0xFF03 joystick: X [0], Y [1], Z/twist [2] (0.4 %/bit, 50 % = centred); thumb-roller [3] = 0;
      //                  button bitmap 0-7 [4], 8-15 [5] (none pressed); deadman/enable [6] = 1.
      n.addPeriodic(0x00FF03, 3, 50, { 0x7D,0x7D,0x7D,0x00,0x00,0x00,0x01,0xFF });
      n.addSignal("Joystick X",   "%", 0x00FF03, 0, 1, false, 0.8, -100.0, -100, 100);
      n.addSignal("Joystick Y",   "%", 0x00FF03, 1, 1, false, 0.8, -100.0, -100, 100);
      n.addSignal("Joystick Z",   "%", 0x00FF03, 2, 1, false, 0.8, -100.0, -100, 100);
      n.addSignal("Thumb roller", "%", 0x00FF03, 3, 1, false, 0.4,    0.0,    0, 100);
      n.addSignal("Buttons",      "",  0x00FF03, 4, 1, false, 1.0,    0.0,    0, 255);
      ehIdentity(n, "GRAYHILL", "3JG2-VDC", "SIM", "00F1", "VDC 3.02", "3JG2-1965");
      n.dmCapable = true;              // DM1 (seeded joystick-signal fault — seedDiagnostics)
      add(std::move(n)); }
}

void EcmSim::serviceBus(unsigned long long now)
{
    for (auto& p : enginePeriodics_) {
        if (now - p.last >= p.periodMs) {
            p.last = now;
            J1939Msg m; m.pgn = p.pgn; m.priority = p.priority; m.src = ECM_SA; m.dst = 0xFF; m.data = p.data;
            enqueue(m);
        }
    }
    if (now - lastClaim_ >= 5000) { enqueue(addressClaimed()); lastClaim_ = now; }
    for (auto& n : nodes_) n.service(now, outq_);

    serviceDiag(now);   // emit DM1 (Active DTCs) broadcasts from live DiagState (engine + dm-capable nodes)

    // Bounded RX buffer: a real adapter has a finite receive FIFO and drops the oldest frames
    // when the client is not reading fast enough. This keeps a continuously-running (never
    // self-terminating) bus from growing unbounded if a client polls slowly. Only runs outside
    // flash mode (the flash reply path is low-volume and drained promptly, so it never drops).
    while (outq_.size() > MAX_QUEUE) outq_.pop_front();
}

// ---- multipacket (BAM) for strings longer than 8 bytes -----------------------------------

void EcmSim::sendBam(uint32_t pgn, const std::vector<uint8_t>& payload)
{
    for (auto& f : buildBam(ECM_SA, pgn, payload)) enqueue(f);   // engine (SA 0x00) multi-packet
}

static std::vector<uint8_t> ascii(const std::string& s)
{
    return std::vector<uint8_t>(s.begin(), s.end());
}

// ---- request handling ---------------------------------------------------------------------

void EcmSim::respondToRequest(uint32_t reqPgn, uint8_t dst)
{
    // SAE J1939-73 diagnostic reads (DM1/DM2/DM5/DM4) are answered from live DiagState.
    if (handleDiagRequest(reqPgn, dst)) return;

    // Engine #1 (SA 0x00) answers if the request is global or addressed to it. It is the only node
    // that answers Software Identification (0xFEDA) — so the tool's resolve_target uniquely hits it.
    if (dst == 0xFF || dst == ECM_SA) {
        switch (reqPgn) {
            case PGN_ADDRESS_CLAIMED: enqueue(addressClaimed()); break;
            // SPN237 VIN: configurable (default a reference-valid 17-char VIN), '*' terminator.
            case PGN_VIN:          sendBam(PGN_VIN, ascii(vin_ + "*")); break;
            case PGN_SOFTWARE_ID: {   // byte0 = field count, then '*'-delimited ASCII (SPN 234/965)
                // Software ID drives the tool's target-family resolution ("ADEM<n>"). Configurable via
                // ADEM_EMU_SWID so the one emulator can present as any generation for bench testing
                // (e.g. set ADEM_EMU_SWID="ADEM6 6441970-00" to exercise the ADEM6/protected path).
                std::vector<uint8_t> p; p.push_back(0x01);
                // Win32 GetEnvironmentVariableA (not std::getenv): the /MT static CRT does not reliably
                // reflect the live process environment, so getenv can miss both inherited and
                // runtime-set vars. GetEnvironmentVariableA reads the actual process env block.
                char swbuf[64] = {0};
                DWORD swn = GetEnvironmentVariableA("ADEM_EMU_SWID", swbuf, (DWORD)sizeof(swbuf));
                std::string swid = (swn > 0 && swn < sizeof(swbuf)) ? std::string(swbuf) : std::string("ADEM4 2660376-00");
                auto s = ascii(swid + "*"); p.insert(p.end(), s.begin(), s.end());
                sendBam(PGN_SOFTWARE_ID, p);
                break;
            }
            case PGN_COMPONENT_ID: sendBam(PGN_COMPONENT_ID, ascii("CATERPILLAR*ADEM4*SIM*0001*")); break;
            case PGN_ECUID: {
                // ECU Identification — the CAT ADEM ECU-info block, '*'-delimited ASCII over TP.
                // Field ORDER follows the RE'd ECU parameter list: HardwareType, LocationCode,
                // ECUPartNumber, PartNumber, ChangeNumber, CustomerECUPartNumber, ECUSerialNumber,
                // BootPartNumber. Generic simulated values (clean-room — no Caterpillar data).
                sendBam(PGN_ECUID, ascii(
                    "ADEM4-SIM*"        // HardwareType
                    "ENGINE*"           // LocationCode
                    "2660376-00*"       // ECUPartNumber
                    "5411785-00*"       // PartNumber
                    "01*"               // ChangeNumber
                    "0000000-00*"       // CustomerECUPartNumber
                    "TPSAS0000000001*"  // ECUSerialNumber
                    "2860911-00*"));    // BootPartNumber
                break;
            }
            case 0x00FEE5: {          // Engine Hours / Revolutions — SPN247 total hours, 0.05 h/bit
                uint32_t raw = (uint32_t)(1234.5 / 0.05);   // 24690 ticks = 1234.5 h
                J1939Msg m; m.pgn = 0x00FEE5; m.priority = 6; m.src = ECM_SA; m.dst = 0xFF;
                m.data = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8), (uint8_t)(raw >> 16), (uint8_t)(raw >> 24), 0xFF, 0xFF, 0xFF, 0xFF };
                enqueue(m);
                break;
            }
            default: break;  // unknown request -> silently ignored (a real ECM may NACK)
        }
    }
    // Every other controller answers for the PGNs it owns.
    for (auto& n : nodes_) if (dst == 0xFF || dst == n.sa()) n.respond(reqPgn, dst, outq_);
    // An address-claim request re-announces every node too.
    if (reqPgn == PGN_ADDRESS_CLAIMED)
        for (auto& n : nodes_) if (dst == 0xFF || dst == n.sa()) enqueue(n.addressClaimed());
}

// ---- SAE J1939-73 diagnostic stack (DTCs / readiness / tests) ------------------------------

void EcmSim::seedDiagnostics()
{
    using namespace j1939diag;
    // Reset + seed a realistic, readable fault set so the tool's read-DTC path returns data;
    // clearing (DM11/DM3) then empties it. Env TPRP_EMU_DTC=clean starts with no faults.
    engineDiag_ = DiagState{};
    char env[16] = { 0 };
    bool clean = (GetEnvironmentVariableA("TPRP_EMU_DTC", env, (DWORD)sizeof(env)) > 0 && _stricmp(env, "clean") == 0);

    if (!clean) {
        engineDiag_.active.push_back(Dtc{ 110, 0, 0, 1 });   // SPN110 coolant temp, FMI0 (data high), OC1
        engineDiag_.active.push_back(Dtc{ 100, 1, 0, 2 });   // SPN100 oil pressure, FMI1 (data low),  OC2
        engineDiag_.stored.push_back(Dtc{ 174, 16, 0, 1 });  // SPN174 fuel temp, FMI16, previously active
    }
    engineDiag_.recomputeLamps();
    engineDiag_.readinessSupported = 0x00FF; engineDiag_.readinessComplete = 0x00FB;
    engineDiag_.tests.push_back(TestResult{ 1, 110, 0, 86, 0, 105 });     // coolant temp (C)
    engineDiag_.tests.push_back(TestResult{ 2, 100, 0, 280, 200, 600 });  // oil pressure (kPa)
    engineDiag_.tests.push_back(TestResult{ 3, 168, 0, 138, 110, 150 });  // battery (0.1 V)

    for (auto& n : nodes_) {
        if (n.sa() == 0x3D && !clean) {
            n.diag.active.push_back(Dtc{ 1761, 18, 0, 3 }); // SPN1761 DEF level, FMI18 (low moderate), OC3
            n.diag.stored.push_back(Dtc{ 3364, 2, 0, 1 });  // SPN3364 DEF quality, previously active
        }
        // Mobile-plant EH controllers: a few seeded faults so the DM read path returns data on the new
        // nodes too. SPN 1638 is the real SAE hydraulic-oil-temperature SPN; the valve/joystick codes
        // are proprietary-range SPNs (519192+), simulated. FMIs are standard J1939-73.
        if (!clean) {
            if (n.sa() == 0x25) n.diag.active.push_back(Dtc{ 1638,   16, 0, 1 }); // hyd oil temp, FMI16 high-moderate
            if (n.sa() == 0x27) n.diag.active.push_back(Dtc{ 520192,  5, 0, 1 }); // valve solenoid sec.3, FMI5 current low / open
            if (n.sa() == 0x80) n.diag.active.push_back(Dtc{ 520300,  7, 0, 1 }); // valve spool, FMI7 mechanical not responding
            if (n.sa() == 0xF1) n.diag.active.push_back(Dtc{ 520400,  2, 0, 1 }); // joystick axis, FMI2 data erratic
        }
        n.diag.recomputeLamps();
        n.diag.readinessSupported = 0x00FF; n.diag.readinessComplete = 0x00FF;
        if (n.sa() == 0x3D) n.diag.tests.push_back(TestResult{ 10, 1761, 0, 200, 50, 255 });
    }
}

std::vector<std::pair<uint8_t, j1939diag::DiagState*>> EcmSim::diagModules()
{
    std::vector<std::pair<uint8_t, j1939diag::DiagState*>> v;
    v.push_back({ ECM_SA, &engineDiag_ });
    for (auto& n : nodes_) if (n.dmCapable) v.push_back({ n.sa(), &n.diag });
    return v;
}

void EcmSim::emitDm(uint8_t sa, uint32_t pgn, const std::vector<uint8_t>& payload, uint8_t dst)
{
    if (payload.size() <= 8) {
        J1939Msg m; m.pgn = pgn; m.priority = 6; m.src = sa; m.dst = dst; m.data = payload;
        if (dst == 0xFF) while (m.data.size() < 8) m.data.push_back(0xFF);  // pad broadcast single frames
        enqueue(m);
    } else {
        for (auto& f : buildBam(sa, pgn, payload)) enqueue(f);   // multi-packet (BAM) for long DM lists
    }
}

void EcmSim::serviceDiag(unsigned long long now)
{
    using namespace j1939diag;
    if (now - engineDm1Last_ >= 1000) {
        engineDm1Last_ = now;
        emitDm(ECM_SA, PGN_DM1, buildDmList(engineDiag_.lamp, engineDiag_.flash, engineDiag_.active));
    }
    for (auto& n : nodes_) {
        if (!n.dmCapable) continue;
        if (now - n.dm1Last >= 1000) {
            n.dm1Last = now;
            emitDm(n.sa(), PGN_DM1, buildDmList(n.diag.lamp, n.diag.flash, n.diag.active));
        }
    }
}

bool EcmSim::handleDiagRequest(uint32_t reqPgn, uint8_t dst)
{
    using namespace j1939diag;
    if (reqPgn == PGN_DM11 || reqPgn == PGN_DM3) {
        // J1939-73: DM3/DM11 are REQUESTED (PGN 59904 carrying the DM PGN); each addressed module
        // clears and answers the requester with a positive ACKM.
        for (auto& mod : diagModules()) {
            uint8_t sa = mod.first; DiagState& d = *mod.second;
            if (dst != 0xFF && dst != sa) continue;
            if (reqPgn == PGN_DM11) d.clearActive(); else d.clearStored();
            sendAck(sa, toolSa_, reqPgn, 0x00 /*positive*/);
        }
        return true;
    }
    if (reqPgn != PGN_DM1 && reqPgn != PGN_DM2 && reqPgn != PGN_DM5 && reqPgn != PGN_DM4) return false;
    for (auto& mod : diagModules()) {
        uint8_t sa = mod.first; DiagState& d = *mod.second;
        if (dst != 0xFF && dst != sa) continue;
        if      (reqPgn == PGN_DM1) emitDm(sa, PGN_DM1, buildDmList(d.lamp, d.flash, d.active));
        else if (reqPgn == PGN_DM2) emitDm(sa, PGN_DM2, buildDmList(d.lamp, d.flash, d.stored));
        else if (reqPgn == PGN_DM5) emitDm(sa, PGN_DM5, buildDm5(d));
        else if (reqPgn == PGN_DM4) emitDm(sa, PGN_DM4, { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF }); // no freeze frame
    }
    return true;
}

void EcmSim::sendAck(uint8_t sa, uint8_t toSa, uint32_t ackedPgn, uint8_t control)
{
    // ACKM (PGN 0xE800): [control][2..4 = 0xFF][address acknowledged][PGN:3 LE].
    J1939Msg m; m.pgn = j1939diag::PGN_ACKM; m.priority = 6; m.src = sa; m.dst = toSa;
    m.data = { control, 0xFF, 0xFF, 0xFF, toSa,
               (uint8_t)(ackedPgn & 0xFF), (uint8_t)((ackedPgn >> 8) & 0xFF), (uint8_t)((ackedPgn >> 16) & 0xFF) };
    enqueue(m);
}

bool EcmSim::handleDiagCommand(const J1939Msg& m)
{
    using namespace j1939diag;
    if (m.pgn == PGN_DM11 || m.pgn == PGN_DM3) {
        for (auto& mod : diagModules()) {                       // global clear: all modules clear + ACK
            uint8_t sa = mod.first; DiagState& d = *mod.second;
            if (m.pgn == PGN_DM11) d.clearActive(); else d.clearStored();
            sendAck(sa, m.src, m.pgn, 0x00 /*positive*/);
        }
        return true;
    }
    if (m.pgn == PGN_DM7) {                                      // run component test -> DM30 results
        for (auto& mod : diagModules()) {
            if (m.dst != mod.first) continue;
            emitDm(mod.first, PGN_DM30, buildDm30(mod.second->tests), m.src);
        }
        return true;
    }
    return false;
}

// ---- management / configuration control (emulator GUI console) ----------------------------

int EcmSim::nodeCount()
{
    std::lock_guard<std::mutex> lk(mu_);
    return 1 + (int)nodes_.size();   // engine + peripheral controllers
}

bool EcmSim::nodeInfo(int index, uint8_t& sa, std::string& label, int& active, int& stored, uint8_t& lamp)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (index == 0) {
        sa = ECM_SA; label = "Engine #1 (ADEM)";
        active = (int)engineDiag_.active.size(); stored = (int)engineDiag_.stored.size(); lamp = engineDiag_.lamp;
        return true;
    }
    int i = index - 1;
    if (i < 0 || i >= (int)nodes_.size()) return false;
    sa = nodes_[i].sa(); label = nodes_[i].label();
    active = (int)nodes_[i].diag.active.size(); stored = (int)nodes_[i].diag.stored.size(); lamp = nodes_[i].diag.lamp;
    return true;
}

void EcmSim::injectDtc(uint8_t sa, uint32_t spn, uint8_t fmi, uint8_t oc)
{
    std::lock_guard<std::mutex> lk(mu_);
    j1939diag::DiagState* d = (sa == ECM_SA) ? &engineDiag_ : nullptr;
    if (!d) for (auto& n : nodes_) if (n.sa() == sa) { d = &n.diag; break; }
    if (!d) return;
    d->active.push_back(j1939diag::Dtc{ spn, fmi, 0, (uint8_t)(oc ? oc : 1) });
    d->recomputeLamps();
}

void EcmSim::clearDtcsFor(uint8_t sa, int which)
{
    std::lock_guard<std::mutex> lk(mu_);
    j1939diag::DiagState* d = (sa == ECM_SA) ? &engineDiag_ : nullptr;
    if (!d) for (auto& n : nodes_) if (n.sa() == sa) { d = &n.diag; break; }
    if (!d) return;
    if (which == 0 || which == 2) d->clearActive();
    if (which == 1 || which == 2) d->clearStored();
}

std::string EcmSim::vin() { std::lock_guard<std::mutex> lk(mu_); return vin_; }
void EcmSim::setVin(const std::string& v) { std::lock_guard<std::mutex> lk(mu_); vin_ = v; }

void EcmSim::setFaultByName(const char* name)
{
    std::lock_guard<std::mutex> lk(mu_);
    fault_ = Fault::None; faultArmed_ = false;
    if (!name) return;
    if      (_stricmp(name, "inappropriate") == 0) fault_ = Fault::Inappropriate;
    else if (_stricmp(name, "badfileinfo")   == 0) fault_ = Fault::BadFileInfo;
    else if (_stricmp(name, "invalidoffset") == 0) { fault_ = Fault::InvalidOffset; faultArmed_ = true; }
    else if (_stricmp(name, "badchecksum")   == 0) fault_ = Fault::BadChecksum;
    else if (_stricmp(name, "erasefail")     == 0) fault_ = Fault::EraseFail;
    else if (_stricmp(name, "stuck")         == 0) fault_ = Fault::Stuck;
    writeSharedFault(name);   // publish so a separate flash client process sees this fault
}

void EcmSim::refreshFaultFromShared()
{
    std::string name = readSharedFault();
    if (name.empty()) return;   // no shared control -> keep the current (env-derived) fault
    fault_ = Fault::None; faultArmed_ = false;
    if      (_stricmp(name.c_str(), "inappropriate") == 0) fault_ = Fault::Inappropriate;
    else if (_stricmp(name.c_str(), "badfileinfo")   == 0) fault_ = Fault::BadFileInfo;
    else if (_stricmp(name.c_str(), "invalidoffset") == 0) { fault_ = Fault::InvalidOffset; faultArmed_ = true; }
    else if (_stricmp(name.c_str(), "badchecksum")   == 0) fault_ = Fault::BadChecksum;
    else if (_stricmp(name.c_str(), "erasefail")     == 0) fault_ = Fault::EraseFail;
    else if (_stricmp(name.c_str(), "stuck")         == 0) fault_ = Fault::Stuck;
}

bool EcmSim::flashActive() { std::lock_guard<std::mutex> lk(mu_); return inFlash_; }

void EcmSim::setToolSa(uint8_t sa) { std::lock_guard<std::mutex> lk(mu_); toolSa_ = sa; }

// Live drivable signals (console sliders). Engine (SA 0x00) exposes none for now; the mobile-plant
// EH BusNodes carry their drivable telemetry. Locked so a set can't race the bus service in poll().
int EcmSim::signalCount(uint8_t sa)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (sa == ECM_SA) return 0;
    for (auto& n : nodes_) if (n.sa() == sa) return n.signalCount();
    return 0;
}
bool EcmSim::signalInfo(uint8_t sa, int idx, std::string& label, std::string& unit, double& minV, double& maxV, double& cur)
{
    std::lock_guard<std::mutex> lk(mu_);
    for (auto& n : nodes_) if (n.sa() == sa) return n.signalInfo(idx, label, unit, minV, maxV, cur);
    return false;
}
bool EcmSim::setSignal(uint8_t sa, int idx, double value)
{
    std::lock_guard<std::mutex> lk(mu_);
    for (auto& n : nodes_) if (n.sa() == sa) return n.setSignal(idx, value);
    return false;
}
// ---- entry points -------------------------------------------------------------------------

void EcmSim::onSend(const J1939Msg& m)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (!running_) return;
    toolSa_ = m.src;   // learn the tool's claimed address

    if (handleDiagCommand(m)) return;   // DM11/DM3 clear, DM7->DM30 test (J1939-73 commands)

    switch (m.pgn) {
        case PGN_REQUEST:
            if (m.data.size() >= 3) {
                uint32_t req = (uint32_t)m.data[0] | ((uint32_t)m.data[1] << 8) | ((uint32_t)m.data[2] << 16);
                respondToRequest(req, m.dst);   // route to the engine and/or the addressed node(s)
            }
            break;
        case PGN_ADDRESS_CLAIMED:
            // Someone claimed an address; our controllers use distinct SAs so there is no
            // contention. Re-announce every node.
            enqueue(addressClaimed());
            for (auto& n : nodes_) enqueue(n.addressClaimed());
            break;
        case PGN_PROPRIETARY_A:
            handleCatExt(m.src, m.data);   // single-frame CAT-ext flash message (<= 8 bytes)
            break;
        case PGN_TP_CM:
            handleTpCm(m);
            break;
        case PGN_TP_DT:
            handleTpDt(m);
            break;
        case catsec::PGN_DM14:
            handleDm14(m);
            break;
        case catsec::PGN_DM16:
            handleDm16(m);
            break;

        default:
            break;
    }
}

// ---- real ADEM J1939 CAT-ext flash protocol (ECM side) ------------------------------------

J1939Msg EcmSim::dm13Sleep() const
{
    // DM13 "stop broadcast" — a real ADEM ECM quiets the bus as it enters flash mode.
    J1939Msg m; m.pgn = catj::PGN_DM13; m.priority = 6; m.src = ECM_SA; m.dst = 0xFF;
    m.data = { 0xFF, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF };
    return m;
}

void EcmSim::beginFlash()
{
    inFlash_ = true; flashMode_ = catj::M_INFLASH;
    curOffset_ = 0; armLen_ = 0; storedChecksum_ = 0; burnUntilMs_ = 0;
    flashBuf_.clear(); flashCrc_ = 0xFFFFFFFFu; swPartNumber_.clear();
    lastFlashMs_ = now_ms();
    enqueue(dm13Sleep());   // quiet the bus (and the ECM stops its own periodic broadcasts below)
}

void EcmSim::checkKillTimer()
{
    // J1939 flash session idle kill timer (map §3.7): drop flash mode if the tester goes quiet.
    if (inFlash_ && (now_ms() - lastFlashMs_) > FLASH_KILL_MS) {
        inFlash_ = false; flashMode_ = catj::M_NOTFLASH;
    }
}

bool EcmSim::parseFileInfo(const uint8_t* p, size_t n)
{
    // Decode the F003 file-information block as a sequence of [SubID][Len][Data] TLVs; capture the
    // software part number if present. Returns false only on a malformed (overrunning) TLV.
    size_t i = 0;
    while (i + 2 <= n) {
        uint8_t sub = p[i];
        uint8_t len = p[i + 1];
        if (i + 2 + (size_t)len > n) return false;
        if (sub == catj::SUB_SOFTWARE_PART && len > 0)
            swPartNumber_.assign((const char*)(p + i + 2), len);
        i += 2 + (size_t)len;
    }
    return true;
}

void EcmSim::replyStatus(uint8_t src, uint8_t cmd, uint8_t mode, const std::vector<uint8_t>& additional)
{
    std::vector<uint8_t> post;
    catj::putBe16(post, catj::packStatus(cmd, mode));                 // 2-byte BE status word
    post.insert(post.end(), additional.begin(), additional.end());   // 0 or 4 AdditionalData bytes
    J1939Msg r; r.pgn = PGN_PROPRIETARY_A; r.priority = 6; r.src = ECM_SA; r.dst = src;
    r.data = catj::buildMsg(catj::CMD_WRITE_RESP, catj::PID_FLASH_STATUS, post);  // [0x83][0x88][...]
    while (r.data.size() < 8) r.data.push_back(0xFF);                // pad single frame to 8 bytes
    enqueue(r);
}

void EcmSim::sendCurrentStatus(uint8_t src)
{
    // Reply to a PID 0x88 READ (the tester polling during the checksum/burn phase).
    lastFlashMs_ = now_ms();   // a poll keeps the flash session alive
    if (flashMode_ == catj::M_FILEPROC) {
        if (now_ms() < burnUntilMs_) { replyStatus(src, catj::FC_BUSY, catj::M_FILEPROC, {}); return; }
        flashMode_ = catj::M_FILEDONE;   // simulated burn elapsed
    }
    if (flashMode_ == catj::M_FILEDONE) {
        uint32_t actual = flashCrc_ ^ 0xFFFFFFFFu;
        // Pass requires the received image's checksum to match what the tester declared (and the
        // 'badchecksum' fault forces a mismatch, to exercise verify-before-reset / recovery).
        bool ok = (actual == storedChecksum_) && !flashBuf_.empty() && fault_ != Fault::BadChecksum;
        std::vector<uint8_t> cs; catj::putBe32(cs, storedChecksum_);  // ECM echoes the stored checksum
        replyStatus(src, ok ? catj::FC_CHECKSUM : catj::FC_BADCHECKSUM, catj::M_FILEDONE, cs);
        return;
    }
    replyStatus(src, catj::FC_OK, flashMode_, {});
}

void EcmSim::handleCatExt(uint8_t src, const std::vector<uint8_t>& payload)
{
    uint8_t cmdByte = 0; uint16_t pid = 0;
    size_t hdr = catj::parseHeader(payload, cmdByte, pid);
    if (hdr == 0) return;
    const uint8_t* post = payload.data() + hdr;
    size_t plen = payload.size() - hdr;
    lastFlashMs_ = now_ms();   // any flash message refreshes the session kill timer

    switch (pid) {
    case catj::PID_FLASH_STATUS: {
        if (cmdByte == catj::CMD_READ) { sendCurrentStatus(src); break; }
        if (plen < 2) break;                          // write carries a status word
        uint8_t cmd  = catj::statusCmd(post[0], post[1]);
        uint8_t mode = catj::statusMode(post[1]);

        if (cmd == catj::FC_OK && mode == catj::M_INFLASH) {
            // Enter flash. A real ECM refuses with InappropriateToFlash if preconditions (e.g.
            // engine stopped) are not met — modelled by the 'inappropriate' fault.
            refreshFaultFromShared();   // pull a fault armed in another process (the console)
            if (fault_ == Fault::Inappropriate) { replyStatus(src, catj::FC_INAPPROPRIATE, catj::M_NOTFLASH, {}); break; }
            beginFlash();
            replyStatus(src, catj::FC_OK, catj::M_INFLASH, {});
        } else if (cmd == catj::FC_CHECKSUM && mode == catj::M_FILEDONE) {
            if (!inFlash_) break;
            storedChecksum_ = (plen >= 6) ? catj::getBe32(post + 2) : 0;
            if (fault_ == Fault::EraseFail) { replyStatus(src, catj::FC_ERASEFAIL, catj::M_FILEDONE, {}); break; }
            flashMode_ = catj::M_FILEPROC;
            burnUntilMs_ = now_ms() + (fault_ == Fault::Stuck ? 3600000ULL : 1000ULL);  // 'stuck' = never finishes
            replyStatus(src, catj::FC_BUSY, catj::M_FILEPROC, {});
        } else if (cmd == catj::FC_TERMINATE) {
            inFlash_ = false; flashMode_ = catj::M_NOTFLASH;
            replyStatus(src, catj::FC_TERMINATE, catj::M_NOTFLASH, {});
        } else if (cmd == catj::FC_ABORT) {
            // Honoured in or out of flash mode (the tool uses it as a recovery step).
            if (mode == catj::M_INFLASH && inFlash_) { curOffset_ = 0; armLen_ = 0; replyStatus(src, catj::FC_ABORT, catj::M_INFLASH, {}); }
            else { inFlash_ = false; flashMode_ = catj::M_NOTFLASH; replyStatus(src, catj::FC_ABORT, catj::M_NOTFLASH, {}); }
        }
        break;
    }
    case catj::PID_FLASH_INFO: {
        if (!inFlash_) break;                 // only meaningful in flash mode
        // A real ECM validates the software part number here; 'badfileinfo' models a rejected one.
        if (fault_ == Fault::BadFileInfo) { replyStatus(src, catj::FC_BADFILEINFO, flashMode_, {}); break; }
        if (!parseFileInfo(post, plen)) { replyStatus(src, catj::FC_BADFILEINFO, flashMode_, {}); break; }
        flashMode_ = catj::M_FILE;
        replyStatus(src, catj::FC_OK, catj::M_FILE, {});
        break;
    }
    case catj::PID_FLASH_CONTROL: {
        if (!inFlash_ || plen < 6) break;
        uint32_t offset = catj::getBe32(post);
        uint16_t len    = catj::getBe16(post + 4);
        if (curOffset_ == 0 && offset != 0) {
            std::vector<uint8_t> expected; catj::putBe32(expected, 0);
            replyStatus(src, catj::FC_INVALIDOFFSET, catj::M_FILE, expected);   // first region must be 0
        } else {
            curOffset_ = offset; armLen_ = len;
            replyStatus(src, catj::FC_PROCEED, catj::M_FILE, {});
        }
        break;
    }
    case catj::PID_FLASH_DATA: {
        if (!inFlash_ || plen < 4) break;
        uint32_t offset = catj::getBe32(post);
        const uint8_t* d = post + 4;
        size_t dlen = plen - 4;
        if (faultArmed_ && fault_ == Fault::InvalidOffset) {
            // One-shot: reject the first data chunk with InvalidOffset (resend request).
            faultArmed_ = false;
            std::vector<uint8_t> expected; catj::putBe32(expected, curOffset_);
            replyStatus(src, catj::FC_INVALIDOFFSET, catj::M_FILE, expected);
        } else if (offset != curOffset_) {
            std::vector<uint8_t> expected; catj::putBe32(expected, curOffset_);
            replyStatus(src, catj::FC_INVALIDOFFSET, catj::M_FILE, expected);
        } else if (armLen_ != 0 && dlen != armLen_) {
            replyStatus(src, catj::FC_BADFILEDATA, catj::M_FILE, {});           // must be exactly DataLength
        } else {
            flashCrc_ = crc32_update(flashCrc_, d, dlen);
            flashBuf_.insert(flashBuf_.end(), d, d + dlen);
            curOffset_ += (uint32_t)dlen;
            std::vector<uint8_t> nextOff; catj::putBe32(nextOff, curOffset_);    // Complete + next offset
            replyStatus(src, catj::FC_COMPLETE, catj::M_FILE, nextOff);
        }
        break;
    }
    default:
        break;   // unknown CAT-ext PID — ignored
    }
}

// ---- CAT access-level (password) over DM14/15/16 (ECM side; RE/04 §3.2) -------------------
void EcmSim::handleDm14(const J1939Msg& m)
{
    uint16_t length = 0, key = 0; uint8_t command = 0; uint32_t pointer = 0;
    if (!catsec::parseDm14(m.data, length, command, pointer, key)) return;
    memCommand_ = command;
    memPointer_ = pointer;

    // Reply DM15 Proceed with an unsecured seed (0xFFFF) — the simulator never secures these
    // regions (RE/04 §8 Q3). A real ECM may return a real seed; that is the verify-on-iron gap.
    J1939Msg r; r.pgn = catsec::PGN_DM15; r.priority = 6; r.src = ECM_SA; r.dst = m.src;
    r.data = catsec::buildDm15(length ? length : 8, catsec::ST_PROCEED, catsec::SEED_UNSECURED);
    enqueue(r);

    // A read: answer with a DM16 carrying the requested value (current level, or a stored config
    // parameter), then a closing DM15 OperationCompleted.
    if (command == catsec::CMD_READ) {
        std::vector<uint8_t> value; bool known = true;
        if (catsec::pointerMatches(pointer, catsec::MAP_LEVEL_READ)) {
            value = { (uint8_t)secLevel_ };
        } else {
            auto it = params_.find(pointer & catsec::PTR_MASK);
            if (it != params_.end()) value = it->second; else known = false;
        }
        if (known) {
            J1939Msg d; d.pgn = catsec::PGN_DM16; d.priority = 6; d.src = ECM_SA; d.dst = m.src;
            d.data = catsec::buildDm16(value);
            enqueue(d);
            J1939Msg done; done.pgn = catsec::PGN_DM15; done.priority = 6; done.src = ECM_SA; done.dst = m.src;
            done.data = catsec::buildDm15((uint16_t)value.size(), catsec::ST_OP_DONE, catsec::SEED_UNSECURED);
            enqueue(done);
        }
    }
}

void EcmSim::handleDm16(const J1939Msg& m)
{
    if (m.data.empty()) return;
    int n = m.data[0]; if (n > (int)m.data.size() - 1) n = (int)m.data.size() - 1;
    std::vector<uint8_t> value(m.data.begin() + 1, m.data.begin() + 1 + n);

    uint8_t status = catsec::ST_OP_DONE;   // becomes OP_FAILED if the ECM refuses the write

    // Password-write pointers carry security material; everything else is a config parameter.
    if (memCommand_ == catsec::CMD_WRITE && catsec::pointerMatches(memPointer_, catsec::MAP_PASSWORD_1)) {
        secPw1_ = catsec::padPassword(value.data(), (int)value.size());
    } else if (memCommand_ == catsec::CMD_WRITE && catsec::pointerMatches(memPointer_, catsec::MAP_PASSWORD_2)) {
        secPw2_ = catsec::padPassword(value.data(), (int)value.size());
        evaluateSecurity();   // the ECM evaluates on receipt of password 2 (RE/04 §3.2)
    } else if (memCommand_ == catsec::CMD_WRITE && catsec::pointerMatches(memPointer_, catsec::MAP_CUST_PW_1)) {
        custPw1_ = catsec::padPassword(value.data(), (int)value.size());
    } else if (memCommand_ == catsec::CMD_WRITE && catsec::pointerMatches(memPointer_, catsec::MAP_CUST_PW_2)) {
        custPw2_ = catsec::padPassword(value.data(), (int)value.size());
    } else if (memCommand_ == catsec::CMD_WRITE) {
        // Config parameter write: the ECM enforces the required level (RE/04 §3.5). Below it, refuse
        // with OperationFailed and leave the stored value untouched — exactly what a real ECM does.
        uint32_t key = memPointer_ & catsec::PTR_MASK;
        auto lv = paramWriteLevel_.find(key);
        int required = (lv != paramWriteLevel_.end()) ? lv->second : catsec::LVL_FACTORY;
        if (secLevel_ >= required) params_[key] = value;
        else status = catsec::ST_OP_FAILED;
    }

    // Closing DM15 (OperationCompleted, or OperationFailed if the write was refused).
    J1939Msg done; done.pgn = catsec::PGN_DM15; done.priority = 6; done.src = ECM_SA; done.dst = m.src;
    done.data = catsec::buildDm15((uint16_t)value.size(), status, catsec::SEED_UNSECURED);
    enqueue(done);
}

// Emulator test parameters: (24-bit pointer -> value) with a required write level. No real rating /
// FLS / FTS / immobiliser PIDs are modelled (RE/04 §7.1). Pointers are in the CAT map range.
void EcmSim::seedConfigParams()
{
    params_.clear(); paramWriteLevel_.clear();
    auto put = [&](uint32_t ptr, std::vector<uint8_t> v, int wl) {
        uint32_t k = ptr & catsec::PTR_MASK; params_[k] = std::move(v); paramWriteLevel_[k] = wl;
    };
    put(0xFFFF10, { 0x00, 0x07 }, catsec::LVL_CUSTOMER);      // "Equipment ID / unit number" (u16 = 7)
    put(0xFFFF11, { 0x02, 0xEE }, catsec::LVL_SERVICE_TOOL);  // "Low idle rpm" (u16 = 750)
    put(0xFFFF12, { 0x01 },       catsec::LVL_FACTORY);       // "Parameter lockout flag" (factory-gated)
}

void EcmSim::evaluateSecurity()
{
    // Byte-compare ladder (RE/04 §3.3), WITHOUT the simulator's permissive "any non-blank pair ->
    // Factory" rule 4 — that is not how a real ECM behaves.
    auto eq = [](const std::vector<uint8_t>& a, const uint8_t* b) {
        if (a.size() != (size_t)catsec::PW_WIDTH) return false;
        for (int i = 0; i < catsec::PW_WIDTH; ++i) if (a[(size_t)i] != b[i]) return false;
        return true;
    };
    if (eq(secPw1_, catsec::SERVICE_TOOL_PW1) && eq(secPw2_, catsec::SERVICE_TOOL_PW2)) {
        secLevel_ = catsec::LVL_SERVICE_TOOL;
    } else if (!custPw1_.empty() && secPw1_ == custPw1_ && secPw2_ == custPw2_) {
        secLevel_ = catsec::LVL_CUSTOMER;
    } else if (eq(secPw1_, catsec::TRAINER_FACTORY_PW1) && eq(secPw2_, catsec::TRAINER_FACTORY_PW2)) {
        // TRAINER/SIMULATOR ONLY (RE/04 §3.3): CAT's trainer factory pair. A real ECM rejects it and
        // needs a Factory Password System credential; the sim honours it so factory edits can be
        // rehearsed end to end.
        secLevel_ = catsec::LVL_FACTORY;
    } else {
        secLevel_ = catsec::LVL_NORMAL;   // no match; real factory pairs are not known to the sim
    }
}

void EcmSim::handleTpCm(const J1939Msg& m)
{

    if (m.data.size() < 8 || m.data[0] != 0x10) return;   // only RTS (connection mode) handled
    tpActive_ = true;
    tpSrc_ = m.src;
    tpTotal_ = (uint16_t)(m.data[1] | (m.data[2] << 8));
    uint8_t packets = m.data[3];
    tpPgn_ = (uint32_t)m.data[5] | ((uint32_t)m.data[6] << 8) | ((uint32_t)m.data[7] << 16);
    tpBuf_.clear();

    // CTS (control 0x11): clear all packets starting at seq 1.
    J1939Msg cts; cts.pgn = PGN_TP_CM; cts.priority = 7; cts.src = ECM_SA; cts.dst = tpSrc_;
    cts.data = { 0x11, packets, 0x01, 0xFF, 0xFF,
                 (uint8_t)(tpPgn_ & 0xFF), (uint8_t)((tpPgn_ >> 8) & 0xFF), (uint8_t)((tpPgn_ >> 16) & 0xFF) };
    enqueue(cts);
}

void EcmSim::handleTpDt(const J1939Msg& m)
{
    if (!tpActive_ || m.src != tpSrc_ || m.data.size() < 2) return;
    for (size_t i = 1; i < m.data.size(); ++i) tpBuf_.push_back(m.data[i]);   // drop seq byte

    if (tpBuf_.size() >= tpTotal_) {
        tpBuf_.resize(tpTotal_);
        // EndOfMsgAck (control 0x13) — sent first so the tester's sendTp() returns before the reply.
        J1939Msg eom; eom.pgn = PGN_TP_CM; eom.priority = 7; eom.src = ECM_SA; eom.dst = tpSrc_;
        eom.data = { 0x13, (uint8_t)(tpTotal_ & 0xFF), (uint8_t)(tpTotal_ >> 8), (uint8_t)((tpTotal_ + 6) / 7), 0xFF,
                     (uint8_t)(tpPgn_ & 0xFF), (uint8_t)((tpPgn_ >> 8) & 0xFF), (uint8_t)((tpPgn_ >> 16) & 0xFF) };
        enqueue(eom);
        uint32_t pgn = tpPgn_;
        uint8_t  src = tpSrc_;
        std::vector<uint8_t> payload = tpBuf_;
        tpActive_ = false;
        // A CAT-ext flash message (0xF001 control / 0xF002 data) transported under PGN 0xEF00:
        // process the reassembled payload now; its status reply is enqueued after the EOM.
        if (pgn == PGN_PROPRIETARY_A) handleCatExt(src, payload);
    }
}

bool EcmSim::poll(J1939Msg& out)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (!running_) return false;
    checkKillTimer();
    if (!inFlash_) serviceBus(now_ms());   // the whole bus goes quiet while the engine is flashing
    if (outq_.empty()) return false;
    out = outq_.front();
    outq_.pop_front();
    return true;
}

} // namespace tpemu
