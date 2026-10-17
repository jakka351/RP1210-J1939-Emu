/* =============================================================================================
 *  ecm_sim.cpp  —  the simulated ADEM ECM behaviour.
 * ============================================================================================= */
#include "ecm_sim.h"
#include "../tp.adem.core/src/cat_flash_j1939.h"   // single source of truth for the CAT-ext protocol
#include <windows.h>
#include <string>
#include <cstring>

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

    // Optional fault injection (env TPRP_EMU_FAULT) so the tool's recovery/verify/precondition
    // paths can be exercised against a faithful ECM.
    fault_ = Fault::None; faultArmed_ = false;
    char fb[32] = { 0 };
    if (GetEnvironmentVariableA("TPRP_EMU_FAULT", fb, (DWORD)sizeof(fb)) > 0) {
        if      (_stricmp(fb, "inappropriate") == 0) fault_ = Fault::Inappropriate;
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
    uint16_t total = (uint16_t)payload.size();
    uint8_t packets = (uint8_t)((total + 6) / 7);
    J1939Msg cm; cm.pgn = 0x00EC00; cm.priority = 7; cm.src = sa; cm.dst = 0xFF;
    cm.data = { 0x20, (uint8_t)(total & 0xFF), (uint8_t)(total >> 8), packets, 0xFF,
                (uint8_t)(pgn & 0xFF), (uint8_t)((pgn >> 8) & 0xFF), (uint8_t)((pgn >> 16) & 0xFF) };
    out.push_back(cm);
    for (uint8_t i = 0; i < packets; ++i) {
        J1939Msg dt; dt.pgn = 0x00EB00; dt.priority = 7; dt.src = sa; dt.dst = 0xFF;
        dt.data.push_back((uint8_t)(i + 1));
        for (int j = 0; j < 7; ++j) { size_t idx = (size_t)i * 7 + j; dt.data.push_back(idx < payload.size() ? payload[idx] : 0xFF); }
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

void BusNode::addRequestResponse(uint32_t pgn, std::vector<uint8_t> payload, bool bam)
{
    responses_.push_back(Resp{ pgn, std::move(payload), bam });
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

void BusNode::respond(uint32_t reqPgn, std::deque<J1939Msg>& out) const
{
    for (auto& r : responses_) {
        if (r.pgn != reqPgn) continue;
        if (r.bam) { for (auto& f : buildBam(sa_, r.pgn, r.payload)) out.push_back(f); }
        else { J1939Msg m; m.pgn = r.pgn; m.priority = 6; m.src = sa_; m.dst = 0xFF; m.data = r.payload; out.push_back(m); }
    }
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

    // Implement / Hydraulics controller (SA 0x25) — mobile-plant work hydraulics (proprietary-B)
    { BusNode n(0x25, 0x23, 0x0025, "Implement/Hydraulics");
      n.addPeriodic(0x00FF20, 6, 500, { 0x20,0x4E,0x10,0x27,0xFF,0xFF,0xFF,0xFF });   // PropB simulated pump/implement pressures
      n.addRequestResponse(0x00FEEB, ascii("CATERPILLAR*HYD*SIM*0025*"), true);
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
                std::vector<uint8_t> p; p.push_back(0x01);
                auto s = ascii("ADEM4 2660376-00*"); p.insert(p.end(), s.begin(), s.end());
                sendBam(PGN_SOFTWARE_ID, p);
                break;
            }
            case PGN_COMPONENT_ID: sendBam(PGN_COMPONENT_ID, ascii("CATERPILLAR*ADEM4*SIM*0001*")); break;
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
    for (auto& n : nodes_) if (dst == 0xFF || dst == n.sa()) n.respond(reqPgn, outq_);
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
    else if (_stricmp(name, "invalidoffset") == 0) { fault_ = Fault::InvalidOffset; faultArmed_ = true; }
    else if (_stricmp(name, "badchecksum")   == 0) fault_ = Fault::BadChecksum;
    else if (_stricmp(name, "erasefail")     == 0) fault_ = Fault::EraseFail;
    else if (_stricmp(name, "stuck")         == 0) fault_ = Fault::Stuck;
}

bool EcmSim::flashActive() { std::lock_guard<std::mutex> lk(mu_); return inFlash_; }

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
