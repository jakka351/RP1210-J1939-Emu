/* =============================================================================================
 *  ecm_sim.h  —  an independent, clean-room SAE J1939 BUS simulator for the "Tester Present RP1210"
 *  emulator.
 *
 *  Models a realistic heavy-diesel powertrain/chassis J1939 bus: a Caterpillar/Perkins ADEM engine
 *  controller (SA 0x00, which also implements the ADEM flash-reprogramming protocol) plus several
 *  other spec'd J1939-71 controllers — transmission, brakes/ABS, instrument cluster and an
 *  aftertreatment/emissions controller — each claiming its own address, broadcasting its spec'd
 *  PGNs on their normal periods, and answering PGN requests.
 *
 *  This is OUR OWN model built from the public/documented SAE J1939 protocol and the project's
 *  clean-room notes — no Caterpillar code, no proprietary data; identity strings and NAMEs are
 *  generic simulated values. It exists so the ADEM Flash Programming Tool (and any J1939 scan /
 *  diagnostic feature) can be exercised end-to-end without real hardware.
 *
 *  Everything works purely in J1939 terms (PGN / priority / source / destination / data); the
 *  RP1210 shim above it handles the wire framing.
 * ============================================================================================= */
#pragma once
#include <cstdint>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <string>
#include <utility>
#include "../tp.adem.core/src/j1939_diag.h"   // shared SAE J1939-73 diagnostic model (DTCs/DM services)

namespace tpemu {

struct J1939Msg {
    uint32_t pgn = 0;
    uint8_t  priority = 6;
    uint8_t  src = 0;
    uint8_t  dst = 0xFF;
    std::vector<uint8_t> data;
};

/* One periodic broadcast a controller emits (static snapshot payload — the sim runs at a fixed
 * operating point, so the signal values do not change). */
struct Periodic {
    uint32_t pgn = 0;
    uint8_t  priority = 6;
    unsigned long long periodMs = 1000;
    std::vector<uint8_t> data;
    unsigned long long last = 0;
};

/* A generic, data-driven J1939 controller node: a NAME + address, a set of periodic broadcasts,
 * and a set of PGN-request responses (single-frame or multi-packet BAM). The ADEM engine is
 * modelled directly by EcmSim (it carries the flash state machine); these nodes are the rest of
 * the bus. */
class BusNode {
public:
    BusNode(uint8_t sa, uint8_t function, uint16_t identity, std::string label);

    uint8_t     sa() const { return sa_; }
    const std::string& label() const { return label_; }

    void addPeriodic(uint32_t pgn, uint8_t priority, unsigned long long periodMs, std::vector<uint8_t> data);
    void addRequestResponse(uint32_t pgn, std::vector<uint8_t> payload, bool bam);

    // SAE J1939-73 diagnostics for this controller (DTCs, lamps, readiness, tests). `dmCapable`
    // nodes broadcast DM1 and answer DM1/DM2/DM5/DM4 requests and DM3/DM11/DM7 commands.
    j1939diag::DiagState diag;
    bool dmCapable = false;
    unsigned long long dm1Last = 0;

    J1939Msg addressClaimed() const;                               // PGN 0xEE00 with this node's NAME
    void service(unsigned long long now, std::deque<J1939Msg>& out);  // enqueue due periodics
    void respond(uint32_t reqPgn, std::deque<J1939Msg>& out) const;   // answer a PGN request

private:
    uint8_t  sa_;
    uint8_t  name_[8];
    std::string label_;
    std::vector<Periodic> periodics_;
    struct Resp { uint32_t pgn; std::vector<uint8_t> payload; bool bam; };
    std::vector<Resp> responses_;
};

class EcmSim {
public:
    EcmSim() = default;

    void start();                       // begin simulating (on client connect)
    void stop();                        // stop and flush
    bool running() const { return running_; }

    void onSend(const J1939Msg& m);     // a frame FROM the tool TO the bus
    bool poll(J1939Msg& out);           // next frame FROM the bus TO the tool (false if none yet)

    // ---- management / configuration control (for the emulator GUI console) ----
    // Thread-safe live control of the running bus: enumerate controllers, inject/clear faults,
    // read/set the VIN, drive flash fault injection, query flash state.
    int  nodeCount();                                      // engine + peripheral controllers
    bool nodeInfo(int index, uint8_t& sa, std::string& label, int& active, int& stored, uint8_t& lamp);
    void injectDtc(uint8_t sa, uint32_t spn, uint8_t fmi, uint8_t oc);   // add an active DTC to a module
    void clearDtcsFor(uint8_t sa, int which);              // 0=active, 1=stored, 2=both
    std::string vin();
    void setVin(const std::string& v);
    void setFaultByName(const char* name);                 // none|inappropriate|invalidoffset|badchecksum|erasefail|stuck
    bool flashActive();                                    // true while an ECM flash is in progress

private:
    void enqueue(const J1939Msg& m);
    void buildBus();                    // construct the engine periodics + peripheral controllers
    void serviceBus(unsigned long long now);   // enqueue all due periodic frames (engine + nodes)
    void respondToRequest(uint32_t reqPgn, uint8_t dst);   // engine + node request responses
    void sendBam(uint32_t pgn, const std::vector<uint8_t>& payload);  // BAM from the engine (SA 0x00)

    // ---- SAE J1939-73 diagnostic stack (engine + dm-capable nodes) ----
    void seedDiagnostics();             // inject the default/ENV faults, readiness, test suites
    void serviceDiag(unsigned long long now);                  // emit DM1 broadcasts (dynamic payload)
    bool handleDiagRequest(uint32_t reqPgn, uint8_t dst);      // DM1/DM2/DM5/DM4 on request (true if handled)
    bool handleDiagCommand(const J1939Msg& m);                 // DM11/DM3 clear, DM7->DM30 (true if handled)
    void emitDm(uint8_t sa, uint32_t pgn, const std::vector<uint8_t>& payload, uint8_t dst = 0xFF);
    void sendAck(uint8_t sa, uint8_t toSa, uint32_t ackedPgn, uint8_t control);  // ACKM 0xE800
    // Engine + every dm-capable node, as (SA, diag) pairs, for uniform DM handling.
    std::vector<std::pair<uint8_t, j1939diag::DiagState*>> diagModules();

    j1939diag::DiagState engineDiag_;   // the ADEM engine's own diagnostics (SA 0x00)
    unsigned long long   engineDm1Last_ = 0;

    // Real ADEM J1939 CAT-ext flash protocol (ECM side; see tp.adem.core/src/cat_flash_j1939.h)
    // + connection-mode transport receive.
    void handleTpCm(const J1939Msg& m);
    void handleTpDt(const J1939Msg& m);
    void handleCatExt(uint8_t src, const std::vector<uint8_t>& payload);  // a CAT-ext msg on 0xEF00
    void sendCurrentStatus(uint8_t src);                                  // reply to a PID 0x88 read
    void replyStatus(uint8_t src, uint8_t cmd, uint8_t mode, const std::vector<uint8_t>& additional);
    void beginFlash();                 // reset state + enter flash mode (bus-quiet + DM13 sleep)
    void checkKillTimer();             // drop flash mode after the idle kill timeout
    bool parseFileInfo(const uint8_t* p, size_t n);   // decode the F003 TLV; false = BadFileInformation
    J1939Msg dm13Sleep() const;        // DM13 bus-quiet broadcast the ECM emits on entering flash

    bool     inFlash_ = false;
    uint8_t  flashMode_ = 0;            // catj::FlashMode
    uint32_t curOffset_ = 0;           // ECM running write offset
    uint16_t armLen_ = 0;              // DataLength armed by the last 0xF001
    uint32_t storedChecksum_ = 0;      // checksum the tester supplied at FileDone
    unsigned long long burnUntilMs_ = 0;
    unsigned long long lastFlashMs_ = 0;   // last accepted flash message, for the session kill timer
    std::string        swPartNumber_;      // software part # parsed from the F003 file-info TLV
    std::vector<uint8_t> flashBuf_;
    uint32_t flashCrc_ = 0xFFFFFFFFu;
    std::string        vin_ = "1CATADEM4FLS00001";   // Vehicle Identification Number (PGN 0xFEEC), configurable

    static constexpr unsigned long long FLASH_KILL_MS = 10000;   // J1939 flash session idle kill (map §3.7)

    // Fault injection (env TPRP_EMU_FAULT) to exercise the tool's recovery / verify / precondition
    // paths against a faithful ECM: inappropriate | invalidoffset | badchecksum | erasefail | stuck.
    enum class Fault { None, Inappropriate, InvalidOffset, BadChecksum, EraseFail, Stuck };
    Fault fault_ = Fault::None;
    bool  faultArmed_ = false;             // one-shot faults (fired once, then cleared)

    bool     tpActive_ = false;
    uint8_t  tpSrc_ = 0;
    uint16_t tpTotal_ = 0;
    uint32_t tpPgn_ = 0;
    std::vector<uint8_t> tpBuf_;

    J1939Msg addressClaimed() const;    // PGN 0xEE00 with the engine ECM NAME

    std::vector<BusNode> nodes_;        // the other controllers on the bus
    std::vector<Periodic> enginePeriodics_;  // the engine's own spec'd broadcasts

    std::mutex mu_;
    std::deque<J1939Msg> outq_;
    std::atomic<bool> running_{ false };   // atomic: read lock-free by running()/emu_running (GUI thread)

    // Bounded receive FIFO depth (frames). The bus runs continuously for as long as a client is
    // connected; this caps memory if the client polls slowly (oldest frames drop, like real iron).
    static constexpr size_t MAX_QUEUE = 16384;

    static constexpr uint8_t ECM_SA = 0x00;   // engine #1
    uint8_t toolSa_ = 0xF9;                    // offboard service tool

    unsigned long long startMs_ = 0;
    unsigned long long lastClaim_ = 0;
};

} // namespace tpemu
