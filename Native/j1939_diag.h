/* =============================================================================================
 *  j1939_diag.h  —  SAE J1939-73 diagnostic model (DTCs, lamps, readiness, tests): the SINGLE
 *  SOURCE OF TRUTH shared by the native core (diagnostic services) and the RP1210 ECM emulator
 *  (diagnostic responder), so the two sides can never drift — same pattern as cat_flash_j1939.h.
 *
 *  Covers the diagnostic message (DM) set the tool uses:
 *    DM1  (0xFECA) Active DTCs                — broadcast + on request
 *    DM2  (0xFECB) Previously Active DTCs     — on request
 *    DM3  (0xFECC) Clear Previously Active    — command (+ ACK)
 *    DM11 (0xFED3) Clear Active               — command (+ ACK)
 *    DM4  (0xFECD) Freeze Frame               — on request
 *    DM5  (0xFECE) Diagnostic Readiness 1     — on request
 *    DM7  (0xE300) Command Non-cont. Test     — destination-specific command
 *    DM30 (0xA400) Scaled Test Results        — destination-specific response to DM7
 *    ACKM (0xE800) Acknowledgment             — positive/negative ack of a command
 *
 *  A DTC is the classic J1939-73 4-byte structure (SPN 19-bit, FMI 5-bit, SPN Conversion Method
 *  1-bit, Occurrence Count 7-bit) in the current ("CM=0") SPN layout.
 * ============================================================================================= */
#pragma once
#include <cstdint>
#include <vector>

namespace j1939diag {

enum : uint32_t {
    PGN_DM1  = 0x00FECA,
    PGN_DM2  = 0x00FECB,
    PGN_DM3  = 0x00FECC,
    PGN_DM4  = 0x00FECD,
    PGN_DM5  = 0x00FECE,
    PGN_DM11 = 0x00FED3,
    PGN_DM7  = 0x00E300,
    PGN_DM8  = 0x00FED0,
    PGN_DM30 = 0x00A400,
    PGN_ACKM = 0x00E800,
};

/* Lamp status: two bits per lamp in DM1/DM2 byte 0 (00=off, 01=on). byte 1 is the matching
 * flash status (00 = not flashing). SAE J1939-73 bit layout: MIL in bits 7-6 (SPN 1213), Red Stop
 * in bits 5-4 (SPN 623), Amber Warning in bits 3-2 (SPN 624), Protect in bits 1-0 (SPN 987). */
enum : int {
    LAMP_PROTECT = 0,   // protect             (bits 1-0)
    LAMP_AMBER   = 2,    // amber warning       (bits 3-2)
    LAMP_RED     = 4,    // red stop            (bits 5-4)
    LAMP_MIL     = 6,    // malfunction indicator (bits 7-6)
};
inline uint8_t lampBits(bool mil, bool red, bool amber, bool protect) {
    return (uint8_t)((protect ? 1 : 0) | (amber ? 1 << 2 : 0) | (red ? 1 << 4 : 0) | (mil ? 1 << 6 : 0));
}

struct Dtc {
    uint32_t spn = 0;    // suspect parameter number (19-bit)
    uint8_t  fmi = 0;    // failure mode indicator (5-bit)
    uint8_t  cm  = 0;    // SPN conversion method (1-bit); 0 = current layout
    uint8_t  oc  = 1;    // occurrence count (7-bit)
};

/* One scaled test result (DM30 entry, in response to a DM7 command). */
struct TestResult {
    uint8_t  tid = 0;       // test identifier
    uint32_t spn = 0;
    uint8_t  fmi = 0;
    uint16_t value = 0;     // measured
    uint16_t min = 0;       // acceptable minimum
    uint16_t max = 0;       // acceptable maximum
    bool pass() const { return value >= min && value <= max; }
};

/* Pack one DTC into its 4 J1939-73 bytes (CM=0 / current SPN layout). */
inline void packDtc(const Dtc& d, std::vector<uint8_t>& out) {
    out.push_back((uint8_t)(d.spn & 0xFF));
    out.push_back((uint8_t)((d.spn >> 8) & 0xFF));
    out.push_back((uint8_t)((((d.spn >> 16) & 0x07) << 5) | (d.fmi & 0x1F)));
    out.push_back((uint8_t)(((d.cm & 0x01) << 7) | (d.oc & 0x7F)));
}
inline Dtc unpackDtc(const uint8_t* p) {
    Dtc d;
    d.spn = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)(p[2] >> 5) << 16);
    d.fmi = (uint8_t)(p[2] & 0x1F);
    d.cm  = (uint8_t)((p[3] >> 7) & 0x01);
    d.oc  = (uint8_t)(p[3] & 0x7F);
    return d;
}

/* Per-ECU diagnostic state (held by the engine and by each emulated controller). */
struct DiagState {
    std::vector<Dtc> active;         // reported by DM1
    std::vector<Dtc> stored;         // reported by DM2 (previously active)
    uint8_t lamp = 0;                // DM1/DM2 lamp status
    uint8_t flash = 0;               // DM1/DM2 flash lamp status
    uint16_t readinessSupported = 0; // DM5 supported monitor bitfield (simplified)
    uint16_t readinessComplete  = 0; // DM5 complete monitor bitfield
    std::vector<TestResult> tests;   // DM7/DM30 component tests

    void recomputeLamps() {
        // Amber if any active DTC; red if any active FMI indicates a severe (0/1/12) failure mode.
        bool amber = !active.empty(), red = false;
        for (auto& d : active) if (d.fmi == 0 || d.fmi == 1 || d.fmi == 12) red = true;
        lamp = lampBits(false, red, amber, false);
        flash = 0;
    }
    void clearActive() { active.clear(); recomputeLamps(); }      // DM11
    void clearStored() { stored.clear(); }                        // DM3
};

/* DM1/DM2 payload: [lamp][flash] then the packed DTCs (a no-fault frame carries one 00 00 00 00
 * entry with lamps off, per the convention). Length > 8 is carried by the caller as a BAM. */
inline std::vector<uint8_t> buildDmList(uint8_t lamp, uint8_t flash, const std::vector<Dtc>& dtcs) {
    std::vector<uint8_t> out; out.push_back(lamp); out.push_back(flash);
    if (dtcs.empty()) { out.push_back(0x00); out.push_back(0x00); out.push_back(0x00); out.push_back(0x00); }
    else for (const auto& d : dtcs) packDtc(d, out);
    while (out.size() < 8) out.push_back(0xFF);   // pad a single-frame list to 8 bytes
    return out;
}

/* DM5 Diagnostic Readiness 1: [active count][previously-active count][OBD compliance]
 * [continuously-monitored support/status][non-continuous support][non-continuous status]. */
inline std::vector<uint8_t> buildDm5(const DiagState& s) {
    uint8_t act = (uint8_t)(s.active.size() > 255 ? 255 : s.active.size());
    uint8_t prev = (uint8_t)(s.stored.size() > 255 ? 255 : s.stored.size());
    return { act, prev, 0x05 /*OBD: HD OBD*/,
             (uint8_t)(s.readinessSupported & 0xFF), (uint8_t)(s.readinessSupported >> 8),
             (uint8_t)(s.readinessComplete & 0xFF), (uint8_t)(s.readinessComplete >> 8), 0xFF };
}

/* DM30 scaled test results for a DM7 command: a sequence of 10-byte records
 * [TID][SPN:2 LE][FMI][value:2 LE][min:2 LE][max:2 LE]. */
inline std::vector<uint8_t> buildDm30(const std::vector<TestResult>& tests) {
    std::vector<uint8_t> out;
    for (const auto& t : tests) {
        out.push_back(t.tid);
        out.push_back((uint8_t)(t.spn & 0xFF));
        out.push_back((uint8_t)((t.spn >> 8) & 0xFF));
        out.push_back(t.fmi);
        out.push_back((uint8_t)(t.value & 0xFF)); out.push_back((uint8_t)(t.value >> 8));
        out.push_back((uint8_t)(t.min & 0xFF));   out.push_back((uint8_t)(t.min >> 8));
        out.push_back((uint8_t)(t.max & 0xFF));   out.push_back((uint8_t)(t.max >> 8));
    }
    if (out.empty()) out.push_back(0xFF);
    return out;
}
static constexpr size_t DM30_RECORD = 10;

} // namespace j1939diag
