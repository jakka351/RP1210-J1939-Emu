/* =============================================================================================
 *  cat_flash_j1939.h  —  Caterpillar/Perkins ADEM J1939 flash-reprogramming wire protocol.
 *
 *  SINGLE SOURCE OF TRUTH for the CAT-extended J1939 flash protocol, shared by the flash engine
 *  (tp.adem.core) and the ECM emulator (tp.rp1210.emu) so the two can never drift. Reverse-
 *  engineered from the decompiled CAT ET ECM simulator and cross-checked against the RE notes;
 *  see docs/ADEM_FLASH_PROTOCOL_MAP.md for the full derivation, trust levels and citations.
 *
 *  Wire model (HIGH trust — the decompiled message triggers/parsers, i.e. what real CAT ET emits):
 *    - All flash traffic rides PGN 0xEF00 (Proprietary-A, destination-specific).
 *    - 29-bit id = (6<<26) | (0xEF<<16) | (dest<<8) | src.
 *    - Payload = [CommandByte][CAT-ext PID][data...]
 *        CommandByte: 0x80 ProprietaryRead, 0x81 WriteWithData, 0x82 WriteWithAck,
 *                     0x83 WriteResponse (ECM -> tester).
 *        CAT-ext PID: 1 byte, OR 2 bytes big-endian when the leading byte is 0xF0..0xFF
 *                     (so 0x88 = 1 byte; 0xF001/0xF002/0xF003 = 2 bytes).
 *    - Status word on PID 0x88 is a 16-bit BIG-ENDIAN value = ((Command<<4) | Mode); decode as
 *        Command = (b0<<4)|(b1>>4), Mode = b1 & 0x0F.   (For the command range used here b0==0x00.)
 *    - All offsets/lengths/checksums are BIG-ENDIAN.
 *    - Messages > 8 bytes (flash control, flash data) ride J1939 Transport Protocol under 0xEF00.
 *
 *  MEDIUM trust / verify-on-iron items (see docs §7) are flagged at each use site, NOT hidden here.
 * ============================================================================================= */
#pragma once
#include <cstdint>
#include <vector>

namespace catj {

/* PGNs */
constexpr uint32_t PGN_PROPRIETARY_A = 0x00EF00;   /* all CAT flash command/data/status */
constexpr uint32_t PGN_DM13          = 0x00DF00;   /* bus-quiet / sleep broadcast        */

/* CommandByte (first payload byte on 0xEF00) */
constexpr uint8_t CMD_READ        = 0x80;   /* ProprietaryRead        (tester -> ECM) */
constexpr uint8_t CMD_WRITE_DATA  = 0x81;   /* ProprietaryWriteWithData               */
constexpr uint8_t CMD_WRITE_ACK   = 0x82;   /* ProprietaryWriteWithAck                */
constexpr uint8_t CMD_WRITE_RESP  = 0x83;   /* ProprietaryWriteResponse (ECM -> tester) */

/* CAT-ext PIDs */
constexpr uint16_t PID_FLASH_STATUS  = 0x0088;   /* status/command word + optional 4-byte data   */
constexpr uint16_t PID_FLASH_CONTROL = 0xF001;   /* region arm: [offset(4 BE)][dataLength(2 BE)]  */
constexpr uint16_t PID_FLASH_DATA    = 0xF002;   /* bulk data : [offset(4 BE)][data...]           */
constexpr uint16_t PID_FLASH_INFO    = 0xF003;   /* file/ECM identity TLV [SubID][Len][Data]...   */

/* Flash command nibble (high nibble of the status word) — J1939FLASHCOMMANDS */
enum FlashCmd : uint8_t {
    FC_OK            = 0x0,
    FC_BUSY          = 0x1,
    FC_CHECKSUM      = 0x2,
    FC_BADFILEINFO   = 0x3,
    FC_ERASEFAIL     = 0x4,
    FC_PROGFAIL      = 0x5,
    FC_INVALIDOFFSET = 0x6,
    FC_PROCEED       = 0x7,
    FC_COMPLETE      = 0x8,
    FC_TERMINATE     = 0x9,
    FC_ABORT         = 0xA,
    FC_INAPPROPRIATE = 0xB,
    FC_BADFILEDATA   = 0xC,
    FC_BADCHECKSUM   = 0xE,
};

/* Flash mode (low nibble of the status word) — J1939FlashCurrentMode */
enum FlashMode : uint8_t {
    M_NOTFLASH  = 0,
    M_INFLASH   = 1,
    M_FILE      = 2,
    M_FILEDONE  = 3,
    M_FILEPROC  = 4,
};

/* TLV SubIDs for the PID 0xF003 file-information block (subset used here). */
enum FlashInfoSubId : uint8_t {
    SUB_SOFTWARE_PART = 0x85,
    SUB_ENCRYPTION_KEY= 0x86,   /* flash-file password/key — semantics UNVERIFIED (docs §7) */
    SUB_TOOL_SERIAL   = 0x88,
};

/* ---- status-word pack/decode ---------------------------------------------------------------- */
inline uint16_t packStatus(uint8_t cmd, uint8_t mode) {
    return (uint16_t)((((uint16_t)(cmd & 0x0F)) << 4) | (mode & 0x0F));
}
inline uint8_t statusCmd(uint8_t b0, uint8_t b1) { return (uint8_t)(((b0 << 4) | (b1 >> 4)) & 0xFF); }
inline uint8_t statusMode(uint8_t b1)            { return (uint8_t)(b1 & 0x0F); }

/* ---- big-endian helpers --------------------------------------------------------------------- */
inline void putBe16(std::vector<uint8_t>& v, uint16_t x) { v.push_back((uint8_t)(x >> 8)); v.push_back((uint8_t)x); }
inline void putBe32(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back((uint8_t)(x >> 24)); v.push_back((uint8_t)(x >> 16));
    v.push_back((uint8_t)(x >> 8));  v.push_back((uint8_t)x);
}
inline uint16_t getBe16(const uint8_t* p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }
inline uint32_t getBe32(const uint8_t* p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/* ---- CAT-ext PID framing -------------------------------------------------------------------- */
inline bool pidIs2Byte(uint16_t pid) { return (uint8_t)(pid >> 8) >= 0xF0; }

inline void appendPid(std::vector<uint8_t>& v, uint16_t pid) {
    if (pidIs2Byte(pid)) { v.push_back((uint8_t)(pid >> 8)); v.push_back((uint8_t)(pid & 0xFF)); }
    else                 { v.push_back((uint8_t)(pid & 0xFF)); }
}

/* Build a CAT-ext message [CommandByte][PID][postpid...] on PGN 0xEF00. */
inline std::vector<uint8_t> buildMsg(uint8_t cmdByte, uint16_t pid, const std::vector<uint8_t>& postpid) {
    std::vector<uint8_t> m;
    m.push_back(cmdByte);
    appendPid(m, pid);
    m.insert(m.end(), postpid.begin(), postpid.end());
    return m;
}

/* Parse the [CommandByte][PID] header of a CAT-ext payload. Returns bytes consumed (header size),
 * or 0 if malformed. On success sets cmdByte, pid. */
inline size_t parseHeader(const std::vector<uint8_t>& d, uint8_t& cmdByte, uint16_t& pid) {
    if (d.size() < 2) return 0;
    cmdByte = d[0];
    uint8_t p0 = d[1];
    if (p0 >= 0xF0) {
        if (d.size() < 3) return 0;
        pid = (uint16_t)(((uint16_t)p0 << 8) | d[2]);
        return 3;
    }
    pid = (uint16_t)p0;
    return 2;
}

} // namespace catj
