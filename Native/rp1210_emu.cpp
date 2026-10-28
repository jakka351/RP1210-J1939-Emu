/* =============================================================================================
 *  rp1210_emu.cpp  —  "Tester Present RP1210": a TMC RP1210 vendor DLL (TPRP32.dll) that presents
 *  the simulated ADEM ECM (ecm_sim) as a standard RP1210 adapter so the ADEM Flash Programming
 *  Tool can enumerate and connect to it exactly as it would a real Caterpillar-family adapter.
 *
 *  Implements the RP1210 entry points the tool uses (and a few common extras). J1939 wire framing
 *  follows the RP1210 convention:
 *      ClientMessage (SendMessage): [PGN:3 LE][priority:1][source:1][destination:1][data:n]
 *      APIMessage    (ReadMessage): [timestamp:4 BE][PGN:3 LE][priority:1][source:1][destination:1][data:n]
 * ============================================================================================= */
#include <windows.h>
#include <cstring>
#include "ecm_sim.h"

using namespace tpemu;

// RP1210 return / error codes (subset) — the TMC RP1210 standard values. ClientConnect returns
// 0..127 = client id or 128..255 = error; every other entry point returns 0 on success and the
// positive error code otherwise, EXCEPT ReadMessage, which returns a byte count (> 0), 0 for
// "nothing available", or the NEGATIVE error code (so a host can never mistake an error for data).
enum {
    RP1210_NO_ERROR             = 0,
    ERR_INVALID_CLIENT_ID       = 129,
    ERR_CLIENT_AREA_FULL        = 131,
    ERR_INVALID_PROTOCOL        = 136,
    ERR_COMMAND_NOT_SUPPORTED   = 143,
    ERR_MESSAGE_NOT_SENT        = 152,
};

// RP1210_SendCommand numbers handled here.
enum {
    RP1210_Set_All_Filters_States_to_Pass = 3,
    RP1210_Protect_J1939_Address          = 19,
};

static EcmSim  g_sim;
static short   g_client = -1;      // single client supported
static bool    g_connected = false;

static bool protocolSupported(const char* p)
{
    if (!p) return false;
    // Accept "J1939", "J1939:Baud=250", "CAT", "J1708" (case-insensitive prefix).
    return (_strnicmp(p, "J1939", 5) == 0) || (_strnicmp(p, "CAT", 3) == 0) || (_strnicmp(p, "J1708", 5) == 0);
}

extern "C" {

short WINAPI RP1210_ClientConnect(HWND /*hwnd*/, short /*nDeviceID*/, const char* fpchProtocol,
                                  long /*lTx*/, long /*lRx*/, short /*nPacketizing*/)
{
    if (!protocolSupported(fpchProtocol)) return ERR_INVALID_PROTOCOL;
    if (g_connected) return ERR_CLIENT_AREA_FULL;
    g_client = 1;
    g_connected = true;
    g_sim.start();
    return g_client;   // >= 0 => client id (success)
}

short WINAPI RP1210_ClientDisconnect(short nClientID)
{
    if (nClientID != g_client || !g_connected) return ERR_INVALID_CLIENT_ID;
    g_sim.stop();
    g_connected = false;
    g_client = -1;
    return RP1210_NO_ERROR;
}

short WINAPI RP1210_SendMessage(short nClientID, char* fpchClientMessage, short nMessageSize,
                                short /*nNotify*/, short /*nBlock*/)
{
    if (nClientID != g_client || !g_connected) return ERR_INVALID_CLIENT_ID;
    if (!fpchClientMessage || nMessageSize < 6) return ERR_MESSAGE_NOT_SENT;

    const unsigned char* b = (const unsigned char*)fpchClientMessage;
    J1939Msg m;
    m.pgn      = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
    m.priority = b[3];
    m.src      = b[4];
    m.dst      = b[5];
    m.data.assign(b + 6, b + nMessageSize);
    g_sim.onSend(m);
    return RP1210_NO_ERROR;
}

short WINAPI RP1210_ReadMessage(short nClientID, char* fpchAPIMessage, short nBufferSize, short /*nBlock*/)
{
    // Errors from ReadMessage are NEGATIVE per RP1210 (a positive value is a byte count).
    if (nClientID != g_client || !g_connected) return (short)-ERR_INVALID_CLIENT_ID;
    if (!fpchAPIMessage || nBufferSize < 11) return 0;

    J1939Msg m;
    if (!g_sim.poll(m)) return 0;   // nothing ready

    unsigned long ts = GetTickCount();
    unsigned char* o = (unsigned char*)fpchAPIMessage;
    o[0] = (unsigned char)((ts >> 24) & 0xFF);
    o[1] = (unsigned char)((ts >> 16) & 0xFF);
    o[2] = (unsigned char)((ts >> 8) & 0xFF);
    o[3] = (unsigned char)(ts & 0xFF);
    o[4] = (unsigned char)(m.pgn & 0xFF);
    o[5] = (unsigned char)((m.pgn >> 8) & 0xFF);
    o[6] = (unsigned char)((m.pgn >> 16) & 0xFF);
    o[7] = m.priority;
    o[8] = m.src;
    o[9] = m.dst;
    int n = (int)m.data.size();
    if (10 + n > nBufferSize) n = nBufferSize - 10;
    for (int i = 0; i < n; ++i) o[10 + i] = m.data[i];
    return (short)(10 + n);
}

short WINAPI RP1210_SendCommand(short nCmd, short nClientID, char* cmd, short size)
{
    if (nClientID != g_client || !g_connected) return ERR_INVALID_CLIENT_ID;
    switch (nCmd) {
        case RP1210_Protect_J1939_Address:
            // [SA][NAME:8][blocking status]: the adapter claims the address on the client's behalf.
            // The simulated controllers use distinct fixed SAs, so any tool SA is uncontested.
            if (!cmd || size < 10) return ERR_MESSAGE_NOT_SENT;
            g_sim.setToolSa((unsigned char)cmd[0]);
            return RP1210_NO_ERROR;
        case RP1210_Set_All_Filters_States_to_Pass:
        default:
            return RP1210_NO_ERROR;   // accept filter/echo/etc. commands
    }
}

short WINAPI RP1210_ReadVersion(char* DLLMajor, char* DLLMinor, char* APIMajor, char* APIMinor)
{
    if (DLLMajor) strcpy(DLLMajor, "1");
    if (DLLMinor) strcpy(DLLMinor, "0");
    if (APIMajor) strcpy(APIMajor, "4");
    if (APIMinor) strcpy(APIMinor, "0");
    return RP1210_NO_ERROR;
}

short WINAPI RP1210_ReadDetailedVersion(short /*nClientID*/, char* APIVersion, char* DLLVersion, char* FWVersion)
{
    if (APIVersion) strcpy(APIVersion, "RP1210A");
    if (DLLVersion) strcpy(DLLVersion, "TPRP32 1.0");
    if (FWVersion)  strcpy(FWVersion, "ECM-SIM 1.0");
    return RP1210_NO_ERROR;
}

short WINAPI RP1210_GetHardwareStatus(short nClientID, char* fpchInfo, short nInfoSize, short /*nBlock*/)
{
    if (nClientID != g_client) return ERR_INVALID_CLIENT_ID;
    if (fpchInfo && nInfoSize > 0) memset(fpchInfo, 0, nInfoSize);
    return RP1210_NO_ERROR;   // 0 => hardware present / bus active
}

short WINAPI RP1210_GetErrorMsg(short errCode, char* fpchMessage)
{
    if (!fpchMessage) return 0;
    const char* s;
    switch (errCode) {
        case RP1210_NO_ERROR:       s = "No error."; break;
        case ERR_CLIENT_AREA_FULL:  s = "Client area full (already connected)."; break;
        case ERR_INVALID_CLIENT_ID: s = "Invalid client id."; break;
        case ERR_INVALID_PROTOCOL:  s = "Invalid or unsupported protocol string."; break;
        case ERR_COMMAND_NOT_SUPPORTED: s = "Command not supported."; break;
        case ERR_MESSAGE_NOT_SENT:  s = "Message not sent (bad length)."; break;
        default:                    s = "Tester Present RP1210 emulator error."; break;
    }
    strcpy(fpchMessage, s);
    return RP1210_NO_ERROR;
}

// =============================================================================================
//  Management / configuration control ABI (for the emulator GUI console, AdemBusEmulator.exe).
//  cdecl (not WINAPI) so a C# [DllImport(CallingConvention=Cdecl)] binds by plain name. These act
//  on the same in-process bus (g_sim) that the RP1210 client path drives, so the GUI can start the
//  bus (RP1210_ClientConnect), watch its traffic (RP1210_ReadMessage) and configure it live.
// =============================================================================================

int emu_node_count()
{
    return g_sim.nodeCount();
}

int emu_node_info(int index, int* sa, char* label, int labelLen, int* active, int* stored, int* lamp)
{
    uint8_t s = 0, lp = 0; std::string lbl; int a = 0, st = 0;
    if (!g_sim.nodeInfo(index, s, lbl, a, st, lp)) return 0;
    if (sa) *sa = s;
    if (active) *active = a;
    if (stored) *stored = st;
    if (lamp) *lamp = lp;
    if (label && labelLen > 0) {
        int n = (int)lbl.size(); if (n > labelLen - 1) n = labelLen - 1;
        memcpy(label, lbl.data(), n); label[n] = '\0';
    }
    return 1;
}

int emu_inject_dtc(int sa, unsigned int spn, int fmi, int oc)
{
    g_sim.injectDtc((unsigned char)sa, spn, (unsigned char)fmi, (unsigned char)oc);
    return 0;
}

int emu_clear_dtcs(int sa, int which)
{
    g_sim.clearDtcsFor((unsigned char)sa, which);
    return 0;
}

int emu_get_vin(char* buf, int len)
{
    if (!buf || len <= 0) return 0;
    std::string v = g_sim.vin();
    int n = (int)v.size(); if (n > len - 1) n = len - 1;
    memcpy(buf, v.data(), n); buf[n] = '\0';
    return n;
}

int emu_set_vin(const char* v) { if (v) g_sim.setVin(v); return 0; }
int emu_set_fault(const char* name) { g_sim.setFaultByName(name); return 0; }
int emu_flash_active() { return g_sim.flashActive() ? 1 : 0; }
int emu_running() { return g_sim.running() ? 1 : 0; }

// Live drivable telemetry signals (console sliders): enumerate/describe/set per-controller signals.
int emu_signal_count(int sa) { return g_sim.signalCount((unsigned char)sa); }

int emu_signal_info(int sa, int idx, char* label, int labelLen, char* unit, int unitLen,
                    double* minV, double* maxV, double* cur)
{
    std::string l, u; double mn = 0, mx = 0, cu = 0;
    if (!g_sim.signalInfo((unsigned char)sa, idx, l, u, mn, mx, cu)) return 0;
    if (minV) *minV = mn; if (maxV) *maxV = mx; if (cur) *cur = cu;
    if (label && labelLen > 0) { int n = (int)l.size(); if (n > labelLen - 1) n = labelLen - 1; memcpy(label, l.data(), n); label[n] = '\0'; }
    if (unit  && unitLen  > 0) { int n = (int)u.size(); if (n > unitLen  - 1) n = unitLen  - 1; memcpy(unit,  u.data(), n); unit[n]  = '\0'; }
    return 1;
}

int emu_set_signal(int sa, int idx, double value) { return g_sim.setSignal((unsigned char)sa, idx, value) ? 1 : 0; }

BOOL APIENTRY DllMain(HMODULE, DWORD reason, LPVOID)
{
    // Do NOT take any lock here: DllMain runs under the loader lock, and g_sim.stop() acquires the
    // sim mutex — if a host thread held it at process exit the detach would deadlock. Process
    // teardown reclaims the state; RP1210_ClientDisconnect already stops the sim on a normal close.
    (void)reason;
    return TRUE;
}

} // extern "C"
