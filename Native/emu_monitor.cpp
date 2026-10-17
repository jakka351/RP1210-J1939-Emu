/* =============================================================================================
 *  emu_monitor.cpp  —  PERSISTENT live bus monitor for the Tester Present RP1210 emulator.
 *
 *  Unlike emu_selftest (a bounded self-check that reads ~2 s and exits), this connects to the
 *  emulator DLL as a real RP1210 client and then STREAMS THE LIVE J1939 BUS CONTINUOUSLY until
 *  you press Ctrl-C. It discovers every controller (address claims + Component IDs), lists the
 *  PGNs each one broadcasts, and prints a once-per-second heartbeat with the running frame rate
 *  — so you can watch a full, never-terminating heavy-vehicle / mobile-plant bus.
 *
 *  Usage:  emu_monitor.exe [path\to\TPRP32.dll] [auto-stop-seconds]
 *          (default DLL: build\TPRP32.dll; auto-stop 0 = run forever until Ctrl-C)
 * ============================================================================================= */
#include <windows.h>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

typedef short (WINAPI *Connect_t)(HWND, short, const char*, long, long, short);
typedef short (WINAPI *Disconnect_t)(short);
typedef short (WINAPI *Send_t)(short, char*, short, short, short);
typedef short (WINAPI *Read_t)(short, char*, short, short);

static volatile bool g_run = true;
static BOOL WINAPI onCtrlC(DWORD) { g_run = false; return TRUE; }

struct Node {
    unsigned char name[8] = {0};
    bool claimed = false;
    std::set<unsigned> pgns;
    unsigned long long frames = 0;
    std::string component;      // from Component ID 0xFEEB
};

static void sendReq(Send_t Send, short client, unsigned reqPgn, unsigned char dst)
{
    unsigned char msg[16];
    msg[0] = 0xEA & 0; // placeholder (overwritten below)
    // ClientMessage: [PGN:3 LE][prio][src][dst][data]; request PGN 0xEA00, body = requested PGN LE
    msg[0] = 0x00; msg[1] = 0xEA; msg[2] = 0x00;   // PGN 0x00EA00 (LE bytes)
    msg[3] = 6; msg[4] = 0xF9; msg[5] = dst;
    msg[6] = (unsigned char)(reqPgn & 0xFF);
    msg[7] = (unsigned char)((reqPgn >> 8) & 0xFF);
    msg[8] = (unsigned char)((reqPgn >> 16) & 0xFF);
    Send(client, (char*)msg, 9, 0, 0);
}

int main(int argc, char** argv)
{
    const char* dllPath = argc > 1 ? argv[1] : "build\\TPRP32.dll";
    long maxSecs = argc > 2 ? atol(argv[2]) : 0;   // 0 => run forever (until Ctrl-C)
    setvbuf(stdout, nullptr, _IONBF, 0);           // unbuffered so piped output appears live
    HMODULE h = LoadLibraryA(dllPath);
    if (!h) { printf("FAIL: LoadLibrary(%s) err=%lu\n", dllPath, GetLastError()); return 2; }

    auto Connect    = (Connect_t)   GetProcAddress(h, "RP1210_ClientConnect");
    auto Disconnect = (Disconnect_t)GetProcAddress(h, "RP1210_ClientDisconnect");
    auto Send       = (Send_t)      GetProcAddress(h, "RP1210_SendMessage");
    auto Read       = (Read_t)      GetProcAddress(h, "RP1210_ReadMessage");
    if (!Connect || !Send || !Read || !Disconnect) { printf("FAIL: missing RP1210 exports\n"); return 3; }

    short client = Connect(nullptr, 1, "J1939:Baud=250", 0, 0, 0);
    if (client < 0) { printf("FAIL: ClientConnect -> %d\n", client); return 4; }

    SetConsoleCtrlHandler(onCtrlC, TRUE);
    printf("Tester Present RP1210 — LIVE bus monitor (%s)\n", dllPath);
    printf("Connected as client %d. Streaming the J1939 bus — press Ctrl-C to stop.\n\n", client);

    sendReq(Send, client, 0x00EE00, 0xFF);   // prompt every controller to (re)announce its address

    std::map<unsigned char, Node> nodes;            // SA -> node
    std::map<unsigned, unsigned long long> pgnCount; // PGN -> total frames
    std::map<unsigned char, std::vector<unsigned char>> bamBuf;  // src -> reassembly
    std::map<unsigned char, unsigned> bamPgn;                    // src -> PGN being reassembled
    std::set<unsigned char> askedId;                 // SAs we've requested Component ID from

    unsigned long long total = 0;
    ULONGLONG t0 = GetTickCount64();
    ULONGLONG lastBeat = t0;
    unsigned long long framesAtLastBeat = 0;

    while (g_run) {
        char buf[256];
        short n = Read(client, buf, sizeof(buf), 0);
        if (n >= 11) {
            unsigned char* b = (unsigned char*)buf;
            unsigned pgn = (unsigned)b[4] | ((unsigned)b[5] << 8) | ((unsigned)b[6] << 16);
            unsigned char src = b[8];
            int dlen = n - 10;

            total++;
            pgnCount[pgn]++;
            Node& nd = nodes[src];
            nd.frames++;

            if (pgn == 0x00EE00 && dlen >= 8) {          // Address Claimed — record NAME
                nd.claimed = true;
                for (int i = 0; i < 8; ++i) nd.name[i] = b[10 + i];
                if (!askedId.count(src)) { askedId.insert(src); sendReq(Send, client, 0x00FEEB, src); }
            } else if (pgn == 0x00EC00 && dlen >= 8 && b[10] == 0x20) {  // TP.CM BAM announce
                unsigned ann = (unsigned)b[15] | ((unsigned)b[16] << 8) | ((unsigned)b[17] << 16);
                bamPgn[src] = ann; bamBuf[src].clear();
            } else if (pgn == 0x00EB00 && dlen >= 2) {   // TP.DT data
                auto& v = bamBuf[src];
                for (int i = 1; i < dlen; ++i) v.push_back(b[10 + i]);
                if (bamPgn.count(src) && bamPgn[src] == 0x00FEEB) {       // Component ID text
                    std::string s;
                    for (unsigned char c : v) if (c >= 32 && c < 127) s.push_back((char)c);
                    if (!s.empty()) nd.component = s;
                }
            } else if (pgn != 0x00EB00 && pgn != 0x00EC00) {
                nd.pgns.insert(pgn);                     // application PGN broadcast by this SA
            }
        }

        ULONGLONG now = GetTickCount64();
        if (maxSecs > 0 && (now - t0) >= (ULONGLONG)maxSecs * 1000) g_run = false;
        if (now - lastBeat >= 1000) {
            double secs = (now - t0) / 1000.0;
            double rate = (double)(total - framesAtLastBeat) * 1000.0 / (double)(now - lastBeat);
            printf("[t=%6.0fs] frames=%llu  (%.0f/s)  nodes=%zu  distinct PGNs=%zu\n",
                   secs, (unsigned long long)total, rate, nodes.size(), pgnCount.size());
            for (auto& kv : nodes) {
                const Node& nd = kv.second;
                printf("   SA 0x%02X  claim=%c  pgns=%2zu  frames=%6llu  %s\n",
                       kv.first, nd.claimed ? 'Y' : '-', nd.pgns.size(),
                       (unsigned long long)nd.frames,
                       nd.component.empty() ? "" : nd.component.c_str());
            }
            printf("\n");
            lastBeat = now; framesAtLastBeat = total;
        }
        if (n < 11) Sleep(1);
    }

    printf("\nStopping — disconnecting cleanly.\n");
    Disconnect(client);
    FreeLibrary(h);
    printf("Monitored %llu frames from %zu controllers across %zu PGNs.\n",
           (unsigned long long)total, nodes.size(), pgnCount.size());
    return 0;
}
