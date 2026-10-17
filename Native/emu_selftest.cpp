/* =============================================================================================
 *  emu_selftest.cpp  —  standalone check of TPRP32.dll WITHOUT system registration.
 *  Loads the DLL by path, connects J1939, requests a few PGNs, reads the bus for ~2 s and prints
 *  the frames (reassembling BAM strings), then disconnects. Proves the emulator independently.
 * ============================================================================================= */
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

typedef short (WINAPI *Connect_t)(HWND, short, const char*, long, long, short);
typedef short (WINAPI *Disconnect_t)(short);
typedef short (WINAPI *Send_t)(short, char*, short, short, short);
typedef short (WINAPI *Read_t)(short, char*, short, short);

static void putClientMsg(char* buf, short& len, unsigned pgn, unsigned char prio, unsigned char src, unsigned char dst, const unsigned char* data, int n)
{
    unsigned char* b = (unsigned char*)buf;
    b[0] = pgn & 0xFF; b[1] = (pgn >> 8) & 0xFF; b[2] = (pgn >> 16) & 0xFF;
    b[3] = prio; b[4] = src; b[5] = dst;
    for (int i = 0; i < n; ++i) b[6 + i] = data[i];
    len = (short)(6 + n);
}

int main(int argc, char** argv)
{
    const char* dllPath = argc > 1 ? argv[1] : "build\\TPRP32.dll";
    HMODULE h = LoadLibraryA(dllPath);
    if (!h) { printf("FAIL: LoadLibrary(%s) err=%lu\n", dllPath, GetLastError()); return 2; }

    auto Connect    = (Connect_t)   GetProcAddress(h, "RP1210_ClientConnect");
    auto Disconnect = (Disconnect_t)GetProcAddress(h, "RP1210_ClientDisconnect");
    auto Send       = (Send_t)      GetProcAddress(h, "RP1210_SendMessage");
    auto Read       = (Read_t)      GetProcAddress(h, "RP1210_ReadMessage");
    if (!Connect || !Send || !Read || !Disconnect) { printf("FAIL: missing exports\n"); return 3; }

    short client = Connect(nullptr, 1, "J1939:Baud=250", 0, 0, 0);
    printf("ClientConnect -> %d\n", client);
    if (client < 0) { printf("FAIL: connect\n"); return 4; }

    // Request VIN (0xFEEC), Software ID (0xFEDA), Component ID (0xFEEB).
    char msg[64]; short len;
    const unsigned pgns[] = { 0xFEEC, 0xFEDA, 0xFEEB, 0xEE00 };
    for (unsigned rp : pgns) {
        unsigned char req[3] = { (unsigned char)(rp & 0xFF), (unsigned char)((rp >> 8) & 0xFF), (unsigned char)((rp >> 16) & 0xFF) };
        putClientMsg(msg, len, 0xEA00, 6, 0xF9, 0x00, req, 3);
        Send(client, msg, len, 0, 0);
    }
    // Also exercise the CAT proprietary read path (PGN 0xEF00, control 0x80).
    { unsigned char p[4] = { 0x80, 0x10, 0x00, 0x01 }; putClientMsg(msg, len, 0xEF00, 6, 0xF9, 0x00, p, 4); Send(client, msg, len, 0, 0); }

    std::map<unsigned, int> counts;
    std::map<unsigned, std::string> bamAscii;   // pgn -> reassembled string
    std::map<unsigned, std::vector<unsigned char>> bamBuf;
    int total = 0;

    for (int iter = 0; iter < 240; ++iter) {
        char buf[256];
        short n = Read(client, buf, sizeof(buf), 0);
        if (n >= 11) {
            unsigned char* b = (unsigned char*)buf;
            unsigned pgn = (unsigned)b[4] | ((unsigned)b[5] << 8) | ((unsigned)b[6] << 16);
            unsigned char src = b[8];
            int dlen = n - 10;
            counts[pgn]++; total++;

            if (pgn == 0xEB00) {                        // TP.DT — collect BAM bytes
                auto& v = bamBuf[src];
                for (int i = 1; i < dlen; ++i) v.push_back(b[10 + i]);
            } else if (pgn == 0xEC00) {                 // TP.CM BAM announce
                // (reset per-source buffer)
                bamBuf[src].clear();
            }
            if (total <= 16) {
                printf("  RX pgn=%05X src=%02X len=%d  ", pgn, src, dlen);
                for (int i = 0; i < dlen && i < 8; ++i) printf("%02X ", b[10 + i]);
                printf("\n");
            }
        }
        Sleep(10);
    }

    // Decode any BAM payloads to ASCII where printable.
    for (auto& kv : bamBuf) {
        std::string s;
        for (unsigned char c : kv.second) if (c >= 32 && c < 127) s.push_back((char)c);
        if (!s.empty()) printf("  BAM from %02X: \"%s\"\n", kv.first, s.c_str());
    }

    printf("\nFrame counts by PGN:\n");
    for (auto& kv : counts) printf("  %05X : %d\n", kv.first, kv.second);
    printf("Total frames: %d\n", total);

    Disconnect(client);
    FreeLibrary(h);
    printf("%s\n", total > 0 ? "OK: emulator produced bus traffic." : "FAIL: no traffic.");
    return total > 0 ? 0 : 5;
}
