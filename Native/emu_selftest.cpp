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
