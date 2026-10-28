// emu_setfault.cpp - calls the REAL emu_set_fault export in TPRP32.dll, which publishes the fault
// to the shared cross-process control file. Proves console-side fault injection without the GUI.
//   emu_setfault.exe <TPRP32.dll> <none|inappropriate|badfileinfo|invalidoffset|badchecksum|erasefail|stuck>
#include <windows.h>
#include <cstdio>
typedef int (*setfault_t)(const char*);
int main(int argc, char** argv)
{
    const char* dll  = argc > 1 ? argv[1] : "TPRP32.dll";
    const char* name = argc > 2 ? argv[2] : "none";
    HMODULE h = LoadLibraryA(dll);
    if (!h) { printf("LoadLibrary(%s) failed %lu\n", dll, GetLastError()); return 2; }
    setfault_t f = (setfault_t)GetProcAddress(h, "emu_set_fault");
    if (!f) { printf("no emu_set_fault export\n"); return 3; }
    int r = f(name);
    printf("emu_set_fault(\"%s\") -> %d  (published to shared control file)\n", name, r);
    return 0;
}
