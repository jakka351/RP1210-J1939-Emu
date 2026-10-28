// ---------------------------------------------------------------------------------------------
//  EmuNative.cs — P/Invoke boundary to TPRP32.dll (the Tester Present RP1210 emulator).
//  The console drives the bus with the standard RP1210 entry points (stdcall) and configures it
//  live with the emulator's management control ABI (cdecl, emu_*).
// ---------------------------------------------------------------------------------------------

using System;
using System.Runtime.InteropServices;
using System.Text;

namespace TesterPresent.Adem.EmuConsole
{
    internal static class EmuNative
    {
        private const string Dll = "TPRP32.dll";

        // ---- RP1210 (stdcall) — start the bus, read its traffic ----
        [DllImport(Dll, CallingConvention = CallingConvention.StdCall, CharSet = CharSet.Ansi)]
        public static extern short RP1210_ClientConnect(IntPtr hwnd, short deviceId, string protocol, int txBuf, int rxBuf, short isAppPacket);

        [DllImport(Dll, CallingConvention = CallingConvention.StdCall)]
        public static extern short RP1210_ClientDisconnect(short clientId);

        [DllImport(Dll, CallingConvention = CallingConvention.StdCall)]
        public static extern short RP1210_ReadMessage(short clientId, byte[] msg, short bufSize, short block);

        [DllImport(Dll, CallingConvention = CallingConvention.StdCall)]
        public static extern short RP1210_SendMessage(short clientId, byte[] msg, short size, short notify, short block);

        // ---- management / configuration control ABI (cdecl, emu_*) ----
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int emu_node_count();

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int emu_node_info(int index, out int sa, StringBuilder label, int labelLen, out int active, out int stored, out int lamp);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int emu_inject_dtc(int sa, uint spn, int fmi, int oc);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int emu_clear_dtcs(int sa, int which);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int emu_get_vin(StringBuilder buf, int len);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int emu_set_vin(string v);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int emu_set_fault(string name);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int emu_flash_active();

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int emu_running();

        // ---- live drivable signals (console sliders) ----
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int emu_signal_count(int sa);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int emu_signal_info(int sa, int idx, StringBuilder label, int labelLen,
            StringBuilder unit, int unitLen, out double minV, out double maxV, out double cur);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int emu_set_signal(int sa, int idx, double value);
    }
}
