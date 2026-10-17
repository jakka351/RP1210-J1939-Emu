// ADEM J1939 Bus Emulator — console entry point.
using System;
using System.Windows.Forms;

namespace TesterPresent.Adem.EmuConsole
{
    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new EmuForm());
        }
    }
}
