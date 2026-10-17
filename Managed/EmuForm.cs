// ---------------------------------------------------------------------------------------------
//  EmuForm.cs — TPSAS-RP1210, the ADEM / J1939 Bus Emulator console (default Windows Forms theme).
//  Runs and controls the Tester Present RP1210 emulator (TPRP32.dll): starts the simulated
//  heavy-vehicle / mobile-plant J1939 bus in-process, shows the live traffic and the controllers,
//  configures it live (inject/clear DTCs, set the VIN, drive flash fault injection), and can
//  register / unregister the emulator as a system RP1210 device.
//
//  Copyright (C) 2026 Jack Leighton — Tester Present, Specialist Automotive Solutions.
//  Licensed under the GNU General Public License v3 (see LICENSE).
// ---------------------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Windows.Forms;

namespace TesterPresent.Adem.EmuConsole
{
    public sealed class EmuForm : Form
    {
        private short _client = -1;
        private readonly System.Windows.Forms.Timer _timer = new() { Interval = 100 };
        private readonly byte[] _rx = new byte[256];

        private long _totalFrames, _framesAtLastRate;
        private DateTime _lastRateAt = DateTime.UtcNow;
        private double _rate;
        private int _uiDivider;
        private readonly Dictionary<uint, long> _pgnCounts = new();
        private readonly Queue<string> _recent = new();
        private readonly List<int> _saList = new();

        private readonly Image? _logo = LoadLogo();
        private readonly Bitmap _flag = FlagFactory.Australian(40);
        private static readonly Font Mono = new("Consolas", 9f);

        // controls
        private Button _startBtn = null!, _stopBtn = null!, _registerBtn = null!, _unregisterBtn = null!, _aboutBtn = null!;
        private Button _injectBtn = null!, _clearActiveBtn = null!, _clearAllBtn = null!, _setVinBtn = null!, _applyFaultBtn = null!;
        private Label _statusLbl = null!;
        private DataGridView _nodeGrid = null!, _pgnGrid = null!;
        private RichTextBox _traffic = null!;
        private TextBox _vinText = null!, _spnText = null!, _fmiText = null!;
        private ComboBox _saCombo = null!, _faultCombo = null!;
        private ToolStripStatusLabel _sbLive = null!;

        public EmuForm()
        {
            Text = "TPSAS-RP1210 — ADEM / J1939 Bus Emulator";
            Font = SystemFonts.MessageBoxFont;           // the Windows default UI font
            MinimumSize = new Size(1000, 680);
            Size = new Size(1080, 760);
            StartPosition = FormStartPosition.CenterScreen;
            TrySetIcon();
            BuildUi();
            _timer.Tick += (_, __) => OnTick();
            FormClosing += (_, __) => StopBus();
            Load += (_, __) => SetRunning(false);
        }

        private void BuildUi()
        {
            var root = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 3 };
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 78));   // logo banner
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));       // toolbar
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));   // body
            root.Controls.Add(BuildHeader(), 0, 0);
            root.Controls.Add(BuildToolbar(), 0, 1);
            root.Controls.Add(BuildBody(), 0, 2);

            Controls.Add(root);
            Controls.Add(BuildStatusStrip());
        }

        private Control BuildHeader()
        {
            var p = new Panel { Dock = DockStyle.Fill, BackColor = Color.White, Padding = new Padding(8, 4, 8, 4) };
            var pic = new PictureBox { Dock = DockStyle.Left, Width = 190, SizeMode = PictureBoxSizeMode.Zoom, Image = _logo, BackColor = Color.White };
            p.Controls.Add(new Label
            {
                Dock = DockStyle.Fill, Text = "RP1210 Heavy-Duty J1939 Bus Emulator", TextAlign = ContentAlignment.MiddleRight,
                Font = new Font(Font.FontFamily, 11f, FontStyle.Bold), ForeColor = Color.Black,
            });
            p.Controls.Add(pic);
            return p;
        }

        private Control BuildToolbar()
        {
            var bar = new FlowLayoutPanel { Dock = DockStyle.Fill, AutoSize = true, FlowDirection = FlowDirection.LeftToRight, WrapContents = false, Padding = new Padding(8, 4, 8, 4) };
            _startBtn = Btn("Start bus", 90); _startBtn.Click += (_, __) => StartBus();
            _startBtn.Font = new Font(Font, FontStyle.Bold);
            _stopBtn = Btn("Stop bus", 90); _stopBtn.Click += (_, __) => StopBus();
            _registerBtn = Btn("Register VCI", 110); _registerBtn.Click += (_, __) => RunElevated("register-vci.ps1", "Register");
            _unregisterBtn = Btn("Unregister VCI", 110); _unregisterBtn.Click += (_, __) => RunElevated("unregister-vci.ps1", "Unregister");
            _aboutBtn = Btn("About…", 80); _aboutBtn.Click += (_, __) => ShowAbout();
            _statusLbl = new Label { Text = "Stopped", AutoSize = true, Margin = new Padding(16, 9, 0, 0), ForeColor = Color.Firebrick };

            bar.Controls.Add(_startBtn);
            bar.Controls.Add(_stopBtn);
            bar.Controls.Add(Gap());
            bar.Controls.Add(_registerBtn);
            bar.Controls.Add(_unregisterBtn);
            bar.Controls.Add(Gap());
            bar.Controls.Add(_aboutBtn);
            bar.Controls.Add(_statusLbl);
            return bar;
        }

        private Control BuildBody()
        {
            var grid = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 2, Padding = new Padding(6) };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 46));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 54));
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 54));
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 46));

            grid.Controls.Add(Group("Controllers on bus", BuildControllersGrid()), 0, 0);
            grid.Controls.Add(Group("Bus activity (live PGN broadcast)", BuildActivityGrid()), 1, 0);
            grid.Controls.Add(Group("Configure (DTC / VIN / flash fault)", BuildConfig()), 0, 1);
            grid.Controls.Add(Group("Recent frames", BuildRecent()), 1, 1);
            return grid;
        }

        private Control BuildControllersGrid()
        {
            _nodeGrid = Grid();
            _nodeGrid.Columns.Add(Col("SA", 12));
            _nodeGrid.Columns.Add(Col("Controller", 40));
            _nodeGrid.Columns.Add(Col("Active", 12));
            _nodeGrid.Columns.Add(Col("Stored", 12));
            _nodeGrid.Columns.Add(Col("Lamps", 24));
            return _nodeGrid;
        }

        private Control BuildActivityGrid()
        {
            _pgnGrid = Grid();
            _pgnGrid.Columns.Add(Col("PGN", 26));
            _pgnGrid.Columns.Add(Col("Acronym", 32));
            _pgnGrid.Columns.Add(Col("Frames", 42));
            return _pgnGrid;
        }

        private Control BuildRecent()
        {
            _traffic = new RichTextBox { Dock = DockStyle.Fill, ReadOnly = true, Font = Mono, WordWrap = false };
            return _traffic;
        }

        private Control BuildConfig()
        {
            var rows = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, AutoSize = true };

            var vinRow = Flow();
            vinRow.Controls.Add(Lab("VIN:", 54));
            _vinText = new TextBox { Width = 190, Font = Mono, Margin = new Padding(0, 2, 6, 2) };
            _setVinBtn = Btn("Set", 54); _setVinBtn.Click += (_, __) => { EmuNative.emu_set_vin(_vinText.Text.Trim()); Status($"VIN set → {_vinText.Text.Trim()}"); };
            vinRow.Controls.Add(_vinText); vinRow.Controls.Add(_setVinBtn);
            rows.Controls.Add(vinRow);

            var saRow = Flow();
            saRow.Controls.Add(Lab("Module:", 54));
            _saCombo = new ComboBox { Width = 260, DropDownStyle = ComboBoxStyle.DropDownList, Margin = new Padding(0, 2, 6, 2) };
            saRow.Controls.Add(_saCombo);
            rows.Controls.Add(saRow);

            var dtcRow = Flow();
            dtcRow.Controls.Add(Lab("SPN:", 54));
            _spnText = new TextBox { Width = 70, Text = "190", Margin = new Padding(0, 2, 4, 2) };
            dtcRow.Controls.Add(_spnText);
            dtcRow.Controls.Add(Lab("FMI:", 34));
            _fmiText = new TextBox { Width = 44, Text = "3", Margin = new Padding(0, 2, 6, 2) };
            dtcRow.Controls.Add(_fmiText);
            _injectBtn = Btn("Inject DTC", 90); _injectBtn.Click += (_, __) => InjectDtc();
            dtcRow.Controls.Add(_injectBtn);
            rows.Controls.Add(dtcRow);

            var clrRow = Flow();
            clrRow.Controls.Add(Lab("", 54));
            _clearActiveBtn = Btn("Clear active", 100); _clearActiveBtn.Click += (_, __) => { if (Target(out int sa)) { EmuNative.emu_clear_dtcs(sa, 0); Status($"Cleared active DTCs on 0x{sa:X2}"); } };
            _clearAllBtn = Btn("Clear all", 84); _clearAllBtn.Click += (_, __) => { if (Target(out int sa)) { EmuNative.emu_clear_dtcs(sa, 2); Status($"Cleared all DTCs on 0x{sa:X2}"); } };
            clrRow.Controls.Add(_clearActiveBtn); clrRow.Controls.Add(_clearAllBtn);
            rows.Controls.Add(clrRow);

            var fltRow = Flow();
            fltRow.Controls.Add(Lab("Flash fault:", 54));
            _faultCombo = new ComboBox { Width = 150, DropDownStyle = ComboBoxStyle.DropDownList, Margin = new Padding(0, 2, 6, 2) };
            _faultCombo.Items.AddRange(new object[] { "none", "inappropriate", "invalidoffset", "badchecksum", "erasefail", "stuck" });
            _faultCombo.SelectedIndex = 0;
            _applyFaultBtn = Btn("Apply", 60); _applyFaultBtn.Click += (_, __) => { EmuNative.emu_set_fault((string)_faultCombo.SelectedItem!); Status($"Flash fault = {_faultCombo.SelectedItem}"); };
            fltRow.Controls.Add(_faultCombo); fltRow.Controls.Add(_applyFaultBtn);
            rows.Controls.Add(fltRow);

            return rows;
        }

        private StatusStrip BuildStatusStrip()
        {
            var strip = new StatusStrip();
            var flag = new ToolStripStatusLabel { Image = _flag, ImageScaling = ToolStripItemImageScaling.None, Margin = new Padding(2, 2, 6, 2) };
            var brand = new ToolStripStatusLabel("Tester Present — Specialist Automotive Solutions") { Font = new Font(Font, FontStyle.Bold) };
            _sbLive = new ToolStripStatusLabel("v1.0  ·  © 2026 Jack Leighton  ·  GPLv3") { Spring = true, TextAlign = ContentAlignment.MiddleRight };
            strip.Items.Add(flag);
            strip.Items.Add(brand);
            strip.Items.Add(_sbLive);
            return strip;
        }

        // ---- bus lifecycle -------------------------------------------------------------------

        private void StartBus()
        {
            if (_client >= 0) return;
            try { _client = EmuNative.RP1210_ClientConnect(IntPtr.Zero, 1, "J1939:Baud=250", 0, 0, 0); }
            catch (DllNotFoundException)
            {
                MessageBox.Show(this, "TPRP32.dll not found beside the app. Build the emulator (build_emu.bat) first.",
                    "Emulator", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (_client < 0) { Status($"ClientConnect failed ({_client})"); _client = -1; return; }

            _totalFrames = 0; _framesAtLastRate = 0; _rate = 0; _pgnCounts.Clear(); _recent.Clear();
            _traffic.Clear(); _pgnGrid.Rows.Clear();
            PopulateControllers();
            var sb = new StringBuilder(64);
            if (EmuNative.emu_get_vin(sb, sb.Capacity) > 0) _vinText.Text = sb.ToString();
            _lastRateAt = DateTime.UtcNow;
            _timer.Start();
            SetRunning(true);
        }

        private void StopBus()
        {
            _timer.Stop();
            if (_client >= 0) { try { EmuNative.RP1210_ClientDisconnect(_client); } catch { } _client = -1; }
            SetRunning(false);
        }

        private void OnTick()
        {
            if (_client < 0) return;
            int drained = 0;
            while (drained < 200)
            {
                short n = EmuNative.RP1210_ReadMessage(_client, _rx, (short)_rx.Length, 0);
                if (n < 11) break;
                drained++; _totalFrames++;
                uint pgn = (uint)(_rx[4] | (_rx[5] << 8) | (_rx[6] << 16));
                byte src = _rx[8], dst = _rx[9];
                int dlen = n - 10;
                _pgnCounts.TryGetValue(pgn, out long c); _pgnCounts[pgn] = c + 1;
                var sb = new StringBuilder();
                sb.Append($"{pgn:X5}  {src:X2}->{dst:X2}  ");
                for (int i = 0; i < dlen && i < 8; i++) sb.Append($"{_rx[10 + i]:X2} ");
                _recent.Enqueue(sb.ToString());
                while (_recent.Count > 40) _recent.Dequeue();
            }

            var now = DateTime.UtcNow;
            double secs = (now - _lastRateAt).TotalSeconds;
            if (secs >= 0.5) { _rate = (_totalFrames - _framesAtLastRate) / secs; _framesAtLastRate = _totalFrames; _lastRateAt = now; }

            if (++_uiDivider % 3 == 0)
            {
                RefreshPgnGrid();
                RefreshControllers();
                _traffic.Text = string.Join(Environment.NewLine, _recent);
                bool flashing = EmuNative.emu_flash_active() != 0;
                _statusLbl.Text = flashing ? "Flashing — bus quiet (DM13)" : "Running";
                _statusLbl.ForeColor = flashing ? Color.DarkGoldenrod : Color.ForestGreen;
                _sbLive.Text = $"{_totalFrames:N0} frames · {_rate:F0}/s · {_pgnCounts.Count} PGNs";
            }
        }

        // ---- refresh -------------------------------------------------------------------------

        private void PopulateControllers()
        {
            _saCombo.Items.Clear(); _saList.Clear();
            int count = EmuNative.emu_node_count();
            for (int i = 0; i < count; i++)
            {
                var lbl = new StringBuilder(64);
                if (EmuNative.emu_node_info(i, out int sa, lbl, lbl.Capacity, out _, out _, out _) == 0) continue;
                _saCombo.Items.Add($"0x{sa:X2}  {lbl}");
                _saList.Add(sa);
            }
            if (_saCombo.Items.Count > 0) _saCombo.SelectedIndex = 0;
        }

        private void RefreshControllers()
        {
            int count = EmuNative.emu_node_count();
            while (_nodeGrid.Rows.Count < count) _nodeGrid.Rows.Add();
            while (_nodeGrid.Rows.Count > count) _nodeGrid.Rows.RemoveAt(_nodeGrid.Rows.Count - 1);
            for (int i = 0; i < count; i++)
            {
                var lbl = new StringBuilder(64);
                if (EmuNative.emu_node_info(i, out int sa, lbl, lbl.Capacity, out int active, out int stored, out int lamp) == 0) continue;
                var r = _nodeGrid.Rows[i];
                r.Cells[0].Value = $"0x{sa:X2}";
                r.Cells[1].Value = lbl.ToString();
                r.Cells[2].Value = active.ToString();
                r.Cells[3].Value = stored.ToString();
                r.Cells[4].Value = Lamps(lamp);
                r.Cells[2].Style.ForeColor = active > 0 ? Color.Firebrick : SystemColors.ControlText;
            }
        }

        private void RefreshPgnGrid()
        {
            var ordered = _pgnCounts.OrderByDescending(kv => kv.Value).ToList();
            while (_pgnGrid.Rows.Count < ordered.Count) _pgnGrid.Rows.Add();
            while (_pgnGrid.Rows.Count > ordered.Count) _pgnGrid.Rows.RemoveAt(_pgnGrid.Rows.Count - 1);
            for (int i = 0; i < ordered.Count; i++)
            {
                var r = _pgnGrid.Rows[i];
                r.Cells[0].Value = $"0x{ordered[i].Key:X5}";
                r.Cells[1].Value = Acronym(ordered[i].Key);
                r.Cells[2].Value = ordered[i].Value.ToString();
            }
        }

        // ---- actions -------------------------------------------------------------------------

        private void InjectDtc()
        {
            if (!Target(out int sa)) return;
            if (!uint.TryParse(_spnText.Text.Trim(), out uint spn)) { Status("Bad SPN."); return; }
            if (!int.TryParse(_fmiText.Text.Trim(), out int fmi) || fmi < 0 || fmi > 31) { Status("Bad FMI (0-31)."); return; }
            EmuNative.emu_inject_dtc(sa, spn, fmi, 1);
            Status($"Injected SPN {spn} FMI {fmi} on 0x{sa:X2}");
        }

        private bool Target(out int sa)
        {
            sa = 0;
            int i = _saCombo.SelectedIndex;
            if (i < 0 || i >= _saList.Count) return false;
            sa = _saList[i];
            return true;
        }

        private void RunElevated(string script, string verb)
        {
            string dir = AppDomain.CurrentDomain.BaseDirectory;
            string path = Path.Combine(dir, script);
            if (!File.Exists(path)) { Status($"{script} not found beside the app."); return; }
            try
            {
                Process.Start(new ProcessStartInfo("powershell.exe", $"-ExecutionPolicy Bypass -NoProfile -File \"{path}\"")
                { Verb = "runas", UseShellExecute = true, WorkingDirectory = dir });
                Status($"{verb} VCI: launched (approve the UAC prompt).");
            }
            catch (Exception ex) { Status($"{verb} VCI cancelled/failed: {ex.Message}"); }
        }

        private void ShowAbout()
        {
            using var dlg = new Form
            {
                Text = "About — TPSAS-RP1210", FormBorderStyle = FormBorderStyle.FixedDialog, StartPosition = FormStartPosition.CenterParent,
                MaximizeBox = false, MinimizeBox = false, ClientSize = new Size(460, 400),
            };
            var face = new Panel { Dock = DockStyle.Top, Height = 90, BackColor = Color.White, Padding = new Padding(8) };
            face.Controls.Add(new PictureBox { Dock = DockStyle.Fill, SizeMode = PictureBoxSizeMode.Zoom, Image = _logo, BackColor = Color.White });

            var stack = new FlowLayoutPanel { Dock = DockStyle.Fill, FlowDirection = FlowDirection.TopDown, WrapContents = false, Padding = new Padding(20, 14, 20, 10), AutoScroll = true };
            Label L(string t, FontStyle fs = FontStyle.Regular) => new() { Text = t, AutoSize = true, Font = new Font(Font, fs), Margin = new Padding(0, 2, 0, 2) };
            LinkLabel Link(string t, string url) { var l = new LinkLabel { Text = t, AutoSize = true, Margin = new Padding(0, 2, 0, 2) }; l.LinkClicked += (_, __) => OpenUrl(url); return l; }

            stack.Controls.Add(L("TPSAS-RP1210 — ADEM / J1939 Bus Emulator  (v1.0)", FontStyle.Bold));
            stack.Controls.Add(L("Tester Present — Specialist Automotive Solutions", FontStyle.Bold));
            stack.Controls.Add(Link("https://testerpresent.com.au", "https://testerpresent.com.au"));
            stack.Controls.Add(Link("https://tester.engineering", "https://tester.engineering"));
            stack.Controls.Add(L(""));
            stack.Controls.Add(L("Author: Jack Leighton"));
            stack.Controls.Add(L("Copyright © 2026 Jack Leighton"));
            stack.Controls.Add(L(""));
            stack.Controls.Add(L("Licensed under the GNU General Public License v3."));
            stack.Controls.Add(Link("View license (LICENSE)", "LICENSE"));

            var ok = new Button { Text = "Close", Width = 90, Dock = DockStyle.Right };
            ok.Click += (_, __) => dlg.Close();
            var okHost = new Panel { Dock = DockStyle.Bottom, Height = 42, Padding = new Padding(10, 7, 14, 7) };
            okHost.Controls.Add(ok);

            dlg.Controls.Add(stack);
            dlg.Controls.Add(okHost);
            dlg.Controls.Add(face);
            dlg.AcceptButton = ok;
            dlg.ShowDialog(this);
        }

        private void OpenUrl(string url)
        {
            try
            {
                if (url == "LICENSE")
                {
                    var p = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "LICENSE");
                    if (File.Exists(p)) { Process.Start(new ProcessStartInfo(p) { UseShellExecute = true }); return; }
                    url = "https://www.gnu.org/licenses/gpl-3.0.html";
                }
                Process.Start(new ProcessStartInfo(url) { UseShellExecute = true });
            }
            catch (Exception ex) { Status($"Open failed: {ex.Message}"); }
        }

        // ---- helpers -------------------------------------------------------------------------

        private void SetRunning(bool on)
        {
            _startBtn.Enabled = !on; _stopBtn.Enabled = on;
            _injectBtn.Enabled = on; _clearActiveBtn.Enabled = on; _clearAllBtn.Enabled = on;
            _setVinBtn.Enabled = on; _applyFaultBtn.Enabled = on;
            _statusLbl.Text = on ? "Running" : "Stopped";
            _statusLbl.ForeColor = on ? Color.ForestGreen : Color.Firebrick;
            if (!on) _sbLive.Text = "v1.0  ·  © 2026 Jack Leighton  ·  GPLv3";
        }

        private void Status(string s) { _statusLbl.Text = s; }

        private static Button Btn(string text, int width) => new() { Text = text, Width = width, Height = 26, Margin = new Padding(3, 0, 3, 0) };
        private static Label Gap() => new() { Text = "", Width = 10, Height = 4 };
        private static FlowLayoutPanel Flow() => new() { FlowDirection = FlowDirection.LeftToRight, WrapContents = false, AutoSize = true, Margin = new Padding(0, 2, 0, 2) };
        private static Label Lab(string t, int w) => new() { Text = t, Width = w, AutoSize = false, TextAlign = ContentAlignment.MiddleLeft, Margin = new Padding(0, 4, 2, 0) };
        private static DataGridViewTextBoxColumn Col(string header, int weight) => new() { HeaderText = header, FillWeight = weight };

        private static GroupBox Group(string title, Control content)
        {
            var g = new GroupBox { Text = title, Dock = DockStyle.Fill, Margin = new Padding(4), Padding = new Padding(6) };
            content.Dock = DockStyle.Fill;
            g.Controls.Add(content);
            return g;
        }

        private static DataGridView Grid() => new()
        {
            Dock = DockStyle.Fill, ReadOnly = true, AllowUserToAddRows = false, AllowUserToDeleteRows = false,
            AllowUserToResizeRows = false, RowHeadersVisible = false, MultiSelect = false,
            SelectionMode = DataGridViewSelectionMode.FullRowSelect, AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill,
            BorderStyle = BorderStyle.FixedSingle,
            ColumnHeadersHeightSizeMode = DataGridViewColumnHeadersHeightSizeMode.DisableResizing,
        };

        private static string Lamps(int b)
        {
            // SAE J1939-73: Protect bits 1-0, Amber 3-2, Red 5-4, MIL 7-6.
            var on = new List<string>();
            if (((b >> 6) & 3) == 1) on.Add("MIL");
            if (((b >> 4) & 3) == 1) on.Add("Red");
            if (((b >> 2) & 3) == 1) on.Add("Amber");
            if ((b & 3) == 1) on.Add("Protect");
            return on.Count == 0 ? "—" : string.Join("+", on);
        }

        private static string Acronym(uint pgn) => pgn switch
        {
            0x00F004 => "EEC1", 0x00F003 => "EEC2", 0x00FEDF => "EEC3", 0x00FEEE => "ET1",
            0x00FEEF => "EFL/P1", 0x00FEF2 => "LFE1", 0x00FEF6 => "IC1", 0x00FEF7 => "VEP1",
            0x00FEBD => "FD1", 0x00FEE4 => "SHUTDN", 0x00FECA => "DM1", 0x00F002 => "ETC1",
            0x00F005 => "ETC2", 0x00FEF8 => "TRF1", 0x00F001 => "EBC1", 0x00FEBF => "EBC2",
            0x00FEF1 => "CCVS1", 0x00FEAE => "AIR1", 0x00FEFA => "Brakes", 0x00F000 => "ERC1",
            0x00FEFC => "DD", 0x00FEC1 => "VDHR", 0x00FEE0 => "VD", 0x00FEF5 => "AMB",
            0x00FE56 => "DEF", 0x00FEE6 => "TD", 0x00FEF3 => "VP", 0x00EE00 => "AddrClaim",
            0x00EC00 => "TP.CM", 0x00EB00 => "TP.DT",
            _ => (pgn >= 0x00FF00 && pgn <= 0x00FFFF) ? "Prop-B" : "",
        };

        private static Image? LoadLogo()
        {
            try
            {
                using var s = Assembly.GetExecutingAssembly().GetManifestResourceStream("tp-logo.jpg");
                if (s == null) return null;
                using var tmp = Image.FromStream(s);
                return new Bitmap(tmp);
            }
            catch { return null; }
        }

        private void TrySetIcon()
        {
            try
            {
                if (_logo == null) return;
                using var bmp = new Bitmap(32, 32);
                using (var g = Graphics.FromImage(bmp))
                {
                    g.Clear(Color.Transparent);
                    g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                    float ar = (float)_logo.Width / _logo.Height;
                    int w = 32, h = (int)(32 / ar); if (h > 32) { h = 32; w = (int)(32 * ar); }
                    g.DrawImage(_logo, (32 - w) / 2, (32 - h) / 2, w, h);
                }
                Icon = Icon.FromHandle(bmp.GetHicon());
            }
            catch { /* leave default icon */ }
        }
    }
}
