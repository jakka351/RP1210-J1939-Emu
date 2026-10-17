# Tester Present RP1210 — J1939 ECM / J1939 bus emulator

`TPRP32.dll` is a TMC **RP1210 vendor DLL** that presents a complete, live **SAE J1939 heavy‑vehicle /
mobile‑plant bus** — built around a simulated Caterpillar/Perkins **ADEM engine controller**and the J1939 Spec.
It enumerates and connects exactly like a real RP1210 adapter, so the whole tool chain (enumerate → connect → scan → diagnose → flash)
can be exercised **end‑to‑end with no hardware**.

```
┌─────────────┐  RP1210 API   ┌──────────────┐   J1939 (PGN/SA/data)   ┌───────────────────────────┐
│  RP1210     │ ───────────▶  │ rp1210_emu   │ ──────────────────────▶ │ ecm_sim(EcmSim + BusNode │
│  client     │  Send/Read    │ (TPRP32.dll) │  wire framing <──────── │ 10‑controller J1939 bus    │
│ (tool)      │ ◀───────────  │  shim        │                         │ + ADEM flash + DM diags   │
└─────────────┘               └──────────────┘                         └───────────────────────────┘
```

The bus lives **inside)the DLL**, in the connecting client's process (standard RP1210 model): every
client that connects gets its own fresh, continuously‑running bus.

## Quick start

```
build_emu.bat                              :: build TPRP32.dll + emu_selftest.exe + emu_monitor.exe (x86)
build\emu_monitor.exe build\TPRP32.dll     :: watch the LIVE bus until Ctrl-C
```

`emu_monitor` connects as an RP1210 client and **streams the running bus continuously** — discovering
every controller (address claims + Component IDs), listing each one's broadcast PGNs, and printing a
once‑per‑second heartbeat with the live frame rate. It runs **until you press Ctrl‑C** — the bus is
alive for as long as a client is connected and never self‑terminates. Pass an optional auto‑stop
seconds argument for a bounded run: `build\emu_monitor.exe build\TPRP32.dll 10`.

```
[t=     3s] frames=371  (112/s)  nodes=10  distinct PGNs=33
   SA 0x00  claim=Y  pgns=11  frames=   128  CATERPILLAR*ADEM4*SIM*0001*
   SA 0x03  claim=Y  pgns= 3  frames=    56  CATERPILLAR*TRANS*SIM*0003*
   ...
```

> `emu_selftest.exe` is a **bounded** check — it reads ~2 s and **exits by design** (a test harness,
> not the emulator). Use `emu_monitor` to see the persistent bus.

## The J1939 bus (10 controllers, 33 PGNs)

Every controller claims its own address (PGN `0xEE00`, J1939‑81 NAME), **broadcasts its PGNs
continuously** on their normal periods, and answers PGN requests (`0xEA00`). The bus goes quiet only
while the engine is being flashed (DM13).

| SA | Controller | Spec'd broadcasts (J1939‑71) | Request responses |
|----|------------|------------------------------|-------------------|
| `0x00` | **Engine #1** (ADEM) | EEC1 `0xF004`, EEC2 `0xF003`, EEC3 `0xFEDF`, ET1 `0xFEEE`, EFL/P1 `0xFEEF`, LFE1 `0xFEF2`, IC1 `0xFEF6`, VEP1 `0xFEF7`, FD1 `0xFEBD`, SHUTDN `0xFEE4`, DM1 `0xFECA` | **VIN `0xFEEC`**, **Software ID `0xFEDA` → "ADEM4 2660376-00"**, Component ID `0xFEEB`, Engine Hours `0xFEE5` |
| `0x03` | Transmission #1 | ETC1 `0xF002`, ETC2 `0xF005`, TRF1 `0xFEF8` | Component ID `0xFEEB` |
| `0x0B` | Brakes / ABS (System Controller) | EBC1 `0xF001`, Wheel Speed (EBC2) `0xFEBF`, CCVS1 `0xFEF1`, AIR1 `0xFEAE`, Brakes `0xFEFA` | Component ID `0xFEEB` |
| `0x0F` | Retarder — Engine | ERC1 `0xF000` | Component ID `0xFEEB` |
| `0x17` | Instrument Cluster #1 | Dash Display `0xFEFC`, VDHR `0xFEC1`, VD `0xFEE0` | High‑res distance `0xFEC1`, Component ID `0xFEEB` |
| `0x21` | Body Controller | Ambient `0xFEF5` | Component ID `0xFEEB` |
| `0x25` | Implement / Hydraulics (mobile plant) | Prop‑B `0xFF20` | Component ID `0xFEEB` |
| `0x29` | Telematics / GPS | Time/Date `0xFEE6`, Vehicle Position `0xFEF3` | Component ID `0xFEEB` |
| `0x31` | Cab Controller — Primary | Prop‑B `0xFF31` | Component ID `0xFEEB` |
| `0x3D` | Aftertreatment / SCR | DEF tank `0xFE56`, DM1 `0xFECA`, Prop‑B `0xFF3D` | Component ID `0xFEEB` |

Only the engine answers **Software ID (`0xFEDA`)** and **VIN (`0xFEEC`)**, so the tool's
`resolve_target` and the J1939 scan's engine detection uniquely identify the ADEM engine on the
populated bus. Adding a controller is one `BusNode` + `addPeriodic` / `addRequestResponse` in
`EcmSim::buildBus()`.

**Broadcast conformance.** Every populated SPN sits at the byte offset and uses the scale/offset of
the generic SAE J1939 broadcast reference (byte indices zero‑based, multi‑byte little‑endian), so a
decoder built from that reference reads sensible in‑range values and treats everything else as *not
available* (`0xFF`). Idle‑point examples: EEC1 engine speed `[3:5]` = 650 rpm; EEC2 load `[2]` = 25 %;
ET1 coolant `[0]` = 86 °C / fuel temp `[1]` = 40 °C; EFL/P1 oil pressure `[3]` = 280 kPa; LFE1 fuel
rate `[0:2]` = 3.0 L/h; VEP1 battery `[4:6]` / keyswitch `[6:8]` = 13.8 V; Dash fuel `[1]` = 75 %;
VDHR total `[0:4]` = 2000 km / trip `[4:8]` = 500 km; Engine Hours `[0:4]` = 1234.5 h.

## Identity enumeration (VIN / Software / Component ID)

The engine answers the standard J1939 identification requests, so the tool can enumerate the machine:

| What | PGN | Value (simulated) |
|---|---|---|
| **VIN** (SPN 237) | `0xFEEC` | `1CATADEM4FLS00001` (reference‑valid 17 chars, no I/O/Q) |
| **Software ID** | `0xFEDA` | `ADEM4 2660376-00` |
| **Component ID** (make\*model\*serial\*unit) | `0xFEEB` | `CATERPILLAR*ADEM4*SIM*0001` (per controller) |
| **Engine Hours** | `0xFEE5` | 1234.5 h |

Read the VIN over J1939 with `DevProbe -- vin TPRP32`, or via the GUI's **J1939 network** scan (the
VIN is shown with the enumerated controllers).

## SAE J1939‑73 diagnostic stack

Diagnostic‑capable controllers (engine `0x00`, transmission, brakes, cluster, aftertreatment)
implement the DM service set, backed by **live DTC state** (not a static frame):

| Service | PGN | Behaviour |
|---|---|---|
| **DM1** Active DTCs | `0xFECA` | broadcast continuously + on request; reflects live faults + lamp status |
| **DM2** Previously Active | `0xFECB` | stored DTCs, on request |
| **DM3** Clear Previously Active | `0xFECC` | command → clears stored + positive ACK |
| **DM11** Clear Active | `0xFED3` | command → clears active, lamps off + positive ACK |
| **DM4** Freeze Frame | `0xFECD` | on request (none stored) |
| **DM5** Diagnostic Readiness | `0xFECE` | active / previously‑active counts, OBD compliance, monitor support/complete |
| **DM7 → DM30** Component test | `0xE300` / `0xA400` | command a non‑continuous test, get scaled results (value + min/max → pass/fail) |
| **ACKM** | `0xE800` | positive acknowledgment of DM3 / DM11 |

**Seeded faults** so reads return data: engine has SPN 110 FMI 0 and SPN 100 FMI 1 **active** (amber +
red lamp) and SPN 174 FMI 16 **previously active**; aftertreatment has SPN 1761 FMI 18 active (DEF
level) and SPN 3364 FMI 2 stored. Clearing (DM11/DM3) empties them. Set `TPRP_EMU_DTC=clean` before
connecting to start with no faults. DTCs are the classic 4‑byte J1939‑73 structure (SPN 19‑bit, FMI
5‑bit, OC 7‑bit), shared with the core via `tp.adem.core/src/j1939_diag.h`.

## Flash reprogramming — the real ADEM CAT‑extended protocol

The engine implements the full **ECM side** of the reverse‑engineered ADEM J1939 CAT‑extended flash
protocol (PGN `0xEF00`):

```
enter-flash (status PID 0x88 write 00 01)
  → file-information TLV (0xF003)
  → region arm (0xF001 → Proceed, first offset must be 0)
  → offset-addressed data (0xF002, flow-controlled by the ECM's returned next offset)
  → checksum / burn (0x88 00 23 + CRC, ~1 s Busy → Checksum / BadChecksum)
  → terminate (0x88 00 90)  /  abort
```

On entering flash it emits **DM13 and goes bus‑quiet**, honours the ~10 s session **kill timer**,
parses the file‑info TLV, and enforces the `InvalidOffset` / `BadFileData` / first‑offset‑must‑be‑0
rules — a faithful target, not a happy‑path stub. The live routine trace (run the flash with
`ADEM_LOG=2`) reads:

```
flash: ECM entered flash mode
flash: file information accepted (FileMode)
flash: 5216262 bytes transferred
flash: ECM checksum/verify OK
flash: session terminated (ECM back to normal)
flash: COMPLETE and verified (CRC 0x94637A5E)
```

### Fault injection (exercise the tool's recovery / verify paths)

Set **`TPRP_EMU_FAULT`** before connecting (read once at `RP1210_ClientConnect`):

| `TPRP_EMU_FAULT` | ECM behaviour | Tool should… |
|---|---|---|
| `inappropriate` | refuses enter‑flash with `InappropriateToFlash` | abort before any write |
| `invalidoffset` | rejects the first data chunk with `InvalidOffset` (one‑shot) | abort the transfer, not reset |
| `badchecksum`   | reports `BadChecksum` after the burn | fail verify, **not** reset (verify‑before‑reset) |
| `erasefail`     | reports `ErasureFailure` at the checksum phase | abort, not reset |
| `stuck`         | stays `Busy` forever | time out the burn poll, abort |

Example: `set TPRP_EMU_FAULT=badchecksum` then flash → the tool returns `IntegrityVerifyFailed` and
performs **no** ECU reset.


## Wire framing (standard RP1210 J1939)

```
ClientMessage (SendMessage):  [PGN:3 LE][priority:1][source:1][destination:1][data:n]
APIMessage    (ReadMessage):  [timestamp:4 BE][PGN:3 LE][priority:1][source:1][destination:1][data:n]
```

Everything inside `ecm_sim` works purely in J1939 terms (PGN / priority / source / destination /
data); the shim handles wire framing. A bounded receive FIFO (`MAX_QUEUE`) drops the oldest frames if
a client reads slowly, like a real adapter.

## Install (so the GUI / third‑party apps enumerate it) — requires administrator

```
register.bat          (self-elevates; UAC prompt)
```

Copies `TPRP32.dll` → `%WINDIR%\SysWOW64`, `TPRP32.INI` → `%WINDIR%`, and adds `TPRP32` to
`[RP1210Support] APIImplementations` in `%WINDIR%\RP121032.INI`. Then in the tool: **Device → Refresh
interfaces** → **Tester Present RP1210** appears with protocols `[J1939, Cdl, Ata]`; select it and
**Connect**. Re‑run `register.bat` after rebuilding to refresh the system‑wide copy. Remove with
`unregister.ps1` (elevated).

## Files

| File | Purpose |
|---|---|
| `rp1210_emu.cpp` | RP1210 API shim — the `TPRP32.dll` exports |
| `ecm_sim.h` / `ecm_sim.cpp` | the independent J1939 bus model (EcmSim + BusNode, flash + diagnostics) |
| `emu_monitor.cpp` | **persistent** live bus monitor (runs until Ctrl‑C) |
| `emu_selftest.cpp` | bounded standalone verification (exits by design) |
| `TPRP32.def` | clean (undecorated) export names |
| `TPRP32.INI` | RP1210 vendor descriptor |
| `build_emu.bat` | build the DLL + both tools (x86, via vcvars) |
| `register.bat` / `register.ps1` / `unregister.ps1` | install / uninstall |


