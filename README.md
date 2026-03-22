# CAN Bus 3D Vehicle Network Simulator

A dual-language interactive 3D simulator demonstrating how a **CAN Bus (ISO 11898)**
network operates inside a real vehicle — visualized as a blueprint sedan with wired ECU nodes.

```
  ╔══════════════════════════════════════════════════════╗
  ║   CAN BUS 3D VEHICLE SIMULATOR                       ║
  ║   ISO 11898 · 500 kbps · 11-bit IDs · Blueprint UI  ║
  ╚══════════════════════════════════════════════════════╝
```

---

## Two Standalone Implementations

| File             | Language | Renderer              | Dependencies         |
|------------------|----------|-----------------------|----------------------|
| `canbus_3d.cpp`  | C++17    | Software 3D + ANSI terminal | `g++`, `pthreads` |
| `canbus_3d.py`   | Python 3 | Matplotlib 3D + interactive GUI | `matplotlib`, `numpy` |

Both are **standalone** — they run independently with no shared state or sockets.

---

## What It Simulates

### CAN Bus Topology
- **Differential pair**: CAN-H and CAN-L backbone running the length of the vehicle
- **120Ω termination resistors** at each end (per ISO 11898)
- **ECU stub connections** from each node down to the backbone

### 6 ECU Nodes
| Node | CAN ID | Role |
|------|--------|------|
| ECU  | 0x28   | Engine Control Unit — RPM, fuel injection |
| TCM  | 0x2A   | Transmission — gear selection, torque |
| BCM  | 0x35   | Body Control Module — central gateway |
| ABS  | 0x18   | Anti-lock Braking — wheel speed, brake pressure |
| TPMS | 0x3F   | Tyre Pressure Monitor — PSI on all 4 wheels |
| OBD  | 0x7DF  | OBD-II Gateway — diagnostic broadcast address |

### CAN Frame Fields (decoded live)
```
SOF  · ID · RTR · IDE · DLC · DATA · CRC · ACK · EOF
```

### Animated Packet Flow
Packets travel the correct physical route:
```
Source ECU → stub → CAN backbone → stub → Destination ECU
```

---

## Quick Start

### C++ Version

**Requirements:** `g++` with C++17, POSIX terminal (Linux / macOS)

```bash
# Build
make

# Run
./canbus_3d
```

Or without make:
```bash
g++ -std=c++17 -O2 -o canbus_3d canbus_3d.cpp -lpthread
./canbus_3d
```

**Terminal requirements:** minimum **120 × 40** characters, monospace font, ANSI 256-color support.

#### Controls (C++ Terminal Version)

| Key     | Action                          |
|---------|---------------------------------|
| `q / e` | Rotate view left / right        |
| `w / s` | Tilt view up / down             |
| `+ / -` | Zoom in / out                   |
| `r`     | Reset camera to default view    |
| `1`     | Fire: ENGINE RPM  (ECU → BCM)   |
| `2`     | Fire: GEAR SHIFT  (TCM → BCM)   |
| `3`     | Fire: ABS TRIGGER (ABS → BCM)   |
| `4`     | Fire: TYRE PSI    (TPMS → BCM)  |
| `5`     | Fire: OBD QUERY   (OBD → ECU)   |
| `6`     | Fire: FUEL REQ    (BCM → ECU)   |
| `a`     | Scenario: Cold Engine Startup   |
| `b`     | Scenario: Emergency Braking     |
| `d`     | Scenario: OBD Diagnostic Scan   |
| `ESC/x` | Quit                            |

---

### Python Version

**Requirements:** Python 3.8+, `matplotlib`, `numpy`, a GUI display (X11/Wayland/macOS)

```bash
# Install dependencies
pip3 install matplotlib numpy

# Or via make
make deps

# Run
python3 canbus_3d.py

# Or via make
make python
```

**GUI requirements:** A display server must be available (works on Linux with X11/Wayland,
macOS, Windows WSL with VcXsrv/XMing).

#### Controls (Python GUI Version)
- **Click ECU buttons** in right panel to fire individual CAN frames
- **Scenario buttons** run multi-frame choreographed sequences
- **Drag** the 3D viewport to rotate the sedan
- **Scroll** to zoom
- Watch the **CAN Frame Decoder** and **Bus Log** update in real time

---

## Scenarios Explained

### Cold Engine Startup (`a` / button)
```
ECU  → BCM  : ECU_INIT      [0x28] ECU announces presence
TCM  → BCM  : TCM_READY     [0x2A] Transmission ready
ABS  → BCM  : ABS_READY     [0x18] ABS system self-check OK
TPMS → BCM  : TPMS_OK       [0x3F] All tyre pressures nominal
BCM  → ECU  : START_ENGINE  [0x35] BCM authorises engine start
ECU  → BCM  : RPM_IDLE      [0x28] Engine running, broadcasting RPM
```

### Emergency Braking (`b` / button)
```
ABS  → BCM  : BRAKE_HARD    [0x18] Hard brake pedal detected
ABS  → BCM  : ABS_ACTIVE    [0x18] ABS intervention begins
TCM  → BCM  : DOWNSHIFT     [0x2A] Transmission downshifts for engine braking
ECU  → BCM  : ENGINE_BRAKE  [0x28] Engine torque reduction
ABS  → BCM  : ABS_RELEASE   [0x18] ABS cycle complete, wheels unlocked
```

### OBD-II Diagnostic Scan (`d` / button)
```
OBD  → ECU  : OBD_QUERY_RPM   [0x7DF] PID 0x0C — request engine RPM
ECU  → OBD  : OBD_RESP_RPM    [0x7E8] Response: 4-byte RPM value
OBD  → ECU  : OBD_QUERY_SPEED [0x7DF] PID 0x0D — request vehicle speed
ECU  → OBD  : OBD_RESP_SPEED  [0x7E8] Response: speed in km/h
OBD  → ECU  : OBD_QUERY_TEMP  [0x7DF] PID 0x05 — coolant temperature
ECU  → OBD  : OBD_RESP_TEMP   [0x7E8] Response: temp in °C
```

> **0x7DF** is the OBD-II functional broadcast address — all ECUs that understand
> the query respond. **0x7E8** is ECU's dedicated response ID (0x7E0 + 8).

---

## Architecture

```
┌─────────────────────────────────────────────┐
│              Application Layer               │
│  User Input → ECU trigger → CANBusEngine     │
├─────────────────────────────────────────────┤
│              Simulation Layer                │
│  Frame queuing · Timestamp · Log management  │
├─────────────────────────────────────────────┤
│              Rendering Layer                 │
│  C++: Software 3D → ANSI Framebuffer        │
│  Python: Matplotlib 3D → FuncAnimation      │
├─────────────────────────────────────────────┤
│              Geometry Layer                  │
│  Sedan wireframe · ECU nodes · Bus backbone  │
└─────────────────────────────────────────────┘
```

### C++ Software 3D Renderer
- **No OpenGL / no external libs** — pure math
- Perspective projection with configurable FOV
- Z-buffer (depth integer per cell) prevents overdraw
- Bresenham line rasterizer maps 3D edges → terminal cells
- ANSI 24-bit colour per character cell

### Python Matplotlib 3D
- `mpl_toolkits.mplot3d` for the 3D sedan and nodes
- `matplotlib.animation.FuncAnimation` at 30fps
- `matplotlib.widgets.Button` for interactive controls
- Threaded `CANBus` engine decoupled from render loop
- Packet interpolated across 4 waypoints per frame

---

## CAN Bus Protocol Notes

- **Multi-master**: any ECU can transmit; arbitration is bitwise (lower ID wins)
- **Non-destructive arbitration**: losing node backs off and retries
- **Broadcast**: all nodes receive all frames; each filters by ID
- **ACK bit**: any receiver that correctly decodes a frame pulls ACK dominant
- **Error detection**: CRC-15, bit stuffing, frame check, ACK check, bit monitoring

---

## File Structure

```
canbus_3d/
├── canbus_3d.cpp      C++17 terminal simulator
├── canbus_3d.py       Python matplotlib simulator
├── Makefile           Build & run targets
└── README.md          This file
```

---

## Platform Notes

| Platform    | C++ binary | Python GUI |
|-------------|-----------|------------|
| Linux (X11) | ✅ native  | ✅ with PyQt5 or GTK |
| macOS       | ✅ native  | ✅ native matplotlib |
| Windows WSL | ✅ with ANSI terminal | ✅ with VcXsrv |
| Windows CMD | ❌ (POSIX only) | ✅ with Anaconda |

For Windows C++, recompile with MinGW or use WSL2.
