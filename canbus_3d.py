#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════╗
║  CAN Bus 3D  —  Powertrain Sensor Network  |  Vispy / OpenGL           ║
║  Tesla Model 3  ·  ISO 11898  ·  500 kbps                              ║
║                                                                          ║
║  Powertrain sensors streaming live CAN frames:                          ║
║    Motor RPM · Torque · Battery % · Motor Temp · Regen · Drive Mode    ║
║                                                                          ║
║  ECU Health Dashboard:                                                   ║
║    Node heartbeat · Message rate · Bus load · Error counters            ║
║                                                                          ║
║  Realistic Model 3 driving physics:                                     ║
║    W/S = Throttle/Brake  ·  A/D = Steer  ·  M = Auto-drive             ║
╚══════════════════════════════════════════════════════════════════════════╝

Install:
    pip install vispy pyopengl pyqt5

Controls:
    W / Up       Throttle          Mouse drag   Rotate 3D view
    S / Down     Brake             Scroll       Zoom
    A / Left     Steer left        R            Reset camera
    D / Right    Steer right       F            Inject fault
    M            Auto-drive        C            Clear faults
    Q / Esc      Quit
"""

import sys, time, math, threading, queue, random, collections, struct
import numpy as np

# ── Vispy ─────────────────────────────────────────────────────────────────────
try:
    import vispy
    from vispy import app, scene
    from vispy.scene import visuals
except ImportError:
    print("\n  ERROR: Run:  pip install vispy pyopengl pyqt5\n")
    sys.exit(1)

def _pick_backend():
    for be in ['pyqt5', 'pyqt6', 'pyside2', 'pyside6', 'tkinter', 'wx']:
        try:
            app.use_app(be)
            print(f"  [vispy] {be}")
            return
        except Exception:
            continue
_pick_backend()

# ─────────────────────────────────────────────────────────────────────────────
#  PALETTE
# ─────────────────────────────────────────────────────────────────────────────
BG       = (0.004, 0.043, 0.094, 1.0)  # #010b18  deep navy
BLUE     = (0.000, 0.706, 1.000, 1.0)  # #00b4ff
BLUE2    = (0.000, 0.400, 0.667, 1.0)  # #0066aa
CYAN     = (0.000, 1.000, 0.906, 1.0)  # #00ffe7
CYAN2    = (0.000, 0.600, 0.780, 1.0)  # #0099c7
GREEN    = (0.000, 1.000, 0.533, 1.0)  # #00ff88
WARN     = (1.000, 0.722, 0.000, 1.0)  # #ffb800
RED      = (1.000, 0.231, 0.361, 1.0)  # #ff3b5c
DIMBLUE  = (0.039, 0.165, 0.227, 1.0)  # #0a2a3a
GRIDCOL  = (0.039, 0.188, 0.314, 1.0)  # #0a3050
TEXT     = (0.722, 0.863, 1.000, 1.0)  # #b8dcff
PANEL    = (0.008, 0.055, 0.110, 1.0)  # #020e1c

def rgb(c):   return c[:3]
def rgba(c, a=1.0): return c[:3] + (a,)

# ─────────────────────────────────────────────────────────────────────────────
#  ECU NODE DEFINITIONS  (6 nodes, each with CAN ID + position in 3D)
# ─────────────────────────────────────────────────────────────────────────────
ECU_DEFS = {
    'ECU':  dict(pos=np.array([-0.50,  0.00,  0.10]),
                 color=CYAN,   id='0x28', label='Motor ECU',
                 desc='Main motor controller\nRPM · Torque · Temp'),
    'BCM':  dict(pos=np.array([ 0.00,  0.00,  0.42]),
                 color=BLUE,   id='0x35', label='Body Control',
                 desc='Central gateway\nSwitches · Lights · Mode'),
    'TCM':  dict(pos=np.array([ 0.55,  0.00,  0.10]),
                 color=BLUE,   id='0x2A', label='Drive Unit',
                 desc='Single-speed drive\nRegen · Gear state'),
    'BMS':  dict(pos=np.array([-0.90,  0.00, -0.20]),
                 color=GREEN,  id='0x3B', label='Battery Mgmt',
                 desc='82 kWh pack\nSOC · Voltage · Current'),
    'VCM':  dict(pos=np.array([ 0.00,  0.00,  0.00]),
                 color=WARN,   id='0x1A', label='Vehicle Ctrl',
                 desc='Top-level coordinator\nPower limits · Modes'),
    'OBD':  dict(pos=np.array([ 0.00,  0.75,  0.00]),
                 color=RED,    id='0x7DF', label='OBD Gateway',
                 desc='Diagnostic port\nISO 15765-4 UDS'),
}

# ─────────────────────────────────────────────────────────────────────────────
#  POWERTRAIN PHYSICS  (Model 3 Long Range)
# ─────────────────────────────────────────────────────────────────────────────
class Powertrain:
    """
    Realistic single-motor Model 3 powertrain simulation.

    State variables that drive CAN sensor frames:
      speed_kmh   → BCM 0x350  vehicle speed
      rpm         → ECU 0x280  motor RPM
      torque_nm   → ECU 0x280  motor torque
      battery_pct → BMS 0x3B0  state of charge
      motor_temp  → ECU 0x280  motor temperature
      regen_kw    → TCM 0x2A0  regenerative braking power
      mode        → VCM 0x1A0  drive mode string
    """
    # --- Model 3 LR constants ---
    MASS_KG        = 1850.0
    WHEEL_R_M      = 0.355
    BATTERY_KWH    = 82.0
    MAX_TORQUE_NM  = 420.0    # peak at wheel
    MAX_POWER_KW   = 258.0
    DRAG_CD        = 0.23
    FRONTAL_AREA   = 2.40
    AIR_RHO        = 1.20
    GEAR_RATIO     = 9.0
    MOTOR_POLES    = 4        # pole pairs → electrical RPM factor
    MAX_MOTOR_RPM  = 18000.0

    def __init__(self):
        # Driver inputs (set by key handler or auto-drive)
        self.throttle   = 0.0    # 0-1
        self.brake_pct  = 0.0    # 0-1
        self.steer      = 0.0    # -1..1

        # Physical state
        self.speed_kmh  = 0.0
        self.rpm        = 0.0
        self.torque_nm  = 0.0
        self.accel_ms2  = 0.0
        self.regen_kw   = 0.0
        self.battery_pct= 87.0
        self.motor_temp = 34.0   # °C ambient start
        self.inverter_v = 396.0  # nominal HV bus voltage
        self.current_a  = 0.0    # HV current
        self.power_kw   = 0.0

        self.mode       = 'PARK'
        self.fault_code = None

        self._t = 0.0
        self._rng = random.Random(7)

    def update(self, dt: float):
        self._t += dt
        v = self.speed_kmh / 3.6   # m/s

        # --- Torque demand ---
        torque_demand = self.throttle * self.MAX_TORQUE_NM
        # Traction-motor torque limit: P_max / omega (with floor to avoid div0)
        omega = max(1.0, v / self.WHEEL_R_M)   # rad/s at wheel
        torque_limit = self.MAX_POWER_KW * 1000 / omega
        torque_demand = min(torque_demand, torque_limit)

        # --- Brake / regen ---
        regen_torque = 0.0
        if self.brake_pct > 0.0:
            # blended braking: first 30% is pure regen
            regen_frac  = min(1.0, self.brake_pct / 0.30)
            regen_torque= regen_frac * 80.0 * min(1.0, v / 5.0)
            self.regen_kw = regen_torque * omega / 1000.0
        elif self.throttle < 0.02 and v > 2.0:
            # coasting regen (one-pedal)
            coasting_regen = 40.0 * min(1.0, v / 10.0)
            regen_torque   = coasting_regen
            self.regen_kw  = regen_torque * omega / 1000.0
        else:
            self.regen_kw = 0.0

        # --- Drag ---
        drag_n = 0.5 * self.AIR_RHO * self.DRAG_CD * self.FRONTAL_AREA * v**2
        # Rolling resistance
        roll_n = self.MASS_KG * 9.81 * 0.012

        # --- Net force ---
        drive_n = torque_demand / self.WHEEL_R_M
        regen_n = regen_torque  / self.WHEEL_R_M
        brake_n = max(0.0, self.brake_pct - 0.30) / 0.70 * 8000.0
        net_n   = drive_n - regen_n - brake_n - drag_n - roll_n

        self.accel_ms2 = net_n / self.MASS_KG
        v = max(0.0, v + self.accel_ms2 * dt)
        self.speed_kmh = v * 3.6

        # --- RPM & torque readback ---
        # motor shaft RPM = wheel RPM × gear_ratio
        wheel_rps   = v / (2 * math.pi * self.WHEEL_R_M)
        self.rpm    = min(self.MAX_MOTOR_RPM,
                          wheel_rps * self.GEAR_RATIO * 60.0)
        self.torque_nm = torque_demand + self._rng.gauss(0, 1.5)

        # --- Electrical ---
        self.power_kw   = drive_n * v / 1000.0
        self.current_a  = self.power_kw * 1000.0 / max(1.0, self.inverter_v)
        # Battery discharge (kWh per second)
        net_kwh_s = (self.power_kw - self.regen_kw) / 3600.0
        self.battery_pct = max(2.0,
                               min(100.0, self.battery_pct
                                   - net_kwh_s / self.BATTERY_KWH * 100.0))

        # --- Thermal model ---
        target_temp = 34.0 + self.throttle * 90.0 + (v * 0.4)
        tau = 12.0   # thermal time constant seconds
        self.motor_temp += (target_temp - self.motor_temp) / tau * dt
        self.motor_temp += self._rng.gauss(0, 0.05)

        # --- Drive mode ---
        if self.fault_code:
            self.mode = 'FAULT'
        elif self.speed_kmh < 0.5:
            self.mode = 'PARK' if self.throttle < 0.02 else 'READY'
        elif self.brake_pct > 0.3:
            self.mode = 'BRAKE'
        elif self.regen_kw > 2.0 and self.throttle < 0.05:
            self.mode = 'REGEN'
        elif self.throttle > 0.6:
            self.mode = 'SPORT'
        else:
            self.mode = 'DRIVE'


# ─────────────────────────────────────────────────────────────────────────────
#  AUTO-DRIVE SEQUENCE
# ─────────────────────────────────────────────────────────────────────────────
class AutoDrive:
    # (duration_s, throttle, brake, steer)
    SCRIPT = [
        (2.5, 0.00, 0.00,  0.00),   # standstill
        (3.0, 0.45, 0.00,  0.00),   # gentle pull-away
        (3.5, 0.80, 0.00,  0.00),   # hard acceleration
        (2.0, 0.35, 0.00,  0.25),   # right corner
        (2.0, 0.35, 0.00, -0.25),   # left corner
        (1.5, 0.10, 0.00,  0.00),   # lift throttle → regen
        (2.5, 0.00, 0.85,  0.00),   # hard brake
        (1.5, 0.00, 0.20,  0.00),   # ease off
        (3.0, 0.65, 0.00,  0.00),   # re-accelerate
        (2.0, 0.00, 0.00,  0.00),   # coast / regen
    ]

    def __init__(self, pt: Powertrain):
        self.pt       = pt
        self._running = False

    def toggle(self):
        self._running = not self._running
        if self._running:
            threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._running = False

    def _run(self):
        while self._running:
            for dur, thr, brk, st in self.SCRIPT:
                if not self._running: return
                t0 = time.time()
                while time.time()-t0 < dur and self._running:
                    self.pt.throttle  = thr
                    self.pt.brake_pct = brk
                    self.pt.steer     = st
                    time.sleep(0.016)
        # Reset on exit
        self.pt.throttle = self.pt.brake_pct = self.pt.steer = 0.0


# ─────────────────────────────────────────────────────────────────────────────
#  CAN BUS ENGINE  — powertrain frames only
# ─────────────────────────────────────────────────────────────────────────────
class CANFrame:
    __slots__ = ('src','dst','can_id','label','payload','ts',
                 'src_pos','dst_pos','t','trail')
    def __init__(self, src, dst, can_id, label, payload, ts, src_pos, dst_pos):
        self.src=src; self.dst=dst; self.can_id=can_id
        self.label=label; self.payload=payload; self.ts=ts
        self.src_pos=np.array(src_pos, np.float32)
        self.dst_pos=np.array(dst_pos, np.float32)
        self.t=0.0; self.trail=[]


class CANBus:
    """
    Transmits powertrain CAN frames at ISO 11898 realistic rates.

    Frame schedule (matches real Model 3 CAN captures):
      0x280  ECU→BCM   Motor RPM, Torque, Temp         10 ms
      0x3B0  BMS→BCM   Battery SOC, Voltage, Current   20 ms
      0x2A0  TCM→BCM   Drive state, Regen kW           20 ms
      0x1A0  VCM→ALL   Vehicle speed, Mode             20 ms
      0x350  BCM→ECU   Odometer, HV contactor state   100 ms
      0x7E8  ECU→OBD   OBD response (on-demand)        --
    """

    # Fault DTC table
    FAULTS = [
        ('ECU', '0x280', 'P0A0F', 'Motor over-temperature'),
        ('BMS', '0x3B0', 'P0A94', 'HV battery isolation fault'),
        ('TCM', '0x2A0', 'P0730', 'Drive unit torque deviation'),
        ('VCM', '0x1A0', 'U0100', 'Lost comm with VCM'),
        ('ECU', '0x280', 'P0563', 'HV system voltage high'),
    ]

    def __init__(self, pt: Powertrain):
        self.pt      = pt
        self.queue   = queue.Queue(maxsize=300)
        self.log     = collections.deque(maxlen=120)
        self._t0     = time.time()
        self._run    = True
        self._lock   = threading.Lock()
        self._rng    = random.Random(3)

        # Health tracking per node
        self.node_health = {
            k: dict(tx_count=0, rx_count=0, error_count=0,
                    last_seen=time.time(), msg_rate=0.0,
                    bus_load_pct=0.0, status='OK')
            for k in ECU_DEFS
        }
        self._rate_cnt  = {k: 0 for k in ECU_DEFS}

        # Last decoded frame (for HUD display)
        self.last_frame = {}
        self.active_faults = []

    # ── internal helpers ──────────────────────────────────────────────────────
    def _ts(self):
        ms = (time.time()-self._t0)*1000
        return f'{int(ms/60000):02d}:{int((ms%60000)/1000):02d}.{int(ms%1000):03d}'

    def _push(self, src, dst, can_id, label, payload: bytes, priority=5):
        sp = ECU_DEFS[src]['pos']
        dp = ECU_DEFS[dst]['pos']
        f  = CANFrame(src, dst, can_id, label, payload,
                      self._ts(), sp, dp)
        try:
            self.queue.put_nowait(f)
        except queue.Full:
            pass

        hex_s = ' '.join(f'{b:02X}' for b in payload)
        entry = f'{self._ts()} [{can_id}] {label:<18} {src}->{dst}  {hex_s}'
        self.log.append(entry)

        # Health update
        with self._lock:
            h = self.node_health[src]
            h['tx_count'] += 1
            h['last_seen'] = time.time()
            self._rate_cnt[src] += 1
            # Bus load: each frame is ~111 bits @ 500kbps = ~0.22ms
            h['bus_load_pct'] = min(99.0, h['bus_load_pct']*0.98 + 2.2)

        self.last_frame = dict(can_id=can_id, label=label,
                               payload=hex_s, src=src, dst=dst,
                               dlc=len(payload))

    # ── sensor loops ──────────────────────────────────────────────────────────
    def _loop_motor(self):
        """0x280 ECU→BCM  Motor RPM, Torque, Temp  @ 10ms"""
        while self._run:
            pt = self.pt
            rpm_raw  = int(np.clip(pt.rpm,   0, 65535))
            torq_raw = int(np.clip(pt.torque_nm * 10, 0, 65535))
            temp_raw = int(np.clip(pt.motor_temp + 40, 0, 255))
            pwr_raw  = int(np.clip(abs(pt.power_kw) * 10, 0, 65535))
            data = struct.pack('>HHBH', rpm_raw, torq_raw, temp_raw, pwr_raw)
            self._push('ECU','BCM','0x280','MOTOR_STATUS', data, 1)
            time.sleep(0.010)

    def _loop_battery(self):
        """0x3B0 BMS→BCM  SOC, Voltage, Current  @ 20ms"""
        while self._run:
            pt = self.pt
            soc_raw = int(np.clip(pt.battery_pct * 10, 0, 1000))
            v_raw   = int(np.clip(pt.inverter_v * 10,   0, 65535))
            i_raw   = int(np.clip(pt.current_a  * 10,   -32768, 32767))
            regen_r = int(np.clip(pt.regen_kw   * 100,  0, 65535))
            data = struct.pack('>HHhH', soc_raw, v_raw, i_raw, regen_r)
            self._push('BMS','BCM','0x3B0','BATTERY_DATA', data, 2)
            time.sleep(0.020)

    def _loop_drive(self):
        """0x2A0 TCM→BCM  Drive state, Regen  @ 20ms"""
        while self._run:
            pt = self.pt
            mode_map = {'PARK':0,'READY':1,'DRIVE':2,'SPORT':3,
                        'REGEN':4,'BRAKE':5,'FAULT':6}
            mode_byte = mode_map.get(pt.mode, 0)
            regen_raw = int(np.clip(pt.regen_kw * 100, 0, 65535))
            steer_raw = int(pt.steer * 2047 + 2048)
            data = struct.pack('>BBHH', mode_byte,
                               int(pt.throttle*255), regen_raw, steer_raw)
            self._push('TCM','BCM','0x2A0','DRIVE_STATE', data, 2)
            time.sleep(0.020)

    def _loop_vcm(self):
        """0x1A0 VCM→ALL  Vehicle speed, power limits  @ 20ms"""
        while self._run:
            pt = self.pt
            spd_raw  = int(np.clip(pt.speed_kmh * 100, 0, 65535))
            accel_r  = int(np.clip(pt.accel_ms2 * 1000, -32768, 32767))
            pwr_lim  = int(np.clip(pt.MAX_POWER_KW * (pt.battery_pct/100), 0,255))
            data = struct.pack('>HhB', spd_raw, accel_r, pwr_lim)
            self._push('VCM','BCM','0x1A0','VEHICLE_STATE', data, 2)
            time.sleep(0.020)

    def _loop_bcm(self):
        """0x350 BCM→ECU  Contactor state, odometer  @ 100ms"""
        odo = 0
        while self._run:
            odo += int(self.pt.speed_kmh / 36)   # rough metre increment
            contactor = 1 if self.pt.battery_pct > 5 else 0
            data = struct.pack('>IB', odo, contactor)
            self._push('BCM','ECU','0x350','BCM_STATUS', data, 4)
            time.sleep(0.100)

    def _loop_rate_decay(self):
        """Decay message rate counters → node glow intensity"""
        while self._run:
            with self._lock:
                for k in self._rate_cnt:
                    h = self.node_health[k]
                    h['msg_rate'] = (h['msg_rate']*0.6
                                     + self._rate_cnt[k]*0.4)
                    self._rate_cnt[k] = 0
                    # Check for node timeout (> 1s since last tx)
                    if time.time() - h['last_seen'] > 1.0:
                        if h['status'] == 'OK':
                            h['status'] = 'WARN'
                    else:
                        if h['status'] == 'WARN':
                            h['status'] = 'OK'
            time.sleep(0.10)

    # ── fault injection ───────────────────────────────────────────────────────
    def inject_fault(self):
        src, cid, code, desc = self._rng.choice(self.FAULTS)
        self.pt.fault_code = f'{code}: {desc}'
        self.active_faults.append(
            dict(ts=self._ts(), ecu=src, code=code, desc=desc))
        # Send DTC frame
        dtc_raw = int(code[1:].replace('A','10').replace('B','11'),
                      16) if code[1:].replace('A','10').replace('B','11').isalnum() else 0
        data = struct.pack('>BBH', 0x03, 0x7F, dtc_raw & 0xFFFF)
        self._push(src,'OBD', cid, f'DTC_{code}', data, 1)

    def clear_faults(self):
        self.active_faults.clear()
        self.pt.fault_code = None

    # ── manual OBD-style query ────────────────────────────────────────────────
    def obd_query(self):
        data = struct.pack('>BBB', 0x02, 0x01, 0x0C)
        self._push('OBD','ECU','0x7DF','OBD_RPM_REQ', data)
        # Simulated response after 5ms
        def _resp():
            time.sleep(0.005)
            rpm_bytes = int(self.pt.rpm).to_bytes(2, 'big')
            resp = struct.pack('>BBB', 0x04, 0x41, 0x0C) + rpm_bytes
            self._push('ECU','OBD','0x7E8','OBD_RPM_RESP', resp)
        threading.Thread(target=_resp, daemon=True).start()

    # ── start all loops ───────────────────────────────────────────────────────
    def start(self):
        for fn in [self._loop_motor, self._loop_battery, self._loop_drive,
                   self._loop_vcm,   self._loop_bcm,   self._loop_rate_decay]:
            threading.Thread(target=fn, daemon=True).start()

    def stop(self):
        self._run = False


# ─────────────────────────────────────────────────────────────────────────────
#  MODEL 3 WIREFRAME GEOMETRY  (returns Line visual batches)
# ─────────────────────────────────────────────────────────────────────────────
def build_car():
    pool = {}   # (rgb3, width) → [(p0,p1)]

    def seg(p0, p1, col, w=1.0):
        key = (rgb(col), round(w,1))
        pool.setdefault(key, []).append(
            (np.array(p0, np.float32), np.array(p1, np.float32)))

    W = 0.92
    spine = [
        (-2.35,-0.55),(-2.28,-0.08),(-1.82, 0.36),(-1.15, 0.60),
        ( 0.00, 0.66),( 0.92, 0.63),( 1.58, 0.14),( 2.12,-0.08),
        ( 2.35,-0.30),( 2.35,-0.55),(-2.35,-0.55),
    ]

    # Side silhouettes
    for ys in [-W, W]:
        pts = [np.array([x,ys,z]) for x,z in spine]
        for i in range(len(pts)-1):
            seg(pts[i], pts[i+1], BLUE, 1.8)

    # Cross ribs
    for i,(x,z) in enumerate(spine):
        seg([x,-W,z],[x,W,z], BLUE2, 1.0 if i in [0,9,10] else 0.55)

    # Roof  (stations 2-5, tapered)
    rst=[2,3,4,5]; rhw=[0.76,0.86,0.86,0.70]
    rl=[np.array([spine[i][0],-rhw[j],spine[i][1]]) for j,i in enumerate(rst)]
    rr=[np.array([spine[i][0], rhw[j],spine[i][1]]) for j,i in enumerate(rst)]
    for i in range(len(rl)-1):
        seg(rl[i],rl[i+1],CYAN,1.5); seg(rr[i],rr[i+1],CYAN,1.5)
        seg(rl[i],rr[i],CYAN2,0.9);  seg(rl[i+1],rr[i+1],CYAN2,0.9)
        for t in np.linspace(0.25,0.75,3):
            seg(rl[i]*(1-t)+rl[i+1]*t, rr[i]*(1-t)+rr[i+1]*t, GRIDCOL, 0.35)
        for ww in np.linspace(-0.45,0.45,3):
            seg([spine[rst[i]][0],ww,spine[rst[i]][1]],
                [spine[rst[i+1]][0],ww,spine[rst[i+1]][1]], GRIDCOL, 0.35)

    # Hood + Boot panels
    for st,hw in [([5,6,7],[0.70,0.82,0.84]),([0,1,2],[0.82,0.84,0.76])]:
        pl=[np.array([spine[i][0],-hw[j],spine[i][1]]) for j,i in enumerate(st)]
        pr=[np.array([spine[i][0], hw[j],spine[i][1]]) for j,i in enumerate(st)]
        for i in range(len(pl)-1):
            seg(pl[i],pl[i+1],BLUE,1.5); seg(pr[i],pr[i+1],BLUE,1.5)
            seg(pl[i],pr[i],BLUE2,0.8);  seg(pl[i+1],pr[i+1],BLUE2,0.8)
            for t in np.linspace(0.2,0.8,4):
                seg(pl[i]*(1-t)+pl[i+1]*t, pr[i]*(1-t)+pr[i+1]*t, GRIDCOL, 0.35)

    # Windscreens
    wsf=[np.array([spine[5][0],-0.70,spine[5][1]]),
         np.array([spine[5][0], 0.70,spine[5][1]]),
         np.array([spine[6][0], 0.84,spine[6][1]]),
         np.array([spine[6][0],-0.84,spine[6][1]])]
    for i in range(4): seg(wsf[i],wsf[(i+1)%4],CYAN,1.7)
    seg(wsf[0],wsf[2],GRIDCOL,0.4); seg(wsf[1],wsf[3],GRIDCOL,0.4)
    wsr=[np.array([spine[2][0],-0.74,spine[2][1]]),
         np.array([spine[2][0], 0.74,spine[2][1]]),
         np.array([spine[3][0], 0.86,spine[3][1]]),
         np.array([spine[3][0],-0.86,spine[3][1]])]
    for i in range(4): seg(wsr[i],wsr[(i+1)%4],CYAN,1.4)

    # Side windows
    for ys in [-W, W]:
        for sw in [
            [np.array([spine[5][0],ys,spine[5][1]]),
             np.array([spine[4][0],ys,spine[4][1]]),
             np.array([0.50,ys,0.60]),
             np.array([spine[6][0],ys,spine[6][1]])],
            [np.array([spine[3][0],ys,spine[3][1]]),
             np.array([spine[2][0],ys,spine[2][1]]),
             np.array([-1.55,ys,0.14]),
             np.array([-0.52,ys,0.58])],
        ]:
            for i in range(len(sw)):
                seg(sw[i],sw[(i+1)%len(sw)],CYAN,1.1)

    # Door lines
    for ys in [-W, W]:
        seg([ 0.50,ys,-0.10],[ 0.52,ys, 0.58], BLUE2, 1.0)
        seg([-2.18,ys,-0.43],[ 2.18,ys,-0.43], BLUE2, 0.8)
        seg([-2.10,ys, 0.08],[ 2.08,ys, 0.08], BLUE2, 0.6)
        for gx in np.linspace(-1.9,-0.1,5):
            seg([gx,ys,-0.43],[gx,ys,0.10], GRIDCOL, 0.35)
        for gx in np.linspace(0.6,2.1,4):
            seg([gx,ys,-0.43],[gx,ys,0.10], GRIDCOL, 0.35)

    # Floor / battery
    fc=[np.array([-2.20,-W,-0.43]),np.array([2.20,-W,-0.43]),
        np.array([2.20,W,-0.43]),  np.array([-2.20,W,-0.43])]
    for i in range(4): seg(fc[i],fc[(i+1)%4],BLUE2,0.7)
    for gx in np.linspace(-2.0,2.0,11):
        seg([gx,-W,-0.43],[gx,W,-0.43], GRIDCOL, 0.28)
    for gy in np.linspace(-W+0.1,W-0.1,7):
        seg([-2.0,gy,-0.43],[2.0,gy,-0.43], GRIDCOL, 0.28)

    # Wheels (all 4)
    def wheel(cx,cy,cz,r=0.37,tw=0.24,n=28):
        th=np.linspace(0,2*math.pi,n,endpoint=False)
        for side in [-tw/2,tw/2]:
            pts=[np.array([cx+r*math.cos(t),cy+side,cz+r*math.sin(t)]) for t in th]
            for i in range(n): seg(pts[i],pts[(i+1)%n],CYAN,1.8)
        for t in th[::2]:
            seg([cx+r*math.cos(t),cy-tw/2,cz+r*math.sin(t)],
                [cx+r*math.cos(t),cy+tw/2,cz+r*math.sin(t)],BLUE,0.5)
        for t in th[::n//6]:
            for side in [-tw/2,tw/2]:
                seg([cx,cy+side,cz],
                    [cx+r*0.88*math.cos(t),cy+side,cz+r*0.88*math.sin(t)],BLUE,1.2)
        for rr,col in [(0.12,BLUE2),(0.25,BLUE2),(0.33,CYAN)]:
            for side in [-tw/2,tw/2]:
                pts=[np.array([cx+rr*math.cos(t),cy+side,cz+rr*math.sin(t)]) for t in th]
                for i in range(n): seg(pts[i],pts[(i+1)%n],col,0.55)

    for wx,wy in [(-1.52,W+0.01),(-1.52,-W-0.01),(1.52,W+0.01),(1.52,-W-0.01)]:
        wheel(wx,wy,-0.18)
        apt=[np.array([wx+0.52*math.cos(t),wy,-0.18+0.44*math.sin(t)])
             for t in np.linspace(math.pi,2*math.pi,22)]
        for i in range(len(apt)-1): seg(apt[i],apt[i+1],CYAN,1.5)

    # Lights
    for ys in [-W+0.06, W-0.06]:
        sg = -1 if ys<0 else 1
        hl=[np.array([2.30,ys,0.04]),np.array([2.30,ys,-0.12]),
            np.array([1.98,ys+sg*0.04,-0.08]),np.array([1.98,ys+sg*0.04,0.08])]
        for i in range(4): seg(hl[i],hl[(i+1)%4],CYAN,2.2)
        seg([2.31,ys,0.0],[2.00,ys+sg*0.04,0.0],CYAN,1.5)
        tl=[np.array([-2.30,ys,0.02]),np.array([-2.30,ys,-0.15]),
            np.array([-2.08,ys+sg*0.04,-0.11]),np.array([-2.08,ys+sg*0.04,0.06])]
        for i in range(4): seg(tl[i],tl[(i+1)%4],RED,1.8)
    seg([-2.34,-W+0.06,-0.04],[-2.34,W-0.06,-0.04],RED,3.0)

    # CAN backbone (CAN-H / CAN-L)
    seg([-2.45,-0.05,-0.50],[2.45,-0.05,-0.50],BLUE,2.5)
    seg([-2.45, 0.05,-0.50],[2.45, 0.05,-0.50],BLUE,2.5)
    for sx in [-2.46,2.46]:
        seg([sx,-0.09,-0.50],[sx,0.09,-0.50],WARN,2.2)

    # ECU stub lines
    for ecu in ECU_DEFS.values():
        p=ecu['pos'].astype(np.float32)
        bus=np.array([p[0],0.0,-0.50],np.float32)
        for t in np.linspace(0,1,7)[:-1]:
            t1=t+0.06
            seg(p*(1-t)+bus*t, p*(1-t1)+bus*t1, DIMBLUE, 0.9)

    # Compile into batches
    batches=[]
    for (col3,w),pairs in pool.items():
        pos=np.empty((len(pairs)*2,3),np.float32)
        for i,(a,b) in enumerate(pairs):
            pos[2*i]=a; pos[2*i+1]=b
        batches.append(dict(pos=pos, color=col3+(1.0,), width=w))
    return batches


# ─────────────────────────────────────────────────────────────────────────────
#  HUD GEOMETRY HELPERS  (2D, y-down screen space)
# ─────────────────────────────────────────────────────────────────────────────
def arc_pts(cx, cy, r, a0, a1, n=48):
    """Arc from a0→a1 degrees, y-down."""
    pts=[]
    for i in range(n+1):
        a=math.radians(a0+(a1-a0)*i/n)
        pts.append([cx+r*math.cos(a), cy-r*math.sin(a)])
    return np.array(pts, np.float32)

def needle_pt(cx, cy, r, angle_deg):
    a=math.radians(angle_deg)
    return np.array([[cx, cy],
                     [cx+r*math.cos(a), cy-r*math.sin(a)]], np.float32)

def hbar_pts(x, y, w, h):
    return np.array([[x,y],[x+w,y],[x+w,y+h],[x,y+h],[x,y]], np.float32)


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN APPLICATION
# ─────────────────────────────────────────────────────────────────────────────
class App:
    PW = 400   # right-panel pixel width
    PH = 860   # panel height

    def __init__(self):
        self.pt    = Powertrain()
        self.can   = CANBus(self.pt)
        self.auto  = AutoDrive(self.pt)

        self._keys      = set()
        self._pkt       = None   # active CANFrame being animated
        self._last_t    = time.time()
        self._rng       = random.Random(9)

        # ── Canvas ────────────────────────────────────────────────────────────
        self.canvas = scene.SceneCanvas(
            title='CAN Bus 3D  |  Powertrain Sensor Network  |  ISO 11898',
            size=(1480, self.PH), bgcolor=BG,
            show=False, keys='interactive')
        self.canvas.events.key_press.connect(self._kp)
        self.canvas.events.key_release.connect(self._kr)

        # Grid: 3D left (3 parts) | HUD right (1 part)
        # Split canvas: 3D left (~75%), HUD panel right (~25%)
        # Using width_max on the right panel — works in all Vispy versions
        grid = self.canvas.central_widget.add_grid(spacing=0)
        self.vb3 = grid.add_view(row=0, col=0)
        self.vb3.camera = scene.cameras.TurntableCamera(
            elevation=18, azimuth=-48, distance=7.5, fov=38)
        self.vb2 = grid.add_view(row=0, col=1)
        self.vb2.camera = scene.cameras.PanZoomCamera(aspect=None)
        # Pin the right panel to exactly PW pixels wide
        self.vb2.width_max = self.PW
        self.vb2.width_min = self.PW

        self._build_3d()
        self._build_hud()

        self._timer = app.Timer(1/60, connect=self._tick, start=False)

    # ── 3D SCENE ──────────────────────────────────────────────────────────────
    def _build_3d(self):
        sc = self.vb3.scene

        # Car geometry
        for b in build_car():
            visuals.Line(pos=b['pos'], color=b['color'], width=b['width'],
                         connect='segments', antialias=True, parent=sc)

        # ECU markers
        npos = np.array([e['pos'] for e in ECU_DEFS.values()], np.float32)
        ncol = np.array([e['color'] for e in ECU_DEFS.values()], np.float32)
        self._ecu_mk = visuals.Markers(parent=sc)
        self._ecu_mk.set_data(pos=npos, face_color=ncol,
                               edge_color='white', edge_width=1.0,
                               size=15, symbol='disc')

        # ECU labels
        for name,e in ECU_DEFS.items():
            p=e['pos']
            visuals.Text(f'{name}\n{e["id"]}', color=e['color'],
                         font_size=7.5, pos=(p[0],p[1],p[2]+0.17),
                         anchor_x='center', parent=sc)

        # CAN bus labels
        for txt,pos,col in [
            ('CAN-H',(-2.6,-0.05,-0.44),BLUE),
            ('CAN-L',(-2.6, 0.05,-0.55),BLUE),
            ('120\u03a9',(-2.6,-0.05,-0.34),WARN),
            ('120\u03a9',( 2.3,-0.05,-0.34),WARN),
        ]:
            visuals.Text(txt,color=col,font_size=7,pos=pos,
                         anchor_x='center',parent=sc)

        # Packet dot + trail
        self._pkt_mk = visuals.Markers(parent=sc)
        self._pkt_mk.set_data(pos=np.array([[0,0,-9]],np.float32),
                               face_color=(0,0,0,0), size=1)
        self._trail_ln = visuals.Line(
            pos=np.zeros((2,3),np.float32), color=(0,0,0,0),
            width=2.5, connect='strip', antialias=True, parent=sc)

        # Stub highlight lines
        self._stubs = {}
        for name,e in ECU_DEFS.items():
            p=e['pos'].astype(np.float32)
            bus=np.array([p[0],0.0,-0.50],np.float32)
            sl=visuals.Line(pos=np.array([p,bus]),
                            color=rgb(DIMBLUE)+(0.5,),
                            width=1.0, connect='strip', parent=sc)
            self._stubs[name]=sl

    # ── HUD ───────────────────────────────────────────────────────────────────
    def _build_hud(self):
        sc  = self.vb2.scene
        PW, PH = self.PW, self.PH

        # Background
        visuals.Rectangle(center=(PW/2,PH/2), width=PW, height=PH,
                          color=PANEL, border_color=rgb(DIMBLUE),
                          border_width=1, parent=sc)

        # ── TITLE ─────────────────────────────────────────────────────────────
        visuals.Text('POWERTRAIN TELEMETRY', color=CYAN,
                     font_size=10, bold=True,
                     pos=(PW/2, 18), anchor_x='center', parent=sc)
        visuals.Text('ISO 11898  ·  500 kbps  ·  CAN 2.0B',
                     color=rgba(DIMBLUE,0.9), font_size=6.5,
                     pos=(PW/2,33), anchor_x='center', parent=sc)

        # ── SPEEDOMETER  (big arc gauge, 0-250 km/h) ─────────────────────────
        GCX, GCY, GR = PW//2, 115, 70
        # Background arc
        visuals.Line(pos=arc_pts(GCX,GCY,GR,220,-40),
                     color=rgba(DIMBLUE,0.7), width=10, parent=sc)
        # Tick marks
        for v_tick in range(0,251,50):
            frac = v_tick/250.0
            ang  = 220 - frac*260
            ar   = math.radians(ang)
            p0   = [GCX+(GR-4)*math.cos(ar), GCY-(GR-4)*math.sin(ar)]
            p1   = [GCX+(GR+4)*math.cos(ar), GCY-(GR+4)*math.sin(ar)]
            visuals.Line(pos=np.array([p0,p1],np.float32),
                         color=rgba(BLUE2,0.7), width=1.5, parent=sc)
            visuals.Text(str(v_tick), color=rgba(TEXT,0.55), font_size=5.5,
                         pos=(GCX+(GR+14)*math.cos(ar),
                               GCY-(GR+14)*math.sin(ar)),
                         anchor_x='center', parent=sc)
        self._spd_arc    = visuals.Line(
            pos=arc_pts(GCX,GCY,GR,220,220),
            color=GREEN, width=10, parent=sc)
        self._spd_needle = visuals.Line(
            pos=needle_pt(GCX,GCY,GR-12,220),
            color=CYAN, width=2.5, parent=sc)
        self._spd_txt = visuals.Text('0', color=GREEN, font_size=28,
                                      bold=True, pos=(GCX,GCY-8),
                                      anchor_x='center', parent=sc)
        visuals.Text('km/h', color=rgba(TEXT,0.6), font_size=8,
                     pos=(GCX,GCY+14), anchor_x='center', parent=sc)
        self._mode_txt = visuals.Text('PARK', color=rgba(DIMBLUE,1.0),
                                       font_size=9, bold=True,
                                       pos=(GCX,GCY+28),
                                       anchor_x='center', parent=sc)
        y = GCY + 55

        # ── RPM BAR ───────────────────────────────────────────────────────────
        y += 6
        visuals.Text('MOTOR RPM', color=rgba(TEXT,0.7), font_size=6.5,
                     pos=(14,y), anchor_x='left', parent=sc)
        self._rpm_txt = visuals.Text('0', color=CYAN, font_size=6.5,
                                      pos=(PW-14,y), anchor_x='right', parent=sc)
        y += 13
        visuals.Line(pos=hbar_pts(14,y,PW-28,9),
                     color=rgba(DIMBLUE,0.7), width=1, parent=sc)
        self._rpm_fill = visuals.Line(
            pos=hbar_pts(14,y,1,9), color=CYAN, width=7, parent=sc)
        y += 22

        # ── TORQUE BAR ────────────────────────────────────────────────────────
        visuals.Text('MOTOR TORQUE', color=rgba(TEXT,0.7), font_size=6.5,
                     pos=(14,y), anchor_x='left', parent=sc)
        self._torq_txt = visuals.Text('0 Nm', color=BLUE,
                                       font_size=6.5, pos=(PW-14,y),
                                       anchor_x='right', parent=sc)
        y += 13
        visuals.Line(pos=hbar_pts(14,y,PW-28,9),
                     color=rgba(DIMBLUE,0.7), width=1, parent=sc)
        self._torq_fill = visuals.Line(
            pos=hbar_pts(14,y,1,9), color=BLUE, width=7, parent=sc)
        y += 22

        # ── MOTOR TEMP BAR ────────────────────────────────────────────────────
        visuals.Text('MOTOR TEMP', color=rgba(TEXT,0.7), font_size=6.5,
                     pos=(14,y), anchor_x='left', parent=sc)
        self._temp_txt = visuals.Text('34°C', color=CYAN,
                                       font_size=6.5, pos=(PW-14,y),
                                       anchor_x='right', parent=sc)
        y += 13
        visuals.Line(pos=hbar_pts(14,y,PW-28,9),
                     color=rgba(DIMBLUE,0.7), width=1, parent=sc)
        self._temp_fill = visuals.Line(
            pos=hbar_pts(14,y,1,9), color=CYAN, width=7, parent=sc)
        y += 22

        # ── BATTERY SOC BAR ───────────────────────────────────────────────────
        visuals.Text('BATTERY SOC', color=rgba(TEXT,0.7), font_size=6.5,
                     pos=(14,y), anchor_x='left', parent=sc)
        self._batt_txt = visuals.Text('87.0%', color=GREEN,
                                       font_size=6.5, pos=(PW-14,y),
                                       anchor_x='right', parent=sc)
        y += 13
        visuals.Line(pos=hbar_pts(14,y,PW-28,9),
                     color=rgba(DIMBLUE,0.7), width=1, parent=sc)
        self._batt_fill = visuals.Line(
            pos=hbar_pts(14,y,1,9), color=GREEN, width=7, parent=sc)
        y += 22

        # ── POWER / REGEN BAR  (bidirectional, centre = 0) ───────────────────
        visuals.Text('NET POWER', color=rgba(TEXT,0.7), font_size=6.5,
                     pos=(14,y), anchor_x='left', parent=sc)
        self._pwr_txt = visuals.Text('0 kW', color=CYAN,
                                      font_size=6.5, pos=(PW-14,y),
                                      anchor_x='right', parent=sc)
        y += 13
        BW = PW-28
        cx_bar = 14 + BW//2
        visuals.Line(pos=hbar_pts(14,y,BW,9),
                     color=rgba(DIMBLUE,0.6), width=1, parent=sc)
        visuals.Line(pos=np.array([[cx_bar,y],[cx_bar,y+9]],np.float32),
                     color=rgba(BLUE2,0.5), width=1, parent=sc)
        self._pwr_fill = visuals.Line(
            pos=hbar_pts(cx_bar,y,1,9), color=GREEN, width=7, parent=sc)
        y += 24

        # ── SEPARATOR ─────────────────────────────────────────────────────────
        visuals.Line(pos=np.array([[10,y],[PW-10,y]],np.float32),
                     color=rgba(DIMBLUE,0.6), width=1, parent=sc)
        y += 10

        # ════════════════════════════════════════════════════════════════════
        #  ECU HEALTH DASHBOARD
        # ════════════════════════════════════════════════════════════════════
        visuals.Text('ECU HEALTH DASHBOARD', color=CYAN, font_size=9,
                     bold=True, pos=(PW/2,y), anchor_x='center', parent=sc)
        y += 18

        # Column headers
        for hdr,hx in [('NODE',12),('STATUS',88),('MSG/s',160),
                        ('BUS%',220),('TX',270),('ERR',320)]:
            visuals.Text(hdr, color=rgba(BLUE2,0.8), font_size=6,
                         pos=(hx,y), anchor_x='left', parent=sc)
        y += 14

        visuals.Line(pos=np.array([[10,y],[PW-10,y]],np.float32),
                     color=rgba(BLUE2,0.4), width=1, parent=sc)
        y += 6

        # One row per ECU node
        self._hrow = {}
        row_h = 30
        for name, ecu in ECU_DEFS.items():
            ry = y

            # Glow dot
            dot = visuals.Markers(parent=sc)
            dot.set_data(pos=np.array([[20, ry+8]],np.float32),
                          face_color=ecu['color'], size=10, symbol='disc')

            # Node name
            lbl = visuals.Text(name, color=ecu['color'],
                                font_size=7, bold=True,
                                pos=(34, ry+4), anchor_x='left', parent=sc)

            # CAN ID subtitle
            visuals.Text(ecu['id'], color=rgba(DIMBLUE,0.9),
                         font_size=5.5, pos=(34, ry+14),
                         anchor_x='left', parent=sc)

            # Status badge
            stat = visuals.Text('OK', color=GREEN, font_size=7,
                                  bold=True, pos=(90, ry+8),
                                  anchor_x='left', parent=sc)

            # Msg rate bar (small)
            visuals.Line(pos=hbar_pts(155,ry+4,55,10),
                         color=rgba(DIMBLUE,0.5), width=1, parent=sc)
            rate_fill = visuals.Line(
                pos=hbar_pts(155,ry+4,1,10), color=ecu['color'],
                width=6, parent=sc)
            rate_txt = visuals.Text('0', color=ecu['color'], font_size=6,
                                     pos=(156,ry+18), anchor_x='left',
                                     parent=sc)

            # Bus load %
            bus_txt = visuals.Text('0%', color=rgba(TEXT,0.7), font_size=6,
                                    pos=(222, ry+8), anchor_x='left',
                                    parent=sc)

            # TX count
            tx_txt = visuals.Text('0', color=rgba(TEXT,0.6), font_size=6,
                                   pos=(272, ry+8), anchor_x='left',
                                   parent=sc)

            # Error count
            err_txt = visuals.Text('0', color=rgba(RED,0.7), font_size=6,
                                    pos=(322, ry+8), anchor_x='left',
                                    parent=sc)

            # Heartbeat pulse ring
            hb_ring = visuals.Line(
                pos=arc_pts(20,ry+8,9,0,360,24),
                color=rgba(ecu['color'],0.0), width=1.5, parent=sc)

            self._hrow[name] = dict(
                dot=dot, stat=stat,
                rate_fill=rate_fill, rate_txt=rate_txt,
                bus_txt=bus_txt, tx_txt=tx_txt, err_txt=err_txt,
                hb_ring=hb_ring, hb_alpha=0.0, ry=ry)

            y += row_h

        y += 6
        visuals.Line(pos=np.array([[10,y],[PW-10,y]],np.float32),
                     color=rgba(DIMBLUE,0.6), width=1, parent=sc)
        y += 10

        # ── FAULT PANEL ───────────────────────────────────────────────────────
        visuals.Text('ACTIVE FAULTS  [F]=inject  [C]=clear',
                     color=RED, font_size=7,
                     pos=(14,y), anchor_x='left', parent=sc)
        y += 15
        self._fault_lines = []
        for i in range(3):
            t = visuals.Text('', color=RED, font_size=5.8,
                              pos=(14, y+i*14), anchor_x='left', parent=sc)
            self._fault_lines.append(t)
        y += 48

        # ── LAST CAN FRAME ────────────────────────────────────────────────────
        visuals.Text('LAST CAN FRAME', color=CYAN, font_size=7,
                     pos=(14,y), anchor_x='left', parent=sc)
        y += 14
        self._lf_id   = visuals.Text('ID: —',     color=GREEN,   font_size=6.5,
                                      pos=(14,y),  anchor_x='left', parent=sc)
        self._lf_lbl  = visuals.Text('Label: —',  color=CYAN,    font_size=6.5,
                                      pos=(14,y+13), anchor_x='left', parent=sc)
        self._lf_data = visuals.Text('Data: —',   color=rgba(TEXT,0.8),
                                      font_size=6.0, pos=(14,y+26),
                                      anchor_x='left', parent=sc)
        y += 44

        # ── CAN LOG ───────────────────────────────────────────────────────────
        visuals.Text('CAN BUS LOG', color=CYAN, font_size=7,
                     pos=(14,y), anchor_x='left', parent=sc)
        y += 14
        self._log_vis = []
        n_rows = max(1, (PH - y - 22) // 12)
        for i in range(n_rows):
            t = visuals.Text('', color=rgba(TEXT,0.75), font_size=5.5,
                              pos=(14, y+i*12), anchor_x='left', parent=sc)
            self._log_vis.append(t)

        # ── Controls hint ─────────────────────────────────────────────────────
        visuals.Text('[W/S] Throttle/Brake  [A/D] Steer  [M] Auto  [Q] Quit',
                     color=rgba(DIMBLUE,0.9), font_size=5.5,
                     pos=(PW/2, PH-8), anchor_x='center', parent=sc)

        # Store bar geometry constants
        self._BW = PW-28
        self._BX = 14
        self._cx_bar = 14 + (PW-28)//2

        # Set 2D camera
        self.vb2.camera.set_range(x=(0,PW), y=(0,PH))

    # ── HUD UPDATE ────────────────────────────────────────────────────────────
    def _update_hud(self):
        pt = self.pt
        BW = self._BW
        BX = self._BX

        # ── Speedometer ───────────────────────────────────────────────────────
        spd_frac = min(1.0, pt.speed_kmh / 250.0)
        a_end    = 220 - spd_frac * 260
        self._spd_arc.set_data(pos=arc_pts(self.PW//2, 115, 70, 220, a_end))
        spd_col  = (GREEN if pt.speed_kmh < 130 else
                    WARN  if pt.speed_kmh < 200 else RED)
        self._spd_arc.set_data(color=spd_col)
        self._spd_needle.set_data(
            pos=needle_pt(self.PW//2, 115, 58, a_end), color=CYAN)
        self._spd_txt.text  = f'{pt.speed_kmh:.0f}'
        self._spd_txt.color = spd_col

        mode_colors = {'PARK':DIMBLUE,'READY':BLUE,'DRIVE':GREEN,
                       'SPORT':WARN,'REGEN':CYAN,'BRAKE':RED,'FAULT':RED}
        self._mode_txt.text  = pt.mode
        self._mode_txt.color = mode_colors.get(pt.mode, TEXT)

        # ── RPM ───────────────────────────────────────────────────────────────
        rpm_frac = min(1.0, pt.rpm / pt.MAX_MOTOR_RPM)
        self._rpm_fill.set_data(pos=hbar_pts(BX,183,max(1,BW*rpm_frac),9))
        self._rpm_txt.text = f'{pt.rpm:,.0f} rpm'

        # ── Torque ────────────────────────────────────────────────────────────
        torq_frac = min(1.0, pt.torque_nm / pt.MAX_TORQUE_NM)
        torq_col  = (WARN if pt.torque_nm > 350 else BLUE)
        self._torq_fill.set_data(
            pos=hbar_pts(BX,218,max(1,BW*torq_frac),9), color=torq_col)
        self._torq_txt.text  = f'{pt.torque_nm:.0f} Nm'
        self._torq_txt.color = torq_col

        # ── Motor temp ────────────────────────────────────────────────────────
        tmp_frac = min(1.0, max(0,(pt.motor_temp-20)/120))
        tmp_col  = (RED  if pt.motor_temp > 115 else
                    WARN if pt.motor_temp > 85  else CYAN)
        self._temp_fill.set_data(
            pos=hbar_pts(BX,253,max(1,BW*tmp_frac),9), color=tmp_col)
        self._temp_txt.text  = f'{pt.motor_temp:.1f}\u00b0C'
        self._temp_txt.color = tmp_col

        # ── Battery ───────────────────────────────────────────────────────────
        batt_frac = max(0.005, pt.battery_pct/100)
        batt_col  = (RED  if pt.battery_pct < 10 else
                     WARN if pt.battery_pct < 25 else GREEN)
        self._batt_fill.set_data(
            pos=hbar_pts(BX,288,max(1,BW*batt_frac),9), color=batt_col)
        self._batt_txt.text  = f'{pt.battery_pct:.1f}%'
        self._batt_txt.color = batt_col

        # ── Net power (bidirectional bar) ─────────────────────────────────────
        cx  = self._cx_bar
        net = pt.power_kw - pt.regen_kw
        if net >= 0:
            w   = max(1, (BW//2) * min(1.0, net/258.0))
            self._pwr_fill.set_data(
                pos=hbar_pts(cx,311,w,9), color=GREEN)
            self._pwr_txt.text  = f'+{net:.0f} kW'
            self._pwr_txt.color = GREEN
        else:
            w   = max(1, (BW//2) * min(1.0, pt.regen_kw/80.0))
            self._pwr_fill.set_data(
                pos=hbar_pts(cx-w,311,w,9), color=CYAN)
            self._pwr_txt.text  = f'-{pt.regen_kw:.0f} kW regen'
            self._pwr_txt.color = CYAN

        # ── ECU Health rows ───────────────────────────────────────────────────
        for name, row in self._hrow.items():
            h  = self.can.node_health[name]
            ecu= ECU_DEFS[name]

            # Status
            status = h['status']
            faulted= any(f['ecu']==name for f in self.can.active_faults)
            if faulted:
                row['stat'].text  = 'FAULT'
                row['stat'].color = RED
                row['dot'].set_data(
                    pos=np.array([[20,row['ry']+8]],np.float32),
                    face_color=RED, size=10)
            elif status == 'WARN':
                row['stat'].text  = 'WARN'
                row['stat'].color = WARN
                row['dot'].set_data(
                    pos=np.array([[20,row['ry']+8]],np.float32),
                    face_color=WARN, size=10)
            else:
                row['stat'].text  = 'OK'
                row['stat'].color = GREEN
                # Glow intensity ∝ message rate
                rate  = min(1.0, h['msg_rate'] / 60.0)
                glow  = tuple(np.clip(np.array(ecu['color'][:3])*(0.3+rate*0.7),0,1))+(1.0,)
                sz    = 8 + rate*7
                row['dot'].set_data(
                    pos=np.array([[20,row['ry']+8]],np.float32),
                    face_color=glow, size=sz)

            # Message rate bar
            rate_frac = min(1.0, h['msg_rate'] / 120.0)
            row['rate_fill'].set_data(
                pos=hbar_pts(155, row['ry']+4, max(1,55*rate_frac), 10))
            row['rate_txt'].text = f"{h['msg_rate']:.0f}/s"

            # Bus load
            row['bus_txt'].text = f"{h['bus_load_pct']:.0f}%"

            # TX count (abbreviated)
            tx = h['tx_count']
            row['tx_txt'].text = (f'{tx//1000}k' if tx > 9999 else str(tx))

            # Error count
            row['err_txt'].text = str(h['error_count'])
            if h['error_count'] > 0:
                row['err_txt'].color = RED

            # Heartbeat ring pulse (fade in on tx, decay)
            row['hb_alpha'] = min(1.0, row['hb_alpha'] + h['msg_rate']*0.001)
            row['hb_alpha'] *= 0.92
            row['hb_ring'].set_data(
                color=rgb(ecu['color'])+(row['hb_alpha'],))

        # ── Faults ────────────────────────────────────────────────────────────
        faults = self.can.active_faults[-3:]
        for i,fl in enumerate(self._fault_lines):
            fl.text = (f"[{faults[i]['ts']}] {faults[i]['ecu']} "
                       f"{faults[i]['code']}: {faults[i]['desc']}"
                       if i < len(faults) else '')

        # ── Last CAN frame ────────────────────────────────────────────────────
        lf = self.can.last_frame
        if lf:
            self._lf_id.text   = f"ID: {lf['can_id']}  DLC: {lf['dlc']}"
            self._lf_lbl.text  = f"Label: {lf['label']}"
            self._lf_data.text = f"Data: {lf['payload'][:36]}"

        # ── CAN log ───────────────────────────────────────────────────────────
        log = list(self.can.log)
        for i,lv in enumerate(self._log_vis):
            idx = len(log) - 1 - i
            if idx >= 0:
                entry = log[idx]
                col = TEXT
                for name,e in ECU_DEFS.items():
                    if name in entry: col = e['color']; break
                lv.text  = entry[:52]
                lv.color = rgba(col, 0.8)
            else:
                lv.text = ''

    # ── PACKET ANIMATION ──────────────────────────────────────────────────────
    def _pkt_pos(self, f, t):
        sp,dp = f.src_pos, f.dst_pos
        bs=np.array([sp[0],0.0,-0.50]); bd=np.array([dp[0],0.0,-0.50])
        wps=[sp,bs,bd,dp]
        lens=[np.linalg.norm(wps[i+1]-wps[i]) for i in range(3)]
        total=sum(lens) or 1e-6
        d=t*total; acc=0.0
        for i in range(3):
            if d<=acc+lens[i]:
                a=(d-acc)/lens[i] if lens[i]>0 else 0
                return wps[i]+a*(wps[i+1]-wps[i])
            acc+=lens[i]
        return dp

    def _flash_stub(self, name):
        if name in self._stubs:
            self._stubs[name].set_data(color=rgb(CYAN)+(1.0,), width=2.8)

    def _reset_stubs(self):
        for sl in self._stubs.values():
            sl.set_data(color=rgb(DIMBLUE)+(0.45,), width=1.0)

    # ── PHYSICS KEYS ──────────────────────────────────────────────────────────
    def _apply_keys(self, dt):
        if self.auto._running: return
        k = self._keys
        # Throttle
        if 'W' in k or 'Up' in k:
            self.pt.throttle   = min(1.0, self.pt.throttle + 2.2*dt)
            self.pt.brake_pct  = max(0.0, self.pt.brake_pct - 5.0*dt)
        else:
            self.pt.throttle   = max(0.0, self.pt.throttle - 1.8*dt)
        # Brake
        if 'S' in k or 'Down' in k:
            self.pt.brake_pct  = min(1.0, self.pt.brake_pct + 3.5*dt)
            self.pt.throttle   = max(0.0, self.pt.throttle - 4.0*dt)
        else:
            self.pt.brake_pct  = max(0.0, self.pt.brake_pct - 3.0*dt)
        # Steer
        if 'A' in k or 'Left' in k:
            self.pt.steer = max(-1.0, self.pt.steer - 2.5*dt)
        elif 'D' in k or 'Right' in k:
            self.pt.steer = min( 1.0, self.pt.steer + 2.5*dt)
        else:
            self.pt.steer *= max(0.0, 1.0 - 5.0*dt)

    # ── KEY EVENTS ────────────────────────────────────────────────────────────
    def _kp(self, ev):
        k = ev.key.name if hasattr(ev.key,'name') else str(ev.key)
        self._keys.add(k)
        kl = k.lower()
        if kl == 'm':
            self.auto.toggle()
        elif kl == 'f':
            self.can.inject_fault()
        elif kl == 'c':
            self.can.clear_faults()
        elif kl == 'o':
            self.can.obd_query()
        elif kl == 'r':
            self.vb3.camera.elevation=18
            self.vb3.camera.azimuth=-48
            self.vb3.camera.distance=7.5
        elif kl in ('q','escape'):
            self.can.stop(); self.auto.stop()
            self.canvas.app.quit()

    def _kr(self, ev):
        k = ev.key.name if hasattr(ev.key,'name') else str(ev.key)
        self._keys.discard(k)

    # ── MAIN TICK ─────────────────────────────────────────────────────────────
    def _tick(self, _):
        now = time.time()
        dt  = min(now - self._last_t, 0.05)
        self._last_t = now

        self._apply_keys(dt)
        self.pt.update(dt)

        # Poll CAN queue
        try:
            while True:
                frame = self.can.queue.get_nowait()
                self._flash_stub(frame.src)
                self._flash_stub(frame.dst)
                self._pkt = frame
                self._pkt.t = 0.0
                self._pkt.trail = []
        except queue.Empty:
            pass

        # Animate packet
        if self._pkt is not None:
            p = self._pkt
            p.t = min(p.t + dt*2.4, 1.3)
            pos = self._pkt_pos(p, min(p.t,1.0)).astype(np.float32)
            self._pkt_mk.set_data(pos=pos.reshape(1,3),
                                   face_color=CYAN, size=13, symbol='disc')
            p.trail.append(pos)
            tr = np.array(p.trail[-22:], np.float32)
            if len(tr) >= 2:
                alphas = np.linspace(0.0, 0.8, len(tr))
                cols   = np.zeros((len(tr),4), np.float32)
                cols[:,:3] = np.array(CYAN[:3])
                cols[:,3]  = alphas
                self._trail_ln.set_data(pos=tr, color=cols)
            if p.t >= 1.3:
                self._pkt_mk.set_data(
                    pos=np.array([[0,0,-9]],np.float32),
                    face_color=(0,0,0,0), size=1)
                self._trail_ln.set_data(
                    pos=np.zeros((2,3),np.float32), color=(0,0,0,0))
                self._pkt = None
                self._reset_stubs()

        self._update_hud()
        self.canvas.update()

    # ── RUN ───────────────────────────────────────────────────────────────────
    def run(self):
        self.can.start()
        self.canvas.show()
        self._timer.start()
        app.run()


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("""
  +==============================================================+
  |  CAN Bus 3D  |  Powertrain Sensor Network  |  Vispy/OpenGL  |
  |  ISO 11898   |  500 kbps  |  Model 3 Blueprint              |
  +==============================================================+

  Powertrain CAN frames (live):
    ECU  0x280  Motor RPM · Torque · Temp · Power    @ 10ms
    BMS  0x3B0  Battery SOC · HV Voltage · Current   @ 20ms
    TCM  0x2A0  Drive mode · Regen kW · Steer angle  @ 20ms
    VCM  0x1A0  Vehicle speed · Accel · Power limit  @ 20ms
    BCM  0x350  HV contactor · Odometer              @ 100ms

  Controls:
    W/S   Throttle / Brake        M   Toggle auto-drive
    A/D   Steer left / right      F   Inject fault DTC
    Mouse Rotate 3D view          C   Clear faults
    R     Reset camera            O   OBD-II query
    Q     Quit

""")
    App().run()