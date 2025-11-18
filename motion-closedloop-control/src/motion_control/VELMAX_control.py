# ===== VELMAX_control_new.py with State Persistence =====
# Features:
# - State persistence: save/load rotation angle and linear position
# - Safe window protection: auto-correct if rotary angle exceeds limit on boot
# - Normalized angles: always store in [-180°, 180°)

import time
import serial
import json
import os

# ---------- STATE PERSISTENCE ----------
# State file location: project-local as requested (Windows path)
# Example: D:\\adaptiveSPECT\\python_code\\.vxm_state.json
STATE_PATH = r"D:\adaptiveSPECT\python_code\.vxm_state.json"

def _atomic_write(path: str, data: dict):
    """Atomically write state dict to file using temp file + rename."""
    try:
        # Ensure directory exists to avoid write errors on first save
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, path)
    except Exception as e:
        print(f"[WARN] state write failed: {e}")

def load_state(path: str = STATE_PATH) -> dict:
    """Load state from file; return defaults if missing."""
    try:
        if os.path.exists(path):
            with open(path, "r") as f:
                return json.load(f)
    except Exception as e:
        print(f"[WARN] state load failed: {e}")
    return {"rot_deg": 0.0, "lin_mm": 0.0, "ts": 0}

def save_state(rot_deg: float, lin_mm: float, path: str = STATE_PATH):
    """Save state: normalize rotation to (-180, 180], persist to file."""
    rot_deg = norm180(float(rot_deg))
    _atomic_write(path, {"rot_deg": rot_deg, "lin_mm": float(lin_mm), "ts": time.time()})

# ---------- USER CONFIG ----------
COM_PORT = "COM4"
BAUD = 9600

# ROTARY (motor 1)
ROT_MOTOR = 1
ROT_MICROSTEP = 16                # microstepping
ROT_GEAR = 9                     # gearbox ratio
ROT_STEPS_PER_REV = 200 * ROT_MICROSTEP * ROT_GEAR   # steps per 360°
ROT_DIR_LIMIT_DEG = 181.0         # direction-accum limit (safe window)

# LINEAR (motor 2)
LIN_MOTOR = 2
LIN_MICROSTEP = 16
LIN_LEAD_MM_PER_REV = 5.08     
LIN_STEPS_PER_REV = 200 * LIN_MICROSTEP
LIN_STEPS_PER_MM  = LIN_STEPS_PER_REV / LIN_LEAD_MM_PER_REV
LIN_SOFT_MIN_MM, LIN_SOFT_MAX_MM = 0.0, 200.0       

# ---------- SERIAL OPEN/CLOSE ----------
def open_vxm(port=COM_PORT, baud=BAUD, timeout=1.0):
    ser = serial.Serial(
        port=port, baudrate=baud,
        bytesize=8, parity='N', stopbits=1,
        timeout=timeout, write_timeout=1.0
    )
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(b'F\r')   # ONLINE
    ser.write(b'V\r')   # probe once (R/B)
    ser.write(b'C\r')   # CLEAR
    ser.write(b'N\r')   # NULL abs registers (soft zero)
    return ser

def close_vxm(ser):
    try:
        ser.write(b'Q\r')
    finally:
        ser.close()

# ---------- WAIT (R/B POLLING) ----------
def wait_motion_complete(ser, timeout_s=600.0, poll_s=0.12):
    """
    Robust motion complete wait:
      - Send V\r poll; accept both 'R' (ready) or '^' (program end)
      - Allow empty reads b'' (device not ready yet, keep polling)
      - Timeout configurable (default 10 minutes)
    """
    t0 = time.time()

    # Clear residual echoes
    try:
        while ser.in_waiting:
            ser.read(ser.in_waiting)
    except Exception:
        pass

    last = b''
    while True:
        try:
            ser.write(b'V\r')
        except Exception:
            pass
        # Give controller time to respond
        time.sleep(0.05)
        try:
            chunk = ser.read(32)
        except Exception:
            chunk = b''
        if chunk:
            last = chunk
            # Motion complete: Ready or program-end symbol
            if (b'R' in chunk) or (b'^' in chunk):
                return True
            # b'B' means busy, continue polling
        # Empty read is OK, just keep waiting
        if time.time() - t0 > timeout_s:
            raise TimeoutError(f"WAIT complete TIMEOUT; last={last!r}")
        time.sleep(poll_s)


def estimate_timeout_for_linear(steps: int, s2m_speed=700, accel_margin=8.0):
    """
    Rough estimate of linear move duration (seconds):
      steps / speed + margin for accel/decel
    - s2m_speed: typical step/sec (adjust to match your S2M setting)
    - accel_margin: safety buffer (seconds)
    """
    steps = abs(int(steps))
    if s2m_speed <= 0:
        return 180.0
    return steps / float(s2m_speed) + accel_margin


def wait_done(ser, timeout=120.0, poll=0.05):
    """Backward-compatible wait; redirects to wait_motion_complete."""
    return wait_motion_complete(ser, timeout_s=timeout, poll_s=max(0.08, poll))


##def wait_for_caret(ser, timeout=120.0):
    """Wait until the controller sends a '^' (program end)."""
    t0 = time.time()
    while True:
        b = ser.read(1)
        if b == b'^':
            return True
        if time.time() - t0 > timeout:
            raise TimeoutError("WAIT '^' TIMEOUT")
        time.sleep(0.01)

# ---------- LOW-LEVEL MOVES ----------
def move_inc_steps(ser, m: int, dsteps: int, timeout: float = None):
    """Send incremental move command and wait for completion."""
    ser.write(b'C\r')
    sign = b'+' if dsteps >= 0 else b''
    cmd = b'I' + str(m).encode() + b'M' + sign + str(int(dsteps)).encode() + b',R'
    # debug: print command being sent
    try:
        print(f"[TX] -> {repr(cmd + b'\\r')}")
    except Exception:
        print(f"[TX] -> {cmd!r} + b'\\r'")
    # print serial metadata for debugging
    try:
        print(f"[SER] port={getattr(ser,'port',None)}, baud={getattr(ser,'baudrate',None)}, in_waiting_before={ser.in_waiting}")
    except Exception:
        pass
    ser.write(cmd + b'\r')
    # small delay then print any immediate echo/error bytes
    time.sleep(0.02)
    try:
        n = ser.in_waiting
        if n:
            data = ser.read(n)
            print(f"[ECHO immediate] {data!r}")
    except Exception:
        pass
    # pass through timeout to wait_done if provided
    try:
        if timeout is None:
            wait_done(ser)
        else:
            wait_done(ser, timeout=timeout)
    except Exception:
        raise



# ---------- CONVERSIONS ----------
def norm180(deg: float) -> float:
    """Normalize angle to (-180, 180]."""
    return ((deg + 180.0) % 360.0) - 180.0

def deg_to_steps(deg: float) -> int:
    """Convert degrees to step count."""
    return int(round(deg / 360.0 * ROT_STEPS_PER_REV))

def mm_to_steps(mm: float) -> int:
    """Convert millimeters to step count."""
    return int(round(mm * LIN_STEPS_PER_MM))

# ---------- THIN WRAPPERS ----------
class RotaryAxisFlat:
    """
    ROTARY axis (#1): professor-style wrapper + state persistence.
    - Maintains normalized angle in (-180, 180]
    - On boot: restores from state file, applies safe-window protection if needed
    - After each move: saves state to disk
    """
    def __init__(self, ser, motor=ROT_MOTOR, dir_limit_deg=ROT_DIR_LIMIT_DEG):
        self.ser = ser; self.m = motor
        self.dir_limit = float(dir_limit_deg)

        # Load and restore state
        st = load_state()
        self.curr_deg = norm180(float(st.get("rot_deg", 0.0)))
        
        # Boot-time safe-window protection: if angle exceeds limit, correct by ±360°
        if abs(self.curr_deg) > self.dir_limit:
            corr = -360.0 if self.curr_deg > 0 else 360.0
            print(f"[ROT][BOOT PROTECT] detected angle {self.curr_deg:+.3f}° out of safe window, jog {corr:+.1f}°")
            move_inc_steps(self.ser, self.m, deg_to_steps(corr))
            self.curr_deg = norm180(self.curr_deg + corr)
            # Persist corrected state
            s = load_state()
            save_state(self.curr_deg, s["lin_mm"])
            print(f"[ROT][BOOT PROTECT] corrected → {self.curr_deg:+.3f}°")

    def set_zero(self):
        """Manually reset to zero (optional)."""
        self.curr_deg = 0.0
        s = load_state()
        save_state(self.curr_deg, s["lin_mm"])
        print("[ROT] software zero → 0.0°")

    def home_if_possible(self):
        """Placeholder: no mechanical homing; use set_zero if needed."""
        self.set_zero()
        print("[ROT] homing: NO LIMIT → software zero")

    def _protect_and_move(self, delta_deg: float):
        """
        Safe window protection: if predicted angle exceeds limit,
        first jog ±360°, then apply the requested delta.
        """
        predicted = self.curr_deg + float(delta_deg)

        # Safe-window protection: if predicted exceeds limit, wrap by ±360°
        if abs(predicted) > self.dir_limit:
            corr = -360.0 if predicted > 0 else 360.0
            print(f"[ROT] PROTECT: predicted {predicted:+.3f}° exceeds limit, jog {corr:+.1f}°")
            move_inc_steps(self.ser, self.m, deg_to_steps(corr))
            self.curr_deg = norm180(self.curr_deg + corr)
            print(f"[ROT] PROTECT: after wrap → {self.curr_deg:+.3f}°")

        # Real move
        move_inc_steps(self.ser, self.m, deg_to_steps(delta_deg))
        self.curr_deg = norm180(self.curr_deg + delta_deg)

        # Persist state: always save normalized angle in (-180, 180]
        s = load_state()
        save_state(self.curr_deg, s["lin_mm"])

    def goto_deg(self, target_deg: float):
        """Move to target angle (safe-window wrapped to (-180, 180])."""
        target_norm = norm180(float(target_deg))
        delta = target_norm - self.curr_deg
        self._protect_and_move(delta)
        print(f"[ROT] goto {self.curr_deg:+.3f}°")

class LinearAxisFlat:
    """
    LINEAR axis (#2): professor-style wrapper + soft limits + state persistence.
    - Uses incremental moves only (safe given no hardware limits)
    - On boot: restores from state file
    - After each move: saves state to disk
    """
    def __init__(self, ser, motor=LIN_MOTOR,
                 soft_min=LIN_SOFT_MIN_MM, soft_max=LIN_SOFT_MAX_MM):
        self.ser = ser
        self.m = motor
        self.soft_min = float(soft_min)
        self.soft_max = float(soft_max)
        
        # Load and restore state
        st = load_state()
        self.curr_mm = float(st.get("lin_mm", 0.0))

    def set_zero(self):
        """Manually reset to zero (optional)."""
        self.curr_mm = 0.0
        s = load_state()
        save_state(s["rot_deg"], self.curr_mm)
        print("[LIN] software zero → 0.0 mm")

    def home_if_possible(self):
        """Placeholder: no mechanical homing; use set_zero if needed."""
        self.set_zero()
        print("[LIN] homing: NO LIMIT → software zero")

    def goto_mm(self, target_mm: float):
        """Move to target position using incremental moves."""
        if not (self.soft_min <= target_mm <= self.soft_max):
            raise ValueError(
                f"TARGET {target_mm:.3f} mm OUT OF SOFT RANGE [{self.soft_min},{self.soft_max}]"
            )
        # Compute delta and convert to steps
        target_mm = float(target_mm)
        curr_mm = float(self.curr_mm)
        delta_mm = target_mm - curr_mm
        dsteps = mm_to_steps(delta_mm)
        print(f"[LIN DEBUG] target_mm={target_mm:.6f}, curr_mm={curr_mm:.6f}, delta_mm={delta_mm:+.6f}, dsteps={dsteps}")
        if dsteps == 0:
            print(f"[LIN] (noop) already at {self.curr_mm:.3f} mm")
            return
        # Clear serial input buffer to avoid leftover characters interfering with wait
        try:
            before = self.ser.in_waiting
            print(f"[LIN DEBUG] in_waiting before reset={before}")
            self.ser.reset_input_buffer()
            after = self.ser.in_waiting
            print(f"[LIN DEBUG] in_waiting after reset={after}")
        except Exception:
            pass
        # Send incremental move with estimated timeout
        est_to = estimate_timeout_for_linear(dsteps)
        print(f"[LIN DEBUG] using timeout={est_to:.1f}s for this move")
        move_inc_steps(self.ser, self.m, dsteps, timeout=est_to)
        # Update software position
        self.curr_mm = target_mm
        print(f"[LIN] goto {self.curr_mm:.3f} mm (moved {delta_mm:+.3f} mm -> {dsteps} steps)")
        
        # Persist state: only save mm after successful move
        s = load_state()
        save_state(s["rot_deg"], self.curr_mm)

# ---------- UPPER LAYER ----------
class PositionPlanner:
    """
    Manage two axes (rot + lin) and map (soft_idx, angle_idx, radius_idx) → (deg, mm)
    Default: 4 * 12 * 5 = 240 points
    """
    def __init__(
        self,
        rot: RotaryAxisFlat,
        lin: LinearAxisFlat,
        soft_zeros_deg=None,
        angle_step_deg: float = 30.0,
        num_angles: int = 12,
        radii_mm=None,
    ):
        self.rot = rot
        self.lin = lin

        if soft_zeros_deg is None:
            soft_zeros_deg = [0.0, 90.0, 180.0, 270.0]
        self.soft_zeros = list(soft_zeros_deg)

        self.angle_step = float(angle_step_deg)
        self.num_angles = int(num_angles)

        if radii_mm is None:
            radii_mm = [0.0, 25.0, 50.0, 75.0, 100.0]
        self.radii = list(radii_mm)

    def _check_indices(self, soft_idx: int, angle_idx: int, radius_idx: int):
        if not (0 <= soft_idx < len(self.soft_zeros)):
            raise ValueError(f"soft_idx {soft_idx} out of range 0~{len(self.soft_zeros)-1}")
        if not (0 <= angle_idx < self.num_angles):
            raise ValueError(f"angle_idx {angle_idx} out of range 0~{self.num_angles-1}")
        if not (0 <= radius_idx < len(self.radii)):
            raise ValueError(f"radius_idx {radius_idx} out of range 0~{len(self.radii)-1}")

    def goto_point(self, soft_idx: int, angle_idx: int, radius_idx: int):
        """
        Move to point (soft_idx, angle_idx, radius_idx).
        ARGS:
        - soft_idx  : 0~3  (select one of soft zeros)
        - angle_idx : 0~11 (each step = angle_step_deg)
        - radius_idx: 0~4  (select one radius in mm list)
        """
        self._check_indices(soft_idx, angle_idx, radius_idx)

        base_deg = self.soft_zeros[soft_idx]
        add_deg  = angle_idx * self.angle_step
        target_deg = base_deg + add_deg

        target_mm = self.radii[radius_idx]

        print(f"[PLAN] goto S{soft_idx} / A{angle_idx} / R{radius_idx} → rot={target_deg:.2f}°, lin={target_mm:.2f} mm")
        # ORDER: rotate first, then linear
        self.rot.goto_deg(target_deg)
        self.lin.goto_mm(target_mm)

    def scan_all(self):
        """Scan all points: soft zero → angle → radius."""
        for s in range(len(self.soft_zeros)):
            for a in range(self.num_angles):
                for r in range(len(self.radii)):
                    self.goto_point(s, a, r)

# ---------- DEMO MAIN ----------
if __name__ == "__main__":
    ser = open_vxm()
    try:
        # init two axes
        rot = RotaryAxisFlat(ser, motor=ROT_MOTOR, dir_limit_deg=ROT_DIR_LIMIT_DEG)
        lin = LinearAxisFlat(ser, motor=LIN_MOTOR,
                             soft_min=LIN_SOFT_MIN_MM, soft_max=LIN_SOFT_MAX_MM)

        # NOTE: Do NOT call set_zero() here unless you want to reset.
        # State is automatically restored from disk on init.
        # Uncomment below only if you want to force a fresh start:
        # rot.set_zero()
        # lin.set_zero()

        # planner (4*12*5)
        planner = PositionPlanner(
            rot=rot,
            lin=lin,
            soft_zeros_deg=[0.0, 90.0, 180.0, 270.0],
            angle_step_deg=30.0,
            num_angles=12,
            radii_mm=[0.0, 25.0, 50.0, 75.0, 100.0],
        )

        # example point: S1/A3/R2 → 180°, 50 mm
        planner.goto_point(soft_idx=1, angle_idx=3, radius_idx=2)

        # long sweep (use with caution)
        # planner.scan_all()

        print("[INFO] Motion complete.")
    finally:
        close_vxm(ser)
