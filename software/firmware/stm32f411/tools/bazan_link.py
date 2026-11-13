"""Talk to the Mico MCU over USB or UART: check it, watch it, log it, run the P0 bench test, calibrate.

    pip install pyserial
    python mico_link.py --port COM5 info                 # Windows; on macOS e.g. /dev/tty.usbmodem2101
    python mico_link.py --port COM5 monitor --csv run.csv
    python mico_link.py --port COM5 stand                 # enable and move to the stand pose over 2 s
    python mico_link.py --port COM5 p0-thermal --joint l_hip_roll --lift-deg 10 --csv p0_left.csv
    python mico_link.py --port COM5 calibrate --joint l_knee
    python mico_link.py --port COM5 clear | disable
    python mico_link.py --port COM5 diag                  # 1 Hz timing / CPU / stack / heap report
    python mico_link.py --port COM5 m0-accept --minutes 10 --report m0_accept.json   # mico_mcu_m0test image

Frames: COBS(type, seq, payload, crc16-ccitt LE) + 0x00, as in lib/core/link/frame.cpp. Message layouts come from
mico_protocol.py, generated together with the firmware headers.
"""
import argparse
import csv
import math
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mico_protocol as P  # noqa: E402

T = {name: tid for name, (tid, _, _) in P.MESSAGES.items() if tid is not None}


# ---------------------------------------------------------------- framing
def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out, block = bytearray(), bytearray()
    for b in data:
        if b == 0:
            out += bytes([len(block) + 1]) + block
            block = bytearray()
        else:
            block.append(b)
            if len(block) == 254:
                out += b"\xff" + block
                block = bytearray()
    return bytes(out + bytes([len(block) + 1]) + block)


def cobs_decode(data):
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError("bad COBS")
        out += data[i + 1:i + code]
        i += code
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def pack(name, **fields):
    _, fmt, layout = P.MESSAGES[name]
    vals = []
    for fn, n in layout:
        v = fields[fn]
        vals += list(v) if n > 1 else [v]
    return struct.pack(fmt, *vals)


def unpack(name, payload):
    _, fmt, layout = P.MESSAGES[name]
    vals, out, i = struct.unpack(fmt, payload[:struct.calcsize(fmt)]), {}, 0
    for fn, n in layout:
        out[fn] = list(vals[i:i + n]) if n > 1 else vals[i]
        i += n
    return out


class Link:
    def __init__(self, port):
        import serial          # pyserial
        self.ser = serial.Serial(port, 921600, timeout=0.01)
        self.buf = bytearray()
        self.seq = 0
        self.lock = threading.Lock()

    def send(self, name, payload=b""):
        raw = bytes([T[name], self.seq & 0xFF]) + payload
        raw += struct.pack("<H", crc16(raw))
        self.seq += 1
        with self.lock:
            self.ser.write(cobs_encode(raw) + b"\x00")

    def frames(self):
        """Yield (type, payload) for every valid frame received, forever."""
        while True:
            self.buf += self.ser.read(512)
            while b"\x00" in self.buf:
                chunk, _, self.buf = self.buf.partition(b"\x00")
                if not chunk:
                    continue
                try:
                    raw = cobs_decode(bytes(chunk))
                except ValueError:
                    continue
                if len(raw) < 4 or crc16(raw[:-2]) != struct.unpack("<H", raw[-2:])[0]:
                    continue
                yield raw[0], raw[2:-2]
            yield None, None

    def states(self):
        for t, p in self.frames():
            if t == T["State"]:
                yield decode_state(p)
            elif t == T["Log"]:
                print(f"[mcu] {p[1:].decode(errors='replace')}")

    def wait(self, name, timeout=1.0):
        end = time.time() + timeout
        for t, p in self.frames():
            if t == T.get(name):
                return p
            if time.time() > end:
                return None

    def traj(self, points, replace=False):
        """points: [(dt_ms, [q x NJ] rad)], sent in chunks of TRAJ_MAX_POINTS."""
        for k in range(0, len(points), P.TRAJ_MAX_POINTS):
            chunk = points[k:k + P.TRAJ_MAX_POINTS]
            body = pack("Traj", flags=1 if (replace and k == 0) else 0, n=len(chunk))
            for dt, q in chunk:
                body += pack("TrajPoint", dt_ms=int(dt), q=[int(round(v * P.Q_SCALE)) for v in q])
            self.send("Traj", body)


def decode_state(p):
    s = unpack("State", p)
    nan = float("nan")
    s["mode_name"] = P.MODES[s["mode"]] if s["mode"] < len(P.MODES) else "?"
    s["fault_names"] = [f for i, f in enumerate(P.FAULTS) if s["faults"] >> i & 1]
    s["q_cmd_rad"] = [v / P.Q_SCALE for v in s["q_cmd"]]
    s["q_meas_rad"] = [nan if v == -32768 else v / P.Q_SCALE for v in s["q_meas"]]
    s["temp_c"] = [nan if v == -32768 else v / 10 for v in s["temp_c10"]]
    return s


def csv_row(s):
    row = {"t_ms": s["t_ms"], "mode": s["mode_name"], "faults": "|".join(s["fault_names"])}
    for k, site in enumerate(P.NTC_SITES):
        row[f"temp_{site}_c"] = s["temp_c"][k]
    for k, side in enumerate("lr"):
        row[f"current_{side}_a"] = s["i_ma"][k] / 1000
        row[f"rail_{side}_v"] = s["v_mv"][k] / 1000
    for j, name in enumerate(P.JOINTS):
        row[f"cmd_{name}"] = s["q_cmd_rad"][j]
        if j < P.NLEG:
            row[f"meas_{name}"] = s["q_meas_rad"][j]
    for k in range(16):
        row[f"adc{k}"] = s["mux_adc"][k]
    row["loop_us_max"] = s["loop_us_max"]
    return row


# ---------------------------------------------------------------- commands
def cmd_info(link, _):
    link.send("Hello")
    p = link.wait("Info")
    if p is None:
        sys.exit("no answer: check the port, and that the firmware is flashed")
    i = unpack("Info", p)
    ok = i["table_hash"] == P.TABLE_HASH and i["proto"] == P.PROTO_VERSION
    print(f"firmware {i['fw_major']}.{i['fw_minor']}, protocol {i['proto']}, {i['n_joints']} joints, "
          f"table 0x{i['table_hash']:08X} ({'matches this tool' if ok else 'DIFFERENT from this tool: reflash'})")


def cmd_monitor(link, a):
    writer, f, last = None, None, 0
    if a.csv:
        f = open(a.csv, "w", newline="")
    try:
        for s in link.states():
            if s is None:
                continue
            if f:
                row = csv_row(s)
                writer = writer or csv.DictWriter(f, fieldnames=list(row))
                if f.tell() == 0:
                    writer.writeheader()
                writer.writerow(row)
            if time.time() - last > 0.2:
                last = time.time()
                t = " ".join(f"{v:5.1f}" for v in s["temp_c"])
                print(f"\r{s['mode_name']:6s} faults={s['fault_names'] or '-'} T={t} C  "
                      f"I={s['i_ma'][0] / 1000:5.2f}/{s['i_ma'][1] / 1000:5.2f} A  "
                      f"V={s['v_mv'][0] / 1000:4.2f} V  loop={s['loop_us_max']} us   ", end="", flush=True)
    finally:
        if f:
            f.close()


def heartbeat(link, stop):
    while not stop.is_set():
        link.send("Hello")
        time.sleep(0.05)


def cmd_stand(link, _):
    link.send("Enable", pack("Enable", on=1))
    time.sleep(0.05)
    link.traj([(2000, P.Q_STAND)], replace=True)
    print("moving to the stand pose; Ctrl+C to stop (the MCU then holds and crouches)")
    stop = threading.Event()
    threading.Thread(target=heartbeat, args=(link, stop), daemon=True).start()
    try:
        cmd_monitor(link, argparse.Namespace(csv=None))
    finally:
        stop.set()


def cmd_p0(link, a):
    """Hip-roll thermal test: hold, and lift the transient mass for `pulse` s every `cycle` s."""
    j = P.JOINTS.index(a.joint)
    q0 = list(P.Q_STAND)
    q0[j] = math.radians(a.hold_deg)
    q1 = list(q0)
    q1[j] = math.radians(a.hold_deg + a.lift_deg)
    link.send("Enable", pack("Enable", on=1))
    time.sleep(0.05)
    link.traj([(1500, q0)], replace=True)
    stop = threading.Event()
    end = time.time() + a.minutes * 60

    def drive():
        while not stop.is_set() and time.time() < end:
            t_move = 80
            link.traj([(t_move, q1), (int(a.pulse * 1000) - t_move, q1), (t_move, q0),
                       (int(a.cycle * 1000) - int(a.pulse * 1000) - t_move, q0)])
            stop.wait(a.cycle)
        stop.set()

    time.sleep(1.6)
    threading.Thread(target=drive, daemon=True).start()
    f = open(a.csv, "w", newline="")
    writer, last, reason, over = None, 0, "time up", 0
    try:
        for s in link.states():
            if stop.is_set():
                break
            if s is None:
                continue
            row = csv_row(s)
            row["position_error_rad"] = s["q_meas_rad"][j] - s["q_cmd_rad"][j] if j < P.NLEG else float("nan")
            writer = writer or csv.DictWriter(f, fieldnames=list(row))
            if f.tell() == 0:
                writer.writeheader()
            writer.writerow(row)
            side = 0 if a.joint.startswith("l_") else 1
            temp, amps = s["temp_c"][side] if side < len(s["temp_c"]) else float("nan"), s["i_ma"][side] / 1000
            if s["mode_name"] == "FAULT":
                reason = f"MCU fault: {s['fault_names']}"
                break
            if temp > a.stop_temp:
                reason = f"case {temp:.1f} C above {a.stop_temp} C"
                break
            over = over + 1 if a.stall_a and amps > 0.9 * a.stall_a else 0
            if over > 100:                                        # 1 s of State at 100 Hz
                reason = f"current above 90% of stall ({a.stall_a} A) for 1 s"
                break
            if time.time() - last > 1:
                last = time.time()
                print(f"\r{(end - time.time()) / 60:4.1f} min left  T={temp:5.1f} C  I={amps:5.2f} A  "
                      f"err={math.degrees(row['position_error_rad']):5.1f} deg   ", end="", flush=True)
    except KeyboardInterrupt:
        reason = "stopped by hand"
    finally:
        stop.set()
        link.send("Enable", pack("Enable", on=0))
        f.close()
    print(f"\nend: {reason}. Log: {a.csv}")


def cmd_calibrate(link, a):
    """Two-point calibration of one joint: pulse width per radian and pot ADC per radian."""
    j = P.JOINTS.index(a.joint)
    pulses = [1500 - a.span_us, 1500 + a.span_us]
    current = {"us": pulses[0]}
    stop = threading.Event()

    def keep_pulsing():
        while not stop.is_set():
            link.send("RawPulse", pack("RawPulse", joint=j, pulse_us=current["us"]))
            time.sleep(0.1)

    threading.Thread(target=keep_pulsing, daemon=True).start()
    states = link.states()
    points = []
    try:
        for us in pulses:
            current["us"] = us
            ang = float(input(f"\n{a.joint} at {us} us. Measured joint angle in degrees (URDF sign): "))
            adc = [s["mux_adc"][j] for s in (next(x for x in states if x) for _ in range(20))] if j < P.NLEG else [0]
            points.append((us, math.radians(ang), sum(adc) / len(adc)))
    finally:
        stop.set()
        link.send("Enable", pack("Enable", on=0))
    (u0, q0, a0), (u1, q1, a1) = points
    us_per_rad = (u1 - u0) / (q1 - q0)
    entry = {"center_us": round(u0 - us_per_rad * q0, 1), "us_per_rad": round(abs(us_per_rad), 1),
             "dir": 1 if us_per_rad > 0 else -1}
    if j < P.NLEG:
        entry["pot"] = {"adc": [round(a0), round(a1)], "q": [round(q0, 4), round(q1, 4)]}
    print("\nAdd to config/calibration.json under \"joints\", then run tools/gen_tables.py and reflash:")
    print(f'  "{a.joint}": {entry},'.replace("'", '"'))


SUP_BITS = {1: "healthy", 2: "latched", 4: "clear_pending", 8: "oe_legs", 16: "oe_arms"}
RESET_CAUSES = ["unknown", "power-on", "pin", "software", "iwdg", "wwdg", "low-power"]


def print_diag(d):
    cpu = " ".join(f"{n}={d['cpu_permille'][k] / 10:.1f}%" for k, n in enumerate(P.TASKS + ["idle"]))
    stk = " ".join(f"{n}={d['stack_free_words'][k]}" for k, n in enumerate(P.TASKS))
    sup = ",".join(v for b, v in SUP_BITS.items() if d["supervisor"] & b) or "-"
    print(f"t={d['t_ms'] / 1000:8.1f}s cycles={d['cycles']} missed={d['missed_deadlines']} "
          f"exec avg/p99/max={d['exec_us_avg']}/{d['exec_us_p99']}/{d['exec_us_max']}us "
          f"jitter p99/max={d['jitter_us_p99']}/{d['jitter_us_max']}us\n"
          f"   supervisor wake jitter={d['sup_jitter_us_max']}us iwdg gap max={d['wdt_gap_us_max']}us "
          f"last latch {d['latch_delay_us']}us after deadline\n   cpu {cpu}\n"
          f"   stack free (words) {stk}  heap free {d['heap_free']} "
          f"(min {d['heap_min_free']}) allocs={d['allocs_after_start']} rx_overruns={d['rx_overruns']} "
          f"tx_drops={d['tx_drops']} supervisor={sup} reset={RESET_CAUSES[min(d['reset_cause'], 6)]}")


def cmd_diag(link, _):
    for t, p in link.frames():
        if t == T["Diag"]:
            print_diag(unpack("Diag", p))
        elif t == T["Log"]:
            print(f"[mcu] {p[1:].decode(errors='replace')}")


class Watcher:
    """Reads frames in the background: last State and Diag, Logs printed."""

    def __init__(self, link):
        self.link, self.state, self.diag, self.diag_n, self.last_rx = link, None, None, 0, time.time()
        self.stop = threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        for t, p in self.link.frames():
            if self.stop.is_set():
                return
            if t is None:
                continue
            self.last_rx = time.time()
            if t == T["State"]:
                self.state = decode_state(p)
            elif t == T["Diag"]:
                self.diag, self.diag_n = unpack("Diag", p), self.diag_n + 1
            elif t == T["Log"]:
                print(f"\n[mcu] {p[1:].decode(errors='replace')}")

    def until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            try:
                if cond():
                    return True
            except (TypeError, KeyError):
                pass
            time.sleep(0.01)
        return False

    def next_diag(self, timeout=3.0):
        n = self.diag_n
        return self.diag if self.until(lambda: self.diag_n > n, timeout) else None


def cmd_m0_accept(link, a):
    """M0 acceptance on the board: full-load timing run, then the overrun and hang fault injections.
    Needs the mico_mcu_m0test image (TestHook enabled). Servos may stay unplugged: M0 drives no actuator.
    The JSON report keeps the raw numbers next to each PASS/FAIL, plus every Diag sample of the run."""
    w = Watcher(link)
    stop = threading.Event()
    threading.Thread(target=heartbeat, args=(link, stop), daemon=True).start()
    results, ok = {"firmware": "mico_mcu_m0test", "minutes": a.minutes, "deadline_us": 5000}, True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        results.setdefault("checks", []).append({"check": name, "pass": bool(cond), "detail": str(detail)})
        print(f"  [{'PASS' if cond else 'FAIL'}] {name} {detail}")

    def oe(d):
        return {"legs": bool(d["supervisor"] & 8), "arms": bool(d["supervisor"] & 16)} if d else None

    if not w.until(lambda: w.state is not None, 3):
        sys.exit("no State from the MCU: wrong port, or not the Mico firmware")
    if w.state["mode_name"] == "FAULT":
        link.send("ClearFault")
        w.until(lambda: w.state["mode_name"] == "IDLE", 1)

    # ---- 1. full load: all sensors, both PCA9685s at 50 Hz, State at 100 Hz, trajectories streamed
    print(f"1/3 full-load run, {a.minutes} min")
    link.send("Enable", pack("Enable", on=1))
    check("enable -> RUN", w.until(lambda: w.state["mode_name"] == "RUN", 1))
    d0 = w.next_diag()
    h0 = time.time()
    samples = []
    end, phase, last_print = time.time() + a.minutes * 60, 0.0, 0
    queued_until = w.state["t_ms"]                       # MCU time the queued motion reaches
    try:
        while time.time() < end:
            # Paced by the MCU's own clock (State.t_ms): keep 160..320 ms of motion queued, never overfill.
            if queued_until - w.state["t_ms"] > 160:
                time.sleep(0.01)
                continue
            queued_until = max(queued_until, w.state["t_ms"]) + 160
            pts = []
            for _ in range(P.TRAJ_MAX_POINTS):          # 8 x 20 ms = 160 ms of a slow 0.5 Hz sway
                phase += 0.02
                q = [s + 0.05 * math.sin(2 * math.pi * 0.5 * phase) for s in P.Q_STAND]
                pts.append((20, q))
            link.traj(pts)
            d = w.diag
            if d and (not samples or d["t_ms"] != samples[-1]["t_ms"]):
                samples.append(dict(d))
            if d and time.time() - last_print > 10:
                last_print = time.time()
                print(f"\r  {(end - time.time()) / 60:4.1f} min left  missed={d['missed_deadlines']} "
                      f"jitter p99={d['jitter_us_p99']}us exec max={d['exec_us_max']}us   ", end="", flush=True)
    except KeyboardInterrupt:
        results["interrupted"] = True
    print()
    d1 = w.next_diag()
    h1 = time.time()
    if not (d0 and d1):
        sys.exit("no Diag frames: is this the FreeRTOS (M0) firmware?")
    print_diag(d1)
    cycles = d1["cycles"] - d0["cycles"]
    missed = d1["missed_deadlines"] - d0["missed_deadlines"]
    mcu_s = (d1["t_ms"] - d0["t_ms"]) / 1000.0
    margin = 5000 - (d1["exec_us_max"] + d1["jitter_us_max"])
    results["control"] = {
        "hz_mcu_clock": round(cycles / mcu_s, 3) if mcu_s > 0 else None,   # cycles over the MCU's own time
        "hz_host_clock": round(cycles / (h1 - h0), 3),                    # includes crystal and link timing
        "cycles_in_run": cycles, "cycles_since_boot": d1["cycles"], "run_s_mcu": mcu_s,
    }
    results["deadline"] = {"missed_in_run": missed, "missed_since_boot": d1["missed_deadlines"]}
    results["execution_us"] = {"avg": d1["exec_us_avg"], "p99": d1["exec_us_p99"], "max": d1["exec_us_max"]}
    results["jitter_us"] = {"p99": d1["jitter_us_p99"], "max": d1["jitter_us_max"]}
    results["timing_margin_us"] = margin
    results["supervisor"] = {"wake_jitter_us_max": d1["sup_jitter_us_max"],
                             "watchdog_refresh_gap_us_max": d1["wdt_gap_us_max"],
                             "iwdg_timeout_ms_min": 34, "iwdg_timeout_ms_max": 94}
    results["cpu_permille"] = dict(zip(P.TASKS + ["idle"], d1["cpu_permille"]))
    results["stack_free_words"] = dict(zip(P.TASKS, d1["stack_free_words"]))
    results["heap_bytes"] = {"free": d1["heap_free"], "min_free": d1["heap_min_free"]}
    results["allocs_after_start"] = d1["allocs_after_start"]
    results["link"] = {"rx_overruns": d1["rx_overruns"], "tx_drops": d1["tx_drops"]}
    hz = results["control"]["hz_mcu_clock"] or 0
    check("200 Hz", abs(hz - 200) <= 0.5, f"{hz} Hz ({cycles} cycles in {mcu_s:.1f} s MCU time)")
    check("zero missed deadlines", missed == 0, f"{missed} in run, {d1['missed_deadlines']} since boot")
    check("jitter p99 < 0.5 ms", d1["jitter_us_p99"] < 500,
          f"p99 {d1['jitter_us_p99']} us, max {d1['jitter_us_max']} us")
    check("max exec + max jitter < 5 ms", margin > 0,
          f"exec {d1['exec_us_max']} + jitter {d1['jitter_us_max']} us, margin {margin} us")
    check("IWDG refresh gap << 34 ms", d1["wdt_gap_us_max"] <= 2000,
          f"max gap {d1['wdt_gap_us_max']} us, supervisor wake jitter {d1['sup_jitter_us_max']} us")
    check("no allocation after start", d1["allocs_after_start"] == 0, d1["allocs_after_start"])
    check("no link overruns", d1["rx_overruns"] == 0 and d1["tx_drops"] == 0,
          f"rx {d1['rx_overruns']} tx {d1['tx_drops']}")
    check("stack headroom >= 64 words", min(d1["stack_free_words"]) >= 64, d1["stack_free_words"])
    check("still RUN with OE on", w.state["mode_name"] == "RUN" and oe(d1) == {"legs": True, "arms": True},
          f"{w.state['mode_name']}, OE {oe(d1)}")

    # ---- 2. overrun one cycle: supervisor latches, outputs off, MCU keeps running
    print("2/3 overrun injection (one 8 ms cycle)")
    t_before = d1["t_ms"]
    t0 = time.time()
    link.send("TestHook", pack("TestHook", action=1, arg_ms=8))
    seen = w.until(lambda: w.state["mode_name"] == "FAULT" and "CONTROL_DEADLINE" in w.state["fault_names"], 0.5)
    host_ms = round((time.time() - t0) * 1000, 1)
    check("overrun -> FAULT CONTROL_DEADLINE", seen, f"seen by the host after {host_ms} ms")
    d2 = w.next_diag()
    results["fault_injection"] = {"overrun": {
        "injected_ms": 8,
        "latch_after_deadline_us": d2 and d2["latch_delay_us"],   # MCU: deadline passed -> latched, OE off
        "host_observed_ms": host_ms,                               # includes link and State period
        "oe_after": oe(d2),
        "missed_deadlines": d2 and d2["missed_deadlines"] - d1["missed_deadlines"]}}
    check("latched and OE off within 1 ms of the deadline", d2 and d2["supervisor"] & 2 and oe(d2) ==
          {"legs": False, "arms": False} and d2["latch_delay_us"] <= 1100, f"{d2 and d2['latch_delay_us']} us")
    check("no reset on an overrun", d2 and d2["t_ms"] > t_before and d2["reset_cause"] == d1["reset_cause"])
    link.send("ClearFault")
    check("ClearFault -> IDLE", w.until(lambda: w.state["mode_name"] == "IDLE", 1))
    d3 = w.next_diag()
    check("latch cleared, OE still off", d3 and not d3["supervisor"] & 2 and d3["supervisor"] & 1
          and oe(d3) == {"legs": False, "arms": False}, oe(d3))
    link.send("Enable", pack("Enable", on=1))
    check("re-enable -> RUN", w.until(lambda: w.state["mode_name"] == "RUN", 1))

    # ---- 3. hang the control task: watchdog not refreshed -> IWDG reset -> boots with outputs off
    print("3/3 hang injection (expects an IWDG reset)")
    t_hang = time.time()
    link.send("TestHook", pack("TestHook", action=2, arg_ms=0))
    w.until(lambda: time.time() - w.last_rx > 0.02, 1)
    w.state, w.diag = None, None
    came_back = w.until(lambda: w.state is not None, 3)
    # The MCU's own detection time is lost in the reset. Host view: command sent -> first State after the
    # reboot = stall detection (<= 10 ms) + IWDG timeout (34..94 ms) + boot + link latency.
    hang = {"back_after_ms": round((time.time() - t_hang) * 1000, 1) if came_back else None}
    results["fault_injection"]["hang"] = hang
    check("MCU reset and came back", came_back, hang)
    if came_back:
        check("boots in FAULT WATCHDOG_RESET", w.until(
            lambda: w.state["mode_name"] == "FAULT" and "WATCHDOG_RESET" in w.state["fault_names"], 1))
        d4 = w.next_diag()
        hang["reset_cause"] = d4 and RESET_CAUSES[min(d4["reset_cause"], 6)]
        hang["oe_after_reset"] = oe(d4)
        check("reset cause IWDG", d4 and d4["reset_cause"] == 4, hang["reset_cause"])
        check("OE off after the reset", oe(d4) == {"legs": False, "arms": False}, oe(d4))
        link.send("Enable", pack("Enable", on=1))
        time.sleep(0.3)
        check("Enable refused after the reset", w.state["mode_name"] == "FAULT", w.state["mode_name"])
        link.send("ClearFault")
        w.until(lambda: w.state["mode_name"] == "IDLE", 1)
        d5 = w.next_diag()
        hang["oe_after_clear"] = oe(d5)
        check("cleared to IDLE, OE still off", w.state["mode_name"] == "IDLE" and oe(d5) ==
              {"legs": False, "arms": False}, f"{w.state['mode_name']}, OE {oe(d5)}")

    stop.set()
    w.stop.set()
    link.send("Enable", pack("Enable", on=0))
    results["pass"] = ok
    results["diag_samples"] = samples
    if a.report:
        Path(a.report).write_text(__import__("json").dumps(results, indent=2))
        print(f"report: {a.report}")
    print("M0 ACCEPTANCE: " + ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="COM5 on Windows, /dev/tty.usbmodem... on macOS")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    m = sub.add_parser("monitor")
    m.add_argument("--csv")
    sub.add_parser("stand")
    sub.add_parser("disable")
    sub.add_parser("clear")
    p0 = sub.add_parser("p0-thermal")
    p0.add_argument("--joint", default="l_hip_roll", choices=P.JOINTS)
    p0.add_argument("--hold-deg", type=float, default=0.0, help="joint angle that holds the lever level")
    p0.add_argument("--lift-deg", type=float, default=10.0, help="extra angle that lifts the transient mass")
    p0.add_argument("--cycle", type=float, default=2.0)
    p0.add_argument("--pulse", type=float, default=0.4)
    p0.add_argument("--minutes", type=float, default=10.0)
    p0.add_argument("--stop-temp", type=float, default=60.0)
    p0.add_argument("--stall-a", type=float, default=0.0, help="measured stall current; 0 skips the check")
    p0.add_argument("--csv", required=True)
    c = sub.add_parser("calibrate")
    c.add_argument("--joint", required=True, choices=P.JOINTS)
    c.add_argument("--span-us", type=int, default=400, help="pulse offset either side of 1500 us")
    sub.add_parser("diag")
    m0 = sub.add_parser("m0-accept")
    m0.add_argument("--minutes", type=float, default=10.0)
    m0.add_argument("--report", help="write the results as JSON")
    a = ap.parse_args()
    link = Link(a.port)
    if a.cmd == "disable":
        link.send("Enable", pack("Enable", on=0))
    elif a.cmd == "clear":
        link.send("ClearFault")
    else:
        {"info": cmd_info, "monitor": cmd_monitor, "stand": cmd_stand, "p0-thermal": cmd_p0,
         "calibrate": cmd_calibrate, "diag": cmd_diag, "m0-accept": cmd_m0_accept}[a.cmd](link, a)


if __name__ == "__main__":
    main()
