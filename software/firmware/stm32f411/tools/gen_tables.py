"""Generate the firmware's joint table and the MCU <-> host protocol from one place.

    python firmware/stm32f411/tools/gen_tables.py           # write the generated files
    python firmware/stm32f411/tools/gen_tables.py --check   # exit 1 if any is out of date (CI)

Inputs (nothing here is typed twice):
    sim/mini_humanoid/gen_robot.py              joint names, axes limits, servo types
    src/humanoid_control/scripts/mico_gait.py   wire order of the 20 joints, stand and crouch poses (leg IK)
    firmware/stm32f411/config/calibration.json   per-joint servo and pot calibration, NTC, FSR
Outputs:
    lib/core/generated/joint_table.h            C++ constants
    lib/core/generated/protocol.h               packed message structs
    firmware/stm32f411/tools/mico_protocol.py    the same messages for Python (struct formats)
"""
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FW = HERE.parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
SIM = REPO / "software" / "ai" / "sim" / "mini_humanoid"
sys.path.insert(0, str(SIM))
import gen_robot  # noqa: E402

_spec = importlib.util.spec_from_file_location("mico_gait", REPO / "src/humanoid_control/scripts/mico_gait.py")
gait = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gait)

PROTO_VERSION = 1
Q_SCALE = 10000          # joint angles on the wire: int16 in 1e-4 rad
NTC_SITES = ["l_hip_roll", "r_hip_roll"]
FSR_SITES = ["l_heel", "l_toe", "r_heel", "r_toe"]
STAND_H = gait.H0        # hip-to-ankle height of the gait's start pose (m)
CROUCH_H = 0.070         # safe crouch: the squat kin_dyn.py checks

# ---------------------------------------------------------------- protocol
# Frame on the wire: COBS(type u8, seq u8, payload, crc16-ccitt u16 LE) followed by 0x00.
# (name, type id, fields); field = (name, C type, count). Traj is a header followed by n TrajPoint records.
NJ = len(gait.JOINTS)
TASKS = ["supervisor", "control", "comm", "diagnostics"]   # FreeRTOS tasks, in Diag order (CPU adds idle last)
N_TASKS = len(TASKS)
FAULT_HISTORY = 4
NLEG = 12
MESSAGES = [
    ("Hello", 0x01, []),
    ("Enable", 0x02, [("on", "uint8_t", 1)]),
    ("Traj", 0x03, [("flags", "uint8_t", 1), ("n", "uint8_t", 1)]),
    ("ClearFault", 0x04, []),
    ("RawPulse", 0x06, [("joint", "uint8_t", 1), ("pulse_us", "uint16_t", 1)]),
    # M0 acceptance only (builds with MICO_TEST_HOOKS): 1 = overrun one control cycle by arg_ms, 2 = hang it
    ("TestHook", 0x07, [("action", "uint8_t", 1), ("arg_ms", "uint16_t", 1)]),
    ("State", 0x80, [("t_ms", "uint32_t", 1), ("mode", "uint8_t", 1), ("faults", "uint16_t", 1),
                     ("q_cmd", "int16_t", NJ), ("q_meas", "int16_t", NLEG), ("mux_adc", "uint16_t", 16),
                     ("temp_c10", "int16_t", len(NTC_SITES)), ("i_ma", "int16_t", 2), ("v_mv", "uint16_t", 2),
                     ("imu", "int16_t", 6), ("loop_us_max", "uint16_t", 1), ("rx_errors", "uint16_t", 1)]),
    ("Info", 0x81, [("fw_major", "uint8_t", 1), ("fw_minor", "uint8_t", 1), ("proto", "uint8_t", 1),
                    ("n_joints", "uint8_t", 1), ("table_hash", "uint32_t", 1)]),
    ("Log", 0x82, [("level", "uint8_t", 1)]),          # followed by UTF-8 text
    # 1 Hz from the diagnostics task: timing, CPU, stacks, heap, link, supervisor, fault history.
    # supervisor bits: 1 healthy, 2 latched, 4 clear pending, 8 OE legs on, 16 OE arms on
    ("Diag", 0x83, [("t_ms", "uint32_t", 1), ("cycles", "uint32_t", 1), ("missed_deadlines", "uint32_t", 1),
                    ("exec_us_max", "uint16_t", 1), ("exec_us_p99", "uint16_t", 1), ("exec_us_avg", "uint16_t", 1),
                    ("jitter_us_max", "uint16_t", 1), ("jitter_us_p99", "uint16_t", 1),
                    # supervisor task: wake-latency spread, longest gap between IWDG refreshes, last latch delay
                    ("sup_jitter_us_max", "uint16_t", 1), ("wdt_gap_us_max", "uint16_t", 1),
                    ("latch_delay_us", "uint16_t", 1),
                    ("cpu_permille", "uint16_t", N_TASKS + 1), ("stack_free_words", "uint16_t", N_TASKS),
                    ("heap_free", "uint32_t", 1), ("heap_min_free", "uint32_t", 1),
                    ("allocs_after_start", "uint16_t", 1), ("rx_overruns", "uint16_t", 1),
                    ("tx_drops", "uint16_t", 1), ("supervisor", "uint8_t", 1), ("reset_cause", "uint8_t", 1),
                    ("fault_t_ms", "uint32_t", FAULT_HISTORY), ("fault_bits", "uint16_t", FAULT_HISTORY)]),
]
TRAJ_POINT = [("dt_ms", "uint16_t", 1), ("q", "int16_t", NJ)]
TRAJ_MAX_POINTS = 8
MODES = ["BOOT", "IDLE", "RUN", "HOLD", "CROUCH", "FAULT", "CALIB"]
FAULTS = ["OVERCURRENT_L", "OVERCURRENT_R", "OVERTEMP", "UNDERVOLT", "JOINT_JAM", "I2C_PWM", "I2C_SENSOR",
          "WATCHDOG_RESET", "CONTROL_DEADLINE"]
PYFMT = {"uint8_t": "B", "int8_t": "b", "uint16_t": "H", "int16_t": "h", "uint32_t": "I", "int32_t": "i"}


def joints():
    bodies = {b.joint.name: b for b in gen_robot.walk(gen_robot.ROBOT) if b.joint}
    cal = json.loads((FW / "config" / "calibration.json").read_text())
    stand = dict(zip(("hip_pitch", "knee", "ankle_pitch"), gait.leg_ik(0, STAND_H)))
    crouch = dict(zip(("hip_pitch", "knee", "ankle_pitch"), gait.leg_ik(0, CROUCH_H)))
    out = []
    for i, name in enumerate(gait.JOINTS):
        j = bodies[name].joint
        d = dict(cal["servo_defaults"][j.servo])
        d.update(dir=1)
        d.update({k: v for k, v in cal["joints"].get(name, {}).items() if k != "pot"})
        pot = cal["joints"].get(name, {}).get("pot")
        key = name[2:] if name[:2] in ("l_", "r_") else name
        out.append(dict(
            name=name, servo=j.servo, qmin=math.radians(j.range_deg[0]), qmax=math.radians(j.range_deg[1]),
            board=0 if i < NLEG else 1, channel=i if i < NLEG else i - NLEG,
            stand=stand.get(key, 0.0), crouch=crouch.get(key, 0.0),
            center_us=d["center_us"], us_per_rad=d["us_per_rad"], dir=d["dir"],
            min_us=d["min_us"], max_us=d["max_us"],
            pot_valid=int(pot is not None and i < NLEG),
            pot_adc=pot["adc"] if pot else [0, 4095], pot_q=pot["q"] if pot else [0.0, 1.0]))
    return out, cal


def f(v):
    return f"{v:.6f}f"


def arr(ctype, name, vals, fmt=str):
    return f"constexpr {ctype} {name}[] = {{{', '.join(fmt(v) for v in vals)}}};"


def joint_table(js, cal):
    body = "\n".join([
        f"constexpr int NJ = {NJ};",
        f"constexpr int NLEG = {NLEG};",
        f"constexpr int NNTC = {len(NTC_SITES)};",
        f"constexpr int NFSR = {len(FSR_SITES)};",
        f"constexpr float Q_SCALE = {Q_SCALE}.0f;  // wire units per rad",
        f"constexpr uint32_t PCA_OSC_HZ = {cal['pca_osc_hz']}u;",
        f"constexpr uint32_t SERVO_FRAME_HZ = {cal['servo_frame_hz']}u;",
        "constexpr const char* JOINT_NAMES[] = {" + ", ".join(f'"{j["name"]}"' for j in js) + "};",
        arr("float", "Q_MIN", [j["qmin"] for j in js], f),
        arr("float", "Q_MAX", [j["qmax"] for j in js], f),
        arr("float", "Q_STAND", [j["stand"] for j in js], f),
        arr("float", "Q_CROUCH", [j["crouch"] for j in js], f),
        arr("uint8_t", "PCA_BOARD", [j["board"] for j in js]),
        arr("uint8_t", "PCA_CHANNEL", [j["channel"] for j in js]),
        arr("float", "CENTER_US", [j["center_us"] for j in js], f),
        arr("float", "US_PER_RAD", [j["us_per_rad"] for j in js], f),
        arr("int8_t", "DIR", [j["dir"] for j in js]),
        arr("uint16_t", "MIN_US", [j["min_us"] for j in js]),
        arr("uint16_t", "MAX_US", [j["max_us"] for j in js]),
        arr("uint8_t", "POT_VALID", [j["pot_valid"] for j in js[:NLEG]]),
        arr("uint16_t", "POT_ADC0", [j["pot_adc"][0] for j in js[:NLEG]]),
        arr("uint16_t", "POT_ADC1", [j["pot_adc"][1] for j in js[:NLEG]]),
        arr("float", "POT_Q0", [j["pot_q"][0] for j in js[:NLEG]], f),
        arr("float", "POT_Q1", [j["pot_q"][1] for j in js[:NLEG]], f),
        "constexpr const char* NTC_SITES[] = {" + ", ".join(f'"{s}"' for s in NTC_SITES) + "};",
        arr("float", "NTC_R0", [cal["ntc"][s]["r0_ohm"] for s in NTC_SITES], f),
        arr("float", "NTC_BETA", [cal["ntc"][s]["beta"] for s in NTC_SITES], f),
        "constexpr const char* FSR_SITES[] = {" + ", ".join(f'"{s}"' for s in FSR_SITES) + "};",
        arr("uint16_t", "FSR_ZERO", [cal["fsr"][s]["zero_adc"] for s in FSR_SITES]),
    ])
    return body


def protocol_h():
    lines = [f"constexpr uint8_t PROTO_VERSION = {PROTO_VERSION};",
             f"constexpr int TRAJ_MAX_POINTS = {TRAJ_MAX_POINTS};",
             f"constexpr int N_TASKS = {N_TASKS};",
             f"constexpr int FAULT_HISTORY = {FAULT_HISTORY};",
             "enum class Mode : uint8_t { " + ", ".join(f"{m} = {i}" for i, m in enumerate(MODES)) + " };",
             "enum Fault : uint16_t { " + ", ".join(f"F_{x} = 1u << {i}" for i, x in enumerate(FAULTS)) + " };",
             "enum MsgType : uint8_t { " + ", ".join(f"MSG_{n.upper()} = 0x{t:02X}" for n, t, _ in MESSAGES) + " };",
             "",
             "#pragma pack(push, 1)"]
    for name, _, fields in MESSAGES + [("TrajPoint", None, TRAJ_POINT)]:
        if not fields:
            continue
        lines.append(f"struct {name} {{")
        for fn, ct, n in fields:
            lines.append(f"  {ct} {fn}{f'[{n}]' if n > 1 else ''};")
        lines.append("};")
    lines.append("#pragma pack(pop)")
    for name, _, fields in MESSAGES + [("TrajPoint", None, TRAJ_POINT)]:
        if fields:
            size = sum({"uint8_t": 1, "uint16_t": 2, "int16_t": 2, "uint32_t": 4}[c] * n for _, c, n in fields)
            lines.append(f"static_assert(sizeof({name}) == {size}, \"{name} layout\");")
    return "\n".join(lines)


def protocol_py(js):
    r = lambda k: [round(j[k], 6) for j in js]   # noqa: E731
    out = [f"Q_MIN = {r('qmin')!r}", f"Q_MAX = {r('qmax')!r}", f"Q_STAND = {r('stand')!r}",
           f"Q_CROUCH = {r('crouch')!r}", f"PROTO_VERSION = {PROTO_VERSION}", f"Q_SCALE = {Q_SCALE}",
           f"NJ = {NJ}", f"NLEG = {NLEG}",
           f"TRAJ_MAX_POINTS = {TRAJ_MAX_POINTS}", f"JOINTS = {list(gait.JOINTS)!r}",
           f"NTC_SITES = {NTC_SITES!r}", f"FSR_SITES = {FSR_SITES!r}", f"MODES = {MODES!r}",
           f"FAULTS = {FAULTS!r}", f"TASKS = {TASKS!r}", "",
           "# name -> (type id, struct format, [(field, count)])", "MESSAGES = {"]
    for name, t, fields in MESSAGES + [("TrajPoint", None, TRAJ_POINT)]:
        fmt = "<" + "".join(f"{n if n > 1 else ''}{PYFMT[c]}" for _, c, n in fields)
        out.append(f"    {name!r}: ({t!r}, {fmt!r}, {[(fn, n) for fn, _, n in fields]!r}),")
    out.append("}")
    return "\n".join(out)


def table_hash(text):
    return int(hashlib.sha256(text.encode()).hexdigest()[:8], 16)


def render():
    js, cal = joints()
    jt = joint_table(js, cal)
    proto = protocol_h()
    h = table_hash(jt + proto)
    note = "Generated by firmware/stm32f411/tools/gen_tables.py. Do not edit."
    files = {
        REPO / "lib" / "core" / "generated" / "joint_table.h":
            f"// {note}\n#pragma once\n#include <stdint.h>\n\nnamespace mico {{\n\n"
            f"constexpr uint32_t TABLE_HASH = 0x{h:08X}u;  // reported in Info; the host checks it\n{jt}\n\n"
            "}  // namespace mico\n",
        REPO / "lib" / "core" / "generated" / "protocol.h":
            f"// {note}\n#pragma once\n#include <stdint.h>\n\nnamespace mico {{\n\n{proto}\n\n}}  // namespace mico\n",
        HERE / "mico_protocol.py":
            f'"""{note}"""\n# flake8: noqa\n{protocol_py(js)}\nTABLE_HASH = 0x{h:08X}\n',
    }
    return files


def main():
    check = "--check" in sys.argv
    stale = []
    for path, text in render().items():
        if not path.exists() or path.read_text() != text:
            stale.append(path.relative_to(REPO).as_posix())
            if not check:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
    if check and stale:
        print("out of date, run python firmware/stm32f411/tools/gen_tables.py:", ", ".join(stale))
        sys.exit(1)
    print("up to date" if check else f"wrote {len(stale)} file(s)")


if __name__ == "__main__":
    main()
