# Mico mechanics (CAD in code)

Parametric CadQuery model of the robot's printed parts, with every hole, insert and bearing seat placed, plus
the checks and documents generated from it. Start at [docs/mechanical/leg_v3.md](../docs/mechanical/leg_v3.md).

| File | What |
| --- | --- |
| `hardware.py` | Servos, horn, screws, inserts, bearing, hole sizes, fit rules. **Measure your servos and update it first** |
| `joint.py` | One servo joint: pocket (holder), horn plate and idler plate (driven), caps, tool paths |
| `leg.py` | Leg v3: six joints, nine printed parts, cable route, tie slots |
| `pose.py` | Places links and bought parts for joint angles |
| `check.py` | Clearance over the joint ranges, servo removal paths, tool access, both legs together |
| `render.py`, `blender_render.py` | Renders and assembly-step images (Blender, Cycles CPU) |
| `gen_docs.py` | Print files (`out/`, STEP + STL, left and right), drawings, the assembly guide |
| `electronics.py` | Board and module envelopes (holes, heights) for the sleds. **Measure your boards** |
| `body.py` | Body v4: pelvis plate, deck 1 and 2 on standoffs, board sleds and bays, arms, neck, head |
| `robot.py` | Whole robot: both legs on the body, static fit, arm/neck/head sweeps, tool access, servo removal |
| `shell.py` | Shell v1 (`BazanShell`): chest/back clamshell, helmet front/back, visor, on the 16 shell inserts; checked against `robot.py` sweeps, writes `out/shell_*.step/.stl` (about 15 min, `--quick` 11) |
| `pic.py`, `gen_manual.py` | Step pictures and the whole-robot manual (`docs/mechanical/mico_v4_lap_rap.md` + `.html`) |

```bash
pip install cadquery bpy matplotlib   # Python 3.11+
cd mechanical
python check.py            # about 1 min; CI runs --quick
python render.py           # about 5 min
python gen_docs.py         # out/*.step, out/*.stl, docs/mechanical/
python robot.py            # about 4 min (--quick: 2 min)
python gen_manual.py       # about 1 min
```

On Windows run the same in PowerShell (`py -m pip install ...`, `py check.py`). The STL/STEP files are not
committed: CI builds them and attaches them to every run (artifact `mico-print-files`).
