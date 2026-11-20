# Mass budget

The single mass model for Bazan: the robot generated from the printable CAD. `hardware/mechanical/gen_urdf.py`
weighs every component once (`audit.weigh`: printed parts with their print recipe, servos with a horn and screw
allowance, boards, battery, shell, TPU soles, wiring) and writes both `bazan_cad.urdf` (what MuJoCo, the gait and the
firmware use) and `docs/engineering/cad_mass.json` (what this page shows).

Tables between `gen:` markers are written by `python docs/engineering/gen_docs.py` from those sources. Do not edit
them by hand; CI fails if they drift.

<!-- gen:stamp -->
Generated from `docs/engineering/cad_mass.json`, `software/ros2/src/humanoid_description/urdf/bazan_cad.urdf`, `software/ai/sim/mujoco/servo_loads.csv`, `software/ros2/src/humanoid_control/scripts/bazan_gait.py`, `hardware/mechanical/hardware.py`, `docs/engineering/bom_aplus.csv` · source hash `f567ce9fcc09`
<!-- /gen:stamp -->

## Current configuration

<!-- gen:summary -->
| Quantity | Value |
| --- | ---: |
| Total | **1803 g** |
| Height (floor to the top of the head) | 540 mm |
| CoM above the floor (straight legs) | 252 mm |
| CoM x / y from the middle of the hip points | 0.1 / 0.1 mm |
| Whole-body inertia about CoM (roll, pitch, yaw) | 43.80 / 41.23 / 3.56 ×10⁻³ kg·m² |
| LIPM time constant √(z/g) | 160 ms |
<!-- /gen:summary -->

Mass figures that still appear in older documents describe robots that will not be built:

| Figure | What it was |
| --- | --- |
| 536 g | the old simulation box model (`mass:=sim`), 303 mm tall, retired |
| 677 g | the Design C Blender shell on the old 303 mm skeleton (`design/blender`), retired as a mass source |

## By group

<!-- gen:groups -->
| Group | g | Share |
| --- | ---: | ---: |
| Servos (with horn and screw allowance) | 755.6 | 42% |
| Printed frame | 719.8 | 40% |
| Printed shell | 104.0 | 6% |
| Electronics and sensors | 95.5 | 5% |
| Battery 2S | 57.0 | 3% |
| TPU soles | 39.1 | 2% |
| Wiring | 30.0 | 2% |
| Metal parts | 2.2 | 0% |
<!-- /gen:groups -->

## Per link

CoM in the link frame, inertia about the link CoM (diagonal shown; the full tensor is in `bazan_cad.urdf`). Right
side mirrors left.

<!-- gen:links -->
| Link | g | CoM x / y / z (mm) | Ixx / Iyy / Izz (g·mm²) |
| --- | ---: | ---: | ---: |
| `base_link` | 533.1 | -3.2 / 0.4 / 128.3 | 1,321,471 / 1,056,368 / 816,183 |
| `neck_yaw_link` | 18.1 | -4.5 / -1.7 / -1.2 | 1,632 / 1,711 / 2,331 |
| `head_pitch_link` | 66.8 | 11.9 / 0.0 / 18.5 | 43,254 / 53,997 / 59,077 |
| `l_shoulder_pitch_link` | 19.4 | -3.3 / -2.4 / -16.5 | 2,830 / 3,057 / 2,108 |
| `l_shoulder_roll_link` | 22.4 | 5.2 / 2.4 / -36.3 | 5,299 / 5,781 / 2,717 |
| `l_elbow_link` | 7.7 | 0.1 / 0.0 / -37.7 | 4,098 / 4,256 / 250 |
| `l_hip_yaw_link` | 18.8 | 5.2 / 0.0 / 19.8 | 3,993 / 9,957 / 9,256 |
| `l_hip_roll_link` | 165.4 | 1.8 / 6.7 / -19.0 | 106,930 / 93,382 / 42,227 |
| `l_hip_pitch_link` | 113.8 | -2.4 / 1.3 / -80.1 | 102,933 / 85,009 / 32,000 |
| `l_knee_link` | 31.3 | 6.1 / 5.6 / -49.6 | 50,262 / 34,796 / 20,739 |
| `l_ankle_pitch_link` | 168.1 | 1.7 / 6.7 / -10.5 | 116,330 / 102,466 / 42,794 |
| `l_ankle_roll_link` | 45.7 | 0.0 / 5.0 / -23.6 | 13,898 / 37,561 / 45,445 |
<!-- /gen:links -->

## Assumptions to check on the bench

- Printed parts: the wall and infill recipe per part group in `hardware/mechanical/audit.py` (`printed_g`). Weigh
  the first prints and correct the recipe if a part differs by more than 10% (test T8 in `mechanical_verification.md`).
- Servos carry a hardware allowance (3 g per leg servo, 1.5 g per arm and neck servo).
- Wiring is a flat 30 g allowance in the pelvis.
