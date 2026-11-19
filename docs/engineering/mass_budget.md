# Mass budget

The single mass model for Bazan: the robot generated from the printable CAD. `hardware/mechanical/gen_urdf.py`
weighs every component once (`audit.weigh`: printed parts with their print recipe, servos with a horn and screw
allowance, boards, battery, shell, TPU soles, wiring) and writes both `bazan_cad.urdf` (what MuJoCo, the gait and the
firmware use) and `docs/engineering/cad_mass.json` (what this page shows).

Tables between `gen:` markers are written by `python docs/engineering/gen_docs.py` from those sources. Do not edit
them by hand; CI fails if they drift.

<!-- gen:stamp -->
Generated from `docs/engineering/cad_mass.json`, `software/ros2/src/humanoid_description/urdf/bazan_cad.urdf`, `software/ai/sim/mujoco/servo_loads.csv`, `software/ros2/src/humanoid_control/scripts/bazan_gait.py`, `hardware/mechanical/hardware.py`, `docs/engineering/bom_aplus.csv` · source hash `2247e7c80149`
<!-- /gen:stamp -->

## Current configuration

<!-- gen:summary -->
| Quantity | Value |
| --- | ---: |
| Total | **1083 g** |
| Height (floor to the top of the head) | 407 mm |
| CoM above the floor (straight legs) | 206 mm |
| CoM x / y from the middle of the hip points | -0.7 / 0.2 mm |
| Whole-body inertia about CoM (roll, pitch, yaw) | 12.57 / 11.38 / 1.95 ×10⁻³ kg·m² |
| LIPM time constant √(z/g) | 145 ms |
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
| Printed frame | 473.1 | 44% |
| Servos (with horn and screw allowance) | 295.6 | 27% |
| Printed shell | 108.6 | 10% |
| Electronics and sensors | 95.5 | 9% |
| Battery 2S | 57.0 | 5% |
| Wiring | 30.0 | 3% |
| TPU soles | 20.8 | 2% |
| Metal parts | 2.2 | 0% |
<!-- /gen:groups -->

## Per link

CoM in the link frame, inertia about the link CoM (diagonal shown; the full tensor is in `bazan_cad.urdf`). Right
side mirrors left.

<!-- gen:links -->
| Link | g | CoM x / y / z (mm) | Ixx / Iyy / Izz (g·mm²) |
| --- | ---: | ---: | ---: |
| `base_link` | 533.1 | -3.2 / 0.4 / 112.2 | 1,300,977 / 1,056,387 / 795,670 |
| `neck_yaw_link` | 18.1 | -4.5 / -1.7 / -1.2 | 1,632 / 1,711 / 2,331 |
| `head_pitch_link` | 71.4 | 12.7 / 0.0 / 26.1 | 52,569 / 65,272 / 64,605 |
| `l_shoulder_pitch_link` | 19.4 | -3.3 / -2.4 / -16.5 | 2,830 / 3,057 / 2,108 |
| `l_shoulder_roll_link` | 22.4 | 5.2 / 2.4 / -36.3 | 5,299 / 5,781 / 2,717 |
| `l_elbow_link` | 7.7 | 0.1 / 0.0 / -37.7 | 4,098 / 4,256 / 250 |
| `l_hip_yaw_link` | 14.9 | 3.3 / 0.0 / 14.2 | 2,353 / 4,806 / 5,244 |
| `l_hip_roll_link` | 46.5 | 1.3 / -1.2 / -12.5 | 12,507 / 10,907 / 5,495 |
| `l_hip_pitch_link` | 35.4 | -2.1 / 0.7 / -35.8 | 14,046 / 9,850 / 6,733 |
| `l_knee_link` | 12.3 | 2.9 / 5.7 / -24.8 | 10,305 / 5,591 / 6,089 |
| `l_ankle_pitch_link` | 46.6 | 1.3 / -1.2 / -7.5 | 12,517 / 10,917 / 5,495 |
| `l_ankle_roll_link` | 24.9 | 0.0 / -0.1 / -15.7 | 3,755 / 12,997 / 14,548 |
<!-- /gen:links -->

## Assumptions to check on the bench

- Printed parts: the wall and infill recipe per part group in `hardware/mechanical/audit.py` (`printed_g`). Weigh
  the first prints and correct the recipe if a part differs by more than 10% (test T8 in `mechanical_verification.md`).
- Servos carry a hardware allowance (3 g per leg servo, 1.5 g per arm and neck servo).
- Wiring is a flat 30 g allowance in the pelvis.
