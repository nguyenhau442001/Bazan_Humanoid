# Mass budget

The single mass model for Bazan. Every part is counted once:

- **Bought parts** (servos, battery, boards, sensors, wiring) come from `sim/mini_humanoid/gen_robot.py`
  and `components()` in `docs/design/blender/mico_parts.py`, at catalogue mass.
- **Printed shell** comes from the Design C meshes in `mico_parts.py`, integrated by
  `docs/design/blender/mico_mass.py` (outer solid minus the solid shrunk by the wall).

Tables between `gen:` markers are written by `python docs/engineering/gen_docs.py` from
`docs/design/blender/mass_report.json`. Do not edit them by hand; CI fails if they drift.

<!-- gen:stamp -->
Generated from `design/blender/mico_parts.py`, `design/blender/mico_mass.py`, `sim/mini_humanoid/gen_robot.py`, `sim/mini_humanoid/tables/mj_servo_loads.csv`, `src/humanoid_control/scripts/mico_gait.py`, `docs/engineering/bom_aplus.csv` · source hash `eef8fd498b39`
<!-- /gen:stamp -->

## Current configuration

<!-- gen:summary -->
| Quantity | Value |
| --- | ---: |
| Total, PLA shell (baseline) | **677 g** |
| Total, LW-PLA shell | 603 g |
| Height (top of head shell) | 303 mm |
| CoM above the floor | 144 mm |
| CoM x / y from the hip point | -0.6 / 0.0 mm |
| Whole-body inertia about CoM (roll, pitch, yaw) | 4.53 / 4.11 / 0.61 ×10⁻³ kg·m² |
| LIPM time constant √(z/g) | 121 ms |
| Simulation model `mass:=sim` (for comparison) | 536 g |
<!-- /gen:summary -->

Other mass figures that still appear in older documents are not current:

| Figure | What it was |
| --- | --- |
| 481 g | `docs/design/sketch_shell_a` boards, before the skeleton was finalised |
| 536 g | the simulation box model (`mass:=sim`, MuJoCo); the shell is only roughed in, so torques from it run low |
| 667 g | the first Blender shell (v2.0, clear dome), replaced by Design C |

## By group

<!-- gen:groups -->
| Group | g | Share |
| --- | ---: | ---: |
| Servos | 247.6 | 37% |
| Printed shell | 186.6 | 28% |
| Electronics | 70.0 | 10% |
| Sensors, brackets, screws | 67.5 | 10% |
| Battery 2S | 57.0 | 8% |
| Wiring | 30.0 | 4% |
| Soles | 18.0 | 3% |
<!-- /gen:groups -->

## Shell material and wall

<!-- gen:variants -->
| Shell material and wall | Total g | vs baseline |
| --- | ---: | ---: |
| PLA, walls as drawn (1.0 / 1.2 mm) | 677 | – |
| PLA, walls +0.4 mm (0.8 mm nozzle, 2 perimeters) | 745 | +68 |
| LW-PLA, walls as drawn | 603 | -74 |
| PLA, walls 0.8 mm (thin, fragile) | 641 | -35 |
<!-- /gen:variants -->

## Per link

CoM in the link frame, inertia about the link CoM (diagonal shown; the full tensor is in
`docs/design/blender/tables/links_mass.csv` and `humanoid_blender.urdf`). Right side mirrors left.

<!-- gen:links -->
| Link | g | CoM x / y / z (mm) | Ixx / Iyy / Izz (g·mm²) |
| --- | ---: | ---: | ---: |
| `base_link` | 243.0 | -1.2 / 0.0 / 65.6 | 245,159 / 178,253 / 123,761 |
| `neck_yaw_link` | 12.8 | 0.0 / 0.0 / 17.1 | 2,190 / 1,852 / 912 |
| `head_pitch_link` | 65.1 | 9.3 / 0.2 / 25.0 | 55,191 / 47,355 / 60,099 |
| `l_shoulder_pitch_link` | 7.4 | 0.0 / 6.4 / -1.6 | 1,660 / 1,421 / 1,655 |
| `l_shoulder_roll_link` | 13.2 | 0.0 / 0.0 / -9.8 | 2,713 / 2,194 / 1,093 |
| `l_elbow_link` | 17.0 | 0.1 / 0.0 / -7.1 | 4,995 / 5,285 / 1,175 |
| `l_hip_yaw_link` | 16.4 | 0.0 / 0.0 / 18.0 | 2,962 / 2,221 / 1,361 |
| `l_hip_roll_link` | 21.0 | -9.3 / 0.0 / -0.8 | 2,708 / 3,223 / 2,564 |
| `l_hip_pitch_link` | 21.9 | 0.0 / 0.0 / -11.1 | 3,905 / 4,386 / 1,760 |
| `l_knee_link` | 24.0 | 0.0 / 0.0 / -4.1 | 4,645 / 5,121 / 1,903 |
| `l_ankle_pitch_link` | 19.8 | 0.0 / 0.0 / 8.3 | 2,207 / 2,668 / 1,423 |
| `l_ankle_roll_link` | 37.2 | -4.7 / 0.3 / -9.9 | 7,028 / 13,485 / 11,766 |
<!-- /gen:links -->

## Assumptions to check on the bench

- Printed parts: PLA 1.24 g/cm³, walls 1.2 mm (torso, pelvis, head) and 1.0 mm (limbs), no internal ribs.
  Weigh the first printed torso and correct `WALL` in `mico_parts.py` if it differs by more than 10%.
- Brackets, horns and screws are an allowance (3 g per leg servo, 1.5 g per arm and neck servo).
- Wiring is a flat 30 g allowance in the pelvis.
