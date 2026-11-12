# Torque validation

Which servos are loaded, how hard, and against which limit. Tables between `gen:` markers are written by
`python docs/engineering/gen_docs.py`; CI fails if they drift from the sources.

<!-- gen:stamp -->
Generated from `design/blender/mico_parts.py`, `design/blender/mico_mass.py`, `sim/mini_humanoid/gen_robot.py`, `sim/mini_humanoid/tables/mj_servo_loads.csv`, `src/humanoid_control/scripts/mico_gait.py`, `docs/engineering/bom_aplus.csv` · source hash `3389b8746293`
<!-- /gen:stamp -->

## Which number to compare with which limit

Two different limits, two different questions:

| Load | Compared with | Question it answers |
| --- | --- | --- |
| Peak (MuJoCo p99) | **stall torque** | Can the servo push through the worst moments at all? |
| RMS over the gait | **continuous limit = 1/3 stall** | Will it overheat over a long run? |
| Static worst case | continuous limit | Can it hold the pose indefinitely? |

1/3 stall is this project's design rule for continuous load, not a manufacturer rating. TowerPro does not
publish one for the MG92B, which is why P0 measures it.

Comparing p99 peaks against the continuous limit (an earlier review did) makes every leg servo look like it
fails. That pairing is wrong; peaks are short by definition.

## Leg servos at the Design C mass

MuJoCo (`sim/mujoco/check_mujoco.py`) runs the gait on the 536 g box model. Peaks and RMS are scaled by the mass
ratio below, which is a fair first estimate because the leg loads are dominated by gravity. Re-run it on the
`mass:=blender` model to replace the estimate with a measurement. Static loads are already computed on the Design C URDF.

<!-- gen:legs -->
Mass ratio used for the MuJoCo columns: 677 g / 536 g = 1.263. Gait run: 35.8 s. Static worst is the larger of the left and right joint, since the static cases stand on the left foot only.

| Joint | Servo | Stall kg·cm | Peak p99 (/ stall) | RMS (/ continuous) | Time above continuous | Static worst (/ continuous) | Flag |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `l_hip_yaw` | MG90S | 2.20 | 0.87 (39%) | 0.19 (26%) | 0% | 0.01 (1%) | – |
| `l_hip_roll` | MG92B | 3.50 | 1.55 (44%) | 0.68 (59%) | 2% | 1.05 (90%) | – |
| `l_hip_pitch` | MG92B | 3.50 | 1.19 (34%) | 0.26 (23%) | 0% | 0.06 (5%) | – |
| `l_knee` | MG92B | 3.50 | 1.82 (52%) | 0.66 (57%) | 4% | 0.72 (61%) | – |
| `l_ankle_pitch` | MG92B | 3.50 | 1.51 (43%) | 0.46 (40%) | 1% | 0.08 (7%) | – |
| `l_ankle_roll` | MG92B | 3.50 | 1.10 (31%) | 0.42 (36%) | 1% | 0.00 (0%) | – |
| `r_hip_yaw` | MG90S | 2.20 | 0.58 (26%) | 0.17 (23%) | 0% | 0.01 (1%) | – |
| `r_hip_roll` | MG92B | 3.50 | 1.69 (48%) | 0.80 (68%) | 7% | 1.05 (90%) | thermal |
| `r_hip_pitch` | MG92B | 3.50 | 1.33 (38%) | 0.29 (25%) | 1% | 0.06 (5%) | – |
| `r_knee` | MG92B | 3.50 | 1.53 (44%) | 0.53 (45%) | 1% | 0.72 (61%) | – |
| `r_ankle_pitch` | MG92B | 3.50 | 1.78 (51%) | 0.49 (42%) | 3% | 0.08 (7%) | – |
| `r_ankle_roll` | MG92B | 3.50 | 1.26 (36%) | 0.39 (33%) | 0% | 0.00 (0%) | – |
<!-- /gen:legs -->

Flags: **peak** when p99 is above 80% of stall, **thermal** when RMS is above 60% of the continuous limit
or the joint spends more than 10% of the run above it.

## Verdict

<!-- gen:verdict -->
No leg servo is shown to fail. These joints need bench data before P1:

- `r_hip_roll`: thermal (peak 48% of stall, RMS 68% of continuous, 7% of the run above continuous)
<!-- /gen:verdict -->

Hip roll is loaded by holding the torso tilted over the stance foot for the whole single-support phase,
not by fast motion. The fixes, in order of cost:

1. Measure it: the P0 thermal test in `hardware_architecture.md`.
2. Lower the load: LW-PLA trunk and head shell (see `mass_budget.md`), shorter single-support phase or
   smaller `SWAY` in `mico_gait.py`.
3. Only if P0 fails: a stronger servo on the two hip-roll joints only.

## Arms, neck and the other joints

<!-- gen:others -->
| Joint | Servo | Stall kg·cm | Peak p99 (/ stall) | RMS (/ continuous) | Time above continuous | Static worst (/ continuous) | Flag |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `l_shoulder_pitch` | MG90S | 2.20 | 0.13 (6%) | 0.05 (7%) | 0% | 0.11 (15%) | – |
| `l_shoulder_roll` | SG90 | 1.80 | 0.02 (1%) | 0.01 (1%) | 0% | 0.11 (18%) | – |
| `l_elbow` | SG90 | 1.80 | 0.13 (7%) | 0.02 (3%) | 0% | 0.01 (2%) | – |
| `r_shoulder_pitch` | MG90S | 2.20 | 0.71 (32%) | 0.12 (17%) | 0% | 0.11 (15%) | – |
| `r_shoulder_roll` | SG90 | 1.80 | 0.07 (4%) | 0.02 (3%) | 0% | 0.11 (18%) | – |
| `r_elbow` | SG90 | 1.80 | 0.48 (26%) | 0.11 (19%) | 0% | 0.01 (2%) | – |
| `neck_yaw` | SG90 | 1.80 | 0.16 (9%) | 0.04 (7%) | 0% | 0.01 (1%) | – |
| `head_pitch` | SG90 | 1.80 | 0.04 (2%) | 0.02 (4%) | 0% | 0.14 (23%) | – |
<!-- /gen:others -->
