# Torque validation

Which servos are loaded, how hard, and against which limit. Tables between `gen:` markers are written by
`python docs/engineering/gen_docs.py`; CI fails if they drift from the sources.

<!-- gen:stamp -->
Generated from `docs/engineering/cad_mass.json`, `software/ros2/src/humanoid_description/urdf/bazan_cad.urdf`, `software/ai/sim/mujoco/servo_loads.csv`, `software/ros2/src/humanoid_control/scripts/bazan_gait.py`, `hardware/mechanical/hardware.py`, `docs/engineering/bom_aplus.csv` · source hash `595b6dd2e927`
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

## Leg servos on the CAD robot

MuJoCo (`sim/mujoco/check_mujoco.py`) runs the demo gait on the robot generated from the CAD (`bazan_cad.urdf`,
the same masses as `mass_budget.md`), so peaks and RMS are measured on the robot that will be printed, not scaled.
Static loads come from the gait's planned poses on the same URDF.

<!-- gen:legs -->
MuJoCo on the CAD robot (1083 g), the demo gait (53.5 s). Static worst is the larger of the left and right joint over two planned poses: the gait's start pose and halfway through a swing on one foot.

| Joint | Servo | Stall kg·cm | Peak p99 (/ stall) | RMS (/ continuous) | Time above continuous | Static worst (/ continuous) | Flag |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `l_hip_yaw` | MG90S | 2.20 | 0.18 (8%) | 0.06 (8%) | 0% | 0.02 (3%) | – |
| `l_hip_roll` | MG92B | 3.50 | 1.76 (50%) | 0.95 (81%) | 35% | 1.54 (132%) | thermal |
| `l_hip_pitch` | MG92B | 3.50 | 0.56 (16%) | 0.21 (18%) | 0% | 0.10 (8%) | – |
| `l_knee` | MG92B | 3.50 | 2.66 (76%) | 1.46 (125%) | 39% | 2.67 (229%) | thermal |
| `l_ankle_pitch` | MG92B | 3.50 | 1.00 (29%) | 0.35 (30%) | 0% | 0.12 (11%) | – |
| `l_ankle_roll` | MG92B | 3.50 | 1.16 (33%) | 0.54 (46%) | 1% | 1.52 (130%) | – |
| `r_hip_yaw` | MG90S | 2.20 | 0.16 (7%) | 0.06 (8%) | 0% | 0.02 (3%) | – |
| `r_hip_roll` | MG92B | 3.50 | 1.80 (51%) | 1.00 (86%) | 39% | 1.54 (132%) | thermal |
| `r_hip_pitch` | MG92B | 3.50 | 0.50 (14%) | 0.19 (17%) | 0% | 0.10 (8%) | – |
| `r_knee` | MG92B | 3.50 | 2.67 (76%) | 1.46 (125%) | 39% | 2.67 (229%) | thermal |
| `r_ankle_pitch` | MG92B | 3.50 | 1.01 (29%) | 0.36 (31%) | 0% | 0.12 (11%) | – |
| `r_ankle_roll` | MG92B | 3.50 | 1.18 (34%) | 0.55 (47%) | 1% | 1.52 (130%) | – |
<!-- /gen:legs -->

Flags: **peak** when p99 is above 80% of stall, **thermal** when RMS is above 60% of the continuous limit
or the joint spends more than 10% of the run above it.

## Verdict

<!-- gen:verdict -->
No leg servo is shown to fail. These joints need bench data before P1:

- `l_hip_roll`: thermal (peak 50% of stall, RMS 81% of continuous, 35% of the run above continuous)
- `l_knee`: thermal (peak 76% of stall, RMS 125% of continuous, 39% of the run above continuous)
- `r_hip_roll`: thermal (peak 51% of stall, RMS 86% of continuous, 39% of the run above continuous)
- `r_knee`: thermal (peak 76% of stall, RMS 125% of continuous, 39% of the run above continuous)
<!-- /gen:verdict -->

Both flags come from single support, not from fast motion. The stance hip roll holds the torso and the swing
leg; the gait already rolls the pelvis toward the stance foot (`LEAN` in `bazan_gait.py`) to cut that load. The
stance knee is bent further than in double support (`H0_LEANED`) so the free foot still reaches the floor under
the leaned pelvis. Less lean means a straighter knee but more hip-roll load: the two trade against each other.
The fixes, in order of cost:

1. Measure it: the P0 thermal test in `hardware_architecture.md`, on the knee as well as the hip roll.
2. Lower the load: lighter printed frame and shell (`mass_budget.md`: the printed frame is the largest group),
   shorter single-support phase (`T_SWING`), or a `LEAN` / `H0` trade-off chosen from the P0 data.
3. Only if P0 fails: a stronger servo on the joints that fail.

## Arms, neck and the other joints

<!-- gen:others -->
| Joint | Servo | Stall kg·cm | Peak p99 (/ stall) | RMS (/ continuous) | Time above continuous | Static worst (/ continuous) | Flag |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `l_shoulder_pitch` | MG90S | 2.20 | 0.15 (7%) | 0.04 (6%) | 0% | 0.07 (9%) | – |
| `l_shoulder_roll` | SG90 | 1.80 | 0.04 (2%) | 0.03 (4%) | 0% | 0.04 (6%) | – |
| `l_elbow` | SG90 | 1.80 | 0.09 (5%) | 0.02 (3%) | 0% | 0.02 (3%) | – |
| `r_shoulder_pitch` | MG90S | 2.20 | 0.61 (28%) | 0.10 (14%) | 0% | 0.07 (9%) | – |
| `r_shoulder_roll` | SG90 | 1.80 | 0.05 (3%) | 0.03 (5%) | 0% | 0.04 (6%) | – |
| `r_elbow` | SG90 | 1.80 | 0.37 (21%) | 0.07 (12%) | 0% | 0.02 (3%) | – |
| `neck_yaw` | SG90 | 1.80 | 0.13 (7%) | 0.03 (5%) | 0% | 0.02 (3%) | – |
| `head_pitch` | SG90 | 1.80 | 0.10 (6%) | 0.09 (15%) | 0% | 0.09 (15%) | – |
<!-- /gen:others -->
