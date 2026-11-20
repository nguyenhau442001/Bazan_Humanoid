# Torque validation

Which servos are loaded, how hard, and against which limit. Tables between `gen:` markers are written by
`python docs/engineering/gen_docs.py`; CI fails if they drift from the sources.

<!-- gen:stamp -->
Generated from `docs/engineering/cad_mass.json`, `software/ros2/src/humanoid_description/urdf/bazan_cad.urdf`, `software/ai/sim/mujoco/servo_loads.csv`, `software/ros2/src/humanoid_control/scripts/bazan_gait.py`, `hardware/mechanical/hardware.py`, `docs/engineering/bom_aplus.csv` · source hash `2c8d7fa93a07`
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
MuJoCo on the CAD robot (1804 g), the demo gait (53.5 s). Static worst is the larger of the left and right joint over two planned poses: the gait's start pose and halfway through a swing on one foot.

| Joint | Servo | Stall kg·cm | Peak p99 (/ stall) | RMS (/ continuous) | Time above continuous | Static worst (/ continuous) | Flag |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `l_hip_yaw` | MG90S | 2.20 | 0.49 (22%) | 0.19 (26%) | 0% | 0.13 (18%) | – |
| `l_hip_roll` | DS3218 | 20.00 | 6.04 (30%) | 3.33 (50%) | 0% | 5.41 (81%) | – |
| `l_hip_pitch` | DS3218 | 20.00 | 2.24 (11%) | 0.88 (13%) | 0% | 0.62 (9%) | – |
| `l_knee` | DS3225 | 22.00 | 7.01 (32%) | 3.58 (49%) | 0% | 6.10 (83%) | – |
| `l_ankle_pitch` | DS3218 | 20.00 | 2.46 (12%) | 0.77 (12%) | 0% | 0.25 (4%) | – |
| `l_ankle_roll` | DS3218 | 20.00 | 2.86 (14%) | 1.08 (16%) | 0% | 3.00 (45%) | – |
| `r_hip_yaw` | MG90S | 2.20 | 0.51 (23%) | 0.19 (25%) | 0% | 0.13 (18%) | – |
| `r_hip_roll` | DS3218 | 20.00 | 5.91 (30%) | 3.33 (50%) | 0% | 5.41 (81%) | – |
| `r_hip_pitch` | DS3218 | 20.00 | 2.20 (11%) | 0.87 (13%) | 0% | 0.62 (9%) | – |
| `r_knee` | DS3225 | 22.00 | 6.98 (32%) | 3.54 (48%) | 0% | 6.10 (83%) | – |
| `r_ankle_pitch` | DS3218 | 20.00 | 2.77 (14%) | 0.82 (12%) | 0% | 0.25 (4%) | – |
| `r_ankle_roll` | DS3218 | 20.00 | 2.15 (11%) | 1.02 (15%) | 0% | 3.00 (45%) | – |
<!-- /gen:legs -->

Flags: **peak** when p99 is above 80% of stall, **thermal** when RMS is above 60% of the continuous limit
or the joint spends more than 10% of the run above it.

## Verdict

<!-- gen:verdict -->
No leg joint is flagged at the current mass.
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
| `l_shoulder_roll` | SG90 | 1.80 | 0.04 (2%) | 0.02 (4%) | 0% | 0.04 (6%) | – |
| `l_elbow` | SG90 | 1.80 | 0.09 (5%) | 0.02 (3%) | 0% | 0.02 (3%) | – |
| `r_shoulder_pitch` | MG90S | 2.20 | 0.61 (28%) | 0.10 (14%) | 0% | 0.07 (9%) | – |
| `r_shoulder_roll` | SG90 | 1.80 | 0.05 (3%) | 0.03 (4%) | 0% | 0.04 (6%) | – |
| `r_elbow` | SG90 | 1.80 | 0.37 (21%) | 0.07 (12%) | 0% | 0.02 (3%) | – |
| `neck_yaw` | SG90 | 1.80 | 0.13 (7%) | 0.03 (5%) | 0% | 0.01 (2%) | – |
| `head_pitch` | SG90 | 1.80 | 0.10 (5%) | 0.08 (13%) | 0% | 0.08 (13%) | – |
<!-- /gen:others -->
