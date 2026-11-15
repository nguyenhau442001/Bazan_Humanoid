# Mechanical audit: Bazan BZN-01

Audit of the printable CAD in `hardware/mechanical/` against five questions: can it be printed, assembled,
serviced, moved through its joint ranges, and does it carry its loads. Numbers come from
`python hardware/mechanical/audit.py` (plus `check.py`, `robot.py`, `shell.py`) run on 2026-11-15; the full results
are in [mechanical_verification.md](mechanical_verification.md). Nothing here has been printed or assembled:
every verdict is about geometry and arithmetic, and the bench tests that would turn CONDITIONAL into PASS are listed
at the end.

## The finding that decides everything else

The repository holds **two different robots under one name**, and only one of them can be printed.

| | Digital twin | Printable CAD |
| --- | --- | --- |
| Source | `software/ai/sim/mini_humanoid/gen_robot.py` → URDF, MuJoCo, firmware tables; shell and mass from `design/blender/bazan_parts.py` (Design C) | `hardware/mechanical/leg.py`, `body.py`, `shell.py` (leg v3, body v4, shell v1) |
| Height | 303 mm | 407 mm (floor to helmet top) |
| Mass | 677 g (PLA), `mass_budget.md` | about 1070 g (estimate, see verification §5) |
| Hip point above the floor | 115 mm | 137.3 mm |
| Leg centre lines | y = ±22 mm | y = ±28 mm |
| Thigh / shin | 45 / 45 mm | 48 / 48 mm |
| Hip roll axis | through the hip point | 20 mm above the hip-pitch axis |
| Ankle roll axis | through the ankle point | 20 mm below the ankle-pitch axis |
| Shoulder pitch axis above the hip point | 97 mm | 169.8 mm |
| Head pitch axis above the hip point | 132 mm | 207.5 mm |
| Fastener, pocket, insert detail | none (boxes and 1.0 / 1.2 mm skins) | every hole placed |

The twin is what the RL policy is trained on, what `mass_budget.md` and `torque_validation.md` describe, and what
the README advertises. The CAD is what `gen_docs.py` exports as STL. The CAD manual already says so in its open
items ("robot cao ~400 mm ... URDF cần sinh lại"); it was never done. Until one is regenerated from the other, a
policy trained in simulation is trained for a robot that will not be built, and the torque validation does not
apply to the parts on the printer.

This audit did **not** change any joint axis, origin or limit on either side. Which robot is Bazan is a project
decision, not a CAD fix; the options are in "Decisions needed" below.

## Inventory

### Authoritative sources (edit these)

| File | Defines |
| --- | --- |
| `hardware/mechanical/hardware.py` | Servo, horn, plug, fastener and hole sizes; fit rules. Marked MEASURE: datasheet values, not calipers |
| `hardware/mechanical/joint.py` | The one joint cartridge used 20 times: pocket, ear inserts, cable window, horn plate, idler plate with MR63ZZ, caps, tool paths |
| `hardware/mechanical/leg.py` | Leg v3: 9 printed parts per leg, 6 joints, tie slots, lead routes |
| `hardware/mechanical/body.py` | Body v4: pelvis plate, decks 1 and 2 on standoffs, 6 sleds, shoulder / arm / neck / head parts, camera mount |
| `hardware/mechanical/shell.py` | Shell v1: chest, back, head front, head back, visor on the 16 shell inserts |
| `hardware/mechanical/electronics.py` | Board envelopes and hole patterns (MEASURE) |
| `hardware/mechanical/pose.py`, `robot.py` | Kinematic chains used to pose the CAD |
| `hardware/mechanical/cabling.py`, `sensors.py`, `cots_loader.py` | Display-only harness, sensor and bought-part solids for the assembly export |
| `software/ai/sim/mini_humanoid/gen_robot.py` | The twin: joint origins, axes, limits, servo allocation |
| `design/blender/bazan_parts.py`, `bazan_mass.py` | Design C skin meshes and the 677 g mass model of the twin |

### Checks

| Script | Asks |
| --- | --- |
| `check.py` | Leg: overlaps over the CAD joint ranges and the gait, servo slide-out, tool access, both legs together |
| `robot.py` | Whole frame: static fit, arm / neck / head sweeps, tool access, servo slide-out |
| `shell.py` | Shell against the frame over the arm / neck / head sweeps |
| `audit.py` (new) | Twin alignment, URDF limits, inward reach, motion matrix with clearance, walls round holes, fits, print orientation, mass and CoM, lead stretch, access to body and shell screws |

### Generated artifacts (never edit)

| Artifact | Written by | Tracked |
| --- | --- | --- |
| `hardware/mechanical/out/`: 41 STL and 41 STEP print files for the legs and frame (left and mirrored right), 5 shell files, `bazan_assembly.step`, `audit.json` | `gen_docs.py`, `gen_manual.py`, `shell.py`, `export_assembly.py`, `audit.py` | no (CI artifact `mico-print-files`) |
| `docs/mechanical/leg_v3.md`, `drawings/*.png` | `gen_docs.py` | yes |
| `docs/mechanical/bazan_v4_lap_rap.md`, `.html`, `v4/` | `gen_manual.py` | yes |
| URDF, `mico.xml`, firmware tables | `gen_robot.py`, `gen_tables.py` | yes |
| `docs/engineering/mass_budget.md`, `torque_validation.md` (blocks between `gen:` markers) | `docs/engineering/gen_docs.py` | yes |

### Mechanical content found

- **Printed parts:** 9 per leg, 17 body parts (6 of them mirrored left / right), 5 shell parts: 31 distinct shapes.
- **Joints:** 20, all on the same cartridge. Idler bearing on the 10 leg roll / pitch joints and head pitch; hip yaw
  carries weight on a printed thrust ring; arm joints and neck yaw hang on the servo spline alone.
- **Fasteners:** M2×6 (112 + 16 for the shell), M3×6 (4), M3×6 button (11), ST2×4 self-tapping (40), heat-set
  inserts M2 (128) and M3 (15), MR63ZZ (11), nylon M3 standoffs 40 and 35 mm (4 each).
- **Mechanical stops:** none. Range is limited only by the servo and the controller.
- **Cable features:** one window per pocket, 5 tie-slot pairs per leg, two slots in the pelvis plate, a back notch in
  decks 1 and 2.

### Missing or unverifiable

- Caliper measurements of the servos, horns, plugs and boards actually bought (all MEASURE).
- Printer, material and slicer profile; no printed part exists, so no fit has been tried.
- Any FEA or load test. Strength statements below are hand estimates.
- A cable model: `cabling.py` draws splines through hand-typed points and is not derived from the parts.
- A Design C shell that fits this frame. `shell.py` v1 is the only skin that does.

## Findings

Severity: **P0** stops assembly, intended motion, or makes a servo unserviceable; **P1** significant manufacturing,
maintenance or reliability problem; **P2** minor.

| # | Sev | Component | Finding | Evidence | Status |
| --- | --- | --- | --- | --- | --- |
| 1 | P0 | Whole robot | Printable CAD and digital twin are different robots (table above) | `audit.py twin`: 6 of 11 joint axes off by 3 to 75 mm; height 407 vs 303 | **Open: decision needed** |
| 2 | P0 | Hip roll | Holding the CAD robot on one foot with the pelvis level takes about 0.29 N·m at the stance hip roll: 84 % of MG92B stall, 2.5 times the project's continuous rule. The same sum on the twin gives 0.146 N·m, so the documented "68 % RMS, thermal" becomes roughly double | `audit.py mass` | **Open: follows from 1** |
| 3 | P0 | Hip roll, ankle roll | The twin allows hip roll ±25° and ankle roll ±30°; the parts were drawn for ±20°. Hip block meets the yaw bracket at −25°; ankle block meets the foot from −25° | `audit.py limits` | **Open: decision needed** |
| 4 | P0 | Hip roll (inward), shoulder roll | A leg adducts 4° before the ankle blocks meet (twin allows 25°); an arm rolls inward 3° before the forearm meets the torso shell (twin allows 10°) | `audit.py reach` | **Open: limits belong in the twin** |
| 5 | P0 | All 20 servo pockets | The cable window was 5.1 × 5.0 mm. A 3-pin plug is 7.7 × 2.6 mm and cannot pass on any diagonal, so no servo could be fitted or removed without depinning its plug | `audit.py fits` | **Fixed:** window 8.3 × 5.0 |
| 6 | P1 | `hip_mount` | Its "cable window" removed half of the boss that holds the far ear insert of the hip-yaw servo; the insert had material on one side only | `audit.py walls`: 0.0 mm ring | **Fixed:** window starts 1.6 mm below the insert |
| 7 | P1 | All servo ears (40 inserts) | The M2 insert hole edge is 0.7 mm from the servo pocket (hole pitch 27.8, case 22.8, 0.2 clearance). A heat-set insert can bulge that wall into the pocket | `audit.py walls`, analytic | Open: coupon test T2; geometry is fixed by the servo |
| 8 | P1 | Leg leads | The lead table counted one joint too few for the ankle-pitch and hip-roll servos (both sit in a block below the joint they drive): 30 mm of slack missing each | `leg.cable_routes` | **Fixed:** 342 → 408 mm, 154 → 200 mm |
| 9 | P1 | Leg module screws | `leg_v3.md` called for M2×8 into the hip mount; `body.py` for M2×6. An M2×8 goes 5.7 mm into a 4.0 mm hole and bottoms before clamping | `audit.py fits` | **Fixed:** M2×6 everywhere, M2×8 dropped from the leg BOM |
| 10 | P1 | Generated documents, CI | Three checks failed before this audit: `docs/engineering/gen_docs.py --check` (renamed source files), `hardware/mechanical/gen_docs.py --check` (stale), and the lint workflow's mass step. `gen_manual.py` wrote to an old file name, and the committed manual described an arm 6 mm narrower than the CAD | reproduced on the untouched tree | **Fixed** and regenerated |
| 11 | P1 | Torso | The STM32 board, UBEC, buck and INA226 on deck 1 can only be reached from above after the shell and deck 2 come off, and deck 2 carries both arms and the head. The four pelvis screws of a hip mount are under the PCA9685 sled | `audit.py access` | Open: documented in the assembly guide; a leg still comes off at its hip-roll joint |
| 12 | P1 | Pelvis plate, decks | About 101 g of flat plate (38 + 31 + 31 g estimated) sits above the hips; it is the largest single contributor to finding 2 that printing can change | `audit.py mass` | Open: proposal below |
| 13 | P2 | Foot, sleds, horn plates | M2×6 through the TPU sole stands 2.0 mm above the foot plate (clear of the ankle block: screws at x = ±31, block spans −16.6…8.3). Sled screws stand 0.2 mm below a deck. ST2×4 is 0.1 mm longer than its 2.4 mm pilot | `audit.py fits` | Open: use M2×4 at the sole if stocked; check the ST2 on coupon T3 |
| 14 | P2 | `cam_mount` | 0.8 mm of plastic under the two screw heads; 0.9 mm wall round the six board inserts | `audit.py walls` | Open: lightly loaded; press those inserts cool or use glue |
| 15 | P2 | Idler pivot (11 joints) | The M3 insert stands 1.3 mm proud of the pocket wall inside a 0.1 mm printed sleeve and its face seats the bearing race. Nothing sets its depth | `joint.py` BOSS | Open: press against a 1.3 mm stop (assembly guide) |
| 16 | P2 | `cabling.py` | Harness points are typed in, not derived (leg y = 28.0 literal); display only | reading | Open |
| 17 | P2 | Shell | Design C (Blender) is drawn on the twin and does not fit the frame; the frame's own shell v1 is boxier | `shell.py`, manual open item 5 | Open: follows from 1 |

### What was verified and found sound

- Within the ranges the CAD was drawn for, nothing overlaps: 86 leg poses, the whole frame at rest, 40 arm / neck /
  head poses, the shell over the same sweeps, and the demo gait.
- All 20 servos slide out along their own shaft once the one driven link of that joint is off.
- All 29 leg screws and every upper-body joint screw have a straight 35 mm tool path, some after removing the parts
  listed in the service tables.
- All 31 printed shapes are valid single solids. No part needs to stand taller than 96 mm.
- Constants copied between files agree: pelvis height, leg-module screw pattern, all 16 shell insert positions.
- Lead stretch over the full twin ranges is at most 28 mm against 60 to 150 mm of slack.

## Corrections implemented

All in the parametric sources; every generated file was regenerated from them.

| Geometry change | Where | Effect on mass |
| --- | --- | --- |
| Cable window widened from `cable_w + 0.6` = 5.1 mm to `max(cable_w, PLUG) + 0.6` = 8.3 mm; new `hardware.PLUG = (7.7, 2.6)` | `joint.py` `WINDOW`, `holder_cut` (all 20 pockets) | −0.04 to −0.09 cm³ per holder part |
| Hip-mount cable window raised to start 1.6 mm below the far ear insert | `leg.py` `_hip_mount_body` | +0.03 cm³ per hip mount |

No servo mounting interface, joint axis, joint origin, link length or fastener position changed. The window sits in
the end wall below the ear boss; the insert itself is untouched (ring check unchanged at 0.7 mm).

Non-geometry: lead-crossing count (`leg.cable_routes`), screw length and wording in `gen_docs.py`, output names in
`gen_manual.py`, source names in `docs/engineering/gen_docs.py` and `.github/workflows/lint.yaml`.

## Decisions needed

1. **Which robot is Bazan?**
   - *Regenerate the twin from the CAD* (recommended): `gen_robot.py` takes its joint origins, link masses and limits
     from `leg.py` / `body.py`; URDF, MuJoCo and firmware tables follow; mass budget and torque validation are re-run;
     the policy is retrained. This is the only route that ends in a printable robot, and it will show hip roll
     over its limit (finding 2), which then has to be solved by mass or by a stronger hip-roll servo.
   - *Shrink the CAD to the twin*: not possible with these servos. Intersecting hip axes at a 22 mm half-spacing do
     not leave room for three 23 × 12 × 23 mm cases; that is why leg v3 offset them.
2. **Roll ranges.** Either narrow the twin to what the parts allow (hip roll −4…+20° per leg in the adduction /
   abduction sense, ankle roll ±20°, shoulder roll −3…160°), or widen the parts: setting `HIP_ROLL_MAX = 25` and
   `ANKLE_ROLL_MAX = 30` in `leg.py` clears both by raising the yaw bracket 1.3 mm and lowering the sole 2.4 mm
   (about +0.5 g per foot, robot 3.7 mm taller, ankle-roll lever to the floor +11 %). The gait uses ±17°.
3. **Mass.** If option 1 is taken, lighten before retraining: windows in the three plates between the bay holes
   (order of 30 to 40 g), 2-perimeter 15 % decks as the manual already suggests, LW-PLA shell. Each is a small
   edit in `body.py`; none was made here because the target mass depends on decision 1.

## Remaining risks

- Every fit depends on MEASURE values. A servo 0.3 mm longer than the datasheet does not enter its pocket.
- Heat-set inserts 0.7 mm from a pocket wall (finding 7) may need a fallback: an M2 thread-forming screw in a
  1.7 mm pilot. That would change `hardware.HOLES` and the BOM, not the part envelopes.
- Driven links printed lying on their side carry spreading loads across the layers of the spine.
- Arm joints and neck yaw have no idler: side loads on the hand go through the servo output bearing.
- No hard stops: a controller fault drives a link into its neighbour at stall torque.
- Lead routing was checked as a polyline for length only. Pinching, twist at the hip yaw, and chafing on printed
  edges are not modelled.

## Acceptance gate

| Category | Verdict | Basis |
| --- | --- | --- |
| 3D printability | CONDITIONAL | All 31 parts inspected for validity, orientation, overhang, walls round holes; no slicer run, no print |
| Assembly | CONDITIONAL | Sequence exists and tool paths are clear in CAD; plug now passes; never assembled |
| Serviceability | CONDITIONAL | Each servo leaves by opening one joint (geometry); deck-1 boards need the upper module off |
| Kinematics | FAIL | Findings 3 and 4: the twin's ranges exceed the parts on hip roll, ankle roll, shoulder roll |
| Collision | FAIL | Clear inside the CAD ranges and the gait; unresolved at the twin's limits |
| Cable routing | CONDITIONAL | Length and plug passage verified; pinch and twist need the bench |
| Structural integrity | NOT VERIFIED | Hand estimates only; no FEA, no load test |
| Mass budget | FAIL | `mass_budget.md` describes the 677 g twin; the CAD estimates about 1070 g |
| URDF compatibility | FAIL | Joint geometry not synchronised; nothing was changed silently |
| Documentation | PASS | Four deliverables written; generated documents regenerated and their checks pass |

## Summary

### Five most important problems

1. The printable CAD is not the robot that is simulated, budgeted and trained (finding 1).
2. On the printable robot the stance hip roll holds about 84 % of stall just standing on one foot (finding 2).
3. Hip roll, ankle roll and shoulder roll cannot reach the limits the twin and firmware allow (findings 3, 4).
4. No servo could be installed with its plug on (finding 5, fixed).
5. Servo ear inserts sit 0.7 mm from the pocket wall on every joint (finding 7), and one was half cut away
   (finding 6, fixed).

### Files modified

Sources: `hardware/mechanical/hardware.py`, `joint.py`, `leg.py`, `gen_docs.py`, `gen_manual.py`, `README.md`;
`docs/engineering/gen_docs.py`; `.github/workflows/lint.yaml`. New: `hardware/mechanical/audit.py`,
`docs/engineering/mechanical_audit.md`, `assembly_guide.md`, `print_guide.md`, `mechanical_verification.md`.
Regenerated: `docs/mechanical/leg_v3.md`, `drawings/*.png`, `bazan_v4_lap_rap.md`, `.html`, `v4/**`, and the stamp
lines of `docs/engineering/mass_budget.md`, `torque_validation.md`, `hardware_architecture.md`.

Not touched: firmware, control, URDF, MuJoCo, RL, M0 criteria.

### Mass before and after

| | Before | After |
| --- | ---: | ---: |
| Printed frame, estimate | 468 g | 467 g |
| Whole robot from the CAD, estimate | 1069 g | 1068 g |
| Mass budget document (twin) | 677 g | 677 g (unchanged) |

### Physical tests before printing the whole robot

See [mechanical_verification.md](mechanical_verification.md) §9 for the procedure and pass criteria: T1 measure the
bought parts, T2 pocket coupon, T3 joint coupon, T4 one leg, T5 leg range and leads, T6 hip-roll thermal, T7 torso
and arms, T8 whole robot on the scale.
