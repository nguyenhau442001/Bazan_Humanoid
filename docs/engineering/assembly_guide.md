# Assembly guide: Bazan BZN-01

The order to build the printable robot in, what each step needs, and how to take it apart again. Step pictures,
hole tables and per-part drawings are in the two generated manuals, which stay the reference for detail:

- [docs/mechanical/leg_v3.md](../mechanical/leg_v3.md): one leg, 14 steps.
- [docs/mechanical/bazan_v4_lap_rap.md](../mechanical/bazan_v4_lap_rap.md): the whole robot, 21 steps.

This guide adds what those do not say: which sub-assemblies block which screws, the order that follows from it,
and the points where the CAD audit found something to watch. Tool access and removal paths were checked on the CAD
(`check.py`, `robot.py`, `audit.py access`); **the robot has not been assembled**, so treat the sequence as verified
for geometry only.

## Tools

| Tool | For |
| --- | --- |
| Hex key 1.5 mm, ball end, and a short-arm L key | Every M2 socket screw. The L key is needed for the leg-module screws under deck 1 (40 mm of headroom) |
| Hex key 2.0 mm | M3 button heads on the idlers |
| Hex key 2.5 mm | M3×6 on top of the SO35 standoffs |
| Phillips #0 | Horn centre screws, ST2×4 |
| Soldering iron with heat-set tips M2 and M3 | 143 inserts |
| A 1.3 mm stop (two feeler blades, or a printed washer) | Depth of the M3 idler inserts |
| Drills 1.6, 2.1, 2.3, 3.2, 3.3, 4.0 mm, by hand | Clearing holes; 2.1 to open the two horn holes |
| Servo tester or the firmware `calibrate` command | Centring every servo at 1500 µs before its horn goes on |
| Flush cutters, 2.5 mm cable ties, lithium grease | Leads, thrust rings |

## Fasteners

Counts for the whole robot from the generated BOM; buy 20 % more of the small screws.

| Code | Qty | Used for |
| --- | ---: | --- |
| M2×6 socket head | 112 + 16 | Servo ears, leg modules, thrust rings, soles, sleds, boards, flanges; 16 for the shell |
| M3×6 socket head | 4 | Deck 2 onto the SO35 standoffs |
| M3×6 button head | 11 | Idler pivots |
| ST2×4 self-tapping | 40 | Horn to horn plate, two per joint |
| Heat-set insert M2 (OD 3.2, 3 long) | 128 | |
| Heat-set insert M3 (OD 4.2, 3 long) | 15 | 11 idlers, 4 standoff seats |
| MR63ZZ 3×6×2.5 | 11 | Idlers |
| M3 shim washer 3.2×5×0.5 | 11 | On each bearing's inner race |
| Nylon standoff M3 40 / 35 mm | 4 / 4 | Deck 0 → 1 → 2 |

One length of M2 screw throughout. Do not substitute M2×8 at the leg modules: it bottoms in the 4 mm insert hole
before it clamps. At the soles an M2×6 stands 2 mm above the foot plate; it touches nothing, but M2×4 sits flush if
you have them.

## Before assembly

1. **Measure** one servo of each type, one horn, one plug and every board against `hardware.py` and
   `electronics.py` (items marked MEASURE). Regenerate and reprint if anything differs by more than 0.2 mm.
2. **Print and pass the two coupons** (`print_guide.md`): a servo drops into both pockets of the hip block with its
   plug fed through the window; a horn plate and an idler plate of the shin take a horn, a bearing and their caps.
3. **Inserts**, all of them, before anything is assembled:
   - M2 into servo ear bosses: iron at 200 °C, about 3 s, with a servo (or a block its size) in the pocket. The
     insert is 0.7 mm from the pocket wall; pushing too hot or too long bulges the wall and the servo no longer fits.
   - M3 into the back wall of each idler pocket: press from outside against the 1.3 mm stop so the brass face stands
     1.3 mm proud. That face is the shoulder the bearing race clamps against.
   - M2 and M3 through the plates: flush with the marked face.
4. **Centre every servo** at 1500 µs and leave it there. A horn goes on only with its servo centred and its link in
   the zero pose.
5. **Label every lead** at its plug (`L hip roll`, …) and fit the extensions from the lead table in `leg_v3.md`:
   ankle roll and ankle pitch +200 mm, knee +100 mm.

## Order of assembly

Build sub-assemblies on the bench, then stack them. Each line gives the manual step with the detail.

### A. Legs (twice) — `leg_v3.md` steps 1 to 13

1. Bearings into the idler plates (thigh, shin ×2, yaw bracket, foot); horns onto horn plates with 2 × ST2×4 from
   the servo side.
2. Servos into pockets: feed the **plug flat through the window** at the far end, lower the servo while drawing the
   lead through, two M2×6 in the ears. Ankle roll and ankle pitch into the ankle block, knee into the thigh, hip
   roll and hip pitch into the hip block, hip yaw into the hip mount from below.
3. Sole under the foot (4 × M2×6 from below).
4. Close the joints from the foot up: ankle roll, ankle pitch, knee, hip pitch, hip roll. Each one: plate onto the
   spline, horn centre screw through the access hole, washer and M3 button head into the idler insert. Snug only;
   the bearing must turn.
5. Hip yaw: bracket onto the yaw servo, centre screw from below, thrust ring with 2 × M2×6, a drop of grease. Check
   0.2 mm of play under the ring.
6. Leads: up the back of the leg through the tie slots, one 30 mm loop behind every joint the lead crosses. Do not
   tie the loops flat.
7. Caps last, after the range test (T5 in `mechanical_verification.md`).

### B. Pelvis — manual steps 8 to 12

8. Both leg modules onto the pelvis plate: 4 × M2×6 each from above. **Do this before anything else goes on the
   plate**: the outer two screws of each module are under the PCA9685 sled.
9. Four SO40 standoffs.
10. Two PCA9685 on their sleds; leg leads up through the two slots at the back of the plate and into PCA #1.
11. Battery between its walls, strap through the slots.

### C. Deck 1 on the bench — steps 13, 14

12. UBEC, buck, two INA226 and the STM32 board on their sleds, sleds onto deck 1, all wiring between them done and
    tested **now**. Once deck 2 is on, deck 1 cannot be reached from above.
13. Deck 1 onto the SO40s with the four SO35s.

### D. Upper module on the bench — steps 15 to 21

14. On deck 2: Radxa sled, neck mount with the neck-yaw servo, both shoulder mounts with their shoulder-pitch
    servos.
15. Neck link, head-pitch servo, head frame, camera mount, display.
16. Each arm on the bench (shoulder link, upper arm, forearm), then onto its shoulder.
17. Arm and head leads down through the back notch of deck 2.
18. The whole upper module onto the SO35s with 4 × M3×6; leads through the back notch of deck 1 to PCA #2.

### E. Shell

19. Chest and back shells: 6 × M2×6 down into deck 2, 6 up into the pelvis plate from below. Head shells: 4 × M2×6
    dropped down the guide tubes from the crown. Visor glued to the head front beforehand.

## Lead connection sequence

| Group | Leaves through | Enters the torso at | Goes to | Slack |
| --- | --- | --- | --- | --- |
| Left leg, 6 leads | Pocket windows, back of the leg, tie slots | Left slot in the pelvis plate | PCA #1 | 30 mm per joint crossed |
| Right leg, 6 leads | Same, mirrored | Right slot | PCA #1, behind the battery wall | 30 mm per joint |
| Arms, 3 + 3 | Pocket windows, back of the arm | Back notch of deck 2, then deck 1 | PCA #2 | 40 mm round the shoulder |
| Neck and head, 2 | Behind the neck mount | Back notch of decks 2 and 1 | PCA #2 | 60 mm at the neck (it turns ±90°) |
| Camera ribbon | Behind the camera mount | Straight down | Radxa CSI | 40 mm |

Plug each servo at its extension near the PCA, not at the PCA itself: a servo is then unplugged without touching the
driver board. Channel order is in `hardware_architecture.md`.

## Taking it apart

What has to come off before a screw can be reached, from `audit.py access` and the service checks:

| To reach | Remove first |
| --- | --- |
| Any shell screw | Nothing |
| Deck 2 standoff screws | Back shell |
| Shoulder and neck flange screws | Nothing (the torso shell stops at deck 2) |
| Radxa sled | The Radxa board |
| Anything on deck 1 (STM32 board, UBEC, buck, INA226) | Shell, then the **upper module** (4 × M3×6, unplug arm and head leads at PCA #2) |
| PCA9685 sled | Shell; reach in between pelvis and deck 1; the board comes off its sled first |
| Battery | Shell |
| Leg-module screws in the pelvis plate | Shell, PCA board and sled on that side; L key under deck 1 |

### Replacing one servo

Only that servo's joint is opened. Pull both caps, undo the horn centre screw and the idler screw, lift the driven
link off, undo the two ear screws, unplug at the extension, slide the servo out along its shaft **with its plug
through the window**. About 6 screws.

| Servo | Lift off | Servo comes out |
| --- | --- | --- |
| Ankle roll | Foot | Forward |
| Ankle pitch | Shin at the ankle | Outboard |
| Knee | Shin at the knee | Outboard |
| Hip pitch | Thigh | Outboard |
| Hip roll | The leg at the yaw bracket | Forward |
| Hip yaw | The leg at the yaw bracket, the yaw bracket (centre screw from below), the thrust ring | Downward |
| Shoulder pitch | The arm (centre screw through the Ø6 hole in the shoulder link) | Outboard |
| Shoulder roll | Upper arm | Forward |
| Elbow | Forearm, with the arm raised sideways to roll 90° | Toward the body |
| Head pitch | Head frame | To the left |
| Neck yaw | Head frame, head-pitch servo, then the neck link (centre screw down through the neck link) | Upward |

The head shell must be off for the neck yaw and head pitch servos; every leg and arm servo is reached with the
shell on.

### Replacing a whole limb or module

| Module | How |
| --- | --- |
| Leg | Open the hip-roll joint (3 screws): the leg below the yaw bracket comes away with five of its six servos. Unplug five leads at their extensions |
| Leg with its hip mount | As in the table above: PCA sled out, 4 × M2×6 with the L key |
| Arm | Shoulder-pitch centre screw; three leads |
| Head | Head-pitch joint (2 screws) for the head frame alone; neck flange (4 × M2×6) for neck and head together |
| Upper module | 4 × M3×6 on the SO35s |

## Not verified by this guide

- That a hand and an L key fit between the pelvis plate and deck 1 with the battery and PCA boards in place.
- That 20 extension plugs and their slack fit inside the shell.
- Insert pull-out and the effect of heat on the 0.7 mm pocket wall.
- The time and screw counts quoted for a servo change.
