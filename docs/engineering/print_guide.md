# Print guide: Bazan BZN-01

How to print the 31 shapes of the frame and shell. Orientation figures come from
`python hardware/mechanical/audit.py print`, which tries the six axis orientations of each part and reports bed
contact, overhang area (faces within 45° of facing down, off the bed) and height. They rank orientations by support
need only; where strength or hole roundness argues for a different one, the table says so.

**Nothing here has been printed.** No printer, material batch or slicer profile is recorded in the repository, so
every fit below is a nominal CAD value to be confirmed on the two coupons before the rest is printed.

Print files: `python hardware/mechanical/gen_docs.py` (legs), `gen_manual.py` (frame), `shell.py` (shell) write
`hardware/mechanical/out/{L,R}_<part>.stl` and `.step`. Right-side files are mirrors; print both.

## Order

1. **Coupons** (22 g): one `L_hip_block` and one `L_shin`. Between them they hold every fit in the robot: two servo
   pockets with cable windows, four ear inserts, two idler inserts, two horn plates, two bearing seats.
2. **One leg** (9 parts, about 110 g).
3. Only when that leg passes its range test: the second leg, then the torso, arms and head, then the shell.

## Material and settings by group

| Group | Parts | Material | Walls | Infill | Why |
| --- | --- | --- | --- | --- | --- |
| Joint parts | hip block, ankle block, thigh, shin, yaw bracket, foot, hip mount, thrust ring, shoulder / arm / neck parts, head frame | PLA (PETG where the print room is above 35 °C: a stalled MG92B warms its pocket) | 4 perimeters (1.6 mm) | 40 % gyroid | 2 mm pocket walls and 4 mm arms print solid; inserts need material round them |
| Plates | pelvis plate, deck 1, deck 2, sleds, camera mount | PLA | pelvis 3 perimeters; decks and sleds 2 | 15 to 20 % | Flat, lightly loaded; this is where mass can be saved |
| Soles | `sole` | TPU 95A | 3 | 30 % gyroid | Grip and shock |
| Shell | chest, back, head front, head back | PETG or LW-PLA | 3 perimeters = the whole 1.2 mm skin | – | Cosmetic; LW-PLA saves mass above the hips |
| Visor | `visor` | clear PETG | 3 | – | Glued on its 2.5 mm lip |

Layer height 0.2 mm, 0.4 mm nozzle (the hole sizes in `hardware.py` assume it). First layer no wider than 105 %:
elephant foot closes the bearing seats and the horn recess.

## Part by part

Orientation: which face of the part (in its CAD frame: x forward, y left, z up) points up on the bed. Overhang is the
supported area in that orientation.

### Leg (per leg; right side mirrored)

| Part | Print with | Bed cm² | Overhang cm² | Height mm | Supports | Note |
| --- | --- | ---: | ---: | ---: | --- | --- |
| `foot` | +z up (sole plate on the bed) | 28.6 | 2.0 | 30.3 | Under the two discs only | Uprights are 4 mm walls standing 30 mm: brim |
| `sole` | flat | 28.7 | 0 | 3.0 | None | TPU: slow, no retraction tuning needed at this size |
| `ankle_block` | +z up (roll pocket wall on the bed) | 5.4 | 7.9 | 50.9 | Inside the pitch pocket and under the window bridges | Two pockets at right angles: one always prints sideways. The manual's "−y face down" stands the part on the 1.3 mm idler boss (0.1 cm² of contact, 10.3 cm² overhang): only with a raft |
| `shin` | −y up (horn plates on the bed) | 9.9 | 7.0 | 44.8 | Under the idler plates | All four plate holes print round. −x up needs less support (4.6 cm²) but prints the bearing seats as horizontal holes: not recommended. The manual's idler-side-down has 3.8 cm² contact and 13.1 cm² overhang |
| `thigh` | +y up (idler plate on the bed), as the manual | 8.2 | 13.2 | 44.8 | Under the horn arm and inside the knee pocket | Round holes. Least support is +x up (spine on the bed: 7.8 cm², 25.8 mm tall) at the cost of horizontal bearing and horn holes. See the note on layer direction below |
| `hip_block` | −z up (roll pocket wall on the bed) | 5.4 | 7.9 | 50.9 | As the ankle block | Coupon part. Same remark about the idler boss |
| `yaw_bracket` | −z up (top plate on the bed) | 11.5 | 5.2 | 33.9 | Under the roll horn recess | The thrust face prints against the bed: smooth |
| `thrust_ring` | −z up | 9.0 | 0 | 4.3 | None | Thrust face on the bed |
| `hip_mount` | −z up (flange on the bed, pocket opening up) | 10.6 | 0.3 | 26.6 | None | |

### Frame

| Part | Print with | Bed cm² | Overhang cm² | Height mm | Supports |
| --- | --- | ---: | ---: | ---: | --- |
| `pelvis_plate` | +z up | 91.8 | 0.5 | 12.5 | None (shell insert holes are blind from below: bridge 3.2 mm) |
| `deck_1`, `deck_2` | flat | 94 | 0 | 3.0 | None |
| `sled_*` (6) | +z up (bosses up) | 4 to 37 | 0 | 6 to 9 | None |
| `shoulder_mount` | +y up (pocket opening up) | 6.3 | 0.6 | 20.0 | None |
| `shoulder_link` | +x up | 4.9 | 3.1 | 29.0 | In the roll pocket |
| `upper_arm` | −y up | 4.9 | 6.8 | 28.2 | In the elbow pocket |
| `forearm` | +z up, or flat on its horn disc for strength | 0.6 | 2.2 | 82.0 | Standing: brim, 82 mm tall on 0.6 cm². Lying on the disc is safer and is what the manual recommends |
| `neck_mount` | +z up (flange on the bed) | 11.4 | 0.5 | 22.7 | None |
| `neck_link` | −z up (yaw horn disc on the bed) | 5.1 | 7.1 | 23.2 | In the head-pitch pocket |
| `head_frame` | −z up (head deck on the bed) | 25.4 | 2.0 | 35.0 | Under the two discs |
| `cam_mount` | +z up (base on the bed) | 6.0 | 1.5 | 32.8 | Under the lens opening |

### Shell

| Part | Print with | Overhang cm² | Height mm | Supports |
| --- | --- | ---: | ---: | --- |
| `chest_shell` | +z up (standing on its bottom rim) | 12.4 | 95.5 | Tree supports under the top ring and tabs; brim (2.4 cm² contact) |
| `back_shell` | +x up (back face on the bed, open side up) | 21.7 | 50.0 | Under the rings and tabs; vents bridge 2 mm |
| `head_front` | −x up (face down) or standing | 19.1 | 62.2 | Inside the dome; the guide tubes print vertical when standing |
| `head_back` | −z up (crown on the bed) | 7.2 | 70.7 | Under the crown curve |
| `visor` | −z up | 1.1 | 60.0 | None; print slowly for clarity |

## Features that need attention

| Feature | Where | Issue | What to do |
| --- | --- | --- | --- |
| Cable window, 8.3 × 5.0 mm through a 2 mm wall | Every pocket | Bridges 8.3 mm when the pocket opens upward | Bridges cleanly at 0.2 mm layers; trim any sag so the plug passes flat |
| Horn recess Ø20.4 × 3.8 deep with a 1.8 mm rim | Every horn plate | Faces down when the plate is on the bed | Support, or print the plate face up and accept a rougher cap face |
| Horn pilot holes Ø1.6, blind, 0.4 mm skin left | Every horn plate | Two layers of skin; ST2×4 reaches 0.1 mm past the bottom | Coupon T3; if the screw dimples the face, shorten it by a turn or drill the pilots through |
| Bearing seat Ø6.05 × 2.5, 0.5 mm lip | Idler plates | Press fit depends on first-layer squish | Ream to a light press; the bearing must spin after pressing |
| M3 idler insert in a 0.1 mm sleeve | Pocket back walls | The sleeve is not structural | Press the insert against a 1.3 mm stop so its face stands 1.3 mm proud |
| Ear insert holes, 0.7 mm from the pocket | Every pocket | Heat can bulge the wall into the pocket | Iron at 200 °C, 3 s, with a servo-sized block in the pocket; test on the coupon |
| Tie slots 3.0 × 1.6 | Thigh, shin, yaw bracket | Small bridges | Clear with a 1.5 mm drill |
| Thin tall walls | Foot uprights, forearm | Wobble | Brim, 40 mm/s outer wall |

## Fits

Nominal CAD values from `hardware/mechanical/hardware.py`. None is verified; "coupon" names the test that confirms it.

| Interface | CAD value | Fit intended | Verify on |
| --- | --- | --- | --- |
| Servo case in pocket | +0.2 mm per side | Slides in by hand, no rattle | T2 |
| Servo plug through the window | 8.3 × 5.0 for a 7.7 × 2.6 plug | Passes flat | T2 |
| M2 heat-set insert (OD 3.2, 3 long) | Ø3.2 hole, 4.0 deep | Melt in flush | T2 |
| M3 heat-set insert (OD 4.2, 3 long) | Ø4.0 hole | Melt in, face 1.3 mm proud | T3 |
| M2 clearance | Ø2.3; counterbore Ø4.4 × 2.2 | Free | T2 |
| M3 clearance (standoffs) | Ø3.3 | Free | Frame |
| ST2×4 pilot | Ø1.6 × 2.4 blind | Thread-forming | T3 |
| MR63ZZ seat | Ø6.05 × 2.5 | Light press | T3 |
| Horn disc in recess | Ø20.4 for a Ø20.0 horn | Drops in | T3 |
| Navy caps | 0.05 mm radial | Push fit | T3 |
| Holder to driven plate | 0.8 mm axial | Never touches | T3 |
| Yaw bracket to thrust ring | 0.2 mm axial | Turns freely, greased | T4 |
| Moving parts, everywhere else | ≥ 1.0 mm | Clear | T5 |
| Shell halves | 0.4 mm seam | Slide together | Shell |

If a coupon fit fails, change the value in `hardware.py` (`CLEAR_SERVO`, `HOLES[...]`), run `gen_docs.py`, and
reprint the coupon. Do not file parts to fit: the next print will have the same error.

## Post-processing

1. Remove supports from pockets with a flat screwdriver; check that a servo drops to the bottom.
2. Clear every hole with the matching drill by hand (1.6, 2.3, 3.2, 3.3, 4.0): this removes strings, it does not
   resize.
3. Press inserts as in the assembly guide, before any part is assembled.
4. Break the edges of every cable window and tie slot with a knife: leads rub there.
5. Weigh each part and write it down. The mass model is an estimate (`mechanical_verification.md` §5); the first
   real numbers replace it.

## Risk to watch: layer direction in the driven links

The thigh and shin lie on their side so that the plate holes print round. Their spine then builds up across the
layers, and a load that spreads the two plates (a sideways knock on the foot) pulls those layers apart. Test T4
squeezes and spreads the first printed thigh by hand before it goes on the robot; if it cracks at the spine,
print the driven links in PETG, which bonds better between layers.
