# Mini humanoid sketches (v0.5)

**Main shell: A2 · Mico v2.0 — cute outside, capable inside.** A1's face in Pico's suit, every part a soft
capsule or dome. Perception and expression are separate: the IMX219 camera and a VL53L0X ToF sit behind
their own forehead window, while one 2.42″ OLED behind the visor draws the eyes and mouth. Ears are I2S MEMS
mic grilles, each foot has heel and toe FSRs, the neck sits in an orange collar ring, Vietnam flag on the chest.

![Mico, main shell](images/a2_mico.png)

![Mico side, rear and feet](images/mico_views.png)

![Mico expressions](images/mico_faces.png)

## Mechanical skeleton v2.1

Drawn from the robot model (gen_robot.py), J01–J20 with every axis, with the Mico shell overlaid to check the fit.
Open items from the fit check: arms sit inside the torso shell (shoulder-roll offset 5 → 13 mm proposed),
elbow and hip/ankle-roll housings need room, and the 2.42″ OLED plus the camera board do not fit the head.

![Mechanical skeleton](images/skeleton.png)

![Skeleton details](images/skeleton_details.png)

![TF frames](images/frames.png)

## Hero family

| A1 · Hero | A3 · Pico | A4 · Luno | A5 · Kiko |
| --- | --- | --- | --- |
| ![A1](images/main.png) | ![Pico](images/a3_pico.png) | ![Luno](images/a4_luno.png) | ![Kiko](images/a5_kiko.png) |

All five share the same 20-DoF skeleton, joint axes and 303 mm height; only the shell changes.

## Reference boards

Claude Design canvas source for the 20-DoF mini humanoid. Shell A (cute) is the hero,
shell B (armor) is the villain; both share the same skeleton and joint axes. All geometry is drawn in mm
from the same dimensions as `sim/mini_humanoid/gen_robot.py` (303 mm tall, 481 g at the time; the current
mass is in [`docs/engineering/mass_budget.md`](../../engineering/mass_budget.md)).

| Front | Side |
| --- | --- |
| ![Front view](images/main.png) | ![Side view](images/side.png) |

### Villain · shell B

![Shell B villain](images/shell_b.png)

Pauldrons widen it to about 132 mm (A: 119) and must ride on the upper arm so shoulder roll stays free.

![20-joint servo map](images/joints.png)

![BOM](images/bom.png)

![GA drawing](images/drawing.png)

| File | Artboard |
| --- | --- |
| `A2_Mico.dc.html` | **Main shell** · Mico v2.0 |
| `Mico_Views.dc.html` | Mico side, rear and foot underside (FSR pads) |
| `Mico_Faces.dc.html` | Mico expression set on the OLED |
| `Skeleton.dc.html` | v2.1 mechanical skeleton J01–J20, front / side / rear, fit check |
| `Skeleton_Details.dc.html` | J01–J20 table, head packaging and sensor view, foot, pelvis, arm |
| `Frames.dc.html` | TF frames (REP-103 / REP-120), sensor frames, TF tree from `humanoid.urdf` |
| `Main.dc.html` | A1 · Hero front view, with part sizes and joint heights |
| `A3_Pico.dc.html`, `A4_Luno.dc.html`, `A5_Kiko.dc.html` | Hero family variants |
| `ShellB.dc.html` | Shell B (villain) armor, front and side |
| `Side.dc.html` | Side view with depths, internal layout and CoM (146 mm) |
| `Joints.dc.html` | 20-joint map, colored by servo (DS843MG / MG90S / SG90) |
| `BOM.dc.html` | Bill of materials v0.7, cheapest HCMC prices (Nov 2026), about 5.65M VND |
| `Drawing.dc.html` | GA drawing, first-angle projection (front, side, top), A1 at 1:1 |
| `canvas.json` | Canvas layout |
| `images/*.png` | Rendered exports of the three artboards |

The `.dc.html` files render inside Claude Design (they load its `support.js`); the SVG
inside each file is plain and can be copied out on its own.
