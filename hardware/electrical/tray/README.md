# Module tray (step 1): what goes in the robot

The electronics stay off-the-shelf modules on a printed tray (see `../mainboard/README.md` for the decision).
[`modules.csv`](modules.csv) is the single list: every electrical item, its quantity, where it sits, its envelope,
mass, mounting, price and where the numbers come from. Dimensions marked MEASURE or ASSUMED come from listings
and must be measured on the part bought before a sled is printed for it.

Prices are the BOM (`design/sketch_shell_a/BOM.dc.html`, v0.7) where the item is there; new items are estimates.

## Where things sit

| Placement | Footprint (L × W, summed) | Mass |
| --- | ---: | ---: |
| Torso tray | ≈ 14 600 mm² (M06 rides on M02, not counted twice) | ≈ 164 g |
| Pelvis (battery, LiPo alarm) | 2 680 mm² | 63 g |
| Head (camera, ToF, display, microphones) | 2 660 mm² | 12 g |
| Feet, servo cases | FSR, NTC | 2.6 g |

One torso deck in the v4 CAD is 89 × 108 mm = 9 612 mm². The tray needs **two tiers** (19 224 mm²), filled to
≈ 76 %: tight, so step 3 places every module and the harness in the CAD rather than estimating.

## Changes to the BOM

| Change | Why | Cost |
| --- | --- | ---: |
| INA226 module 2 → 4 | the arms + neck branch is measured too; one on the battery lead feeds the software fuel gauge (battery %) | +130 000 |
| Inline mini blade fuses: 15 A battery lead, 5 A per leg branch, 4 A arms + neck, with spares | a shorted servo cable must not burn (P-8) | +64 000 |
| Main switch or disconnect, ≥ 15 A DC | power off without pulling a plug | +30 000 |
| LiPo low-voltage alarm on the balance lead | over-discharge warning that does not depend on the firmware | +25 000 |
| VBAT divider to PA3, 1 kΩ on every PWM line | firmware battery cut-off; one-pin-off plug protection | +6 000 |
| XT30 pairs, silicone wire by gauge, JST leads (replaces the 60 000 lump) | sized wiring instead of a guess | +20 000 |
| Wiper buffers MCP6002 on 3.3 V instead of LMV358 on 5 V | the buffer output can never exceed the ADC input range | ≈ 0 |
| **Total** | | **≈ +275 000 VND** |

## The BOM follows the CAD

The CAD is the reference. BOM v0.8 now matches it: 2S ≈ 1000 mAh pack that fits 56 × 30 × 19 mm, Waveshare 1.9″
LCD face, Radxa Zero 3W with eMMC. The two INMP441 microphones are in the BOM but not in the CAD yet.
