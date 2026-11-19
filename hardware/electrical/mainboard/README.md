# BZN-MB custom mainboard: ON HOLD

Decision 2026-11-20 (H. Nguyen Ngoc): no custom PCB for now. Every board in the robot stays an off-the-shelf
module, mounted on a printed module tray (one or two tiers) in the torso. Reason: the owner cannot review a power
PCB himself, and a board he cannot verify is a safety risk he does not want to take.

What here is still used by the module tray:

- `requirements.md`: the functions, pins, addresses and channels (firmware unchanged) and the safety requirement
  P-8 (no single fault may overheat, burn or vent anything).
- `architecture.md` and the BZN-MB-BD-001 sheets: the power split (servo rail vs logic), the battery
  under-voltage gap, the protection layers.
- `design_calc.py` / `design_calc.md`: branch currents, fuse coordination, rail droop and copper numbers,
  recalculated for wires and inline fuses in the tray work.

`bom_key_parts.csv` lists the chips a custom board would have used; it is kept only as the record of that study.
