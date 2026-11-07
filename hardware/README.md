# Synapse Humanoid — Hardware Domain

This directory contains the physical embodiment, mechanical CAD, electrical schematics, and manufacturing files for the Synapse Humanoid platform.

## Directory Structure

- **`mechanical/`**: Parametric Python/CadQuery models, joint geometry, link generation, and printable STL/STEP files.
  - `check.py`: Automatic geometric validation for clearances and tool access.
  - `gen_docs.py`: Print file generator and assembly manual synchronization.
  - `gen_manual.py`: Whole-robot assembly manual and visual guide generation.
- **`electrical/`**: Schematics, pinouts, power distribution, and BOM.
- **`assembly/`**: Physical fabrication instructions, screw torque specifications, and wiring routing.
