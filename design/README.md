# Bazan — Industrial Design & Concept

Concept art only. The engineering robot (geometry, masses, joint limits) is the printable CAD in
`hardware/mechanical/`, and everything that simulates or drives the robot is generated from it
(`bazan_cad.urdf`, see `software/ai/sim/mujoco/README.md`).

## Directory Structure

- **`blender/`**: `bazan.blend` and `renders/` of Design C (Mico look: white/navy/orange, oval LED eyes). This
  concept was modelled on the retired 303 mm skeleton; its generator scripts and mass report were removed with that
  skeleton, so treat the file and the renders as a visual reference for the shell, not as dimensions.
- **`sketch_shell_a/`**: 2D design canvases, shell variations (Mico, Pico, Luno, Kiko) and drawings, also drawn on
  the retired 303 mm skeleton.
