# Synapse Humanoid — Software Domain

This directory contains the software stack for the Synapse Humanoid platform, organized across four tiers:

## Directory Structure

- **`firmware/`**: Microcontroller firmware (STM32F411, FreeRTOS).
  - Handles hard real-time safety supervision, servo pulse generation, sensor ADC reading (pots, NTC, FSR), and UART/USB communications.
- **`core/`**: Hardware-independent real-time C++ control libraries (`lib/core`).
  - Contains kinematics, trajectory generation, supervisor state machine, ring buffers, seqlocks, and protocol definitions.
- **`ros2/`**: High-level ROS 2 Jazzy workspace.
  - `src/humanoid_control`: PPO RL inference, MuJoCo bridge, playback, and visualizer nodes.
  - `src/humanoid_description`: URDF, Xacro, visual meshes, RViz configurations.
  - `src/humanoid_perception`: Vision, camera pipelines, sensor processing.
  - `src/humanoid_infrastructure`: System diagnostics, communications, parameters.
  - `src/humanoid_interfaces`: Custom ROS 2 msg/srv/action definitions.
- **`ai/`**: Reinforcement Learning (RL) and Physics Simulation.
  - `sim/`: MuJoCo simulation environment, kinematic tree, and URDF generation.
  - `models/`: Cataloged neural policy checkpoints (`.onnx`, `.pt`), model registry, and evolution tracker.
