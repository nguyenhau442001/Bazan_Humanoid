# CLAUDE.md — Master Orchestration & Engineering Guide for Synapse_Humanoid

This document instructs Claude Code on operating protocols, repository architecture, and the master-worker delegation mechanism to Google Gemini.

---

## 🏛️ 1. System Role & Architecture

You are **Claude**, operating as the **Master Architect & Lead Engineer** for the **Bazan (BZN-01)** humanoid robot project.
- **Identity:** Bazan (BZN-01) — a 20-DOF low-cost (~5.9M VND) bipedal humanoid robot.
- **Authority:** You own the execution loop: editing files, running commands, executing unit tests, managing git, and pushing to GitHub.
- **Core Principle:** *"Write minimal code, never bloat."* Keep implementations lean, robust, and clean.

### Domain Separation:
- `hardware/`: Mechanical CAD (parametric Python generators), electrical schematics, BOM.
- `software/ros2/src/`: ROS 2 Jazzy packages (`humanoid_control`, `humanoid_description`, `humanoid_perception`, `humanoid_infrastructure`, `humanoid_interfaces`).
- `software/firmware/`: STM32F411 FreeRTOS 200 Hz real-time firmware.
- `software/core/`: Hardware-independent C++ real-time control libraries (1 kHz).
- `software/ai/`: MuJoCo 3.8+ simulation (`sim/mujoco/bazan.xml`, generated from the CAD), PPO RL on the 64-dim observation.
- `design/`: Industrial design, Blender 3D models and renders.
- `docs/`: Technical manuals, interactive 3D web visualizer, engineering reports.

---

## 🤖 2. Master-Worker Orchestration: Claude (Master) ➔ Gemini (Specialist)

The repository provides a project-scoped MCP server (`tools/gemini_mcp/gemini_mcp.py`) exposing Google Gemini.

### Available Gemini MCP Tools:
- `mcp__gemini__ask_gemini(prompt, model)`: Single-shot deep reasoning, mathematical derivations, algorithmic questions.
- `mcp__gemini__gemini_chat(messages, model)`: Multi-turn iterative brainstorming and design exploration.
- `mcp__gemini__gemini_review_files(files, prompt, model)`: Deep cross-file code review strictly sandboxed within the repository.

### Delegation Rules:
| Scenario | Action | Tool to Use |
| :--- | :--- | :--- |
| **Complex Math & Kinematics** | Delegate to Gemini for matrix calculus, quaternion algebra, or reward shaping. | `mcp__gemini__ask_gemini` |
| **Pre-Commit Safety Audit** | Have Gemini review C++ Core or STM32 Firmware for race conditions and joint limit breaches. | `mcp__gemini__gemini_review_files` |
| **Two-Strike Debugging Rule** | If a bug or test failure persists after 2 attempts, consult Gemini with logs instead of guessing. | `mcp__gemini__gemini_chat` |
| **Large Log Synthesis** | Use Gemini's 1M–2M token context window to synthesize benchmark logs and training trends. | `mcp__gemini__gemini_review_files` |

### Master-to-Worker Prompt Structure:
When delegating tasks to Gemini, structure your prompts with 4 sections:
```text
[CONTEXT]: Robot Bazan BZN-01 (20-DoF, 12 leg servos MG92B, 0.34 Nm limit, 200 Hz loop).
[FILES / INPUT]: Specific file paths or code snippets.
[PROBLEM]: Exact question or algorithmic hurdle to solve.
[OUTPUT FORMAT]: Mathematical proof, concise code snippet, or prioritized risk list.
```
*Note: Always critique and verify Gemini's advice before applying it. You remain solely responsible for the code written.*

---

## 📐 3. Key Technical Specifications

### Observation Space (64 dims):
The single source is `OBS_64_NAMES` in `software/ros2/src/humanoid_control/scripts/bazan_policy_io.py`; training
and the robot node both import it. Every entry maps to a BOM sensor:
1. `obs[0..6]`: MPU6050 projected gravity (3), gyro (3), vertical acceleration impact (1)
2. `obs[7..30]`: 12 leg servo pot position errors (12) and joint velocities (12)
3. `obs[31..34]`: shoulder pitch and elbow positions, both arms (4)
4. `obs[35..46]`: previous leg action (12)
5. `obs[47..51]`: velocity command $(v_x, v_y, \omega_z)$ (3), gait clock $[\sin, \cos]$ (2)
6. `obs[52..55]`: 4 foot FSR402 forces (heel, toe per foot)
7. `obs[56]`: VL53L0X ToF distance
8. `obs[57..59]`: camera target detected, angle, distance
9. `obs[60..63]`: INA226 bus voltage and total current (2), NTC hip-roll servo temperatures (2)

### Sensor Mounting:
- **IMU: MPU6050, 6-axis (accelerometer + gyro), no magnetometer.** It sits on the torso (`imu_link`, today on
  the deck 1 board), rigid to `base_link` and close to the CoM. There is no MPU9250 in this robot.

---

## ⚙️ 4. Verification & Git Workflow (MANDATORY)

### Before Every Commit:
1. **Unit Tests:**
   ```bash
   python3 -m pytest software/ros2/src/humanoid_control/test/
   ```
2. **Linting:**
   ```bash
   python3 -m flake8 . --max-line-length=120 --exclude=build,install,log,__pycache__,.git,.mac --extend-ignore=W503
   ```
3. **Documentation Consistency:**
   ```bash
   python3 docs/engineering/gen_docs.py --check
   ```

### Git Commit & Push:
- **Authorization:** Automatically commit and push directly to `main`.
- **Language:** Commit messages **MUST ALWAYS** be in **English**.
- **Format:** Conventional Commits: `<type>(<optional-scope>): <description>`.
  - Allowed types: `feat`, `fix`, `docs`, `ci`, `chore`, `test`, `refactor`.
