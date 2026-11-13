# AGENTS.md — Agent Operating Rules & Conventions

This document defines the mandatory operating protocols, coding standards, and Git workflows for AI agents working in the `Synapse_Humanoid` repository.

---

## 📌 1. Git Commit & Push Workflow (MANDATORY)

Whenever an agent completes a task, milestone, or bug fix:

1. **Auto-Push Authorization:**
   - The agent is **explicitly permitted and required** to commit changes and push directly to the default working branch (`jazzy` or `main`).

2. **Commit Message Language:**
   - Commit messages **MUST ALWAYS** be written in **English**.

3. **Commit Message Format:**
   - Commit messages **MUST** strictly adhere to the Conventional Commits specification:
     ```text
     <type>(<optional-scope>): <description>
     ```
     or
     ```text
     <scope>: <description>
     ```
   - **Allowed Types / Scopes:**
     - `feat`: A new feature, ROS 2 node, message definition, or capability.
     - `fix`: A bug fix, compilation error resolution, or runtime patch.
     - `refactor`: Code restructuring without changing external behavior.
     - `docs`: Documentation updates (README, markdown guides, architecture specs).
     - `ci`: CI/CD workflow updates (GitHub Actions, build scripts).
     - `chore`: Maintenance tasks, configuration changes, or dependency cleanups.
     - `test`: Adding or updating unit tests and integration tests.
   - **Examples:**
     - `fix: resolve missing sensor_msgs image_encodings header`
     - `feat(control): implement realtime joint limit safety interface`
     - `docs: update OrbStack setup instructions in README`
     - `refactor(perception): migrate inference pipeline to ONNX Runtime`
     - `ci: enforce flake8 and package.xml validation on jazzy branch`

---

## 🏗️ 2. Workspace & Coding Standards

1. **Workspace Layout:**
   - Modular Domain Separation:
     - `hardware/`: Mechanical (CAD, 3D printing), electrical, wiring.
     - `software/`:
       - `software/ros2/src/`: All ROS 2 packages (`humanoid_description`, `humanoid_control`, `humanoid_perception`, `humanoid_infrastructure`, `humanoid_interfaces`).
       - `software/firmware/`: MCU firmware (STM32, FreeRTOS).
       - `software/core/`: Real-time hardware-independent C++ controller libraries.
       - `software/ai/`: Reinforcement learning, simulation (MuJoCo), model policies & checkpoints.
     - `design/`: Industrial design, Blender models, renders, aesthetic concepts.
     - `docs/`: Technical manuals, engineering reports, architecture specifications.
   - For backward compatibility and colcon build tooling, root symlinks (`src/`, `firmware/`, `mechanical/`, `models/`, `sim/`, `lib/core`) are maintained.

2. **C++ Guidelines:**
   - Target C++17 / C++20 standards.
   - Use smart pointers (`std::shared_ptr`, `std::unique_ptr`) and avoid raw pointer ownership.
   - Real-time safety: Decouple high-frequency control loops (1 kHz) from perception/vision loops using lock-free queues (`SPSCQueue`) or shared memory.

3. **Python & Launch File Guidelines:**
   - Strictly follow PEP 8 standards.
   - Maintain 2 blank lines before all top-level functions and class definitions (`E302`).
   - Remove unused imports (`F401`).
   - Maximum line length: 120 characters.

4. **Verification Before Push:**
   - Run `python3 -m flake8 . --max-line-length=120 --exclude=build,install,log,__pycache__,.git` to ensure CI linting passes.
   - Ensure `package.xml` dependencies match `CMakeLists.txt` targets.

---

## 🤖 3. Delegating to Gemini

The repository provides a project-scoped Model Context Protocol (MCP) server for Google Gemini (`tools/gemini_mcp/gemini_mcp.py`), configured in `.mcp.json`.

1. **Available MCP Tools:**
   - `mcp__gemini__ask_gemini(prompt, model)`: Direct single-prompt queries for architectural advice, mathematical derivations, or algorithmic second opinions.
   - `mcp__gemini__gemini_chat(messages, model)`: Multi-turn reasoning for iterative problem solving and design brainstorming.
   - `mcp__gemini__gemini_review_files(files, prompt, model)`: In-depth code reviews, safety audits, and cross-file verification strictly sandboxed within the repository.

2. **Security & Governance:**
   - **No Secret Leaks:** `tools/gemini_mcp/.env` is strictly git-ignored. Never stage or commit API keys.
   - **Repository Sandbox:** `gemini_review_files` strictly forbids path traversal outside the repository root.
   - **Default Model:** `gemini-2.5-pro` (configurable via `GEMINI_MODEL` in `.env`).

