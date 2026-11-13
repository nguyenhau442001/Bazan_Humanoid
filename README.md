# Synapse Humanoid · Bazan (BZN-01)

<div align="center">

<p align="center">
  <img src="docs/images/social_preview.png" alt="Synapse Humanoid · Bazan Social Preview" width="100%">
</p>

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=for-the-badge&logo=ros)](https://docs.ros.org/en/jazzy/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.8%2B-orange?style=for-the-badge&logo=google)](https://mujoco.org)
[![RViz2](https://img.shields.io/badge/RViz2-native_macOS-purple?style=for-the-badge)](docs/getting_started.md)
[![PyTorch](https://img.shields.io/badge/PyTorch-MPS_Metal-EE4C2C?style=for-the-badge&logo=pytorch)](docs/engineering/rl_benchmark.md)
[![Generations](https://img.shields.io/badge/Evolved_Generations-218_Gen-success?style=for-the-badge&logo=dna)](docs/engineering/rl_benchmark.md)
[![Training Time](https://img.shields.io/badge/Cumulative_Training-21.5_Hours-blueviolet?style=for-the-badge&logo=clockify)](docs/engineering/rl_benchmark.md)
[![License](https://img.shields.io/badge/License-Apache_2.0-lightgrey?style=for-the-badge)](LICENSE)

<br/>

<table>
  <tr>
    <th width="33%" align="center">🤖 3D Digital Twin</th>
    <th width="33%" align="center">🚶‍♂️ Dynamic Locomotion</th>
    <th width="33%" align="center">👋 Interactive Gestures</th>
  </tr>
  <tr>
    <td align="center" valign="middle">
      <img src="docs/images/mico.png" width="220" alt="Bazan humanoid robot in MuJoCo">
    </td>
    <td align="center" valign="middle">
      <img src="docs/images/dynamic_walk.gif" width="260" alt="Bazan dynamic upright walking in MuJoCo">
    </td>
    <td align="center" valign="middle">
      <img src="docs/images/mico_wave.gif" width="260" alt="Bazan waving in MuJoCo">
    </td>
  </tr>
</table>

<p>
  🎬 <b><a href="docs/images/mico_demo.mp4">▶️ Watch Full MuJoCo Simulation Demo (Walk · Turn · Wave · 60fps MP4)</a></b>
</p>

</div>

**Bazan (BZN-01)** is a 303 mm, 20-DoF bipedal humanoid built from parts sold in Ho Chi Minh City, simulated in MuJoCo with ROS 2 and viewed in RViz2. *Cute outside, capable inside. Made in Vietnam 🇻🇳.*

<!-- gen:readme_spec -->
| Height | Mass | DoF | Compute | Sensors | BOM |
| --- | --- | --- | --- | --- | --- |
| 303 mm | 677 g (603 g in LW-PLA) | 20 (legs 2×6, arms 2×3, neck 2) | Radxa Zero 3W (Cortex-A55 + NPU) · STM32F411 (Cortex-M4) | IMU, camera, ToF, 2 mics, 4 foot FSRs | ≈ 5.88M VND |
<!-- /gen:readme_spec -->

> **Documentation Subpages:** [Getting Started & Sim Guide](docs/getting_started.md) · [Upgrade Roadmap](docs/engineering/upgrade_roadmap.md) · [RL Benchmark Suite](docs/engineering/rl_benchmark.md) · [Theory & References](docs/references.md) · [Hardware Architecture](docs/engineering/hardware_architecture.md) · [Mass Budget](docs/engineering/mass_budget.md) · [Torque Validation](docs/engineering/torque_validation.md) · [Firmware Docs](firmware/stm32f411/README.md) · [Web Dashboard](https://nguyenhau442001.github.io/Synapse_Humanoid/)

---

### 🌋 Why Bazan?

**Bazan** (model code **BZN-01**) draws its name from the fertile, ancient red volcanic basalt soil (*đất đỏ bazan*) characteristic of the Kon Tum and Central Highlands (*Tây Nguyên*) plateau of Vietnam.

Just as the basalt earth anchors the ancient forests with grounded firmness and endures harsh tropical monsoons, **Bazan** embodies:
- **Grounded Stability & Low Center-of-Mass:** High-inertia foot placement and continuous Zero Moment Point (ZMP) control to maintain absolute footing across uneven terrain.
- **Resilience Against Violent Disturbances:** Autonomous recovery under sudden lateral kick impulses (up to $0.08\text{ Ns}$), low-friction ice shocks, and tilt drops.
- **Homegrown Vietnamese Engineering:** Conceived, designed, printed, and programmed in Vietnam—combining accessible, off-the-shelf local hardware with state-of-the-art robot learning algorithms.

---

## 🏛️ System Architecture

Core principle: **The application domain plans, the real-time domain acts.**

### 🌐 1. High-Level Layered Architecture

The platform architecture is structured into **5 decoupled vertical tiers**, strictly separating high-level AI cognition and robotic middleware from hard real-time execution and physical mechatronics:

<p align="center">
  <img src="docs/images/platform_architecture_5layers.svg" alt="Synapse Platform Architecture" width="100%"/>
</p>

| Layer | Tier Name | Core Responsibilities & Technologies | Execution Domain |
| :--- | :--- | :--- | :---: |
| **Layer 5** | **Applications & Cognition** | Goal-Conditioned RL Policy (Titan / ResNet), Behavior & Gait FSM, Vision & 3D Perception | Application (50 Hz) |
| **Layer 4** | **Robotics Framework** | ROS 2 Jazzy Core, `robot_state_publisher`, TF2 Dynamic Transforms, Telemetry & RViz2 | Host Middleware |
| **Layer 3** | **Native Core & Digital Twin** | MuJoCo 3.8+ Physics Engine, ONNX Runtime C++, Lock-Free SPSC Queues, Shared Memory IPC | Native Compute (1 kHz) |
| **Layer 2** | **Hardware Abstraction Layer (HAL)** | STM32F411 Firmware, FreeRTOS Deterministic Scheduler, S-Curve Trajectories, Safety Interlocks | Real-Time MCU (200 Hz) |
| **Layer 1** | **Hardware & Mechatronics** | 20-DoF Joint Actuators (MG92B/SG90), MPU6050 IMU, 4× Foot FSRs, 2S LiPo & Dual Power Grid | Physical Foundation |

> Detailed hardware interconnects, firmware state machines, and real-time control pipelines for each layer are described in the sections below.

---

### ⚡ 2. Hardware Interconnect & Power Grid

Subsystems are visually color-coded: Host Compute (Blue), Real-Time MCU (Amber), PWM Actuator Drivers (Coral), and Telemetry Sensors (Purple).

```mermaid
flowchart TB
  subgraph APPHW["🖥️ Compute Domain · Radxa Zero 3W · RK3566"]
    RADXA["<b>Linux · ROS 2 Jazzy</b><br/>vision, planning, RL inference"]
    CAMH["📷 RGB-D Camera"] --> RADXA
    TOF["📡 ToF Distance Sensor"] --> RADXA
    MIC["🎤 Microphones × 2"] --> RADXA
    RADXA --> OLED["📺 OLED Face Display"]
  end

  RADXA <-->|"USART1 · PA9 / PA10 · 921600 8N1 · 3.3 V"| STM

  subgraph RTHW["⚙️ Real-Time Domain · STM32F411CEU6 Black Pill"]
    STM["<b>FreeRTOS Firmware</b><br/>200 Hz Control · Telemetry · Safety"]
  end

  STM -->|"I²C1 0x40 · OE PB0"| PCA1["🎛️ PCA9685 #1"]
  STM -->|"I²C1 0x41 · OE PB1"| PCA2["🎛️ PCA9685 #2"]
  PCA1 -->|"PWM 50 Hz"| LEGS["🦾 <b>12 Leg Servos</b><br/>MG92B × 10 · MG90S × 2"]
  PCA2 -->|"PWM 50 Hz"| ARMS["🦾 <b>8 Arm + Neck Servos</b><br/>MG90S × 2 · SG90 × 6"]
  STM ---|"I²C2 0x44 / 0x45"| INA["⚡ INA226 × 2<br/>leg branch V / I sensing"]
  STM ---|"I²C2 0x68"| IMU["🧭 MPU6050 IMU<br/>pelvis top plate"]
  STM ---|"ADC PA0 · PB12–PB15"| MUX["🔀 CD74HC4067 Mux"]
  MUX --- POTS["📏 12 Leg Pot Wipers"]
  MUX --- FSR["👟 4 Foot FSRs"]
  STM ---|"ADC PA1 / PA2"| NTC["🌡️ NTC × 2<br/>hip-roll temperature"]
  LEGS -.- POTS

  %% Hardware Styling Palette
  style APPHW fill:#0c2d48,stroke:#2e8bc0,stroke-width:2px,color:#d1e8e2
  style RADXA fill:#145da0,stroke:#2e8bc0,stroke-width:1.5px,color:#ffffff
  style CAMH fill:#145da0,stroke:#2e8bc0,stroke-width:1px,color:#ffffff
  style TOF fill:#145da0,stroke:#2e8bc0,stroke-width:1px,color:#ffffff
  style MIC fill:#145da0,stroke:#2e8bc0,stroke-width:1px,color:#ffffff
  style OLED fill:#145da0,stroke:#2e8bc0,stroke-width:1px,color:#ffffff

  style RTHW fill:#4a2810,stroke:#e06d53,stroke-width:2px,color:#fdebd0
  style STM fill:#873e23,stroke:#e06d53,stroke-width:1.5px,color:#ffffff

  style PCA1 fill:#782939,stroke:#c94c4c,stroke-width:1.5px,color:#ffffff
  style PCA2 fill:#782939,stroke:#c94c4c,stroke-width:1.5px,color:#ffffff
  style LEGS fill:#4a1525,stroke:#9e2a2b,stroke-width:1.5px,color:#ffffff
  style ARMS fill:#4a1525,stroke:#9e2a2b,stroke-width:1.5px,color:#ffffff

  style INA fill:#3b0764,stroke:#a855f7,stroke-width:1px,color:#ffffff
  style IMU fill:#3b0764,stroke:#a855f7,stroke-width:1px,color:#ffffff
  style MUX fill:#3b0764,stroke:#a855f7,stroke-width:1px,color:#ffffff
  style POTS fill:#2e1065,stroke:#a855f7,stroke-width:1px,color:#ffffff
  style FSR fill:#2e1065,stroke:#a855f7,stroke-width:1px,color:#ffffff
  style NTC fill:#2e1065,stroke:#a855f7,stroke-width:1px,color:#ffffff
```

#### 🔋 Regulated Power Distribution Tree

```mermaid
flowchart LR
  BAT["🔋 <b>2S LiPo Battery</b><br/>7.4 V · 1100 mAh 30C"] --> FUSE["⚡ Fuse 10A + Reverse Protect"]
  FUSE --> UBEC["🔥 <b>6 V 8 A UBEC</b><br/>High-Current Actuator Rail"]
  FUSE --> BUCK["⚡ <b>5 V 3 A Buck</b><br/>Compute Rail"]
  FUSE --> LDO["⚡ <b>3.3 V LDO</b><br/>Clean Logic Rail"]
  UBEC -->|"6V Servo Power"| SERVOS["🦾 <b>20 Servos Rail</b><br/>12 Legs + 8 Arms"]
  UBEC -.->|"Current Sense"| INA2["📊 INA226 Telemetry"]
  BUCK -->|"5V USB-C"| RADXA2["🖥️ Radxa Zero 3W"]
  LDO -->|"3.3V VCC"| LOGIC["🧠 STM32 & Sensor Array"]

  %% Power Tree Styling
  style BAT fill:#422006,stroke:#eab308,stroke-width:2px,color:#fef08a
  style FUSE fill:#422006,stroke:#eab308,stroke-width:1.5px,color:#ffffff
  style UBEC fill:#431407,stroke:#f97316,stroke-width:2px,color:#ffedd5
  style SERVOS fill:#7c2d12,stroke:#ea580c,stroke-width:1.5px,color:#ffffff
  style INA2 fill:#3b0764,stroke:#a855f7,stroke-width:1px,color:#ffffff
  style BUCK fill:#082f49,stroke:#0ea5e9,stroke-width:2px,color:#e0f2fe
  style RADXA2 fill:#0369a1,stroke:#38bdf8,stroke-width:1.5px,color:#ffffff
  style LDO fill:#022c22,stroke:#10b981,stroke-width:2px,color:#d1fae5
  style LOGIC fill:#047857,stroke:#34d399,stroke-width:1.5px,color:#ffffff
```
*Schematics and complete BOM details in [docs/engineering/hardware_architecture.md](docs/engineering/hardware_architecture.md).*

---

### 🔍 3. Dataflow & Firmware Pipeline (Deep-Dive)

<details>
<summary><b>▶️ Click here to inspect the wire-level ROS 2 topic graph and FreeRTOS firmware pipeline</b></summary>

<br/>

#### A. Application Software & ROS 2 Graph

```mermaid
flowchart TB
  subgraph APP["Application Domain · ROS 2 Jazzy (Mac / Ubuntu VM / Radxa Zero 3W)"]
    direction TB
    GAIT["<b>mico_gait.py</b><br/>demo gait: walk 8 steps, turn 60°, wave"]
    RL_NODE["<b>rl_locomotion_node</b><br/>ONNX Policy inference @ 50 Hz"]
    PLAY["<b>mico_playback.py</b><br/>kinematic playback, no physics"]
    RSP["<b>robot_state_publisher</b><br/>mico.urdf.xacro"]
    RVIZ["<b>RViz2</b><br/>rviz/mico.rviz · fixed frame odom"]
    subgraph PERC["humanoid_perception"]
      CAM["camera_subscriber_node<br/>RGB-D sync"]
      ONNX["onnx_inference_node<br/>ONNX Runtime"]
    end
    INFRA["<b>humanoid_infrastructure</b><br/>SPSC queue · thread pool · shared-memory IPC"]
    HWI["<b>ros2_control hardware interface</b><br/>planned, M1"]
  end

  subgraph SIM["Physics Backends · Interchangeable Topics"]
    MJ["<b>MuJoCo 3.8+</b><br/>mico_mujoco_bridge.py · 1 ms steps"]
  end

  subgraph MCU["Real-Time Domain · STM32F411 · FreeRTOS"]
    FW["<b>firmware/stm32f411</b><br/>supervisor · control 200 Hz · comm · diagnostics"]
    CORE["<b>lib/core</b><br/>modes · safety · trajectory · link framing"]
  end

  GAIT -->|"/joint_trajectory_controller/joint_trajectory"| MJ
  GAIT -.->|"same trajectories"| HWI
  RL_NODE -.->|"/joint_trajectory_controller/joint_trajectory"| MJ
  MJ -->|"/joint_states · /odom · /imu · /clock"| RSP
  PLAY -->|"/joint_states · odom → base_link"| RSP
  RSP -->|"/tf"| RVIZ
  CAM -->|"frames"| INFRA
  ONNX -.->|"/humanoid/perception/detections_3d"| GAIT
  HWI <-.->|"UART 921600 · COBS + CRC-16<br/>Traj ↓ · State 100 Hz ↑ · Diag 1 Hz ↑<br/>micro-ROS from M1"| FW
  FW --- CORE
```

#### B. FreeRTOS Firmware Pipeline (200 Hz Deterministic Loop)

```mermaid
flowchart LR
  RX["USART1 RX DMA"] --> COMM
  COMM["<b>comm</b> · prio 3 · 1 ms<br/>frame decode / encode"] -->|"SPSC ring: Traj, Enable, ClearFault"| CTRL
  CTRL["<b>control</b> · prio 4 · 5 ms (200 Hz)<br/>sense → Controller → PWM"] -->|"seqlock snapshot: State"| COMM
  COMM --> TX["USART1 TX DMA"]
  CTRL -->|"cycle start / end (atomics)"| SUP
  SUP["<b>supervisor</b> · prio 5 · 1 ms<br/>deadline check"] --> OE["OE pins PB0 / PB1<br/>servo outputs on / off"]
  SUP --> IWDG["IWDG ≈ 50 ms<br/>MCU reset on a hang"]
  DIAG["<b>diagnostics</b> · prio 1 · 1 s<br/>timing, CPU, stacks, faults"] -->|"Diag"| COMM
```
*Details in [firmware/stm32f411/README.md](firmware/stm32f411/README.md).*

</details>

---

---

## 🧠 SOTA RL Locomotion Policies

End-to-End Goal-Conditioned RL Policy trained with hardware-accelerated MuJoCo and Apple Silicon Metal GPU (PyTorch MPS).

```mermaid
flowchart LR
  subgraph CPU["Multi-Threaded CPU (Apple M4 / Host)"]
    W1["MuJoCo Worker 1"]
    W2["MuJoCo Worker 2"]
    W8["MuJoCo Worker 8"]
  end

  subgraph GPU["Apple Silicon GPU (Metal / MPS)"]
    TENSORS["Zero-Copy Rollout Buffers<br/>(Unified Memory 16 GB)"]
    PPO["PPO Actor-Critic<br/>DeepRes / Transformer"]
    OPT["Metal Accelerated Update<br/>769k samples/s"]
  end

  subgraph DEPLOY["Deployment & Runtime"]
    ONNX_MODEL["mico_policy.onnx<br/>(< 1.0 ms inference)"]
    ROS_CTRL["ROS 2 Locomotion Node<br/>50 Hz JointTrajectory"]
  end

  CPU -->|148k steps/s| TENSORS
  TENSORS --> PPO
  PPO --> OPT
  OPT -->|Iterate| CPU
  OPT -.->|Export| ONNX_MODEL
  ONNX_MODEL --> ROS_CTRL
```

<!-- evolution:telemetry:start -->
### ⏱️ Autonomous Evolution Counter & Stagnation Self-Review
| Evolutionary Metric | Live Counter Value | Status / Description |
| :--- | :---: | :--- |
| **Cumulative Training Wall-Time** | **`21h 29m 16s`** | Total physical learning time across all generations |
| **Total Evolutionary Generations**| **`218 Generations`** | Number of candidate policies trained & evaluated |
| **Physics Experience Transitions**| **`44,523,520 steps`** | Total MuJoCo simulation steps gathered on Apple Metal |
| **Active World Champion Policy**  | **`v13.5_bidirectional_sprint_champion_rev96`** | **HLI-20: 65.3 / 100** · **19/20 Passed** |
| **10-Gen Stagnation Sentinel**    | **ACTIVE** | Auto-triggers 5 Pain Points &rarr; 25 Solutions &rarr; 5 Fixes |
| **Latest Stagnation Audit**       | [`docs/engineering/evolution_reviews/stagnation_review_gen_218.md`](docs/engineering/evolution_reviews/stagnation_review_gen_218.md) (5 Pain Points Analyzed & Solved) | Comprehensive diagnostic report on file |
<!-- evolution:telemetry:end -->

### 🏆 Foundation Model Evaluation Matrix (Multi-Dimensional Benchmarking)

Mirroring frontier AI evaluation protocols (e.g. Anthropic Claude 3.5 Sonnet / OpenAI GPT-4o model cards), Bazan policies are benchmarked across a multi-dimensional matrix spanning **Environmental Disturbance Robustness (10 Criteria)**, **Biomechanical Human-Likeness (HLI-20 Suite)**, and **Edge Computational Latency**:

| Model Version / Architecture | Overall Score | HLI-20 Index | 20 Motions Passed | Roll Sway (RMS) | Violent Kick (0.08 Ns) | Ice Shock (μ=0.15) | Velocity Reversal | Slope (±10°) | 50% Payload (+280g) | Torque Sag (60%) | P99 Latency (M4) | Params |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 🥇 **`v12.0_adaptive_pitch_sprint`** *(Deep ResNet)* | **95.0** | **59.4 / 100** | **18 / 20** (90%) | **2.5°** | 67% | 100% | 100% | 100% | 100% | 100% | **196 μs** | 2.14M |
| 🥈 **`v9.0_titan_transformer`** *(Multi-Stream TF)* | **96.2** | 53.7 / 100 | 8 / 20 (40%) | 3.8° | 75% | 100% | 100% | 100% | 100% | 100% | 647 μs | 6.67M |
| 🥉 **`v8.1_apex_titan`** *(Titan Deep ResNet)* | **95.6** | 51.4 / 100 | 7 / 20 (35%) | 4.1° | 71% | 100% | 100% | 100% | 100% | 100% | 616 μs | 7.15M |
| ⚡ **`v1.0_light_50iter`** *(Compact MLP Baseline)* | **97.5** | 56.8 / 100 | 13 / 20 (65%) | 3.6° | **83%** | 100% | 100% | 100% | 100% | 100% | **6 μs** | **46k** |
| 📐 **`Baseline Kinematic`** *(Analytical LIPM IK)* | 95.2 | 59.6 / 100 | 18 / 20 (90%) | 11.5° | 79% | 100% | 100% | 100% | 100% | 100% | 0.3 μs | N/A |

> **Active Model Symlink:** `models/mico_policy.onnx` points to **`v12.0_adaptive_pitch_sprint`** — achieving the lowest torso roll sway ($2.5^\circ$), agile forward sprint ($0.5\text{ m/s}$), and full slope compensation.

```bash
# Model Registry CLI:
python3 src/humanoid_control/scripts/model_registry.py activate v12.0_adaptive_pitch_sprint # Activate #1 Champion
python3 src/humanoid_control/scripts/model_registry.py activate v9.0_titan_transformer       # Activate #1 Transformer
python3 src/humanoid_control/scripts/model_registry.py activate v8.1_apex_titan              # Activate ResNet Champion
python3 src/humanoid_control/scripts/model_registry.py list                                  # List all registered policies

# Run 10-Criteria Violent Disturbance Benchmark across all models:
python3 src/humanoid_control/scripts/eval_mico_benchmark.py --compare baseline \
  src/humanoid_control/models/mico_policy_v1.0_light_50iter.onnx \
  src/humanoid_control/models/mico_policy_v8.1_apex_titan.onnx \
  src/humanoid_control/models/mico_policy_v9.0_titan_transformer.onnx \
  src/humanoid_control/models/mico_policy_v12.0_adaptive_pitch_sprint.onnx
```
*Full 10-criteria suite, 20-motion breakdown & throughput specs in [docs/engineering/rl_benchmark.md](docs/engineering/rl_benchmark.md).*

### 🧬 20-Motion Canonical Benchmark & Biomechanical HLI-20

To evaluate true embodied humanoid agility, the robot is benchmarked across **20 canonical motions** measuring **20 biomechanical human-likeness parameters** (HLI-20 composite index):

| Motion Group | Tested Capabilities (20 Canonical Scenarios) |
| :--- | :--- |
| **Locomotion** | Straight Nominal Walk (0.2 m/s), High-Speed Sprint (0.5 m/s), Backward Walk (-0.2 m/s), Standing Vertical Jump |
| **Agility & Turning** | Side-Strafe Left & Right ($\pm 0.15$ m/s), Turn in Place Left & Right ($\pm 0.6$ rad/s), Emergency Brake (0.45 $\to$ 0 m/s) |
| **Terrain & Slopes** | Uphill Climb ($+10^\circ$), Downhill Descent ($-10^\circ$), Hurdle Obstacle Clearance |
| **Posture & Flexibility**| Crouch Walk ($H_0 = 80$ mm), Deep Squat & Stand (65 $\to$ 112 mm), Forward Bow ($30^\circ$), Lateral Lean ($\pm 15^\circ$), Single-Leg Balance |
| **Recovery & Safety** | Getup from Prone (face-down), Getup from Supine (face-up), Omnidirectional 10N Push Impulse Recovery |

```bash
# Run full 20-Motion Benchmark Suite & display 20-parameter HLI report:
python3 src/humanoid_control/scripts/humanoid_benchmark_20motions.py

# Launch nonstop self-improving training loop with warm-start checkpoint resuming:
python3 src/humanoid_control/scripts/autonomous_locomotion_loop.py --nonstop --target-hli 85.0
```

### ⚖️ Quasi-Static vs. Dynamic Locomotion (Anti-Sway Benchmark)

To solve the bipedal sway problem inherent in narrow support polygons ($45 \times 32\text{ mm}$ feet), the policy was trained with an upright tilt penalty $R_{\text{upright}} = -0.50 \cdot (\theta_{\text{roll}})^2 - 0.05 \cdot (\omega_{\text{roll}})^2$:

<div align="center">
  <img src="docs/images/gait_sway_comparison.gif" alt="Quasi-Static Gait vs Dynamic RL Locomotion Comparison" width="700">
</div>

| Gait Strategy | Max Roll Tilt ($\theta_{\text{roll, max}}$) | RMS Roll Sway | Dynamic Behavior | Physical Mechanism |
| :--- | :---: | :---: | :--- | :--- |
| **Quasi-Static Baseline** (`mico_gait.py`) | $\pm 11.46^\circ$ ($\pm 0.20\text{ rad}$) | $11.46^\circ$ | High lateral torso sway | Forces CoM shift directly over stance foot to avoid tipping |
| **Dynamic Policy** (`v9.1_upright_anti_sway`) | **$5.75^\circ$** | **$2.67^\circ$** | **Upright, steady walk (76.7% sway reduction)** | Momentum transfer & Capture Point foot placement to catch falling CoM |

### 🔢 Mathematical Formulation: End-to-End ONNX Locomotion Inference

The locomotion policy is deployed via **ONNX Runtime** at 50 Hz on the Radxa Zero 3W (or host macOS/Linux), executing a continuous mapping from sensory observations to joint target angles:

$$\mathbf{a}_t = \pi_{\boldsymbol{\theta}}(\mathbf{o}_t) \in [-1.0, 1.0]^{12}, \qquad \mathbf{q}_{\text{target}} = \mathbf{q}_{\text{nominal}} + \mathbf{a}_t \odot \Delta \mathbf{q}_{\text{scale}}$$

#### 1. Observation Vector Anatomy ($\mathbf{o}_t \in \mathbb{R}^{47}$)

Every 20 ms, the state estimator packs a 47-dimensional float32 vector:

| Feature Dimension | Quantity | Source / Sensor | Description |
| :--- | :---: | :--- | :--- |
| $\mathbf{g}_{\text{proj}} \in \mathbb{R}^3$ | 3 | MPU-6050 IMU | Projected gravity vector in robot base frame: $\mathbf{R}^T \begin{bmatrix} 0 & 0 & -1 \end{bmatrix}^T$ |
| $\boldsymbol{\omega}_{\text{base}} \in \mathbb{R}^3$ | 3 | MPU-6050 Gyro | Base angular velocity in robot coordinates: $\mathbf{R}^T \boldsymbol{\omega}_{\text{world}}$ |
| $\Delta \mathbf{q} \in \mathbb{R}^{12}$ | 12 | Joint Encoders / Pots | Joint angle error relative to nominal stance: $\mathbf{q}_{\text{meas}} - \mathbf{q}_{\text{nominal}}$ |
| $\dot{\mathbf{q}} \in \mathbb{R}^{12}$ | 12 | Derivative / Velocity | Measured joint angular velocities |
| $\mathbf{a}_{t-1} \in \mathbb{R}^{12}$ | 12 | Controller Memory | Previous action vector (enforces continuous, chatter-free trajectories) |
| $\mathbf{v}_{\text{cmd}} \in \mathbb{R}^3$ | 3 | ROS 2 `/cmd_vel` | Commanded target velocity: $[v_x, v_y, \dot{\psi}]$ |
| $\boldsymbol{\phi} \in \mathbb{R}^2$ | 2 | Phase Clock | Normalized cyclical gait phase: $[\sin(2\pi \phi), \cos(2\pi \phi)]$ |

#### 2. Forward Feed-Forward Execution

Inside `models/mico_policy.onnx`, the input vector passes through layer-normalized residual blocks:

$$\mathbf{h}_1 = \text{GELU}(\text{LayerNorm}(\mathbf{W}_1 \mathbf{o}_t + \mathbf{b}_1)), \qquad \mathbf{h}_{k+1} = \mathbf{h}_k + \text{GELU}(\text{LayerNorm}(\mathbf{W}_k \mathbf{h}_k + \mathbf{b}_k))$$

$$\mathbf{a}_t = \tanh(\mathbf{W}_{\text{out}} \mathbf{h}_{\text{final}} + \mathbf{b}_{\text{out}}) \in [-1.0, 1.0]^{12}$$

#### 3. Actuator Target Translation to Microcontroller PWM

With an action scaling factor $\Delta q_{\text{scale}} = 0.25\text{ rad} \approx 14.3^\circ$, the joint command and corresponding 50 Hz PWM pulse width sent across the PCA9685 driver are:

$$\mathbf{q}_{\text{target}, i} = \mathbf{q}_{\text{nominal}, i} + \mathbf{a}_{t, i} \cdot 0.25$$

$$\text{PWM}_i (\mu\text{s}) = 1500 + \left( \mathbf{q}_{\text{target}, i} \times \frac{180}{\pi} \right) \cdot \frac{1000}{180}$$

---

### ⚠️ Kinematic Singularity Mitigation

Kinematic singularities occur when the leg Jacobian matrix $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{3 \times 3}$ loses rank ($\det(\mathbf{J}) \to 0$), demanding infinite joint velocity ($\dot{\mathbf{q}} = \mathbf{J}^{-1} \dot{\mathbf{x}} \to \infty$) for finite foot Cartesian velocity. Bazan implements a **4-tier defense architecture** against singularities:

```mermaid
flowchart TD
  S["<b>Kinematic Singularity Risks</b><br/>• Knee Lock-Out: θ_knee → 0° (Det(J) → 0)<br/>• Ankle Gimbal Lock<br/>• Out-of-Reach Workspace"]
  
  S --> D1["<b>1. Geometric Pre-Bent Offset (Nominal Crouch)</b><br/>H₀ = 87 mm < L₁ + L₂ = 90 mm<br/>Guarantees θ_knee ≥ 15° at all nominal phases"]
  S --> D2["<b>2. Closed-Form Sagittal IK Clamping</b><br/>d = min(√(x² + z²), L₁ + L₂ - 1e-6)<br/>Clamps domain before arccos() evaluation"]
  S --> D3["<b>3. Damped Least Squares (DLS / Levenberg-Marquardt)</b><br/>J† = Jᵀ (J Jᵀ + λ² I)⁻¹<br/>Trades minor position tracking error for bounded joint velocity"]
  S --> D4["<b>4. RL Soft Joint-Limit Penalty Barrier</b><br/>R_limit = -w · Σ max(0, |q_i| - q_safe)²<br/>Actor naturally stays inside high-manipulability zones"]
```

1. **Nominal Crouch Geometry ($H_0 < L_1 + L_2$):**
   Nominal hip-to-ankle height is clamped at $H_0 = 87\text{ mm}$ while total leg length is $L_1 + L_2 = 45 + 45 = 90\text{ mm}$. The knee is mechanically prevented from locking straight ($\theta_{\text{knee}} = 0$), preserving continuous non-zero Jacobian determinant $\det(\mathbf{J}) > 0$.
2. **Analytical IK Clamping (`mico_gait.py`):**
   ```python
   d = min(math.hypot(x, z), L1 + L2 - 1e-6)
   beta = math.acos(d / (2 * L1))  # Clamped argument strictly in [0, 1), preventing NaN
   ```
3. **Damped Inverse Jacobian (DLS):**
   In numerical inverse kinematics tasks, regularized pseudo-inversion ensures bounded torque and velocity near workspace boundaries:
   $$\mathbf{J}^\dagger = \mathbf{J}^T (\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I})^{-1}, \qquad \lambda^2 = \begin{cases} 0 & \text{if } \sigma_{\min} > \epsilon \\ \lambda_0^2 \left(1 - \frac{\sigma_{\min}}{\epsilon}\right)^2 & \text{otherwise} \end{cases}$$
4. **Reinforcement Learning Workspace Penalty:**
   The PPO reward function enforces a quadratic barrier penalty against joint limit proximity, conditioning the neural policy to exploit regions of maximum kinematic manipulability $w(\mathbf{q}) = \sqrt{\det(\mathbf{J}\mathbf{J}^T)}$.

---

## 📐 Kinematics & Coordinate Frames

<p align="center">
  <img src="docs/design/sketch_shell_a/images/frames.png" width="450" alt="TF frames">
</p>

Conforms to REP-103 (x forward, y left, z up) and REP-120. `base_link` sits at the hip center (z = 115 mm), `imu_link` sits on the pelvis top plate (z = 145 mm).

| Model Target | Source File |
| :--- | :--- |
| ROS 2, RViz2 (URDF / Xacro) | [`src/humanoid_description/urdf/mico.urdf.xacro`](src/humanoid_description/urdf/mico.urdf.xacro) |
| MuJoCo Engine | [`sim/mini_humanoid/humanoid.urdf`](sim/mini_humanoid/humanoid.urdf) |

---

## ⚡ Quick Start

```bash
# 1. Interactive 3D Simulation (Keys: K = kick, 1/2/3 = force, I = ice, +/- = speed):
python3 src/humanoid_control/scripts/mico_visualizer.py

# 2. Native macOS MuJoCo + RViz2 Demo:
conda activate ros2_jazzy && source .mac/install/setup.zsh
ros2 launch humanoid_description mico_mujoco.launch.py gait:=true

# 3. Interactive Web Dashboard:
open docs/index.html   # or visit https://nguyenhau442001.github.io/Synapse_Humanoid/
```
*For detailed macOS / Ubuntu VM installation, see [Getting Started & Simulation Guide](docs/getting_started.md).*

---

## 📁 Repository Structure

| Path | Description |
| Domain | Path & Description |
| :--- | :--- |
| **Hardware** | [`hardware/`](hardware/) · Mechanical CAD (`hardware/mechanical/`), 3D print specs, electrical schematics (`hardware/electrical/`) |
| **Software - ROS 2** | [`software/ros2/src/`](software/ros2/src/) · `humanoid_control`, `humanoid_description`, `humanoid_perception`, `humanoid_infrastructure`, `humanoid_interfaces` |
| **Software - Firmware** | [`software/firmware/`](software/firmware/) · FreeRTOS firmware: 200 Hz control loop, supervisor, PWM, telemetry |
| **Software - Core** | [`software/core/`](software/core/) · Platform-independent real-time C++ core (safety, trajectories, host-tested) |
| **Software - AI & Sim**| [`software/ai/`](software/ai/) · Neural policies (`software/ai/models/`), MuJoCo simulation environment (`software/ai/sim/`) |
| **Design** | [`design/`](design/) · Blender models (`design/blender/`), aesthetic concepts (`design/sketch_shell_a/`), product renders |
| **Documentation** | [`docs/`](docs/) · System architecture, mass budget, torque validation, and engineering manuals |

---

## 🗺️ Roadmap & Embodied AI Engineering Curriculum

### 1. Platform Engineering Milestones

- [x] 20-DoF kinematic model, TF tree, and URDF/Xacro generators
- [x] MuJoCo native bridge + ROS 2 Jazzy integration on macOS
- [x] Blender Design C shell: visual meshes, mass budget (684 g), 3D print fit check
- [x] Apple Silicon M4 training acceleration (148k SPS MuJoCo, 769k SPS Metal MPS)
- [x] Goal-Conditioned PPO Locomotion & SOTA Hybrid Transformer policy
- [x] Real-time 50 Hz ONNX inference node & 3D Interactive Simulation
- [ ] P0 actuator bench: MG92B hip-roll thermal test & analog mux telemetry
- [ ] ZMP / LIPM footstep trajectory planner & closed-form leg IK
- [ ] micro-ROS UART communication between Radxa Zero 3W and STM32F411

### 2. Personal Action Plan: 4-Month Embodied AI Curriculum (Todo-List)

A structured technical roadmap from spatial kinematics to hardware-in-the-loop Embodied AI deployment:

| Phase | Core Domain | Key Checklists & Hands-On Deliverables | Code Reference |
| :---: | :--- | :--- | :--- |
| **Month 1** | **Level 1 & 2: 3D Spatial Geometry & ROS 2** | • Master $3 \times 3$ Rotation Matrices, Shepperd Quaternions & SE(3) Homogeneous Transforms.<br/>• Derive closed-form leg IK geometrically ($\beta = \arccos\frac{d}{2L_1}$), verify singularity bounds.<br/>• Inspect URDF/Xacro, link coordinate trees (`REP-103`/`REP-120`) and RViz2 TF broadcasting. | [`mico_playback.py`](src/humanoid_control/scripts/mico_playback.py)<br/>[`mico_gait.py`](src/humanoid_control/scripts/mico_gait.py)<br/>[`mico.urdf.xacro`](src/humanoid_description/urdf/mico.urdf.xacro) |
| **Month 2** | **Level 4: Bipedal Dynamics & MuJoCo Physics** | • Analyze Linear Inverted Pendulum Model ($T_c = \sqrt{z_c/g} = 121\text{ ms}$) and Pratt Capture Point.<br/>• Study Zero Moment Point (ZMP) stability criterion during single-support stance.<br/>• Run MuJoCo 1 kHz simulation (`mico.xml`), experiment with contact solvers, friction, and crash scenarios. | [`mass_budget.md`](docs/engineering/mass_budget.md)<br/>[`check_mujoco.py`](sim/mujoco/check_mujoco.py)<br/>[`mico.xml`](sim/mujoco/mico.xml) |
| **Month 3** | **Level 5: Deep RL (PPO) & Sim-to-Real Transfer** | • Master Markov Decision Processes, Advantage Estimation (GAE), and PPO clipped objective.<br/>• Analyze 47-dim observation construction, reward shaping, and action rate filtering in `mico_rl_env.py`.<br/>• Train custom policy on Apple Silicon MPS; export and benchmark ONNX Runtime (< 500 μs latency). | [`train_mico_ppo.py`](src/humanoid_control/scripts/train_mico_ppo.py)<br/>[`mico_rl_env.py`](src/humanoid_control/scripts/mico_rl_env.py)<br/>[`model_registry.py`](src/humanoid_control/scripts/model_registry.py) |
| **Month 4** | **Level 3: Real-Time Embedded Systems (M0/M1)** | • Study heterogeneous 2-domain architecture: Radxa Linux (planning) vs STM32 FreeRTOS (action).<br/>• Verify 200 Hz control loop, zero-allocation lock-free SPSC rings, and 1 ms IWDG safety supervisor.<br/>• Run automated `m0-accept` test suite over UART 921600 baud, confirm zero missed deadlines. | [`hardware_architecture.md`](docs/engineering/hardware_architecture.md)<br/>[`m0_report.md`](docs/engineering/m0_report.md)<br/>[`firmware/stm32f411`](firmware/stm32f411) |

---

## 📄 License

Apache 2.0.
