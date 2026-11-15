# Comprehensive Post-Mortem & Lessons Learned from Sub-90 Point RL Models

> **Document Version:** 1.0.0  
> **Repository:** Synapse / Bazan Humanoid (BZN-01)  
> **Scope:** Analysis of 249 sub-90 point neural locomotion policies across Day 1 to Generation 248  
> **Key Objective:** Synthesize root causes of failure modes, instability, and biomechanical deficits to inform the architecture scaling to 10M Titan Transformer.

---

## Executive Summary

Across continuous evolutionary training and violent disturbance benchmarks, 249 policy generations scored under **90.0 / 100 points**. Rather than treating these as wasted compute, this audit compiles the exact empirical failure telemetry, mathematical causes, and the corresponding design fixes incorporated into our state-of-the-art controllers.

---

## 1. Six Primary Failure Clusters

### Cluster 1: Attention Saturation & Transformer Gradient Collapse
* **Representative Models:** `v8.2_transformer_prime` (48.5 pts), `v8.0_massive_res_17M` (87.2 pts).
* **Failure Symptoms:**
  - Forward velocity commands caused explosive divergence in joint velocities ($\dot{q} > 30\text{ rad/s}$).
  - Attention maps in Multi-Head Self-Attention collapsed to identity matrices or uniform distributions.
* **Root Mathematical Cause:**
  - In `v8.2`, Post-LayerNorm was used without pre-activation scaling. During PPO gradient updates with clip ratio $\epsilon = 0.20$, large advantage weights caused gradient explosion in the query-key projection layers ($W_Q, W_K$).
* **Permanent Architectural Fix:**
  - Standardized on **Pre-LayerNorm (`norm_first=True`)** with GELU activations.
  - Added a direct residual identity shortcut connecting the input stem directly to the fused output head: $h_{\text{fused}} = h_{\text{res}} + \text{Transformer}(h_{\text{res}})$. This guarantees baseline proprioception passes cleanly even if attention weights oscillate.

---

### Cluster 2: Perception Normalization & Telemetry Noise Inundation
* **Representative Models:** `v14.1_obstacle` (23.7 pts), `v14.0_hardware_perceptive_84dim` (82.7 pts).
* **Failure Symptoms:**
  - Robot walked stably on flat ground, but fell instantly upon encountering micro-bumps or obstacle hurdles.
  - Foot placement exhibited high-frequency twitching.
* **Root Engineering Cause:**
  - Raw un-normalized sensor readings (ToF distance in millimeters ranging $0 \dots 2000$, and raw analog contact voltages from CD74HC4067 FSR) were fed directly into the input layer. The high numerical variance of these channels dominated the small variance of proprioceptive joint angles ($[-0.5, 0.5]\text{ rad}$).
* **Permanent Architectural Fix:**
  - Implemented non-linear sigmoid normalization:
    $$\text{FSR}_{\text{norm}} = \frac{1}{1 + \exp(-0.8 \cdot (\text{Force} - 2.5))}$$
  - Laser ToF distance was range-clamped to $[0.0, 1.0]$ and compensated by head-pitch kinematic tilt.
  - Streamlined input representation into the whole-body 64-dimensional space (49 proprio + 15 normalized perception).

---

### Cluster 3: Lateral Kick Disturbance & Narrow Stance Instability
* **Representative Models:** `v3.1_deep_curriculum` (65.8 pts), `v5.5_evolved` (47.4 pts), `v7.10` (49.9 pts).
* **Failure Symptoms:**
  - Robot tipped over sideways when subjected to lateral impulses $> 0.06\text{ Ns}$ (survival $< 50\%$).
* **Root Biomechanical Cause:**
  - The policy maintained feet parallel with lateral spacing equal to nominal hip width ($y \approx \pm 0.038\text{ m}$). When perturbed laterally, the Center of Pressure (CoP) immediately hit the lateral edge of the support polygon, leaving zero margin for ankle roll torque compensation.
* **Permanent Architectural Fix:**
  - Dedicated **Decoupled Lateral Balance Head (`head_lateral`)** outputting independent roll joint actions for hip roll and ankle roll.
  - Integrated 6-DOF arm swing flaring (`head_arms`) to generate counter-torques against roll axis disturbances.
  - Progressive domain randomization with `perturb_prob` scaled up to $0.06 \dots 0.08$ with omnidirectional kick angles ($0 \dots 2\pi$).

---

### Cluster 4: Forward Sprint Pitch-Lean & ZMP Backward Lag
* **Representative Models:** `v4.1_ultra_res` (60.8 pts), `v7.11` (52.2 pts), `v7.22` (60.8 pts).
* **Failure Symptoms:**
  - Robot stumbled and fell backward at speeds $> 0.35\text{ m/s}$ (Sprint test survival $< 30\%$).
* **Root Dynamic Cause:**
  - At higher velocities, accelerating the limbs forward produces inertial reaction torques that push the pelvis backward. Without leaning the torso forward, the Center of Mass (CoM) lags behind the Zero Moment Point (ZMP).
* **Permanent Architectural Fix:**
  - Coordinated torso pitch-lean parameter ($0.08\text{ rad}$) modulated through the adaptive head (`head_adaptive`).
  - Dynamic cadence scaling (shortening step period from $0.42\text{ s}$ to $0.28\text{ s}$) to prevent over-striding.

---

### Cluster 5: Ice / Low Friction Yaw Spin
* **Representative Models:** `v3.4_robust_kicks` (84.6 pts), `v5.6_evolved` (52.0 pts), `v7.2` (60.0 pts).
* **Failure Symptoms:**
  - Upon transitioning to low friction ($\mu = 0.15$), the robot yawed $> 90^\circ$ and spun out of control.
* **Root Control Cause:**
  - High stiffness on ankle yaw and excessive hip yaw torque while ground reaction forces were near zero.
* **Permanent Architectural Fix:**
  - Straight-track yaw constraint penalty:
    $$r_{\text{yaw}} = -\exp(12 \cdot |\theta_{\text{yaw}}|)$$
  - Cyclic phase clock conditioning ($[\sin(2\pi\phi), \cos(2\pi\phi)]$) enforcing compliance during the swing-to-stance transition.

---

### Cluster 6: High-Frequency Servo Chattering & Voltage Sag
* **Representative Models:** `v6.1_grandmaster` (78.3 pts), `v7.31` (74.8 pts).
* **Failure Symptoms:**
  - Actuator chattering caused bus voltage sag down to $6.8\text{ V}$ and motor casing overheating ($> 65^\circ\text{C}$).
* **Permanent Architectural Fix:**
  - Exponential moving average action filter ($\alpha = 0.60$):
    $$a_t^{\text{exec}} = 0.60 a_{t-1}^{\text{exec}} + 0.40 a_t^{\text{raw}}$$
  - Action smoothness penalty: $r_{\text{smooth}} = -0.05 \cdot \|a_t - a_{t-1}\|^2$.

---

## 2. Summary of Models Retained ($\ge 90.0$ Points)

All sub-90 point models are pruned from disk and registry. The 9 gold-standard checkpoints retained are:

| Model Version | Architecture | Parameters | Benchmark Score | Role |
| :--- | :--- | :---: | :---: | :--- |
| **`v9.0_titan_transformer`** | `titan_transformer` | 6,674,828 | **96.2 / 100** | All-time highest locomotion benchmark |
| **`v8.1_apex_titan`** | `deep_res` | 3,184,332 | **95.6 / 100** | ResNet apex reference |
| **`v7.25_evolved`** | `deep_res` | 2,136,876 | **95.2 / 100** | High-cadence sprint champion |
| **`v12.0_adaptive_pitch_sprint`**| `deep_res` | 2,142,012 | **95.0 / 100** | Adaptive pitch lean champion |
| **`v6.0_apex_master`** | `deep_res` | 2,136,876 | **94.0 / 100** | Violent disturbance rejection |
| **`v8.3_transformer_hybrid`** | `transformer` | 10,060,300 | **92.3 / 100** | 10M Transformer hybrid reference |
| **`v7.3_evolved`** | `deep_res` | 2,136,876 | **91.2 / 100** | Energy efficiency champion |
| **`v7.20_evolved`** | `deep_res` | 2,136,876 | **91.2 / 100** | Zero-fall endurance reference |
| **`v16.0_perceptive_transformer_gen1`**| `perceptive_transformer`| 6,301,088 | **90.4 / 100** | Active whole-body 64-in/32-out champion |

---

## 3. Architecture Specification: 10M Titan Transformer

In accordance with user direction, the next evolutionary jump transitions from `perceptive_transformer` to **`titan_transformer`** scaled to **10.01M Actor parameters**:

* **Inputs:** 64 dimensions (Proprioception + 15 Perception channels).
* **Outputs:** 32 dimensions (20 whole-body DOF joints + 12 adaptive controls).
* **Hidden Dimension:** $512$
* **Residual Backbone:** $6$ Pre-Activation ResBlocks (`num_blocks=6`).
* **Transformer Attention:** $3$ Multi-Head Self-Attention layers (`num_layers=3`, $4$ heads, `dim_feedforward=1024`, Pre-LayerNorm).
* **Specialty Heads:** $6$ Decoupled Heads (`head_sagittal`, `head_lateral`, `head_yaw`, `head_arms`, `head_neck`, `head_adaptive`).
* **Actor Parameters:** **`10,010,752` (10.01 Million)**.
* **Total Actor-Critic Parameters:** **`19,653,889` (19.65 Million)**.
* **Apple Silicon M4 Inference Latency:** **`1.77 ms`** (yielding $> 11\times$ real-time headroom within the $20\text{ ms}$ control budget).
