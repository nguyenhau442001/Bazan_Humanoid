# RL Locomotion Benchmark Suite & Model Evaluation

This document details the multi-dimensional evaluation methodology, foundation model comparison cards, 10-criteria violent disturbance benchmarks, 20 canonical motion scenarios, and 20 biomechanical human-likeness parameters (HLI-20) for the Bazan humanoid platform.

---

## 🏆 Foundation Model Evaluation Leaderboard

Mirroring frontier foundation AI benchmarks (e.g. Anthropic Claude Opus / OpenAI GPT-4o model cards), Bazan policies are evaluated across a multi-dimensional matrix rather than a single scalar score:

| Evaluation Dimension | Metric / Stress Range | Baseline Kinematic | v1.0 Light MLP | v8.1 Apex Titan | v9.0 Titan Transformer | v12.0 Adaptive Pitch *(Active Champion)* |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **Overall Robustness** | Composite Score (0 - 100) | 95.2 | **97.5** | 95.6 | 96.2 | **95.0** |
| **HLI-20 Biomechanics** | 20 Parameters (0 - 100) | 59.6 | 56.8 | 51.4 | 53.7 | **59.4** |
| **20-Motion Agility** | Tasks Passed (/20) | 18 / 20 | 13 / 20 | 7 / 20 | 8 / 20 | **18 / 20** (90%) |
| **Roll Sway Suppression** | RMS Torso Roll Tilt ($^\circ$) | 11.5° *(Waddle)* | 3.6° | 4.1° | 3.8° | **2.5°** *(Steady)* |
| **1. Violent Kick Shock** | Omnidirectional 0.08 Ns | 79% | **83%** | 71% | 75% | 67% |
| **2. Extreme Ice Shock** | Friction drop $\mu = 0.15$ | 100% (Yaw: 4.0°) | 100% (Yaw: 0.0°) | 100% (Yaw: 0.6°) | 100% (Yaw: 1.4°) | 100% (Yaw: 1.5°) |
| **3. Velocity Reversal** | $+0.4 \leftrightarrow -0.4$ m/s rapid | 100% | 100% | 100% | 100% | 100% |
| **4. Drop Impact Shock** | 10 cm freefall drop + 15° tilt | 83% | 100% | 100% | 100% | 100% |
| **5. Continuous Slope** | $\pm 10^\circ$ continuous incline | 100% | 100% | 100% | 100% | 100% |
| **6. Rough Terrain** | $\pm 15$ mm dynamic bumps | 100% | 100% | 100% | 100% | 100% |
| **7. Payload Mass Shock** | +50% bodyweight (+0.28 kg) | 100% | 100% | 100% | 100% | 100% |
| **8. Torque Saturation** | 60% torque limit (voltage sag) | 100% | 100% | 100% | 100% | 100% |
| **9. Sensor Noise & Lag** | 10 ms lag + IMU Gaussian jitter| 100% | 100% | 100% | 100% | 100% |
| **10. Edge P99 Latency** | Apple Silicon M4 single-step | **0.3 μs** | **5.8 μs** | 616.0 μs | 646.7 μs | **196.3 μs** |
| **Model Parameters** | Weights count | 0 *(Analytical)*| 46,220 | 7,153,164 | 6,674,828 | 2,141,196 |
| **Sprint Max Velocity** | Linear $v_x$ forward limit | 0.20 m/s | 0.25 m/s | 0.35 m/s | 0.35 m/s | **0.50 m/s** |

---

## 🏋️ 10-Criteria Comprehensive & Violent Locomotion Benchmark Suite

Validated via [`eval_mico_benchmark.py`](../../src/humanoid_control/scripts/eval_mico_benchmark.py):

1. **Omnidirectional Violent Kick Disturbance (15%):** Impulses $(0.04 - 0.20\text{ Ns})$ across 6 angles ($0^\circ, 45^\circ, 90^\circ, 135^\circ, 180^\circ, 270^\circ$).
2. **Extreme Friction Drop / Ice Shock (10%):** Instantaneous friction drop $\mu = 1.0 \to 0.15$ with yaw drift tracking.
3. **Dynamic Velocity Reversal & Spin Agility (10%):** Rapid command $+0.25 \leftrightarrow -0.20\text{ m/s}$ with $\pm 0.6\text{ rad/s}$ yaw spin.
4. **Drop Impact & Tilt Recovery (10%):** Dropped from elevation ($0.05 - 0.10\text{ m}$) with $\pm 15^\circ$ pitch/roll orientation stumbles.
5. **Continuous Slope / Ramp Incline Traversal (10%):** Uphill and downhill continuous slopes ($\pm 8^\circ, \pm 12^\circ$ gravity pitch).
6. **Rough Terrain Ground Jitter (10%):** Dynamic vertical micro-bumps ($\pm 0.015\text{ m}$) and terrain cracks during continuous walking.
7. **Base Payload Mass Perturbation (10%):** Carrying up to $+0.28\text{ kg}$ payload mass on base link (> 50% robot body weight).
8. **Actuator Torque Saturation / Voltage Sag (10%):** Motor torque limits scaled down to $75\%$ and $60\%$ simulating battery voltage drops.
9. **Proprioceptive Sensor Noise & Observation Lag (10%):** Joint encoder noise $\mathcal{N}(0, 0.04^2)$ with 1-step latency lag.
10. **Hardware Inference Latency & Efficiency on M4 (5%):** P99 single-inference latency evaluated against the 20 ms (50 Hz) ROS 2 control cycle.

To run the head-to-head comparison across all policies:
```bash
python3 src/humanoid_control/scripts/eval_mico_benchmark.py --compare baseline \
  src/humanoid_control/models/mico_policy_v1.0_light_50iter.onnx \
  src/humanoid_control/models/mico_policy_v8.1_apex_titan.onnx \
  src/humanoid_control/models/mico_policy_v9.0_titan_transformer.onnx \
  src/humanoid_control/models/mico_policy_v12.0_adaptive_pitch_sprint.onnx
```

---

## 🧬 20-Motion Canonical Agility & HLI-20 Biomechanical Index

Evaluated via [`humanoid_benchmark_20motions.py`](../../src/humanoid_control/scripts/humanoid_benchmark_20motions.py):

### A. 20 Canonical Motion Scenarios
1. **Straight Nominal Walk:** Steady forward translation at $0.20\text{ m/s}$.
2. **High-Speed Sprint:** Dynamic sprint at $0.50\text{ m/s}$ with forward torso pitch compensation.
3. **Backward Walk:** Reverse translation at $-0.20\text{ m/s}$.
4. **Side-Strafe Left:** Lateral translation at $+0.15\text{ m/s}$.
5. **Side-Strafe Right:** Lateral translation at $-0.15\text{ m/s}$.
6. **Turn in Place Left:** Pure rotational velocity $+0.60\text{ rad/s}$.
7. **Turn in Place Right:** Pure rotational velocity $-0.60\text{ rad/s}$.
8. **Uphill Slope Climb:** Navigating $+10^\circ$ continuous incline without falling backwards.
9. **Downhill Slope Descent:** Navigating $-10^\circ$ decline with controlled braking.
10. **Crouch Walk:** Low center-of-mass translation ($H_0 = 80\text{ mm}$).
11. **Deep Squat & Stand:** Vertical compression from $112\text{ mm} \to 65\text{ mm}$ and recovery.
12. **Forward Bow Lean:** Torso flexed $30^\circ$ forward maintaining balance.
13. **Lateral Torso Lean:** Torso tilted $\pm 15^\circ$ laterally without foot lift.
14. **Standing Vertical Jump:** Explosive vertical extension and compliant landing.
15. **Hurdle Obstacle Clearance:** Stepping over ground obstacles with high swing clearance.
16. **Get-Up from Prone:** Ground recovery from face-down horizontal orientation.
17. **Get-Up from Supine:** Ground recovery from face-up horizontal orientation.
18. **Single-Leg Stance Balance:** Static equilibrium supported on one foot for $\ge 2.0\text{ s}$.
19. **Emergency Brake Stop:** Rapid deceleration from $0.45\text{ m/s} \to 0\text{ m/s}$ within $0.2\text{ s}$.
20. **10N Push Shock Recovery:** Re-establishing upright equilibrium after a violent $10\text{ N}$ lateral push.

### B. 20 Biomechanical Parameters (HLI-20 Scoring Weights)
Each parameter evaluates biological fidelity on a normalized scale (5.0 points max each, 100.0 total points):

| # | Parameter Name | Biological Target / Formula | Metric Focus | Weight |
| :- | :--- | :--- | :--- | :-: |
| **P01** | `roll_sway` | $\text{RMS}(\theta_{\text{roll}}) < 3.0^\circ$ | Torso anti-waddle stability | 5 pts |
| **P02** | `pitch_posture` | $\text{Mean}(\lvert \theta_{\text{pitch}} - \theta_{\text{target}} \rvert) < 7.0^\circ$ | Upright spinal posture | 5 pts |
| **P03** | `yaw_drift` | $\lvert \dot{\psi}_{\text{drift}} \rvert < 1.0^\circ/\text{s}$ | Straight-line heading hold | 5 pts |
| **P04** | `lateral_drift` | $\lvert y(t) - y_0 \rvert < 0.05\text{ m}$ | Minimal cross-track error | 5 pts |
| **P05** | `speed_tracking` | $\text{RMSE}(v_x - v_{\text{cmd}}) < 0.05\text{ m/s}$ | Linear command compliance | 5 pts |
| **P06** | `pelvis_bobbing` | $\sigma(z_{\text{base}}) < 8.0\text{ mm}$ | Smooth inverted-pendulum CoM | 5 pts |
| **P07** | `swing_clearance` | $\Delta z_{\text{foot, max}} \ge 15.0\text{ mm}$ | Stumble-free toe clearance | 5 pts |
| **P08** | `stance_symmetry` | $\lvert T_{\text{stance, L}} - T_{\text{stance, R}} \rvert / T < 0.05$ | Left-right gait symmetry | 5 pts |
| **P09** | `arm_reciprocity` | $\text{Corr}(q_{\text{arm}}, q_{\text{leg, opp}}) > 0.80$ | Natural counter-arm swing | 5 pts |
| **P10** | `knee_naturalness` | $q_{\text{knee}} \in [0.15, 0.70]\text{ rad}$ (No hyperextension) | Joint anatomical safety | 5 pts |
| **P11** | `ankle_alignment` | Foot sole parallel to floor $(\Delta \theta_{\text{sole}} < 3.0^\circ)$ | Ground contact compliance | 5 pts |
| **P12** | `jerk_smoothness` | $\int \lVert \dddot{\mathbf{q}} \rVert^2 dt < 10^3\text{ rad}^2/\text{s}^5$ | Minimal actuator vibration | 5 pts |
| **P13** | `energy_efficiency` | $\text{Cost of Transport (CoT)} < 2.5$ | Battery endurance | 5 pts |
| **P14** | `impact_shock` | $\max(F_{\text{ground}}) < 2.0\text{ bodyweights}$ | Soft heel strikes | 5 pts |
| **P15** | `getup_recovery` | Standing restore time $< 3.0\text{ s}$ | Fall self-recovery | 5 pts |
| **P16** | `zero_fall_rate` | Fall count $= 0$ across all trials | Zero catastrophic falls | 5 pts |
| **P17** | `turning_agility` | Dynamic yaw rate $\dot{\psi} \ge 0.50\text{ rad/s}$ | High-speed maneuverability | 5 pts |
| **P18** | `squat_compliance` | Leg deflection under vertical load $> 90\%$ | Compliant suspension | 5 pts |
| **P19** | `yaw_roll_decoupling` | Cross-coupling roll $< 2.0^\circ$ during turns | Decoupled rotational DOF | 5 pts |
| **P20** | `push_recovery` | Settling time after $10\text{ N}$ push $< 0.4\text{ s}$ | Rapid balance restore | 5 pts |

To run the complete 20-motion suite and view the HLI-20 report:
```bash
python3 src/humanoid_control/scripts/humanoid_benchmark_20motions.py --model models/mico_policy.onnx
```

---

## ⚡ Apple Silicon M4 Hardware Acceleration Profile

Validated via [`benchmark_mico_sim.py`](../../src/humanoid_control/scripts/benchmark_mico_sim.py):

* **Native C MuJoCo Engine:** **147,970 steps/second** (148× real-time) with 8 parallel worker threads.
* **PyTorch MPS (Metal GPU):** **769,271 transitions/second** forward + backward throughput for policy optimization.
* **Unified Memory (UMA) Zero-Copy:** Experience batches transfer between CPU physics and Metal GPU without PCIe bandwidth bottlenecks.
* **Policy Footprint:** Lightweight MLP to Titan Deep ResNet & Hybrid Transformer (46k to 10M+ parameters), ensuring real-time execution (< 1.0 ms) on host and onboard compute.

Run simulation and training throughput benchmark:
```bash
python3 src/humanoid_control/scripts/benchmark_mico_sim.py
```

---

## 🛠️ Model Registry CLI

All trained locomotion models are cataloged in [`models/`](../../models/) and managed via [`model_registry.py`](../../src/humanoid_control/scripts/model_registry.py).

```bash
# List all trained policies and metadata
python3 src/humanoid_control/scripts/model_registry.py list

# Activate top model (v7.25 evolved)
python3 src/humanoid_control/scripts/model_registry.py activate v7.25_evolved

# Activate SOTA Hybrid Transformer (v8.3)
python3 src/humanoid_control/scripts/model_registry.py activate v8.3_transformer_hybrid

# Rollback to lightweight MLP baseline (v1.0)
python3 src/humanoid_control/scripts/model_registry.py activate v1.0_light_50iter
```

---

## 📚 Mathematical Formulations & References

For complete mathematical definitions of PPO, GAE, observation vector derivations, and scientific literature citations, see **[Theory, Formulations & References](../references.md)**.

