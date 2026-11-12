#!/usr/bin/env python3
"""Comprehensive & Violent Locomotion Benchmark Suite for Mico Humanoid.

Implements cutting-edge humanoid robotics evaluation protocols across 10 rigorous criteria:
1. Omnidirectional Violent Kick Disturbance Rejection (Impulses 0.04 to 0.20 Ns across 6 angles).
2. Extreme Friction Drop / Ice Shock (Instantaneous drop from mu=1.0 to mu=0.15).
3. Dynamic Agility & Rapid Velocity Reversal (+0.25 m/s to -0.20 m/s with aggressive spin).
4. Drop Impact & Tilt Shock Recovery (Dropped from height with pitch/roll tilt).
5. Continuous Slope / Ramp Incline Traversal (Uphill and downhill slopes +-8 deg and +-12 deg).
6. Rough Terrain Micro-Bumps & Ground Jitter (Dynamic vertical micro-bumps and terrain shocks).
7. Base Payload Mass Perturbation (+0.12 kg to +0.28 kg extra payload, up to 50% robot body mass).
8. Actuator Torque Saturation / Battery Voltage Sag (Torque limits scaled to 75% and 60%).
9. Proprioceptive Sensor Noise & 1-Step Latency Lag (Observation noise and delayed feedback).
10. Hardware Compute Latency & Real-Time Headroom on Apple Silicon M4 (< 20 ms control budget).

Outputs a formatted Markdown comparison leaderboard.

Usage:
    # Evaluate active model
    python3 src/humanoid_control/scripts/eval_mico_benchmark.py

    # Compare baseline vs v1.0 vs v2.0
    python3 src/humanoid_control/scripts/eval_mico_benchmark.py --compare \
        baseline \
        src/humanoid_control/models/mico_policy_v1.0_light_50iter.onnx \
        src/humanoid_control/models/mico_policy_v2.0_deep_500iter.onnx
"""
import argparse
import os
from pathlib import Path
import sys
import time

import numpy as np
import onnx
import onnxruntime as ort

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE))
import mico_gait as gait  # noqa: E402
import mico_rl_env as env_mod  # noqa: E402
import model_registry as reg_mod  # noqa: E402


class BenchmarkEvaluator:
    """Evaluates humanoid locomotion controller against violent disturbances."""

    def __init__(self, model_target="active", decimation=20):
        self.decimation = decimation
        self.model_target = model_target
        self.is_baseline = (model_target == "baseline")

        if self.is_baseline:
            self.model_name = "Baseline Kinematic (mico_gait)"
            self.session = None
            self.param_count = 0
            self.file_size_kb = 0.0
        else:
            path = self._resolve_model_path(model_target)
            self.model_path = path
            self.model_name = path.stem
            self.session = ort.InferenceSession(str(path))
            self.input_name = self.session.get_inputs()[0].name

            # Inspect ONNX
            try:
                onnx_m = onnx.load(str(path))
                self.param_count = reg_mod.count_onnx_parameters(onnx_m)
                self.file_size_kb = path.stat().st_size / 1024.0
            except Exception:
                self.param_count = 0
                self.file_size_kb = 0.0

        # MuJoCo env - Detect residual gait policies & phase dimension
        is_res = False
        reg = reg_mod.load_registry()
        active_ver = reg.get("active_version")
        if self.model_name == "mico_policy.onnx" and active_ver in reg.get("models", {}):
            val = reg["models"][active_ver].get("residual")
            if val is True or str(val).lower() == "true":
                is_res = True
        else:
            for k, v in reg.get("models", {}).items():
                if k in self.model_name:
                    val = v.get("residual")
                    if val is True or str(val).lower() == "true":
                        is_res = True
                    break
        if not is_res:
            is_res = any(kw in self.model_name for kw in [
                "residual", "v2.5", "v3", "v4", "v5", "v6", "v7", "v8", "apex", "titan", "perceptive"
            ])
        self.is_residual = is_res

        use_phase = False
        use_perception = False
        if self.session is not None:
            try:
                inp_shape = self.session.get_inputs()[0].shape
                if inp_shape[-1] in (47, 49, 51, 60, 84, 1024):
                    use_phase = True
                use_perception = (inp_shape[-1] in (60, 84, 1024))
            except Exception:
                use_phase = False
                use_perception = False

        self.env = env_mod.MicoEnv(
            decimation=decimation,
            max_episode_steps=600,
            use_residual=self.is_residual,
            use_phase=use_phase,
            use_perception=use_perception,
            action_filter=0.60,
        )

    def _resolve_model_path(self, target):
        cand = Path(target)
        if cand.exists():
            return cand.resolve()

        # Try registry version
        registry = reg_mod.load_registry()
        if target in registry.get("models", {}):
            rel = registry["models"][target]["relative_path"]
            return (reg_mod.MODELS_DIR / rel).resolve()

        if target == "active":
            if reg_mod.ACTIVE_SYMLINK.exists():
                return reg_mod.ACTIVE_SYMLINK.resolve()
            # fallback
            def_path = reg_mod.MODELS_DIR / "mico_policy.onnx"
            if def_path.exists():
                return def_path.resolve()

        raise FileNotFoundError(f"Cannot resolve policy model from target: '{target}'")

    def _get_action(self, obs, t_sec, command):
        """Get action from policy or baseline gait."""
        if self.is_baseline:
            # Kinematic gait action
            phase = (t_sec % gait.T_STEP) / gait.T_STEP
            left_swing = (phase < 0.5)
            sub_phase = (phase * 2.0) if left_swing else ((phase - 0.5) * 2.0)
            x_step = command[0] * gait.T_STEP

            hp, kn, ap = gait.leg_ik(0, gait.H0)
            # Baseline offset from nominal (normalized roughly to [-1, 1])
            action = np.zeros(12, dtype=np.float32)
            # Swing leg pitch motion
            swing_hp, swing_kn, swing_ap = gait.leg_ik(x_step * np.sin(sub_phase * np.pi), gait.H0 + 0.015)
            if left_swing:
                action[2] = (swing_hp - hp) / 0.25
                action[3] = (swing_kn - kn) / 0.25
                action[4] = (swing_ap - ap) / 0.25
            else:
                action[8] = (swing_hp - hp) / 0.25
                action[9] = (swing_kn - kn) / 0.25
                action[10] = (swing_ap - ap) / 0.25
            return np.clip(action, -1.0, 1.0)
        else:
            obs_batch = obs.reshape(1, -1).astype(np.float32)
            out = self.session.run(None, {self.input_name: obs_batch})
            return np.clip(out[0][0], -1.0, 1.0)

    # =========================================================================
    # Test 1: Omnidirectional Violent Kick Disturbance (Kick Resistance)
    # =========================================================================
    def test_violent_kick_resistance(self, impulses=(0.04, 0.08, 0.14, 0.20), angles_deg=(0, 45, 90, 135, 180, 270)):
        """Subject robot to horizontal kicks while walking forward."""
        survived_count = 0
        total_trials = 0
        max_survived_impulse = 0.0
        settle_times = []

        command = np.array([0.20, 0.0, 0.0])

        for impulse in impulses:
            impulse_survived = True
            for ang_deg in angles_deg:
                total_trials += 1
                rad = np.radians(ang_deg)
                # Force applied over 5 control steps (0.10s) -> F = J / dt
                dt_impulse = 5 * (self.decimation * 0.001)  # 0.10s
                force_mag = impulse / dt_impulse
                force_vec = np.array([force_mag * np.cos(rad), force_mag * np.sin(rad), 0.0])

                obs = self.env.reset(command=command)
                # Let robot establish steady walking for 40 steps (0.8s)
                fallen = False
                for step in range(40):
                    act = self._get_action(obs, step * 0.02, command)
                    obs, _, term, _, _ = self.env.step(act)
                    if term:
                        fallen = True
                        break

                if fallen:
                    impulse_survived = False
                    continue

                # Deliver the violent kick!
                self.env.apply_perturbation(force_vec, duration_steps=5)

                survived_kick = True
                settled_step = None
                for post_step in range(80):
                    t_sec = (40 + post_step) * 0.02
                    act = self._get_action(obs, t_sec, command)
                    obs, _, term, _, info = self.env.step(act)

                    if term:
                        survived_kick = False
                        break

                    # Check settling (upright gravity z < -0.96 and ang vel < 0.5)
                    if settled_step is None and post_step > 8:
                        if info["proj_gravity_z"] < -0.96 and abs(self.env.data.qvel[3]) < 0.3:
                            settled_step = post_step

                if survived_kick:
                    survived_count += 1
                    t_rec = (settled_step * 0.02) if settled_step is not None else 1.6
                    settle_times.append(t_rec)
                else:
                    impulse_survived = False

            if impulse_survived:
                max_survived_impulse = impulse

        survival_rate = (survived_count / max(1, total_trials)) * 100.0
        avg_settle = float(np.mean(settle_times)) if settle_times else 2.0
        return {
            "survival_rate": survival_rate,
            "max_survived_impulse": max_survived_impulse,
            "avg_recovery_time_s": avg_settle,
            "total_trials": total_trials,
            "survived_trials": survived_count,
        }

    # =========================================================================
    # Test 2: Extreme Friction Drop (Ice Shock)
    # =========================================================================
    def test_ice_shock(self, trials=5, ice_friction=0.15):
        """Instantaneous drop from normal friction (1.0) to ice (0.15)."""
        survived_count = 0
        yaw_drifts = []
        command = np.array([0.20, 0.0, 0.0])

        for _ in range(trials):
            obs = self.env.reset(command=command)
            self.env.set_friction(1.0)

            # Warmup walk
            for step in range(30):
                act = self._get_action(obs, step * 0.02, command)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    break

            # Hit the ice!
            self.env.set_friction(ice_friction)
            survived = True
            init_yaw = self.env.data.qpos[6]  # approximate heading

            for step in range(120):
                t_sec = (30 + step) * 0.02
                act = self._get_action(obs, t_sec, command)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

            if survived:
                survived_count += 1
                final_yaw = self.env.data.qpos[6]
                yaw_drift_deg = abs(final_yaw - init_yaw) * (180.0 / np.pi)
                yaw_drifts.append(yaw_drift_deg)

        # Restore friction
        self.env.set_friction(1.0)
        return {
            "ice_survival_rate": (survived_count / trials) * 100.0,
            "avg_yaw_drift_deg": float(np.mean(yaw_drifts)) if yaw_drifts else 90.0,
        }

    # =========================================================================
    # Test 3: Dynamic Agility & Rapid Velocity Reversal
    # =========================================================================
    def test_velocity_reversal(self, trials=5):
        """Command +0.25 m/s forward, then sharp reversal to -0.20 m/s with yaw spin."""
        survived_count = 0
        vx_errors = []
        wz_errors = []

        for _ in range(trials):
            # Phase 1: Forward
            cmd1 = np.array([0.25, 0.0, 0.0])
            obs = self.env.reset(command=cmd1)

            survived = True
            for step in range(50):
                act = self._get_action(obs, step * 0.02, cmd1)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

            if not survived:
                continue

            # Phase 2: Instant Reversal & Spin
            cmd2 = np.array([-0.20, 0.0, 0.6])
            self.env.command = cmd2.copy()

            for step in range(80):
                t_sec = (50 + step) * 0.02
                act = self._get_action(obs, t_sec, cmd2)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

                # Collect tracking errors in base frame
                r_mat = self.env.data.xmat[self.env.base_id].reshape(3, 3)
                v_base = r_mat.T @ self.env.data.qvel[:3]
                w_base = r_mat.T @ self.env.data.qvel[3:6]

                vx_errors.append((v_base[0] - cmd2[0]) ** 2)
                wz_errors.append((w_base[2] - cmd2[2]) ** 2)

            if survived:
                survived_count += 1

        rmse_vx = float(np.sqrt(np.mean(vx_errors))) if vx_errors else 1.0
        rmse_wz = float(np.sqrt(np.mean(wz_errors))) if wz_errors else 2.0
        return {
            "reversal_survival_rate": (survived_count / trials) * 100.0,
            "rmse_vx": rmse_vx,
            "rmse_wz": rmse_wz,
        }

    # =========================================================================
    # Test 4: Drop Impact & Tilt Recovery
    # =========================================================================
    def test_drop_and_tilt(self, drop_height=0.05, tilt_deg=10.0, trials=6):
        """Drop robot from elevation with initial pitch/roll tilt."""
        survived_count = 0
        angles = [
            (np.radians(tilt_deg), 0.0),
            (-np.radians(tilt_deg), 0.0),
            (0.0, np.radians(tilt_deg)),
            (0.0, -np.radians(tilt_deg)),
            (np.radians(tilt_deg * 0.7), np.radians(tilt_deg * 0.7)),
            (-np.radians(tilt_deg * 0.7), -np.radians(tilt_deg * 0.7)),
        ]

        command = np.array([0.15, 0.0, 0.0])

        for tilt in angles[:trials]:
            obs = self.env.reset(command=command, height_offset=drop_height, tilt_angle=tilt)
            survived = True

            for step in range(80):
                act = self._get_action(obs, step * 0.02, command)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

            if survived:
                survived_count += 1

        return {
            "drop_survival_rate": (survived_count / len(angles[:trials])) * 100.0,
        }

    # =========================================================================
    # Test 5: Continuous Slope & Ramp Incline Traversal
    # =========================================================================
    def test_slope_traversal(self, slopes=(8.0, -8.0, 12.0, -12.0)):
        """Traverse continuous uphill and downhill slope inclines."""
        orig_grav = self.env.model.opt.gravity.copy()
        survived_count = 0
        command = np.array([0.16, 0.0, 0.0])

        for slope_deg in slopes:
            rad = np.radians(slope_deg)
            # Tilt gravity vector to simulate continuous slope
            self.env.model.opt.gravity[:] = [-9.81 * np.sin(rad), 0.0, -9.81 * np.cos(rad)]
            obs = self.env.reset(command=command)
            survived = True

            for step in range(80):
                act = self._get_action(obs, step * 0.02, command)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

            if survived:
                survived_count += 1

        # Restore nominal gravity
        self.env.model.opt.gravity[:] = orig_grav
        return {
            "slope_survival_rate": (survived_count / len(slopes)) * 100.0,
            "slopes_tested": list(slopes),
        }

    # =========================================================================
    # Test 6: Rough Terrain Micro-Bumps & Obstacle Hurdle Crossings
    # =========================================================================
    def test_rough_terrain(self, trials=6, jitter_range=0.010, obstacle_height=0.014):
        """Walk over dynamic vertical micro-bumps, non-flat cracks, and obstacle hurdles."""
        survived_count = 0
        command = np.array([0.18, 0.0, 0.0])

        for _ in range(trials):
            obs = self.env.reset(command=command)
            survived = True

            for step in range(80):
                # Every 8 steps, inject ground vertical perturbation
                if step > 10 and step % 8 == 0:
                    self.env.data.qpos[2] += float(np.random.uniform(-jitter_range, jitter_range))

                # Obstacle hurdle encounters at steps 25 and 55
                if step in (25, 55):
                    l_z = self.env.data.xpos[self.env.sole_ids[0]][2]
                    r_z = self.env.data.xpos[self.env.sole_ids[1]][2]
                    swing_z = max(l_z, r_z)
                    if swing_z < obstacle_height:
                        # Stub toe on obstacle
                        self.env.data.xfrc_applied[self.env.base_id, 0] -= float(np.random.uniform(1.0, 2.0))

                act = self._get_action(obs, step * 0.02, command)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

            if survived:
                survived_count += 1

        return {
            "rough_survival_rate": (survived_count / trials) * 100.0,
        }

    # =========================================================================
    # Test 7: Base Payload Mass Perturbation
    # =========================================================================
    def test_payload_capacity(self, payloads=(0.12, 0.20, 0.28)):
        """Walk carrying extra payload mass on base link (up to ~50% robot body mass)."""
        orig_mass = float(self.env.model.body_mass[self.env.base_id])
        survived_count = 0
        max_survived_payload = 0.0
        command = np.array([0.15, 0.0, 0.0])

        for p in payloads:
            self.env.model.body_mass[self.env.base_id] = orig_mass + p
            obs = self.env.reset(command=command)
            survived = True

            for step in range(80):
                act = self._get_action(obs, step * 0.02, command)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

            if survived:
                survived_count += 1
                max_survived_payload = p

        # Restore original base mass
        self.env.model.body_mass[self.env.base_id] = orig_mass
        return {
            "payload_survival_rate": (survived_count / len(payloads)) * 100.0,
            "max_payload_kg": max_survived_payload,
        }

    # =========================================================================
    # Test 8: Actuator Torque Saturation / Battery Voltage Sag
    # =========================================================================
    def test_torque_saturation(self, sag_ratios=(0.75, 0.60)):
        """Operate under actuator torque saturation simulating battery voltage sag."""
        orig_fr = self.env.model.actuator_forcerange.copy()
        survived_count = 0
        command = np.array([0.16, 0.0, 0.0])

        for sag in sag_ratios:
            self.env.model.actuator_forcerange[:] = orig_fr * sag
            obs = self.env.reset(command=command)
            survived = True

            for step in range(80):
                act = self._get_action(obs, step * 0.02, command)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

            if survived:
                survived_count += 1

        # Restore original torque limits
        self.env.model.actuator_forcerange[:] = orig_fr
        return {
            "torque_survival_rate": (survived_count / len(sag_ratios)) * 100.0,
        }

    # =========================================================================
    # Test 9: Proprioceptive Sensor Noise & 1-Step Latency Lag
    # =========================================================================
    def test_sensor_noise_lag(self, trials=6, noise_std=0.04):
        """Walk under Gaussian encoder noise and 1-step observation lag."""
        survived_count = 0
        command = np.array([0.18, 0.0, 0.0])

        for _ in range(trials):
            obs = self.env.reset(command=command)
            prev_obs = obs.copy()
            survived = True

            for step in range(80):
                # Apply 1-step delayed observation with Gaussian sensor noise
                noisy_lag_obs = prev_obs + np.random.normal(0.0, noise_std, size=obs.shape).astype(np.float32)
                prev_obs = obs.copy()
                act = self._get_action(noisy_lag_obs, step * 0.02, command)
                obs, _, term, _, _ = self.env.step(act)
                if term:
                    survived = False
                    break

            if survived:
                survived_count += 1

        return {
            "sensor_survival_rate": (survived_count / trials) * 100.0,
        }

    # =========================================================================
    # Test 10: Latency & Compute FLOPs Benchmark on Apple Silicon M4
    # =========================================================================
    def test_compute_latency(self, iterations=1000):
        """Benchmark single-inference latency in microseconds and compute headroom."""
        if self.is_baseline:
            # Measure IK computation
            t_starts = []
            for _ in range(iterations):
                t0 = time.perf_counter()
                _ = gait.leg_ik(0.02, gait.H0)
                t_starts.append((time.perf_counter() - t0) * 1e6)
            lat = np.array(t_starts)
        else:
            dummy = np.zeros((1, self.env.obs_dim), dtype=np.float32)
            # Warmup
            for _ in range(50):
                _ = self.session.run(None, {self.input_name: dummy})

            times = []
            for _ in range(iterations):
                t0 = time.perf_counter()
                _ = self.session.run(None, {self.input_name: dummy})
                times.append((time.perf_counter() - t0) * 1e6)
            lat = np.array(times)

        p50 = float(np.percentile(lat, 50))
        p95 = float(np.percentile(lat, 95))
        p99 = float(np.percentile(lat, 99))
        margin_pct = ((20000.0 - p99) / 20000.0) * 100.0
        latency_score = 100.0 if p99 <= 20000.0 else max(0.0, 100.0 * (40000.0 - p99) / 20000.0)

        return {
            "p50_us": p50,
            "p95_us": p95,
            "p99_us": p99,
            "headroom_pct": margin_pct,
            "latency_score": latency_score,
            "params": self.param_count,
            "size_kb": self.file_size_kb,
        }

    def run_full_suite(self):
        """Execute all 10 benchmark tests and return composite report."""
        print(f"\n{'='*75}")
        print(f"  RUNNING 10-CRITERIA VIOLENT BENCHMARK SUITE: {self.model_name}")
        print(f"{'='*75}")

        print("[1/10] Evaluating Omnidirectional Violent Kick Disturbance...")
        kick_res = self.test_violent_kick_resistance()
        print(f"       Kick Survival: {kick_res['survival_rate']:.1f}% | "
              f"Max Impulse: {kick_res['max_survived_impulse']} Ns")

        print("[2/10] Evaluating Extreme Friction Drop (Ice Shock)...")
        ice_res = self.test_ice_shock()
        print(f"       Ice Survival: {ice_res['ice_survival_rate']:.1f}% | "
              f"Yaw Drift: {ice_res['avg_yaw_drift_deg']:.1f}°")

        print("[3/10] Evaluating Dynamic Velocity Reversal & Agility...")
        rev_res = self.test_velocity_reversal()
        print(f"       Reversal Survival: {rev_res['reversal_survival_rate']:.1f}% | "
              f"RMSE vx: {rev_res['rmse_vx']:.3f} m/s")

        print("[4/10] Evaluating Drop Impact & Tilt Recovery...")
        drop_res = self.test_drop_and_tilt()
        print(f"       Drop Survival: {drop_res['drop_survival_rate']:.1f}%")

        print("[5/10] Evaluating Continuous Slope & Ramp Incline Traversal...")
        slope_res = self.test_slope_traversal()
        print(f"       Slope Survival: {slope_res['slope_survival_rate']:.1f}%")

        print("[6/10] Evaluating Rough Terrain Micro-Bumps & Ground Jitter...")
        rough_res = self.test_rough_terrain()
        print(f"       Rough Terrain Survival: {rough_res['rough_survival_rate']:.1f}%")

        print("[7/10] Evaluating Base Payload Mass Perturbation...")
        payload_res = self.test_payload_capacity()
        print(f"       Payload Survival: {payload_res['payload_survival_rate']:.1f}% | "
              f"Max Payload: +{payload_res['max_payload_kg']:.2f} kg")

        print("[8/10] Evaluating Actuator Torque Saturation / Voltage Sag...")
        torque_res = self.test_torque_saturation()
        print(f"       Torque Sag Survival: {torque_res['torque_survival_rate']:.1f}%")

        print("[9/10] Evaluating Proprioceptive Sensor Noise & Observation Lag...")
        sensor_res = self.test_sensor_noise_lag()
        print(f"       Sensor Noise Survival: {sensor_res['sensor_survival_rate']:.1f}%")

        print("[10/10] Profiling Hardware Inference Latency on Apple Silicon M4...")
        lat_res = self.test_compute_latency()
        print(f"        Latency P50: {lat_res['p50_us']:.1f} μs | P99: {lat_res['p99_us']:.1f} μs | "
              f"Params: {lat_res['params']:,}")

        # Composite Robustness Score across all 10 criteria (0 - 100)
        composite_score = (
            0.15 * kick_res["survival_rate"] +
            0.10 * ice_res["ice_survival_rate"] +
            0.10 * rev_res["reversal_survival_rate"] +
            0.10 * drop_res["drop_survival_rate"] +
            0.10 * slope_res["slope_survival_rate"] +
            0.10 * rough_res["rough_survival_rate"] +
            0.10 * payload_res["payload_survival_rate"] +
            0.10 * torque_res["torque_survival_rate"] +
            0.10 * sensor_res["sensor_survival_rate"] +
            0.05 * lat_res["latency_score"]
        )
        score_rounded = round(composite_score, 1)

        # Update registry metadata
        try:
            reg = reg_mod.load_registry()
            for ver in reg.get("models", {}):
                if ver == self.model_target or ver in self.model_name:
                    reg["models"][ver]["benchmark_score"] = score_rounded
                    reg_mod.save_registry(reg)
                    break
        except Exception:
            pass

        return {
            "model_name": self.model_name,
            "composite_score": score_rounded,
            "kick": kick_res,
            "ice": ice_res,
            "reversal": rev_res,
            "drop": drop_res,
            "slope": slope_res,
            "rough": rough_res,
            "payload": payload_res,
            "torque": torque_res,
            "sensor": sensor_res,
            "latency": lat_res,
        }


def print_leaderboard(reports):
    """Print markdown formatted 10-criteria benchmark leaderboard."""
    print("\n" + "#" * 105)
    print("                              HUMANOID 10-CRITERIA BENCHMARK LEADERBOARD")
    print("#" * 105 + "\n")

    header = (
        "| Model Version / Architecture | Score (/100) | Kick Surv | Ice Surv | Rev Surv | Drop Surv "
        "| Slope Surv | Rough Surv | Payload Surv | Torque Surv | Sensor Surv | Latency P99 | Params |"
    )
    sep = (
        "|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|"
    )
    print(header)
    print(sep)

    for r in sorted(reports, key=lambda x: x["composite_score"], reverse=True):
        name = r["model_name"]
        score = r["composite_score"]
        k_surv = f"{r['kick']['survival_rate']:.0f}%"
        ice_surv = f"{r['ice']['ice_survival_rate']:.0f}%"
        rev_surv = f"{r['reversal']['reversal_survival_rate']:.0f}%"
        drop_surv = f"{r['drop']['drop_survival_rate']:.0f}%"
        slope_surv = f"{r.get('slope', {}).get('slope_survival_rate', 0.0):.0f}%"
        rough_surv = f"{r.get('rough', {}).get('rough_survival_rate', 0.0):.0f}%"
        pay_surv = f"{r.get('payload', {}).get('payload_survival_rate', 0.0):.0f}%"
        torq_surv = f"{r.get('torque', {}).get('torque_survival_rate', 0.0):.0f}%"
        sens_surv = f"{r.get('sensor', {}).get('sensor_survival_rate', 0.0):.0f}%"
        p99 = f"{r['latency']['p99_us']:.0f} μs"
        params = f"{r['latency']['params']:,}" if r['latency']['params'] > 0 else "N/A (IK)"

        row = (
            f"| **{name}** | **{score}** | {k_surv} | {ice_surv} | {rev_surv} | {drop_surv} | "
            f"{slope_surv} | {rough_surv} | {pay_surv} | {torq_surv} | {sens_surv} | {p99} | {params} |"
        )
        print(row)
    print("\n")


def main():
    parser = argparse.ArgumentParser(description="Evaluate Humanoid Locomotion Policies against Disturbances")
    parser.add_argument("--model", type=str, default="active", help="Model to evaluate ('active', 'baseline', or path)")
    parser.add_argument("--compare", nargs="+", help="List of models or versions to compare head-to-head")
    args = parser.parse_args()

    targets = args.compare if args.compare else [args.model]
    reports = []
    for tgt in targets:
        evaluator = BenchmarkEvaluator(model_target=tgt)
        report = evaluator.run_full_suite()
        reports.append(report)

    if len(reports) > 1:
        print_leaderboard(reports)


if __name__ == "__main__":
    main()
