#!/usr/bin/env python3
"""Autonomous Continuous Humanoid Training Loop with 20-Motion HLI-20 Evaluation.

Features:
1. Resumes training from the current best checkpoint (.pt) without losing learned weights.
2. Evaluates across 20 Canonical Motions (Sprint, Slopes, Bow, Squat, Getup, Jump, Push Recovery).
3. Measures 20 Biomechanical Human-Likeness Parameters (HLI-20 composite 100-point index).
4. Supports nonstop continuous evolution (--nonstop).
5. Automatically renders Before vs After side-by-side comparison GIFs when a new champion is discovered.

Usage:
    # Run Stage 2 resumed from current champion
    python3 src/humanoid_control/scripts/autonomous_locomotion_loop.py --stages 2

    # Nonstop continuous training loop
    python3 src/humanoid_control/scripts/autonomous_locomotion_loop.py --nonstop --target-hli 85.0
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import cv2
import imageio.v3 as iio
import mujoco
import numpy as np

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE))
import evolution_tracker as trk_mod  # noqa: E402
import humanoid_benchmark_20motions as b20  # noqa: E402
import mico_rl_env as env_mod  # noqa: E402
import model_registry as reg_mod  # noqa: E402

EVO_DIR = REPO / "docs" / "images" / "evolution"
EVO_DIR.mkdir(parents=True, exist_ok=True)


CURRICULUM = [
    {
        "stage": 1,
        "tag": "v15.0_titan_transformer_evolution",
        "arch": "titan_transformer",
        "iterations": 160,
        "target_speed": 0.35,
        "perturb_prob": 0.035,
        "lr": "8e-5",
        "clip_coef": "0.15",
        "resume_from": "v9.0_titan_transformer",
        "desc": "Titan Multi-Stream Residual Transformer Policy (MHSA 4-Head + Decoupled Specialty Heads)",
    },
]


def render_side_by_side_comparison(model_a_path, model_b_path, label_a, label_b,
                                   score_a, score_b, output_gif_path,
                                   duration_steps=120, target_speed=0.35):
    """Render a side-by-side comparison GIF between two models."""

    # Capture frames for both
    def capture_run(m_path):
        import onnxruntime as ort
        sess = ort.InferenceSession(str(m_path))
        inp_dim = sess.get_inputs()[0].shape[1]
        input_name = sess.get_inputs()[0].name
        output_name = sess.get_outputs()[0].name
        use_phase = (inp_dim in (47, 49, 51, 60, 84, 1024))
        use_perception = (inp_dim in (60, 84, 1024))

        env = env_mod.MicoEnv(
            use_residual=True,
            use_phase=use_phase,
            use_perception=use_perception,
            straight_line=True,
            target_speed=target_speed,
        )
        obs = env.reset(command=np.array([target_speed, 0.0, 0.0], dtype=np.float64))

        renderer = mujoco.Renderer(env.model, width=360, height=360)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        camera.trackbodyid = env.base_id
        camera.distance = 0.95
        camera.elevation = -12.0
        camera.azimuth = 145.0

        frames = []
        for s in range(duration_steps):
            act = sess.run([output_name], {input_name: obs[None, :].astype(np.float32)})[0][0]
            obs, r, term, trunc, info = env.step(act)
            if s % 2 == 0:
                renderer.update_scene(env.data, camera=camera)
                frames.append(renderer.render().copy())
            if term:
                break
        return frames

    print(f"[*] Capturing Model A: {label_a}...")
    frames_a = capture_run(model_a_path)
    print(f"[*] Capturing Model B: {label_b}...")
    frames_b = capture_run(model_b_path)

    min_len = min(len(frames_a), len(frames_b))
    if min_len == 0:
        print("[!] Warning: Could not capture frames for comparison render.")
        return None

    h, w = 360, 360
    combined = []
    for i in range(min_len):
        fa = cv2.resize(frames_a[i], (w, h))
        fb = cv2.resize(frames_b[i], (w, h))

        hdr_a = np.full((40, w, 3), (25, 25, 25), dtype=np.uint8)
        text_a = f"{label_a} (HLI: {score_a:.1f})"
        cv2.putText(hdr_a, text_a, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 160, 255), 1, cv2.LINE_AA)

        hdr_b = np.full((40, w, 3), (25, 25, 25), dtype=np.uint8)
        text_b = f"{label_b} (HLI: {score_b:.1f})"
        cv2.putText(hdr_b, text_b, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 150), 1, cv2.LINE_AA)

        col_a = np.vstack([hdr_a, fa])
        col_b = np.vstack([hdr_b, fb])
        row = np.hstack([col_a, col_b])
        combined.append(row)

    temp_gif = output_gif_path.with_suffix(".tmp.gif")
    iio.imwrite(temp_gif, combined, duration=50, loop=0)

    cmd = [
        "ffmpeg", "-y", "-i", str(temp_gif),
        "-vf", "fps=18,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
        str(output_gif_path)
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if temp_gif.exists():
        temp_gif.unlink()

    snapshot_png = output_gif_path.with_suffix(".png")
    mid_idx = min_len // 2
    cv2.imwrite(str(snapshot_png), cv2.cvtColor(combined[mid_idx], cv2.COLOR_RGB2BGR))
    print(f"[✓] Comparison GIF saved to: {output_gif_path} ({output_gif_path.stat().st_size / 1e6:.2f} MB)")
    return output_gif_path


def train_stage(cfg):
    """Execute training for one curriculum stage with checkpoint resuming."""
    tag = cfg["tag"]
    print("\n" + "=" * 80)
    print(f"🏋️ [STAGE {cfg['stage']}] CONTINUOUS TRAINING: {tag.upper()}")
    print(f"   Target Speed: {cfg['target_speed']} m/s | Architecture: {cfg['arch']}")
    print(f"   Iterations:   {cfg['iterations']} | Disturbance Prob: {cfg['perturb_prob']}")

    # Determine checkpoint to resume from (Prioritize continuous chain)
    resume_path = None
    if cfg.get("resume_override") and Path(cfg["resume_override"]).exists():
        resume_path = Path(cfg["resume_override"])
    elif cfg.get("resume_from"):
        candidate = REPO / "src" / "humanoid_control" / "models" / f"mico_policy_{cfg['resume_from']}.pt"
        if candidate.exists():
            resume_path = candidate
    if resume_path is None:
        if cfg.get("arch") == "titan_transformer":
            active_pt = REPO / "src" / "humanoid_control" / "models" / "mico_policy_v9.0_titan_transformer.pt"
        else:
            active_pt = REPO / "src" / "humanoid_control" / "models" / "mico_policy_v12.0_adaptive_pitch_sprint.pt"
        if active_pt.exists():
            resume_path = active_pt

    if resume_path:
        print(f"   Resume Checkpoint: {resume_path.name}")
    print("   Trajectory:   Locked Straight-Track (y=0, yaw=0) & 20-Motion Robustness")
    print("=" * 80)

    cmd = [
        sys.executable,
        str(HERE / "train_mico_ppo.py"),
        "--arch", cfg["arch"],
        "--iterations", str(cfg["iterations"]),
        "--num-envs", "16",
        "--minibatch-size", "512",
        "--residual",
        "--use-phase",
        "--straight-line",
        "--target-speed", str(cfg["target_speed"]),
        "--lr", str(cfg.get("lr", "7e-5")),
        "--clip-coef", str(cfg.get("clip_coef", "0.15")),
        "--domain-rand",
        "--perturb-prob", str(cfg["perturb_prob"]),
        "--version-tag", tag,
        "--log-interval", "40",
    ]
    if resume_path:
        cmd.extend(["--resume-checkpoint", str(resume_path)])

    t0 = time.time()
    subprocess.run(cmd, check=True)
    dt = time.time() - t0
    print(f"[✓] Stage {cfg['stage']} training finished in {dt:.1f}s!")
    return dt


def main():
    parser = argparse.ArgumentParser(description="Autonomous Continuous Humanoid Training Loop")
    parser.add_argument("--stages", type=str, default="1", help="Comma-separated stage indices to run")
    parser.add_argument("--nonstop", action="store_true", help="Run indefinitely in a nonstop loop")
    parser.add_argument("--target-hli", type=float, default=85.0, help="Target HLI-20 score out of 100")
    args = parser.parse_args()

    selected_stages = [int(s.strip()) for s in args.stages.split(",") if s.strip()]

    print("\n" + "#" * 80)
    print("     SYNAPSE HUMANOID: NONSTOP HUMAN-LIKENESS TRAINING & BENCHMARK LOOP")
    print("     Target: 20 Canonical Motions · 20 Biomechanical Parameters (HLI-20)")
    print(f"     Target Composite HLI Score: {args.target_hli} / 100.0")
    print("#" * 80 + "\n")

    evaluator = b20.HumanLikeness20Evaluator()
    tracker = trk_mod.EvolutionTracker()

    # Determine baseline champion
    current_champion_path = REPO / "src" / "humanoid_control" / "models" / "mico_policy.onnx"
    champion_tag = tracker.data.get("champion_tag", "baseline_active")
    champion_score = float(tracker.data.get("champion_score", 0.0))

    if current_champion_path.exists() and champion_score == 0.0:
        print("[*] Running full 20-Motion Benchmark on initial champion...")
        champion_score, p_scores, telem, _ = evaluator.evaluate_20_parameters(current_champion_path)
        print(f"[*] Initial Champion HLI-20 Score: {champion_score:.1f} / 100.0")
        print(f"    Roll: {telem['P01_roll_sway_deg']:.1f}° | Pitch: {telem['P02_pitch_posture_deg']:.1f}° "
              f"| Y-Drift: {telem['P04_lateral_drift_cm']:.1f} cm | Zero-Fall: {telem['P16_zero_fall_rate_pct']:.0f}%")

    # Continuous Learning Chain: determine starting revision from history
    first_cfg = next((c for c in CURRICULUM if c["stage"] in selected_stages), CURRICULUM[0])
    base_tag_prefix = first_cfg["tag"]
    existing_revs = [
        int(h["tag"].split("_rev")[-1]) for h in tracker.data.get("history", [])
        if "_rev" in h.get("tag", "") and base_tag_prefix in h.get("tag", "")
    ]
    loop_idx = max(existing_revs) if existing_revs else 0
    best_arch_scores = {}

    while True:
        loop_idx += 1
        for stage_idx in selected_stages:
            cfg = next((c for c in CURRICULUM if c["stage"] == stage_idx), None)
            if not cfg:
                continue

            base_tag = cfg["tag"]
            tag = f"{base_tag}_rev{loop_idx}"
            cfg = dict(cfg)
            cfg["tag"] = tag

            # Life-long Continuous Learning Chain: always inherit from latest/best checkpoint
            chain_ckpt = REPO / "src" / "humanoid_control" / "models" / f"mico_policy_{base_tag}_chain_latest.pt"
            best_arch_ckpt = REPO / "src" / "humanoid_control" / "models" / f"mico_policy_{base_tag}_chain_best.pt"
            if chain_ckpt.exists():
                cfg["resume_override"] = chain_ckpt
            elif best_arch_ckpt.exists():
                cfg["resume_override"] = best_arch_ckpt

            model_path = REPO / "src" / "humanoid_control" / "models" / f"mico_policy_{tag}.onnx"

            prev_path = current_champion_path
            prev_tag = champion_tag
            prev_score = champion_score

            # 1. Train resumed from checkpoint
            dt = train_stage(cfg)

            # 2. Evaluate across all 20 motions and 20 parameters
            print(f"\n[🔬] Evaluating candidate '{tag}' across 20-Motion Suite...")
            score, p_scores, telem, motion_res = evaluator.evaluate_20_parameters(model_path)

            passed_count = sum(1 for _, r in motion_res if r["survived"])
            print(f"[✓] Candidate '{tag}' HLI-20 Score: {score:.1f} / 100.0 (Passed: {passed_count}/20 motions)")
            print(f"    • Posture: Roll {telem['P01_roll_sway_deg']:.1f}°, Pitch {telem['P02_pitch_posture_deg']:.1f}°")
            print(f"    • Track:   Y-Drift {telem['P04_lateral_drift_cm']:.1f} cm, "
                  f"Bobbing {telem['P06_pelvis_bobbing_mm']:.1f} mm")
            print(f"    • Safety:  Zero-Fall Rate {telem['P16_zero_fall_rate_pct']:.0f}%")

            # 3. Always maintain and chain the latest checkpoint
            candidate_pt = model_path.with_suffix(".pt")
            if candidate_pt.exists():
                shutil.copyfile(candidate_pt, chain_ckpt)
                print(f"[🔗 CHAIN] Inherited & updated continuous training checkpoint: {chain_ckpt.name}")

                if base_tag not in best_arch_scores or score > best_arch_scores[base_tag]:
                    best_arch_scores[base_tag] = score
                    shutil.copyfile(candidate_pt, best_arch_ckpt)
                    print(f"[⭐ ARCH BEST] Saved architecture best checkpoint ({score:.2f} pts): {best_arch_ckpt.name}")

            is_new_champ = bool(score > champion_score)

            # 4. Record in Evolution Tracker & check 10-Gen Stagnation Sentinel
            audit_res = tracker.record_generation(
                tag=tag,
                wall_time_s=dt,
                iterations=cfg["iterations"],
                steps=cfg["iterations"] * 1024,
                hli_score=score,
                passed_motions=passed_count,
                is_new_champion=is_new_champ,
                evaluator=evaluator,
                champion_path=current_champion_path,
            )

            if audit_res:
                print("\n" + "*" * 80)
                gen_idx = tracker.data['total_generations']
                print(f"🤖 [AUTONOMOUS ACTION] Stagnation Sentinel injected 5 Micro-Fixes for Gen #{gen_idx}:")
                print(f"   1. Reward Shaping: Yaw penalty set to {audit_res['p_straight_yaw']}")
                print(f"   2. Action Smoothing: Filter alpha {audit_res['action_filter']}, "
                      f"rate penalty {audit_res['p_action_rate']}")
                print(f"   3. Domain Randomization: Perturbation {audit_res['perturb_range'][0]}-"
                      f"{audit_res['perturb_range'][1]} N")
                print(f"   4. Kinematics: Sprint pitch lean set to {audit_res['pitch_lean_gain']} rad")
                print(f"   5. Biomechanics: Swing clearance target set to "
                      f"{audit_res['clearance_target'] * 1000:.0f} mm")
                print(f"   📄 Stagnation Report saved: {audit_res['report_path']}")
                print("*" * 80 + "\n")

            # 5. Promote & Render if improved
            if is_new_champ:
                print(f"\n🎉 [NEW CHAMPION!] '{tag}' ({score:.1f}) surpassed previous champion ({champion_score:.1f})!")
                cmp_gif = EVO_DIR / f"comparison_stage_{stage_idx}_{tag}.gif"
                latest_gif = EVO_DIR / "comparison_latest.gif"

                render_side_by_side_comparison(
                    model_a_path=prev_path,
                    model_b_path=model_path,
                    label_a=prev_tag,
                    label_b=tag,
                    score_a=prev_score,
                    score_b=score,
                    output_gif_path=cmp_gif,
                    duration_steps=120,
                    target_speed=cfg["target_speed"],
                )

                shutil.copyfile(cmp_gif, latest_gif)
                shutil.copyfile(cmp_gif.with_suffix(".png"), latest_gif.with_suffix(".png"))

                reg_mod.activate_model(tag)
                current_champion_path = model_path
                champion_tag = tag
                champion_score = score
            else:
                print(f"[-] Candidate '{tag}' ({score:.1f}) did not beat Champion "
                      f"'{champion_tag}' ({champion_score:.1f}).")
                # Clean up intermediate candidate files (preserving chain_latest & chain_best)
                if candidate_pt.exists() and candidate_pt not in (chain_ckpt, best_arch_ckpt, current_champion_path):
                    candidate_pt.unlink()
                if model_path.exists() and model_path != current_champion_path:
                    model_path.unlink()

            if champion_score >= args.target_hli:
                print(f"\n🏆 TARGET HLI-20 SCORE REACHED ({champion_score:.1f} >= {args.target_hli})! Halting.")
                return

        if not args.nonstop:
            break

    print("\n" + "=" * 80)
    print("🏁 Autonomous Training Session Finished!")
    print(f"   Final Champion:  {champion_tag} (HLI-20: {champion_score:.1f} / 100.0)")
    print(f"   Visual Archive:  {EVO_DIR}")
    print("=" * 80 + "\n")


if __name__ == "__main__":
    main()
