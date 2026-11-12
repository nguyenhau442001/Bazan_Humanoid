#!/usr/bin/env python3
"""Interactive 3D Simulation Visualizer for Mico Humanoid.

Launches a native MuJoCo 3D viewer window to visually observe the humanoid robot walking
stably and continuously, and interactively trigger disturbances on demand:
- [V] / [C] Toggle First-Person View (FPV: head camera) vs Third-Person Orbit View
- [K] Trigger kick disturbance (Light: 3N, Medium: 8N, Violent: 18N)
- [1] Set kick intensity to Light (3 N)
- [2] Set kick intensity to Medium (8 N)
- [3] Set kick intensity to Violent (18 N)
- [I] Trigger slippery ice shock (friction drops to 0.15)
- [R] Reverse velocity & spin (+0.25 m/s <-> -0.20 m/s)
- [D] Drop shock (drop from 0.06m in air with tilt)
- [O] Instant reset to nominal standing pose
- [+] / [-] Speed up or slow down simulation playback (1x, 1.5x, 2x, 3x)
- [Space] Pause / Resume simulation

Usage:
    # Smooth continuous walking with interactive controls
    python3 src/humanoid_control/scripts/mico_visualizer.py

    # Fast forward (2x real-time speed)
    python3 src/humanoid_control/scripts/mico_visualizer.py --speed 2.0

    # Auto-demo cycle
    python3 src/humanoid_control/scripts/mico_visualizer.py --demo
"""
import argparse
import math
import os
from pathlib import Path
import shutil
import sys
import time

# Auto re-exec under mjpython on macOS if launched with normal python3
if sys.platform == "darwin" and os.environ.get("MJPYTHON_EXEC") != "1":
    os.environ["MJPYTHON_EXEC"] = "1"
    mjpy = shutil.which("mjpython")
    if mjpy:
        os.execv(mjpy, [mjpy] + sys.argv)

import cv2  # noqa: E402
import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
import numpy as np  # noqa: E402
import onnxruntime as ort  # noqa: E402

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE))
import mico_gait as gait  # noqa: E402
import mico_rl_env as env_mod  # noqa: E402
import model_registry as reg_mod  # noqa: E402


def get_screen_resolution():
    """Detect main display resolution across macOS and Linux."""
    try:
        import AppKit
        frame = AppKit.NSScreen.mainScreen().frame()
        return int(frame.size.width), int(frame.size.height)
    except Exception:
        pass
    try:
        import tkinter
        root = tkinter.Tk()
        root.withdraw()
        return root.winfo_screenwidth(), root.winfo_screenheight()
    except Exception:
        pass
    return 1440, 900


def resolve_model(target):
    if target == "baseline" or target == "gait":
        return None, "Baseline Gait (mico_gait)"

    cand = Path(target)
    if cand.exists():
        return cand.resolve(), cand.stem

    registry = reg_mod.load_registry()
    if target in registry.get("models", {}):
        rel = registry["models"][target]["relative_path"]
        return (reg_mod.MODELS_DIR / rel).resolve(), target

    if target == "active":
        if reg_mod.ACTIVE_SYMLINK.exists():
            return reg_mod.ACTIVE_SYMLINK.resolve(), "Active Policy"
        def_path = reg_mod.MODELS_DIR / "mico_policy.onnx"
        if def_path.exists():
            return def_path.resolve(), "Active Policy"

    # Search for substring match in models directory
    matches = list(reg_mod.MODELS_DIR.glob(f"*{target}*.onnx"))
    if matches:
        # Prefer latest or apex/titan
        matches.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        return matches[0].resolve(), matches[0].stem

    raise FileNotFoundError(f"Cannot resolve policy: '{target}'")


TRAJECTORY_STAGES = [
    {
        "name": "1. Gentle Forward Walk Initializer",
        "cmd": np.array([0.12, 0.0, 0.0]),
        "duration_s": 3.0,
        "desc": "Establishes initial gait cadence and stabilizes Center of Mass",
    },
    {
        "name": "2. Cruise Forward Walk",
        "cmd": np.array([0.22, 0.0, 0.0]),
        "duration_s": 3.5,
        "desc": "Extended stride length and steady cadence at nominal cruise speed",
    },
    {
        "name": "3. Left Arc Turn",
        "cmd": np.array([0.15, 0.0, 0.35]),
        "duration_s": 3.5,
        "desc": "Combined forward locomotion and left pelvis steering with adaptive ankle roll",
    },
    {
        "name": "4. Right Arc Turn",
        "cmd": np.array([0.15, 0.0, -0.35]),
        "duration_s": 3.5,
        "desc": "Smooth curvature steering to the right without jerk or oscillation",
    },
    {
        "name": "5. In-Place Yaw Spin",
        "cmd": np.array([0.00, 0.0, 0.45]),
        "duration_s": 2.5,
        "desc": "Reorients heading angle in-place via micro footstep steps",
    },
    {
        "name": "6. Safe Backward Walk",
        "cmd": np.array([-0.10, 0.0, 0.0]),
        "duration_s": 2.5,
        "desc": "Controlled backward displacement suppressing backward stumble",
    },
    {
        "name": "7. Brake to Nominal Stand",
        "cmd": np.array([0.00, 0.0, 0.0]),
        "duration_s": 2.5,
        "desc": "Returns joints to default standing stance with absolute balance",
    },
]


def main():
    parser = argparse.ArgumentParser(description="Interactive 3D Visualizer for Mico Humanoid")
    parser.add_argument("--model", type=str, default="v8.1_apex_titan",
                        help="Model version, path, or 'v8.1_apex_titan' / 'v7.25_evolved' / 'gait'")
    parser.add_argument("--speed", type=float, default=1.0, help="Simulation playback speed multiplier (default: 1.0x)")
    parser.add_argument("--trajectory", action="store_true", default=True,
                        help="Automatically sequence through small trajectories (default: True)")
    parser.add_argument("--demo", action="store_true", default=False,
                        help="Enable automated disturbance test cycle")
    parser.add_argument("--fpv", action="store_true", default=False,
                        help="Launch in First-Person View from head_camera (toggle: V key)")
    parser.add_argument("--pip", "--cam-window", dest="pip", action="store_true", default=True,
                        help="Display camera view at bottom-right corner of screen (toggle: P key)")
    parser.add_argument("--no-pip", dest="pip", action="store_false",
                        help="Disable bottom-right camera window")
    args = parser.parse_args()

    model_path, model_label = resolve_model(args.model)
    use_gait = (model_path is None)
    is_residual = False
    use_phase = False
    session = None
    inp_name = None

    use_perception = False
    if not use_gait:
        session = ort.InferenceSession(str(model_path))
        inp_name = session.get_inputs()[0].name
        inp_dim = session.get_inputs()[0].shape[-1]
        use_phase = (inp_dim in (47, 49, 51, 60, 84, 1024))
        use_perception = (inp_dim in (60, 84, 1024))
        # Check registry first for residual metadata
        is_res = False
        try:
            reg = reg_mod.load_registry()
            for k, v in reg.get("models", {}).items():
                if k in model_path.name:
                    is_res = bool(v.get("residual", False))
                    break
        except Exception:
            pass
        if not is_res:
            is_res = any(k in model_path.name for k in (
                "residual", "v2.5", "v3", "v4", "v5", "v6", "v7", "v8", "apex", "titan"
            ))
        is_residual = is_res
        print(
            f"[*] Loaded ONNX policy: {model_path.name} "
            f"(obs_dim={inp_dim}, residual={is_residual}, use_phase={use_phase}, perception={use_perception})"
        )
    else:
        print("[*] Running Stable Dynamic Biped Gait (continuous walking)")

    env = env_mod.MicoEnv(
        decimation=20,
        max_episode_steps=100000,
        use_residual=is_residual,
        use_phase=use_phase,
        use_perception=use_perception,
        action_filter=0.65,
    )

    print("\n" + "=" * 80)
    print("             MICO HUMANOID 3D VISUALIZER & TRAJECTORY CONTROLLER")
    print("=" * 80)
    print("  Active Model : " + f"{model_label} (Residual: {is_residual})")
    print("  Camera View Modes:")
    print("    [P]           : Toggle Bottom-Right Companion Camera Window (OpenCV)")
    print("    [V] / [C]     : Toggle Main 3D View between FPV (head_camera) & Orbit View")
    print("  Interactive Trajectory Controls:")
    print("    [W] / [S]     : Forward (+0.18 m/s) / Backward (-0.10 m/s)")
    print("    [A] / [D]     : Turn Left (+0.35 rad/s) / Turn Right (-0.35 rad/s)")
    print("    [X]           : Instant Brake to Stop (v=0, w=0)")
    print("    [T]           : Toggle 7-Stage Auto-Trajectory Sequence")
    print("  Dynamic Stress Perturbations:")
    print("    [K]           : Pelvis Kick Impulse")
    print("    [1] / [2] / [3] : Kick Intensity: 1=Light (3N), 2=Medium (8N), 3=Violent (18N)")
    print("    [I]           : Ice Slip Shock (Friction = 0.15 for 3s)")
    print("    [D]           : Tilt Drop Impact (Elevation 6cm at 10 deg tilt)")
    print("    [O]           : Instant Reset to Nominal Stance")
    print("    [+] / [-]     : Increase / Decrease simulation speed")
    print("    [Space]       : Pause / Resume simulation")
    print("=" * 80 + "\n")

    current_cmd = np.array([0.12, 0.0, 0.0], dtype=np.float64)
    obs = env.reset(command=current_cmd)

    base_dt = env.decimation * 0.001  # 20 ms
    speed_mult = max(0.5, float(args.speed))
    step_idx = 0
    ice_timer = 0
    kick_force_mag = 5.0
    paused = False

    # Trajectory execution state
    auto_trajectory = args.trajectory
    traj_idx = 0
    stage_time_left = TRAJECTORY_STAGES[0]["duration_s"]
    current_cmd = TRAJECTORY_STAGES[0]["cmd"].copy()
    env.command = current_cmd.copy()

    print(f"\n[🚀 TRAJECTORY STARTED] {TRAJECTORY_STAGES[0]['name']}")
    print(
        f"   Targets: vx={current_cmd[0]:.2f} m/s, wz={current_cmd[2]:.2f} rad/s "
        f"({TRAJECTORY_STAGES[0]['desc']})"
    )

    # Launch passive viewer
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        fpv_mode = [args.fpv]
        cam_names = [env.model.camera(i).name for i in range(env.model.ncam)]
        cam_id = env.model.camera("head_camera").id if "head_camera" in cam_names else -1

        # Companion Bottom-Right Camera Window (OpenCV)
        pip_active = [args.pip and cam_id >= 0]
        r_cam = None
        cam_win_name = "Mico Head Camera (FPV)"
        if cam_id >= 0 and pip_active[0]:
            r_cam = mujoco.Renderer(env.model, 240, 320)
            cv2.namedWindow(cam_win_name, cv2.WINDOW_AUTOSIZE)
            sw, sh = get_screen_resolution()
            cv2.moveWindow(cam_win_name, max(0, sw - 340), max(0, sh - 300))

        def apply_camera_view():
            if fpv_mode[0] and cam_id >= 0:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = cam_id
            else:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                viewer.cam.distance = 0.85
                viewer.cam.elevation = -15.0
                viewer.cam.azimuth = 140.0

        apply_camera_view()

        def key_callback(keycode):
            nonlocal paused, ice_timer, obs, kick_force_mag, speed_mult, auto_trajectory, current_cmd, r_cam
            # P = Toggle Bottom-Right PiP Camera Window
            if keycode in (ord('P'), ord('p')):
                pip_active[0] = not pip_active[0]
                if pip_active[0] and cam_id >= 0:
                    if r_cam is None:
                        r_cam = mujoco.Renderer(env.model, 240, 320)
                    cv2.namedWindow(cam_win_name, cv2.WINDOW_AUTOSIZE)
                    sw, sh = get_screen_resolution()
                    cv2.moveWindow(cam_win_name, max(0, sw - 340), max(0, sh - 300))
                    print("[📷] Bottom-Right Floating Camera Window: ENABLED")
                else:
                    cv2.destroyWindow(cam_win_name)
                    print("[📷] Bottom-Right Floating Camera Window: DISABLED")
            # V / C = Toggle Camera View (FPV / Orbit)
            elif keycode in (ord('V'), ord('v'), ord('C'), ord('c')):
                fpv_mode[0] = not fpv_mode[0]
                apply_camera_view()
                mode_str = "FPV (head_camera)" if fpv_mode[0] else "Third-Person Orbit View"
                print(f"[📷 CAMERA] Switched to {mode_str}")
            # Space
            elif keycode == 32:
                paused = not paused
                print(f"[!] Simulation {'PAUSED' if paused else 'RESUMED'}")
            # W = Forward
            elif keycode in (ord('W'), ord('w')):
                auto_trajectory = False
                current_cmd = np.array([0.20, 0.0, 0.0])
                env.command = current_cmd.copy()
                print("[⌨️ Key W] Command: Forward (+0.20 m/s)")
            # S = Backward
            elif keycode in (ord('S'), ord('s')):
                auto_trajectory = False
                current_cmd = np.array([-0.10, 0.0, 0.0])
                env.command = current_cmd.copy()
                print("[⌨️ Key S] Command: Backward (-0.10 m/s)")
            # A = Left
            elif keycode in (ord('A'), ord('a')):
                auto_trajectory = False
                current_cmd = np.array([0.14, 0.0, 0.35])
                env.command = current_cmd.copy()
                print("[⌨️ Key A] Command: Turn Left (+0.35 rad/s)")
            # D = Right
            elif keycode in (ord('D'), ord('d')):
                auto_trajectory = False
                current_cmd = np.array([0.14, 0.0, -0.35])
                env.command = current_cmd.copy()
                print("[⌨️ Key D] Command: Turn Right (-0.35 rad/s)")
            # X = Stop
            elif keycode in (ord('X'), ord('x')):
                auto_trajectory = False
                current_cmd = np.array([0.0, 0.0, 0.0])
                env.command = current_cmd.copy()
                print("[⌨️ Key X] Command: Brake to Stop")
            # T = Toggle Auto-Trajectory
            elif keycode in (ord('T'), ord('t')):
                auto_trajectory = not auto_trajectory
                print(f"[🎯] Auto-Trajectory Sequence: {'ENABLED (7-stage loop)' if auto_trajectory else 'DISABLED'}")
            # K = Kick
            elif keycode in (ord('K'), ord('k')):
                print(f"[💥] KICK IMPULSE APPLIED! (Force: {kick_force_mag:.1f} N to pelvis)")
                env.apply_perturbation([kick_force_mag, 0.0, 0.0], duration_steps=4)
            # 1, 2, 3 = Kick intensity
            elif keycode == ord('1'):
                kick_force_mag = 3.0
                print("[⚙️] Kick Level: LIGHT (3 N) - Observing balance feedback")
            elif keycode == ord('2'):
                kick_force_mag = 8.0
                print("[⚙️] Kick Level: MEDIUM (8 N) - Dynamic stumble recovery")
            elif keycode == ord('3'):
                kick_force_mag = 18.0
                print("[⚙️] Kick Level: VIOLENT (18 N) - Extreme shock perturbation")
            # I = Ice
            elif keycode in (ord('I'), ord('i')):
                print("[❄️] ICE FRICTION SHOCK! Ground friction dropped to 0.15")
                env.set_friction(0.15)
                ice_timer = 150
            # D = Drop
            elif keycode in (ord('D'), ord('d')):
                print("[🪂] TILT DROP CHALLENGE: Dropped from 0.06m at 10 deg tilt")
                obs = env.reset(command=current_cmd, height_offset=0.06, tilt_angle=(0.15, 0.0))
            # O = Reset
            elif keycode in (ord('O'), ord('o')):
                print("[↺] INSTANT RESET: Robot restored to nominal stance")
                obs = env.reset(command=current_cmd)
            # + / = : Speed up
            elif keycode in (ord('+'), ord('=')):
                speed_mult = min(5.0, speed_mult + 0.5)
                print(f"[⏩] Simulation Speed: {speed_mult:.1f}x")
            # - : Slow down
            elif keycode == ord('-'):
                speed_mult = max(0.5, speed_mult - 0.5)
                print(f"[⏪] Simulation Speed: {speed_mult:.1f}x")

        viewer._key_callback = key_callback

        last_status_print = time.time()

        while viewer.is_running():
            step_start = time.perf_counter()

            if not paused:
                # Handle ice shock duration
                if ice_timer > 0:
                    ice_timer -= 1
                    if ice_timer == 0:
                        env.set_friction(1.0)
                        print("[✓] Ground friction restored to 1.0")

                # Progress through Auto-Trajectory Stages
                if auto_trajectory:
                    stage_time_left -= base_dt
                    if stage_time_left <= 0.0:
                        traj_idx = (traj_idx + 1) % len(TRAJECTORY_STAGES)
                        st = TRAJECTORY_STAGES[traj_idx]
                        stage_time_left = st["duration_s"]
                        current_cmd = st["cmd"].copy()
                        env.command = current_cmd.copy()
                        print("\n" + "-" * 75)
                        print(f"[🚀 SWITCHING TRAJECTORY STAGE] {st['name']}")
                        print(f"   Targets: vx={current_cmd[0]:+.2f} m/s, wz={current_cmd[2]:+.2f} rad/s")
                        print(f"   Description: {st['desc']}")
                        print("-" * 75)

                # Periodic terminal telemetry
                if time.time() - last_status_print >= 2.0:
                    r_mat = env.data.xmat[env.base_id].reshape(3, 3)
                    v_base = r_mat.T @ env.data.qvel[:3]
                    w_base = r_mat.T @ env.data.qvel[3:6]
                    base_z = env.data.xpos[env.base_id][2]
                    print(
                        f"  [Tele] Height: {base_z:.3f}m | "
                        f"V_act: vx={v_base[0]:+.2f}, vy={v_base[1]:+.2f} m/s | "
                        f"Yaw_rate: {w_base[2]:+.2f} rad/s | Cmd: vx={current_cmd[0]:.2f}, wz={current_cmd[2]:.2f}"
                    )
                    last_status_print = time.time()

                # Compute Action
                if use_gait:
                    t_sec = 1.0 + step_idx * base_dt
                    q_dict = gait.walk(t_sec - 1.0)
                    full_target = np.array([q_dict.get(j, 0.0) for j in gait.JOINTS], dtype=np.float64)
                    env.data.ctrl[env.all_act_ids] = full_target

                    # Apply perturbation if active
                    if env.perturb_steps_left > 0:
                        env.data.xfrc_applied[env.base_id, :3] = env.perturb_force
                        env.perturb_steps_left -= 1
                    else:
                        env.data.xfrc_applied[env.base_id, :3] = 0.0

                    for _ in range(env.decimation):
                        mujoco.mj_step(env.model, env.data)

                    obs = env._get_obs()
                    base_z = env.data.xpos[env.base_id][2]
                    term = bool(base_z < 0.055)
                else:
                    obs_batch = obs.reshape(1, env.obs_dim).astype(np.float32)
                    outputs = session.run(None, {inp_name: obs_batch})
                    action = np.clip(outputs[0][0], -1.0, 1.0)
                    obs, reward, term, trunc, info = env.step(action)
                    base_z = info["base_z"]

                step_idx += 1

                # Quick recovery on fall
                if term:
                    print(f"[!] Robot fell (base height={base_z:.3f}m). Auto-recovering in 0.4s...")
                    time.sleep(0.4)
                    obs = env.reset(command=current_cmd)

                # Render and update bottom-right companion camera window (~25 Hz)
                if pip_active[0] and r_cam is not None and step_idx % 2 == 0:
                    r_cam.update_scene(env.data, camera="head_camera")
                    rgb_f = r_cam.render()
                    bgr_f = cv2.cvtColor(rgb_f, cv2.COLOR_RGB2BGR)

                    r_mat = env.data.xmat[env.base_id].reshape(3, 3)
                    roll_d = math.degrees(math.atan2(r_mat[2, 1], r_mat[2, 2]))
                    pitch_d = math.degrees(math.atan2(-r_mat[2, 0], math.sqrt(r_mat[2, 1]**2 + r_mat[2, 2]**2)))
                    b_z = env.data.xpos[env.base_id][2]

                    cv2.rectangle(bgr_f, (0, 0), (320, 22), (18, 22, 28), -1)
                    cv2.putText(bgr_f, "MICO HEAD CAM // FPV", (8, 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 240, 255), 1, cv2.LINE_AA)
                    cv2.rectangle(bgr_f, (0, 218), (320, 240), (18, 22, 28), -1)
                    cv2.putText(bgr_f, f"R:{roll_d:+.1f} deg P:{pitch_d:+.1f} deg Z:{b_z * 100:.0f}cm", (8, 233),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (220, 220, 220), 1, cv2.LINE_AA)

                    cv2.imshow(cam_win_name, bgr_f)
                    cv2.waitKey(1)

            # Keep camera tracking the robot as it walks forward (when in free orbit mode)
            if not fpv_mode[0]:
                base_pos = env.data.xpos[env.base_id]
                viewer.cam.lookat = [base_pos[0], base_pos[1], 0.11]
            viewer.sync()

            # Dynamic speed pacing
            target_sleep = (base_dt / speed_mult) - (time.perf_counter() - step_start)
            if target_sleep > 0:
                time.sleep(target_sleep)

        if pip_active[0]:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
