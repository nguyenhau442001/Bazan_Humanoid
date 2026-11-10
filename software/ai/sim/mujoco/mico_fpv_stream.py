#!/usr/bin/env python3
"""Mico Humanoid FPV (First Person View) Perception Stream.

Renders real-time camera feed from Mico's head camera (`head_camera`) as the robot walks.
Supports Picture-in-Picture (PiP) mode with the camera view displayed in the bottom-right corner,
tactical perception HUD overlay, and automatic desktop corner docking.

Usage:
    # Picture-in-Picture mode (Main 3rd-person view + Bottom-Right Head Camera)
    python3 sim/mujoco/mico_fpv_stream.py

    # Full-screen FPV view only
    python3 sim/mujoco/mico_fpv_stream.py --fpv-only

    # Export an animated walking GIF with bottom-right camera view
    python3 sim/mujoco/mico_fpv_stream.py --gif docs/images/mico_pip_walk.gif
"""
import argparse
import math
from pathlib import Path
import sys
import time

import cv2
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE))
SCRIPTS_DIR = REPO / "software" / "ros2" / "src" / "humanoid_control" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
import mico_gait as gait  # noqa: E402
import mico_playback as mp  # noqa: E402

DT = 0.02  # 50 Hz sim rate


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


def add_virtual_obstacles(spec):
    """Add colorful obstacles to the MuJoCo world for perception testing."""
    wb = spec.worldbody
    # Red obstacle cylinder at x=0.9m
    c1 = wb.add_body(name="obstacle_cylinder", pos=[0.9, 0.05, 0.12])
    c1.add_geom(name="geom_obs_cyl", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.05, 0.12],
                rgba=[0.9, 0.2, 0.2, 1.0], contype=0, conaffinity=0, group=1)

    # Orange caution box at x=1.6m, y=-0.12m
    b1 = wb.add_body(name="obstacle_box1", pos=[1.6, -0.12, 0.08])
    b1.add_geom(name="geom_obs_box1", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.06, 0.06, 0.08],
                rgba=[1.0, 0.55, 0.0, 1.0], contype=0, conaffinity=0, group=1)

    # Cyan beacon pillar at x=2.3m, y=0.15m
    b2 = wb.add_body(name="obstacle_beacon", pos=[2.3, 0.15, 0.15])
    b2.add_geom(name="geom_obs_b2", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.04, 0.15],
                rgba=[0.1, 0.8, 0.9, 1.0], contype=0, conaffinity=0, group=1)

    # Yellow marker block at x=0.5m, y=-0.18m
    b3 = wb.add_body(name="obstacle_marker", pos=[0.5, -0.18, 0.04])
    b3.add_geom(name="geom_obs_b3", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.04, 0.04, 0.04],
                rgba=[0.95, 0.9, 0.1, 1.0], contype=0, conaffinity=0, group=1)


def draw_hud(frame, roll_deg, pitch_deg, height_m, step_freq, dist_to_obs=None, compact=False):
    """Draw a clean, tactical robot vision perception overlay on the camera frame."""
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # Semi-transparent top & bottom telemetry bars
    bar_h = 28 if compact else 40
    cv2.rectangle(overlay, (0, 0), (w, bar_h), (18, 22, 28), -1)
    cv2.rectangle(overlay, (0, h - bar_h), (w, h), (18, 22, 28), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    # Header info
    font_scale = 0.36 if compact else 0.52
    cv2.putText(frame, "FPV // MICO HEAD_CAM", (10, 19 if compact else 25),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 240, 255), 1, cv2.LINE_AA)
    rec_time = time.strftime("%H:%M:%S")
    cv2.putText(frame, f"REC {rec_time}", (w - (95 if compact else 135), 19 if compact else 25),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 60, 255), 1, cv2.LINE_AA)

    # Artificial Horizon Crosshair
    cx, cy = w // 2, h // 2
    horizon_shift = int(pitch_deg * (2.0 if compact else 3.5))
    center_y = np.clip(cy + horizon_shift, bar_h + 10, h - bar_h - 10)

    # Horizon tilt angle
    rad = math.radians(roll_deg)
    line_len = 45 if compact else 80
    gap = 15 if compact else 25
    dx = int(line_len * math.cos(rad))
    dy = int(line_len * math.sin(rad))

    cv2.line(frame, (cx - dx, center_y - dy), (cx - gap, center_y - int(gap * math.sin(rad))),
             (0, 230, 255), 1, cv2.LINE_AA)
    cv2.line(frame, (cx + gap, center_y + int(gap * math.sin(rad))), (cx + dx, center_y + dy),
             (0, 230, 255), 1, cv2.LINE_AA)

    # Reticle Center Box
    box_s = 10 if compact else 16
    cv2.rectangle(frame, (cx - box_s, cy - box_s), (cx + box_s, cy + box_s), (0, 255, 180), 1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 2, (0, 255, 180), -1)

    # Telemetry Bottom bar
    if compact:
        tele_text = f"ALT:{height_m * 100:.0f}cm R:{roll_deg:+.0f}° P:{pitch_deg:+.0f}°"
        cv2.putText(frame, tele_text, (8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1, cv2.LINE_AA)
    else:
        tele_text = f"ALT: {height_m * 100:.1f}cm  |  ROLL: {roll_deg:+.1f} deg  |  PITCH: {pitch_deg:+.1f} deg"
        cv2.putText(frame, tele_text, (15, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)

    # Obstacle Proximity Warning
    if dist_to_obs is not None:
        color = (0, 60, 255) if dist_to_obs < 0.8 else ((0, 215, 255) if dist_to_obs < 1.5 else (0, 255, 120))
        obs_text = f"OBS: {dist_to_obs:.2f}m" if compact else f"OBSTACLE AHEAD: {dist_to_obs:.2f} m"
        offset_x = (w - 95) if compact else (w - 230)
        cv2.putText(frame, obs_text, (offset_x, h - (8 if compact else 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36 if compact else 0.45, color, 1, cv2.LINE_AA)

    return frame


def embed_pip_view(main_bgr, fpv_bgr, pip_w=320, pip_h=240, pad=20):
    """Embed the FPV camera frame into the bottom-right corner of the main frame."""
    h, w = main_bgr.shape[:2]
    x1 = w - pip_w - pad
    y1 = h - pip_h - pad
    x2 = x1 + pip_w
    y2 = y1 + pip_h

    # Shadow & Neon Cyan Border
    cv2.rectangle(main_bgr, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), (15, 18, 24), -1)
    cv2.rectangle(main_bgr, (x1 - 2, y1 - 2), (x2 + 2, y2 + 2), (0, 230, 255), 2)

    # Place resized FPV frame
    resized_fpv = cv2.resize(fpv_bgr, (pip_w, pip_h))
    main_bgr[y1:y2, x1:x2] = resized_fpv

    # Corner Title Tag
    tag_w = 190
    tag_h = 22
    cv2.rectangle(main_bgr, (x1, y1 - tag_h), (x1 + tag_w, y1), (0, 230, 255), -1)
    cv2.putText(main_bgr, "MICO HEAD CAM (FPV) [LIVE]", (x1 + 6, y1 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (15, 18, 24), 1, cv2.LINE_AA)

    return main_bgr


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", type=Path, default=HERE / "mico.xml", help="Path to mico.xml")
    parser.add_argument("--obstacles", action="store_true", default=True, help="Spawn virtual obstacle markers")
    parser.add_argument("--no-obstacles", action="store_false", dest="obstacles", help="Do not spawn obstacles")
    parser.add_argument("--hud", action="store_true", default=True, help="Enable perception HUD overlay")
    parser.add_argument("--fpv-only", action="store_true", default=False, help="Display only full-screen FPV view")
    parser.add_argument("--gif", type=Path, help="Export walking sequence to animated GIF and exit")
    parser.add_argument("--frames", type=int, default=150, help="Number of frames for GIF export (default: 150 = 3s)")
    args = parser.parse_args()

    # Load and customize model with virtual obstacles if requested
    spec = mujoco.MjSpec.from_file(str(args.model))
    if args.obstacles:
        add_virtual_obstacles(spec)

    model = spec.compile()
    data = mujoco.MjData(model)

    # Check for head_camera
    cam_names = [model.camera(i).name for i in range(model.ncam)]
    if "head_camera" not in cam_names:
        sys.exit("Error: 'head_camera' not defined in model.")

    # Renderers
    # Main 3rd-person Chase Renderer: 960x540
    r_main = mujoco.Renderer(model, 540, 960)
    # FPV Head Camera Renderer: 480x640
    r_fpv = mujoco.Renderer(model, 480, 640)

    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[1] = opt.geomgroup[2] = 1  # Blender shells + floor/obstacles

    chase_cam = mujoco.MjvCamera()
    chase_cam.distance, chase_cam.elevation, chase_cam.azimuth = 0.85, -15, 140

    # Setup kinematics playback
    play = mp.Playback(model, data) if hasattr(mp, "Playback") else None
    if play is None:
        from mico_viewer import Playback
        play = Playback(model, data)

    pip_mode = not args.fpv_only

    # Export GIF mode
    if args.gif:
        from PIL import Image
        mode_label = "Picture-in-Picture (Chase + Bottom-Right FPV)" if pip_mode else "Full FPV"
        print(f"[*] Rendering {args.frames} frames ({mode_label}) to GIF ({args.frames * DT:.1f}s)...")
        gif_frames = []
        for i in range(args.frames):
            t = i * DT
            base = play.set(t)

            # Compute roll/pitch & distance
            r_mat = base[:3, :3]
            roll_deg = math.degrees(math.atan2(r_mat[2, 1], r_mat[2, 2]))
            pitch_deg = math.degrees(math.atan2(-r_mat[2, 0], math.sqrt(r_mat[2, 1]**2 + r_mat[2, 2]**2)))
            height_m = base[2, 3]
            dist = max(0.0, 0.9 - base[0, 3])

            # Render FPV
            r_fpv.update_scene(data, camera="head_camera", scene_option=opt)
            fpv_rgb = r_fpv.render().copy()
            fpv_bgr = cv2.cvtColor(fpv_rgb, cv2.COLOR_RGB2BGR)
            if args.hud:
                fpv_bgr = draw_hud(fpv_bgr, roll_deg, pitch_deg, height_m, step_freq=2.0,
                                   dist_to_obs=dist, compact=pip_mode)

            if pip_mode:
                chase_cam.lookat[:] = base[:3, 3] + [0, 0, 0.03]
                r_main.update_scene(data, chase_cam, opt)
                main_rgb = r_main.render().copy()
                main_bgr = cv2.cvtColor(main_rgb, cv2.COLOR_RGB2BGR)
                final_bgr = embed_pip_view(main_bgr, fpv_bgr, pip_w=320, pip_h=240, pad=20)
            else:
                final_bgr = fpv_bgr

            final_rgb = cv2.cvtColor(final_bgr, cv2.COLOR_BGR2RGB)
            gif_frames.append(Image.fromarray(final_rgb))

            if (i + 1) % 50 == 0:
                print(f"  Frame {i + 1} / {args.frames} rendered...")

        args.gif.parent.mkdir(parents=True, exist_ok=True)
        gif_frames[0].save(
            str(args.gif),
            save_all=True,
            append_images=gif_frames[1:],
            duration=int(DT * 1000),
            loop=0,
            optimize=True
        )
        print(f"[✓] Saved walking animation: {args.gif} ({len(gif_frames)} frames)")
        return

    # Live OpenCV window mode
    print("\n" + "=" * 75)
    print("      MICO HUMANOID PERCEPTION STREAM // BOTTOM-RIGHT CAMERA VIEW")
    print("=" * 75)
    print("  View Layout:")
    print("    • Main View          : 3rd-Person Orbit Chase Cam (Body & Gait)")
    print("    • Bottom-Right Corner: Real-time FPV Head Camera (Tactical HUD)")
    print("  Keyboard Shortcuts:")
    print("    [Tab] / [V] : Toggle between Picture-in-Picture and Full FPV")
    print("    [Space]     : Pause / Resume walking playback")
    print("    [H]         : Toggle HUD perception overlay")
    print("    [S]         : Save snapshot to /tmp/mico_camera_snapshot.png")
    print("    [Q] / [Esc] : Quit")
    print("=" * 75 + "\n")

    win_name = "Mico Perception Stream (Head Camera at Bottom-Right)"
    cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)

    # Position window conveniently on desktop
    screen_w, screen_h = get_screen_resolution()
    win_w = 960 if pip_mode else 640
    win_h = 540 if pip_mode else 480
    pos_x = max(0, (screen_w - win_w) // 2)
    pos_y = max(0, (screen_h - win_h) // 2)
    cv2.moveWindow(win_name, pos_x, pos_y)

    t0 = time.monotonic()
    paused = False
    pause_time = 0.0
    hud_enabled = args.hud
    sim_t = 0.0

    while True:
        loop_start = time.monotonic()

        if not paused:
            sim_t = (time.monotonic() - t0 - pause_time) % gait.T_END

        base = play.set(sim_t)

        r_mat = base[:3, :3]
        roll_deg = math.degrees(math.atan2(r_mat[2, 1], r_mat[2, 2]))
        pitch_deg = math.degrees(math.atan2(-r_mat[2, 0], math.sqrt(r_mat[2, 1]**2 + r_mat[2, 2]**2)))
        height_m = base[2, 3]
        dist = max(0.0, 0.9 - base[0, 3])

        # Render FPV
        r_fpv.update_scene(data, camera="head_camera", scene_option=opt)
        fpv_rgb = r_fpv.render()
        fpv_bgr = cv2.cvtColor(fpv_rgb, cv2.COLOR_RGB2BGR)

        if hud_enabled:
            fpv_bgr = draw_hud(fpv_bgr, roll_deg, pitch_deg, height_m, step_freq=2.0,
                               dist_to_obs=dist, compact=pip_mode)

        if pip_mode:
            chase_cam.lookat[:] = base[:3, 3] + [0, 0, 0.03]
            r_main.update_scene(data, chase_cam, opt)
            main_rgb = r_main.render()
            main_bgr = cv2.cvtColor(main_rgb, cv2.COLOR_RGB2BGR)
            display_frame = embed_pip_view(main_bgr, fpv_bgr, pip_w=320, pip_h=240, pad=20)
        else:
            display_frame = fpv_bgr

        cv2.imshow(win_name, display_frame)

        key = cv2.waitKey(max(1, int(DT * 1000) - int((time.monotonic() - loop_start) * 1000))) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            break
        elif key == 32:  # Space
            paused = not paused
            print(f"[!] Stream {'PAUSED' if paused else 'RESUMED'}")
        elif key in (9, ord('v'), ord('V')):  # Tab or V
            pip_mode = not pip_mode
            print(f"[!] View Mode: {'Picture-in-Picture (Bottom-Right Camera)' if pip_mode else 'Full-Screen FPV'}")
        elif key in (ord('h'), ord('H')):
            hud_enabled = not hud_enabled
            print(f"[!] HUD {'ENABLED' if hud_enabled else 'DISABLED'}")
        elif key in (ord('s'), ord('S')):
            snap_path = "/tmp/mico_camera_snapshot.png"
            cv2.imwrite(snap_path, display_frame)
            print(f"[📸] Snapshot saved to {snap_path}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
