"""Unit tests for Mico Humanoid Expressive Dance Choreography.

Runs with:
    python3 -m pytest src/humanoid_control/test/test_mico_dance.py
"""
from pathlib import Path
import shutil
import sys
import tempfile
import wave

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE.parent / "scripts"))

try:
    import bazan_dance as dance
except ImportError:
    try:
        import mico_dance as dance
    except ImportError:
        dance = None

try:
    import bazan_rl_env as env_mod
except ImportError:
    try:
        import mico_rl_env as env_mod
    except ImportError:
        env_mod = None


@pytest.fixture(scope="module")
def check_prerequisites():
    if dance is None or env_mod is None:
        pytest.skip("mico_dance or dependencies not installed")
    if shutil.which("xacro") is None:
        pytest.skip("xacro not found in PATH")


def test_nhe_buong_kinematics(check_prerequisites):
    """Verify kinematics outputs 20 joints and respects joint angle boundaries."""
    test_times = [0.0, 3.0, 10.0, 18.0, 28.0, 35.0, 42.0, 50.0]
    for t in test_times:
        q = dance.get_nhe_buong_pose(t, bpm=76.0)
        assert len(q) == 20
        assert "l_hip_pitch" in q
        assert "r_shoulder_pitch" in q
        assert "neck_yaw" in q
        # Ensure values are finite and within reasonable bounds
        for j, val in q.items():
            assert not (val != val)  # Not NaN
            assert abs(val) < 3.2  # Max allowable angle ~pi


def test_soundtrack_synthesis(check_prerequisites):
    """Verify WAV soundtrack generator creates valid uncompressed PCM audio."""
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmp:
        tmp_path = Path(tmp.name)
        dance.generate_nhe_buong_soundtrack(tmp_path, duration=2.0, bpm=76.0)
        assert tmp_path.exists()
        with wave.open(str(tmp_path), "rb") as f:
            assert f.getnchannels() == 1
            assert f.getsampwidth() == 2
            assert f.getframerate() == 22050
            frames = f.getnframes()
            assert frames == int(2.0 * 22050)


def test_dance_simulation_short(check_prerequisites):
    """Verify robot dances for 150 steps (3.0s) without falling."""
    env = env_mod.BazanEnv()
    env.reset()

    curr_ctrl = env.q_nominal.copy()
    alpha = 0.70
    kp_roll = 0.45
    kd_roll = 0.03
    kp_pitch = 0.35
    kd_pitch = 0.02

    for s in range(150):
        t_sec = s * 0.02
        q_target = dance.get_nhe_buong_pose(t_sec, bpm=76.0)

        # Dynamic balance feedback
        r_mat = env.data.xmat[env.base_id].reshape(3, 3)
        proj_g = r_mat.T @ [0.0, 0.0, -1.0]
        omega_base = r_mat.T @ env.data.qvel[3:6]

        roll_corr = kp_roll * proj_g[1] - kd_roll * omega_base[0]
        pitch_corr = -kp_pitch * proj_g[0] - kd_pitch * omega_base[1]

        q_target["l_ankle_roll"] += roll_corr
        q_target["r_ankle_roll"] += roll_corr
        q_target["l_ankle_pitch"] += pitch_corr
        q_target["r_ankle_pitch"] += pitch_corr

        import numpy as np
        target_arr = np.array([q_target[j] for j in dance.gait.JOINTS], dtype=np.float64)
        curr_ctrl = alpha * curr_ctrl + (1.0 - alpha) * target_arr

        env.data.ctrl[env.all_act_ids] = curr_ctrl
        for _ in range(env.decimation):
            dance.mujoco.mj_step(env.model, env.data)

        z = float(env.data.xpos[env.base_id][2])
        assert z > 0.070, f"Robot fell at step {s} with height {z}m"
