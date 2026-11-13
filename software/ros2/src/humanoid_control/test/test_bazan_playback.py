"""Offline checks of mico_playback.py (numpy only, no ROS): FK from the URDF and stance-foot odometry.

    python3 -m pytest src/humanoid_control/test/test_mico_playback.py
"""
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE.parent / "scripts"))
try:
    import bazan_gait as gait
except ImportError:
    import mico_gait as gait  # noqa: E402
try:
    import bazan_playback as mp
except ImportError:
    import mico_playback as mp  # noqa: E402

XACRO = HERE.parents[1] / "humanoid_description" / "urdf" / "bazan.urdf.xacro"
URDF = REPO / "software" / "ai" / "sim" / "mini_humanoid" / "humanoid.urdf"
DT = 0.02       # 50 Hz, as the node
MM = 1e-3


def kinematics(path=XACRO):
    return mp.Kinematics(path.read_text())


def run_demo(kin):
    """Play the demo once. Per tick: time, stance foot, odom->base_link, world pose of both soles."""
    anchor = mp.SoleAnchor(kin)
    rows = []
    for i in range(int(gait.T_END / DT) + 1):
        t = i * DT
        q = gait.targets(t)
        base = anchor.reset(q) if i == 0 else anchor.update(q)
        rows.append((t, anchor.stance, base, {s: base @ kin.pose(q, s) for s in mp.SOLES}))
    return rows


def test_standing_pose():
    kin = kinematics()
    q = gait.targets(0.0)
    for side, y in (("l_sole", 0.022), ("r_sole", -0.022)):
        sole = kin.pose(q, side)
        assert abs(sole[2, 3] + 0.115) < 1 * MM, "base_link is 115 mm above the soles"
        assert abs(sole[1, 3] - y) < 1 * MM and abs(sole[0, 3]) < 1 * MM
    base = mp.SoleAnchor(kin).reset(q)
    assert abs(base[2, 3] - 0.115) < 1 * MM
    assert np.allclose(base[:2, 3], 0.0, atol=1 * MM) and np.allclose(base[:3, :3], np.eye(3), atol=1e-9)


def test_fk_matches_plain_urdf():
    """The xacro and sim/mini_humanoid/humanoid.urdf come from the same generator: same FK."""
    a, b = kinematics(XACRO), kinematics(URDF)
    for t in np.linspace(0.0, gait.T_END, 40):
        q = gait.targets(t)
        for link in mp.SOLES + ("l_gripper", "r_gripper", "camera_link"):
            assert np.allclose(a.pose(q, link), b.pose(q, link), atol=1e-9)


def test_stance_foot_stays_put():
    """During every stance phase the stance sole keeps its world pose (< 1 mm), flat on the floor."""
    rows = run_demo(kinematics())
    phases = []
    for t, stance, _, soles in rows:
        if not phases or phases[-1][0] != stance:
            phases.append((stance, soles[stance]))
        ref = phases[-1][1]
        pose = soles[stance]
        assert np.linalg.norm(pose[:3, 3] - ref[:3, 3]) < 1 * MM, f"stance foot moved at t={t:.2f}"
        assert abs(pose[2, 3]) < 1e-9 and abs(pose[2, 2] - 1.0) < 1e-9, f"stance foot not flat at t={t:.2f}"
    assert len(phases) > 10, "the stance foot changes at every step"


def test_feet_on_the_floor_do_not_slide():
    """Any foot touching the floor (z < 0.2 mm) stays within 1 mm of where it touched down, and no foot
    sinks deeper than the 1 mm hysteresis."""
    rows = run_demo(kinematics())
    for side in mp.SOLES:
        down = None
        for t, _, _, soles in rows:
            p = soles[side][:3, 3]
            assert p[2] > -1.0 * MM - 1e-6, f"{side} below the floor at t={t:.2f}"
            if p[2] < 0.2 * MM:
                down = p if down is None else down
                assert np.linalg.norm(p[:2] - down[:2]) < 1 * MM, f"{side} slides at t={t:.2f}"
            else:
                down = None


def test_walks_forward_and_turns_left_60():
    rows = run_demo(kinematics())
    end_walk = next(base for t, _, base, _ in rows if t >= gait.T_TURN0)
    assert abs(end_walk[0, 3] - 7 * gait.STEP) < 2 * MM, "8 steps: 7 full strides of 24 mm"
    assert abs(end_walk[1, 3]) < 2 * MM and abs(math.degrees(mp.yaw_of(end_walk))) < 0.5
    xs = [base[0, 3] for t, _, base, _ in rows if t <= gait.T_TURN0]
    assert all(b >= a - 1 * MM for a, b in zip(xs, xs[1:])), "x never goes backwards while walking"
    end = rows[-1][2]
    assert abs(math.degrees(mp.yaw_of(end)) - 60.0) < 1.0
    assert 0.105 < min(base[2, 3] for _, _, base, _ in rows) and max(base[2, 3] for _, _, base, _ in rows) < 0.116


def test_trajectory_points():
    """mico_gait.trajectory_points (what the gait node sends to MuJoCo) samples targets() at 50 Hz."""
    times, positions = gait.trajectory_points(50.0, 2.0)
    assert len(times) == len(positions) == int(round((2.0 + gait.T_END) * 50)) + 1
    assert all(p == [0.0] * len(gait.JOINTS) for t, p in zip(times, positions) if t <= 2.0)
    for i in (150, 800, 1500):
        q = gait.targets(times[i] - 2.0)
        assert positions[i] == [float(q.get(j, 0.0)) for j in gait.JOINTS]


def test_quaternion():
    for r, p, y in ((0, 0, 0), (0.1, -0.2, 1.0), (0, 0, math.pi), (3.0, 0.1, -2.5)):
        m = mp.rpy_matrix(r, p, y)
        x, qy, z, w = mp.quaternion(m)
        assert abs(x * x + qy * qy + z * z + w * w - 1) < 1e-9
        back = np.array([[1 - 2 * (qy * qy + z * z), 2 * (x * qy - z * w), 2 * (x * z + qy * w)],
                         [2 * (x * qy + z * w), 1 - 2 * (x * x + z * z), 2 * (qy * z - x * w)],
                         [2 * (x * z - qy * w), 2 * (qy * z + x * w), 1 - 2 * (x * x + qy * qy)]])
        assert np.allclose(back, m, atol=1e-9)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
