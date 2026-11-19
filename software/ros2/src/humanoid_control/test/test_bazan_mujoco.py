"""MuJoCo physics of bazan_mujoco.py (the ROS 2 bridge core) on the robot generated from the CAD, no ROS. Skipped
where MuJoCo is missing (pip install mujoco).

    python3 -m pytest software/ros2/src/humanoid_control/test/test_bazan_mujoco.py
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE.parent / "scripts"))
try:
    import bazan_gait as gait
except ImportError:
    import mico_gait as gait  # noqa: E402

try:                       # skip test by test (a module-level skip collects nothing: pytest exit 5, ament fails)
    try:
        import bazan_mujoco as mm
    except ImportError:
        import mico_mujoco as mm
except ImportError:
    mm = None


@pytest.fixture(scope="module")
def mjcf():
    if mm is None:
        pytest.skip("MuJoCo not installed (pip install mujoco)")
    return mm.cad_mjcf()


def stand_height():
    """Pelvis (hip pitch axes) above the floor with straight legs, from the CAD URDF."""
    return gait.SOLE + gait.ROLL_DOWN + gait.L1 + gait.L2


def test_model(mjcf):
    sim = mm.BazanSim(mjcf)
    m = sim.model
    assert m.nu == 20 and m.opt.timestep == pytest.approx(0.001)
    assert m.body_mass.sum() == pytest.approx(gait.MODEL.total), "the masses of the CAD URDF"
    stall = {j: abs(m.actuator_forcerange[m.actuator(j).id, 1]) for j in gait.JOINTS}
    assert stall["l_knee"] == pytest.approx(0.343) and stall["l_hip_yaw"] == pytest.approx(0.216)
    assert stall["neck_yaw"] == pytest.approx(0.177)


def test_stands_still(mjcf):
    sim = mm.BazanSim(mjcf)
    sim.reset()
    z0 = stand_height()
    assert sim.base_pose()[0][2] == pytest.approx(z0 + 0.001, abs=0.0005), "spawned with the soles 1 mm above the floor"
    sim.step(2000)
    xyz, _ = sim.base_pose()
    assert abs(xyz[2] - z0) < 0.003 and np.linalg.norm(xyz[:2]) < 0.003
    assert math.degrees(sim.tilt()) < 1.0
    acc, gyro, _ = sim.imu()
    assert acc[2] == pytest.approx(9.81, abs=0.05) and np.linalg.norm(gyro) < 0.01


def test_trajectory_interpolates_like_jtc(mjcf):
    sim = mm.BazanSim(mjcf)
    sim.reset()
    sim.step(100)
    t0 = sim.time
    sim.set_trajectory(["l_knee"], [1.0, 2.0], [[0.4], [0.8]])         # other joints hold their target
    assert sim.current_target()[gait.JOINTS.index("l_knee")] == pytest.approx(0.0)
    sim.step(500)                                                       # halfway to the first point
    assert sim.time - t0 == pytest.approx(0.5)
    assert sim.current_target()[gait.JOINTS.index("l_knee")] == pytest.approx(0.2, abs=1e-6)
    sim.step(1000)
    assert sim.current_target()[gait.JOINTS.index("l_knee")] == pytest.approx(0.6, abs=1e-6)
    sim.step(1000)
    target = sim.current_target()
    assert target[gait.JOINTS.index("l_knee")] == pytest.approx(0.8) and not np.any(np.delete(target, 3))


def test_demo_walks_and_turns(mjcf):
    """The trajectory bazan_gait.py sends: no fall, about the planned distance and heading."""
    sim = mm.BazanSim(mjcf)
    sim.reset()
    sim.step(1000)
    times, positions = gait.trajectory_points(50.0, 2.0)
    sim.set_trajectory(gait.JOINTS, times, positions)
    worst = 0.0
    for _ in range(int(times[-1] + 1.0) * 10):
        sim.step(100)
        worst = max(worst, sim.tilt())
        assert sim.base_pose()[0][2] > 0.8 * stand_height(), f"fell at t={sim.time:.1f} s"
    xyz, (w, x, y, z) = sim.base_pose()
    heading = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    planned = (gait.WALK_STEPS - 1.5) * gait.STEP                       # both feet end 6.5 strides ahead
    assert abs(xyz[0] - planned) < 0.025 and abs(heading - 60) < 6
    assert math.degrees(worst) < math.degrees(gait.LEAN) + 6, "only the planned pelvis roll, plus servo sag"
