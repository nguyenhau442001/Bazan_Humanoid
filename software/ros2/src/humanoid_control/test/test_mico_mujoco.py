"""MuJoCo physics of mico_mujoco.py (the ROS 2 bridge core), no ROS. Skipped where MuJoCo or xacro is missing
(the Ubuntu CI has neither MuJoCo nor a reason to install it; run it in the ros2_jazzy conda env).

    python3 -m pytest src/humanoid_control/test/test_mico_mujoco.py
"""
import math
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE.parent / "scripts"))
import mico_gait as gait  # noqa: E402

try:                       # skip test by test (a module-level skip collects nothing: pytest exit 5, ament fails)
    import mico_mujoco as mm
except ImportError:
    mm = None

DESCRIPTION = REPO / "software" / "ros2" / "src" / "humanoid_description"


def resolve(filename):
    return str(DESCRIPTION / filename[len("package://humanoid_description/"):])


@pytest.fixture(scope="module")
def mjcf():
    if mm is None:
        pytest.skip("MuJoCo not installed (pip install mujoco)")
    exe = shutil.which("xacro")
    if exe is None:
        pytest.skip("xacro not found")
    urdf = subprocess.run([exe, str(DESCRIPTION / "urdf" / "mico.urdf.xacro"), "look:=mico",
                           "mass:=sim"],
                          check=True, capture_output=True, text=True).stdout
    return mm.build_mjcf(urdf, resolve)


def test_model(mjcf):
    sim = mm.MicoSim(mjcf)
    m = sim.model
    assert m.nu == 20 and m.nmesh == 48 and m.opt.timestep == pytest.approx(0.001)
    stall = {j: abs(m.actuator_forcerange[m.actuator(j).id, 1]) for j in gait.JOINTS}
    assert stall["l_knee"] == pytest.approx(0.343) and stall["l_hip_yaw"] == pytest.approx(0.216)
    assert stall["neck_yaw"] == pytest.approx(0.177)


def test_stands_still(mjcf):
    sim = mm.MicoSim(mjcf)
    sim.reset()
    assert sim.base_pose()[0][2] == pytest.approx(0.116, abs=0.001), "spawned with the soles 1 mm above the floor"
    sim.step(2000)
    xyz, _ = sim.base_pose()
    assert abs(xyz[2] - 0.115) < 0.003 and np.linalg.norm(xyz[:2]) < 0.003
    assert math.degrees(sim.tilt()) < 1.0
    acc, gyro, _ = sim.imu()
    assert acc[2] == pytest.approx(9.81, abs=0.05) and np.linalg.norm(gyro) < 0.01


def test_trajectory_interpolates_like_jtc(mjcf):
    sim = mm.MicoSim(mjcf)
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
    """The trajectory mico_gait.py sends: no fall, about the expected distance and heading."""
    sim = mm.MicoSim(mjcf)
    sim.reset()
    sim.step(1000)
    times, positions = gait.trajectory_points(50.0, 2.0)
    sim.set_trajectory(gait.JOINTS, times, positions)
    worst = 0.0
    for _ in range(int(times[-1] + 1.0) * 10):
        sim.step(100)
        worst = max(worst, sim.tilt())
        assert sim.base_pose()[0][2] > 0.09, f"fell at t={sim.time:.1f} s"
    xyz, (w, x, y, z) = sim.base_pose()
    heading = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    assert 0.16 < xyz[0] < 0.21 and abs(heading - 60) < 6 and math.degrees(worst) < 15
