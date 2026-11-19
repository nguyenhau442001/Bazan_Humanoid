"""Training and deployment see the same robot: bazan_policy_io models and the node <-> environment equivalence.

    python3 -m pytest software/ros2/src/humanoid_control/test/test_bazan_policy_io.py

The equivalence test needs MuJoCo, onnx and onnxruntime and is skipped without them.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import bazan_gait as gait  # noqa: E402
import bazan_policy_io as io  # noqa: E402


def test_thermal_steady_state_and_lag():
    th = io.ThermalModel()
    th.reset(ambient=30.0)
    stall = io.STALL_NM[1]
    for _ in range(10):                                      # 10 s at half stall: barely warmer (5 min lag)
        th.step(1.0, [0.5 * stall, 0.0], [stall, stall])
    assert 30.0 < th.temps[0] < 31.5 and th.temps[1] == pytest.approx(30.0)
    for _ in range(3600):                                    # an hour: copper loss x thermal resistance
        th.step(1.0, [0.5 * stall, 0.0], [stall, stall])
    assert th.temps[0] == pytest.approx(30.0 + io.R_TH * io.P_STALL_W * 0.25, abs=0.01)


def test_power_follows_servo_torque():
    pm = io.PowerModel()
    v0, i0 = pm.read(np.zeros(20))
    v1, i1 = pm.read(np.full(20, 0.1))
    assert i0 == pytest.approx(20 * io.IDLE_A) and i1 > i0 and v1 < v0 < io.BATT_V


def test_action_map_residual_follows_the_gait():
    am = io.ActionMap(residual=True, action_filter=0.0)
    t, cmd = 7.3, np.array([io.V_REF, 0.0, 0.0])
    q = am(np.zeros(32), t, cmd)
    leg = [gait.JOINTS.index(j) for j in io.LEG_JOINTS]
    assert np.allclose(q[leg], io.reference(t, cmd), atol=1e-5)
    assert np.allclose(np.delete(q, leg), np.delete(am.q_nominal, leg))
    one = am(np.eye(32)[3], t, cmd)                          # one unit on the left knee
    knee = gait.JOINTS.index("l_knee")
    assert one[knee] - q[knee] == pytest.approx(io.ACTION_SCALE, abs=1e-5)     # the gait IK is solved to ~1e-6


def test_reference_roll_is_the_planned_lean():
    q = gait.single_support("l")
    assert io.pelvis_roll(q) == pytest.approx(-gait.LEAN, abs=1e-3)     # left side down: roll about x < 0


def tiny_policy(path, obs_dim, act_dim, meta):
    """A fixed random linear policy, tanh(obs W), written as ONNX with the trainer's metadata."""
    onnx = pytest.importorskip("onnx")
    from onnx import TensorProto, helper, numpy_helper
    W = np.random.default_rng(0).normal(0, 0.05, (obs_dim, act_dim)).astype(np.float32)
    g = helper.make_graph(
        [helper.make_node("MatMul", ["obs", "W"], ["h"]), helper.make_node("Tanh", ["h"], ["action"])], "p",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, obs_dim])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, act_dim])],
        [numpy_helper.from_array(W, "W")])
    model = helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    for k, v in meta.items():
        p = model.metadata_props.add()
        p.key, p.value = k, str(v)
    onnx.save(model, str(path))
    return path


def test_node_sees_and_acts_like_the_environment(tmp_path):
    """Drive the environment with a policy and feed the node's PolicyRunner the topics the bridge would publish
    at the same instants: every observation and every joint target must match."""
    ort = pytest.importorskip("onnxruntime")
    pytest.importorskip("mujoco")
    import bazan_rl_env as E
    import bazan_rl_node as N

    path = tiny_policy(tmp_path / "p.onnx", 64, 32, io.policy_meta(residual=True, straight_line=False,
                                                                   perception=True))
    runner = N.PolicyRunner(ort.InferenceSession(str(path)))
    env = E.BazanEnv(use_perception=True, use_residual=True)
    cmd = np.array([io.V_REF, 0.0, 0.0])
    obs = env.reset(command=cmd)
    m, d = env.model, env.data
    qadr = [m.jnt_qposadr[m.joint(j).id] for j in gait.JOINTS]
    dadr = [m.jnt_dofadr[m.joint(j).id] for j in gait.JOINTS]
    acc = m.sensor_adr[m.sensor("imu_acc").id]
    gyro = m.sensor_adr[m.sensor("imu_gyro").id]
    quat = m.sensor_adr[m.sensor("imu_quat").id]
    import bazan_mujoco as mm
    for k in range(60):
        w, x, y, z = d.sensordata[quat:quat + 4]
        topics = dict(q=d.qpos[qadr].copy(), dq=d.qvel[dadr].copy(), effort=d.actuator_force[env.all_act_ids].copy(),
                      gravity=N.quat_to_rot_matrix(x, y, z, w).T @ np.array([0.0, 0.0, -1.0]),
                      gyro=d.sensordata[gyro:gyro + 3].copy(), acc_z=float(d.sensordata[acc + 2]),
                      foot_forces=mm.foot_forces(m, d), tof_m=mm.tof_range(m, d), command=cmd)
        node_obs = runner.observe(topics)                   # what runner.step() does, split to compare obs
        bad = np.flatnonzero(~np.isclose(node_obs, obs, atol=1e-5))
        assert not len(bad), f"step {k}: {[io.OBS_64_NAMES[i] for i in bad]}"
        action = runner.session.run(None, {runner.input_name: node_obs[None]})[0][0]
        node_q = runner.map(np.clip(action, -1, 1), runner.t, cmd)
        runner.prev_action = np.clip(action, -1, 1)
        runner.t += runner.meta["control_dt"]
        obs, _, term, _, _ = env.step(action)
        assert np.allclose(d.ctrl[env.all_act_ids], node_q, atol=1e-5), f"step {k}: joint targets differ"
        assert not term
    assert math.isclose(runner.t, 60 * io.CONTROL_DT)
