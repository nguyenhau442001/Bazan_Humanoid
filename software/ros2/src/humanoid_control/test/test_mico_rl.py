"""Unit tests for Mico Reinforcement Learning environment and ONNX policy inference.

Runs with:
    python3 -m pytest src/humanoid_control/test/test_mico_rl.py
"""
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE.parent / "scripts"))

try:
    import mico_rl_env as env_mod
except ImportError:
    env_mod = None

try:
    import model_registry as reg_mod
except ImportError:
    reg_mod = None

try:
    import onnxruntime as ort
except ImportError:
    ort = None

try:
    from mico_rl_node import quat_to_rot_matrix
except ImportError:
    quat_to_rot_matrix = None

try:
    from train_mico_ppo import DeepResidualActorCritic, PerceptiveTitanTransformerActorCritic
except ImportError:
    DeepResidualActorCritic = None
    PerceptiveTitanTransformerActorCritic = None


@pytest.fixture(scope="module")
def check_prerequisites():
    if env_mod is None:
        pytest.skip("mico_rl_env or dependencies not installed")
    if shutil.which("xacro") is None:
        pytest.skip("xacro not found in PATH")


def test_quat_to_rot_matrix():
    if quat_to_rot_matrix is None:
        pytest.skip("mico_rl_node not importable")
    # Identity quaternion [0, 0, 0, 1]
    r = quat_to_rot_matrix(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(r, np.eye(3))


def test_single_env(check_prerequisites):
    env = env_mod.MicoEnv(decimation=2, max_episode_steps=50)
    obs = env.reset(command=[0.2, 0.0, 0.0])

    assert obs.shape == (45,)
    assert obs.dtype == np.float32

    # Step with zero actions
    action = np.zeros(12)
    next_obs, rew, term, trunc, info = env.step(action)

    assert next_obs.shape == (45,)
    assert isinstance(rew, (float, np.floating))
    assert isinstance(term, (bool, np.bool_))
    assert isinstance(trunc, (bool, np.bool_))
    assert "base_z" in info


def test_vector_env(check_prerequisites):
    num_envs = 2
    vec = env_mod.VectorMicoEnv(num_envs=num_envs, decimation=2, max_episode_steps=50)
    obs = vec.reset()

    assert obs.shape == (num_envs, 45)
    actions = np.zeros((num_envs, 12))
    obs, rews, terms, truncs, infos = vec.step(actions)

    assert obs.shape == (num_envs, 45)
    assert len(rews) == num_envs
    assert len(terms) == num_envs
    assert len(truncs) == num_envs
    vec.close()


def test_onnx_inference(check_prerequisites):
    if ort is None:
        pytest.skip("onnxruntime not installed")
    onnx_path = REPO / "src/humanoid_control/models/mico_policy.onnx"
    if not onnx_path.exists():
        pytest.skip("mico_policy.onnx not yet generated")

    sess = ort.InferenceSession(str(onnx_path))
    input_name = sess.get_inputs()[0].name
    obs_dim = sess.get_inputs()[0].shape[-1]
    dummy_obs = np.zeros((1, obs_dim), dtype=np.float32)

    outputs = sess.run(None, {input_name: dummy_obs})
    assert len(outputs) == 1
    assert outputs[0].shape == (1, 12)


def test_env_perturbation(check_prerequisites):
    env = env_mod.MicoEnv(decimation=2, max_episode_steps=50, domain_rand=True)
    env.reset()

    # Test friction adjustment
    env.set_friction(0.2)
    assert env.model.geom_friction[env.floor_geom_id, 0] == pytest.approx(0.2)

    # Test perturbation
    env.apply_perturbation([10.0, 0.0, 0.0], duration_steps=3)
    assert env.perturb_steps_left == 3
    env.step(np.zeros(12))
    assert env.perturb_steps_left == 2


def test_deep_res_network():
    if DeepResidualActorCritic is None:
        pytest.skip("train_mico_ppo not importable")

    net = DeepResidualActorCritic(obs_dim=45, act_dim=12, hidden_dim=512, num_blocks=4)
    total_params = sum(p.numel() for p in net.parameters() if p.requires_grad)
    actor_params = sum(p.numel() for p in net.actor_mean.parameters() if p.requires_grad)

    assert total_params > 4_000_000
    assert actor_params > 2_000_000


def test_model_registry_functions():
    if reg_mod is None:
        pytest.skip("model_registry not importable")

    reg = reg_mod.load_registry()
    assert isinstance(reg, dict)
    assert "models" in reg


def test_perceptive_env_60dim(check_prerequisites):
    env = env_mod.MicoEnv(decimation=2, max_episode_steps=50, use_perception=True)
    obs = env.reset(command=[0.2, 0.0, 0.0])

    assert env.obs_dim == 60
    assert obs.shape == (60,)
    assert obs.dtype == np.float32

    # Step with zero actions
    action = np.zeros(12)
    next_obs, rew, term, trunc, info = env.step(action)

    assert next_obs.shape == (60,)
    assert isinstance(rew, (float, np.floating))


def test_perceptive_titan_transformer_network():
    if PerceptiveTitanTransformerActorCritic is None:
        pytest.skip("train_mico_ppo not importable")

    net = PerceptiveTitanTransformerActorCritic(
        obs_dim=60, act_dim=12, prop_dim=51, hidden_dim=256, num_blocks=3, num_layers=2, num_heads=4
    )
    total_params = sum(p.numel() for p in net.parameters() if p.requires_grad)
    assert total_params > 1_000_000
