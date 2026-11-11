#!/usr/bin/env python3
"""PPO Locomotion Trainer for Mico Humanoid.

Trains high-capacity Actor-Critic policies utilizing Apple Silicon Metal (MPS) GPU acceleration.
Supports both deep residual networks (2.1M+ parameters) and lightweight MLPs, with automatic
domain randomization (push disturbances & friction shock) and semantic ONNX version registration.

Usage:
    # Train 500 iterations of Deep ResNet on M4 GPU with domain randomization
    python3 src/humanoid_control/scripts/train_mico_ppo.py \
        --arch deep_res \
        --iterations 500 \
        --num-envs 16 \
        --domain-rand \
        --version-tag v2.0_deep_500iter
"""
import argparse
import os
from pathlib import Path
import sys
import time

import numpy as np
import torch
import torch.nn as nn
from torch.optim import Adam

# Workaround for duplicate OpenMP initialization on macOS conda
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE))
import mico_rl_env as env_mod  # noqa: E402
import model_registry as reg_mod  # noqa: E402


class ResBlock(nn.Module):
    """Residual Pre-Activation Block with Layer Normalization and GELU."""

    def __init__(self, dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(dim, dim),
            nn.LayerNorm(dim),
            nn.GELU(),
            nn.Linear(dim, dim),
            nn.LayerNorm(dim),
        )
        self.act = nn.GELU()

    def forward(self, x):
        return self.act(x + self.net(x))


class DeepResidualActorCritic(nn.Module):
    """High-capacity Deep Residual MLP Actor-Critic policy (~4.3M total parameters).

    Actor: 2.14M parameters across 4 residual blocks with LayerNorm & GELU.
    Critic: 2.14M parameters across 4 residual blocks with LayerNorm & GELU.
    """

    def __init__(self, obs_dim=45, act_dim=12, hidden_dim=512, num_blocks=4):
        super().__init__()
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.arch_name = "deep_res"

        # Actor Network (~2.14M parameters)
        actor_blocks = [
            nn.Linear(obs_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
        ]
        for _ in range(num_blocks):
            actor_blocks.append(ResBlock(hidden_dim))
        actor_last = nn.Linear(hidden_dim, act_dim)
        nn.init.orthogonal_(actor_last.weight, gain=0.01)
        nn.init.constant_(actor_last.bias, 0.0)
        actor_blocks.append(actor_last)
        actor_blocks.append(nn.Tanh())
        self.actor_mean = nn.Sequential(*actor_blocks)

        self.log_std = nn.Parameter(torch.ones(act_dim) * -1.2)

        # Critic Network (~2.14M parameters)
        critic_blocks = [
            nn.Linear(obs_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
        ]
        for _ in range(num_blocks):
            critic_blocks.append(ResBlock(hidden_dim))
        critic_last = nn.Linear(hidden_dim, 1)
        nn.init.orthogonal_(critic_last.weight, gain=1.0)
        nn.init.constant_(critic_last.bias, 0.0)
        critic_blocks.append(critic_last)
        self.critic = nn.Sequential(*critic_blocks)

    def forward(self, obs):
        return self.actor_mean(obs)

    def get_action_and_value(self, obs, action=None):
        mu = self.actor_mean(obs)
        std = self.log_std.exp()
        dist = torch.distributions.Normal(mu, std)

        if action is None:
            action = dist.sample()

        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        value = self.critic(obs).squeeze(-1)
        return action, log_prob, entropy, value

    def get_value(self, obs):
        return self.critic(obs).squeeze(-1)


class MLPActorCritic(nn.Module):
    """Baseline Lightweight MLP Actor-Critic policy (~90k total parameters)."""

    def __init__(self, obs_dim=45, act_dim=12, hidden_dims=(256, 128)):
        super().__init__()
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.arch_name = "mlp_light"

        actor_layers = []
        in_dim = obs_dim
        for h in hidden_dims:
            actor_layers.append(nn.Linear(in_dim, h))
            actor_layers.append(nn.ELU())
            in_dim = h
        actor_last = nn.Linear(in_dim, act_dim)
        nn.init.orthogonal_(actor_last.weight, gain=0.01)
        nn.init.constant_(actor_last.bias, 0.0)
        actor_layers.append(actor_last)
        actor_layers.append(nn.Tanh())
        self.actor_mean = nn.Sequential(*actor_layers)

        self.log_std = nn.Parameter(torch.ones(act_dim) * -1.2)

        critic_layers = []
        in_dim = obs_dim
        for h in hidden_dims:
            critic_layers.append(nn.Linear(in_dim, h))
            critic_layers.append(nn.ELU())
            in_dim = h
        critic_layers.append(nn.Linear(in_dim, 1))
        self.critic = nn.Sequential(*critic_layers)

    def forward(self, obs):
        return self.actor_mean(obs)

    def get_action_and_value(self, obs, action=None):
        mu = self.actor_mean(obs)
        std = self.log_std.exp()
        dist = torch.distributions.Normal(mu, std)

        if action is None:
            action = dist.sample()

        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        value = self.critic(obs).squeeze(-1)
        return action, log_prob, entropy, value

    def get_value(self, obs):
        return self.critic(obs).squeeze(-1)


class HumanoidTransformerActorCritic(nn.Module):
    """SOTA Hybrid Humanoid Transformer Actor-Critic Policy (20M to 50M+ parameters).

    Combines Deep Residual Pre-Activation Blocks with Multi-Head Self-Attention (MHSA)
    and Pre-LayerNorm residual connections for rock-solid bipedal dynamic balance,
    exploiting Apple Silicon Metal (MPS) GPU compute.
    """

    def __init__(self, obs_dim=47, act_dim=12, hidden_dim=640, num_blocks=4, num_layers=2, num_heads=8):
        super().__init__()
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.arch_name = "transformer"

        # Actor Network
        self.actor_stem = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
        )
        self.actor_res = nn.ModuleList([ResBlock(hidden_dim) for _ in range(num_blocks)])
        actor_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 2,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.actor_attn = nn.TransformerEncoder(actor_layer, num_layers=num_layers)
        self.actor_head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 256),
            nn.GELU(),
            nn.Linear(256, act_dim),
            nn.Tanh(),
        )
        nn.init.orthogonal_(self.actor_head[3].weight, gain=0.01)
        nn.init.constant_(self.actor_head[3].bias, 0.0)

        class ActorMean(nn.Module):
            def __init__(self, stem, res_blocks, attn, head):
                super().__init__()
                self.stem = stem
                self.res = res_blocks
                self.attn = attn
                self.head = head

            def forward(self, obs):
                h = self.stem(obs)
                for blk in self.res:
                    h = blk(h)
                h_attn = self.attn(h.unsqueeze(1)).squeeze(1)
                return self.head(h + h_attn)

        self.actor_mean = ActorMean(self.actor_stem, self.actor_res, self.actor_attn, self.actor_head)
        self.log_std = nn.Parameter(torch.ones(act_dim) * -1.2)

        # Critic Network
        self.critic_stem = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
        )
        self.critic_res = nn.ModuleList([ResBlock(hidden_dim) for _ in range(num_blocks)])
        critic_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 2,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.critic_attn = nn.TransformerEncoder(critic_layer, num_layers=num_layers)
        self.critic_head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 256),
            nn.GELU(),
            nn.Linear(256, 1),
        )
        nn.init.orthogonal_(self.critic_head[3].weight, gain=1.0)
        nn.init.constant_(self.critic_head[3].bias, 0.0)

    def forward(self, obs):
        return self.actor_mean(obs)

    def get_action_and_value(self, obs, action=None):
        mu = self.actor_mean(obs)
        std = self.log_std.exp()
        dist = torch.distributions.Normal(mu, std)

        if action is None:
            action = dist.sample()

        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)

        h_c = self.critic_stem(obs)
        for blk in self.critic_res:
            h_c = blk(h_c)
        h_c_attn = self.critic_attn(h_c.unsqueeze(1)).squeeze(1)
        value = self.critic_head(h_c + h_c_attn).squeeze(-1)
        return action, log_prob, entropy, value

    def get_value(self, obs):
        h_c = self.critic_stem(obs)
        for blk in self.critic_res:
            h_c = blk(h_c)
        h_c_attn = self.critic_attn(h_c.unsqueeze(1)).squeeze(1)
        return self.critic_head(h_c + h_c_attn).squeeze(-1)


class TitanTransformerActorCritic(nn.Module):
    """SOTA Multi-Stream Residual Transformer Policy (15.2M parameters).

    Combines 4 Pre-Activation Residual blocks with 2-layer Multi-Head Self-Attention
    Transformer Encoder and Decoupled Multi-Stream Specialty Heads:
    - Sagittal IK Head (6 joints: pitch joints for forward drive & obstacle clearance)
    - Lateral Balance Head (4 joints: roll joints for lateral stance & kick rejection)
    - Pelvis Steering Head (2 joints: yaw joints for yaw heading & ice slip recovery)
    """

    def __init__(self, obs_dim=47, act_dim=12, hidden_dim=512, num_blocks=4, num_layers=2, num_heads=4):
        super().__init__()
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.arch_name = "titan_transformer"

        # Shared Actor Backbone
        self.actor_stem = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
        )
        self.actor_res = nn.ModuleList([ResBlock(hidden_dim) for _ in range(num_blocks)])
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 2,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.actor_attn = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

        # Decoupled Multi-Stream Specialty Heads
        self.head_sagittal = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 256),
            nn.GELU(),
            nn.Linear(256, 6),
            nn.Tanh(),
        )
        self.head_lateral = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 256),
            nn.GELU(),
            nn.Linear(256, 4),
            nn.Tanh(),
        )
        self.head_yaw = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 128),
            nn.GELU(),
            nn.Linear(128, 2),
            nn.Tanh(),
        )

        for head, out_dim in [(self.head_sagittal, 6), (self.head_lateral, 4), (self.head_yaw, 2)]:
            nn.init.orthogonal_(head[3].weight, gain=0.01)
            nn.init.constant_(head[3].bias, 0.0)

        class MultiStreamActor(nn.Module):
            def __init__(self, stem, res, attn, h_sag, h_lat, h_yaw):
                super().__init__()
                self.stem = stem
                self.res = res
                self.attn = attn
                self.h_sag = h_sag
                self.h_lat = h_lat
                self.h_yaw = h_yaw

            def forward(self, obs):
                h = self.stem(obs)
                for blk in self.res:
                    h = blk(h)
                h_attn = self.attn(h.unsqueeze(1)).squeeze(1)
                h_fused = h + h_attn

                sag = self.h_sag(h_fused)
                lat = self.h_lat(h_fused)
                yaw = self.h_yaw(h_fused)

                return torch.stack([
                    yaw[:, 0], lat[:, 0], sag[:, 0], sag[:, 1], sag[:, 2], lat[:, 1],
                    yaw[:, 1], lat[:, 2], sag[:, 3], sag[:, 4], sag[:, 5], lat[:, 3],
                ], dim=-1)

        self.actor_mean = MultiStreamActor(
            self.actor_stem, self.actor_res, self.actor_attn,
            self.head_sagittal, self.head_lateral, self.head_yaw
        )
        self.log_std = nn.Parameter(torch.ones(act_dim) * -1.2)

        # Critic Backbone
        self.critic_stem = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
        )
        self.critic_res = nn.ModuleList([ResBlock(hidden_dim) for _ in range(num_blocks)])
        critic_encoder = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 2,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.critic_attn = nn.TransformerEncoder(critic_encoder, num_layers=num_layers)
        self.critic_head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 256),
            nn.GELU(),
            nn.Linear(256, 1),
        )
        nn.init.orthogonal_(self.critic_head[3].weight, gain=1.0)
        nn.init.constant_(self.critic_head[3].bias, 0.0)

    def forward(self, obs):
        return self.actor_mean(obs)

    def get_action_and_value(self, obs, action=None):
        mu = self.actor_mean(obs)
        std = self.log_std.exp()
        dist = torch.distributions.Normal(mu, std)

        if action is None:
            action = dist.sample()

        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)

        h_c = self.critic_stem(obs)
        for blk in self.critic_res:
            h_c = blk(h_c)
        h_c_attn = self.critic_attn(h_c.unsqueeze(1)).squeeze(1)
        value = self.critic_head(h_c + h_c_attn).squeeze(-1)
        return action, log_prob, entropy, value

    def get_value(self, obs):
        h_c = self.critic_stem(obs)
        for blk in self.critic_res:
            h_c = blk(h_c)
        h_c_attn = self.critic_attn(h_c.unsqueeze(1)).squeeze(1)
        return self.critic_head(h_c + h_c_attn).squeeze(-1)


class PerceptiveTitanTransformerActorCritic(nn.Module):
    """SOTA Hardware-Faithful Perceptive Titan Transformer Policy (~1.6M parameters).

    Grounded strictly in Bazan BZN-01 physical BOM (~5.9M VND):
    - Stream 1: Proprioception & Foot Contact (51 dims):
      * MPU9250 6-DOF IMU (proj gravity 3, gyro 3)
      * 12 Leg servo potentiometers via CD74HC4067 (q_err 12, dq 12)
      * 4 Foot sole FSR402 contact forces (L/R heel & toe normal forces 4)
      * Action buffer (12), Velocity command (3), Gait clock phase (2)
    - Stream 2: Physical Exteroception & Telemetry (9 dims):
      * MPU9250 3-axis Magnetometer heading vector (3 dims)
      * VL53L0X ToF frontal laser distance (1 dim)
      * INA226 Bus voltage & total current telemetry (2 dims)
      * NTC 10 kΩ Left & right hip-roll servo temperatures (2 dims)
      * Radxa Zero 3W NPU camera visual target detected flag (1 dim)
    Total Observation Dimension: Exactly 60 dims.
    """

    def __init__(
        self,
        obs_dim=60,
        act_dim=12,
        prop_dim=51,
        hidden_dim=256,
        num_blocks=3,
        num_layers=2,
        num_heads=4,
    ):
        super().__init__()
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.prop_dim = prop_dim
        self.arch_name = "perceptive_transformer"
        perc_dim = obs_dim - prop_dim

        # Stream 1: Proprioceptive Stem (51 -> 128)
        self.prop_stem = nn.Sequential(
            nn.Linear(prop_dim, 128),
            nn.LayerNorm(128),
            nn.GELU(),
        )

        # Stream 2: Physical Perception Stem (33 -> 128)
        self.perc_stem = nn.Sequential(
            nn.Linear(perc_dim, 128),
            nn.LayerNorm(128),
            nn.GELU(),
            ResBlock(128),
        )

        # Cross-Fusion (128 + 128 = 256 -> hidden_dim)
        self.actor_fusion = nn.Sequential(
            nn.Linear(256, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
        )
        self.actor_res = nn.ModuleList([ResBlock(hidden_dim) for _ in range(num_blocks)])
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 2,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.actor_attn = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

        # Decoupled Multi-Stream Specialty Heads
        self.head_sagittal = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 128),
            nn.GELU(),
            nn.Linear(128, 6),
            nn.Tanh(),
        )
        self.head_lateral = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 128),
            nn.GELU(),
            nn.Linear(128, 4),
            nn.Tanh(),
        )
        self.head_yaw = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 64),
            nn.GELU(),
            nn.Linear(64, 2),
            nn.Tanh(),
        )

        for head in (self.head_sagittal, self.head_lateral, self.head_yaw):
            nn.init.orthogonal_(head[3].weight, gain=0.01)
            nn.init.constant_(head[3].bias, 0.0)

        class PerceptiveActor(nn.Module):
            def __init__(self, prop_stem, perc_stem, fusion, res, attn, h_sag, h_lat, h_yaw, prop_dim):
                super().__init__()
                self.prop_stem = prop_stem
                self.perc_stem = perc_stem
                self.fusion = fusion
                self.res = res
                self.attn = attn
                self.h_sag = h_sag
                self.h_lat = h_lat
                self.h_yaw = h_yaw
                self.prop_dim = prop_dim

            def forward(self, obs):
                prop = obs[..., :self.prop_dim]
                perc = obs[..., self.prop_dim:]
                h_prop = self.prop_stem(prop)
                h_perc = self.perc_stem(perc)
                h = self.fusion(torch.cat([h_prop, h_perc], dim=-1))
                for blk in self.res:
                    h = blk(h)
                h_attn = self.attn(h.unsqueeze(1)).squeeze(1)
                h_fused = h + h_attn

                sag = self.h_sag(h_fused)
                lat = self.h_lat(h_fused)
                yaw = self.h_yaw(h_fused)

                return torch.stack([
                    yaw[:, 0], lat[:, 0], sag[:, 0], sag[:, 1], sag[:, 2], lat[:, 1],
                    yaw[:, 1], lat[:, 2], sag[:, 3], sag[:, 4], sag[:, 5], lat[:, 3],
                ], dim=-1)

        self.actor_mean = PerceptiveActor(
            self.prop_stem, self.perc_stem, self.actor_fusion, self.actor_res, self.actor_attn,
            self.head_sagittal, self.head_lateral, self.head_yaw, self.prop_dim
        )
        self.log_std = nn.Parameter(torch.ones(act_dim) * -1.2)

        # Critic Two-Stream Backbone (Grounded in Mico A+ Hardware)
        self.critic_prop_stem = nn.Sequential(
            nn.Linear(prop_dim, 128),
            nn.LayerNorm(128),
            nn.GELU(),
        )
        self.critic_perc_stem = nn.Sequential(
            nn.Linear(perc_dim, 128),
            nn.LayerNorm(128),
            nn.GELU(),
            ResBlock(128),
        )
        self.critic_fusion = nn.Sequential(
            nn.Linear(256, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
        )
        self.critic_res = nn.ModuleList([ResBlock(hidden_dim) for _ in range(num_blocks)])
        critic_encoder = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 2,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.critic_attn = nn.TransformerEncoder(critic_encoder, num_layers=num_layers)
        self.critic_head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 128),
            nn.GELU(),
            nn.Linear(128, 1),
        )
        nn.init.orthogonal_(self.critic_head[3].weight, gain=1.0)
        nn.init.constant_(self.critic_head[3].bias, 0.0)

    def forward(self, obs):
        return self.actor_mean(obs)

    def _forward_critic(self, obs):
        prop = obs[..., :self.prop_dim]
        perc = obs[..., self.prop_dim:]
        h_prop = self.critic_prop_stem(prop)
        h_perc = self.critic_perc_stem(perc)
        h_c = self.critic_fusion(torch.cat([h_prop, h_perc], dim=-1))
        for blk in self.critic_res:
            h_c = blk(h_c)
        h_c_attn = self.critic_attn(h_c.unsqueeze(1)).squeeze(1)
        return self.critic_head(h_c + h_c_attn).squeeze(-1)

    def get_action_and_value(self, obs, action=None):
        mu = self.actor_mean(obs)
        std = self.log_std.exp()
        dist = torch.distributions.Normal(mu, std)

        if action is None:
            action = dist.sample()

        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        value = self._forward_critic(obs)
        return action, log_prob, entropy, value

    def get_value(self, obs):
        return self._forward_critic(obs)


def compute_gae(rewards, values, dones, next_value, gamma=0.99, lam=0.95):
    """Compute Generalized Advantage Estimation (GAE)."""
    steps, num_envs = rewards.shape
    advantages = torch.zeros_like(rewards)
    last_gae = torch.zeros(num_envs, device=rewards.device)

    for t in reversed(range(steps)):
        if t == steps - 1:
            next_non_terminal = 1.0 - dones[t].float()
            next_v = next_value
        else:
            next_non_terminal = 1.0 - dones[t].float()
            next_v = values[t + 1]

        delta = rewards[t] + gamma * next_v * next_non_terminal - values[t]
        last_gae = delta + gamma * lam * next_non_terminal * last_gae
        advantages[t] = last_gae

    returns = advantages + values
    return advantages, returns


def export_policy_onnx(model, save_path):
    """Export the actor policy network to ONNX format."""
    save_path = Path(save_path)
    save_path.parent.mkdir(parents=True, exist_ok=True)

    class ActorWrapper(nn.Module):
        def __init__(self, actor_mean):
            super().__init__()
            self.actor = actor_mean

        def forward(self, obs):
            return self.actor(obs)

    wrapper = ActorWrapper(model.actor_mean).cpu().eval()
    dummy_input = torch.zeros(1, model.obs_dim, dtype=torch.float32)

    torch.onnx.export(
        wrapper,
        dummy_input,
        str(save_path),
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch_size"}, "action": {0: "batch_size"}},
        dynamo=False,
    )
    print(f"Policy successfully exported to ONNX: {save_path} ({save_path.stat().st_size / 1024:.1f} KB)")


def train(args):
    device_str = "mps" if torch.backends.mps.is_available() and not args.cpu else "cpu"
    device = torch.device(device_str)
    print(f"[*] Training Mico PPO Locomotion Policy on: {device_str.upper()}")

    num_envs = args.num_envs
    rollout_len = args.rollout_length
    batch_size = num_envs * rollout_len
    minibatch_size = args.minibatch_size

    if args.arch == "perceptive_transformer" or getattr(args, "perception", False):
        args.perception = True
        args.use_phase = True
        if args.arch == "titan_transformer":
            args.arch = "perceptive_transformer"

    vec_env = env_mod.VectorMicoEnv(
        num_envs=num_envs,
        domain_rand=args.domain_rand,
        perturb_prob=args.perturb_prob if args.domain_rand else 0.0,
        use_residual=args.residual,
        use_phase=args.use_phase,
        obstacle_rand=getattr(args, "obstacle_rand", False),
        terrain_roughness=getattr(args, "terrain_roughness", 0.0),
        straight_line=getattr(args, "straight_line", False),
        target_speed=getattr(args, "target_speed", None),
        use_perception=getattr(args, "perception", False),
    )
    obs_dim = vec_env.obs_dim
    act_dim = vec_env.act_dim

    # Choose Architecture
    if args.arch in ("perceptive_transformer", "perceptive_titan"):
        policy = PerceptiveTitanTransformerActorCritic(
            obs_dim=obs_dim,
            act_dim=act_dim,
            prop_dim=51 if (getattr(args, "perception", False) or getattr(args, "use_fsr", False)) else (
                47 if args.use_phase else 45
            ),
            hidden_dim=args.hidden_dim,
            num_blocks=args.num_blocks,
            num_layers=args.num_layers,
            num_heads=args.num_heads,
        ).to(device)
    elif args.arch == "titan_transformer":
        policy = TitanTransformerActorCritic(
            obs_dim=obs_dim,
            act_dim=act_dim,
            hidden_dim=args.hidden_dim,
            num_blocks=args.num_blocks,
            num_layers=args.num_layers,
            num_heads=args.num_heads,
        ).to(device)
    elif args.arch == "transformer":
        policy = HumanoidTransformerActorCritic(
            obs_dim=obs_dim,
            act_dim=act_dim,
            hidden_dim=args.hidden_dim,
            num_blocks=args.num_blocks,
            num_layers=args.num_layers,
            num_heads=args.num_heads,
        ).to(device)
    elif args.arch == "deep_res":
        policy = DeepResidualActorCritic(
            obs_dim=obs_dim, act_dim=act_dim, hidden_dim=args.hidden_dim, num_blocks=args.num_blocks
        ).to(device)
    else:
        policy = MLPActorCritic(obs_dim=obs_dim, act_dim=act_dim).to(device)

    # Resume from existing checkpoint if specified
    if getattr(args, "resume_checkpoint", None) and Path(args.resume_checkpoint).exists():
        ckpt_path = Path(args.resume_checkpoint)
        print(f"[*] Resuming weights from checkpoint: {ckpt_path.name}")
        try:
            ckpt = torch.load(ckpt_path, map_location=device, weights_only=True)
            state_dict = policy.state_dict()
            matched = 0
            for k, v in ckpt.items():
                if k in state_dict:
                    if state_dict[k].shape == v.shape:
                        state_dict[k].copy_(v)
                        matched += 1
                    elif state_dict[k].dim() == 2 and v.dim() == 2 and state_dict[k].shape[0] == v.shape[0]:
                        # Adapt input dimension difference (45 vs 47)
                        min_in = min(state_dict[k].shape[1], v.shape[1])
                        state_dict[k][:, :min_in].copy_(v[:, :min_in])
                        matched += 1
            policy.load_state_dict(state_dict)
            print(f"[✓] Checkpoint loaded successfully! ({matched}/{len(state_dict)} tensors matched/adapted)")
        except Exception as e:
            print(f"[!] Warning: Could not load checkpoint weights: {e}")

    total_params = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    actor_params = sum(p.numel() for p in policy.actor_mean.parameters() if p.requires_grad)

    teacher_sess = None
    if getattr(args, "teacher_onnx", None):
        teacher_path = Path(args.teacher_onnx)
        if teacher_path.exists():
            import onnxruntime as ort
            teacher_sess = ort.InferenceSession(str(teacher_path))
            print(f"[*] Teacher policy loaded for knowledge distillation: {teacher_path.name}")

    effective_lr = 8e-5 if args.arch == "transformer" and args.lr == 2e-4 else args.lr
    optimizer = Adam(policy.parameters(), lr=effective_lr, eps=1e-5)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.iterations, eta_min=1e-5)

    # Rollout storage
    obs_buf = torch.zeros((rollout_len, num_envs, obs_dim), device=device)
    act_buf = torch.zeros((rollout_len, num_envs, act_dim), device=device)
    logp_buf = torch.zeros((rollout_len, num_envs), device=device)
    rew_buf = torch.zeros((rollout_len, num_envs), device=device)
    done_buf = torch.zeros((rollout_len, num_envs), dtype=torch.bool, device=device)
    val_buf = torch.zeros((rollout_len, num_envs), device=device)

    current_obs = torch.tensor(vec_env.reset(), dtype=torch.float32, device=device)
    start_time = time.time()
    total_steps = 0

    print("=" * 80)
    print(f"  MODEL ARCHITECTURE:  {policy.arch_name.upper()} ({total_params:,} total params, {actor_params:,} actor)")
    print(f"  DOMAIN RANDOMIZATION:{args.domain_rand} (perturb_prob={args.perturb_prob})")
    print(f"  PPO CONFIG:          {num_envs} envs x {rollout_len} steps = {batch_size} batch per iter")
    print(f"  TOTAL ITERATIONS:    {args.iterations} (~ {args.iterations * batch_size:,} steps)")
    print(f"  VERSION TAG:         {args.version_tag}")
    print("=" * 80)

    for iteration in range(1, args.iterations + 1):
        iter_start = time.time()

        # 1. Collect Rollout
        policy.eval()
        for step in range(rollout_len):
            total_steps += num_envs
            obs_buf[step] = current_obs

            with torch.no_grad():
                action, log_prob, _, value = policy.get_action_and_value(current_obs)

            act_buf[step] = action
            logp_buf[step] = log_prob
            val_buf[step] = value

            # Step physical environments
            act_np = action.cpu().numpy()
            next_obs_np, rewards_np, term_np, trunc_np, _ = vec_env.step(act_np)
            dones_np = term_np | trunc_np

            rew_buf[step] = torch.tensor(rewards_np, dtype=torch.float32, device=device)
            done_buf[step] = torch.tensor(dones_np, dtype=torch.bool, device=device)
            current_obs = torch.tensor(next_obs_np, dtype=torch.float32, device=device)

        # 2. Compute Advantages with GAE
        with torch.no_grad():
            next_val = policy.get_value(current_obs)
            advantages, returns = compute_gae(
                rew_buf, val_buf, done_buf, next_val, gamma=args.gamma, lam=args.gae_lambda
            )

        # Flatten batch
        b_obs = obs_buf.reshape(-1, obs_dim)
        b_act = act_buf.reshape(-1, act_dim)
        b_logp = logp_buf.reshape(-1)
        b_adv = advantages.reshape(-1)
        b_ret = returns.reshape(-1)

        # Normalize advantages
        b_adv = (b_adv - b_adv.mean()) / (b_adv.std() + 1e-8)

        # 3. PPO Epochs
        policy.train()
        clip_fracs = []
        for _ in range(args.ppo_epochs):
            indices = np.random.permutation(batch_size)
            for start_idx in range(0, batch_size, minibatch_size):
                end_idx = start_idx + minibatch_size
                mb_idx = indices[start_idx:end_idx]

                _, new_logp, entropy, new_val = policy.get_action_and_value(b_obs[mb_idx], b_act[mb_idx])
                log_ratio = new_logp - b_logp[mb_idx]
                ratio = log_ratio.exp()

                with torch.no_grad():
                    clip_frac = ((ratio - 1.0).abs() > args.clip_coef).float().mean().item()
                    clip_fracs.append(clip_frac)

                # Policy Loss
                mb_adv = b_adv[mb_idx]
                pg_loss1 = -mb_adv * ratio
                pg_loss2 = -mb_adv * torch.clamp(ratio, 1.0 - args.clip_coef, 1.0 + args.clip_coef)
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()

                # Value Loss
                v_loss = 0.5 * ((new_val - b_ret[mb_idx]) ** 2).mean()

                # Entropy Loss
                entropy_loss = entropy.mean()

                loss = pg_loss - args.ent_coef * entropy_loss + args.vf_coef * v_loss

                # Teacher Distillation Loss
                if teacher_sess is not None and getattr(args, "distill_coef", 0.0) > 0.0:
                    mb_obs_np = b_obs[mb_idx].cpu().numpy()
                    teacher_acts_np = teacher_sess.run(None, {"obs": mb_obs_np})[0]
                    t_target = torch.tensor(teacher_acts_np, dtype=torch.float32, device=device)
                    distill_loss = nn.functional.mse_loss(policy.actor_mean(b_obs[mb_idx]), t_target)
                    loss = loss + args.distill_coef * distill_loss

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(policy.parameters(), args.max_grad_norm)
                optimizer.step()

        scheduler.step()

        iter_dt = time.time() - iter_start
        sps = batch_size / iter_dt
        avg_rew = rew_buf.mean().item()

        if iteration % args.log_interval == 0 or iteration == args.iterations:
            print(f"Iter {iteration:3d}/{args.iterations} | "
                  f"Avg Reward: {avg_rew:6.2f} | "
                  f"SPS: {sps:6.0f} | "
                  f"Clip Frac: {np.mean(clip_fracs):.3f} | "
                  f"Elapsed: {time.time() - start_time:5.1f}s")

    vec_env.close()

    # Determine Versioned File Paths
    version = args.version_tag
    versioned_onnx_name = f"mico_policy_{version}.onnx"
    versioned_onnx_path = REPO / "src" / "humanoid_control" / "models" / versioned_onnx_name
    torch_path = versioned_onnx_path.with_suffix(".pt")

    # Export Model
    policy = policy.cpu()
    export_policy_onnx(policy, versioned_onnx_path)
    torch.save(policy.state_dict(), torch_path)
    print(f"Torch model saved to: {torch_path}")

    # Register into Model Registry
    desc = f"{policy.arch_name.upper()} ({actor_params:,} actor params, {args.iterations} iters, Metal MPS)"
    reg_mod.register_model(
        versioned_onnx_path,
        version=version,
        description=desc,
        arch=policy.arch_name,
        iterations=args.iterations,
        extra_meta={"residual": args.residual},
        activate=args.activate,
    )

    print("=" * 80)
    print(f"[*] Training finished successfully in {time.time() - start_time:.1f} seconds!")
    print("=" * 80)


def parse_args():
    parser = argparse.ArgumentParser(description="Train Mico PPO Locomotion")
    parser.add_argument("--arch", type=str, default="titan_transformer",
                        choices=["perceptive_transformer", "perceptive_titan",
                                 "titan_transformer", "transformer", "deep_res", "mlp"],
                        help="Network architecture")
    parser.add_argument("--perception", action="store_true", default=False,
                        help="Enable hardware-grounded 60-dim perception layer (FSR + ToF + Mag + Telemetry)")
    parser.add_argument("--hidden-dim", type=int, default=512, help="Hidden dimension for deep_res / transformer")
    parser.add_argument("--num-blocks", type=int, default=4, help="Number of residual blocks for deep_res")
    parser.add_argument("--num-layers", type=int, default=2, help="Number of transformer layers")
    parser.add_argument("--num-heads", type=int, default=4, help="Number of transformer attention heads")
    parser.add_argument("--num-envs", type=int, default=16, help="Parallel environments")
    parser.add_argument("--rollout-length", type=int, default=64, help="Rollout length per env")
    parser.add_argument("--iterations", type=int, default=500, help="Training iterations")
    parser.add_argument("--minibatch-size", type=int, default=512, help="Minibatch size")
    parser.add_argument("--ppo-epochs", type=int, default=4, help="PPO update epochs per iteration")
    parser.add_argument("--lr", type=float, default=2e-4, help="Learning rate")
    parser.add_argument("--gamma", type=float, default=0.99, help="Discount factor")
    parser.add_argument("--gae-lambda", type=float, default=0.95, help="GAE lambda")
    parser.add_argument("--clip-coef", type=float, default=0.2, help="PPO clip coefficient")
    parser.add_argument("--ent-coef", type=float, default=0.001, help="Entropy coefficient")
    parser.add_argument("--vf-coef", type=float, default=0.5, help="Value loss coefficient")
    parser.add_argument("--max-grad-norm", type=float, default=0.5, help="Gradient clipping")
    parser.add_argument("--domain-rand", action="store_true", help="Enable push disturbances and friction shock")
    parser.add_argument("--perturb-prob", type=float, default=0.06, help="Probability of kick perturbation per step")
    parser.add_argument("--obstacle-rand", action="store_true", help="Enable obstacle hurdle domain randomization")
    parser.add_argument("--terrain-roughness", type=float, default=0.0, help="Terrain micro-bump roughness in meters")
    parser.add_argument("--resume-checkpoint", type=str, default=None,
                        help="Path to existing PyTorch .pt checkpoint to resume fine-tuning from")
    parser.add_argument("--teacher-onnx", type=str, default=None, help="Teacher ONNX model for distillation")
    parser.add_argument("--distill-coef", type=float, default=0.0, help="Distillation loss coefficient")
    parser.add_argument("--residual", action="store_true", help="Enable residual reinforcement learning on biped gait")
    parser.add_argument("--straight-line", action="store_true",
                        help="Lock straight track (y=0, yaw=0) and upright spine")
    parser.add_argument("--target-speed", type=float, default=None,
                        help="Target forward speed in m/s (e.g. 0.35, 0.50)")
    parser.add_argument("--use-phase", action="store_true", help="Enable cyclic phase clock in observations (47-dim)")
    parser.add_argument("--version-tag", type=str, default="v9.0_titan_transformer",
                        help="Version tag for registration")
    parser.add_argument("--activate", action="store_true", help="Immediately activate the exported model")
    parser.add_argument("--log-interval", type=int, default=25, help="Log interval in iterations")
    parser.add_argument("--cpu", action="store_true", help="Force CPU training instead of MPS")
    return parser.parse_args()


if __name__ == "__main__":
    train(parse_args())
