#!/usr/bin/env python3
"""Persistent Evolution Knowledge Base & Adaptive Memory System.

Stores validated successes (bài học đúng), avoided mistakes (bài học sai), and
actionable corrective heuristics (cách khắc phục) across evolutionary generations.
Ensures subsequent generations inherit accumulated wisdom and never repeat past errors.
"""
from datetime import datetime
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
KB_FILE = REPO / "software" / "ai" / "models" / "evolution_knowledge_base.json"


class EvolutionKnowledgeBase:
    """Persistent storage and retrieval engine for evolutionary lessons learned."""

    def __init__(self, kb_path=None):
        self.kb_path = Path(kb_path) if kb_path else KB_FILE
        self.kb_path.parent.mkdir(parents=True, exist_ok=True)
        self.data = self._load()

    def _default_kb(self):
        return {
            "version": "1.0.0",
            "last_updated": datetime.now().isoformat(),
            "champion_tag": "v17.0_titan_transformer_wholebody_10M_gen1",
            "champion_score": 91.0,
            "total_cycles_recorded": 1,
            "active_heuristics": {
                "arch": "titan_transformer",
                "hidden_dim": 512,
                "num_blocks": 6,
                "num_layers": 3,
                "num_heads": 4,
                "action_scale": 0.08,
                "min_iterations": 80,
                "perturb_prob": 0.065,
                "lr": "7e-5",
                "clip_coef": "0.15",
                "straight_line": False,
            },
            "validated_successes": [
                {
                    "id": "VAL-01",
                    "lesson": "Pre-LayerNorm with direct residual shortcut prevents attention saturation during PPO.",
                    "origin_model": "v8.3_transformer_hybrid",
                    "impact": "+43.8 pts stability gain",
                },
                {
                    "id": "VAL-02",
                    "lesson": "Sigmoid FSR normalization prevents raw contact spikes from drowning proprioception.",
                    "origin_model": "v16.0_wholebody_perceptive",
                    "impact": "+66.1 pts perception robustness",
                },
                {
                    "id": "VAL-03",
                    "lesson": "Decoupled 6-Head specialty architecture allows independent sagittal/lateral balance.",
                    "origin_model": "v16.0_gen1",
                    "impact": "Decoupled Whole-Body 20 DOF + 12 Adaptive Controls",
                },
                {
                    "id": "VAL-04",
                    "lesson": "10.01M Titan Transformer (6 blocks, 3 layers) achieves < 1.0 ms P99 latency on M4.",
                    "origin_model": "v17.0_gen1",
                    "impact": "987 μs P99 latency (> 11x real-time headroom)",
                },
            ],
            "avoided_mistakes": [
                {
                    "id": "ERR-01",
                    "mistake": "Setting action_scale > 0.10 when resuming pre-trained weights causes joint flaring.",
                    "observed_in": "mico_rl_env legacy straight_line override",
                    "mitigation": "Strictly lock action_scale = 0.08 across all training and evaluation runs.",
                },
                {
                    "id": "ERR-02",
                    "mistake": ("Running < 60 iterations on 10M Transformer policy causes "
                                "PPO gradient under-convergence."),
                    "observed_in": "v16.0_gen2 (20 iters -> 31.4 pts)",
                    "mitigation": "Enforce minimum 80 iterations per evolutionary cycle.",
                },
                {
                    "id": "ERR-03",
                    "mistake": "Narrow foot stance (y < 4 cm) leaves zero margin for ankle roll torque against kicks.",
                    "observed_in": "v3.1, v5.5 (survival < 50%)",
                    "mitigation": "Dynamic arm flaring + hip roll pre-tension via decoupled lateral head.",
                },
            ],
            "persistent_weaknesses": [
                {
                    "criterion": "Kick Disturbance",
                    "current_survival": 62.5,
                    "target_survival": 85.0,
                    "consecutive_failures": 3,
                    "action_plan": "Incrementally scale perturb_prob from 0.065 to 0.080 and boost hip damping.",
                },
                {
                    "criterion": "Drop Impact & Tilt",
                    "current_survival": 66.7,
                    "target_survival": 85.0,
                    "consecutive_failures": 3,
                    "action_plan": "Reinforce spine pitch posture compliance during landing shock.",
                },
                {
                    "criterion": "Velocity Reversal Agility",
                    "current_rmse": 0.213,
                    "target_rmse": 0.150,
                    "consecutive_failures": 2,
                    "action_plan": "Tune fine learning rate (7e-5) with cosine annealing.",
                },
            ],
            "cycle_history": [
                {
                    "cycle": 1,
                    "tag": "v17.0_titan_transformer_wholebody_10M_gen1",
                    "score": 91.0,
                    "delta": 0.0,
                    "kick_surv": 62.5,
                    "drop_surv": 66.7,
                    "rev_rmse": 0.213,
                    "status": "CHAMPION_BASELINE",
                }
            ],
        }

    def _load(self):
        if self.kb_path.exists():
            try:
                with open(self.kb_path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                pass
        kb = self._default_kb()
        self._save(kb)
        return kb

    def _save(self, data=None):
        if data is None:
            data = self.data
        data["last_updated"] = datetime.now().isoformat()
        with open(self.kb_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def record_cycle_result(self, tag, score, report, champ_score, fixes_applied):
        """Analyze evaluation report, update lessons, and persist into knowledge base."""
        self.data["total_cycles_recorded"] = self.data.get("total_cycles_recorded", 0) + 1
        delta = score - champ_score

        kick_s = report.get("kick", {}).get("survival_rate", 0.0) if report else 0.0
        drop_s = report.get("drop", {}).get("drop_survival_rate", 0.0) if report else 0.0
        rev_rmse = report.get("reversal", {}).get("rmse_vx", 0.0) if report else 0.0

        # Update cycle history
        entry = {
            "cycle": self.data["total_cycles_recorded"],
            "tag": tag,
            "score": score,
            "delta": round(delta, 2),
            "kick_surv": kick_s,
            "drop_surv": drop_s,
            "rev_rmse": rev_rmse,
            "fixes_tested": fixes_applied,
            "status": "IMPROVED_CHAMPION" if delta > 0 else ("TIED" if abs(delta) < 1e-4 else "REGRESSED"),
        }
        self.data.setdefault("cycle_history", []).append(entry)

        # Check for positive discovery
        if delta > 0:
            self.data["champion_tag"] = tag
            self.data["champion_score"] = score
            self.data["validated_successes"].append({
                "id": f"VAL-{len(self.data['validated_successes']) + 1:02d}",
                "lesson": f"Adaptive recipe ({', '.join(fixes_applied)}) surpassed previous best.",
                "origin_model": tag,
                "impact": f"+{delta:.1f} pts gain (new score: {score:.1f})",
            })
        elif delta < -5.0:
            # Significant regression: register avoided mistake
            self.data["avoided_mistakes"].append({
                "id": f"ERR-{len(self.data['avoided_mistakes']) + 1:02d}",
                "mistake": f"Configuration for '{tag}' led to score drop (-{abs(delta):.1f} pts).",
                "observed_in": tag,
                "mitigation": "Safeguard rollback activated; revert to champion weights.",
            })

        # Update persistent weakness tracking
        for pw in self.data.get("persistent_weaknesses", []):
            crit = pw.get("criterion", "")
            if "Kick" in crit:
                pw["current_survival"] = kick_s
                if kick_s < pw.get("target_survival", 85.0):
                    pw["consecutive_failures"] = pw.get("consecutive_failures", 0) + 1
                    # Auto-escalate perturbation heuristic
                    curr_p = self.data["active_heuristics"].get("perturb_prob", 0.065)
                    self.data["active_heuristics"]["perturb_prob"] = min(0.085, curr_p + 0.005)
                else:
                    pw["consecutive_failures"] = 0
            elif "Drop" in crit:
                pw["current_survival"] = drop_s
                if drop_s < pw.get("target_survival", 85.0):
                    pw["consecutive_failures"] = pw.get("consecutive_failures", 0) + 1
                else:
                    pw["consecutive_failures"] = 0
            elif "Velocity" in crit:
                pw["current_rmse"] = rev_rmse
                if rev_rmse > pw.get("target_rmse", 0.150):
                    pw["consecutive_failures"] = pw.get("consecutive_failures", 0) + 1
                else:
                    pw["consecutive_failures"] = 0

        self._save()

    def get_adaptive_recipe(self, gen_idx, chain_ckpt_path):
        """Synthesize next generation configuration leveraging all accumulated lessons."""
        heuristics = self.data.get("active_heuristics", {})
        perturb_prob = heuristics.get("perturb_prob", 0.065)
        lr = heuristics.get("lr", "7e-5")
        min_iters = heuristics.get("min_iterations", 80)

        # Check persistent weaknesses to inject targeted fixes
        fixes = []
        for pw in self.data.get("persistent_weaknesses", []):
            if pw.get("consecutive_failures", 0) >= 2:
                crit = pw.get("criterion", "")
                if "Kick" in crit:
                    fixes.append(f"EscalateKick(p={perturb_prob:.3f})")
                elif "Drop" in crit:
                    fixes.append("LandingShockCompliance")
                elif "Velocity" in crit:
                    fixes.append(f"FineLR({lr})")

        desc_fixes = ", ".join(fixes) if fixes else "Continuous 10M Titan Refinement"

        return {
            "tag": f"v17.0_titan_transformer_wholebody_10M_gen{gen_idx}",
            "arch": "titan_transformer",
            "hidden_dim": heuristics.get("hidden_dim", 512),
            "num_blocks": heuristics.get("num_blocks", 6),
            "num_layers": heuristics.get("num_layers", 3),
            "num_heads": heuristics.get("num_heads", 4),
            "iterations": min_iters,
            "num_envs": 8,
            "rollout_length": heuristics.get("rollout_length", 32),
            "minibatch_size": heuristics.get("minibatch_size", 128),
            "perturb_prob": perturb_prob,
            "residual": True,
            "use_phase": True,
            "perception": True,
            "straight_line": heuristics.get("straight_line", False),
            "lr": lr,
            "resume_checkpoint": str(chain_ckpt_path),
            "desc": f"Whole-Body 64-in/32-out 10.01M Titan Transformer Gen {gen_idx} (Knowledge: {desc_fixes})",
            "fixes_applied": fixes,
        }
