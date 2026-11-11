#!/usr/bin/env python3
"""Model Registry and Version Management for Mico Humanoid RL Policies.

Provides semantic versioning, ONNX metadata embedding/inspection, and active model switching
(symlink management) for rolling forward and rolling back models safely.

Usage:
    # List all registered policy versions
    python3 src/humanoid_control/scripts/model_registry.py list

    # Inspect metadata embedded inside an ONNX policy
    python3 src/humanoid_control/scripts/model_registry.py inspect src/humanoid_control/models/mico_policy.onnx

    # Activate (switch or rollback) to a specific version
    python3 src/humanoid_control/scripts/model_registry.py activate v2.0_deep_500iter

    # Register a new model version
    python3 src/humanoid_control/scripts/model_registry.py register \
        src/humanoid_control/models/mico_policy_v2.0.onnx \
        --version v2.0_deep_500iter \
        --desc "Deep ResNet 2.6M params trained for 500 iters with kick disturbance"
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path

try:
    import onnx
except ImportError:
    onnx = None

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
MODELS_DIR = REPO / "software" / "ai" / "models" if (REPO / "software" / "ai" / "models").exists() else REPO / "models"
REGISTRY_FILE = MODELS_DIR / "model_registry.json"
ACTIVE_SYMLINK = MODELS_DIR / "mico_policy.onnx"


def load_registry():
    """Load registry JSON or initialize empty registry."""
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    if REGISTRY_FILE.exists():
        try:
            with open(REGISTRY_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"[!] Warning: Failed to parse {REGISTRY_FILE}: {e}. Creating new registry.")
    return {"active_version": None, "models": {}}


def save_registry(registry_data):
    """Save registry data back to JSON."""
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    with open(REGISTRY_FILE, "w", encoding="utf-8") as f:
        json.dump(registry_data, f, indent=2)


def get_onnx_metadata(onnx_path):
    """Extract embedded metadata key-values from an ONNX model file."""
    if onnx is None:
        return {}, None
    model = onnx.load(str(onnx_path))
    meta = {}
    for prop in model.metadata_props:
        meta[prop.key] = prop.value
    if model.doc_string:
        meta["doc_string"] = model.doc_string
    return meta, model


def set_onnx_metadata(onnx_path, metadata_dict, doc_string=None):
    """Embed or update metadata properties directly inside an ONNX file."""
    if onnx is None:
        raise RuntimeError("onnx package is required to write ONNX metadata")
    model = onnx.load(str(onnx_path))

    existing_keys = {prop.key: prop for prop in model.metadata_props}
    for k, v in metadata_dict.items():
        if k in existing_keys:
            existing_keys[k].value = str(v)
        else:
            prop = model.metadata_props.add()
            prop.key = str(k)
            prop.value = str(v)

    if doc_string:
        model.doc_string = doc_string

    onnx.save(model, str(onnx_path))


def count_onnx_parameters(onnx_model):
    """Calculate total number of trainable weights/parameters from initializers."""
    total_params = 0
    for init in onnx_model.graph.initializer:
        dims = list(init.dims)
        param_count = 1
        for d in dims:
            param_count *= d
        total_params += param_count
    return total_params


def register_model(model_path, version, description="", arch="deep_res",
                   iterations=0, extra_meta=None, activate=False):
    """Register an ONNX model into the version registry and embed metadata."""
    model_path = Path(model_path).resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"ONNX model file not found: {model_path}")

    # Inspect ONNX model
    _, model = get_onnx_metadata(model_path)
    param_count = count_onnx_parameters(model)
    file_size_kb = model_path.stat().st_size / 1024.0

    meta_to_embed = {
        "model_version": version,
        "architecture": arch,
        "parameters": str(param_count),
        "iterations": str(iterations),
        "created_at": datetime.now().isoformat(),
        "file_size_kb": f"{file_size_kb:.1f}",
        "description": description,
    }
    if extra_meta:
        meta_to_embed.update(extra_meta)

    # Embed metadata into ONNX binary
    set_onnx_metadata(model_path, meta_to_embed, doc_string=f"Mico Policy {version} ({arch})")

    # Update JSON registry
    registry = load_registry()
    if model_path.is_relative_to(MODELS_DIR):
        relative_path = str(model_path.relative_to(MODELS_DIR))
    else:
        relative_path = str(model_path)

    registry["models"][version] = {
        "version": version,
        "relative_path": relative_path,
        "architecture": arch,
        "parameters": param_count,
        "iterations": iterations,
        "file_size_kb": round(file_size_kb, 1),
        "created_at": meta_to_embed["created_at"],
        "description": description,
        "residual": extra_meta.get("residual", False) if extra_meta else False,
        "benchmark_score": extra_meta.get("benchmark_score", None) if extra_meta else None,
    }

    if activate or registry.get("active_version") is None:
        activate_model(version, registry)
    else:
        save_registry(registry)

    print(f"[✓] Registered policy version '{version}': {param_count:,} params, {file_size_kb:.1f} KB")


def activate_model(version, registry=None):
    """Point the active mico_policy.onnx symlink to the specified version."""
    if registry is None:
        registry = load_registry()

    if version not in registry["models"]:
        available = list(registry["models"].keys())
        raise ValueError(f"Version '{version}' not found in registry. Available: {available}")

    target_rel = registry["models"][version]["relative_path"]
    target_path = (MODELS_DIR / target_rel).resolve()

    if not target_path.exists():
        raise FileNotFoundError(f"Target model file does not exist: {target_path}")

    # Remove existing active symlink or file
    if ACTIVE_SYMLINK.is_symlink() or ACTIVE_SYMLINK.exists():
        ACTIVE_SYMLINK.unlink()

    # Create relative symlink
    try:
        os.symlink(target_path.name, str(ACTIVE_SYMLINK))
    except Exception:
        # Fallback to copy if symlinks not supported on filesystem
        import shutil
        shutil.copy2(target_path, ACTIVE_SYMLINK)

    registry["active_version"] = version
    save_registry(registry)
    print(f"[✓] Activated policy version '{version}' -> {ACTIVE_SYMLINK.name}")


def list_models():
    """Display all registered models in a formatted table."""
    registry = load_registry()
    models = registry.get("models", {})
    active_ver = registry.get("active_version")

    print("\n" + "=" * 90)
    print(f"{'ACTIVE':<8} {'VERSION':<22} {'ARCH':<12} {'PARAMS':<12} {'ITERS':<8} {'SIZE':<10} {'DESCRIPTION'}")
    print("=" * 90)

    if not models:
        print("  No models registered yet.")
    else:
        for ver, info in sorted(models.items()):
            is_act = "[ACTIVE]" if ver == active_ver else ""
            params_str = f"{info.get('parameters', 0):,}"
            size_str = f"{info.get('file_size_kb', 0):.1f} KB"
            arch_str = info.get("architecture", "unknown")
            iters = str(info.get("iterations", "-"))
            desc = info.get("description", "")
            print(f"{is_act:<8} {ver:<22} {arch_str:<12} {params_str:<12} {iters:<8} {size_str:<10} {desc}")
    print("=" * 90 + "\n")


def inspect_model(model_path):
    """Inspect ONNX inputs, outputs, nodes, and metadata."""
    model_path = Path(model_path)
    if not model_path.exists():
        raise FileNotFoundError(f"File not found: {model_path}")

    meta, model = get_onnx_metadata(model_path)
    total_params = count_onnx_parameters(model)

    print("\n" + "=" * 70)
    print(f"ONNX Model Inspection: {model_path.name}")
    print("=" * 70)
    print(f"File Size:       {model_path.stat().st_size / 1024:.1f} KB")
    print(f"Total Params:    {total_params:,}")
    print(f"IR Version:      {model.ir_version}")
    print(f"Producer:        {model.producer_name} {model.producer_version}")
    if model.doc_string:
        print(f"Doc String:      {model.doc_string}")

    print("\nInputs:")
    for inp in model.graph.input:
        shape = [d.dim_value or d.dim_param for d in inp.type.tensor_type.shape.dim]
        print(f"  - {inp.name}: shape={shape}")

    print("\nOutputs:")
    for out in model.graph.output:
        shape = [d.dim_value or d.dim_param for d in out.type.tensor_type.shape.dim]
        print(f"  - {out.name}: shape={shape}")

    print("\nEmbedded Metadata Properties:")
    if meta:
        for k, v in meta.items():
            if k != "doc_string":
                print(f"  {k:<20}: {v}")
    else:
        print("  (No custom metadata found)")
    print("=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Mico Humanoid RL Model Registry & Version Manager")
    subparsers = parser.add_subparsers(dest="subcommand", help="Subcommand to execute")

    # list
    subparsers.add_parser("list", help="List all registered model versions")

    # activate / rollback
    act_parser = subparsers.add_parser("activate", help="Activate or rollback to a policy version")
    act_parser.add_argument("version", type=str, help="Version tag to activate")

    # register
    reg_parser = subparsers.add_parser("register", help="Register a model file into the registry")
    reg_parser.add_argument("model_path", type=str, help="Path to the ONNX file")
    reg_parser.add_argument("--version", "-v", type=str, required=True, help="Version tag (e.g. v2.0_deep_500iter)")
    reg_parser.add_argument("--desc", "-d", type=str, default="", help="Description")
    reg_parser.add_argument("--arch", type=str, default="deep_res", help="Architecture name")
    reg_parser.add_argument("--iterations", type=int, default=0, help="Training iterations")
    reg_parser.add_argument("--activate", action="store_true", help="Immediately set as active model")

    # inspect
    ins_parser = subparsers.add_parser("inspect", help="Inspect an ONNX file's metadata and structure")
    ins_parser.add_argument("model_path", type=str, help="Path to ONNX file")

    args = parser.parse_args()

    if args.subcommand == "list":
        list_models()
    elif args.subcommand == "activate":
        activate_model(args.version)
    elif args.subcommand == "register":
        register_model(
            args.model_path,
            version=args.version,
            description=args.desc,
            arch=args.arch,
            iterations=args.iterations,
            activate=args.activate,
        )
    elif args.subcommand == "inspect":
        inspect_model(args.model_path)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
