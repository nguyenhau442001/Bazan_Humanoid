"""Export full Bazan Humanoid assembly to a single named STEP file.

The exported STEP file preserves:
  - Full hierarchical tree with named parts for every link, bracket, servo, horn, standoff, and board
  - Exact spatial positioning in zero pose
  - Distinct colors for visual inspection in CAD tools (Fusion 360, SolidWorks, FreeCAD, Onshape)

Usage:
    /Users/haunguyen/.ros2_venv/bin/python3 export_assembly.py
"""
import os
import time

import cadquery as cq

import body as B
import cabling
import leg as LEG
import robot as ROBOT
import sensors as SENS


def build_assembly(q=None, include_electronics=True, include_servos=True, include_cables=True):
    """Build a cq.Assembly containing all parts with human-readable names and colors."""
    print("Initializing Body and Leg models...")
    body = B.Body()
    leg = LEG.Leg()

    print("Assembling whole-body geometry...")
    parts = ROBOT.assemble(body, leg, q=q, include_legs=True)

    assy = cq.Assembly(name="Bazan_Humanoid_v4")

    # Color palette
    COLOR_PRINTED = cq.Color(0.85, 0.85, 0.88, 1.0)       # Light gray / white printed parts
    COLOR_ACCENT = cq.Color(0.95, 0.45, 0.10, 1.0)        # Orange accent (ears / joints)
    COLOR_SERVO = cq.Color(0.20, 0.22, 0.25, 1.0)         # Dark slate / metallic servo case
    COLOR_HORN = cq.Color(0.90, 0.90, 0.90, 1.0)          # White nylon horn
    COLOR_STANDOFF = cq.Color(0.80, 0.70, 0.20, 1.0)      # Brass / gold standoff
    COLOR_PCB = cq.Color(0.10, 0.55, 0.25, 1.0)           # Green PCB boards
    COLOR_BATT = cq.Color(0.15, 0.35, 0.85, 1.0)          # Blue LiPo battery
    COLOR_CAP = cq.Color(0.10, 0.20, 0.50, 1.0)           # Navy cap
    COLOR_CABLE = cq.Color(0.12, 0.12, 0.14, 1.0)         # Charcoal black braided wire harness
    COLOR_POWER = cq.Color(0.95, 0.30, 0.05, 1.0)         # Orange/red high-current power cable
    COLOR_RIBBON = cq.Color(0.92, 0.92, 0.95, 1.0)        # Light silver camera FFC ribbon

    for name, solid in parts.items():
        # Clean name for CAD software compatibility
        cad_name = name.replace(".", "_")

        # Skip bought parts if requested
        if not include_electronics and (cad_name.startswith("board_") or cad_name == "battery"):
            continue
        if not include_servos and (cad_name.find("servo_") != -1 or cad_name.find("horn_") != -1):
            continue

        # Assign color
        if "servo_" in cad_name:
            col = COLOR_SERVO
        elif "horn_" in cad_name:
            col = COLOR_HORN
        elif "cap_" in cad_name:
            col = COLOR_CAP
        elif cad_name.startswith("so35_") or cad_name.startswith("so40_"):
            col = COLOR_STANDOFF
        elif cad_name.startswith("board_"):
            col = COLOR_PCB
        elif cad_name == "battery":
            col = COLOR_BATT
        elif "ear" in cad_name or "accent" in cad_name:
            col = COLOR_ACCENT
        else:
            col = COLOR_PRINTED

        assy.add(solid, name=cad_name, color=col)

    # Add 3D cable harnesses
    if include_cables:
        cables = cabling.get_all_cables()
        for name, solid in cables.items():
            if "power" in name:
                col = COLOR_POWER
            elif "ribbon" in name or "camera" in name:
                col = COLOR_RIBBON
            else:
                col = COLOR_CABLE
            assy.add(solid, name=name, color=col)

    return assy, len(parts) + (len(cables) if include_cables else 0)


def main():
    t0 = time.time()
    out_dir = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out_dir, exist_ok=True)
    out_step = os.path.join(out_dir, "bazan_assembly.step")

    print("Generating full Bazan CAD assembly...")
    assy, n_parts = build_assembly(q=None, include_electronics=True, include_servos=True)
    print(f"Assembly built with {n_parts} parts. Exporting to {out_step}...")

    assy.save(out_step, "STEP")
    size_mb = os.path.getsize(out_step) / (1024 * 1024)
    dt = time.time() - t0

    print(f"SUCCESS: Exported {out_step} ({size_mb:.2f} MB) in {dt:.1f}s")
    print("\nAlso exporting sensors manifest alongside assembly...")
    manifest_path = os.path.join(out_dir, "sensors_manifest.json")
    SENS.export_sensors_json(manifest_path)
    print(f"SUCCESS: Sensors manifest saved to {manifest_path}")


if __name__ == "__main__":
    main()
