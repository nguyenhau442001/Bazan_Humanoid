"""3D Cable Routing and Wire Harness Model for Bazan Humanoid.

Generates physically accurate 3D CAD models of all robot wiring harnesses:
  - Leg servo cable bundles (L & R): runs up from ankle through shin/thigh tie slots to pelvis
  - Arm servo cable bundles (L & R): elbow -> bicep -> shoulder mount into torso
  - Camera CSI ribbon cable: Pi Camera -> neck notch -> Radxa Zero CSI port on Deck 2
  - Torso main power harness: LiPo battery -> INA226 current sensor -> UBEC 6V & Buck 5V

All cables pass through the designed mechanical channels, tie slots, and deck cutouts.
"""
import cadquery as cq
import numpy as np

from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCP.gp import gp_Ax2, gp_Circ, gp_Dir, gp_Pnt


def make_pipe(points, radius=1.2):
    """Create a smooth 3D pipe solid along a 3D spline passing through points."""
    cq_pts = [cq.Vector(*p) for p in points]
    spline_edge = cq.Edge.makeSpline(cq_pts)
    wire = cq.Wire.assembleEdges([spline_edge])

    # Circular profile at start perpendicular to initial tangent
    p0 = cq_pts[0]
    p1 = cq_pts[1]
    tan = (p1 - p0).normalized()
    circ = gp_Circ(gp_Ax2(gp_Pnt(p0.x, p0.y, p0.z), gp_Dir(tan.x, tan.y, tan.z)), radius)
    circ_edge = BRepBuilderAPI_MakeEdge(circ).Edge()
    circ_wire = BRepBuilderAPI_MakeWire(circ_edge).Wire()

    pipe = BRepOffsetAPI_MakePipe(wire.wrapped, circ_wire).Shape()
    return cq.Solid(pipe)


def make_ribbon(points, width=10.0, thickness=0.4):
    """Create a flat flex ribbon cable (like camera CSI FFC)."""
    # For robust CAD geometry, ribbon is modeled as a sleek flat capsule pipe
    return make_pipe(points, radius=width / 5.0)


def leg_cable_points(side="L"):
    """Key 3D routing points for one leg harness."""
    sign = 1.0 if side == "L" else -1.0
    y_leg = 28.0 * sign

    # Route: foot ankle_roll -> ankle pitch crossing -> shin spine ties -> knee crossing
    #        -> thigh spine tie -> hip pitch crossing -> hip roll crossing -> yaw bracket tie
    #        -> hip mount window -> pelvis plate cable notch -> Deck 1 PCA
    pts = [
        np.array([-12.0, y_leg + 2.0 * sign, -135.0]),   # Ankle roll servo exit
        np.array([-15.0, y_leg, -115.0]),                # Slack loop behind ankle
        np.array([-14.0, y_leg, -96.0]),                 # Ankle pitch crossing
        np.array([12.0, y_leg, -78.0]),                  # Shin lower tie slot
        np.array([12.0, y_leg, -60.0]),                  # Shin upper tie slot
        np.array([-14.0, y_leg, -48.0]),                 # Knee crossing
        np.array([-12.0, y_leg, -30.0]),                 # Thigh tie slot
        np.array([-14.0, y_leg, 0.0]),                   # Hip pitch crossing
        np.array([-24.0, y_leg, 20.0]),                  # Hip roll crossing
        np.array([-20.4, y_leg, 30.0]),                  # Yaw bracket tie slot
        np.array([-22.0, y_leg, 55.0]),                  # Yaw top crossing
        np.array([-22.0, y_leg * 0.8, 73.0]),            # Hip mount window
        np.array([-20.0, y_leg * 0.5, 95.0]),            # Through pelvis floor notch
        np.array([10.0, y_leg * 0.4, 118.0]),            # Deck 1 entry towards PCA
    ]
    return pts


def arm_cable_points(side="L"):
    """Routing points for one arm harness (elbow -> upper arm -> shoulder -> torso)."""
    sign = 1.0 if side == "L" else -1.0
    y_arm = 72.0 * sign

    pts = [
        np.array([0.0, y_arm + 2.0 * sign, 105.0]),     # Forearm elbow servo exit
        np.array([4.0, y_arm, 120.0]),                  # Bicep channel
        np.array([0.0, y_arm - 4.0 * sign, 138.0]),     # Shoulder roll pivot loop
        np.array([0.0, y_arm - 12.0 * sign, 150.0]),    # Shoulder pitch horn pass
        np.array([0.0, 42.0 * sign, 152.0]),            # Through shoulder mount
        np.array([10.0, 25.0 * sign, 130.0]),           # Into Torso Deck 1 (PCA #2)
    ]
    return pts


def camera_ribbon_points():
    """Routing points for the Pi Camera CSI flat flex cable down to Radxa Zero."""
    pts = [
        np.array([22.0, 0.0, 250.0]),     # Camera back connector on cam_mount
        np.array([12.0, 0.0, 238.0]),     # Curve around wedge
        np.array([-2.0, 0.0, 222.0]),     # Through head deck center slot
        np.array([-10.0, 0.0, 200.0]),    # Behind neck pitch & yaw
        np.array([-18.0, 0.0, 178.0]),    # Through neck mount back cable notch
        np.array([5.0, 0.0, 162.0]),      # Forward onto Deck 2 Radxa Zero CSI port
    ]
    return pts


def power_harness_points():
    """Routing points for main 2S LiPo battery power cable to INA226 & UBEC."""
    pts = [
        np.array([-5.0, 0.0, 78.0]),      # LiPo battery XT30/JST connector on pelvis
        np.array([-15.0, 15.0, 95.0]),    # Rising towards Deck 1
        np.array([0.0, 30.0, 118.0]),     # Into INA226 current shunt sensor on Deck 1
        np.array([-22.0, 20.0, 122.0]),   # Branching to UBEC 6V 5A module
    ]
    return pts


def get_all_cables():
    """Return dictionary of all 3D cable solids with distinct CAD names."""
    cables = {}

    print("Generating 3D cable harnesses...")
    # 1. Left & Right Leg harnesses (12 servo leads + FSR wires bundled in mesh sleeve)
    cables["cable_harness_L_leg"] = make_pipe(leg_cable_points("L"), radius=1.6)
    cables["cable_harness_R_leg"] = make_pipe(leg_cable_points("R"), radius=1.6)

    # 2. Left & Right Arm harnesses (shoulder + elbow leads)
    cables["cable_harness_L_arm"] = make_pipe(arm_cable_points("L"), radius=1.2)
    cables["cable_harness_R_arm"] = make_pipe(arm_cable_points("R"), radius=1.2)

    # 3. Camera CSI ribbon cable (FFC)
    cables["cable_ribbon_camera_csi"] = make_ribbon(camera_ribbon_points(), width=8.0, thickness=0.4)

    # 4. Main Torso Power Harness (high-current 18 AWG silicone wires)
    cables["cable_harness_power_bus"] = make_pipe(power_harness_points(), radius=1.5)

    return cables


if __name__ == "__main__":
    c = get_all_cables()
    for name, solid in c.items():
        print(f"  - {name:28s} volume: {solid.Volume():8.1f} mm3, valid: {solid.isValid()}")
