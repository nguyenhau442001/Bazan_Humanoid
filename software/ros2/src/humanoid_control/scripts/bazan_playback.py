#!/usr/bin/env python3
"""Kinematic playback of the Mico demo for RViz2 (mico_view.launch.py): no physics.

Publishes the mico_gait targets as sensor_msgs/JointState on /joint_states (50 Hz, joint order of
mico_gait.JOINTS) and odom -> base_link on /tf. The base pose comes from anchoring the stance foot:
forward kinematics from the URDF gives l_sole and r_sole in base_link, the lower foot is the stance foot
(1 mm hysteresis), and when the stance foot changes its world pose is locked flat on the floor (z = 0,
roll = pitch = 0, yaw kept). Every tick T_odom_base = T_odom_sole * inv(T_base_sole), so a planted foot
never slides.

    ros2 run humanoid_control mico_playback.py --ros-args -p robot_description:="$(xacro mico.urdf.xacro)"
    ros2 run humanoid_control mico_playback.py --ros-args -p motion:=stand -p robot_description:=...

The kinematics (Kinematics, SoleAnchor) only need numpy, so they can be tested without ROS.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
try:
    import bazan_gait as gait
except ImportError:
    import mico_gait as gait  # noqa: E402

SOLES = ("l_sole", "r_sole")


def rpy_matrix(r, p, y):
    """URDF convention: R = Rz(y) * Ry(p) * Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * kx @ kx


def transform(rot=None, xyz=(0.0, 0.0, 0.0)):
    t = np.eye(4)
    if rot is not None:
        t[:3, :3] = rot
    t[:3, 3] = xyz
    return t


def inverse(t):
    r = t[:3, :3].T
    return transform(r, -r @ t[:3, 3])


def yaw_of(t):
    return math.atan2(t[1, 0], t[0, 0])


def flatten(t):
    """Same x, y and yaw, on the floor (z = 0) and level (roll = pitch = 0)."""
    return transform(rpy_matrix(0.0, 0.0, yaw_of(t)), (t[0, 3], t[1, 3], 0.0))


def quaternion(rot):
    """(x, y, z, w) of a rotation matrix (Shepperd: divide by the largest component)."""
    m = rot
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 2 * math.sqrt(1 + tr)
        return (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
    v = [0.0, 0.0, 0.0]
    v[i] = s / 4
    v[j] = (m[j, i] + m[i, j]) / s
    v[k] = (m[k, i] + m[i, k]) / s
    return v[0], v[1], v[2], (m[k, j] - m[j, k]) / s


class Kinematics:
    """Forward kinematics read from a URDF (or the xacro: joints there are plain XML): origin xyz / rpy and
    axis of every joint, revolute and fixed."""

    def __init__(self, urdf_text, root="base_link"):
        self.root = root
        self.parent_joint = {}
        for j in ET.fromstring(urdf_text).findall("joint"):      # not the <ros2_control> joints
            o, a = j.find("origin"), j.find("axis")
            xyz = [float(v) for v in o.get("xyz", "0 0 0").split()] if o is not None else [0.0] * 3
            rpy = [float(v) for v in o.get("rpy", "0 0 0").split()] if o is not None else [0.0] * 3
            axis = [float(v) for v in a.get("xyz").split()] if a is not None else [1.0, 0.0, 0.0]
            self.parent_joint[j.find("child").get("link")] = (
                j.get("name"), j.get("type"), j.find("parent").get("link"), transform(rpy_matrix(*rpy), xyz), axis)

    def chain(self, link):
        joints = []
        while link != self.root:
            if link not in self.parent_joint:
                raise KeyError(f"{link} is not below {self.root}")
            joints.append(self.parent_joint[link])
            link = self.parent_joint[link][2]
        return joints[::-1]

    def pose(self, q, link):
        """4x4 pose of link in the root frame for joint angles q (dict, missing joints are 0)."""
        t = np.eye(4)
        for name, kind, _, origin, axis in self.chain(link):
            t = t @ origin
            if kind in ("revolute", "continuous"):
                t = t @ transform(axis_angle(axis, q.get(name, 0.0)))
            elif kind == "prismatic":
                t = t @ transform(xyz=q.get(name, 0.0) * np.asarray(axis))
        return t


class SoleAnchor:
    """odom -> base_link from the planted foot. The stance foot keeps its world pose until the other foot
    is more than `hysteresis` lower; then the other foot becomes the stance foot, locked flat at z = 0."""

    def __init__(self, kin, hysteresis=0.001):
        self.kin = kin
        self.hysteresis = hysteresis
        self.stance = None
        self.odom_sole = None

    def soles(self, q):
        return {s: self.kin.pose(q, s) for s in SOLES}

    def reset(self, q):
        """Start with the lower foot flat on the floor; base_link then starts above the odom origin."""
        soles = self.soles(q)
        self.stance = "l_sole" if soles["l_sole"][2, 3] <= soles["r_sole"][2, 3] + 1e-9 else "r_sole"
        self.odom_sole = flatten(soles[self.stance])
        return self.update(q, soles)

    def update(self, q, soles=None):
        if soles is None:
            soles = self.soles(q)
        if self.stance is None:
            return self.reset(q)
        odom_base = self.odom_sole @ inverse(soles[self.stance])
        other = "r_sole" if self.stance == "l_sole" else "l_sole"
        other_world = odom_base @ soles[other]
        if other_world[2, 3] < -self.hysteresis:
            self.stance = other
            self.odom_sole = flatten(other_world)
            odom_base = self.odom_sole @ inverse(soles[other])
        return odom_base


def make_node():
    """ROS imports live here so the kinematics above can be imported without ROS (test_mico_playback.py)."""
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster

    class BazanPlayback(Node):
        def __init__(self):
            super().__init__("bazan_playback")
            self.declare_parameter("robot_description", "")
            self.declare_parameter("motion", "demo")
            self.declare_parameter("rate", 50.0)
            self.declare_parameter("odom_frame", "odom")
            self.declare_parameter("base_frame", "base_link")
            urdf = self.get_parameter("robot_description").value
            if not urdf:
                raise RuntimeError("parameter robot_description is empty")
            self.motion = self.get_parameter("motion").value
            if self.motion not in ("demo", "stand"):
                raise RuntimeError(f"motion must be demo or stand, not {self.motion!r}")
            self.anchor = SoleAnchor(Kinematics(urdf))
            self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.tf_pub = TransformBroadcaster(self)
            self.start = self.get_clock().now()
            self.last_t = None
            self.create_timer(1.0 / self.get_parameter("rate").value, self.tick)
            self.get_logger().info(f"motion {self.motion}: "
                                   + (f"walk, turn 60 deg, wave, looping every {gait.T_END:.1f} s"
                                      if self.motion == "demo" else "standing pose"))

        def tick(self):
            now = self.get_clock().now()
            t = 0.0 if self.motion == "stand" else ((now - self.start).nanoseconds * 1e-9) % gait.T_END
            q = gait.targets(t)
            if self.last_t is None or t < self.last_t:
                odom_base = self.anchor.reset(q)          # each loop starts again at the odom origin
            else:
                odom_base = self.anchor.update(q)
            self.last_t = t
            stamp = now.to_msg()

            js = JointState()
            js.header.stamp = stamp
            js.name = list(gait.JOINTS)
            js.position = [float(q.get(j, 0.0)) for j in gait.JOINTS]
            self.js_pub.publish(js)

            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.get_parameter("odom_frame").value
            tf.child_frame_id = self.get_parameter("base_frame").value
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
                float(v) for v in odom_base[:3, 3])
            (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
             tf.transform.rotation.w) = (float(v) for v in quaternion(odom_base[:3, :3]))
            self.tf_pub.sendTransform(tf)

    return BazanPlayback()


def main():
    import rclpy
    import rclpy.executors
    rclpy.init()
    node = make_node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
