#!/usr/bin/env python3
"""MuJoCo <-> ROS 2 bridge for Mico: physics in MuJoCo, the topics a ros2_control robot would use.

Publishes (stamped with sim time):
    /clock                      rosgraph_msgs/Clock
    /joint_states               sensor_msgs/JointState (position, velocity, servo torque), mico_gait.JOINTS order
    /tf  odom -> base_link      ground truth of the free base
    /odom                       nav_msgs/Odometry, same pose, twist in base_link
    /imu                        sensor_msgs/Imu in imu_link (+9.81 m/s^2 on z at rest)
Subscribes:
    /joint_trajectory_controller/joint_trajectory   trajectory_msgs/JointTrajectory, as joint_trajectory_controller:
                                                    linear interpolation on sim time, zero stamp = start now
    /position_controller/commands                   std_msgs/Float64MultiArray, 20 targets in JOINTS order

    ros2 launch humanoid_description mico_mujoco.launch.py gait:=true        # bridge + RViz2 + demo gait
    ros2 run humanoid_control mico_mujoco_bridge.py --ros-args -p robot_description:="$(xacro ...)"

The servo model and the MJCF are in mico_mujoco.py. viewer:=true opens the MuJoCo viewer (macOS: run the bridge
with mjpython, the launch file does that).
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
try:
    import bazan_mujoco as mm
except ImportError:
    import mico_mujoco as mm  # noqa: E402


def resolve_mesh(filename):
    from ament_index_python.packages import get_package_share_directory
    if filename.startswith("package://"):
        pkg, _, rel = filename[len("package://"):].partition("/")
        return os.path.join(get_package_share_directory(pkg), rel)
    return filename[len("file://"):] if filename.startswith("file://") else filename


def make_node():
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray
    from tf2_ros import TransformBroadcaster
    from trajectory_msgs.msg import JointTrajectory

    class BazanMujocoBridge(Node):
        def __init__(self):
            super().__init__("bazan_mujoco_bridge")
            self.declare_parameter("robot_description", "")
            self.declare_parameter("publish_rate", 100.0)
            self.declare_parameter("real_time_factor", 1.0)
            self.declare_parameter("viewer", False)
            self.declare_parameter("kp", mm.KP)
            self.declare_parameter("kv", mm.KV)
            self.declare_parameter("armature", mm.ARMATURE)
            urdf = self.get_parameter("robot_description").value
            if not urdf:
                raise RuntimeError("parameter robot_description is empty")
            p = self.get_parameter
            self.sim = mm.BazanSim(
                mm.build_mjcf(urdf, resolve_mesh, p("kp").value, p("kv").value, p("armature").value)
            )
            # Initialize with nominal crouch standing pose so robot never falls during node startup
            try:
                import bazan_gait as gait
            except ImportError:
                import mico_gait as gait
            hp, kn, ap = gait.leg_ik(0, gait.H0)
            nominal_pose = {
                "l_hip_pitch": hp, "l_knee": kn, "l_ankle_pitch": ap,
                "r_hip_pitch": hp, "r_knee": kn, "r_ankle_pitch": ap,
                "l_elbow": -0.4, "r_elbow": -0.4,
            }
            self.sim.reset(pose=nominal_pose)
            self.steps = max(1, int(round(1.0 / (p("publish_rate").value * self.sim.model.opt.timestep))))
            self.rtf = p("real_time_factor").value

            self.clock_pub = self.create_publisher(Clock, "/clock", 10)
            self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self.imu_pub = self.create_publisher(Imu, "/imu", 10)
            self.tf_pub = TransformBroadcaster(self)
            self.create_subscription(JointTrajectory, "/joint_trajectory_controller/joint_trajectory",
                                     self.on_trajectory, 10)
            self.create_subscription(Float64MultiArray, "/position_controller/commands", self.on_commands, 10)
            self.msgs = (Time, TransformStamped, Odometry, Clock, Imu, JointState)
            self.get_logger().info(f"MuJoCo {mm.mujoco.__version__}: {self.sim.model.nbody} bodies, "
                                   f"{len(mm.JOINTS)} servos, step {self.sim.model.opt.timestep * 1e3:.1f} ms, "
                                   f"publishing every {self.steps} steps")

        # -------------------------------------------------------------- commands
        def on_trajectory(self, msg):
            if not msg.points:
                self.sim.set_target(self.sim.current_target())
                return
            unknown = [j for j in msg.joint_names if j not in mm.JOINTS]
            if unknown:
                self.get_logger().error(f"unknown joints {unknown}, trajectory ignored")
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            times = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 for pt in msg.points]
            self.sim.set_trajectory(msg.joint_names, times, [list(pt.positions) for pt in msg.points],
                                    start=stamp if stamp > 0 else None)
            self.get_logger().info(f"trajectory: {len(times)} points, {times[-1]:.1f} s")

        def on_commands(self, msg):
            if len(msg.data) != len(mm.JOINTS):
                self.get_logger().error(f"expected {len(mm.JOINTS)} commands, got {len(msg.data)}")
                return
            self.sim.set_target(list(msg.data))

        # -------------------------------------------------------------- state
        def publish(self):
            Time, TransformStamped, Odometry, Clock, Imu, JointState = self.msgs
            ns = int(round(self.sim.time * 1e9))
            stamp = Time(sec=ns // 10**9, nanosec=ns % 10**9)
            self.clock_pub.publish(Clock(clock=stamp))

            js = JointState(name=list(mm.JOINTS))
            js.header.stamp = stamp
            js.position = self.sim.joint_positions().tolist()
            js.velocity = self.sim.joint_velocities().tolist()
            js.effort = self.sim.joint_efforts().tolist()
            self.js_pub.publish(js)

            xyz, q = self.sim.base_pose()
            lin, ang = self.sim.base_twist()
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
            t, r = tf.transform.translation, tf.transform.rotation
            t.x, t.y, t.z = (float(v) for v in xyz)
            r.w, r.x, r.y, r.z = (float(v) for v in q)
            self.tf_pub.sendTransform(tf)

            od = Odometry()
            od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "odom", "base_link"
            od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = t.x, t.y, t.z
            od.pose.pose.orientation = r
            rot = self.sim.data.xmat[self.sim.base].reshape(3, 3)
            v = rot.T @ lin                                      # world -> base_link
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = (float(c) for c in v)
            od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = (float(c) for c in ang)
            self.odom_pub.publish(od)

            acc, gyro, quat = self.sim.imu()
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, "imu_link"
            o = imu.orientation
            o.w, o.x, o.y, o.z = (float(c) for c in quat)
            a, g = imu.linear_acceleration, imu.angular_velocity
            a.x, a.y, a.z = (float(c) for c in acc)
            g.x, g.y, g.z = (float(c) for c in gyro)
            self.imu_pub.publish(imu)

    return BazanMujocoBridge()


def main():
    import rclpy
    import rclpy.executors
    rclpy.init()
    node = make_node()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    viewer = None
    if node.get_parameter("viewer").value:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(node.sim.model, node.sim.data)
        viewer.opt.geomgroup[0] = 0                          # hide the collision boxes
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.8, -15, 150
    wall0, sim0 = time.monotonic(), node.sim.time
    try:
        node.publish()
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            executor.spin_once(timeout_sec=0)
            node.sim.step(node.steps)
            node.publish()
            if viewer is not None:
                viewer.cam.lookat[:] = node.sim.base_pose()[0] + [0, 0, 0.03]
                viewer.sync()
            ahead = (node.sim.time - sim0) / node.rtf - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)
            elif ahead < -0.5:                               # cannot keep up: do not try to catch up
                wall0, sim0 = time.monotonic(), node.sim.time
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if viewer is not None:
            viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
