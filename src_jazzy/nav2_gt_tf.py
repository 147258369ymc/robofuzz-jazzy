#!/usr/bin/env python3
"""Ground-truth map->odom TF publisher for the nav2_jazzy target.

Why this exists
---------------
The nav2 stack normally localizes with amcl against a laser map. On the
TurtleBot4 Jazzy base that path carries a known scan-frame / map->odom TF
"blood debt" (see the slam_toolbox target notes and project memory): amcl was
brittle to bring up headlessly. For a *navigability* fuzzing target we do not
need a realistic localizer -- we only need nav2 to believe the robot is exactly
where it truly is, so that goal arrival / collision / liveness oracles judge
the planner+controller, not localization noise.

So we bypass amcl entirely and synthesize the map->odom transform from the
simulator ground truth.

TF math
-------
nav2's TF chain is:  map -> odom -> base_link  (+ base_link -> sensors)
The TurtleBot4 base already publishes odom -> base_link (wheel odometry, which
drifts). Gazebo publishes the true robot pose on /sim_ground_truth_pose
(nav_msgs/Odometry), which is effectively  map -> base_link  (the true pose of
base_footprint/base_link in the world/map frame; the robot is spawned at the
world origin so world == map here).

We want the robot to appear, in the map frame, exactly at ground truth:

    T_map_base(desired) = T_gt          (from /sim_ground_truth_pose)
    T_map_base(actual)  = T_map_odom * T_odom_base   (nav2's chain)

Solving for the correction we must publish:

    T_map_odom = T_gt * inverse(T_odom_base)

where T_odom_base comes from /odom (nav_msgs/Odometry, odom -> base_link).
This keeps map->odom continuously updated so that even as wheel odom drifts,
map->base_link tracks truth. All math is done with plain quaternion/vector
helpers (no tf_transformations dependency) so the script runs under a bare
`python3 nav2_gt_tf.py` in a ROS-sourced env.

Fallback / assumption
---------------------
If /odom has not been seen yet we publish map->odom = T_gt (i.e. assume
odom == base_link). If neither topic has data we publish identity. Because the
robot is spawned at the world origin (x=y=yaw=0, see
turtlebot4_fuzz_spawn.launch.py) identity is the correct seed pose, so nav2 is
never left without a map->odom transform. This matches the "static/seeded at
the known spawn pose" approach the task brief permits.

Rate: >= 10 Hz (default 20 Hz). use_sim_time is enabled so stamps align with
/clock from the gz clock bridge.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def quat_mul(a, b):
    """Hamilton product a*b; quats are (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rotate(q, v):
    """Rotate 3-vector v by quaternion q (x,y,z,w)."""
    vq = (v[0], v[1], v[2], 0.0)
    r = quat_mul(quat_mul(q, vq), quat_conj(q))
    return (r[0], r[1], r[2])


def pose_to_tf(px, py, pz, q):
    """Return (translation xyz, quaternion) for a pose."""
    return (px, py, pz), q


def invert_tf(t, q):
    """Invert a rigid transform (translation t, rotation q)."""
    qi = quat_conj(q)
    ti = quat_rotate(qi, (-t[0], -t[1], -t[2]))
    return ti, qi


def compose_tf(t1, q1, t2, q2):
    """Compose two rigid transforms: result = T1 * T2."""
    t = quat_rotate(q1, t2)
    t = (t[0] + t1[0], t[1] + t1[1], t[2] + t1[2])
    q = quat_mul(q1, q2)
    return t, q


def odom_to_tf(msg):
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)


class GroundTruthMapOdom(Node):
    def __init__(self):
        super().__init__("nav2_gt_map_odom_publisher")
        # Ground-truth is published best-effort by the gz bridge; odom is
        # typically reliable. Use best-effort readers so we never miss data due
        # to a QoS mismatch (best-effort reader accepts reliable publishers).
        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.map_frame = "map"
        self.odom_frame = "odom"

        self._gt = None    # T_map_base  (translation, quat)
        self._odom = None  # T_odom_base (translation, quat)

        self.br = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, "/sim_ground_truth_pose", self._on_gt, be_qos)
        self.create_subscription(
            Odometry, "/odom", self._on_odom, be_qos)

        rate_hz = 20.0
        self.create_timer(1.0 / rate_hz, self._publish)
        self._logged_first = False

    def _on_gt(self, msg):
        self._gt = odom_to_tf(msg)

    def _on_odom(self, msg):
        self._odom = odom_to_tf(msg)

    def _publish(self):
        if self._gt is not None and self._odom is not None:
            # T_map_odom = T_gt * inverse(T_odom_base)
            ot, oq = self._odom
            oit, oiq = invert_tf(ot, oq)
            gt_t, gt_q = self._gt
            t, q = compose_tf(gt_t, gt_q, oit, oiq)
        elif self._gt is not None:
            # No odom yet: assume odom == base_link (map->odom == ground truth).
            t, q = self._gt
        else:
            # Nothing yet: identity (robot spawns at the world origin).
            t, q = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])
        self.br.sendTransform(tf)

        if not self._logged_first and self._gt is not None:
            self.get_logger().info(
                "publishing ground-truth map->odom TF (first fix received)")
            self._logged_first = True


def main(args=None):
    rclpy.init(args=args)
    # Force use_sim_time so TF stamps align with /clock.
    node = GroundTruthMapOdom()
    node.set_parameters(
        [rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
