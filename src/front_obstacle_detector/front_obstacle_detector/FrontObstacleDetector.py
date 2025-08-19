#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import Twist

class SpaceNavigator(Node):
    """
    Minimal reactive navigator (front / left / right sectors).
    Progress rules:
      - Go forward if front sufficiently clear (relaxed conditions).
      - Else turn toward clearer side.
      - After turning same direction for a few cycles and front not too close, add forward creep.
    Never commands reverse.
    """

    def __init__(self):
        super().__init__('space_navigator')
        d = self.declare_parameter

        # Topics / timing
        d('pointcloud_topic', '/velodyne_points')
        d('publish_rate_hz', 10.0)

        # Sector geometry (deg)
        d('front_half_angle_deg', 20.0)
        d('side_half_angle_deg', 80.0)
        d('side_gap_deg', 5.0)

        # Filters
        d('min_height', -1.0)
        d('max_height',  2.0)
        d('min_radius',  0.05)
        d('max_radius',  0.0)
        d('max_considered_range', 8.0)

        # Forward decision thresholds
        d('forward_clear_min', 0.60)
        d('forward_advantage_ratio', 1.18)
        d('forward_margin', 0.35)
        d('extra_forward_slack', 0.15)   # small additional allowed advantage (f >= best+slack)
        d('min_forward_speed', 0.08)     # ensure some motion once forward_ok

        # Side ambiguity
        d('side_diff_min', 0.12)

        # Speeds
        d('max_linear_speed', 0.55)
        d('max_angular_speed', 1.60)
        d('min_turn_angular_speed', 0.45)

        # Angular scaling vs front distance
        d('pivot_front_thresh', 2.2)     # below this push toward max angular
        d('linear_scale_stop', 1.0)      # scale forward by f / this (<=1)

        # Bias & creep
        d('prefer_left', True)
        d('turn_creep_enabled', True)
        d('turn_creep_fraction', 0.15)        # fraction of max_lin while turning (after commit)
        d('turn_creep_after_cycles', 6)       # cycles of same turn before allowing creep
        d('turn_creep_front_min', 0.90)       # need at least this front clearance to creep

        # Smoothing
        d('smoothing_alpha', 0.35)
        d('downsample_stride', 1)

        # Logging
        d('debug', True)
        d('log_every_n', 10)

        g = self.get_parameter
        self.topic = g('pointcloud_topic').value
        self.rate  = float(g('publish_rate_hz').value)

        self.front_half = math.radians(g('front_half_angle_deg').value)
        self.side_half  = math.radians(g('side_half_angle_deg').value)
        self.side_gap   = math.radians(g('side_gap_deg').value)

        self.min_h = g('min_height').value
        self.max_h = g('max_height').value
        self.min_r = g('min_radius').value
        self.max_r = g('max_radius').value
        self.max_range = g('max_considered_range').value

        self.forward_clear_min = g('forward_clear_min').value
        self.forward_adv_ratio = g('forward_advantage_ratio').value
        self.forward_margin    = g('forward_margin').value
        self.extra_slack       = g('extra_forward_slack').value
        self.min_forward_speed = g('min_forward_speed').value

        self.side_diff_min = g('side_diff_min').value

        self.max_lin = g('max_linear_speed').value
        self.max_ang = g('max_angular_speed').value
        self.min_turn_ang = g('min_turn_angular_speed').value

        self.pivot_front_thresh = g('pivot_front_thresh').value
        self.lin_scale_stop     = g('linear_scale_stop').value

        self.prefer_left = g('prefer_left').value
        self.turn_creep_enabled = g('turn_creep_enabled').value
        self.turn_creep_fraction = g('turn_creep_fraction').value
        self.turn_creep_after = int(g('turn_creep_after_cycles').value)
        self.turn_creep_front_min = g('turn_creep_front_min').value

        self.alpha  = g('smoothing_alpha').value
        self.stride = max(1, int(g('downsample_stride').value))

        self.debug = g('debug').value
        self.log_every_n = int(g('log_every_n').value)

        # Smoothed sector values
        self.f_clear = None
        self.l_clear = None
        self.r_clear = None

        self.frame = 0
        self.last_cloud_frame = -1

        # Turn persistence
        self.last_turn_dir = 0   # -1 right, +1 left
        self.same_turn_count = 0

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(PointCloud2, self.topic, self.pc_cb, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.rate, self.loop)

        self.get_logger().info("SpaceNavigator simplified + creep started")

    # ----------- Utility -----------
    def ema(self, prev, v):
        return v if prev is None else prev + self.alpha * (v - prev)

    def sector_clear(self, arr):
        if arr.size == 0:
            return self.max_range
        # percentile for robustness
        return float(min(np.percentile(arr, 30), self.max_range))

    # ----------- PointCloud callback -----------
    def pc_cb(self, msg: PointCloud2):
        self.frame += 1
        xs = []; ys = []; zs = []
        try:
            for i, p in enumerate(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)):
                if self.stride > 1 and (i % self.stride):
                    continue
                xs.append(p[0]); ys.append(p[1]); zs.append(p[2])
        except Exception as e:
            if self.debug:
                self.get_logger().warn(f"read_points error {e}")
            return
        if not xs:
            return

        x = np.asarray(xs, np.float32)
        y = np.asarray(ys, np.float32)
        z = np.asarray(zs, np.float32)

        hmask = (z >= self.min_h) & (z <= self.max_h)
        if not hmask.any():
            return
        x = x[hmask]; y = y[hmask]
        r = np.hypot(x, y)

        if self.min_r > 0:
            m = r >= self.min_r
            x = x[m]; y = y[m]; r = r[m]
        if self.max_r > 0:
            m = r <= self.max_r
            x = x[m]; y = y[m]; r = r[m]
        if x.size == 0:
            return

        ang = np.arctan2(y, x)
        fh = self.front_half
        gap = self.side_gap
        sh = self.side_half

        front = r[(np.abs(ang) <= fh)]
        left  = r[(ang > (fh + gap)) & (ang <= (fh + gap + sh))]
        right = r[(ang < -(fh + gap)) & (ang >= -(fh + gap + sh))]

        f = self.sector_clear(front)
        l = self.sector_clear(left)
        r_ = self.sector_clear(right)

        self.f_clear = self.ema(self.f_clear, f)
        self.l_clear = self.ema(self.l_clear, l)
        self.r_clear = self.ema(self.r_clear, r_)
        self.last_cloud_frame = self.frame

        if self.debug and (self.frame % self.log_every_n == 0):
            self.get_logger().info(
                f"[{self.frame}] F/L/R={self.f_clear:.2f}/{self.l_clear:.2f}/{self.r_clear:.2f} "
                f"raw={f:.2f}/{l:.2f}/{r_:.2f}"
            )

    # ----------- Decision -----------
    def decide(self):
        if self.f_clear is None:
            return 0.0, 0.0
        f = self.f_clear; l = self.l_clear; r = self.r_clear
        best_side = max(l, r)
        side_diff = l - r  # + left better

        # Relaxed forward condition (any of the advantage clauses)
        forward_ok = (
            f >= self.forward_clear_min and (
                f >= self.forward_adv_ratio * best_side or
                (f - best_side) >= self.forward_margin or
                f >= (best_side + self.extra_slack)
            )
        )

        if forward_ok:
            lin = self.max_lin * min(1.0, f / max(0.01, self.lin_scale_stop))
            if lin < self.min_forward_speed:
                lin = self.min_forward_speed
            ang = 0.0
            self.last_turn_dir = 0
            self.same_turn_count = 0
            return lin, ang

        # Decide turn direction
        if abs(side_diff) < self.side_diff_min:
            turn_left = self.prefer_left
        else:
            turn_left = side_diff > 0.0
        turn_dir = 1 if turn_left else -1

        # Maintain turn persistence counter
        if turn_dir == self.last_turn_dir:
            self.same_turn_count += 1
        else:
            self.last_turn_dir = turn_dir
            self.same_turn_count = 1

        # Angular magnitude scaled by how tight front is
        tight_factor = 1.0 - min(1.0, f / self.pivot_front_thresh)  # 0 -> far, 1 -> very close
        ang_mag = self.min_turn_ang + (self.max_ang - self.min_turn_ang) * max(0.25, tight_factor)
        ang = turn_dir * ang_mag

        # Forward creep after sustained turning
        allow_creep = (
            self.turn_creep_enabled and
            self.same_turn_count >= self.turn_creep_after and
            f >= self.turn_creep_front_min
        )
        lin = self.max_lin * self.turn_creep_fraction if allow_creep else 0.0
        return lin, ang

    # ----------- Loop -----------
    def loop(self):
        if (self.frame - self.last_cloud_frame) > int(self.rate * 1.5):
            lin = ang = 0.0
        else:
            lin, ang = self.decide()
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)
        if self.debug and (self.frame % self.log_every_n == 0):
            self.get_logger().info(f"cmd lin={lin:.2f} ang={ang:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SpaceNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()