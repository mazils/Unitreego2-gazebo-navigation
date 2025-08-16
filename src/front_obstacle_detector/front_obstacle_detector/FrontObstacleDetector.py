#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Bool

class FrontObstacleDetector(Node):
    def __init__(self):
        super().__init__('front_obstacle_detector')

        # Parameters (front)
        self.declare_parameter('topic', '/velodyne_points')
        self.declare_parameter('distance_threshold', 1.5)          # forward distance (x)
        self.declare_parameter('lateral_half_width', 0.4)          # half width for front ROI (|y|)
        self.declare_parameter('min_height', -0.10)
        self.declare_parameter('max_height', 1.50)
        self.declare_parameter('min_points_for_obstacle', 25)
        self.declare_parameter('use_radial', False)                # front ROI radial vs rectangular

        # Side sector parameters
        self.declare_parameter('side_distance_threshold', 1.0)     # lateral reach for left/right (|y|)
        self.declare_parameter('side_longitudinal_half', 0.6)      # |x| < this for side ROIs
        self.declare_parameter('side_min_points_for_obstacle', 15) # min points for side obstacles
        self.declare_parameter('min_forward_clearance', 0.05)      # ignore very near sensor points
        self.declare_parameter('publish_always', True)             # publish every frame or only on change

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.lateral_half_width = self.get_parameter('lateral_half_width').get_parameter_value().double_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.min_points_for_obstacle = self.get_parameter('min_points_for_obstacle').get_parameter_value().integer_value
        self.use_radial = self.get_parameter('use_radial').get_parameter_value().bool_value

        self.side_distance_threshold = self.get_parameter('side_distance_threshold').get_parameter_value().double_value
        self.side_longitudinal_half = self.get_parameter('side_longitudinal_half').get_parameter_value().double_value
        self.side_min_points_for_obstacle = self.get_parameter('side_min_points_for_obstacle').get_parameter_value().integer_value
        self.min_forward_clearance = self.get_parameter('min_forward_clearance').get_parameter_value().double_value
        self.publish_always = self.get_parameter('publish_always').get_parameter_value().bool_value

        # Subscription
        self.subscription = self.create_subscription(PointCloud2, topic, self.pc_callback, 10)

        # Publishers
        self.front_pub = self.create_publisher(Bool, '/front_obstacle_detected', 10)
        self.left_pub = self.create_publisher(Bool, '/left_obstacle_detected', 10)
        self.right_pub = self.create_publisher(Bool, '/right_obstacle_detected', 10)
        self.combined_pub = self.create_publisher(Bool, '/any_obstacle_detected', 10)

        # Last states (for edge publishing if publish_always == False)
        self._last_front = None
        self._last_left = None
        self._last_right = None
        self._last_any = None

        self.get_logger().info(f"Subscribed to {topic}")

    def _maybe_publish(self, pub, new_state, last_attr_name):
        if self.publish_always or getattr(self, last_attr_name) != new_state:
            msg = Bool()
            msg.data = new_state
            pub.publish(msg)
        setattr(self, last_attr_name, new_state)

    def pc_callback(self, msg):
        log = self.get_logger().info

        total_points = msg.width * msg.height
        if total_points == 0:
            return

        # Decode raw point cloud (Velodyne layout assumed)
        try:
            if len(msg.data) < total_points * msg.point_step:
                log("Incomplete data buffer.")
                return

            # Adjust dtype if your point_step changes
            dtype = np.dtype([
                ('x', '<f4'),
                ('y', '<f4'),
                ('z', '<f4'),
                ('intensity', '<f4'),
                ('ring', '<u2'),
                ('time', '<f4')
            ])
            raw = np.frombuffer(msg.data, dtype=dtype, count=total_points)
            points = np.stack((raw['x'], raw['y'], raw['z']), axis=1)
        except Exception as e:
            log(f"Decode failed: {e}")
            return

        # Remove exact zeros (sensor artifacts)
        mask_non_zero = ~((points[:,0] == 0.0) & (points[:,1] == 0.0) & (points[:,2] == 0.0))
        points = points[mask_non_zero]
        if points.size == 0:
            return

        # Height filter
        hmask = (points[:,2] >= self.min_height) & (points[:,2] <= self.max_height)
        points = points[hmask]
        if points.size == 0:
            return

        x = points[:,0]
        y = points[:,1]

        # FRONT ROI
        if self.use_radial:
            dist = np.sqrt(x**2 + y**2)
            front_roi = (x > self.min_forward_clearance) & (dist < self.distance_threshold) & (np.abs(y) < self.lateral_half_width)
        else:
            front_forward = (x > self.min_forward_clearance) & (x < self.distance_threshold)
            front_lateral = (np.abs(y) < self.lateral_half_width)
            front_roi = front_forward & front_lateral
        front_count = int(np.count_nonzero(front_roi))

        # LEFT ROI (side rectangular window centered laterally, limited in |x|)
        left_roi = (
            (y > 0.05) &
            (y < self.side_distance_threshold) &
            (np.abs(x) < self.side_longitudinal_half)
        )
        left_count = int(np.count_nonzero(left_roi))

        # RIGHT ROI
        right_roi = (
            (y < -0.05) &
            (y > -self.side_distance_threshold) &
            (np.abs(x) < self.side_longitudinal_half)
        )
        right_count = int(np.count_nonzero(right_roi))

        front_detected = front_count >= self.min_points_for_obstacle
        left_detected = left_count >= self.side_min_points_for_obstacle
        right_detected = right_count >= self.side_min_points_for_obstacle
        any_detected = front_detected or left_detected or right_detected

        log(f"Counts front/left/right: {front_count}/{left_count}/{right_count}")

        if front_detected:
            fmin = float(np.min(x[front_roi])) if front_count else float('nan')
            log(f"Front obstacle (min x {fmin:.2f} m)")
        if left_detected:
            lminy = float(np.min(y[left_roi]))
            log(f"Left obstacle (closest y {lminy:.2f} m)")
        if right_detected:
            rmaxy = float(np.max(y[right_roi]))
            log(f"Right obstacle (closest y {rmaxy:.2f} m)")
        if not any_detected:
            log("No obstacles in sectors.")

        # Publish states
        self._maybe_publish(self.front_pub, front_detected, '_last_front')
        self._maybe_publish(self.left_pub, left_detected, '_last_left')
        self._maybe_publish(self.right_pub, right_detected, '_last_right')
        self._maybe_publish(self.combined_pub, any_detected, '_last_any')


def main(args=None):
    rclpy.init(args=args)
    node = FrontObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()