#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')

        # Parameters
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # Obstacle states (None until first message; treat None as False in logic)
        self.front_obstacle = None
        self.left_obstacle = None
        self.right_obstacle = None
        self.any_obstacle = None  # not strictly needed, but can be logged

        # Subscriptions
        self.create_subscription(Bool, '/front_obstacle_detected', self.front_cb, 10)
        self.create_subscription(Bool, '/left_obstacle_detected', self.left_cb, 10)
        self.create_subscription(Bool, '/right_obstacle_detected', self.right_cb, 10)
        self.create_subscription(Bool, '/any_obstacle_detected', self.any_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("MovementController started (forward unless blocked).")

    # Callbacks
    def front_cb(self, msg): self.front_obstacle = msg.data
    def left_cb(self, msg): self.left_obstacle = msg.data
    def right_cb(self, msg): self.right_obstacle = msg.data
    def any_cb(self, msg): self.any_obstacle = msg.data

    def obstacle(self, flag):
        # Treat None (no data yet) as False (unblocked)
        return bool(flag) if flag is not None else False

    def control_loop(self):
        twist = Twist()

        front = self.obstacle(self.front_obstacle)
        left = self.obstacle(self.left_obstacle)
        right = self.obstacle(self.right_obstacle)

        # Decision logic:
        # 1. If no front obstacle: drive forward.
        # 2. If front blocked:
        #    a. Turn right if right is free.
        #    b. Else turn left if left is free.
        #    c. Else stop (boxed in).
        if not front:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            action = "forward"
        else:
            if not right:
                twist.linear.x = 0.0
                twist.angular.z = -self.turn_speed   # right turn
                action = "turn_right"
            elif not left:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed    # left turn
                action = "turn_left"
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                action = "stop_blocked"

        self.cmd_pub.publish(twist)

        # Optional lightweight debug (only every N loops if desired)
        self.get_logger().info(
            f"front={front} left={left} right={right} action={action}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()