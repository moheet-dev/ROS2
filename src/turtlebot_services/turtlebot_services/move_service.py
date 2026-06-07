import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
import math
from math import atan2
from nav_msgs.msg import Odometry
from turtlebot_interfaces.srv import MoveRobot
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MoveService(Node):
    def __init__(self):
        super().__init__('move_service')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.obstacle_detected = False
        self.front_distance = float('inf')

        # ReentrantCallbackGroup lets callbacks run concurrently
        cb_group = ReentrantCallbackGroup()

        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=cb_group
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=cb_group
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(
            MoveRobot, 'move_service', self.move_callback,
            callback_group=cb_group
        )
        self.get_logger().info("Move service ready")

    def scan_callback(self, msg):
        front_ranges = (
            msg.ranges[0:20] +
            msg.ranges[-20:]
        )
        valid_ranges = [
            r for r in front_ranges
            if not math.isinf(r)
            and not math.isnan(r)
        ]
        if valid_ranges:
            self.front_distance = min(valid_ranges)
            self.obstacle_detected = (
                self.front_distance < 0.4
            )
            # self.get_logger().info(
            #     f"Front distance: "
            #     f"{self.front_distance:.2f}"
            # )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
        self.current_yaw = yaw

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)

    def rotate_robot(self, target_angle):
        rate = self.create_rate(20)   # 20 Hz — yields to executor each cycle
        tolerance = 0.05
        while rclpy.ok():
            angle_error = math.atan2(
                math.sin(target_angle - self.current_yaw),
                math.cos(target_angle - self.current_yaw)
            )
            # self.get_logger().info(
            #     f"Yaw: {self.current_yaw:.2f}  Target: {target_angle:.2f}  Error: {angle_error:.2f}"
            # )
            if abs(angle_error) < tolerance:
                break
            msg = Twist()
            msg.angular.z = 0.3 if angle_error > 0 else -0.3
            self.cmd_vel_publisher.publish(msg)
            rate.sleep()
        self.stop_robot()

    def move_forward(self, target_x, target_y):
        rate = self.create_rate(20)
        tolerance = 0.1
        while rclpy.ok():
            if self.obstacle_detected:
                self.get_logger().warn(
                    "Obstacle detected!"
                )
                self.stop_robot()
                return False;
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            if math.sqrt(dx**2 + dy**2) < tolerance:
                break
            msg = Twist()
            msg.linear.x = 0.2
            self.cmd_vel_publisher.publish(msg)
            rate.sleep()
        self.stop_robot()
        return True;

    def move_callback(self, req, res):
        self.get_logger().info(
            f"Moving to ({req.target_x}, {req.target_y}) "
            f"from ({self.current_x:.2f}, {self.current_y:.2f})"
        )
        target_angle = atan2(
            req.target_y - self.current_y,
            req.target_x - self.current_x
        )
        self.rotate_robot(target_angle)
        success = self.move_forward(req.target_x, req.target_y)
        if success:
            self.get_logger().info(
                "Reached target"
            )
            res.success = True
            res.message = "Reached target"
        else:
            res.success = False
            res.message = (
                "Obstacle detected"
            )
        return res

def main(args=None):
    rclpy.init(args=args)
    node = MoveService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()          # runs all callbacks concurrently across threads
    rclpy.shutdown()

if __name__ == '__main__':
    main()