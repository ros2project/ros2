import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math

class GoalCheckerNode(Node):

    def __init__(self):
        super().__init__('goal_checker_node')
        self.declare_parameter('goal_threshold', 0.5)
        self.goal_threshold = self.get_parameter('goal_threshold').get_parameter_value().double_value

        self.goal_pose = None

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.path_subscriber = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        self.goal_reached_publisher = self.create_publisher(
            String,
            '/goal_reached',
            10
        )

        self.get_logger().info("Goal Checker Node has been started.")

    def path_callback(self, msg):
        if msg.poses:
            self.goal_pose = msg.poses[-1].pose
            self.get_logger().info(f"Received new goal pose: {self.goal_pose}")

    def odom_callback(self, msg):
        if self.goal_pose:
            current_pose = msg.pose.pose
            distance = self.calculate_distance(current_pose, self.goal_pose)

            if distance < self.goal_threshold:
                self.get_logger().info("The robot has reached its destination!")
                self.publish_goal_reached()

    def calculate_distance(self, pose1, pose2):
        return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2 +
            (pose1.position.y - pose2.position.y) ** 2 +
            (pose1.position.z - pose2.position.z) ** 2
        )

    def publish_goal_reached(self):
        msg = String()
        msg.data = "The robot has reached its destination!"
        self.goal_reached_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    goal_checker_node = GoalCheckerNode()
    rclpy.spin(goal_checker_node)
    goal_checker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()