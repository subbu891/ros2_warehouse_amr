import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool

from amr_validation.traffic import robot_blocks_lane


# Drives the simulated forklift back and forth across the main route and brakes for the robot like a driver
class ForkliftMover(Node):
    def __init__(self):
        super().__init__("forklift_mover")
        self.enabled = self.declare_parameter("enabled", True).value
        self.speed = self.declare_parameter("speed", 0.5).value
        self.lo = self.declare_parameter("min_y", 1.5).value
        self.hi = self.declare_parameter("max_y", 9.0).value

        # Forklift half length along y, lane half width (forklift + robot half extents) and braking distance
        self.half_length = self.declare_parameter("half_length", 0.4).value
        self.lane_half_width = self.declare_parameter("lane_half_width", 0.95).value
        self.brake_distance = self.declare_parameter("brake_distance", 1.0).value

        self.direction = 1.0
        self.xy = None
        self.robot_xy = None
        self.braking = False
        self.pub = self.create_publisher(Twist, "forklift/cmd_vel", 10)
        self.create_subscription(PoseStamped, "forklift/pose", self.on_pose, 10)
        self.create_subscription(Odometry, "ground_truth/odom", self.on_robot, 10)
        self.create_service(SetBool, "forklift/enable", self.on_enable)
        self.create_timer(0.1, self.tick)

    def on_pose(self, msg):
        self.xy = (msg.pose.position.x, msg.pose.position.y)

    def on_robot(self, msg):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_enable(self, req, res):
        self.enabled = req.data
        res.success, res.message = True, f"forklift {'enabled' if req.data else 'stopped'}"
        return res

    # Reverse direction at the lane ends; the velocity is applied in the forklift frame
    def tick(self):
        cmd = Twist()
        if self.enabled and self.xy is not None:
            if self.xy[1] >= self.hi:
                self.direction = -1.0
            elif self.xy[1] <= self.lo:
                self.direction = 1.0
            blocked = self.robot_xy is not None and robot_blocks_lane(
                self.xy, self.robot_xy, self.direction, self.half_length,
                self.lane_half_width, self.brake_distance)
            if blocked != self.braking:
                self.get_logger().info("braking for the robot" if blocked else "lane clear, driving on")
                self.braking = blocked
            if not blocked:
                cmd.linear.y = self.direction * self.speed
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ForkliftMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
