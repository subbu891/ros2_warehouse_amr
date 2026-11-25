import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_srvs.srv import SetBool


# Drives the simulated forklift back and forth across the main route
class ForkliftMover(Node):
    def __init__(self):
        super().__init__("forklift_mover")
        self.enabled = self.declare_parameter("enabled", True).value
        self.speed = self.declare_parameter("speed", 0.5).value
        self.lo = self.declare_parameter("min_y", 1.5).value
        self.hi = self.declare_parameter("max_y", 9.0).value
        self.direction = 1.0
        self.y = None
        self.pub = self.create_publisher(Twist, "forklift/cmd_vel", 10)
        self.create_subscription(PoseStamped, "forklift/pose", self.on_pose, 10)
        self.create_service(SetBool, "forklift/enable", self.on_enable)
        self.create_timer(0.1, self.tick)

    def on_pose(self, msg):
        self.y = msg.pose.position.y

    def on_enable(self, req, res):
        self.enabled = req.data
        res.success, res.message = True, f"forklift {'enabled' if req.data else 'stopped'}"
        return res

    # Reverse direction at the lane ends; the velocity is applied in the forklift frame
    def tick(self):
        cmd = Twist()
        if self.enabled and self.y is not None:
            if self.y >= self.hi:
                self.direction = -1.0
            elif self.y <= self.lo:
                self.direction = 1.0
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
