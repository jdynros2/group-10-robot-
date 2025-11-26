import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Subscribe to /cmd_vel_raw (from teleop)
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.twist_callback,
            10
        )

        # Publish to /cmd_vel (TwistStamped → Gazebo listens here)
        self.pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

    def twist_callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self.pub.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
