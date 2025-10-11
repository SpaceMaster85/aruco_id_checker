import rclpy
from rclpy.node import Node
from aruco_markers_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

class ArucoIDChecker(Node):
    def __init__(self):
        super().__init__('aruco_id_checker')

        self.declare_parameter('target_id', 0)
        self.target_id = self.get_parameter('target_id').get_parameter_value().integer_value
        self.get_logger().info(f"Looking for Aruco ID: {self.target_id}")

        self.sub = self.create_subscription(
            MarkerArray,
            'aruco/markers',
            self.marker_callback,
            10
        )

        self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)

    def marker_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.id == self.target_id:
                pose_stamped = PoseStamped()
                pose_stamped.header = marker.pose.header
                pose_stamped.pose = marker.pose.pose
                self.pub.publish(pose_stamped)
                self.get_logger().info(f"Published pose for Aruco ID {self.target_id}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = ArucoIDChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
