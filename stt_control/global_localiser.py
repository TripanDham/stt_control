# my_static_transform_publisher/static_transform_publisher.py

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('global_localiser')

        # Create a tf2 broadcaster to publish the transform
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Create the static transforms
        self.create_static_transform('/odom', '/a_odom', 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.create_static_transform('/odom', '/b_odom', 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    def create_static_transform(self, parent_frame, child_frame, x, y, z, qx, qy, qz, qw):
        transform = TransformStamped()

        # Fill in the transform details
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        transform.transform.translation = Vector3(x=x, y=y, z=z)
        transform.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Broadcast the static transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    static_transform_publisher = StaticTransformPublisher()

    rclpy.spin(static_transform_publisher)

    static_transform_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
