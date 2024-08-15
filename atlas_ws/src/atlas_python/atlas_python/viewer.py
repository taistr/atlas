import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Viewer(Node):
    def __init__(self):
        super().__init__('viewer')
        self.bridge = CvBridge()

        self.image_subscription = self.create_subscription(
            Image,
            'atlas/capture',
            self.image_callback,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )
        self.image_subscription  # prevent unused variable warning

    def image_callback(self, msg):
        """Convert ROS Image message to OpenCV format and show it."""
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Viewer Window', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    viewer = Viewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        viewer.get_logger().error(f'An error occurred: {e}')
        sys.exit(1)
    finally:
        viewer.destroy_node()
        cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
