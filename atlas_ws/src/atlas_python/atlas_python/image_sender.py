import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import asyncio
import websockets

class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')
        self.init_params()
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'atlas/capture',
            self.image_callback,
            10
        )

        self.websocket_uri = f"ws://{self.get_parameter('ip_address').value}:8765"

        self.get_logger().info("Image sender node initialized.")

    def init_params(self):
        self.declare_parameter(
            "ip_address",
            value='192.168.137.234',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="IP address of the WebSocket server"
            )
        )

    def image_callback(self, msg):
        try:
            # Check the image encoding and convert appropriately
            if msg.encoding == '8UC3':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Encode the image to JPEG format
            _, encoded_image = cv2.imencode('.jpg', cv_image)
            image_data = encoded_image.tobytes()

            # Send the image data over WebSockets asynchronously
            self.get_logger().info("test")
            asyncio.run(self.send_image(image_data))

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')


    async def send_image(self, image_data):
        try:
            async with websockets.connect(self.websocket_uri) as websocket:
                await websocket.send(image_data)
                self.get_logger().info("Image sent to WebSocket client.")
        except Exception as e:
            self.get_logger().error(f"Failed to send image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
