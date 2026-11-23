#!/usr/bin/env python3
"""Camera node: captures webcam frames and publishes them to /camera/image_raw."""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device', 0)
        self.declare_parameter('fps', 20.0)

        self.cap = cv2.VideoCapture(self.get_parameter('device').value, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open webcam')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.create_timer(1.0 / self.get_parameter('fps').value, self.tick)
        self.get_logger().info('Camera node started')

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
