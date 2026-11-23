#!/usr/bin/env python3
"""Gesture node: counts fingers in /camera/image_raw and drives the robot via Twist."""
import math
from collections import deque, Counter

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# finger count -> (linear speed, angular speed)
MOVES = {
    0: (0.0, 0.0),    # fist: stop
    1: (0.2, 0.0),    # forward
    2: (-0.2, 0.0),   # backward
    3: (0.0, 0.8),    # left
    4: (0.0, -0.8),   # right
}
NAMES = {0: 'STOP', 1: 'FORWARD', 2: 'BACK', 3: 'LEFT', 4: 'RIGHT'}

# rectangle where the hand must be shown (x1, y1, x2, y2)
X1, Y1, X2, Y2 = 300, 50, 600, 350


def count_fingers(roi):
    ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    mask = cv2.inRange(ycrcb, (0, 135, 85), (255, 180, 135))  # skin colour
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0, mask
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < 3000:
        return 0, mask

    hull_pts = cv2.convexHull(c)
    solidity = area / max(cv2.contourArea(hull_pts), 1)

    hull = cv2.convexHull(c, returnPoints=False)
    defects = cv2.convexityDefects(c, hull)
    gaps = 0
    if defects is not None:
        for s, e, f, d in defects[:, 0]:
            a, b, far = c[s][0], c[e][0], c[f][0]
            A = math.dist(b, far)
            B = math.dist(a, far)
            C = math.dist(a, b)
            if A * B == 0:
                continue
            cos_val = (A * A + B * B - C * C) / (2 * A * B)
            angle = math.acos(max(-1.0, min(1.0, cos_val)))
            if angle < math.radians(90) and d > 8000:
                gaps += 1

    if gaps == 0:
        return (0 if solidity > 0.85 else 1), mask
    return min(gaps + 1, 5), mask


class GestureNode(Node):
    def __init__(self):
        super().__init__('gesture_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel')   # turtlesim: /turtle1/cmd_vel
        self.declare_parameter('speed_scale', 1.0)        # turtlesim: 10.0

        self.scale = self.get_parameter('speed_scale').value
        self.bridge = CvBridge()
        self.history = deque(maxlen=7)

        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.cb, 10)
        self.pub = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10)
        self.get_logger().info('Gesture node (OpenCV) started')

    def cb(self, msg):
        frame = cv2.flip(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), 1)
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = X1, Y1, min(X2, w), min(Y2, h)

        n_raw, mask = count_fingers(frame[y1:y2, x1:x2])

        # smoothing: a gesture is accepted only if held for ~0.3 s
        self.history.append(n_raw)
        val, cnt = Counter(self.history).most_common(1)[0]
        n = val if cnt >= 5 else 0

        lin, ang = MOVES.get(n, (0.0, 0.0))
        t = Twist()
        t.linear.x = lin * self.scale
        t.angular.z = ang * self.scale
        self.pub.publish(t)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'fingers={n} {NAMES.get(n, "")}', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('robot', frame)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)

    def stop_robot(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
