import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from spinnyrobot.taskmanager import TaskManager
from std_msgs.msg import Float32
import cv2

bridge = CvBridge()


class SpinnyRobot(Node):
    def __init__(self):
        self.tm = TaskManager()
        super().__init__('spinnyrobot')
        self.anglesub = self.create_subscription(Float32, '/desired_angle', self.angle_sub_callback, 10)
        self.imagepub = self.create_publisher(Image, '/robotcam', 10)
        self.anglepub = self.create_publisher(Float32, '/current_angle', 10)
        self.pubtimer = self.create_timer(1 / 30, self.timer_callback)

    def angle_sub_callback(self, message: Float32):
        # self.get_logger().info('bruhh')
        self.tm.set_joint_angle(message.data)

    def timer_callback(self):
        rendered_image = self.tm.render_image()
        joint_angle = self.tm.get_joint_angle()
        image = bridge.cv2_to_imgmsg(rendered_image, encoding='bgr8')
        self.imagepub.publish(image)
        angle = Float32()
        angle.data = joint_angle
        self.anglepub.publish(angle)


import numpy as np
class RedBoxAimer(Node):
    def __init__(self):
        super().__init__('red_box_aimer')
        self.imagesub = self.create_subscription(Image, '/robotcam', self.image_callback, 9)
        self.anglesub = self.create_subscription(Float32, '/current_angle', self.angle_callback, 9)
        self.anglepub = self.create_publisher(Float32, '/desired_angle', 10)

        self.bridge = CvBridge()
        # Constants
        self.camera_fov_rad = np.pi / 2  # 90° FOV in radians
        self.max_yaw_correction = self.camera_fov_rad / 2  # ±π/2 max correction
        self.current_angle = 0.0  # Latest received yaw angle

    def angle_callback(self, msg):
        self.current_angle = msg.data
        self.get_logger().info(f'curr angle: {self.current_angle}')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        #convert to hsv
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # red mask
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        # Create masks for red
        mask = cv2.inRange(hsv, lower_red1, upper_red1)

        # Compute centroid of the red mask
        M = cv2.moments(mask)
        if M["m00"] > 0:
            box_center_x = int(M["m10"] / M["m00"])
            img_width = cv_image.shape[1]
            img_center_x = img_width // 2  # Image midpoint

            # Calculate yaw correction **in radians**, clamped to ±π/2
            pixel_offset = box_center_x - img_center_x
            yaw_correction = (pixel_offset / img_width) * self.camera_fov_rad
            yaw_correction = np.clip(yaw_correction, -self.max_yaw_correction, self.max_yaw_correction)
            
            self.get_logger().info(f'correction: {yaw_correction}')
            # if yaw_correction == 0:
            #     self.get_logger().info('centered, stop publishing')
            #     return

            # Compute new desired angle relative to current angle
            desired_angle = self.current_angle - yaw_correction
            self.publish_angle(desired_angle)
        else:
            self.publish_angle(self.current_angle)  # No red detected → hold position
            


            
        cv2.waitKey(1)  # Required to update the OpenCV window
        cv2.imshow("Original", cv_image)
        cv2.imshow("red mask", mask)

    def publish_angle(self, angle):
        """Publish desired angle."""
        msg = Float32()
        msg.data = float(angle)
        self.anglepub.publish(msg)
        self.get_logger().info(f'Published angle: {angle:.5f}')

def main(args=None):
    print("10000")
    rclpy.init(args=args)
    sr = SpinnyRobot()
    # rba = RedBoxAimer()

    # executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    # executor.add_node(sr)
    # executor.add_node(rba)

    # try:
    #     executor.spin()  # Spin both nodes
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     sr.destroy_node()
    #     rba.destroy_node()
    #     rclpy.shutdown()
    while True:
        sr.tm.spin_once()
        rclpy.spin_once(sr)
