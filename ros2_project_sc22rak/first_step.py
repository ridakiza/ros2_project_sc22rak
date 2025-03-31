import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import numpy as np
import time
from math import sin, cos, pi
from geometry_msgs.msg import Twist

class GoToPose(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bridge = CvBridge()

        self.sensitivity = 15
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.goals = [(-10.8, 3.6, 0.0), (-9.5, -14.7, 0.0), (9.0, -13.3, 0.0)]
        self.current_goal_index = 0

        self.blue_detected = False
        self.moving_towards_blue = False
        self.blue_detected_during_rotation = False

        cv2.namedWindow('Filtered Camera Feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Filtered Camera Feed', 320, 240)

    def send_next_goal(self):
        if self.moving_towards_blue:
            self.get_logger().info("Following blue, skipping goal.")
            return  

        if self.blue_detected:
            self.get_logger().info("Blue detected, prioritizing blue tracking.")
            return  

        if self.current_goal_index < len(self.goals):
            x, y, yaw = self.goals[self.current_goal_index]
            self.current_goal_index += 1
            self.send_goal(x, y, yaw)
        else:
            self.get_logger().info("All goals reached.")


    def send_goal(self, x, y, yaw):

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle  # Store the goal handle for cancellation

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        
        self.look_around()
        self.send_next_goal()

    def stop(self):

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def move_towards_blue(self, x_offset, area):
        if not self.moving_towards_blue:
            self.get_logger().info("Cancelling goal navigation to prioritize blue object.")
            
            if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()  
            
        self.moving_towards_blue = True  

        twist = Twist()
        distance_to_blue = 77000 - area

        if distance_to_blue < 1.0:
            self.get_logger().info(f"Blue object close, stopping. Area: {area}")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.moving_towards_blue = False  
            return

        twist.linear.x = 0.2
        twist.angular.z = -0.2 if x_offset > 20 else (0.2 if x_offset < -20 else 0.0)
        
        self.publisher.publish(twist)
        self.get_logger().info(f"Moving towards blue. Offset: {x_offset}, Area: {area}")

    def feedback_callback(self, feedback_msg):

        self.get_logger().info("Receiving navigation feedback...")

    def look_around(self):

        self.get_logger().info("Looking around...")
        self.blue_detected_during_rotation = False

        twist = Twist()
        twist.angular.z = pi / 4  # Rotate in place

        start_time = time.time()
        while time.time() - start_time < 8:
            self.publisher.publish(twist)
            time.sleep(0.1)

            if self.blue_detected:
                self.get_logger().info("Blue detected during rotation, stopping look around.")
                self.stop()
                self.moving_towards_blue = True
                return

        self.stop()
        self.get_logger().info("Finished looking around.")

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

            # color ranges
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

            hsv_red_lower1 = np.array([0, 100, 100])
            hsv_red_upper1 = np.array([self.sensitivity, 255, 255])
            hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
            hsv_red_upper2 = np.array([180, 255, 255])

            hsv_blue_lower = np.array([110 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([110 + self.sensitivity, 255, 255])

            # Create masks for each color
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            red_mask = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1) | cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

            # Combine 
            final_mask = cv2.bitwise_or(red_mask, cv2.bitwise_or(green_mask, blue_mask))


            def process_contours(mask):
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(c)
                    if area > 100:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            img_center_x = (self.cv_image.shape[1] // 2)
                            x_offset = cx - img_center_x
                            return x_offset, area
                return None, None

            found_blue = process_contours(blue_mask)
            self.blue_detected = found_blue[0] is not None

            if self.blue_detected:
                self.get_logger().info(f"Blue detected at offset: {found_blue[0]}, Area: {found_blue[1]}")
                self.move_towards_blue(found_blue[0], found_blue[1])
            else:
                self.moving_towards_blue = False  # Reset if blue is lost
            

            colored_objects = cv2.bitwise_and(self.cv_image, self.cv_image, mask=final_mask)


            cv2.imshow('Filtered Camera Feed', colored_objects)

            
            cv2.waitKey(3)
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    rclpy.init(args=args)
    go_to_pose = GoToPose()
    go_to_pose.send_next_goal()

    try:
        rclpy.spin(go_to_pose)
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
