import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from math import sin, cos
import cv2
import numpy as np
import threading
import signal
from geometry_msgs.msg import Twist


class GoToPose(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')
                
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bridge = CvBridge()

        self.sensitivity = 15
        self.subscription = self.create_subscription(Image,'camera/image_raw',self.callback,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        

        self.closest_distance = float('inf') 
        self.moving = True
        self.obstacle_detected = False 

        cv2.namedWindow('Filtered Camera Feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Filtered Camera Feed', 320, 240)


    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
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
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
 
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert the image from BGR to HSV
            # https://stackoverflow.com/questions/12357732/hsv-color-ranges-table

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

            # Apply filter if 1 keep if 0 throw
            filtered_img = cv2.bitwise_and(self.cv_image, self.cv_image, mask=final_mask)

            # Process contours and check if a marker is found
            def process_contours(mask, color_name):
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(c)
                    if area > 100:  # Only consider significant areas
                        # print(f"{color_name.capitalize()} color detected!")
                        return True
                return False

            found_red = process_contours(red_mask, "red")
            found_green = process_contours(green_mask, "green")
            found_blue = process_contours(blue_mask, "blue")

            if found_red:
                print("Red detected")
            if found_green:
                print("Green detected")
            if found_blue:
                print("Blue detected")

    
                blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if blue_contours:
                    largest_blue_area = cv2.contourArea(max(blue_contours, key=cv2.contourArea))

                    if largest_blue_area > 1000:  
                        print("Blue object is almost 1 meter close.")
                        self.stop()
                        self.moving = False  
                        return

            # Display 
            cv2.resizeWindow('Filtered Camera Feed', 320, 240)
            cv2.imshow('Filtered Camera Feed', filtered_img)  
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):

    def signal_handler(sig, frame):
    rclpy.shutdown()
    # Instantiate your class
    # And rclpy.init the entire node

    rclpy.init(args=args)
    go_to_pose = GoToPose()

    go_to_pose.send_goal(-4.7, -9.0, 0.0)  # goal coordinates
    rclpy.spin(go_to_pose)
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()