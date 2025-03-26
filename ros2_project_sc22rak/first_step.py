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
import time
from math import sin, cos, pi


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

        self.goals = [(7.0, 5.19, 0.0), (-10.8,3.6, 0.0), (-9.5, -14.7, 0.0), (9.0, -13.3, 0.0)]
        self.current_goal_index = 0       

        self.closest_distance = float('inf') 
        self.moving = True
        self.obstacle_detected = False 
        self.blue_detected = False 
        self.moving_towards_blue = False

        cv2.namedWindow('Filtered Camera Feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Filtered Camera Feed', 320, 240)

    def send_next_goal(self):
        if not self.blue_detected:  # Only move to next goal if blue is not detected
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
        """ Callback when a goal is completed. """
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        
        # Rotate in place after reaching the goal
        self.look_around()

        # Move to the next goal
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # # Access the current pose
        # current_pose = feedback_msg.feedback.current_pose
        # position = current_pose.pose.position
        # orientation = current_pose.pose.orientation

        # # Access other feedback fields
        # navigation_time = feedback_msg.feedback.navigation_time
        # distance_remaining = feedback_msg.feedback.distance_remaining

        # # Print or process the feedback data
        # self.get_logger().info(f'Current Pose: [x: {position.x}, y: {position.y}, z: {position.z}]')
        # self.get_logger().info(f'Distance Remaining: {distance_remaining}')


    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def move_forward(self, speed=0.2):
        """ Moves the robot forward at a specified speed. """
        twist = Twist()  # Create a new Twist message
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)
    
    def look_around(self):
        """ Rotates the robot 360 degrees in place. """
        self.get_logger().info("Looking around...")

        twist = Twist()
        twist.angular.z = pi / 4  # Rotate at 45 degrees/sec

        # Rotate for 8 seconds to complete 360 degrees
        start_time = time.time()
        while time.time() - start_time < 8:
            self.publisher.publish(twist)
            time.sleep(0.1)

        # Stop rotation
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Finished looking around.")

    def move_towards_blue(self, x_offset):
        """
        Move towards the detected blue object.
        Adjusts based on the position of the object in the camera frame.
        """
        twist = Twist()

        # Move forward
        twist.linear.x = 0.2

        # Adjust direction based on x_offset
        if x_offset < -20:
            twist.angular.z = 0.1  # Turn left
        elif x_offset > 20:
            twist.angular.z = -0.1  # Turn right
        else:
            twist.angular.z = 0.0  # Move straight

        self.publisher.publish(twist)


    def stop_moving_towards_blue(self):
        """ Stop the robot from moving towards blue and allow it to proceed to the next goal. """
        self.moving_towards_blue = False
        self.get_logger().info("Stopped moving towards blue object.")

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
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])  # X position of object in camera view
                            img_center_x = self.cv_image.shape[1] // 2
                            x_offset = cx - img_center_x  # Offset from center
                            return x_offset
                return None

            found_red = process_contours(red_mask, "red")
            found_green = process_contours(green_mask, "green")
            found_blue = process_contours(blue_mask, "blue")

            if found_red:
                print("Red detected")
            if found_green:
                print("Green detected")

            if found_blue:
                self.get_logger().info("Blue detected!")
                self.blue_detected = True  # Set the flag to True

                # Ensure the x, y positions are float values
                x = float(self.cv_image.shape[1] // 2)  # Convert to float
                y = float(self.cv_image.shape[0] // 2)  # Convert to float
                self.send_goal(x, y, 0.0)  # Set goal to the center of the image (assuming blue is center)
                
                self.move_towards_blue(found_blue)

            else:
                if self.blue_detected:  # Only stop if we were moving towards blue
                    self.stop_moving_towards_blue()
                    self.blue_detected = False
                    self.send_next_goal()

            # Display 
            cv2.resizeWindow('Filtered Camera Feed', 320, 240)
            cv2.imshow('Filtered Camera Feed', filtered_img)  
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    def signal_handler(sig, frame):
        rclpy.shutdown()

    rclpy.init(args=args)
    go_to_pose = GoToPose()
    go_to_pose.send_next_goal()

    signal.signal(signal.SIGINT, signal_handler)

    # Send goal coordinates
    # go_to_pose.send_goal(-4.7, -9.0, 0.0)

    try:
        rclpy.spin(go_to_pose)  # Keep the node alive
    except KeyboardInterrupt:
        print("Shutting down")

    # Cleanup before exiting
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()