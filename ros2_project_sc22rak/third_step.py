# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image,LaserScan
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal
# import matplotlib.pyplot as plt

class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()

        self.sensitivity = 15
        self.subscription = self.create_subscription(Image,'camera/image_raw',self.callback,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # LIDAR
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.min_distance = 0.3  
        self.obstacle_detected = False 

        self.left_distance = float('inf')  
        self.right_distance = float('inf')
        
        self.moving = True  
        self.twist = Twist()  

        cv2.namedWindow('Filtered Camera Feed', cv2.WINDOW_NORMAL)

    def avoid_obstacle(self, left_distance, right_distance):
        self.stop()
        time.sleep(0.5)  # Pause before turning

        if left_distance > self.min_distance and right_distance > self.min_distance:
            print("No walls nearby, moving into open space")
            self.explore_open_space()
        elif left_distance > right_distance:
            print("Following right wall")
            self.turn_left()
        else:
            print("Following left wall")
            self.turn_right()

        time.sleep(0.5)  
        self.move_forward()
        self.obstacle_detected = False

    def explore_open_space(self):
        print("Exploring open area...")
        self.turn_left(duration=2)  # Turn towards the open space
        self.move_forward(speed=0.3)  # Move forward a bit more aggressively
        time.sleep(2)  # Move for 2 seconds
        self.turn_right(duration=2)  # Reorient toward original direction



    def turn_left(self, speed=0.2, duration=1):

        self.twist.linear.x = 0.0
        self.twist.angular.z = speed  # Positive value turns left
        self.publisher.publish(self.twist)
        time.sleep(duration)  # Let the turn complete
        self.stop() 

    def turn_right(self, speed=0.2, duration=1):
 
        self.twist.linear.x = 0.0
        self.twist.angular.z = -speed  # Negative value turns right
        self.publisher.publish(self.twist)
        time.sleep(duration)  
        self.stop()  



    def scan_callback(self, data):
        # Check distances in all directions (360° scan)
        min_distance_all = min(data.ranges) if data.ranges else float('inf')        # Store distances for obstacle avoidance
     
        front_ranges = data.ranges[0:30] + data.ranges[-30:]  
        left_ranges = data.ranges[60:90]  
        right_ranges = data.ranges[-90:-60]  

        self.min_distance = 1.5  

        min_front = min(front_ranges) if front_ranges else float('inf')
        min_left = min(left_ranges) if left_ranges else float('inf')
        min_right = min(right_ranges) if right_ranges else float('inf')

        self.left_distance = min_left
        self.right_distance = min_right

        if self.moving is False: 
            return

        if min_front > 1.5 and min_left > 1.5 and min_right > 1.5:
            print("No walls detected nearby. Exploring open space...")
            self.explore_open_space()
        elif min_front < self.min_distance:
            print(f"Obstacle detected {min_front:.2f}m ahead! Turning now.")
            self.obstacle_detected = True
            self.avoid_obstacle(min_left, min_right)
        else:
            self.obstacle_detected = False

        self.closest_distance = min_distance_all

    def move_forward(self, speed=0.2):

        self.twist.linear.x = speed
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

    def stop(self):

        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

    def callback(self, data):

        try:
            
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert the image from BGR to HSV
            # https://stackoverflow.com/questions/12357732/hsv-color-ranges-table
            hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

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
                print(" Red detected")
            if found_green:
                print(" Green detected")
            if found_blue:
                print(" Blue detected")
                
            if found_blue and self.closest_distance < 1.0:  
                print("Stopping Blue detected within 1 meter!")
                self.stop()
                self.moving = False  # Stop explorig
                return

            if self.moving:
                if self.obstacle_detected:
                    print("Avoiding obstacle...")
                    self.avoid_obstacle(self.left_distance, self.right_distance)
                else:
                    self.move_forward()

            # Display 
            cv2.resizeWindow('Filtered Camera Feed', 320, 240)
            cv2.imshow('Filtered Camera Feed', filtered_img)
            cv2.waitKey(3)

        except CvBridgeError as e :
            self.get_logger().error(f"Error converting Image {e}")
        

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():

    def signal_handler(sig, frame):
        rclpy.shutdown()
    # Instantiate your class
    # And rclpy.init the entire node

    rclpy.init(args=None)
    cI = colourIdentifier()


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()


    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    

# Check if the node is executing in the main path
if __name__ == '__main__':
    main()


# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/uolstore/home/users/sc22rak/3edyear/robotics/ros2_ws/src/lab5/map/map.yaml 

