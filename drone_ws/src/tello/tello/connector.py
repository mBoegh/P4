#!/usr/bin/env python3

import tellopy
import av
import cv2 as cv
import numpy as np
import sys
import time
import traceback
import keyboard as kb
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

class Video_publisher(Node):
    def __init__(self):
        # Initialise the Tello drone and set the exposure
        self.drone = tellopy.Tello()
        self.drone.connect()
        self.drone.wait_for_connection(60)
        self.drone.set_exposure(0)

        self.last_twist = time.time()   # used for keeping track of the time when the last twist message was recieved 
        self.takeoff_time = time.time() # used for delay before taking off
        self.climb_time = 0             # used to control the rise time of the drone during takeoff
        self.takeoff_state = True       # has the drone taken off yet
        self.adjust = False             # sets wheter the drone should adjust its position relative to the robot
        self.not_found = True           # used to control wheter the 
        threading.Thread(target=self.reciever, args=[self.not_found]).start() # starts a thread running the reciver function

        ########## Creating a publisher for the topic '/video_feed' on which the tello drone video feed will be publishing frame by frame
        super().__init__('video_publisher')

        self.bridge = CvBridge() # Creates bridge to convert between ROS and opencv formats
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10) # Publisher that publishes drone video frames to the /camera/image_raw topic
        self.sub = self.create_subscription(Twist, 'drone_vel', self.twist_callback, 10) # Subscriber that subs to the drone_vel topic and runs twist_callback
        timer_period = 0.1  # sets how often the timer triggers, set to 0.1 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Timer that runs timer_callback every 0.1 seconds

        self.i = 0 # variable that is iterated throughout the runtime of the program
        # self.frame_skip = 0         
        # self.startupframes = 300

        # variables that hold the roll pitch and yaw commands recieved from the drone_vel topic
        self.x = 0 
        self.y = 0
        self.z = 0
        
    # Function is run whenever a new twist message is posted to the drone_vel topic
    def twist_callback(self, data):
        print("Twist recieved")         # Print a debugging message to the terminal
        self.last_twist = time.time()   # Reset the last_twist timer
        if self.adjust:                 # If the drone is in a state where it should adjust it's position, then...
            self.x = data.angular.x     # Set roll...
            self.y = data.angular.y     # ...pitch...
            self.z = data.angular.z     # ...and yaw

            self.drone.set_pitch((self.y/2)) # Send the pitch command to the drone. Positive = forward
            self.drone.set_roll((self.x/2)) # Send the roll command to the drone. Positive = right
            self.drone.set_yaw((self.z)) # Send the yaw command to the drone. Currently deactivated

    # Function that recieves video frames from the drone and publishes the to the ROS network on the /camera/image_raw topic
    def reciever(self, status):
        print("I'm a reciever") # Print a debugging message to the terminal

        # Status is always true so this loop runs forever
        while status:
            container = av.open(self.drone.get_video_stream()) # opens the video stream to the Tello drone
            status = False
            print("Container success") # Print a debugging message to the terminal
            d = 0
        
        try:
            for frame in container.decode(video=0): # run through all frames sent by the drone
                # In this oneliner the frame from the drone is converted from RGB to BGR, rotated 180 degrees to adjust for the inverted image
                # coming from capturing the light trough the slanted mirror, and then it is converted from CV2 to ROS image type.
                # Lastly it is published to the topic '/video_feed'.
                self.pub.publish(self.bridge.cv2_to_imgmsg(cv.rotate(cv.flip(cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR), 1), cv.ROTATE_180), "bgr8"))
            print("number of frame: " + str(d)) # print number of frames in batch
        except:
            print("Reciver error") # Print a debugging message to the terminal
        

    def timer_callback(self):
        print("TIMER -----> " + str(self.i) + " <-----") # Print easy-to-find timer state in the terminal

        # Wait 5 seconds from program start before taking off. This allows the operator to cancel flight and the video feed to catch up 
        if time.time() - self.takeoff_time > 5 and self.takeoff_state:
            self.takeoff_state = False      # After takeoff this state is switched so the drone cannot take off again
            self.climb_time = time.time()   # Start the climb_time timer
            print("##################TAKEOFF!###################") # Print a debugging message to the terminal
            self.drone.takeoff()            # Send the takeoff command to the drone

        # Fly directly up for the duration of the climb time, here set to 4 seconds
        if time.time() - self.climb_time < 4:
            self.drone.set_throttle(1)      # Set the upward throttle of the drone to full
            print("Climbing...")            # Print a debugging message to the terminal
        else:
            self.adjust = True              # After rise time is over turn on adjust
            self.drone.set_throttle(0)      # And, set throttle to 0

        # check the time since last twist message, if it exceeds 0.5 seconds, land the drone
        if time.time() - self.last_twist > 0.5:
            print("##################Landing!###################") # Print a debugging message to the terminal
            self.drone.land()               # Send land command to the drone

        self.i += 1 # iterate the i variable, showing how many times the timer has been called

def main(args=None):
    rclpy.init(args=args) # Initialise rclpy

    videofeed_publisher = Video_publisher() # Create an instance of the Video_publisher node

    rclpy.spin(videofeed_publisher) # Spin the video publisher

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    videofeed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

