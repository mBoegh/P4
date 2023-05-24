#!/usr/bin/env python3

import tellopy
import av
import cv2 as cv
import numpy as np
import sys
import time
import traceback
import keyboard as kb

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

class Video_publisher(Node):
    def __init__(self):
        self.drone = tellopy.Tello()
        self.drone.connect()
        self.drone.wait_for_connection(60)

        # Creating a publisher for the topic '/video_feed' on which the tello drone video feed will be publishing frame by frame
        super().__init__('video_publisher')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.sub = self.create_subscription(Twist, 'drone_vel', self.twist_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.frame_skip = 0
        self.startupframes = 300
        self.x = 0
        self.y = 0
        # self.drone.takeoff()  

    def twist_callback(self, data):
        print("Go fuck yourself :)")
        self.x = data.angular.x
        self.y = data.angular.y
        print(self.x)
        print(self.y)
        self.drone.set_pitch(self.x/10)
        self.drone.set_roll(self.y/10)

    def reciever(self, drone):
        print("I'm a reciever")

        global startupframes
        try:
            container = av.open(self.drone.get_video_stream())
            d = 0
            print("HERE IT COMES!")
            print(next(container.decode(video=0)))
            for frame in container.decode(video=0):
                
                # if 0 < self.startupframes:
                #     self.startupframes = self.startupframes - 1
                #     continue
                start_time = time.time()
                print("For frame "+str(d))
                print("x=" + str(self.x) + " y=" + str(self.y))
                d+=1
                # In this oneliner the frame from the drone is converted from RGB to BGR, rotated 180 degrees to adjust for the inverted image
                # coming from capturing the light trough the slanted mirror, and then it is converted from CV2 to ROS image type.
                # Lastly it is published to the topic '/video_feed'.
                self.pub.publish(self.bridge.cv2_to_imgmsg(cv.rotate(cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR), cv.ROTATE_180), "bgr8"))

                if frame.time_base < 1.0/60:
                    time_base = 1.0/60
                else:
                    time_base = frame.time_base
                self.frame_skip = int((time.time() - start_time)/time_base)
                break
            print("number of frame: " + str(d))
        except Exception as ex:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            print(ex)

        

    def timer_callback(self):
        print("LOOK AT THIS -----> " + str(self.i) + " <-----")
        # if kb.is_pressed("r"):    # If we are not flying and we press r, initiate take off (off ground)
        #     self.drone.takeoff()  
        # if kb.is_pressed("q"):  # If we press q on the keyboard land
        #     self.drone.land()
        self.reciever(self.frame_skip)
        # self.twist_callback()
        self.i += 1

def main(args=None):

    rclpy.init(args=args)

    # skip first 300 frames
    # global startupframes
    # startupframes = 300

    # global frame_skip 
    

    videofeed_publisher = Video_publisher()

    rclpy.spin(videofeed_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    videofeed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
