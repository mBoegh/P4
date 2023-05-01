#!/usr/bin/env python3

import tellopy
import av
import cv2 as cv
import numpy as np
import sys
import time
import traceback

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
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
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        

    def timer_callback(self):
        def reciever(self, drone):
            global startupframes

        
            try:
                container = av.open(self.drone.get_video_stream())

                while True:
                    for frame in container.decode(video=0):
                        if 0 < startupframes:
                            startupframes = startupframes - 1
                            continue
                        start_time = time.time()
                        
                        # In this oneliner the frame from the drone is converted from RGB to BGR, rotated 180 degrees to adjust for the inverted image
                        # coming from capturing the light trough the slanted mirror, and then it is converted from CV2 to ROS image type.
                        # Lastly it is published to the topic '/video_feed'.
                        self.pub.publish(self.bridge.cv2_to_imgmsg(cv.rotate(cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR), cv.ROTATE_180), "bgr8"))

                        if frame.time_base < 1.0/60:
                            time_base = 1.0/60
                        else:
                            time_base = frame.time_base
                        frame_skip = int((time.time() - start_time)/time_base)
            except Exception as ex:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                traceback.print_exception(exc_type, exc_value, exc_traceback)
                print(ex)
            
        reciever(self, frame_skip)
        self.i += 1


#def handler(event, sender, data, **args):
   # global prev_flight_data
   # global flight_data
    #global log_data
    #drone = sender
    #if event is drone.EVENT_FLIGHT_DATA:
     #   if prev_flight_data != str(data):
      #      #print(data)
       #     prev_flight_data = str(data)
        #flight_data = data
    #elif event is drone.EVENT_LOG_DATA:
     #   #print(data)
      #  log_data = data
    #else:
     #   print('event="%s" data=%s' % (event.getname(), str(data)))


def main(args=None):
    #drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    #drone.subscribe(drone.EVENT_LOG_DATA, handler)

    rclpy.init(args=args)

    # skip first 300 frames
    global startupframes
    startupframes = 300

    global frame_skip 
    frame_skip = 0

    videofeed_publisher = Video_publisher()

    rclpy.spin(videofeed_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    videofeed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()