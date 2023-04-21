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

from geometry_msgs.msg import Twist, Vector3


class Connector(Node):

    def __init__(self):
        drone = tellopy.Tello()
        drone.connect()
        drone.wait_for_connection(60)
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
        drone.subscribe(drone.EVENT_LOG_DATA, handler)

        # skip first 300 frames
        frame_skip = 300    
        
        super().__init__('connector')
        self.publisher_ = self.create_publisher(String, 'video_feed', 10)

        self.subscription = self.create_subscription(
            Twist,
            'movement',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        def reciever(self, drone):
            global frame_skip
            try:
                container = av.open(drone.get_video_stream())
                
                while True:
                    for frame in container.decode(video=0):
                        #print(frame)
                        if 0 < frame_skip:
                            frame_skip = frame_skip - 1
                            continue
                        start_time = time.time()
                        
                        image = cv.rotate(cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR), cv.ROTATE_180)

                        self.msg.data = image

                        if frame.time_base < 1.0/60:
                            time_base = 1.0/60
                        else:
                            time_base = frame.time_base
                        frame_skip = int((time.time() - start_time)/time_base)
            except Exception as ex:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                traceback.print_exception(exc_type, exc_value, exc_traceback)
                print(ex)
            
        msg = String()
        reciever(self, frame_skip)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def handler(event, sender, data, **args):
    global prev_flight_data
    global flight_data
    global log_data
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        if prev_flight_data != str(data):
            #print(data)
            prev_flight_data = str(data)
        flight_data = data
    elif event is drone.EVENT_LOG_DATA:
        #print(data)
        log_data = data
    else:
        print('event="%s" data=%s' % (event.getname(), str(data)))


def main(args=None):
    rclpy.init(args=args)

    connector = Connector()

    rclpy.spin(connector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    connector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()