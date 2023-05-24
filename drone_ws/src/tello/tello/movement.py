#!/usr/bin/env python3

import keyboard as kb
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3


class Movement(Node):
    def __init__(self):
        remote_control = True

        #### DRONE FLIGHT SETUP ####
        drone_flying = False    
        drone_roll = 0
        drone_pitch = 0
        drone_yaw = 0
        drone_altitude = 0

        ### Initializing the node ### 
        super().__init__('movement')
        self.publisher_ = self.create_publisher(Twist, 'movement', 10) # We are publishing the node 
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Every 0.5 seconds we publish our position
        self.i = 0

    ### Message containing Position ### 
    def timer_callback(self):
        # We set our message to be:
        msg = Twist( 
            linear = Vector3(
                x = None,
                y = None,
                z = float(drone_altitude)
            ),
            angular = Vector3(
                rotX = float(drone_roll),
                rotY = float(drone_pitch),
                rotZ = float(drone_yaw)
            )
        )
        # Why are we saying Hello World? 
        msg.data = 'Hello World: %d' % self.i

        self.publisher_.publish(msg) # We send our message
        self.get_logger().info('Publishing: "%s"' % msg.data) # We send Hello world? 
        self.i += 1   #Iteration number? 


###############################################################
#################    MOVEMENT FUNCTIONS    ####################
###############################################################

def drone_movement(drone):  # is this in the loop atm? And if yes, can someone explain it to me? 
    global drone_flying
    global drone_roll
    global drone_pitch
    global drone_yaw
    global drone_altitude
    
    ### Landing and takeoff ###
    while True:
        if kb.is_pressed("q"):  # If we press q on the keyboard land
            drone.land()

        if drone_flying == False and kb.is_pressed("r"):    # If we are not flying and we press r, initiate take off (off ground)
            drone.takeoff()
            drone_flying = True

        if drone_flying == False and kb.is_pressed("t"):    # If we are not flyingand we press t, initiate take off (from hand?)
            drone.throw_and_go()
            drone_flying = True
        
        if drone_flying == True and kb.is_pressed("l"):     # If we are flying and we press l, land and stop flying? 
            drone.land()
            drone_flying = False

        
    ## WASD ##  Why are these capital when the others were not, does this make a difference? 
        if kb.is_pressed("W") and (drone_pitch == 0 or drone_pitch == -1):  # If we press w and the drone is standing still or moving backwards, move forward? 
            drone.set_pitch(1)
            drone_pitch = 1
        elif not kb.is_pressed("W") and drone_pitch == 1: # If we press w and the drone is moving forward, stop moving forward?
            drone.set_pitch(0)
            drone_pitch = 0

        if kb.is_pressed("S") and (drone_pitch == 0 or drone_pitch == 1):   # If we press s and the drone is currently standing still or moving forward, move backwards? 
            drone.set_pitch(-1)
            drone_pitch = -1
        elif not kb.is_pressed("S") and drone_pitch == -1:  #If we press s and the drone is moving backwards, stop moving backwards? 
            drone.set_pitch(0)
            drone_pitch = 0

        if kb.is_pressed("A") and (drone_roll == 0 or drone_roll == 1): # If a is pressed and the drone is standing still or moving the the right, move to the left? 
            drone.set_roll(-1)
            drone_roll = -1
        elif not kb.is_pressed("A") and drone_roll == -1:   # If a is pressed and the drone is moving to the left, stop moving to the left? 
            drone.set_roll(0)
            drone_roll = 0

        if kb.is_pressed("D") and (drone_roll == 0 or drone_roll == -1):    # If d is pressed and the drone is standing still or moving to the left, move to the right? 
            drone.set_roll(1)
            drone_roll = 1
        elif not kb.is_pressed("D") and drone_roll == 1:    # If d is pressed and the drone is moving to the right, stop moving to the right? 
            drone.set_roll(0)
            drone_roll = 0


        ## ARROWS ##
        # Same as WASD but just not as arrows
        if kb.is_pressed("UP") and (drone_altitude == 0 or drone_altitude == -1):
            drone.set_throttle(1)
            drone_altitude = 1
        elif not kb.is_pressed("UP") and drone_altitude == 1:
            drone.set_throttle(0)
            drone_altitude = 0

        if kb.is_pressed("DOWN") and (drone_altitude == 0 or drone_altitude == 1):
            drone.set_throttle(-1)
            drone_altitude = -1
        elif not kb.is_pressed("DOWN") and drone_altitude == -1:
            drone.set_throttle(0)
            drone_altitude = 0

        if kb.is_pressed("LEFT") and (drone_yaw == 0 or drone_yaw == 1):
            drone.set_yaw(-1)
            drone_yaw = -1
        elif not kb.is_pressed("LEFT") and drone_yaw == -1:
            drone.set_yaw(0)
            drone_yaw = 0

        if kb.is_pressed("RIGHT") and (drone_yaw == 0 or drone_yaw == -1):
            drone.set_yaw(1)
            drone_yaw = 1
        elif not kb.is_pressed("RIGHT") and drone_yaw == 1:
            drone.set_yaw(0)
            drone_yaw = 0


def main(args=None):
    rclpy.init(args=args)

    movement = Movement() # We run the movement node 

    rclpy.spin(movement)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    movement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()