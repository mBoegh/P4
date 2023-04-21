import keyboard as kb

###############################################################
#################    MOVEMENT FUNCTIONS    ####################
###############################################################

def drone_movement(drone):
    global remote_control
    global speed_remote_control
    global drone_flying
    global drone_roll
    global drone_pitch
    global drone_yaw
    global drone_altitude
    
    while True:
        if kb.is_pressed("q"):
            drone.land()

        if drone_flying == False and kb.is_pressed("r"):
            drone.takeoff()
            drone_flying = True

        if drone_flying == False and kb.is_pressed("t"):
            drone.throw_and_go()
            drone_flying = True
        
        if drone_flying == True and kb.is_pressed("l"):
            drone.land()
            drone_flying = False

        if remote_control == True and drone_flying == True:
             ## WASD ##
            if kb.is_pressed("W") and (drone_pitch == 0 or drone_pitch == -1):
                drone.set_pitch(1)
                drone_pitch = 1
            elif not kb.is_pressed("W") and drone_pitch == 1:
                drone.set_pitch(0)
                drone_pitch = 0

            if kb.is_pressed("S") and (drone_pitch == 0 or drone_pitch == 1):
                drone.set_pitch(-1)
                drone_pitch = -1
            elif not kb.is_pressed("S") and drone_pitch == -1:
                drone.set_pitch(0)
                drone_pitch = 0

            if kb.is_pressed("A") and (drone_roll == 0 or drone_roll == 1):
                drone.set_roll(-1)
                drone_roll = -1
            elif not kb.is_pressed("A") and drone_roll == -1:
                drone.set_roll(0)
                drone_roll = 0

            if kb.is_pressed("D") and (drone_roll == 0 or drone_roll == -1):
                drone.set_roll(1)
                drone_roll = 1
            elif not kb.is_pressed("D") and drone_roll == 1:
                drone.set_roll(0)
                drone_roll = 0


            ## ARROWS ##
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
