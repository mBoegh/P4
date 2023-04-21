import cv2 as cv
import numpy as np
import sys
import time
import traceback
import tellopy

##############################################################
#################    MAIN  &  SETTINGS    ####################
##############################################################

## SETTINGS ##

# video settings
receive_drone_video = True
show_drone_video = True
rotate_image = False

process_image = False
show_detected_ar_tag = True
show_augmented_image_on_drone_video = True

# plot settings
xy_plot_setting = False  # Unfininshed

# Drone settings
movement_enabled = True

remote_control = True
speed_remote_control = 50


## MAIN ##

def main():
    num = 0
    try:
        while 1:
            time.sleep(0.01)
            if current_image is not new_image:
                cv.resize(new_image, (0, 0), fx=0.5, fy=0.5)
                image_process(new_image, p1)
                current_image = new_image

    except KeyboardInterrupt as e:
        print(e)
    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(e)

    run_recv_thread = False
    cv.destroyAllWindows()
    drone.quit()
    exit(1)

if __name__ == "__main__":
    
    #### VIDEO FEED SETUP ####
    prev_flight_data = None
    run_recv_thread = True
    new_image = None
    flight_data = None
    log_data = None
    
    #### DRONE FLIGHT SETUP ####
    drone_flying = False
    drone_roll = 0
    drone_pitch = 0
    drone_yaw = 0
    drone_altitude = 0
    
    main()