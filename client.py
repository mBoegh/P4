import av
import cv2 as cv
import numpy as np
import sys
import threading
import time
import traceback
from matplotlib import pyplot as plt
import tellopy
import keyboard as kb
from functools import wraps
import socket
import datetime
import base64

class ClientSocket:
    def __init__(self, ip, port):
        self.TCP_SERVER_IP = ip
        self.TCP_SERVER_PORT = port
        self.connectCount = 0
        self.connectServer()

    def connectServer(self):
        try:
            self.sock = socket.socket()
            self.sock.connect((self.TCP_SERVER_IP, self.TCP_SERVER_PORT))
            print(u'Client socket is connected with Server socket [ TCP_SERVER_IP: ' + self.TCP_SERVER_IP + ', TCP_SERVER_PORT: ' + str(self.TCP_SERVER_PORT) + ' ]')
            self.connectCount = 0
            self.sendImages()
        except Exception as e:
            print(e)
            self.connectCount += 1
            if self.connectCount == 10:
                print(u'Connect fail %d times. exit program'%(self.connectCount))
                sys.exit()
            print(u'%d times try to connect with server'%(self.connectCount))
            self.connectServer()

    def sendImages(self, Image):
        cnt = 0
        capture = Image
        capture.set(cv.CAP_PROP_FRAME_WIDTH, 480)
        capture.set(cv.CAP_PROP_FRAME_HEIGHT, 315)
        try:
            while capture.isOpened():
                ret, frame = capture.read()
                resize_frame = cv.resize(frame, dsize=(480, 315), interpolation=cv.INTER_AREA)

                now = time.localtime()
                stime = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')

                encode_param=[int(cv.IMWRITE_JPEG_QUALITY),90]
                result, imgencode = cv.imencode('.jpg', resize_frame, encode_param)
                data = np.array(imgencode)
                stringData = base64.b64encode(data)
                length = str(len(stringData))
                self.sock.sendall(length.encode('utf-8').ljust(64))
                self.sock.send(stringData)
                self.sock.send(stime.encode('utf-8').ljust(64))
                print(u'send images %d'%(cnt))
                cnt+=1
                time.sleep(0.095)
        except Exception as e:
            print(e)
            self.sock.close()
            time.sleep(1)
            self.connectServer()
            self.sendImages()

################################################################
#################    VIDEO FEED FUNCTIONS   ####################
################################################################

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


def draw_text(image, text, row):
    font = cv.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_size = 24
    font_color = (255, 255, 255)
    bg_color = (0, 0, 0)
    d = 2
    height, width = image.shape[:2]
    left_mergin = 10
    if row < 0:
        pos = (left_mergin, height + font_size * row + 1)
    else:
        pos = (left_mergin, font_size * (row + 1))
    
    # Draw text on image
    '''
    cv.putText(image, text, pos, font, font_scale, bg_color, 6)
    cv.putText(image, text, pos, font, font_scale, font_color, 1)
    '''

def recv_thread(drone):
    global run_recv_thread
    global new_image
    global flight_data
    global log_data
    global rotate_image

    try:
        container = av.open(drone.get_video_stream())
        # skip first 300 frames
        frame_skip = 300
        while True:
            for frame in container.decode(video=0):
                #print(frame)
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                
                if rotate_image == True:
                    image = cv.rotate(cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR), cv.ROTATE_180)
                else:
                    image = cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR)

                # Draw text on image
                '''
                if flight_data:
                    draw_text(image, 'Drone ' + str(flight_data), 0)
                if log_data:
                    draw_text(image, 'MVO: ' + str(log_data.mvo), -3)
                    draw_text(image, ('IMU: ' + str(log_data.imu))[0:52], -2)
                    draw_text(image, '     ' + ('IMU: ' + str(log_data.imu))[52:], -1)
                '''
                new_image = image
                if frame.time_base < 1.0/60:
                    time_base = 1.0/60
                else:
                    time_base = frame.time_base
                frame_skip = int((time.time() - start_time)/time_base)

                client.sendImages(new_image)

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)


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
    global new_image
    global run_recv_thread
    current_image = None

    drone = tellopy.Tello()
    drone.connect()
    drone.wait_for_connection(60)
    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.subscribe(drone.EVENT_LOG_DATA, handler)

    #if receive_drone_video == True: ## Setting for turning on/off receiving drone video feed data over UDP
    threading.Thread(target=recv_thread, args=[drone]).start()

    if movement_enabled == True: ## Setting for turning on/off movement in general
        threading.Thread(target=drone_movement, args=[drone]).start()

    if xy_plot_setting == True:
        fig, plot = plt.subplots()
        plot_points_x = []
        plot_points_y = []
        prev_plot_points_x = []
        prev_plot_points_y = [] 
        plot_color = ["green", "red", "yellow", "magenta", "pink", "cyan"]
        key_pressed = 0

    num = 0
    try:
        while 1:
            time.sleep(0.01)

            if xy_plot_setting == True:
                if num % 10 == 0:
                    posx = drone.log_data.mvo.pos_x
                    posy = drone.log_data.mvo.pos_y
                    plot_points_x.append(posx)
                    plot_points_y.append(posy)
                num += 1
                
                # Upon pressing the "p" button on the keyboard the current data is plotted in the following process
                if kb.is_pressed("å"):
                    if len(plot_points_x) > 0 and len(plot_points_y) > 0 and len(plot_points_x) == len(plot_points_y):
                        
                        # plot current data as blue
                        for i in range(len(plot_points_x)):
                            plot.scatter(plot_points_x[i], plot_points_y[i], c="blue")
                        
                        # plot previous data as a color corresponding to how many iterations ago the data was collected with newest being green and oldest being cyan (see the plot_color array)
                        if len(prev_plot_points_x) > 0 and len(prev_plot_points_y) > 0 and len(prev_plot_points_x) == len(prev_plot_points_y):
                            for i in range(len(prev_plot_points_x)):
                                plot.scatter(prev_plot_points_x[i], prev_plot_points_y[i], c=plot_color[i])
                        
                        # make plot centered around x=0 and y=0
                        x_abs_max = abs(max(plot.get_xlim(), key=abs))
                        y_abs_max = abs(max(plot.get_ylim(), key=abs))
                        plot.set_xlim(xmin=-x_abs_max, xmax=x_abs_max)
                        plot.set_ylim(ymin=-y_abs_max, ymax=y_abs_max)
                        
                        # show plot
                        plt.show()

                        # save current data for show in next plot
                        prev_plot_points_x.append([plot_points_x])
                        prev_plot_points_y.append([plot_points_y])
                        
                        key_pressed += 1

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
    
    #### CLIENT SETUP ####
    TCP_IP = 'localhost'
    TCP_PORT = 8080
    client = ClientSocket(TCP_IP, TCP_PORT)

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

    #### AR TAG SETUP ####
    augmented_image_img = cv.imread('Morshu.png', 1)
    augmented_image_resize = cv.resize(augmented_image_img, (200, 200))
    dim = 200
    p1 = np.array([
        [0, 0],
        [dim - 1, 0],
        [dim - 1, dim - 1],
        [0, dim - 1]], dtype="float32")
    
    main()