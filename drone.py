import av
import cv2 as cv
import numpy as np
import sys
import threading
import time
import traceback
from matplotlib import pyplot as plt
import copy
import tellopy
import keyboard as kb
from functools import wraps


def memoize(func):
    cache = {}

    @wraps(func)
    def wrapper(*args, **kwargs):
        key = str(args) + str(kwargs)

        return cache[key]
    
    return wrapper


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
                image = cv.rotate(cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR), cv.ROTATE_180)
                
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
    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)

############################################################
#################    AR TAG FUNCTIONS   ####################
############################################################

# Decode the tag ID and 4 digit binary and orientation
def id_decode(image):
    ret, img_bw = cv.threshold(image, 200, 255, cv.THRESH_BINARY)
    corner_pixel = 255
    cropped_img = img_bw[50:150, 50:150]

    block_1 = cropped_img[37, 37]
    block_3 = cropped_img[62, 37]
    block_2 = cropped_img[37, 62]
    block_4 = cropped_img[62, 62]
    white = 255
    if block_3 == white:
        block_3 = 1
    else:
        block_3 = 0
    if block_4 == white:
        block_4 = 1
    else:
        block_4 = 0
    if block_2 == white:
        block_2 = 1
    else:
        block_2 = 0
    if block_1 == white:
        block_1 = 1
    else:
        block_1 = 0

    if cropped_img[85, 85] == corner_pixel:
        return list([block_3, block_4, block_2, block_1]), "BR"
    elif cropped_img[15, 85] == corner_pixel:
        return list([block_4, block_2, block_1, block_3]), "TR"
    elif cropped_img[15, 15] == corner_pixel:
        return list([block_2, block_1, block_3, block_4]), "TL"
    elif cropped_img[85, 15] == corner_pixel:
        return list([block_1, block_3, block_4, block_2]), "BL"

    return None, None


# Function to return the order of points in camera frame
def order(pts):
    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    # print(np.argmax(s))
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    # print(np.argmax(diff))
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    # return the ordered coordinates
    return rect

# Function to compute homography between world and camera frame 
def homograph(p, p1):
    A = []
    p2 = order(p)

    for i in range(0, len(p1)):
        x, y = p1[i][0], p1[i][1]
        u, v = p2[i][0], p2[i][1]
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y, -u])
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y, -v])
    A = np.array(A)
    U, S, Vh = np.linalg.svd(A)
    l = Vh[-1, :] / Vh[-1, -1]
    h = np.reshape(l, (3, 3))
    # print(l)
    # print(h)
    return h

# Generate contours to detect corners of the tag
def contour_generator(frame):
    test_img1 = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    test_blur = cv.GaussianBlur(test_img1, (5, 5), 0)
    edge = cv.Canny(test_blur, 150, 255)
    edge1 = copy.copy(edge)
    contour_list = list()

    cnts, h = cv.findContours(edge1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    index = list()
    for hier in h[0]:
        if hier[3] != -1:
            index.append(hier[3])

    # loop over the contours
    for c in index:
        peri = cv.arcLength(cnts[c], True)
        approx = cv.approxPolyDP(cnts[c], 0.02 * peri, True)

        if len(approx) > 4:
            peri1 = cv.arcLength(cnts[c - 1], True)
            corners = cv.approxPolyDP(cnts[c - 1], 0.02 * peri1, True)
            contour_list.append(corners)

    new_contour_list = list()
    for contour in contour_list:
        if len(contour) == 4:
            new_contour_list.append(contour)
    final_contour_list = list()
    for element in new_contour_list:
        if cv.contourArea(element) < 20000:
            final_contour_list.append(element)

    return final_contour_list

# Reorient the tag based on the original orientation
def reorient(location, maxDim):
    if location == "BR":
        p1 = np.array([
            [0, 0],
            [maxDim - 1, 0],
            [maxDim - 1, maxDim - 1],
            [0, maxDim - 1]], dtype="float32")
        return p1
    elif location == "TR":
        p1 = np.array([
            [maxDim - 1, 0],
            [maxDim - 1, maxDim - 1],
            [0, maxDim - 1],
            [0, 0]], dtype="float32")
        return p1
    elif location == "TL":
        p1 = np.array([
            [maxDim - 1, maxDim - 1],
            [0, maxDim - 1],
            [0, 0],
            [maxDim - 1, 0]], dtype="float32")
        return p1

    elif location == "BL":
        p1 = np.array([
            [0, maxDim - 1],
            [0, 0],
            [maxDim - 1, 0],
            [maxDim - 1, maxDim - 1]], dtype="float32")
        return p1

# main function to process the tag
@memoize
def image_process(frame, p1):
    if process_image == True:  # Setting for enabling/disabling the processing of the drone video feed
        try:
            final_contour_list = contour_generator(frame)

            # Computing the FOV center pixel
            height, width = frame.shape[:2]
            frame_mid_pixel = [height/2, width/2]
            
            # Computing euclidian distance to all points [y,x] on the contour from the center pixel
            distance_to_contour = []
            for i in range(len(final_contour_list)):
                euclidian_distance = np.sqrt(((final_contour_list[i])[0]-frame_mid_pixel[0])**2+((final_contour_list[i])[1]-frame_mid_pixel[1])**2) 
                distance_to_contour.append(euclidian_distance)
            # Computing the mean euclidian distance
            mean_distance_to_contour = np.mean(distance_to_contour)
            #print(mean_distance_to_contour)
            
            augmented_image_list = list()
            for i in range(len(final_contour_list)):
                cv.drawContours(frame, [final_contour_list[i]], -1, (0, 255, 0), 2)
                #cv.imshow("Outline", frame)

                # warped = homogenous_transform(small, final_contour_list[i][:, 0])

                c_rez = final_contour_list[i][:, 0]
                H_matrix = homograph(p1, order(c_rez))

                # H_matrix = homo(p1,order(c))
                tag = cv.cvtColor(cv.warpPerspective(frame, H_matrix, (200, 200)), cv.COLOR_BGR2GRAY)
                            
                height, width = tag.shape[:2]

                # Calculate threshold value
                threshold_val_white = 200  # low limit with high being 255
                threshold_val_black = 55  # high limit with low being 0

                # Apply thresholding
                ret, thresh_white = cv.threshold(tag, threshold_val_white, 255, cv.THRESH_BINARY)
                ret, thresh_black = cv.threshold(tag, 0, threshold_val_black, cv.THRESH_BINARY)

                # Count number of pixels below threshold
                count_white = cv.countNonZero(thresh_white)
                count_black = cv.countNonZero(thresh_black)

                # Calculate percentage of pixels below threshold
                white_percentage_below_threshold = (count_white / (height * width)) * 100
                black_percentage_below_threshold = (count_black / (height * width)) * 100

                if show_drone_video == True:
                    cv.imshow("Tello", frame)
                
                if show_detected_ar_tag == True and white_percentage_below_threshold <= 25 and black_percentage_below_threshold >= 75:
                    cv.imshow("AR TAG", tag)

                tag1 = cv.cvtColor(tag, cv.COLOR_BGR2GRAY)
                decoded, location = id_decode(tag1)
                empty = np.full(frame.shape, 0, dtype='uint8')
                if not location == None:
                    p2 = reorient(location, 200)
                    if not decoded == None:
                        print("ID detected: " + str(decoded))
                    H_augmented_image = homograph(order(c_rez), p2)
                    augmented_image_overlap = cv.warpPerspective(augmented_image_resize, H_augmented_image, (frame.shape[1], frame.shape[0]))
                    if not np.array_equal(augmented_image_overlap, empty):
                        augmented_image_list.append(augmented_image_overlap.copy())
                        # print(augmented_image_overlap.shape)
            
            if show_augmented_image_on_drone_video == True:
                mask = np.full(frame.shape, 0, dtype='uint8')
                if augmented_image_list != []:
                    for augmented_image in augmented_image_list:
                        temp = cv.add(mask, augmented_image.copy())
                        mask = temp

                    augmented_image_gray = cv.cvtColor(mask, cv.COLOR_BGR2GRAY)
                    r, augmented_image_bin = cv.threshold(augmented_image_gray, 10, 255, cv.THRESH_BINARY)

                    mask_inv = cv.bitwise_not(augmented_image_bin)

                    mask_3d = frame.copy()
                    mask_3d[:, :, 0] = mask_inv
                    mask_3d[:, :, 1] = mask_inv
                    mask_3d[:, :, 2] = mask_inv
                    img_masked = cv.bitwise_and(frame, mask_3d)
                    final_image = cv.add(img_masked, mask)

                    cv.imshow('Tello', final_image)
                    cv.waitKey(1)

                    if cv.waitKey(1) & 0xff == 27:
                        cv.destroyAllWindows()
        except:
            if show_drone_video == True:
                cv.imshow('Tello', frame)
                cv.waitKey(1)
    
    if show_drone_video == True:
        cv.imshow('Tello', frame)
        cv.waitKey(1)

###############################################################
#################    MOVEMENT FUNCTIONS    ####################
###############################################################

def drone_movement(drone):
    global remote_control
    global speed_remote_control
    global drone_flying
    
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
            if kb.is_pressed("W"):
                drone.set_pitch(1)
                while kb.is_pressed("W"):
                    redundant = True
            else:
                drone.set_pitch(0)

            if kb.is_pressed("S"):
                drone.set_pitch(-1)
                while kb.is_pressed("S"):
                    redundant = True
            else:
                drone.set_pitch(0)

            if kb.is_pressed("A"):
                drone.set_roll(-1)
                while kb.is_pressed("A"):
                    redundant = True
            else:
                drone.set_roll(0)

            if kb.is_pressed("D"):
                drone.set_roll(1)
                while kb.is_pressed("D"):
                    redundant = True
            else:
                drone.set_roll(0)


            ## ARROWS ##
            if kb.is_pressed("UP"):
                drone.set_throttle(1)
                while kb.is_pressed("UP"):
                    redundant = True
            else:
                drone.set_throttle(0)

            if kb.is_pressed("DOWN"):
                drone.set_throttle(-1)
                while kb.is_pressed("DOWN"):
                    redundant = True
            else:
                drone.set_throttle(0)

            if kb.is_pressed("LEFT"):
                drone.set_yaw(-1)
                while kb.is_pressed("LEFT"):
                    redundant = True
            else:
                drone.set_yaw(0)

            if kb.is_pressed("RIGHT"):
                drone.set_yaw(1)
                while kb.is_pressed("RIGHT"):
                    redundant = True
            else:
                drone.set_yaw(0)


##############################################################
#################    MAIN  &  SETTINGS    ####################
##############################################################

## SETTINGS ##

# video settings
show_drone_video = True
show_detected_ar_tag = True
show_augmented_image_on_drone_video = True
process_image = False

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
    threading.Thread(target=recv_thread, args=[drone]).start()
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

            if current_image is not new_image:
                cv.resize(new_image, (0, 0), fx=0.5, fy=0.5)
                image_process(new_image, p1)
                current_image = new_image

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

            if movement_enabled == True:
                threading.Thread(target=drone_movement, args=[drone]).start()

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
