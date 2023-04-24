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
import socket
import datetime
import base64

class ServerSocket:
    global new_image

    def __init__(self, ip, port):
        self.TCP_IP = ip
        self.TCP_PORT = port
        self.socketOpen()
        self.receiveThread = threading.Thread(target=self.receiveImages)
        self.receiveThread.start()

    def socketClose(self):
        self.sock.close()
        print(u'Server socket [ TCP_IP: ' + self.TCP_IP + ', TCP_PORT: ' + str(self.TCP_PORT) + ' ] is close')

    def socketOpen(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.TCP_IP, self.TCP_PORT))
        self.sock.listen(1)
        print(u'Server socket [ TCP_IP: ' + self.TCP_IP + ', TCP_PORT: ' + str(self.TCP_PORT) + ' ] is open')
        self.conn, self.addr = self.sock.accept()
        print(u'Server socket [ TCP_IP: ' + self.TCP_IP + ', TCP_PORT: ' + str(self.TCP_PORT) + ' ] is connected with client')

    def receiveImages(self):

        try:
            while True:
                length = self.recvall(self.conn, 64)
                length1 = length.decode('utf-8')
                stringData = self.recvall(self.conn, int(length1))
                stime = self.recvall(self.conn, 64)
                print('send time: ' + stime.decode('utf-8'))
                now = time.localtime()
                print('receive time: ' + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f'))
                data = np.frombuffer(base64.b64decode(stringData), np.uint8)
                new_image = cv.imdecode(data, 1)
    
        except Exception as e:
            print(e)
            self.socketClose()
            cv.destroyAllWindows()
            self.socketOpen()
            self.receiveThread = threading.Thread(target=self.receiveImages)
            self.receiveThread.start()

    def recvall(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf


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
def image_process(frame, p1):
    global new_image
    global augmented_image_resize
    global p1
    
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

            cv.imshow("Tello", frame)
            
            if white_percentage_below_threshold <= 25 and black_percentage_below_threshold >= 75:
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
        cv.imshow('Tello', frame)
        cv.waitKey(1)




def main():
    new_image = None

    augmented_image_img = cv.imread('Morshu.png', 1)
    augmented_image_resize = cv.resize(augmented_image_img, (200, 200))
    dim = 200
    p1 = np.array([
        [0, 0],
        [dim - 1, 0],
        [dim - 1, dim - 1],
        [0, dim - 1]], dtype="float32")
    
    server = ServerSocket('localhost', 8080)

    while True:
        image_process(new_image)

if __name__ == "__main__":
    main()
