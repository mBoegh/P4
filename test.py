import cv2 as cv

cap = cv.VideoCapture("video.h264")
while cap.isOpened():
    ret, frame = cap.read()
    cv.imshow("test", frame)
    cv.waitKey(1)