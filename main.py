import cv2
import time
import numpy as np
import serial

webcam = cv2.VideoCapture(0)
arduino = serial.Serial(port='COM6', baudrate=112500, timeout=0.1)

def write_data(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.06)
    data = arduino.readline()
    return data


while(1):
    _, imageFrame = webcam.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    blue_lower = np.array([133, 90, 110], np.uint8)
    blue_upper = np.array([175, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    kernal = np.ones((5, 5), "uint8")
    blue_mask = cv2.dilate(blue_mask, kernal)

    bit_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)

    contours, hierachy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        global value
        value = 0
        if(value == "12"):
            write_data("0")
            value = "0"
        if(area > 300):
            value = write_data("11")
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,0,255), 2)
            cv2.putText(imageFrame, "red color", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
    cv2.imshow("nha dien mau esp32cam", imageFrame)
    if cv2.waitKey(10) & 0xff == ord('q'):
        imageFrame.release()
        cv2.destroyWindow()
        break