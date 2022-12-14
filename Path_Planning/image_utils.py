from xml.etree.ElementTree import tostring
import cv2
import numpy as np
import imutils


def getImages(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    result_Red = image.copy()
    result_Green = image.copy()
    lower_green = np.array([55, 0, 0])
    upper_green = np.array([68, 255, 255])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # lower boundary RED color range values; Hue (0 - 10)
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])
    # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160, 100, 20])
    upper2 = np.array([179, 255, 255])
    lower_mask = cv2.inRange(hsv, lower1, upper1)
    upper_mask = cv2.inRange(hsv, lower2, upper2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    full_mask = lower_mask + upper_mask
    result_Red = cv2.bitwise_and(result_Red, result_Red, mask=full_mask)
    result_Green = cv2.bitwise_and(result_Green, result_Green, mask=mask_green)
    gray_red = cv2.cvtColor(result_Red, cv2.COLOR_BGR2GRAY)
    gray_green = cv2.cvtColor(result_Green, cv2.COLOR_BGR2GRAY)
    gray = cv2.bitwise_and(gray, cv2.bitwise_and(
        cv2.bitwise_not(gray_red), cv2.bitwise_not(gray_green)))

    ret, bw = cv2.threshold(cv2.blur(gray, (3, 3)),
                            127, 255, cv2.THRESH_BINARY)
    ret, bw_start = cv2.threshold(
        cv2.blur(gray_green, (3, 3)), 127, 255, cv2.THRESH_BINARY)
    ret, bw_goal = cv2.threshold(
        cv2.blur(gray_red, (3, 3)), 10, 255, cv2.THRESH_BINARY)

    return bw, bw_start, bw_goal


def getCircles(img):
    # detected_circles = cv2.HoughCircles(img,
    #                                     cv2.HOUGH_GRADIENT, 1, 10, param1=80,
    #                                     param2=25, minRadius=5, maxRadius=20)
    cnts = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    black = np.zeros(img.shape, dtype=np.uint8)
    # loop over the contours
    pts = []
    for c in cnts:
        # compute the center of the contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(img.shape[0]-int(M["m01"] / M["m00"]))
        
        # draw the contour and center of the shape on the image
        # cv2.drawContours(black, [c], -1,  230, 2)
        # cv2.putText(black, ""+str(cX)+" "+str(cY), (cX - 20, cY - 20),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, 240, 2)
        pts.append([cX, cY])
    
    # cv2.imshow("circles", black)
    # cv2.waitKey(0)
    return pts
