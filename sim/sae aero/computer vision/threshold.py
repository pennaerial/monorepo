import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy

cap = cv2.VideoCapture('DJI_0033.mov')

def distance(c1, c2):
    return (c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2

def detect_contour(threshold, frame):
    hsl = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    hsl = cv2.GaussianBlur(hsl, (5, 5), 0)

    thresh = cv2.inRange(hsl, (0, threshold[0], 0), (180, threshold[1], 255))
    #thresh = cv2.inRange(hsl, (80, 100, 0), (180, 255, 255))
    #thresh = cv2.inRange(hsl, (140, 50, 80), (180, 255, 255))

    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = []
    centers = []
    area_ratios = []

    #cv2.imshow('Image 2 Matches', thresh)
    # Draw contours
    for i in contours:
        M = cv2.moments(i)
        if cv2.contourArea(i) > 20:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            rect = cv2.minAreaRect(i)

            (center_x, center_y), (width, height), angle = rect

            if cv2.contourArea(i) / (width * height) > 0.57:
                valid_contours.append(i)
                centers.append((cx, cy))
                area_ratios.append(cv2.contourArea(i) / (width * height))

    if valid_contours:
        total = list(zip(area_ratios, valid_contours, centers))
        total.sort(key = lambda x : x[0], reverse=True)
        _, valid_contours, centers = [list(t) for t in zip(*total)]
    
    return valid_contours, centers

def threshold(thresh, prev_center, frame):
    new_contours, new_centers = detect_contour(thresh, frame)

    contour = None
    center = None
    closest = float('inf')
    for i, c2 in enumerate(new_centers):
        center_distance = distance(prev_center, c2)
        if center_distance < 800 and center_distance < closest:
            contour = new_contours[i]
            center = c2
            closest = center_distance
            

    if contour is not None:
        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
        cv2.circle(frame, center, 2, (0, 0, 255), -1)
        contour = [x[0] for x in contour]

    #cv2.imshow('Image 2 Matches', frame)
    #print(contour)
    return contour, center

thresh = (170, 255)
ret, prev = cap.read()
prev = cv2.resize(prev, (2704 // 2, 1520 // 2))
_, prev_centers = detect_contour(thresh, prev)
prev_center = None
if prev_centers:
    prev_center = prev_centers[0]

while cap.isOpened():
    ret, frame = cap.read()
    frame = cv2.resize(frame, (2704 // 2, 1520 // 2))
    if frame is not None:
        # If the payload was in the previous frame
        if prev_center is not None:
            _, new_center = threshold(thresh, prev_center, frame)
            prev_center = new_center
        # If payload is obstructed
        else:
            _, prev_centers = detect_contour(thresh, frame)
            if prev_centers:
                prev_center = prev_centers[0]
        cv2.imshow('Image 2 Matches', frame)
    if cv2.waitKey(1) == ord('q'):
        break