import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy

cap = cv2.VideoCapture('DJI_0035.mov')

def distance(c1, c2):
    return (c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2

def detect_contour(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    thresh = cv2.inRange(hsv, (150, 100, 100), (180, 255, 255))

    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = []
    centers = []
    area_ratios = []

    cv2.imshow('Image 2 Matches', thresh)
    # Draw contours
    for i in contours:
        M = cv2.moments(i)
        if cv2.contourArea(i) > 30:
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

def detect_payload(prev_centers, frame):
    #print(prev_centers)
    new_contours, new_centers = detect_contour(frame)

    contours = []
    centers = []
    for c1 in prev_centers:
        for i, c2 in enumerate(new_centers):
            if distance(c1, c2) < 800:
                contours.append(new_contours[i])
                centers.append(c2)

    #for i in range(len(new_contours)):
        #cv2.drawContours(frame, [new_contours[i]], -1, (0, 0, 255), 2)

    for i in range(len(contours)):
        cv2.drawContours(frame, [contours[i]], -1, (0, 255, 0), 2)
        cv2.circle(frame, centers[i], 2, (0, 0, 255), -1)

    #cv2.imshow('Image 2 Matches', frame)
    return centers[:min(len(centers), 10)]

ret, prev = cap.read()
prev = cv2.resize(prev, (2704 // 2, 1520 // 2))
_, prev_centers = detect_contour(prev)

while cap.isOpened():
    ret, frame = cap.read()
    frame = cv2.resize(frame, (2704 // 2, 1520 // 2))
    if frame is not None:
        new_centers = detect_payload(prev_centers, frame)
        prev_centers = new_centers
        if not prev_centers:
            _, prev_centers = detect_contour(frame)
    if cv2.waitKey(1) == ord('q'):
        break