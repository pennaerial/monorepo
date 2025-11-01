import cv2
import numpy as np
from typing import Optional, Tuple
import os

def find_hoop(
        image: np.ndarray,
):
    """
    Detect hoop center in image using color thresholding.

    Args:
        image (np.ndarray): Input BGR image.
        debug (bool): If True, return an image with visualizations.

    Returns:
        Optional[Tuple[int, int, bool]]: A tuple (cx, cy, img) if detection is successful;
        otherwise, a tuple (None, None, img).
    """
    img = cv2.resize(image, (1532//2, 796//2))
    copy = img.copy()
    copy = cv2.cvtColor(copy, cv2.COLOR_RGB2GRAY, None)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV, None)

    lower_bound = np.array([0, 50, 50])
    upper_bound = np.array([10, 255, 255])

    mask1 = cv2.inRange(hsv, lower_bound, upper_bound)
    mask2 = cv2.inRange(hsv, (0, 0, 0), (180, 80, 80))
    combined_mask = cv2.bitwise_or(mask1, mask2)
    blurred = cv2.blur(combined_mask, (7, 7), None)
    retval, thresh1 = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY_INV)
    h, w = img.shape[:2]

    ys, xs = np.where(mask1 == 255)
    if len(xs) > 0:
        avg_x = xs.mean()
        avg_y = ys.mean()
    else:
        avg_x, avg_y = None, None
    if avg_x is not None:
        if avg_x >= w//2:
            if avg_y <= h//2: #right top
                seed_points = [(0, 0), (0, h-1), (w-1, h-1)]
            else: #right bottom
                seed_points = [(0, 0), (w-1, 0), (0, h-1)]
        else:
            if avg_y <= h//2: #left top
                seed_points = [(w-1, 0), (0, h-1), (w-1, h-1)]
            else: #left bottom
                seed_points = [(0, 0), (w-1, 0), (w-1, h-1)]
    else:
        seed_points = [(0, 0), (w-1, 0), (0, h-1), (w-1, h-1)]

    mask = np.zeros((h + 2, w + 2), np.uint8)
    for point in seed_points:
        if thresh1[point[1]][point[0]] != 0:
            cv2.floodFill(thresh1, mask, point, 0)
    contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        selected_contours = contours[0]
    else: selected_contours = []
    maxArea = 0
    maxScore = 0

    cv2.imshow("thresh", mask1)
    for contour in contours:
        score = 0
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        # if area > 1000:
        #     if perimeter > 0:
        #         circularity = (4 * math.pi * area) / (perimeter ** 2)
        #         if circularity > 0.3:
        #             selected_contours.append(contour)
        if area > maxArea:
            score += area
            maxArea = area
        moments = cv2.moments(contour) #returns a dictionary of moments
        if moments['m00'] != 0:
            Cx = int(moments['m10'] / moments['m00'])
            Cy = int(moments['m01'] / moments['m00'])
        else:
            Cx = w//2
        score -= abs(Cx - w//2)
        if score > maxScore:
            selected_contours = [contour]
    for contour in selected_contours:
        # cv2.drawContours(img, contour, -1, (255, 0, 0))
        moments = cv2.moments(contour) #returns a dictionary of moments
        if moments['m00'] != 0:
            Cx = int(moments['m10'] / moments['m00'])
            Cy = int(moments['m01'] / moments['m00'])
            cv2.circle(img, (Cx, Cy), 4, (255, 255, 255), -1)
            cv2.putText(img, "Center", (Cx-20, Cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.arrowedLine(img, (w//2, h//2), (Cx, Cy ), (255, 0, 0), 2, cv2.LINE_AA, 0, 0.1)
            return Cx - w//2, Cy - h//2, img
        return None, None, img
    return None, None, img
