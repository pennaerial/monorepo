import cv2
import numpy as np


def fit_quadrilateral(contour):
    print(contour)
    return cv2.boxPoints(cv2.minAreaRect(np.array(list(contour))))


def rect_confidence(contour, height, width):
    # Create empty masks
    mask1 = np.zeros((height, width), dtype=np.uint8)
    mask2 = np.zeros((height, width), dtype=np.uint8)

    # Convert the box into a contour (it needs to be a list of points)
    box = np.int0(fit_quadrilateral(contour))  # Convert to integer points

    # Draw the contour of the original contour on mask1
    cv2.drawContours(mask1, [contour], 0, 255, thickness=cv2.FILLED)

    # Draw the quadrilateral bounding box on mask2
    cv2.drawContours(mask2, [box], 0, 255, thickness=cv2.FILLED)

    # Compute the intersection using bitwise AND
    intersection = cv2.bitwise_xor(mask1, mask2)

    # Calculate the area of the intersection
    area = cv2.countNonZero(intersection)
    confidence = 1 - area / (cv2.contourArea(contour) + cv2.contourArea(box))
    return confidence


def confidence(contour, target_area, height, width):
    # use target_area negative as a proxy for not suppying a target area
    if target_area < 0:
        return rect_confidence(contour, height, width)
    area = cv2.contourArea(contour)
    area_confidence = (1 - area / target_area) ** 2
    if area_confidence > 1:
        area_confidence = 0
    else:
        area_confidence = 1 - area_confidence

    shape_confidence = rect_confidence(contour, height, width)

    return shape_confidence * area_confidence
