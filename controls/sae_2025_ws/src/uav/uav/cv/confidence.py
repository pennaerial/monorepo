import cv2
import numpy as np


def _contour_array(contour) -> np.ndarray:
    return np.asarray(contour, dtype=np.int32)


def fit_quadrilateral(contour) -> np.ndarray:
    return cv2.boxPoints(cv2.minAreaRect(_contour_array(contour)))


def rect_confidence(contour, height, width) -> float:
    contour_array = _contour_array(contour)

    # Create empty masks
    mask1 = np.zeros((height, width), dtype=np.uint8)
    mask2 = np.zeros((height, width), dtype=np.uint8)

    # Convert the box into a contour (it needs to be a list of points)
    box = fit_quadrilateral(contour_array).astype(np.int32)

    # Draw the contour of the original contour on mask1
    cv2.drawContours(mask1, [contour_array], 0, 255, thickness=cv2.FILLED)

    # Draw the quadrilateral bounding box on mask2
    cv2.drawContours(mask2, [box], 0, 255, thickness=cv2.FILLED)

    # Compute the intersection using bitwise AND
    intersection = cv2.bitwise_xor(mask1, mask2)

    # Calculate the area of the intersection
    area = cv2.countNonZero(intersection)
    confidence = 1 - area / (cv2.contourArea(contour_array) + cv2.contourArea(box))
    return confidence


def confidence(contour, target_area, height, width) -> float:
    contour_array = _contour_array(contour)

    # use target_area negative as a proxy for not suppying a target area
    if target_area < 0:
        return rect_confidence(contour_array, height, width)
    area = cv2.contourArea(contour_array)
    area_confidence = (1 - area / target_area) ** 2
    if area_confidence > 1:
        area_confidence = 0
    else:
        area_confidence = 1 - area_confidence

    shape_confidence = rect_confidence(contour_array, height, width)

    return shape_confidence * area_confidence
