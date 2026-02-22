import numpy as np
import cv2
import scipy.stats as stats
from recalibrate import detect_contour
from confidence import confidence


def calibrate(frame):
    """
    Returns color range with best confidence interval
    Args:
        frame (List[int] * List[int] * List[int]): 3 * 2-D list of pixels
    Returns:
        range (Tuple[Int, Int]): The color optimal color rnage
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = hsv[:, :, 1].copy()
    # Initialize variables
    color = 131
    curr_range = (0.0, 1.0)
    best_confidence = 0.0
    variance = np.var(frame)
    std = np.sqrt(variance)

    # Binary search
    low = 0.0
    high = 1.0
    mid = (low + high) / 2

    while high - low > 0.01:
        mid = (low + high) / 2
        points, range = find_points_range(mid, std, color, frame)

        if len(points) > 0:
            curr_range = range
            low = mid
            best_confidence = (low + high) / 2
        else:
            high = mid

    # Perform a neighborhood search on 20 points within a 2% buffer
    neighborhood_low = best_confidence - 0.02
    neighborhood_high = best_confidence
    neighborhood_conf = np.linspace(neighborhood_low, neighborhood_high, 20)

    best_neighborhood_range = curr_range
    for conf in neighborhood_conf:
        points, range = find_points_range(conf, std, color, frame)
        curr_conf = confidence(points, -1, *frame.shape)
        if len(points) > 0 and curr_conf > best_confidence:
            best_confidence = curr_conf
            best_neighborhood_range = range

    return best_neighborhood_range


def find_points_range(prob, std, color, frame):
    """
    Finds the points and color value range corresponding to some probability, std, and color

    Args:
        prob (Float): probaility ranging from 0.0 - 1.0
        std (Float): standar d deviation value
        color (Float): Ground truth or mean of distribution
    Returns:
        points (List[Int]): list of points in the image which correspond to range
        range (Tuple[Int, Int]): color range corresponding to prob
    """
    z_score = stats.norm.ppf(prob)
    range = (color - z_score * std, color + z_score * std)
    points = detect_contour(frame, range)
    return points, range
