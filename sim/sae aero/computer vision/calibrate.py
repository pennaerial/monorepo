import numpy as np
from scipy.stats import norm
import scipy.stats as stats

def calibrate(frame, color):
    # frame: array of pixels
    # color: a V value from HSV (ground truth for payload)

    low = 0.0
    high = 1.0
    mid = (low + high) / 2
    curr_range = (0.0, 1.0)
    best_confidence = 0.0
    best_points = []
    while (high - low > 0.01):
        mid = (low + high) / 2

        # Compute Z-score and range using probability
        prob = mid
        z_score = stats.norm.ppf(prob)
        variance = np.var(frame)
        std = np.sqrt(variance)

        # Get threshold range based on color and Z-score
        range = (color - z_score * std, color + z_score * std)

        # Update the current range
        
        points = output_contour(range)
        if len(points) > 0: 
            # If the contour set is non-empty, 
            # continue searching with a higher confidence, but save previous
            # only update if confidence is higher
            if mid > best_confidence:
                best_points = points
                best_confidence = mid
                curr_range = range
    
            low = mid
        else:
            # if countour set is empty
            # continue search with lower cofidence
            high = mid
    


    # neighborhood search
    # best_points : set of points
    # best_ confidence : highest confidence that returns something
    # best_range : corresponds to confidence

    # find  20 points within 2 percent buffer
    neighborhood_low = best_confidence - 0.02 * best_confidence
    neighborhood_high = best_confidence + 0.02 * best_confidence
    neighborhood_conf = np.linspace(neighborhood_low, neighborhood_high, 20)  # 20 points within the range


    #init variables to track the best neighborhood confidence and range
    best_neighborhood_confidence = best_confidence
    best_neighborhood_points = best_points
    best_neighborhood_range = curr_range

    for conf in neighborhood_conf:
        z_score = stats.norm.ppf(conf)
        range = (color - z_score * std, color + z_score * std)
        points = output_contour(range)

        # Evaluate confidence based on `points` and update if better than current best
        if len(points) > 0 and confidence(points) > confidence(best_neighborhood_points):
            best_neighborhood_confidence = conf
            best_neighborhood_points = points
            best_neighborhood_range = range

    # output best range. potentially could output best conf, if necessary
    return best_neighborhood_range



def output_contour(threshold):
    # Kevin + Arjun Function
    # returns a list of points given threshold, 
    return []

def confidence(points):
    # dimitris function
    # given points: list of points
    # outputs a confidence score
    return 1



# Example call
# print(stats.norm.ppf(0.95))  # Correct usage of stats.norm.ppf
