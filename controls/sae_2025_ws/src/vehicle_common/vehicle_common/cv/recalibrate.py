import cv2
import numpy as np


def detect_contour(frame, range):
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    # Threshold to isolate potential contours
    thresh = cv2.inRange(frame, *range)

    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = []
    centers = []
    area_ratios = []

    for contour in contours:
        if (
            cv2.contourArea(contour) > 30
        ):  # Only consider contours with significant area
            M = cv2.moments(contour)
            if M["m00"] != 0:  # Prevent division by zero
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                rect = cv2.minAreaRect(contour)
                (center_x, center_y), (width, height), angle = rect

                if (
                    width > 0
                    and height > 0
                    and cv2.contourArea(contour) / (width * height) > 0.57
                ):
                    valid_contours.append(contour)
                    centers.append((cx, cy))
                    area_ratios.append(cv2.contourArea(contour) / (width * height))

    if valid_contours:
        total = list(zip(area_ratios, valid_contours, centers))
        total.sort(
            key=lambda x: x[0], reverse=True
        )  # Sort by area ratio, highest first
        _, best_contour, _ = total[0]  # Choose the best contour based on area ratio
        return best_contour
    else:
        return set()


def contourNeighborhood(img, contour, margin):
    if contour is None:
        raise ValueError("No valid contour found.")

    x, y, w, h = cv2.boundingRect(contour)
    x = max(0, x - margin)
    y = max(0, y - margin)
    w = min(img.shape[1] - x, w + 2 * margin)
    h = min(img.shape[0] - y, h + 2 * margin)

    cropped_image = img[y : y + h, x : x + w]
    return cropped_image


def kl_divergence(img1, img2):
    # Convert images to grayscale if needed
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV) if len(img1.shape) == 3 else img1
    img1 = img1[:, :, 0]
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV) if len(img2.shape) == 3 else img2
    img2 = img2[:, :, 0]

    # Normalize pixel values to create probability distributions
    hist1 = cv2.calcHist([img1], [0], None, [256], [0, 256])
    hist1 /= hist1.sum()
    hist2 = cv2.calcHist([img2], [0], None, [256], [0, 256])
    hist2 /= hist2.sum()

    # Calculate KL Divergence
    kl = np.sum(np.where(hist1 != 0, hist1 * np.log(hist1 / (hist2 + 1e-10)), 0))

    return kl.astype(np.float32)


def check(frame1, frame2, range):
    contour1 = detect_contour(frame1, range)
    contour2 = detect_contour(frame2, range)

    if contour1 is None or contour2 is None:
        raise ValueError("No valid contours detected in one or both images.")

    margin = 20
    region1 = contourNeighborhood(frame1, contour1, margin)
    region2 = contourNeighborhood(frame2, contour2, margin)

    return kl_divergence(region1, region2)


def recalibrate(frame1, frame2, range):
    try:
        print("THE DIVERGENCE IS " + (str)(check(frame1, frame2, range)))
    except ValueError as e:
        print(f"Error: {e}")
    return (check(frame1, frame2, range)) > 5


if __name__ == "__main__":
    # Example usage
    range = ((0, 0, 180), (225, 225, 255))

    img1 = cv2.imread("/Users/rushilpatel/Desktop/Recalibrate1.png")
    img2 = cv2.imread("/Users/rushilpatel/Desktop/Recalibrate2.png")
    img3 = cv2.imread("/Users/rushilpatel/Desktop/Recalibrate3.png")

    print(recalibrate(img1, img2, range))
    print(recalibrate(img2, img3, range))
    print(recalibrate(img1, img3, range))
