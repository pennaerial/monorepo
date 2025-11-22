import cv2
import numpy as np

class ColorSpace:
    def __init__(self, mode="HSV"):
        self.mode = mode

    def apply(self, image):
        if self.mode == "HSV":
            converted = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        elif self.mode == "LAB":
            converted = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        elif self.mode == "GRAY":
            converted = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            converted = image  # default: no change
        # cv2.imshow(f"ColorSpace-{self.mode}", converted)
        return converted

class PreProcessing:
    def __init__(self, method="gaussian", kernel_size=5):
        self.method = method
        self.k_size = kernel_size

    def apply(self, image):
        # image = cv2.normalize(image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)

        if self.method == "gaussian":
            processed = cv2.GaussianBlur(image, (self.k_size, self.k_size), 0)
        elif self.method == "bilateral":
            processed = cv2.bilateralFilter(image, 9, 75, 75)
        elif self.method == "median":
            processed = cv2.medianBlur(image, self.k_size)
        
        # Morphology Opening (remove small noise)
        elif self.method == "opening":
            # The larger the kernel, the stronger the smoothing effect, 
            # but it may swallow small targets
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size, self.k_size))
            processed = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        # Morphology Closing (fill small holes in objects)
        elif self.method == "closing":
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size, self.k_size))
            processed = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        # Morphology Opening + Closing combined
        elif self.method == "open_close":
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size, self.k_size))
            temp = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
            processed = cv2.morphologyEx(temp, cv2.MORPH_CLOSE, kernel)

        else:
            processed = image  # default: no preprocessing
        # cv2.imshow(f"PreProcessing-{self.method}", processed)
        return processed

def filter_red_orange(hsv_img):
    """Keep only red and orange regions in HSV image."""
    # Red (two ranges)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Orange
    lower_orange = np.array([10, 100, 100])
    upper_orange = np.array([25, 255, 255])

    # Create masks
    mask_red1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
    mask_orange = cv2.inRange(hsv_img, lower_orange, upper_orange)

    # Combine red and orange
    mask = cv2.bitwise_or(mask_red1, mask_red2)
    mask = cv2.bitwise_or(mask, mask_orange)

    return mask

class Segmentation:
    def __init__(self, method="threshold"):
        self.method = method

    def apply(self, image):
        if self.method == "threshold":
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        elif self.method == "canny":
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            mask = cv2.Canny(gray, 100, 120, L2gradient=False)
        elif self.method == "kmeans":
            Z = image.reshape((-1, 3))
            Z = np.float32(Z)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            _, labels, centers = cv2.kmeans(Z, 2, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
            mask = labels.reshape((image.shape[0], image.shape[1])).astype(np.uint8) * 255
        else:
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
        # cv2.imshow(f"Segmentation-{self.method}", mask)
        return mask
    
class PostProcessing:
    def __init__(self, kernel_open=7, kernel_close=5, area_ratio=0.01):
        # different kernels for open and close
        self.kernel_open = np.ones((kernel_open, kernel_open), np.uint8)
        self.kernel_close = np.ones((kernel_close, kernel_close), np.uint8)
        self.area_ratio = area_ratio

    def apply(self, mask):
        h, w = mask.shape[:2]
        mask_clean = mask.copy()

        # Open calculation to remove small noise
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, self.kernel_open)

        # Contour Filtering
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask_filtered = np.zeros_like(mask_clean)
        valid_cnts = []
        for cnt in contours:
            if cv2.contourArea(cnt) >= self.area_ratio*h*w:
                cv2.drawContours(mask_filtered, [cnt], -1, 255, -1)
                valid_cnts.append(cnt)

        # Closing operation, smoothing the edge of the target and filling small holes
        mask_final = cv2.morphologyEx(mask_filtered, cv2.MORPH_CLOSE, self.kernel_close)

        # Find the largest contour and fit an ellipse.
        if len(valid_cnts) == 0:
            return mask_final, None, None
        max_cnt = max(valid_cnts, key=cv2.contourArea)
        if len(max_cnt) < 5:
            return mask_final, None, None

        # Fit ellipse to the largest contour
        ellipse = cv2.fitEllipse(max_cnt)
        (ex, ey), (a, b), angle = ellipse

        # Generate ellipse contour points (needed for PnP)
        ellipse_contour = []
        for t in np.linspace(0, 2*np.pi, 200):
            x = ex + (a/2)*np.cos(t)*np.cos(np.deg2rad(angle)) - (b/2)*np.sin(t)*np.sin(np.deg2rad(angle))
            y = ey + (a/2)*np.cos(t)*np.sin(np.deg2rad(angle)) + (b/2)*np.sin(t)*np.cos(np.deg2rad(angle))
            ellipse_contour.append((x, y))

        return mask_final, ellipse_contour, ellipse

class Visualization:
    def __init__(self):
        pass

    def apply(self, image, ellipse_contour, ellipse_params):
        result = image.copy()
        if ellipse_params is None:
            return result

        (ex, ey), (a, b), angle = ellipse_params

        cv2.ellipse(result, ellipse_params, (0, 255, 0), 8) # Draw ellipse
        cv2.circle(result, (int(ex), int(ey)), 8, (255, 255, 255), -1) # Draw ellipse center
        if ellipse_contour is not None:                     # Draw contour points       
            for (x, y) in ellipse_contour:
                cv2.circle(result, (int(x), int(y)), 2, (0, 255, 255), -1)

        return result


def detect_contour(image):
    """
    Detect hoop and return its largest contour.
    Returns:
        ellipse_contour: np.ndarray (N, 1, 2) the largest contour points if found
        ellipse_params: tuple ((center), (major, minor), angle) if found
    """
    # Initialize pipeline
    pre = PreProcessing(method="open_close", kernel_size=5)
    seg = Segmentation(method="threshold")
    post = PostProcessing(kernel_open=7, kernel_close=7, area_ratio=0.005)
    vis = Visualization()

    # hoop detection pipeline
    hsv = ColorSpace(mode="HSV").apply(image)
    hsv_pre = pre.apply(hsv)
    filtered_hsv = filter_red_orange(hsv_pre) # Filter only red & orange
    mask_hsv = seg.apply(filtered_hsv)
    mask_final, ellipse_contour, ellipse_params = post.apply(mask_hsv)

    # visualization (commented out for production - uncomment for debugging)
    # out = vis.apply(image, ellipse_contour, ellipse_params)
    # scale = 1  # rescale display
    # disp_frame = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    # cv2.imshow("detect result", disp_frame)
    # cv2.waitKey(1)  # Use waitKey(1) instead of waitKey(0) to avoid blocking

    return ellipse_contour, ellipse_params


def estimate_pose_from_contour(
    ellipse_contour: list,
    ellipse_params: tuple,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    hoop_radius: float = 0.63825,
    num_samples: int = 40,
    reprojection_error: float = 3.0,
    max_iterations: int = 100,
    confidence: float = 0.99
):
    """
    Args:
        ellipse_contour: Contour points of the hoop in image space (numpy array of shape (N, 1, 2))
        ellipse_params: Fitted ellipse parameters ((center), (major, minor), angle)
        camera_matrix: Camera's intrinsic matrix (3x3 numpy array)
        dist_coeffs: Camera's distortion coefficients (numpy array)
        hoop_radius: Radius of the hoop in meters (default: 0.63825)
        num_samples: Number of points to sample from fitted ellipse (default: 8)
        reprojection_error: Maximum reprojection error threshold for RANSAC in pixels (default: 3.0)
        max_iterations: Maximum iterations for RANSAC (default: 100)
        confidence: Confidence level for RANSAC 0-1 (default: 0.99)
        min_contour_area: Minimum contour area required (default: 100.0)
    
    Returns:
        success: Boolean indicating if pose estimation was successful
        rvec: Rotation vector from camera to hoop (3x1 numpy array) or None
        tvec: Translation vector from camera to hoop (3x1 numpy array) or None
        c_obj: Camera position in hoop coordinates (3x1 numpy array) or None
        ellipse: Fitted ellipse parameters ((center), (major, minor), angle) or None
    """

    # Validate contour
    if ellipse_contour is None or len(ellipse_contour) < num_samples:
        return False, None, None, None

    # Convert list to numpy
    ellipse_contour = np.array(ellipse_contour, dtype=np.float32)

    # Uniformly sample points from contour
    idx = np.linspace(0, len(ellipse_contour)-1, num_samples).astype(int)
    image_points = ellipse_contour[idx]   # [N,2]

    # Build 3D circular model
    thetas = np.linspace(0, 2*np.pi, num_samples, endpoint=False).astype(np.float32)
    object_points = np.stack([
        hoop_radius * np.cos(thetas),
        hoop_radius * np.sin(thetas),
        np.zeros_like(thetas)
    ], axis=1)

    # Solve PnP RANSAC
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        objectPoints=object_points,
        imagePoints=image_points,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        reprojectionError=reprojection_error,
        iterationsCount=max_iterations,
        confidence=confidence,
        flags=cv2.SOLVEPNP_EPNP  # Efficient PnP solver
    )

    if not success:
        return False, None, None, None

    # Compute camera position in hoop frame
    R, _ = cv2.Rodrigues(rvec)
    c_obj = -R.T @ tvec

    return True, rvec, tvec, c_obj

def draw_reprojected_circle(image, rvec, tvec, camera_matrix, dist_coeffs, radius=0.63):
    img = image.copy()

    # Sample many 3D points on the hoop
    thetas = np.linspace(0, 2*np.pi, 200)
    circle_3d = np.vstack([
        radius * np.cos(thetas),
        radius * np.sin(thetas),
        np.zeros_like(thetas)
    ]).T.astype(np.float32)

    # Project them into the image
    proj, _ = cv2.projectPoints(circle_3d, rvec, tvec, camera_matrix, dist_coeffs)
    proj = proj.reshape(-1, 2).astype(int)
    for p in proj:  # Draw
        cv2.circle(img, tuple(p), 1, (0, 255, 0), 8)

    return img


if __name__ == "__main__":
    # Fake camera data used for testing
    camera_matrix = np.array([
        [1200.0,    0.0, 960.0],
        [   0.0, 1200.0, 540.0],
        [   0.0,    0.0,   1.0]
    ], dtype=np.float32)
    dist_coeffs = np.zeros(5, dtype=np.float32)

    image = cv2.imread("image_files/image3.png")
    ellipse_contour, ellipse_params = detect_contour(image)
    success, rvec, tvec, c_obj = estimate_pose_from_contour(ellipse_contour, ellipse_params, camera_matrix, dist_coeffs)
    print(rvec)
    print(tvec)

    img_reproj = draw_reprojected_circle(image, rvec, tvec, camera_matrix, dist_coeffs)
    cv2.imshow("Reprojected circle", img_reproj)
    cv2.waitKey(0)
    cv2.destroyAllWindows()