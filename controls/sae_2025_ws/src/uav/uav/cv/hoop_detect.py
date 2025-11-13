import cv2
import numpy as np

# ========== Color Space ==========
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


# ========== Pre Processing ==========
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

# ========== Segmentation ==========
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
    

# ========== Post Processing ==========
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
        for cnt in contours:
            if cv2.contourArea(cnt) >= self.area_ratio*h*w:
                cv2.drawContours(mask_filtered, [cnt], -1, 255, -1)

        # Closing operation, smoothing the edge of the target and filling small holes
        mask_final = cv2.morphologyEx(mask_filtered, cv2.MORPH_CLOSE, self.kernel_close)

        return mask_final


# =============== Visualization ===============
class Visualization:
    def __init__(self):
        # Camera intrinsics
        self.fx = 2564.3186869
        self.fy = 2569.70273111
        self.cx = 0
        self.cy = 0
        self.radius_in = 10.0  # known radius in inches

    def apply(self, image, mask):
        h, w = image.shape[:2]
        result = image.copy()

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            cv2.drawContours(result, [cnt], -1, (0, 128, 255), 2)
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx_img = int(M["m10"] / M["m00"])
                cy_img = int(M["m01"] / M["m00"])
                cv2.circle(result, (cx_img, cy_img), 10, (255, 255, 255), -1)

                # Estimate object size in pixels
                (x, y), r_px = cv2.minEnclosingCircle(cnt)
                if r_px > 1:  # avoid division by zero
                    # Depth estimation
                    Z = (self.fx * self.radius_in) / r_px
                    X = (cx_img - self.cx) * Z / self.fx
                    Y = (cy_img - self.cy) * Z / self.fy

                    # --- Fit ellipse for rotation estimation ---
                    if len(cnt) >= 5:
                        ellipse = cv2.fitEllipse(cnt)
                        (ex, ey), (a, b), angle = ellipse
                        cv2.ellipse(result, ellipse, (0, 255, 0), 8)

                        # Compute 4 boundary points on ellipse (up, right, down, left)
                        pts = []
                        for theta_deg in [0, 90, 180, 270]:
                            t = np.deg2rad(theta_deg)
                            x_ell = ex + (a/2) * np.cos(t) * np.cos(np.deg2rad(angle)) - (b/2) * np.sin(t) * np.sin(np.deg2rad(angle))
                            y_ell = ey + (a/2) * np.cos(t) * np.sin(np.deg2rad(angle)) + (b/2) * np.sin(t) * np.cos(np.deg2rad(angle))
                            pts.append((int(x_ell), int(y_ell)))
                            cv2.circle(result, (int(x_ell), int(y_ell)), 10, (0, 255, 255), -1)

                        # Estimate Z for each point (using local radius ~ distance to ellipse boundary)
                        Z_vals = []
                        for (px, py) in pts:
                            r_local = np.sqrt((px - cx_img)**2 + (py - cy_img)**2)
                            if r_local > 1:
                                Z_i = (self.fx * self.radius_in) / r_local
                            else:
                                Z_i = Z
                            Z_vals.append(Z_i)

                        # Define Z order: [right, top, left, bottom]
                        Z_right, Z_top, Z_left, Z_bottom = Z_vals[0], Z_vals[1], Z_vals[2], Z_vals[3]

                        # Compute rotation (pitch, yaw)
                        # pitch = np.degrees(np.arctan((Z_top - Z_bottom) / (2 * self.radius_in)))  # tilt around x-axis
                        yaw = np.degrees(np.arctan((Z_left - Z_right) / (2 * self.radius_in)))   # tilt around y-axis

                        # Ensure a >= b
                        if b > a:
                            a, b = b, a
                        # Clamp ratio to [-1, 1] to avoid invalid arccos input
                        ratio = np.clip(b / a, -1.0, 1.0)

                        # Estimate tilt based on ellipse aspect ratio
                        tilt_rad = np.arccos(ratio)
                        pitch = np.degrees(tilt_rad)  # assume tilt around x-axis

                        # Use ellipse rotation angle as yaw approximation
                        # yaw = angle  # adjust sign depending on coordinate convention

                        # cv2.putText(result, f"pitch: {pitch:.1f} deg", (cx_img - 180, cy_img + 160),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
                        # cv2.putText(result, f"yaw: {yaw:.1f} deg", (cx_img - 180, cy_img + 200),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)


                    # Display 3D coordinates
                    # coord_text = f"coords: [{X:.1f}, {Y:.1f}, {Z:.1f}]"
                    # cv2.putText(result, coord_text,
                    #             (cx_img - 200, cy_img + 140),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    #             (255, 255, 255), 2)
                    coord_text = f"coords: [{X:.1f}, {Y:.1f}]"
                    # cv2.putText(result, coord_text,
                    #             (cx_img - 200, cy_img + 100),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    #             (255, 255, 255), 2)

        return result
    

def find_hoop(image):
    """
    Detect hoop and return its largest contour (Nx2 array).
    Returns:
        np.ndarray: contour points (N, 2) if found
        None: if nothing detected
    """
    # Initialize pipeline
    pre = PreProcessing(method="open_close", kernel_size=5)
    seg = Segmentation(method="threshold")
    post = PostProcessing(kernel_open=7, kernel_close=7, area_ratio=0.005)

    hsv = ColorSpace(mode="HSV").apply(image)
    hsv_pre = pre.apply(hsv)
    filtered = filter_red_orange(hsv_pre)
    mask = seg.apply(filtered)
    mask = post.apply(mask)

    # Find largest contour
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)

    # Convert contour format from (N,1,2) → (N,2)
    # largest = largest.reshape(-1, 2).astype(float)

    return largest


# ========== Image Pipeline ==========
if __name__ == "__main__":
    # Load image
    image = cv2.imread("resources/Test5.png")

    # Initialize pipeline modules
    pre = PreProcessing(method="open_close", kernel_size=5)
    seg = Segmentation(method="threshold")
    post = PostProcessing(kernel_open=7, kernel_close=7, area_ratio=0.005)
    vis = Visualization()

    # Color space conversion
    hsv = ColorSpace(mode="HSV").apply(image)
    # Pre prcessing
    pre_hsv = pre.apply(hsv)
    # Filter only red & orange
    filtered_hsv = filter_red_orange(pre_hsv)

    # Segmentation
    mask_hsv = seg.apply(filtered_hsv)

    # Post processing and visualization
    mask_post = post.apply(mask_hsv)
    final_frame = vis.apply(image, mask_post)

    scale = 0.5  # rescale display
    disp_frame = cv2.resize(final_frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cv2.imshow("Pipeline-Result", disp_frame)
    cv2.imwrite("results/test_result5.png", final_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()