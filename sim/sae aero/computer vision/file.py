import cv2
import numpy as np
import scipy.stats as stats

class PayloadTracker:
    def __init__(self):
        self.prev_frame = None
        self.prev_center = None
        self.threshold_range = None
        self.hsv_range = ((0, 0, 180), (225, 225, 255))
        
    def fit_quadrilateral(self, contour):
        """Fits a quadrilateral to a given contour."""
        return cv2.boxPoints(cv2.minAreaRect(np.array(list(contour))))

    def calculate_confidence(self, contour, target_area, height, width):
        """Calculate confidence score based on shape and area matching."""
        if target_area < 0:
            return self._rect_confidence(contour, height, width)
            
        area = cv2.contourArea(contour)
        area_confidence = (1-area/target_area)**2
        area_confidence = 1 - area_confidence if area_confidence <= 1 else 0
        shape_confidence = self._rect_confidence(contour, height, width)
        
        return shape_confidence * area_confidence

    def _rect_confidence(self, contour, height, width):
        """Calculate rectangular confidence of contour."""
        mask1 = np.zeros((height, width), dtype=np.uint8)
        mask2 = np.zeros((height, width), dtype=np.uint8)
        
        box = np.int0(self.fit_quadrilateral(contour))
        
        cv2.drawContours(mask1, [contour], 0, 255, thickness=cv2.FILLED)
        cv2.drawContours(mask2, [box], 0, 255, thickness=cv2.FILLED)
        
        intersection = cv2.bitwise_xor(mask1, mask2)
        area = cv2.countNonZero(intersection)
        return 1 - area/(cv2.contourArea(contour)+cv2.contourArea(box))

    def detect_contour(self, frame, threshold_range=None, use_hsv=False):
        """Detect contours in frame using either HSV or threshold range."""
        # Apply Gaussian blur
        if len(frame.shape) == 3:
            frame = cv2.GaussianBlur(frame, (5, 5), 0)
        
        if use_hsv:
            if len(frame.shape) == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(frame, *self.hsv_range)
        else:
            # For single channel image during calibration
            if len(frame.shape) == 2:
                thresh = cv2.inRange(frame, threshold_range[0], threshold_range[1])
            else:
                frame_hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
                thresh = cv2.inRange(frame_hls, (0, threshold_range[0], 0), 
                                   (180, threshold_range[1], 255))

        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = []
        centers = []
        area_ratios = []

        for contour in contours:
            if cv2.contourArea(contour) > 30:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    rect = cv2.minAreaRect(contour)
                    (center_x, center_y), (width, height), angle = rect

                    if width > 0 and height > 0 and cv2.contourArea(contour) / (width * height) > 0.57:
                        valid_contours.append(contour)
                        centers.append((cx, cy))
                        area_ratios.append(cv2.contourArea(contour) / (width * height))

        if valid_contours:
            total = list(zip(area_ratios, valid_contours, centers))
            total.sort(key=lambda x: x[0], reverse=True)
            return total[0][1], centers  # Return best contour and all centers
        return None, []

    def threshold(self, threshold_range, prev_center, frame):
        """Apply threshold and find closest contour to previous center."""
        contours, centers = self.detect_contour(frame, threshold_range)
        
        if not centers or prev_center is None:
            return None, None
            
        # Find closest center to previous
        distances = [np.sqrt((c[0] - prev_center[0])**2 + (c[1] - prev_center[1])**2) 
                    for c in centers]
        closest_idx = np.argmin(distances)
        if distances[closest_idx] < 28:  # Distance threshold
            return [p[0] for p in contours], centers[closest_idx]
            
        return None, None

    def calibrate(self, frame):
        """Calibrate threshold range for optimal detection."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_s = hsv[:, :, 1].copy()
        color = 131
        curr_range = (0.0, 1.0)
        best_confidence = 0.0
        std = np.sqrt(np.var(frame_s))

        # Binary search for optimal range
        low, high = 0.0, 1.0
        while (high - low > 0.01):
            mid = (low + high) / 2
            z_score = stats.norm.ppf(mid)
            range_val = (color - z_score * std, color + z_score * std)
            points, _ = self.detect_contour(frame_s, range_val)

            if points is not None:
                curr_range = range_val
                low = mid
                best_confidence = (low + high) / 2
            else:
                high = mid

        # Neighborhood search refinement
        neighborhood_conf = np.linspace(best_confidence - 0.02, best_confidence, 20)
        best_range = curr_range
        for conf in neighborhood_conf:
            z_score = stats.norm.ppf(conf)
            range_val = (color - z_score * std, color + z_score * std)
            points, _ = self.detect_contour(frame_s, range_val)
            if points is not None:
                curr_conf = self.calculate_confidence(points, -1, *frame_s.shape)
                if curr_conf > best_confidence:
                    best_confidence = curr_conf
                    best_range = range_val
        
        return best_range

    def needs_recalibration(self, frame1, frame2):
        """Check if recalibration is needed based on KL divergence."""
        contour1, _ = self.detect_contour(frame1, use_hsv=True)
        contour2, _ = self.detect_contour(frame2, use_hsv=True)

        if contour1 is None or contour2 is None:
            return True

        margin = 20
        region1 = self._get_contour_region(frame1, contour1, margin)
        region2 = self._get_contour_region(frame2, contour2, margin)
        
        divergence = self._calculate_kl_divergence(region1, region2)
        return divergence > 5

    def _get_contour_region(self, img, contour, margin):
        """Extract region around contour with margin."""
        x, y, w, h = cv2.boundingRect(contour)
        x = max(0, x - margin)
        y = max(0, y - margin)
        w = min(img.shape[1] - x, w + 2 * margin)
        h = min(img.shape[0] - y, h + 2 * margin)
        return img[y:y + h, x:x + w]

    def _calculate_kl_divergence(self, img1, img2):
        """Calculate KL divergence between two image regions."""
        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)[:,:,0]
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)[:,:,0]

        hist1 = cv2.calcHist([img1], [0], None, [256], [0, 256])
        hist1 /= hist1.sum()
        hist2 = cv2.calcHist([img2], [0], None, [256], [0, 256])
        hist2 /= hist2.sum()

        return np.sum(np.where(hist1 != 0, hist1 * np.log(hist1 / (hist2 + 1e-10)), 0))

    def convert_to_world_coordinates(self, pixel, altitude):
        """Convert pixel coordinates to world coordinates."""
        K = np.array([[1, 0, 1],
                     [0, 1, 1],
                     [0, 0, 10]])
        u, v = pixel
        Z = altitude
        X_c = (u - K[0, 2]) * Z / K[0, 0]
        Y_c = (v - K[1, 2]) * Z / K[1, 1]
        
        R = np.array([[1, 0, 0],
                     [0, 0, -1],
                     [0, 1, 0]])
        
        coords = np.dot(R, np.array([X_c, Y_c, Z]))
        return coords

    def process_frame(self, frame):
        """Process a single frame and track the payload."""
        if self.prev_frame is None:
            self.threshold_range = self.calibrate(frame)
            self.prev_frame = frame
            _, centers = self.detect_contour(frame, self.threshold_range)
            if centers:
                self.prev_center = centers[0]
            return None, None, 0

        if self.needs_recalibration(frame, self.prev_frame):
            self.threshold_range = self.calibrate(frame)

        contour, center = self.threshold(self.threshold_range, self.prev_center, frame)
        
        if contour is not None:
            conf = self.calculate_confidence(contour, -1, *frame.shape[:2])
            world_coords = [self.convert_to_world_coordinates(p, 100) for p in contour]
        else:
            conf = 0
            world_coords = None

        self.prev_frame = frame
        self.prev_center = center
        
        return world_coords, center, conf

def main():
    tracker = PayloadTracker()
    cap = cv2.VideoCapture('a.MOV')  # Use 0 for webcam or provide video path
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
            
        coords, center, confidence = tracker.process_frame(frame)
        
        if coords is not None:
            print(f"World coordinates: {coords}")
            print(f"Confidence: {confidence:.2f}, Center: {center}")
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()