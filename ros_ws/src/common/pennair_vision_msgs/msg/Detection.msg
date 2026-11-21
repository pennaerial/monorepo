# One detected object in a camera image
int32 class_id    # index into the detector's class_labels
float32 score     # confidence in [0, 1]
# Axis-aligned bounding box in pixels of the original camera image (not the model input).
# (0, 0) is the top-left pixel; x grows right, y grows down.
float32 x_min
float32 y_min
float32 x_max
float32 y_max
