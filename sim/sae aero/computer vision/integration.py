import cv2
from calibrate import calibrate
from recalibrate import recalibrate
from threshold import threshold


def confidence():
    pass


def process_video(input_path: str):
    video_capture = cv2.VideoCapture(input_path)

    _, first_frame = video_capture.read()

    prev_frame = first_frame
    prev_center = (0, 0)

    threshold_range = calibrate(first_frame)

    while video_capture.isOpened():
        _, frame = video_capture.read()

        if recalibrate(frame, prev_frame):
            threshold_range = calibrate(frame)

        points, center = threshold(threshold_range, prev_center, frame)

        conf = confidence(points, center)
        print(f"{conf:.2f}, {center}")

        prev_frame = frame
        prev_center = center

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    video_capture.release()
    cv2.destroyAllWindows()


input_path = ""
process_video(input_path)
