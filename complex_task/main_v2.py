import time
import detect
import cv2

def getPredictPosition(position, last_position, time_interval):
    velocity = (position.center_point - last_position.center_point) / time_interval
    time_start = time.time()
    new_position = position
    new_position.center_point = position.center_point + velocity * time_interval
    time_end = time.time()
    return new_position, time_end - time_start

def main():
    print("Hello World!")
    capture = cv2.VideoCapture(0)
    if capture.isOpened():
        print("Capture is opened!")
        get_image_time = time.time()
        open, raw_image = capture.read()
    else:
        print("Capture is not opened!")
        open = False

    threshold = 0.01
    time_interval = 0
    last_position, detect_time = detect.detect(raw_image)

    while open:
        ret, raw_image = capture.read()
        position, detect_time = detect.detect(raw_image)
        print("Time cost of detect: ", detect_time)
        time_interval += detect_time
        if time_interval >= threshold:
            predict_position = getPredictPosition(position, last_position, time_interval)
            last_position = position
            time_interval = 0
        cv2.imshow("Camera", frame)

if __name__ == "__main__":
    main()