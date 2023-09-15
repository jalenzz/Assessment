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

def draw_predict(vertices):
    pass # TODO: draw line

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

    last_position, detect_time = detect.detect(raw_image)
    while open:
        ret, raw_image = capture.read()
        position, detect_time = detect.detect(raw_image)
        print("Time cost of detect: ", detect_time)
        predict_position, predict_time = getPredictPosition(position, last_position, detect_time)
        last_position = position
        predict = draw_predict(predict_position.vertices)
        cv2.imshow("pre", predict)


if __name__ == "__main__":
    main()