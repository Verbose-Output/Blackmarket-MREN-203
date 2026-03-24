import cv2
import numpy as np
from collections import deque

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Use your external camera index

if not cap.isOpened():
    print("No stream :(")
    exit()

# for stable detection across frames
counter = 0
THRESHOLD = 5  # number of consecutive frames required to detect color object

# smoothing: keep last N distance values
distance_history = deque(maxlen=5)

# HSV bounds for dull yellow
lower_yellow = np.array([15, 80, 80])
upper_yellow = np.array([40, 255, 255])

# Polynomial coefficients from your Excel calibration
# y = -143310*x^2 + 7754.3*x - 1.8324
# x = 1 / pixel_width
poly_a = -143310
poly_b = 7754.3
poly_c = -1.8324

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # reduce noise
    frame = cv2.GaussianBlur(frame, (5, 5), 0)

    # convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # create mask
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # clean mask
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    distance = None

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 200:  # filter small noise
            detected = True
            x, y, w, h = cv2.boundingRect(largest)

            # draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "DETECTED", (x, y-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # calculate distance using polynomial
            if w > 0:
                inv_w = 1 / w
                distance = poly_a * (inv_w**2) + poly_b * inv_w + poly_c
                distance_history.append(distance)

                # smooth distance
                smooth_distance = sum(distance_history) / len(distance_history)

                cv2.putText(frame, f"Distance: {smooth_distance:.2f} cm", (x, y-35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    # stability check
    if detected:
        counter += 1
    else:
        counter = 0
        distance_history.clear()  # reset smoothing when object lost

    # display status
    if counter >= THRESHOLD:
        cv2.putText(frame, "STATUS: YELLOW DETECTED", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
    else:
        cv2.putText(frame, "STATUS: NOT DETECTED", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    # show windows
    cv2.imshow("Mask", mask)
    cv2.imshow("Detection", frame)

    # exit on ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()