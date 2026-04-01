import cv2
import numpy as np
from collections import deque
import math

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("No stream :(")
    exit()

# Configuration
counter = 0
THRESHOLD = 5  
CENTER_TOLERANCE = 25
CAMERA_HEIGHT = 12.0 # cm
distance_history = deque(maxlen=5)

# --- HSV BOUNDS FOR PINK ---
lower_pink = np.array([160, 60, 60])
upper_pink = np.array([180, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Reduce noise
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Create Pink Mask
    mask = cv2.inRange(hsv, lower_pink, upper_pink)

    # --- MORPHOLOGY 
    # Closing fills gaps inside the pink object to keep the box solid
    kernel_bridge = np.ones((25, 25), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_bridge)
    # Opening removes small random background noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    direction = "NO TARGET"

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 300:  
            detected = True
            x, y, w, h = cv2.boundingRect(largest)

            # Draw UI
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "PINK TARGET", (x, y-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # ----- CENTERING LOGIC -----
            object_center_x = x + w // 2
            frame_width = frame.shape[1]
            frame_center_x = frame_width // 2

            # Center Line
            cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame.shape[0]), (255, 0, 0), 2)

            if object_center_x < frame_center_x - CENTER_TOLERANCE:
                direction = "GO LEFT"
            elif object_center_x > frame_center_x + CENTER_TOLERANCE:
                direction = "GO RIGHT"
            else:
                direction = "CENTERED"

            cv2.putText(frame, f"Action: {direction}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            # ----- DISTANCE LOGIC (Power Regression) -----
            if w > 0:
                inv_w = 1.0 / w
                # Equation: y = 5289.2 * x^1.0861
                hypotenuse = 5289.2 * (inv_w ** 1.0861)
                distance_history.append(hypotenuse)

                if len(distance_history) > 0:
                    avg_hyp = sum(distance_history) / len(distance_history)

                    # Pythagorean theorem for Ground Distance
                    if avg_hyp > CAMERA_HEIGHT:
                        ground_dist = math.sqrt(avg_hyp**2 - CAMERA_HEIGHT**2)
                    else:
                        ground_dist = 0.0

                    cv2.putText(frame, f"Ground Dist: {ground_dist:.2f} cm", (x, y-55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Status Display
    counter = counter + 1 if detected else 0
    status_text = "STATUS: DETECTED" if counter >= THRESHOLD else "STATUS: SEARCHING"
    status_color = (0, 255, 0) if counter >= THRESHOLD else (0, 0, 255)
    cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

    cv2.imshow("Pink Mask", mask)
    cv2.imshow("Duck Tracker", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()