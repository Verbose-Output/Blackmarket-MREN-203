import cv2
import numpy as np

cap = cv2.VideoCapture(1)  #0 standard computer camera

if not cap.isOpened():
    print("No stream :(")
    exit()

#for stable detection across frames
counter = 0
THRESHOLD = 5  #number of consecutive frames required to be able to detect color object

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    #reduce noise in color detection
    frame = cv2.GaussianBlur(frame, (5, 5), 0)

    #convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #lower and upper bound range, wider range would be easier to detect
    lower_green = np.array([50, 100, 100])
    upper_green = np.array([75, 255, 255])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    #clean mask
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 300:  # adjust if needed
            detected = True

            x, y, w, h = cv2.boundingRect(largest)

            # Draw box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Label object
            cv2.putText(frame, "DETECTED", (x, y-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # Stability check
    if detected:
        counter += 1
    else:
        counter = 0

    # Display status
    if counter >= THRESHOLD:
        cv2.putText(frame, "STATUS: GREEN DETECTED", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
    else:
        cv2.putText(frame, "STATUS: NOT DETECTED", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    # Show windows
    cv2.imshow("Mask", mask)
    cv2.imshow("Detection", frame)

    # Exit on ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()