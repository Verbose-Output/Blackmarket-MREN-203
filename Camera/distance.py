import cv2
import numpy as np
import csv

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Force resolution 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("No stream :(")
    exit()

# ---- CHANGE THIS EACH TIME YOU MOVE THE OBJECT ----
KNOWN_DISTANCE = 30  # cm (measure from camera lens to object)

# HSV bounds for dull yellow
lower_color = np.array([160, 60, 60])
upper_color = np.array([180, 255, 255])

data = []  # store (distance, pixel_size)

print("Press 's' to save a measurement, 'q' to quit")

while True:
    ret, frame = cap.read() 
    if not ret:
        break

    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_color, upper_color)

    kernel_bridge = np.ones((25,25), np.uint8)
    # MORPH_CLOSE fills the black "gap" at the neck
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_bridge)

    # MORPH_OPEN removes tiny random noise dots
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    pixel_size = None

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 300:  # filter noise
            x, y, w, h = cv2.boundingRect(largest)

            # Use the largest dimension (robust to rotation)
            pixel_size = max(w, h)

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)

            # Display pixel size
            cv2.putText(frame, f"Pixel Size: {pixel_size}px", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF

    # Save measurement
    if key == ord('s') and pixel_size is not None:
        print(f"Saved: Distance={KNOWN_DISTANCE} cm, Pixel Size={pixel_size}px")
        data.append((KNOWN_DISTANCE, pixel_size))

    # Quit
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
