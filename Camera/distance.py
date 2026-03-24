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


KNOWN_DISTANCE = 70 # cm (distance from camera to object)
KNOWN_WIDTH = 9      # cm (real width of object)

lower_color = np.array([15, 80, 80])
upper_color = np.array([40, 255, 255])

data = []  # store (distance, pixel_width)

print("Press 's' to save a measurement, 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_color, upper_color)

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    w = None

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 300:
            x, y, w, h = cv2.boundingRect(largest)

            # Draw box
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)

            cv2.putText(frame, f"Width: {w}px", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF

    # Save measurement
    if key == ord('s') and w is not None:
        print(f"Saved: Distance={KNOWN_DISTANCE} cm, Width={w}px")
        data.append((KNOWN_DISTANCE, w))

    # Quit
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# ---- Compute focal length ----
if len(data) > 0:
    focal_lengths = []

    for D, w in data:
        f = (w * D) / KNOWN_WIDTH
        focal_lengths.append(f)

    avg_focal = sum(focal_lengths) / len(focal_lengths)

    print("\nCalibration Results:")
    print(f"Average Focal Length = {avg_focal:.2f}")

    # Save to CSV for Excel (optional)
    with open("calibration_data.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Distance_cm", "Pixel_Width"])
        writer.writerows(data)

    print("Data saved to calibration_data.csv")

else:
    print("No data collected.")