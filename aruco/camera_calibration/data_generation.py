import cv2
import os
import time

# Initialize camera
camera = cv2.VideoCapture(2)
if not camera.isOpened():
    print("Error: Camera not accessible.")
    exit(1)

# Ensure output directory exists
path = "aruco_data/"
os.makedirs(path, exist_ok=True)

print("Press 'c' to capture an image.")
print("Press 'q' to quit.\n")

capture_count = 0

while True:
    ret, img = camera.read()
    if not ret:
        print("Frame capture failed.")
        continue

    # Show live camera feed
    cv2.imshow("Live Feed", img)

    # Check keyboard input (non-blocking)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):  
        filename = os.path.join(path, f"{capture_count}.jpg")
        cv2.imwrite(filename, img)
        print(f"Captured image {capture_count}: {filename}")
        capture_count += 1

    elif key == ord('q'):
        print("Exiting...")
        break

camera.release()
cv2.destroyAllWindows()
