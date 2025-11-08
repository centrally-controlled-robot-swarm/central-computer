import cv2
import time
import os

# Initialize camera
camera = cv2.VideoCapture(2)
if not camera.isOpened():
    print("Error: Camera not accessible.")
    exit(1)

# Ensure output directory exists
path = "aruco_data/"
os.makedirs(path, exist_ok=True)

# Countdown before capture starts
for i in range(5, 0, -1):
    print(f"Capturing begins in {i} seconds...", end="\r", flush=True)
    time.sleep(1)
print("\nStarting capture!\n")

# Capture settings
num_of_samples = 20
interval = 1.0  # seconds between saved frames
start_time = time.time()
captured = 0

while captured < num_of_samples:
    ret, img = camera.read()
    if not ret:
        print("Frame capture failed.")
        continue

    # Show live feed (no lag)
    cv2.imshow("Live Feed", img)

    # Check if it’s time to save a frame
    if time.time() - start_time >= interval:
        print(f"Capturing frame {captured+1} of {num_of_samples}")
        time.sleep(1)
        filename = os.path.join(path, f"{captured+1}.jpg")
        cv2.imwrite(filename, img)
        print(f"Saved frame {captured+1} of {num_of_samples}")
        captured += 1
        start_time = time.time()

    # Press 'q' to quit early
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("\nCapture interrupted by user.")
        break

print("\nCapture complete!")
camera.release()
cv2.destroyAllWindows()
