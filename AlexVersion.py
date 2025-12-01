import cv2
import pyautogui
import numpy as np
from aruco.marker_detection import ArucoMarkers

def main():
    # Initialize ArUco detector
    marker_detector = ArucoMarkers()

    cap = cv2.VideoCapture(0)  # adjust camera index if needed
    if not cap.isOpened():
        print("Cannot open camera")
        return

    # Simple Robot object to store marker info
    class Robot:
        x = 0
        y = 0
        heading = 0

    robot = Robot()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Detect ArUco marker center and heading
        marker_detector.detect_markers([robot], frame)

        # Get mouse position
        mouse_x, mouse_y = pyautogui.position()

        # Draw the marker center
        cv2.circle(frame, (robot.x, robot.y), 8, (0, 0, 255), -1)  # red
        cv2.putText(frame, "Marker", (robot.x + 10, robot.y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

        # Draw the heading vector
        length = 50  # pixels
        rad = np.radians(robot.heading)
        end_x = int(robot.x + length * np.cos(rad))
        end_y = int(robot.y + length * np.sin(rad))
        cv2.arrowedLine(frame, (robot.x, robot.y), (end_x, end_y), (0, 255, 255), 2)

        # Draw the mouse cursor
        cv2.circle(frame, (mouse_x, mouse_y), 8, (255, 0, 0), -1)
        cv2.putText(frame, "Mouse", (mouse_x + 10, mouse_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

        # Draw line from marker to mouse
        cv2.line(frame, (robot.x, robot.y), (mouse_x, mouse_y), (0, 255, 0), 2)

        # Show the frame
        cv2.imshow("Marker + Mouse + Heading", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()