import cv2
import numpy as np
from aruco.marker_detection import ArucoMarkers

# Simple robot object
class Robot:
    def __init__(self, name=""):
        self.x = 0
        self.y = 0
        self.heading = 0
        self.name = name
        self.detected = False

# Global mouse position relative to OpenCV window
mouse_pos = [0, 0]

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_pos[0] = x
        mouse_pos[1] = y

def main():
    # Initialize ArUco detector
    marker_detector = ArucoMarkers()
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    # Create robots
    robot1 = Robot("Robot1")  # Marker ID 1
    robot2 = Robot("Robot2")  # Marker ID 2
    robot3 = Robot("Robot3")  # Marker ID 3

    # Map marker IDs to robots
    marker_id_to_robot = {
        1: robot1,
        2: robot2,
        3: robot3
    }

    cv2.namedWindow("ArUco + Mouse + Heading")
    cv2.setMouseCallback("ArUco + Mouse + Heading", mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Detect all markers and update their robots
        marker_detector.detect_markers(marker_id_to_robot, frame)

        # Get mouse position relative to frame
        mouse_x, mouse_y = mouse_pos[0], mouse_pos[1]

        # Draw all robots that are currently detected
        for robot in marker_id_to_robot.values():
            if not robot.detected:
                continue  # skip robots not detected this frame

            # Draw marker center
            cv2.circle(frame, (robot.x, robot.y), 8, (0, 0, 255), -1)
            cv2.putText(frame, robot.name, (robot.x + 10, robot.y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

            # Draw heading vector
            length = 50
            rad = np.radians(robot.heading)
            end_x = int(robot.x + length * np.cos(rad))
            end_y = int(robot.y + length * np.sin(rad))
            cv2.arrowedLine(frame, (robot.x, robot.y), (end_x, end_y), (0, 255, 255), 2)

            # Draw line to mouse
            cv2.line(frame, (robot.x, robot.y), (mouse_x, mouse_y), (0, 255, 0), 2)

        # Draw mouse cursor
        cv2.circle(frame, (mouse_x, mouse_y), 8, (255, 0, 0), -1)
        cv2.putText(frame, "Mouse", (mouse_x + 10, mouse_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

        # Show frame
        cv2.imshow("ArUco + Mouse + Heading", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
