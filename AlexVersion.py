import cv2
import numpy as np
from aruco.marker_detection import ArucoMarkers
from heading_error_detection import heading_error, distance_error
from command_transmission import CommandTransmission

# Simple robot object
class Robot:
    def __init__(self, id="", name=""):
        self.name = name
        self.id = id
        self.x = 0
        self.y = 0
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.stop_mag = 0
        self.l_pwm = 0
        self.r_pwm = 0
        self.detected = False

'''def compute_pwm_test(error):
    # tunable gains
    Kp = 1.2

    turn = Kp * error

    # clamp output
    turn = max(-255, min(255, turn))

    # simple differential drive
    left_pwm  = 150 - turn
    right_pwm = 150 + turn

    # clamp again
    left_pwm  = max(0, min(255, left_pwm))
    right_pwm = max(0, min(255, right_pwm))

    return int(left_pwm), int(right_pwm)
'''
def angle_diff(target, current):
    diff = (target - current + 180) % 360 - 180
    return diff

def compute_pwm(
        robot,
        heading_err,
        max_heading_pwm: int = 200,
        min_heading_pwm: int = 10,
        h_scale: float = 0.2,
        min_distance: float = 150, #Figure out how many pixels we want
        forward_pwm: int = 30
):
    # DISTANCE
    distance_err = distance_error(
        (robot.x, robot.y),
        (robot.goal_x, robot.goal_y)
    )

    # NOTE: ensure distance_err units match min_distance (meters vs pixels)
    if distance_err < min_distance:
        robot.l_pwm = 0
        robot.r_pwm = 0
        return
    else:
        # constant forward speed; tune as needed
        distance_pwm = forward_pwm

    robot.l_pwm = int(distance_pwm)
    robot.r_pwm = int(distance_pwm)

    heading_pwm = heading_err * h_scale
    heading_pwm = max(-max_heading_pwm, min(heading_pwm, max_heading_pwm))

    # if abs(heading_pwm) < min_heading_pwm:
    #     heading_pwm = 0

    if heading_pwm < 0:
        robot.l_pwm += int(-heading_pwm)  # negative reduces left motor
    else:
        robot.r_pwm += int(heading_pwm)

    # Final safety clamp (ensure driver accepts only [0, 255])
    robot.l_pwm = max(0, min(robot.l_pwm, 255))
    robot.r_pwm = max(0, min(robot.r_pwm, 255))

# Global mouse position relative to OpenCV window
mouse_pos = [0, 0]

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_pos[0] = x
        mouse_pos[1] = y

def main():
    # Initialize ArUco detector
    marker_detector = ArucoMarkers()
    
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    # Create robots
    robot1 = Robot("1","robot_1")  # Marker ID 1
    robot2 = Robot("2","robot_1")  # Marker ID 2
    robot3 = Robot("3","robot_1")  # Marker ID 3

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
            
            robot.goal_x = mouse_x
            robot.goal_y = mouse_y

            dx_m = mouse_x - robot.x
            dy_m = mouse_y - robot.y
            mouse_angle = np.degrees(np.arctan2(dy_m, dx_m))

            #heading_error = angle_diff(mouse_angle, robot.heading)

            heading_vector_error = heading_error((robot.x,robot.y),(mouse_x,mouse_y),robot.heading)

            compute_pwm(robot, heading_vector_error, min_distance=robot.stop_mag)
            print("Id: ",robot.id)

            print("Left/Right PWM: ",robot.l_pwm, robot.r_pwm)
            transmission.send_pwm(int(robot.id),robot.r_pwm,robot.l_pwm)
            #print("PWM:", left_pwm, right_pwm)
            #print(angle_btwn_vectors)
            #print(robot.heading)
            #print(mouse_angle)
            #print(heading_error)

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
    transmission = CommandTransmission()
    main()
