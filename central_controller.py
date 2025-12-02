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

def angle_diff(target, current):
    #always stays between 180 and -180, so robot never turns more than that
    diff = (target - current + 180) % 360 - 180
    return diff

#Function that computes bwm based on adjustable parameters and heading_errs
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
        robot.r_pwm += int(-heading_pwm)  # negative reduces left motor
    else:
        robot.l_pwm += int(heading_pwm)

    # Final safety clamp (ensure driver accepts only [0, 255])
    robot.l_pwm = max(0, min(robot.l_pwm, 255))
    robot.r_pwm = max(0, min(robot.r_pwm, 255))

#to compute the position followers should follow, adjust offset depending on 
#worskpace size
def behind_position(robot, offset=75):
    rad = np.radians(robot.heading)
    offset_x = -offset * np.cos(rad)
    offset_y = -offset * np.sin(rad)
    return robot.x + offset_x, robot.y + offset_y

#Global mouse position for call back event in cv2 window
mouse_pos = [0, 0]

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_pos[0] = x
        mouse_pos[1] = y

def main():
    #Initialize ArUco system
    marker_detector = ArucoMarkers()
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    #define robot instances
    robot1 = Robot(1,"robot_1") 
    robot2 = Robot(2,"robot_2") 
    robot3 = Robot(3,"robot_3")  
    robot4 = Robot(4,"robot_4")

    #map these to ids
    marker_id_to_robot = {
        1: robot1,
        2: robot2,
        3: robot3,
        4: robot4
    }

    cv2.namedWindow("Centrally Controled Robot Chain")
    cv2.setMouseCallback("Centrally Controled Robot Chain", mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        #print("---------------------------------------------------------------")
        #ArUco marker detection
        marker_detector.detect_markers(marker_id_to_robot, frame)

        #mouse position in the frame
        mouse_x, mouse_y = mouse_pos[0], mouse_pos[1]


        #Sorts based on ids, but only for robots present ie. [1,2,3] or [1,3]
        #lets us determine leader/follower and skip gaps in frames
        robot_ids = [pseudo_id for pseudo_id in 
                     sorted(marker_id_to_robot.keys()) if marker_id_to_robot
                     [pseudo_id].detected]
        #Robot positional calcs and controls
        for i, robot_id in enumerate(robot_ids):
            robot = marker_id_to_robot[robot_id]

            #first element in sorted list, therefore highest id = leader
            if i == 0:
                robot.goal_x = mouse_x
                robot.goal_y = mouse_y

            #anything else with a lower id becomes a follower to the previous 
            #id in the sorted list ie 3 -> 2 -> 1 OR 3 -> 1
            else:
                leader_id = robot_ids[i-1]
                leader_robot = marker_id_to_robot[leader_id]

                bx, by = behind_position(leader_robot)
                robot.goal_x = bx
                robot.goal_y = by

            heading_vector_error = heading_error((robot.x,robot.y),
                                                 (robot.goal_x,robot.goal_y),
                                                 robot.heading)

            compute_pwm(robot, heading_vector_error, 
                        min_distance=robot.stop_mag)

            print("Id:",robot.id, "| Left/Right PWM:",robot.l_pwm, robot.r_pwm)

        #Drawing robot positions and vectors on the frame (may be bloat)
        for i, robot_id in enumerate(robot_ids):
            robot = marker_id_to_robot[robot_id]
            if not robot.detected:
                continue  #skipping trying to draw nonexistent robots (redundant?)

            #ArUco center display
            cv2.circle(frame, 
                       (robot.x, robot.y), 
                       8, (0, 0, 255), -1)
            cv2.putText(frame, robot.name,
                         (robot.x + 10, robot.y), 
                         cv2.FONT_HERSHEY_SIMPLEX, 
                         1, (255,0,255), 2)

            #ArUco heading display
            length = 75
            rad = np.radians(robot.heading)
            end_x = int(robot.x + length * np.cos(rad))
            end_y = int(robot.y + length * np.sin(rad))
            cv2.arrowedLine(frame, 
                            (robot.x, robot.y), 
                            (end_x, end_y), 
                            (0, 0, 255), 4)
            
            #Goal lines
            if i == 0:# leader
                cv2.line(frame, (robot.x, robot.y),
                        (int(robot.goal_x), int(robot.goal_y)),
                        (0,255,0), 2)
            else : #follower
                cv2.line(frame, (robot.x, robot.y),
                        (int(robot.goal_x), int(robot.goal_y)),
                        (255,255,0), 2)
                #Leader behind position circles
                follower = marker_id_to_robot[robot_ids[i]]
                leader = marker_id_to_robot[robot_ids[i-1]]
                behind_x, behind_y = behind_position(leader)
                cv2.circle(frame, 
                           (int(behind_x), int(behind_y)), 6, 
                           (255,255,0), -1)
                
        # Draw mouse cursor
        cv2.circle(frame, 
                   (mouse_x, mouse_y), 8, 
                   (0, 0, 255), -1)
        cv2.putText(frame, "Mouse", 
                    (mouse_x + 10, mouse_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (0,0,255), 2)

        # Show frame
        cv2.imshow("Centrally Controled Robot Chain", frame)
        print("---------------------------------------------------------------")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    transmission = CommandTransmission()
    main()
