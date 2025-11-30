##################
# PYTHON IMPORTS #
##################

import cv2
import time
import threading
from math import cos, sin, radians, degrees

#################
# LOCAL IMPORTS #
#################

from aruco.marker_detection import ArucoMarkers
from command_transmission import CommandTransmission
from heading_error_detection import heading_error, distance_error


class Robot():
    # TODO: Do I want to implement a FIFO buffer for the behind point?
    x               = 0.0
    y               = 0.0
    heading         = 0.0
    behind_point_x  = 0.0
    behind_point_y  = 0.0
    goal_point_x    = 0.0
    goal_point_y    = 0.0
    l_pwm           = 0
    r_pwm           = 0


class MainProgram():
    
    def __init__(self):
        # Variables shared across threads:
        self.mouse_pos = {"x": 0, "y": 0}
        self.latest_frame = None
        self.running = True

        # Used to store each robot's instantaneous values
        self.robots = {
            1: Robot(),
            2: Robot(),
            3: Robot()
        }

        self.aruco = ArucoMarkers()
        self.transmission = CommandTransmission()

        self.t1 = threading.Thread(target=self.camera_thread, daemon=True)
        self.t2 = threading.Thread(target=self.compute_thread, daemon=True)
        
        self.t1.start()
        self.t2.start()
        
        self.t1.join()
        self.running = False
        self.t2.join()
    

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_pos["x"] = x
            self.mouse_pos["y"] = y


    def camera_thread(self):
        cap = cv2.VideoCapture(2)
        cv2.namedWindow("camera")
        cv2.setMouseCallback("camera", self.mouse_callback)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Update latest_frame so the aruco object can access it
            self.latest_frame = frame
                
            cv2.imshow("camera", frame)
            if cv2.waitKey(1) == 27:
                break

        cap.release()
        cv2.destroyAllWindows()


    def compute_pwm(
            self, 
            max_heading_pwm = 200, 
            min_heading_pwm = 10,
            h_scale = 1,
            min_distance = 0.3
    ):
        
        for robot in self.robots:

            ############
            # DISTANCE #
            ############

            # Set the baseline forward velocity
            distance_err = distance_error(
                (self.robots[robot].x, self.robots[robot].y),
                (self.robots[robot].goal_point_x, self.robots[robot].goal_point_y)
            )

            # TODO: We decided to just set a constant forward velocity
            # # Apply to both motors
            # distance_pwm = distance_err * d_scale

            # # If the robot is within a certain scaled distance of its goal, stop it
            # if distance_pwm < 5:
            #     distance_pwm = 0
            # else:
            #     distance_pwm = max(20, min(distance_pwm, 40))

            # TODO: what units is distance_err in?
            if distance_err < min_distance:
                distance_pwm = 0
            else:
                distance_pwm = 30

            self.robots[robot].l_pwm = int(distance_pwm)
            self.robots[robot].r_pwm = int(distance_pwm)
            
            ###########
            # HEADING #
            ###########

            heading_err = heading_error(
                (self.robots[robot].x, self.robots[robot].y),
                (self.robots[robot].goal_point_x, self.robots[robot].goal_point_y),
                self.robots[robot].heading
            )

            # Apply to the outer motor
            heading_pwm = heading_err * h_scale

            # Constrain to abs(0, 255). 
            heading_pwm = max(-max_heading_pwm, min(heading_pwm, max_heading_pwm))

            if abs(heading_pwm) < min_heading_pwm:
                heading_pwm = 0

            # A (-) heading means the robot needs to turn right (L motor needs to outpace R motor)
            if heading_pwm < 0:
                self.robots[robot].l_pwm += int(heading_pwm)
            else:
                self.robots[robot].r_pwm += int(heading_pwm)


    def back_interpolate(self, offset_distance = 0.3):
        robot_1_behind_x = self.robots[1].x - offset_distance * cos(radians(self.robots[1].heading))
        robot_1_behind_y = self.robots[1].y - offset_distance * sin(radians(self.robots[1].heading))
        self.robots[2].goal_point_x = robot_1_behind_x
        self.robots[2].goal_point_y = robot_1_behind_y

        robot_2_behind_x = self.robots[2].x - offset_distance * cos(radians(self.robots[2].heading))
        robot_2_behind_y = self.robots[2].y - offset_distance * sin(radians(self.robots[2].heading))
        self.robots[3].goal_point_x = robot_2_behind_x
        self.robots[3].goal_point_y = robot_2_behind_y


    def compute_thread(self):
        while True:
            frame = self.latest_frame
            if frame is None:
                time.sleep(0.001)
                continue
            
            # Get the mouse position
            mx, my = self.mouse_pos["x"], self.mouse_pos["y"]

            # Add the current position of each robot, in place
            self.aruco.detect_markers(self.robots)

            # Operates on self.robots to compute a point behind each robot
            self.back_interpolate()

            # Set each robots goal
            self.robots[1].goal_point_x = mx
            self.robots[1].goal_point_y = my
            self.robots[2].goal_point_x = self.robots[1].behind_point_x
            self.robots[2].goal_point_y = self.robots[1].behind_point_y
            self.robots[3].goal_point_x = self.robots[2].behind_point_x
            self.robots[3].goal_point_y = self.robots[2].behind_point_y

            # Operates on self.robots to compute l and r PWMs
            self.compute_pwm()
            
            self.transmission.send_pwm(
                1,
                self.robots[1].l_pwm, 
                self.robots[1].r_pwm
            )
            self.transmission.send_pwm(
                2,
                self.robots[2].l_pwm, 
                self.robots[2].r_pwm
            )
            self.transmission.send_pwm(
                3,
                self.robots[3].l_pwm, 
                self.robots[3].r_pwm
            )

            time.sleep(0.001)

 
    
if __name__=="__main__":
    command = input("Would you like to emergency stop? (y/n)")
    if command.lower() == 'y':
        command = CommandTransmission()
        command.send_halt()
    else:
        obj = MainProgram()
