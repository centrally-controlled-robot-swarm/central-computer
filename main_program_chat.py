"""
Fixed and improved robot controller.
- Robot uses instance attributes (no shared class vars)
- Thread-safe access to latest_frame and robots (locks)
- Clean start/stop using run() and KeyboardInterrupt
- compute_pwm: uses items(), clamps PWMs after heading adjustment
- back_interpolate stores behind_point in each robot
- Assumes ArucoMarkers.detect_markers(self.robots) sets per-robot attribute `valid` (True/False)
  If your ArucoMarkers implementation doesn't set `valid`, either add it or adapt the
  checks in compute_thread/compute_pwm.

Notes:
- Check units of distance_error() and heading_error() and set min_distance appropriately.
- Adjust camera index and loop sleep times for your hardware.
"""

import cv2
import time
import threading
from math import cos, sin, radians
from typing import Dict

# Local imports (assumed to exist)
from aruco.marker_detection import ArucoMarkers
from command_transmission import CommandTransmission
from heading_error_detection import heading_error, distance_error


class Robot:
    def __init__(self):
        # pose
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        # behind / goal points
        self.behind_point_x = 0.0
        self.behind_point_y = 0.0
        self.goal_point_x = 0.0
        self.goal_point_y = 0.0

        # outputs
        self.l_pwm = 0
        self.r_pwm = 0

        # detection flag (Aruco detector should set this)
        self.valid = False


class MainProgram:
    def __init__(self, camera_index: int = 0):
        # Shared variables
        self.mouse_pos = {"x": 0, "y": 0}
        self.latest_frame = None
        self.running = False

        # Locks for thread-safety
        self.frame_lock = threading.Lock()
        self.robots_lock = threading.Lock()

        # Robots dictionary
        self.robots: Dict[int, Robot] = {
            1: Robot(),
            2: Robot(),
            3: Robot()
        }

        # Hardware / helper objects
        self.aruco = ArucoMarkers()
        self.transmission = CommandTransmission()

        # Camera index (device)
        self.camera_index = camera_index

        # Threads
        self.t1 = threading.Thread(target=self.camera_thread, daemon=True)
        self.t2 = threading.Thread(target=self.compute_thread, daemon=True)

    # -----------------
    # Mouse callback
    # -----------------
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            # small critical section; okay without lock for responsiveness
            self.mouse_pos["x"] = x
            self.mouse_pos["y"] = y

    # -----------------
    # Camera thread
    # -----------------
    def camera_thread(self):
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            print(f"Error: camera index {self.camera_index} not available")
            # stop the program by clearing running
            self.running = False
            return

        cv2.namedWindow("camera")
        cv2.setMouseCallback("camera", self.mouse_callback)

        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("Warning: failed to read frame from camera")
                time.sleep(0.01)
                continue

            # store a copy under lock
            with self.frame_lock:
                # store a shallow copy reference; if you mutate frame elsewhere call .copy()
                self.latest_frame = frame.copy()

            cv2.imshow("camera", frame)
            # waitKey required to refresh window and capture keypress
            key = cv2.waitKey(1)
            if key == 27:  # ESC key to stop
                self.running = False
                break

        cap.release()
        cv2.destroyAllWindows()

    # -----------------
    # Compute PWM
    # -----------------
    def compute_pwm(
            self,
            max_heading_pwm: int = 200,
            min_heading_pwm: int = 10,
            h_scale: float = 1.0,
            min_distance: float = 0.3
    ):
        # Operates on self.robots; caller should hold robots_lock if needed
        for robot_id, robot in self.robots.items():
            # If the aruco detector indicates this robot wasn't seen, skip control for it
            if hasattr(robot, 'valid') and not robot.valid:
                # keep previous PWM or set to zero to be safe
                robot.l_pwm = 0
                robot.r_pwm = 0
                continue

            # DISTANCE
            distance_err = distance_error(
                (robot.x, robot.y),
                (robot.goal_point_x, robot.goal_point_y)
            )

            # NOTE: ensure distance_err units match min_distance (meters vs pixels)
            if distance_err < min_distance:
                distance_pwm = 0
            else:
                # constant forward speed; tune as needed
                distance_pwm = 30

            robot.l_pwm = int(distance_pwm)
            robot.r_pwm = int(distance_pwm)

            # HEADING
            heading_err = heading_error(
                (robot.x, robot.y),
                (robot.goal_point_x, robot.goal_point_y),
                robot.heading
            )

            heading_pwm = heading_err * h_scale
            heading_pwm = max(-max_heading_pwm, min(heading_pwm, max_heading_pwm))

            if abs(heading_pwm) < min_heading_pwm:
                heading_pwm = 0

            if heading_pwm < 0:
                robot.l_pwm += int(heading_pwm)  # negative reduces left motor
            else:
                robot.r_pwm += int(heading_pwm)

            # Final safety clamp (ensure driver accepts only [0, 255])
            robot.l_pwm = max(0, min(robot.l_pwm, 255))
            robot.r_pwm = max(0, min(robot.r_pwm, 255))

    # -----------------
    # Back interpolation
    # -----------------
    def back_interpolate(self, offset_distance: float = 0.3):
        # compute behind points for each robot and store them in the robot object
        with self.robots_lock:
            for i in (1, 2):  # compute for robot 1 and 2 so 2 and 3 can follow
                r = self.robots[i]
                r.behind_point_x = r.x - offset_distance * cos(radians(r.heading))
                r.behind_point_y = r.y - offset_distance * sin(radians(r.heading))

    # -----------------
    # Main compute thread
    # -----------------
    def compute_thread(self):
        # aim for ~50 Hz control loop
        loop_period = 0.02

        while self.running:
            # copy the latest frame safely
            with self.frame_lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()

            if frame is None:
                # no frame yet
                time.sleep(0.001)
                continue

            # Let ArucoMarkers read the shared frame or internal camera if it needs to.
            # If ArucoMarkers requires the frame to be set on the object, adapt accordingly.

            # Update robot poses using the aruco detector. We lock robots while detector updates them
            with self.robots_lock:
                # It's expected that detect_markers writes to self.robots[*].(x,y,heading,valid)
                try:
                    # If your detect_markers function takes frame, pass it. Otherwise leave as-is.
                    # self.aruco.detect_markers(self.robots, frame)
                    self.aruco.detect_markers(self.robots, frame)
                except TypeError:
                    # Fallback if detect_markers requires different args
                    self.aruco.detect_markers(self.robots, frame)

                # Compute behind points (stores behind_point_* in robot objects)
                self.back_interpolate()

                # Set goals: robot 1 -> mouse, robot 2 -> robot1.behind, robot3 -> robot2.behind
                mx, my = self.mouse_pos["x"], self.mouse_pos["y"]
                self.robots[1].goal_point_x = mx
                self.robots[1].goal_point_y = my

                # If behind_point hasn't been set (robot invalid), keep existing goal
                if getattr(self.robots[1], 'valid', False):
                    self.robots[2].goal_point_x = self.robots[1].behind_point_x
                    self.robots[2].goal_point_y = self.robots[1].behind_point_y

                if getattr(self.robots[2], 'valid', False):
                    self.robots[3].goal_point_x = self.robots[2].behind_point_x
                    self.robots[3].goal_point_y = self.robots[2].behind_point_y

                # Compute PWMs
                self.compute_pwm()

                # Send PWMs out
                for rid in (1, 2, 3):
                    r = self.robots[rid]
                    # If robot not valid, we already set pwms to zero in compute_pwm
                    self.transmission.send_pwm(rid, r.l_pwm, r.r_pwm)

            # sleep to maintain loop rate
            time.sleep(loop_period)

    # -----------------
    # Run / Shutdown
    # -----------------
    def run(self):
        self.running = True
        self.t1.start()
        self.t2.start()

        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Shutting down (KeyboardInterrupt)")
            self.running = False

        # join threads
        self.t1.join()
        self.t2.join()

    def emergency_stop(self):
        # Immediately send halt/zero commands to robots
        for rid in (1, 2, 3):
            self.transmission.send_pwm(rid, 0, 0)


if __name__ == "__main__":
    resp = input("Would you like to emergency stop? (y/n) ")
    if resp.strip().lower() == 'y':
        cmd = CommandTransmission()
        cmd.send_halt()
    else:
        mp = MainProgram(camera_index=2)
        try:
            mp.run()
        finally:
            mp.emergency_stop()
