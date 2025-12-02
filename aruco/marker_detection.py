import cv2
import cv2.aruco as aruco
import numpy as np
import math

# TODO: add the calibration as a function in this class
class ArucoMarkers():
    def __init__(self, ran_from_main_program=1):
        if not ran_from_main_program:
            # Initialize the webcam 
            self.cap = cv2.VideoCapture(2)
            if not self.cap.isOpened():
                print("Error: Could not open webcam.")
                return
        
        # Initialize ArUco detection parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.marker_length = 0.127  # The actual length of each marker, in meters (5 inches = 0.127 meters)
        
        # Define the calibration intrinsics
        # self.camera_matrix = np.array([
        #     [785.4211781621483, 0.0, 318.05277676515107],
        #     [0.0, 784.137732741561, 240.83254986145354],
        #     [0.0, 0.0, 1.0]
        # ], dtype=np.float32)
        
        # self.dist_coeffs = np.array([
        #     -0.7874504210687944,
        #     7.18045442894109,
        #     0.03732237657226428,
        #     -0.013297080410922117,
        #     -36.37566733804244
        # ])

        self.camera_matrix = np.array([
            [618.38619457, 0.0, 319.89973985],
            [0.0, 620.82463148, 250.87075894],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.array([
            -0.15575992,
            0.57450687,
            0.00850961,
            -0.00882722,
            -1.18844877
        ])

        # The allows ids. One for each robot.
        self.marker_ids = {1, 2, 3}

        # id: {"x": x, "y": y, "yaw": yaw}
        self.marker_dict = {}


    def generate_markers(self) -> None:

        # Choose a dictionary (e.g., 4x4 markers with 50 unique IDs)
        marker_size = 200  # In pixels
        
        for id in self.marker_ids:
            if id not in range(0,50):
                print(f"The marker {id} was not in the allowed range of [0,50)")
                continue
            
            marker_image = aruco.generateImageMarker(self.aruco_dict, id, marker_size)

            # Save the marker image
            cv2.imwrite(f"markers/marker_id_{id}.png", marker_image)

            # Display the marker
            cv2.imshow("ArUco Marker", marker_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


    def calculate_yaw(self, rotation):
        # Convert the rotation vector into a 3x3 matrix
        rmat, _ = cv2.Rodrigues(rotation)
        sy = np.sqrt(rmat[0,0]**2 + rmat[1,0]**2)
        singular = sy < 1e-6
        
        # Z-Y-X decomposition
        if not singular:
            x_angle = np.arctan2(rmat[2,1], rmat[2,2])
            y_angle = np.arctan2(-rmat[2,0], sy)
            z_angle = np.arctan2(rmat[1,0], rmat[0,0])
        else:
            x_angle = np.arctan2(-rmat[1,2], rmat[1,1])
            y_angle = np.arctan2(-rmat[2,0], sy)
            z_angle = 0
        
        yaw = np.degrees(z_angle)
        return yaw

    def detect_markers(self, marker_id_to_robots, frame):
        # Clear detection flags each frame
        for robot in marker_id_to_robots.values():
            robot.detected = False

        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in marker_id_to_robots:
                    continue

                robot = marker_id_to_robots[marker_id]
                robot.detected = True 

                c = corners[i][0]
                center_x = int(c[:, 0].mean())
                center_y = int(c[:, 1].mean())

                dx = c[0][0] - c[3][0]
                dy = c[0][1] - c[3][1]

                c = corners[i][0]  # 4 corners: top-left, top-right, bottom-right, bottom-left

                # Length of top side (top-left to top-right)
                top_length = np.linalg.norm(c[1] - c[0])

                # Length of right side (top-right to bottom-right)
                right_length = np.linalg.norm(c[2] - c[1])

                # Length of bottom side (bottom-right to bottom-left)
                bottom_length = np.linalg.norm(c[3] - c[2])

                # Length of left side (bottom-left to top-left)
                left_length = np.linalg.norm(c[0] - c[3])

                # Average side length
                avg_length = (top_length + right_length + bottom_length + left_length) / 4

                robot.stop_mag = avg_length * 2

                robot.heading = np.degrees(np.arctan2(dy, dx))
                #print(robot.heading)

                robot.x = center_x
                robot.y = center_y
                robot.stop_mag



if __name__=="__main__":
    # Initialize the detector
    marker_obj = ArucoMarkers()
    
    # Generate markers
    # marker_obj.generate_markers()

    # Detect markers
    while True:
        marker_obj.detect_markers()

    # Shut everything down
    # marker_obj.shutdown()
