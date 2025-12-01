import cv2
import cv2.aruco as aruco
import numpy as np

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


    #def detect_markers(self, robots, frame):
        # ret, frame = self.cap.read()
        # if not ret: return
        '''
        # Detect all markers in the scene
        corners, ids, rejected = self.detector.detectMarkers(frame)
        
        # Extract the pose of each marker
        if ids is not None:
            # Draw the detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estimate pose for each detected marker
            rotation_vectors, translation_vectors, _ = aruco.estimatePoseSingleMarkers(
                corners, 
                self.marker_length, 
                self.camera_matrix, 
                self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rotation = rotation_vectors[i]
                translation = translation_vectors[i]

                # Draw the coordinate axes on each marker in the frame
                cv2.drawFrameAxes(
                    frame, 
                    self.camera_matrix, 
                    self.dist_coeffs, 
                    rotation, 
                    translation, 
                    0.03
                )

                # Extract the pose
                x, y, z = translation[0]
                yaw = self.calculate_yaw(rotation)

                # Print and store the pose
                # print(f"Marker ID: {marker_id} | X: {x:.3f}m | Y: {y:.3f}m | Z: {z:.3f}m | Yaw: {yaw:.1f}°")
                # self.marker_dict[marker_id] = {"x": x, "y": y, "yaw": yaw}
                # return self.marker_dict
            
                robots[i].x = x
                robots[i].y = y
                robots[i].heading = yaw
            '''

        
        # Debug
        # cv2.imshow("ArUco Detection", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     return

    
    # def shutdown(self):
    #     self.cap.release()
    #     cv2.destroyAllWindows

    def detect_markers(self, robots, frame):
        """
        Detect ArUco markers and store their pixel centers in robots[i].x, robots[i].y.
        `robots` is a list of robot objects (each with .x, .y, .heading).
        """
        # Detect markers in the frame
        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is not None:
            # Draw detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                c = corners[i][0]  # 4 corners (x, y)
                center_x = int(c[:, 0].mean())
                center_y = int(c[:, 1].mean())

                # Save pixel coordinates
                robots[i].x = center_x
                robots[i].y = center_y
                robots[i].heading = 0  # optional, heading not used in pixels

                # Optional: draw the center
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

                # Store in marker_dict if needed
                self.marker_dict[marker_id] = {"x": center_x, "y": center_y}

        # Optional: show the frame for debugging
        # cv2.imshow("ArUco Detection", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     return



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
