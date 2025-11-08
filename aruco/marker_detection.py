import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoMarkers():
    def __init__(self):
        # Initialize the webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open webcam.")
            return
        
        # Initialize ArUco detection parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Define the calibration intrinsics
        # TODO

        # id: {"x": x, "y": y, "yaw": yaw}
        self.marker_dict = {}


    def generate_markers(self, marker_ids: set) -> None:

        # Choose a dictionary (e.g., 4x4 markers with 50 unique IDs)
        marker_size = 200  # In pixels
        
        for id in marker_ids:
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


    def detect_markers(self):
        ret, frame = self.cap.read()
        if not ret: return

        # Detect all markers in the scene
        corners, ids, rejected = self.detector.detectMarkers(frame)
        
        # Extract the pose of each marker
        if ids is not None:
            # Draw the detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose for each detected marker
            rotation_vectors, translation_vectors, _ = aruco.estimatePoseSingleMarkers(
                corners, 
                marker_length, 
                camera_matrix, 
                dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = rotation_vectors[i], translation_vectors[i]

                # Draw the coordinate axes on each marker
                aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

                # Extract position
                x, y, z = tvec[0]
                # Extract yaw (rotation around z-axis)
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                yaw = np.degrees(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))

                print(f"Marker ID: {marker_id} | X: {x:.3f}m | Y: {y:.3f}m | Z: {z:.3f}m | Yaw: {yaw:.1f}°")
                self.marker_dict[id] = {"x": x, "y": y, "yaw": yaw}
        
        # cv2.imshow("ArUco Detection", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     return

    
    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows




if __name__=="__main__":
    # Initialize the detector
    marker_obj = ArucoMarkers()
    
    # Generate markers
    marker_ids = {1, 2, 3}
    marker_obj.generate_markers(marker_ids)

    # Detect markers
    for i in range(100):
        marker_obj.detect_markers()

    # Shut everything down
    marker_obj.shutdown()
