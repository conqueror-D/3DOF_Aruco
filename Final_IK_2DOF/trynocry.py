import cv2 as cv
from cv2 import aruco
import numpy as np
import socket
import time

ESP32_IP = '192.168.27.6'  # Change this to your ESP32 IP address
ESP32_PORT = 80

def send_integers_to_esp32(integers):
    try:
        # Connect to the ESP32 server
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((ESP32_IP, ESP32_PORT))
            # Send the integers to the ESP32 server
            s.sendall(integers.encode())
            # Receive response from the server
            response = s.recv(1024)
            print("Response from ESP32:", response.decode())
    except Exception as e:
        print("Error:", e)
# Load the calibration data
calib_data_path = "./camera_calibration_data.npz"
calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camera_matrix"]
dist_coef = calib_data["distortion_coefficients"]
r_vectors = calib_data["rotation_vectors"]
t_vectors = calib_data["translation_vectors"]

MARKER_SIZE = 3  # Marker size in centimeters (measure your printed marker size)
CM_TO_INCH = 1 / 2.54  # Conversion factor from centimeters to inches

# Define the ArUco dictionary and parameters
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
param_markers = aruco.DetectorParameters()

# Initialize the video capture
cap = cv.VideoCapture("http://192.168.27.104:4747/video")
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 30)

# Define the rotation matrix from the robotic base frame (frame 0) to the camera frame (frame c).
rot_angle = 180  # angle between axes in degrees
rot_angle = np.deg2rad(rot_angle)
rot_mat_0_c = np.array([[1, 0, 0],
                        [0, np.cos(rot_angle), -np.sin(rot_angle)],
                        [0, np.sin(rot_angle), np.cos(rot_angle)]])

# Define the displacement vector from frame 0 to frame c
disp_vec_0_c = np.array([[3],
                         [17],  # This was originally 23.0 but I modified it for accuracy
                         [0.0]])

# Row vector for bottom of homogeneous transformation matrix
extra_row_homgen = np.array([[0, 0, 0, 1]])

# Create the homogeneous transformation matrix from frame 0 to frame c
homgen_0_c = np.concatenate((rot_mat_0_c, disp_vec_0_c), axis=1)  # side by side
homgen_0_c = np.concatenate((homgen_0_c, extra_row_homgen), axis=0)  # one above the other

# Initialize coordinates in the robotic base frame
coord_base_frame = np.array([[0.0],
                             [0.0],
                             [0.0],
                             [1]])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)

            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Extract the translation vectors for x and y distances in centimeters
            x_dist = tVec[i][0][0]
            y_dist = tVec[i][0][1]

            # Coordinates of the object in the camera reference frame
            cam_ref_coord = np.array([[x_dist],
                                      [y_dist],
                                      [0.0],
                                      [1]])

            # Coordinates of the object in base reference frame
            coord_base_frame = homgen_0_c @ cam_ref_coord

            x_base_frame = coord_base_frame[0][0]
            y_base_frame = coord_base_frame[1][0]

            # Display the pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            #cv.putText(frame, f"id: {ids[0]}", bottom_right, cv.FONT_HERSHEY_PLAIN, 1.3, (255, 255, 255), 2, cv.LINE_AA)
            cv.putText(frame, f"x: {round(x_base_frame, 1)} cm y: {round(y_base_frame, 1)} cm", bottom_left, cv.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 2, cv.LINE_AA)

            print(f"ID: {ids[0]}, Corners: {corners}, X distance: {x_base_frame} cm, Y distance: {y_base_frame} cm")

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    
        
    if key == ord("f"):
    	CM_INCHES = 1/2.54
    	x_in = (coord_base_frame[0][0] - 4) * CM_INCHES
    	y_in = (coord_base_frame[1][0]) * CM_INCHES
    	integers = f"{x_in:.0f} {y_in:.0f} {0} {0}"
    	print(integers)
    	send_integers_to_esp32(integers)
    elif key == ord("p"):
    	integers = f"{x_in:.0f} {y_in:.0f} {0} {1}"
    	send_integers_to_esp32(integers)
    elif key == ord("q"):
        break
camera.release()
cv2.destroyAllWindows()
