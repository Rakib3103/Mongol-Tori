import pyrealsense2 as rs
import cv2
import numpy as np
import cv2.aruco as aruco

# Configure the RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Define the ArUco dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
aruco_params = aruco.DetectorParameters_create()

while True:
    # Capture a frame from the RealSense camera
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    # Convert the color frame to an OpenCV image
    color_image = np.asanyarray(color_frame.get_data())

    # Detect the ArUco markers in the color image
    corners, ids, rejected = aruco.detectMarkers(color_image, aruco_dict, parameters=aruco_params)

    # If any markers are detected, estimate their distance
    if ids is not None:
        # Draw the detected markers on the color image
        aruco.drawDetectedMarkers(color_image, corners, ids)

        # Calculate the distance to each marker using the depth frame
        for i in range(len(ids)):
            id = ids[i][0]
            marker_corners = corners[i][0]
            x, y = int(np.mean(marker_corners[:, 0])), int(np.mean(marker_corners[:, 1]))
            depth = depth_frame.get_distance(x, y)
            print(f"AR tag {id}: distance {depth:.2f} meters")

    # Display the color image with the detected markers
    cv2.imshow("AR tag detection", color_image)
    if cv2.waitKey(1) == ord("q"):
        break

# Clean up
pipeline.stop()
cv2.destroyAllWindows()
