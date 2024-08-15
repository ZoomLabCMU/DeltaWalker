from processing_helpers import *


# Livecam of undistorted video for debugging
def livecam(cap, camera_matrix, dist_matrix, new_camera_matrix):
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 0)

        undistorted = cv2.undistort(frame, camera_matrix, dist_matrix, None, new_camera_matrix)

        # undist_data, undistorted = tracking(undistorted, 0)

        # Display the frame
        cv2.imshow('Camera Feed', undistorted)
        # print(undist_data)

        # Check for the 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def debug(cap, camera_matrix, dist_matrix, new_camera_matrix):
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 0)

        undistorted = cv2.undistort(frame, camera_matrix, dist_matrix, None, new_camera_matrix)

        undist_data, undist_frame = mask_tuning(undistorted, 0)

        # Display the frame
        cv2.imshow('Camera Feed', undist_frame)
        print(undist_data)

        # Check for the 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    # Capture Info
    cap = cv2.VideoCapture(0)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))      # 1920
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))     # 1080
    size = (w, h)
    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')

    # Get camera matrix
    camera_matrix, dist_matrix = load_coefficients('camera.yml')
    # Update camera matrix
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, size, 0, size)

    livecam(cap, camera_matrix, dist_matrix, new_camera_matrix)
    # debug(cap, camera_matrix, dist_matrix, new_camera_matrix)

