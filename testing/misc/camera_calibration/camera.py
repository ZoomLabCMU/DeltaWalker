import cv2
import time
import matplotlib.pyplot as plt

# Open the default camera (usually the first one)
cap = cv2.VideoCapture(0)


def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


def camera_dist_test(camera_matrix, dist_matrix):
    ret, frame = cap.read()
    size = (frame.shape[0], frame.shape[1])
    # operations on the frame come here

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, size, 0, size)
    img = cv2.undistort(frame, camera_matrix, dist_matrix, None, new_camera_matrix)

    # img = cv2.undistort(frame, cameraMatrix=camera_matrix, distCoeffs=dist_matrix)

    cv2.imshow('distorted', frame)

    cv2.waitKey(1000)

    cv2.imshow('undistorted', img)
    cv2.waitKey(1000)

    cap.release()
    cv2.destroyAllWindows()

    print(frame.shape)
    print(img.shape)


def show_frame():
    vid = cv2.VideoCapture("../videos/unannotated/undistorted/trajA_3x_cable_up.avi")
    ret, frame = vid.read()
    frame = cv2.flip(frame, 0)
    cv2.imshow('frame', frame)
    cv2.waitKey(0)
    plt.imshow(frame)
    plt.show()

    # distorted     29.6 pixels / 1 cm      center at 991, 530
    # undistorted   29.8 pixels / 1 cm      center at 986, 531


def main():
    idx = 0

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    num_frames = 0
    start = time.time()
    # Loop to continuously read frames from the camera

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    size = (w, h)
    fps = 12
    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
    out = cv2.VideoWriter(f'test.avi', fourcc, fps, size)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        num_frames += 1
        out.write(frame)

        # Display the frame
        cv2.imshow('Camera Feed', frame)

        # if cv2.waitKey(1) & 0xFF == ord('s'):
        #     savename = f"./imgs/calib_{idx}.png"
        #     cv2.imwrite(savename, frame)
        #     print(f"saved image {idx}")
        #     idx += 1
        #     time.sleep(1)

        # Check for the 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            end = time.time()
            break

    elapsed_time = end - start
    print(elapsed_time)
    print(num_frames/elapsed_time)
    print(num_frames)

    # Release the camera and close OpenCV windows
    cap.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # main()
    file = '../camera.yml'
    # camera_matrix, dist_matrix = load_coefficients(file)

    # camera_dist_test(camera_matrix, dist_matrix)
    show_frame()
