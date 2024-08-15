import cv2
import numpy as np
import time

'''
https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/

follow the same structure as ^ 
find each box center of each frame
store the positions of each box center

what are the box sizes? how many pixels does this camera read?
how to convert pixel position to world position?
do i need to do any camera calibration to prevent distortion? 

'''
# Green (Delta 1), Red (Delta 2), Orange (Delta 3), Blue (Delta 4)
# HSV - 0-180, 0-255, 0-255

green = {'name': 'green',
         'lower': np.array([50, 125, 70], np.uint8),
         'upper': np.array([100, 255, 255], np.uint8),
         'bgr': (0, 255, 0)}

red = {'name': 'red',
       'lower': np.array([170, 120, 70], np.uint8),
       'upper': np.array([180, 255, 255], np.uint8),
       'bgr': (0, 0, 255)}

orange = {'name': 'orange',
          'lower': np.array([0, 150, 150], np.uint8),
          'upper': np.array([25, 255, 255], np.uint8),
          'bgr': (0, 128, 255)}

blue = {'name': 'blue',
        'lower': np.array([100, 100, 50], np.uint8),
        'upper': np.array([120, 255, 255], np.uint8),
        'bgr': (255, 0, 0)}

colors = [green, red, orange, blue]


# Load camera coefficients
def load_coefficients(path):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode('K').mat()
    dist_matrix = cv_file.getNode('D').mat()

    cv_file.release()
    return camera_matrix, dist_matrix


# Record video
def recorder(cap, savename, fourcc, size):
    all_frames = []
    num_frames = 0
    start = time.time()

    # Live camera feed
    while True:
        ret, frame = cap.read()

        # Store the frame
        all_frames.append(frame)

        # Display the frame
        cv2.imshow('Camera Feed', frame)

        num_frames += 1

        # Check for the 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            end = time.time()
            break

    # Calculate frame rate
    elapsed_time = end - start
    fps = num_frames/elapsed_time

    # Write video frames at frame rate
    out = cv2.VideoWriter(f'./videos/unannotated/distorted/{savename}.avi', fourcc, fps, size)
    for i in np.arange(num_frames):
        out.write(all_frames[i])

    # Release the camera and video and close OpenCV windows
    cap.release()
    out.release()
    cv2.destroyAllWindows()

    return


# Livecam of undistorted video for debugging
def debugging_livecam(cap, camera_matrix, dist_matrix, new_camera_matrix):
    while True:
        ret, frame = cap.read()

        undistorted = cv2.undistort(frame, camera_matrix, dist_matrix, None, new_camera_matrix)

        undist_data, undist_frame = tracking(undistorted, 0)

        # Display the frame
        cv2.imshow('Camera Feed', undist_frame)
        print(undist_data)

        # Check for the 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# Undistort video
def undistort(savename, fourcc, size, camera_matrix, dist_matrix, new_camera_matrix, show=False):
    start = time.time()

    vid = cv2.VideoCapture(f'./videos/unannotated/distorted/{savename}.avi')
    fps = vid.get(cv2.CAP_PROP_FPS)
    num_frames = vid.get(cv2.CAP_PROP_FRAME_COUNT)
    # print(fps)

    # Video file
    unann_undist = cv2.VideoWriter(f'./videos/unannotated/undistorted/{savename}.avi', fourcc, fps, size)

    for _ in np.arange(num_frames):
        ret, frame = vid.read()
        undistorted = cv2.undistort(frame, camera_matrix, dist_matrix, None, new_camera_matrix)
        unann_undist.write(undistorted)
        if show:
            cv2.imshow("undistorted", undistorted)
            cv2.waitKey(1)

    unann_undist.release()
    vid.release()
    cv2.destroyAllWindows()

    end = time.time()
    print(f"Duration {end - start} for {num_frames} frames")
    return


# Foot tracking helper function
def tracking(frame, idx, min_circle_area=3500, max_circle_area=7000, min_contour_area=1500, max_contour_area=5000):
    # Convert BGR to HSV
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    data = np.zeros(12)
    black = (0, 0, 0)

    for i in np.arange(4):
        # Extract info from the color
        color = colors[i]
        lower = color['lower']
        upper = color['upper']
        bgr = color['bgr']

        # Create mask
        mask = cv2.inRange(hsvFrame, lower, upper)

        # Erosion Kernel
        erosion_kernel = np.ones((1, 1), 'uint8')
        mask = cv2.erode(mask, erosion_kernel)

        # Dilation kernel
        dilation_kernel = np.ones((5, 5), 'uint8')
        mask = cv2.dilate(mask, dilation_kernel)

        # Creating contour to track color
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        #
        # cv2.imshow('mask', mask)
        # cv2.waitKey(0)

        # Find contour
        for pic, contour in enumerate(contours):
            contour_area = cv2.contourArea(contour)

            # Check that contour is within acceptable area range
            if (contour_area >= min_contour_area and contour_area <= max_contour_area):
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)

                circularity = 4 * np.pi * (contour_area / (cv2.arcLength(contour, True) ** 2))

                # Filter out non-circular contours based on circularity

                # Check that corresponding circle is foot sized
                circle_area = np.pi * radius**2
                # if 0.7 <= circularity <= 1.3:
                if (circle_area >= min_circle_area and circle_area <= max_circle_area):
                    # Annotate frame
                    frame = cv2.circle(frame, center, radius, bgr, 2)
                    frame = cv2.circle(frame, center, 5, black, -1)

                    # Store data for this frame and color
                    data[i*3:(i+1)*3] = np.array([int(x), int(y), radius])
                    found = True

        if not found:
            print(f"No contour found for color {color['name']} at frame {idx}")
            for pic, contour in enumerate(contours):
                contour_area = cv2.contourArea(contour)
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                circle_area = np.pi * radius ** 2
                if circle_area > 2000:
                    print(f"Circle Area {circle_area} \t\t Contour Area {contour_area}")
                    print(f"Circle Center {center} \t\t Circle Radius {radius}")
                    mask = cv2.circle(mask, center, radius, (255, 255, 255), 5)
                    cv2.imshow("mask", mask)
                    cv2.waitKey(0)

    return data, frame


# Process video with tracking
def process(savename, fourcc, size, distorted=False, show=False):
    start = time.time()

    if distorted:
        folder = "distorted"
    else:
        folder = "undistorted"

    vid = cv2.VideoCapture(f'./videos/unannotated/{folder}/{savename}.avi')

    fps = vid.get(cv2.CAP_PROP_FPS)
    num_frames = vid.get(cv2.CAP_PROP_FRAME_COUNT)
    # print(fps)

    # Video files
    ann = cv2.VideoWriter(f'./videos/annotated/{folder}/{savename}.avi', fourcc, fps, size)

    # Centers and radius data
    data_all = []

    for i in np.arange(num_frames):
        ret, frame = vid.read()
        data, frame = tracking(frame, i)

        ann.write(frame)
        data_all.append(data)

        if show:
            cv2.imshow('undistorted', frame)
            cv2.waitKey(1)

    ann.release()
    vid.release()

    data_all = np.array(data_all)
    heading = "X_D1, Y_D1, R_D1, X_D2, Y_D2, R_D2, X_D3, Y_D3, R_D3, X_D4, Y_D4, R_D4"

    np.savetxt(f"./data/pixels/{folder}/{savename}.csv", data_all, fmt='%i', delimiter=",", header=heading, comments='')

    cv2.destroyAllWindows()

    end = time.time()
    print(f"Duration {end - start} for {num_frames} frames")
    return data_all


# Undistort and process video
def process_and_undistort(savename, fourcc, size, camera_matrix, dist_matrix, new_camera_matrix, show=False):
    start = time.time()

    vid = cv2.VideoCapture(f'./videos/unannotated/distorted/{savename}.avi')
    fps = vid.get(cv2.CAP_PROP_FPS)
    num_frames = vid.get(cv2.CAP_PROP_FRAME_COUNT)
    # print(fps)

    # Video files
    unann_undist = cv2.VideoWriter(f'./videos/unannotated/undistorted/{savename}.avi', fourcc, fps, size)
    ann_undist = cv2.VideoWriter(f'./videos/annotated/undistorted/{savename}.avi', fourcc, fps, size)

    # Centers and radius data
    undist_data_all = []

    print(num_frames)
    for i in np.arange(num_frames):
        ret, frame = vid.read()
        undistorted = cv2.undistort(frame, camera_matrix, dist_matrix, None, new_camera_matrix)

        unann_undist.write(undistorted)

        undist_data, undist_frame = tracking(undistorted, i)

        ann_undist.write(undist_frame)

        undist_data_all.append(undist_data)

        if show:
            cv2.imshow('undistorted', undist_frame)
            cv2.waitKey(1)

    unann_undist.release()
    ann_undist.release()

    undist_data_all = np.array(undist_data_all)

    heading = "X_D1, Y_D1, R_D1, X_D2, Y_D2, R_D2, X_D3, Y_D3, R_D3, X_D4, Y_D4, R_D4"
    np.savetxt(f"./data/pixels/undistorted/{savename}.csv", undist_data_all, fmt='%i', delimiter=",", header=heading,
               comments='')

    cv2.destroyAllWindows()

    end = time.time()
    print(f"Duration {end - start} for {num_frames} frames")

    return undist_data_all


def correct_results(pix_data):
    err = 20
    pts = pix_data.shape[0]
    for i in np.arange(4):
        prev_x = pix_data[0, i * 3]
        prev_y = pix_data[0, i * 3 + 1]
        curr_x = pix_data[1, i * 3]
        curr_y = pix_data[1, i * 3 + 1]
        for j in np.arange(1, pts - 1):
            next_x = pix_data[j + 1, i * 3]
            next_y = pix_data[j + 1, i * 3 + 1]
            if np.abs(curr_x - prev_x) >= err:
                print(curr_x, prev_x)
                curr_x = (next_x - prev_x) / 2
                pix_data[j, i * 3] = curr_x
                print(f"corr x at idx {j}")

            if np.abs(curr_y - prev_y) >= err:
                curr_y = (next_y - prev_y) / 2
                pix_data[j, i * 3 + 1] = curr_y
                print(f"corr y at idx {j}")
            prev_x = curr_x
            prev_y = curr_y
            curr_x = next_x
            curr_y = next_y
    return pix_data


# def plot_results_filtered(data, name, pixels=False, show=False):
#     frames = data.shape[0]
#     alpha = np.linspace(0.1, 1.0, num=frames - 1)
#
#     legends = []
#     labels = []
#
#     step = 5
#
#     for i in np.arange(4):
#         c = all_colors[i]
#         pts = data[:, (i*3):(i*3)+2]
#         lines = []
#         label = f"Delta {i+1} Position"
#         count = 0
#         for j in np.arange(0, frames - step, step):
#             a = alpha[j]
#             line1, = plt.plot(pts[[j, j + step], 0], pts[[j, j + step], 1], color=c, alpha=a, linestyle="solid")
#             lines.append(line1)
#             count += 1
#
#         legends.append(lines[-1])  # res
#         labels.append(label)
#
#     plt.legend(legends, labels)
#
#     plt.xlabel(f"X ({scale_label})")
#     plt.ylabel(f"Y ({scale_label})")
#     plt.title(f"Delta Feet Positions in {scale_label}")
#
#     # savename = f"./results_julia/images/xy/xyplot_{name}.png"
#     # # plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
#     # if show:
#     plt.show()
#     # plt.close()
#     plt.close()
#
#
# def plot_results_all(savename, cm=True, show=False):
#     if cm:
#         scale_label = "CM"
#         folder = "cm"
#     else:
#         scale_label = "Pixels"
#         folder = "pixels"
#
#     data = np.loadtxt(f"./data/{folder}/{savename}.csv", delimiter=",", skiprows=1)
#
#     frames = data.shape[0]
#     alpha = np.linspace(0.1, 1.0, num=frames - 1)
#
#     legends = []
#     labels = []
#
#     step = 10
#
#     for i in np.arange(4):
#         c = all_colors[i]
#         pts = data[:, (i*2):(i*2)+2]
#         lines = []
#         label = f"Delta {i+1} Position"
#         for j in np.arange(frames - 1):
#             a = alpha[j]
#             line1, = plt.plot(pts[j:j + 2, 0], pts[j:j + 2, 1], color=c, alpha=a, marker='x', linestyle="solid")
#             lines.append(line1)
#
#         legends.append(lines[-1])  # res
#         labels.append(label)
#
#     plt.legend(legends, labels)
#
#     plt.xlabel(f"X ({scale_label})")
#     plt.ylabel(f"Y ({scale_label})")
#     plt.title(f"Delta Feet Positions in {scale_label}")
#
#     plt.savefig(f"./plots/{folder}/{savename}.png", bbox_inches="tight", pad_inches=0.05)
#     # if show:
#     #     plt.show()
#     #     plt.close()
#     plt.show()
#     plt.close()

# Convert results from pixels to mm
def convert_results():
    dist_ratio = 29.6
    undist_ratio = 29.8

    # TODO conversion from pixels to inches
    # Need to figure out location of grid center (grid 0,0 in cm) in pixels on the image
    # Need offset pixel locations to be according to the center of the grid
    # Need to scale from pixels to cm (actually want mm)
    pass


# Plot results
def plot_results(savename, undistorted=True, mm=True):
    data = np.loadtxt()
    pass


if __name__ == '__main__':
    savename = 'trajA_3x_cable_lifted'

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

    # debugging_livecam(cap, camera_matrix, dist_matrix, new_camera_matrix)


    # undistort(savename, fourcc, size, camera_matrix, dist_matrix, new_camera_matrix, show=False)
    process(savename, fourcc, size, distorted=False, show=False)

    # process(savename, fourcc, size, camera_matrix, dist_matrix, new_camera_matrix, show=False)

    # debugging_livecam(cap, camera_matrix, dist_matrix, new_camera_matrix)

    # Undistort = 163.27s
    # Process undistorted = 122.56 erode 1,1 dilate 5,5




