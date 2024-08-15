import time
from processing_helpers import *
import sys
import matplotlib.pyplot as plt


# Undistort video
def undistort(savename, folder, fourcc, camera_matrix, dist_matrix, new_camera_matrix, H, show=False, debug=False):
    start = time.time()

    vid = cv2.VideoCapture(f'./{folder}/videos/raw/{savename}.avi')
    fps = vid.get(cv2.CAP_PROP_FPS)
    num_frames = vid.get(cv2.CAP_PROP_FRAME_COUNT)
    # print(fps)

    crop_x1 = 159
    crop_x2 = 1770
    crop_y1 = 196
    crop_y2 = 923
    size = (crop_x2 - crop_x1, crop_y2 - crop_y1) # Image size 1611, 727

    if not debug:
        # Video file
        adj = cv2.VideoWriter(f'./{folder}/videos/adjusted/{savename}.mp4', fourcc, fps, size)

    if debug:
        num_frames = 1

    for _ in np.arange(num_frames):
        ret, frame = vid.read()
        undistorted = cv2.undistort(frame, camera_matrix, dist_matrix, None, new_camera_matrix)
        unwarped = cv2.warpPerspective(undistorted, H, (undistorted.shape[1], undistorted.shape[0]))
        cropped = unwarped[crop_y1:crop_y2, crop_x1:crop_x2]

        if debug:
            plt.imshow(cropped)
            plt.show()
            plt.close()
        else:
            adj.write(cropped)

        if show:
            cv2.imshow("adjusted", cropped)
            cv2.waitKey(1)

    if not debug:
        adj.release()
    vid.release()
    cv2.destroyAllWindows()

    end = time.time()
    print(f"Duration {end - start} for {num_frames} frames")
    return


if __name__ == '__main__':
    w = 1920
    h = 1080
    size = (w, h)
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')

    # Get camera matrix
    camera_matrix, dist_matrix = load_coefficients('camera.yml')
    # Update camera matrix
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, size, 1, size)

    corners1 = np.float32([[130, 106], [130, 1018], [1800, 1007], [1800, 123]]).reshape(-1, 1, 2)
    corners2 = np.float32([[130, 106], [130, 1018], [1800, 1018], [1800, 106]]).reshape(-1, 1, 2)
    H, _ = cv2.findHomography(corners1, corners2)

    folders = ["0_manual_design", "1_ocrl_trials", "2_ocrl_variations", "3_ocrl_further_studies"]
    folder_int = int(sys.argv[1])
    folder = folders[folder_int]

    if folder_int == 0:
        trials = [1]
        orientations = [1, 2]
        traj_labels = ["A", "B", "C"]
        versions = ["0", "1", "2"]
        iters = 3

    elif folder_int == 1:
        trials = [1, 2, 3]
        orientations = [1]
        iters = 3
        traj_labels = ["A", "B", "C", "D"]
        versions = ["a", "b", "c", "d"]

    elif folder_int == 2:
        trials = [1]
        orientations = [1]
        traj_labels = ["A", "B", "C", "D"]
        versions = ["1a", "2a", "3a", "4a"]
        iters = 3

    elif folder_int == 3:
        # trials = [1, 2, 3]
        # orientations = [1]
        # traj_labels = ["C"]#["A", "C"]
        # versions = ["e"]
        # iters = 1
        #
        trials = [1]
        orientations = [1]
        traj_labels = ['E']
        versions = ['a']
        iters = 3

    print(folder)

    show = False
    debug = False
    if debug:
        traj_label = "A"
        version = "a"
        trial = "1"
        folder = "ocrl_trials"
        savename = f"traj{traj_label}{version}_t{trial}_o1_3x"
        show = True
        undistort(savename, folder, fourcc, camera_matrix, dist_matrix, new_camera_matrix, H, show=show)

    else:
        for traj_label in traj_labels:
            for version in versions:
                for orientation in orientations:
                    for trial in trials:
                        savename = f"traj{traj_label}{version}_t{trial}_o{orientation}_{iters}x"
                        print(savename)
                        undistort(savename, folder, fourcc, camera_matrix, dist_matrix, new_camera_matrix, H, show=show)
        # savename = "demo"
        # undistort(savename, folder, fourcc, camera_matrix, dist_matrix, new_camera_matrix, H, show=show)