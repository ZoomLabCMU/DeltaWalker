import time
from processing_helpers import *
import sys


def process(savename, folder, fourcc, show=False):
    start = time.time()

    vid = cv2.VideoCapture(f'./{folder}/videos/trimmed/{savename}.mp4')

    fps = vid.get(cv2.CAP_PROP_FPS)
    num_frames = vid.get(cv2.CAP_PROP_FRAME_COUNT)
    w = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
    size = (w, h)
    # print(fps)

    # Video files
    ann = cv2.VideoWriter(f'./{folder}/videos/annotated/{savename}.mp4', fourcc, fps, size)

    # Centers and radius data
    data_all = []

    for i in np.arange(num_frames):
        ret, frame = vid.read()
        if frame is None:
            print(f"Stopped on frame {i}, out of total {num_frames} frame")
            break
        data, frame = tracking(frame, i)

        ann.write(frame)
        data_all.append(data)

        if show:
            cv2.imshow('undistorted', frame)
            cv2.waitKey(1)

    ann.release()
    vid.release()

    data_all = np.array(data_all)
    heading = "X_D1, Y_D1, X_D2, Y_D2, X_D3, Y_D3, X_D4, Y_D4"

    np.savetxt(f"./{folder}/data/pixels/{savename}.csv", data_all, fmt='%i', delimiter=",", header=heading, comments='')

    cv2.destroyAllWindows()

    end = time.time()
    print(f"Duration {end - start} for {num_frames} frames")
    return data_all


if __name__ == "__main__":
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')

    folders = ["0_manual_design", "1_ocrl_trials", "2_ocrl_variations", "3_ocrl_further_studies"]
    folder_int = int(sys.argv[1])
    folder = folders[folder_int]

    if folder_int == 1:
        exit()

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
        trials = [1, 2, 3]
        orientations = [1]
        traj_labels = ["A", "C"]
        versions = ["e"]
        iters = 1

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
        process(savename, folder, fourcc, show=show)

    else:
        for traj_label in traj_labels:
            for version in versions:
                for orientation in orientations:
                    for trial in trials:
                        savename = f"traj{traj_label}{version}_t{trial}_o{orientation}_{iters}x"
                        print(savename)
                        process(savename, folder, fourcc, show=show)
