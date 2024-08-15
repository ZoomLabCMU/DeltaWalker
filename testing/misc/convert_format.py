import time
from processing_helpers import *
import sys
import matplotlib.pyplot as plt

# convert from avi to mp4


def convert(savename, show=False):
    vid = cv2.VideoCapture(f'./videos/adjusted/{savename}.avi')
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    fps = vid.get(cv2.CAP_PROP_FPS)
    num_frames = vid.get(cv2.CAP_PROP_FRAME_COUNT)
    w = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

    size = (w, h)

    # Video file
    adj = cv2.VideoWriter(f'./videos/adjusted/{savename}.mp4', fourcc, fps, size)

    start = time.time()

    for _ in np.arange(num_frames):
        ret, frame = vid.read()
        adj.write(frame)
        if show:
            cv2.imshow("frame", frame)
            cv2.waitKey(1)

    adj.release()
    vid.release()
    cv2.destroyAllWindows()

    end = time.time()
    print(f"Duration {end - start} for {num_frames} frames")
    return


if __name__ == '__main__':
    traj_label = sys.argv[1]
    versions = ["a", "b", "c", "d"]
    trials = [1, 2, 3]

    for version in versions:
        for trial in trials:
            savename = f"traj{traj_label}{version}_t{trial}_o1_3x"
            show_str = "f"

            true_list = ["True", "true", "T", "t"]

            if show_str in true_list:
                show = True
            else:
                show = False

            print(savename)
            convert(savename, show=show)

