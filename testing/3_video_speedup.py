import time
from processing_helpers import *
import sys
import matplotlib.pyplot as plt


def speedup(savename, folder, speed, show=False):
    vid = cv2.VideoCapture(f'./{folder}/videos/adjusted/{savename}.mp4')
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    fps = vid.get(cv2.CAP_PROP_FPS) * speed
    num_frames = vid.get(cv2.CAP_PROP_FRAME_COUNT)
    w = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

    size = (w, h)

    # Video file
    adj = cv2.VideoWriter(f'./{folder}/videos/speedy/{savename}_{speed}x_speed.mp4', fourcc, fps, size)

    start = time.time()

    text = f"{speed}X Speed"
    font = cv2.FONT_HERSHEY_DUPLEX
    pos = (50, 50)
    font_scale = 2
    font_thickness = 3
    text_color = (255, 255, 255)
    text_color_bg = (0, 0, 0)
    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    pad = 20

    for _ in np.arange(num_frames):
        ret, frame = vid.read()

        cv2.rectangle(frame, (x-pad, y-pad), (x + text_w + pad, y + text_h + pad), text_color_bg, -1)
        cv2.putText(frame, text, (x, y + text_h + font_scale - 1), font, font_scale, text_color, font_thickness)

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
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')

    folders = ["0_manual_design", "1_ocrl_trials", "2_ocrl_variations", "3_ocrl_further_studies"]
    folder_int = int(sys.argv[1])
    folder = folders[folder_int]

    # if folder_int == 1:
    #     exit()

    if folder_int == 0:
        trials = [1]
        orientations = [1, 2]
        traj_labels = ["A", "B", "C"]
        versions = ["0"] #, "1", "2"]
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

        trials = [1]
        orientations = [1]
        traj_labels = ['E']
        versions = ['a']
        iters = 3


    print(folder)

    speed = 4

    show = False
    debug = False
    if debug:
        traj_label = "A"
        version = "a"
        trial = "1"
        folder = "ocrl_trials"
        savename = f"traj{traj_label}{version}_t{trial}_o1_3x"
        show = True
        speedup(savename, folder, fourcc, show=show)

    else:
        for traj_label in traj_labels:
            for version in versions:
                for orientation in orientations:
                    for trial in trials:
                        savename = f"traj{traj_label}{version}_t{trial}_o{orientation}_{iters}x"
                        print(savename)
                        speedup(savename, folder, speed, show=show)
