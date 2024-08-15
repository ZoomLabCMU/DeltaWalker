import cv2
import numpy as np
import time
import sys


green = {'name': 'green',
         'lower': np.array([50, 125, 70], np.uint8),
         'upper': np.array([100, 255, 255], np.uint8),
         'bgr': (0, 255, 0)}

red = {'name': 'red',
       'lower': np.array([140, 100, 150], np.uint8),  # 170, 120, 70
       'upper': np.array([180, 255, 255], np.uint8),
       'bgr': (0, 0, 255)}

orange = {'name': 'orange',
          'lower': np.array([0, 150, 150], np.uint8),
          'upper': np.array([25, 255, 255], np.uint8),
          'bgr': (0, 128, 255)}

orange = {'name': 'orange',
          'lower': np.array([0, 120, 150], np.uint8),  # 0, 150, 150
          'upper': np.array([25, 255, 255], np.uint8),
          'bgr': (0, 128, 255)}

blue = {'name': 'blue',
        'lower': np.array([100, 100, 50], np.uint8),
        'upper': np.array([120, 255, 255], np.uint8),
        'bgr': (255, 0, 0)}

all_colors = [green, red, orange, blue]


def speedy_ann(savename, folder, speed, show=False):
    vid = cv2.VideoCapture(f'./{folder}/videos/adjusted/{savename}.mp4')
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    fps = vid.get(cv2.CAP_PROP_FPS) * speed
    # num_frames = vid.get(cv2.CAP_PROP_FRAME_COUNT)
    w = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

    size = (w, h)

    pix_data = np.loadtxt(f"./{folder}/data/pixels/{savename}.csv", dtype="int", delimiter=",", skiprows=1)
    num_frames = pix_data.shape[0]
    first_loc = pix_data[0, :]
    last_loc = pix_data[-1, :]
    first_centers = [(first_loc[0], first_loc[1]), (first_loc[2], first_loc[3]), (first_loc[4], first_loc[5]), (first_loc[6], first_loc[7])]
    last_centers = [(last_loc[0], last_loc[1]), (last_loc[2], last_loc[3]), (last_loc[4], last_loc[5]), (last_loc[6], last_loc[7])]

    # Video file
    adj = cv2.VideoWriter(f'./{folder}/videos/speedy_ann/{savename}_{speed}x_speed.mp4', fourcc, fps, size)

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
        for i in range(4):
            bgr = all_colors[i]['bgr']
            first_center = first_centers[i]
            last_center = last_centers[i]
            frame = cv2.line(frame, first_center, last_center, bgr, 7)
            frame = cv2.circle(frame, first_center, 11, bgr, -1)
            frame = cv2.circle(frame, last_center, 11, bgr, -1)
            frame = cv2.line(frame, first_center, last_center, text_color_bg, 2)
            frame = cv2.circle(frame, first_center, 5, text_color_bg, -1)
            frame = cv2.circle(frame, last_center, 5, text_color_bg, -1)
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

        # trials = [1]
        # orientations = [1]
        # traj_labels = ['E']
        # versions = ['a']
        # iters = 3


    print(folder)

    speed = 4

    show = False
    debug = False
    if debug:
        traj_label = "A"
        version = "a"
        trial = "1"
        folder = "1_ocrl_trials"
        savename = f"traj{traj_label}{version}_t{trial}_o1_3x"
        show = True
        speedy_ann(savename, folder, fourcc, show=show)

    else:
        for traj_label in traj_labels:
            for version in versions:
                for orientation in orientations:
                    for trial in trials:
                        savename = f"traj{traj_label}{version}_t{trial}_o{orientation}_{iters}x"
                        print(savename)
                        speedy_ann(savename, folder, speed, show=show)

