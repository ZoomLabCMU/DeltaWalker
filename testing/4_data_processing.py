import numpy as np
import matplotlib.pyplot as plt
import sys


# take in the data
# take in the savename
# interpolate between prev and next values for zero values
# if next value is zero then go onto next
# convert the data to mm scale
# plot results

all_colors = ['tab:green', 'tab:red', 'tab:orange', 'tab:blue', 'tab:purple']


def convert(savename, folder):
    pix_data = np.loadtxt(f"./{folder}/data/pixels/{savename}.csv", dtype="int", delimiter=",", skiprows=1)

    x_idx = np.arange(0, 8, 2)
    y_idx = x_idx + 1

    pix_data[:, y_idx]

    # Ratio is pixels/cm
    ratio = 29.8
    center_x = 820 / ratio
    center_y = 360 / ratio

    real_data = pix_data / ratio

    real_data[:, x_idx] -= center_x
    real_data[:, y_idx] -= center_y
    real_data[:, y_idx] *= -1
    # TODO flip the y in pixels and update real accordingly

    real_data = np.round(real_data, decimals=1)

    max_movement = np.zeros((5, 3))

    avg_x = 0
    avg_y = 0
    avg_dist = 0

    for i in np.arange(4):
        pos_x = real_data[:, (i*2)]
        pos_y = real_data[:, (i*2)+1]

        dx = np.max(pos_x) - np.min(pos_x)
        dy = np.max(pos_y) - np.min(pos_y)
        dist = np.sqrt(dx**2 + dy**2)
        max_movement[i, :] = np.array([dx, dy, dist])
        avg_x += dx
        avg_y += dy
        avg_dist += dist

    avg_x /= 4
    avg_y /= 4
    avg_dist /= 4

    max_movement[4, :] = np.round(np.array([avg_x, avg_y, avg_dist]), 1)

    heading = "X_D1, Y_D1, X_D2, Y_D2, X_D3, Y_D3, X_D4, Y_D4"
    np.savetxt(f"./{folder}/data/cm/{savename}.csv", real_data, delimiter=",", fmt="%.2f", header=heading, comments='')
    return real_data, max_movement


if __name__ == "__main__":
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

    print(folder)
    max_movements = []
    all_names = []
    for traj_label in traj_labels:
        for version in versions:
            for orientation in orientations:
                for trial in trials:
                    savename = f"traj{traj_label}{version}_t{trial}_o{orientation}_{iters}x"
                    print(savename)
                    _, max_movement = convert(savename, folder)
                    max_movements.append(max_movement.flatten())
                    all_names.append(savename)

    heading = "traj, dx1, dy1, d1, dx2, dy2, d2, dx3, dy3, d3, dx4, dy4, d4, avgx, avgy, avgd"
    max_movements = np.round(np.array(max_movements), 1)
    all_names = np.array(all_names).reshape((-1, 1))
    max_movements = np.hstack((all_names, max_movements))
    np.savetxt(f"./{folder}/data/cm/max_movements.csv", max_movements, delimiter=",", fmt="%s", header=heading,
               comments='')

