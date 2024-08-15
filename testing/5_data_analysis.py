import numpy as np
import sys

# data: X_D1, Y_D1, X_D2, Y_D2, X_D3, Y_D3, X_D4, Y_D4
# Shape num_steps x 8
# Columns   +x, -x, dx, +y, -y, dy, dist, angle, percent error for distance
# Rows      delta 1, delta 2, delta 3, delta 4, average


def analyze(savename, folder, exp):
    data = np.loadtxt(f"./{folder}/data/cm/{savename}.csv", delimiter=",", skiprows=1)
    movements = np.zeros((4, 9))
    for i in np.arange(4):
        xs = data[:, i*2]
        ys = data[:, i*2 + 1]
        x0 = xs[0]
        y0 = ys[0]
        xp = max(xs) - x0   # how much x moved in the pos direction relative to its start
        xn = x0 - min(xs)   # how much x moved in the neg direction relative to its start
        yp = max(ys) - y0   # how much y moved in the pos direction relative to its start
        yn = y0 - min(ys)   # how much y moved in the neg direction relative to its start
        dx = xs[-1] - x0       # total x movement
        dy = ys[-1] - y0        # total y movement
        # dx = max(xs) - min(xs)  # total x movement
        # dy = max(ys) - min(ys)  # total y movement
        d = np.sqrt(xp**2 + yp**2)
        angle = np.rad2deg(np.arctan2(yp, xp))
        err = (d - exp) / exp * 100
        movements[i, :] = np.array([xp, xn, dx, yp, yn, dy, d, angle, err])

    average = np.average(movements, axis=0)
    results = np.vstack((movements, average))
    heading = "+x, -x, dx, +y, -y, dy, dist, angle, % err"
    np.savetxt(f"./{folder}/data/movements/{savename}.csv", results, delimiter=",", fmt="%.2f", header=heading, comments='')
    return movements, average


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
        anticipated_movement = [3, 3, 3]

    elif folder_int == 1:
        trials = [1, 2, 3]
        orientations = [1]
        iters = 3
        traj_labels = ["A", "B", "C", "D"]
        versions = ["a", "b", "c", "d"]
        anticipated_movement = [3, 4.5, 6, 7.5]

    elif folder_int == 2:
        trials = [1]
        orientations = [1]
        traj_labels = ["A", "B", "C", "D"]
        versions = ["1a", "2a", "3a", "4a"]
        iters = 3
        anticipated_movement = [3, 3, 3, 3]

    elif folder_int == 3:
        trials = [1, 2, 3]
        orientations = [1]
        traj_labels = ["A", "C"]
        versions = ["e"]
        iters = 1
        anticipated_movement = [6]

    print(folder)
    traj_names = []
    traj_avgs = []
    for traj_label in traj_labels:
        for i, version in enumerate(versions):
            exp = anticipated_movement[i]
            for orientation in orientations:
                trial_avgs = np.zeros((max(trials), 9))
                for trial in trials:
                    savename = f"traj{traj_label}{version}_t{trial}_o{orientation}_{iters}x"
                    print(savename)
                    movements, average = analyze(savename, folder, exp)
                    trial_avgs[trial - 1, :] = average
                traj_names.append(f"Traj{traj_label}{version}_o{orientation}")
                traj_avgs.append(np.round(np.average(trial_avgs, axis=0), 1))

    heading = "Traj, +x, -x, dx, +y, -y, dy, dist, angle, % err"
    traj_names = np.array(traj_names).reshape((-1, 1))
    traj_avgs = np.array(traj_avgs)

    traj_results = np.hstack((traj_names, traj_avgs))

    np.savetxt(f"./{folder}/data/movements/trajectory_averages.csv", traj_results, delimiter=",", fmt="%s", header=heading,
               comments='')
