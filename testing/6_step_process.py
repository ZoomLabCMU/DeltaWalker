import numpy as np
import matplotlib.pyplot as plt
import cv2


all_colors = ['tab:green', 'tab:red', 'tab:orange', 'tab:blue', 'tab:purple']


def get_points(savename, folder, id, version, fps, step=12):
    if id == "A":
        order = [3, 2, 4, 1]
    elif id == "B":
        order = [3, 1, 4, 2]
    elif id == "C":
        order = [1, 2, 4, 3]
    elif id == "D":
        order = [1, 2, 3, 4]

    if version == "a":
        step_goal = 1.0
    elif version == "b":
        step_goal = 1.5
    elif version == "c":
        step_goal = 2.0
    elif version == "d":
        step_goal = 2.5

    order.reverse()
    print(savename)

    data = np.loadtxt(f"./{folder}/data/cm/{savename}.csv", dtype="float", delimiter=",", skiprows=1)
    total_steps = data.shape[0]
    plot_idx = np.arange(0, total_steps - step, step)
    num_frames = len(plot_idx)
    time_step = np.arange(total_steps)
    alpha = np.linspace(0.25, 1.0, num=num_frames)

    legends = []
    labels = []
    plt.figure(figsize=(10, 6))

    for i in np.arange(4):
        if i == order[0] - 1 or i == order[-1] - 1:
            c = all_colors[i]
            pts = data[:, (i*2):(i*2)+2]
            lines = []
            label = f"Delta {i+1} Position"
            for a, j in enumerate(plot_idx):
                # line1, = plt.plot(pts[[j, j + step], 0], pts[[j, j + step], 1], color=c, alpha=alpha[a], linestyle="solid", marker='.')
                line1, = plt.plot(time_step[[j, j+step]], pts[[j, j + step], 0], color=c, alpha=alpha[a], linestyle="solid") #, marker='.')
                lines.append(line1)

            legends.append(lines[-1])
            labels.append(label)

    plt.legend(legends, labels)

    plt.xlabel(f"Timestep")
    plt.ylabel(f"X (cm)")
    plt.title(f"{savename}: Last delta to move is {order[0]}")
    plt.grid(which='major', alpha=0.75)
    plt.grid(which='minor', alpha=0.25)
    plt.minorticks_on()

    plt.waitforbuttonpress()
    num_pts = 2
    timesteps = [0]

    while True:
        pts = np.asarray(plt.ginput(1, timeout=-1))
        timestep = int(pts[0][0])
        timesteps.append(timestep)
        plt.axvline(x=timestep, c=all_colors[order[0]-1])
        plt.pause(0.05)
        if (len(timesteps) - 1 == num_pts):
            plt.close()
            break

    plt.show()
    # plt.close()

    timesteps.append(total_steps - 1)
    times = np.round(np.array(timesteps) / fps, 1)
    step_points = data[timesteps, :]

    full_results = analyze_points(savename, times, step_points, step_goal)
    return full_results


def analyze_points(savename, times, step_points, step_goal):
    # step points is size 4 x 8 (4 timestep indices x 8 delta x y positions)
    results = []
    for i in np.arange(3):
        step_results = np.array([i + 1])
        avg_x = 0
        avg_y = 0
        avg_d = 0
        avg_a = 0
        avg_e = 0
        for j in np.arange(4):
            dx = step_points[i+1, j*2] - step_points[i, j*2]
            dy = step_points[i+1, j*2 + 1] - step_points[i, j*2 + 1]
            d = np.sqrt(dx**2 + dy**2)
            a = np.rad2deg(np.arctan2(dy, dx))
            e = (d - step_goal) / step_goal * 100
            avg_x += dx
            avg_y += dy
            avg_d += d
            avg_a += a
            avg_e += e
            delta_results = np.array([dx, dy, d, a, e])
            step_results = np.append(step_results, delta_results)

        step_averages = np.array([avg_x, avg_y, avg_d, avg_a, avg_e])/4
        step_results = np.append(step_results, step_averages)
        dt = np.array([times[i + 1] - times[i]])
        step_results = np.append(step_results, dt)
        results.append(step_results)

    results = np.array(results)
    average = np.average(results, axis=0)
    average[0] = 0
    full_results = np.vstack((results, average))
    full_results = np.round(full_results, 1)

    heading = "step, dx1, dy1, d1, a1, %e1, dx2, dy2, d2, a2, %e2, dx3, dy3, d3, a3, %e3, dx4, dy4, d4, a4, %e4, avgx, avgy, avgd, avga, avg%e, time"

    np.savetxt(f"./{folder}/data/steps/{savename}.csv", full_results, delimiter=",", fmt="%.1f", header=heading, comments='')
    return full_results


def trial_averages(traj, folder):
    trials = []
    for i in np.arange(3):
        data = np.loadtxt(f"./{folder}/data/steps/{traj}_t{i+1}_o1_3x.csv", dtype="float", delimiter=",", skiprows=1)
        data_avgs = data[-1, 1:]
        avg_vel = data_avgs[-4] * 10 / data_avgs[-1]
        data_avgs = np.append(data_avgs, avg_vel)
        trials.append(data_avgs)

    trials = np.array(trials)
    averages = np.round(np.average(trials, axis=0), 1)

    return averages


if __name__ == "__main__":
    traj_ids = ["A", "B", "C", "D"]
    traj_versions = ["a", "b", "c", "d"]
    trial_nums = [1, 2, 3]

    all_fps = []

    folder = "1_ocrl_trials"

    for id in traj_ids:
        for i, version in enumerate(traj_versions):
            # trial_avgs = np.zeros((max(trial_nums), 9))
            for trial in trial_nums:
                file = f"Traj{id}{version}_t{trial}_o1_3x"
                vid = cv2.VideoCapture(f'./{folder}/videos/annotated/{file}.mp4')
                fps = vid.get(cv2.CAP_PROP_FPS)
                vid.release()
                cv2.destroyAllWindows()
                step = int(fps)
                all_fps.append(fps)
                full_results = get_points(file, folder, id, version, fps, step=step)

    all_fps = np.array(all_fps)
    print(all_fps)
    print(np.average(all_fps))

    '''
    traj_ids = ["A", "B", "C", "D"]
    traj_versions = ["a", "b", "c", "d"]
    traj_names = []
    traj_avgs = []
    for id in traj_ids:
        for version in traj_versions:
            traj = f"Traj{id}{version}"
            avg = trial_averages(traj, folder)
            traj_names.append(traj)
            traj_avgs.append(avg)

    traj_names = np.array(traj_names).reshape((-1, 1))
    traj_avgs = np.array(traj_avgs)

    results = np.hstack((traj_names, traj_avgs))
    heading = "traj, dx1, dy1, d1, a1, %e1, dx2, dy2, d2, a2, %e2, dx3, dy3, d3, a3, %e3, dx4, dy4, d4, a4, %e4, avgx, avgy, avgd, avga, avg%e, time, avgvel mm/s"
    np.savetxt(f"./{folder}/data/steps/step_averages.csv", results, delimiter=",", fmt="%s", header=heading, comments='')
    '''


