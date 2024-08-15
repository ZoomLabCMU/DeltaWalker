import math
import numpy as np
import matplotlib.pyplot as plt

PI = math.pi
all_colors = ['tab:green', 'tab:red', 'tab:orange', 'tab:blue', 'tab:purple']
plt.rc('text', usetex=True)
plt.rc('font', family='serif', size=15, weight='bold')
plt.rcParams['text.latex.preamble'] = r'\boldmath'


def plot_both(save_name, title, st, des_com, des_ee, res_com, res_ee, steps):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    legends = []
    labels = []

    for i in np.arange(5):
        c = all_colors[i]

        if i < 4:
            res = res_ee[(i * 3):(i * 3) + 3, :]
            des = des_ee[(i * 3):(i * 3) + 3, :]
            key = f"Foot {i + 1}"
        else:
            res = res_com
            des = des_com
            key = "Body COM"

        res_label = r"$\textbf{Simulated Position}$" + "\n" + r"$\textbf{" + key + r"}$"
        des_label = r"$\textbf{Desired Position}$" + "\n" + r"$\textbf{" + key + r"}$"

        lines = []

        for j in np.arange(steps - 1):
            line1_res, = ax.plot(res[0, j:j + 2], res[1, j:j + 2], zs=res[2, j:j + 2], color=c,
                                 linestyle="solid")
            lines.append(line1_res)

            line1_des, = ax.plot(des[0, j:j + 2], des[1, j:j + 2], zs=des[2, j:j + 2], color=c,
                                 linestyle="dotted")
            lines.append(line1_des)

        legends.append(lines[-2])  # res
        legends.append(lines[-1])  # des
        labels.append(res_label)  # res
        labels.append(des_label)  # des

    ax.set_xlabel(r"$\textbf{X (cm)}$", labelpad=8)
    ax.set_ylabel(r"$\textbf{Y (cm)}$", labelpad=8)
    ax.set_zlabel(r"$\textbf{Z (cm)}$", labelpad=8)
    ax.set_xlim3d([-6, 6])
    ax.set_ylim3d([-6, 6])
    ax.set_zlim3d([0, 5])
    ax.set_title(r"$\textbf{" + title + "}$" + "\n" + r"$\textbf{" + st + "}$")

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    # ax.legend(legends, labels, loc='upper left', bbox_to_anchor=(1.2, 1.0), ncol=1)
    plt.subplots_adjust(left=0.1, right=0.9)

    savename = f"./results/plots/both_traj/{save_name}.png"
    # fig.set_size_inches(5, 4.5)
    # plt.savefig(savename)
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.5)
    # if show:
    # plt.show()
    plt.close()


def plot_og(save_name, title, st, des_com, des_ee, steps):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    legends = []
    labels = []

    for i in np.arange(4):
        c = all_colors[i]

        if i < 4:
            des = des_ee[(i * 3):(i * 3) + 3, :]
            key = f"Foot {i + 1}"
        else:
            des = des_com
            key = "Body COM"

        des_label = r"$\textbf{Desired Position}$" + "\n" + r"$\textbf{" + key + r"}$"

        lines = []

        for j in np.arange(steps - 1):
            line1_des, = ax.plot(des[0, j:j + 2], des[1, j:j + 2], zs=des[2, j:j + 2], color=c,
                                 linestyle="solid")
            lines.append(line1_des)

        legends.append(lines[-1])  # des
        labels.append(des_label)  # des

    ax.set_xlabel(r"$\textbf{X (cm)}$")
    ax.set_ylabel(r"$\textbf{Y (cm)}$")
    ax.set_zlabel(r"$\textbf{Z (cm)}$")
    ax.set_xlim3d([-6, 6])
    ax.set_ylim3d([-6, 6])
    ax.set_zlim3d([0, 1])
    ax.set_title(r"$\textbf{" + title + "}$" + "\n" + r"$\textbf{" + st + "}$")

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    ax.legend(legends, labels, loc='upper left', bbox_to_anchor=(1.2, 0.75), ncol=1)

    savename = f"./results/plots/og_traj/{save_name}.png"
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
    # if show:
    # plt.show()
    plt.close()


def plot_sim(save_name, title, st, res_com, res_ee, steps):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    legends = []
    labels = []

    for i in np.arange(4):
        c = all_colors[i]

        if i < 4:
            res = res_ee[(i * 3):(i * 3) + 3, :]
            key = f"Foot {i + 1}"
        else:
            res = res_com
            key = "Body COM"

        des_label = r"$\textbf{Simulated Position}$" + "\n" + r"$\textbf{" + key + r"}$"

        lines = []

        for j in np.arange(steps - 1):
            line1_des, = ax.plot(res[0, j:j + 2], res[1, j:j + 2], zs=res[2, j:j + 2], color=c,
                                 linestyle="solid")
            lines.append(line1_des)

        legends.append(lines[-1])  # des
        labels.append(des_label)  # des

    ax.set_xlabel(r"$\textbf{X (cm)}$")
    ax.set_ylabel(r"$\textbf{Y (cm)}$")
    ax.set_zlabel(r"$\textbf{Z (cm)}$")
    ax.set_xlim3d([-6, 6])
    ax.set_ylim3d([-6, 6])
    ax.set_zlim3d([0, 1])
    ax.set_title(r"$\textbf{" + title + "}$" + "\n" + r"$\textbf{" + st + "}$")

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    ax.legend(legends, labels, loc='upper left', bbox_to_anchor=(1.2, 0.75), ncol=1)

    savename = f"./results/plots/sim_traj/{save_name}.png"
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
    # if show:
    # plt.show()
    plt.close()


def plot_sim_wcom(save_name, title, st, res_com, res_ee, steps):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    legends = []
    labels = []

    for i in np.arange(5):
        c = all_colors[i]

        if i < 4:
            res = res_ee[(i * 3):(i * 3) + 3, :]
            key = f"Foot {i + 1}"
        else:
            res = res_com
            key = "Body COM"

        des_label = r"$\textbf{Simulated Position}$" + "\n" + r"$\textbf{" + key + r"}$"

        lines = []

        for j in np.arange(steps - 1):
            line1_des, = ax.plot(res[0, j:j + 2], res[1, j:j + 2], zs=res[2, j:j + 2], color=c,
                                 linestyle="solid")
            lines.append(line1_des)

        legends.append(lines[-1])  # des
        labels.append(des_label)  # des

    ax.set_xlabel(r"$\textbf{X (cm)}$")
    ax.set_ylabel(r"$\textbf{Y (cm)}$")
    ax.set_zlabel(r"$\textbf{Z (cm)}$")
    ax.set_xlim3d([-6, 6])
    ax.set_ylim3d([-6, 6])
    ax.set_zlim3d([0, 6])
    ax.set_title(r"$\textbf{" + title + "}$" + "\n" + r"$\textbf{" + st + "}$")

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    ax.legend(legends, labels, loc='upper left', bbox_to_anchor=(1.2, 0.75), ncol=1)

    savename = f"./results/plots/sim_traj/{save_name}_wcom.png"
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
    # if show:
    # plt.show()
    plt.close()


def get_mse(des_com, des_ee, res_com, res_ee):
    mse = np.zeros(5)
    for i in np.arange(4):
        ee_res = res_ee[i * 3:(i + 1) * 3, :]
        ee_des = des_ee[i * 3:(i + 1) * 3, :]
        all_mse = np.linalg.norm(ee_des - ee_res, axis=0)
        mse[i] = np.average(all_mse)
    mse[4] = np.average(mse[0:3])
    mse = np.round(mse * 10, 2)
    print(f"{mse[0]} & {mse[1]} & {mse[2]} & {mse[3]} & net mse {mse[4]}")
    return mse


def save_errs(des_com, des_ee, res_com, res_ee):
    mpe = np.zeros(5)

    close_zero = 0.5
    for i in np.arange(des_com.shape[0]):
        zeros_com = np.where(np.any(des_com[i, :] == 0))
        des_com[i, zeros_com] = close_zero

    for i in np.arange(des_ee.shape[0]):
        zeros_ee = np.where(np.any(des_ee[i, :] == 0))
        des_ee[i, zeros_ee] = close_zero

    # print(res_ee)
    print(des_ee)

    for i in np.arange(5):
        if i < 4:
            ee_res = res_ee[i * 3:(i + 1) * 3, :]
            ee_des = des_ee[i * 3:(i + 1) * 3, :]
            all_mpe = (ee_des - ee_res) / (ee_des)
        else:
            all_mpe = (des_com - res_com) / (des_com)
        mpe[i] = np.average(all_mpe * 100)
    mpe = np.round(mpe, 1)
    print(f"{mpe[4]} & {mpe[0]} & {mpe[1]} & {mpe[2]} & {mpe[3]}")
    return mpe


def update_des(des_data_all):
    z_off = (np.min(des_data_all[5, :])) * 100
    offset = np.array([0, 0, z_off]).reshape((3, 1))
    des_com = des_data_all[0:3, :] * 100 - offset
    des_ee = des_data_all[3:15, :] * 100 - np.vstack((offset, offset, offset, offset))

    des_ee[[0, 4], :] += 0.04 * 100
    des_ee[[6, 10], :] -= 0.04 * 100

    return des_com, des_ee


def get_dist(res_com, res_ee):
    com_dx = np.max(res_com[0, :]) - np.min(res_com[0, :])
    com_dy = np.max(res_com[1, :]) - np.min(res_com[1, :])
    com_d = np.sqrt(com_dx**2 + com_dy**2)
    print(com_d)


if __name__ == "__main__":
    traj_labels = ["A", "B", "C"]
    gait_names = ["Walk ", "Amble ", "S-"]
    gait_abbr = ["walk", "ambl", "mtri", "adjs"]
    traj_versions = ["0", "1", "2"]
    step_sizes = ["1.0 cm", "1.0 cm", "1.0 cm"]
    orientations = [1]

    scale_conversion = 100

    mse_header = "d1, d2, d3, d4, avg"
    for i, label in enumerate(traj_labels):
        for j, version in enumerate(traj_versions):
            for orientation in orientations:
                save_name = f"traj{label}{version}_o{orientation}"
                title = f"{gait_names[i]}Gait Version {version}"
                st = f"Step Size {step_sizes[j]}"
                des_path = f'./results/gen_traj/traj{label}{version}_o{orientation}.csv'
                res_path = f'./results/positions/traj{label}{version}_o{orientation}.csv'

                print(title)

                des_data_all = np.loadtxt(des_path, skiprows=1, delimiter=',')
                des_com, des_ee = update_des(des_data_all)

                res_data_all = np.loadtxt(res_path, skiprows=1, delimiter=',')
                res_com = res_data_all[0:3, :] * 100
                res_ee = res_data_all[3:15, :] * 100

                steps = des_com.shape[1]

                plot_both(save_name, title, st, des_com, des_ee, res_com, res_ee, steps)
                plot_og(save_name, title, st, des_com, des_ee, steps)
                plot_sim(save_name, title, st, res_com, res_ee, steps)
                plot_sim_wcom(save_name, title, st, res_com, res_ee, steps)
                # save_errs(des_com, des_ee, res_com, res_ee)

                mse = get_mse(des_com, des_ee, res_com, res_ee)
                get_dist(res_com, res_ee)
                np.savetxt(f'./results/errors/traj{label}{version}_o{orientation}_mse.csv', mse.reshape((1, -1)),
                           header=mse_header, fmt="%.2f", delimiter=",")
