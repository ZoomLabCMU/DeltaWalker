import numpy as np
import matplotlib.pyplot as plt
import sys


all_colors = ['tab:green', 'tab:red', 'tab:orange', 'tab:blue', 'tab:purple']
plt.rc('text', usetex=True)
plt.rc('font', family='serif', size=12, weight='bold')
plt.rcParams['text.latex.preamble'] = r'\boldmath'

'''
def plot_trial(savename, folder, step=20):
    data = np.loadtxt(f"./{folder}/data/{savename}.csv", dtype="float", delimiter=",", skiprows=1)
    plot_idx = np.arange(0, data.shape[0] - step, step)
    num_frames = len(plot_idx)
    alpha = np.linspace(0.1, 1.0, num=num_frames)

    legends = []
    labels = []

    for i in np.arange(4):
        c = all_colors[i]
        pts = data[:, (i*2):(i*2)+2]
        lines = []
        label = f"Delta {i+1} Position"
        for a, j in enumerate(plot_idx):
            line1, = plt.plot(pts[[j, j + step], 0], pts[[j, j + step], 1], color=c, alpha=alpha[a], linestyle="solid")
            lines.append(line1)

        legends.append(lines[-1])
        labels.append(label)

    plt.legend(legends, labels)

    plt.xlabel(f"X (cm)")
    plt.ylabel(f"Y (cm)")
    plt.title(savename)
    plt.grid(which='major', alpha=0.75)
    plt.grid(which='minor', alpha=0.25)
    plt.minorticks_on()

    plt.savefig(f"./{folder}/plots/{savename}.png", bbox_inches="tight", pad_inches=0.05)
    plt.close()
'''


def plot_traj_wrapper(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names, step=20):
    for orientation in orientations:
        for i, traj_label in enumerate(traj_labels):
            name = traj_names[i]
            if orientation == 1:
                if traj_label == "A" or traj_label == "B":
                    order = ["Front Right", "Front Left", "Back Left", "Back Right"]
                else:
                    order = ["Front", "Left", "Back", "Right"]
            elif orientation == 2:
                if traj_label == "A" or traj_label == "B":
                    order = ["Back Right", "Front Right", "Front Left", "Back Left"]
                else:
                    order = ["Right", "Front", "Left", "Back"]
            for j, version in enumerate(versions):
                sizes_step = sizes_steps[j]
                total_step = total_steps[j]
                traj_id = "Traj" + traj_label + version
                plot_traj(traj_id, folder, name, order, sizes_step, total_step)


def plot_traj(traj_label, folder, name, order, step_size, step_total, fsz, pos, step=20):
    styles = ["dashed", "solid", "dotted"]
    legends = []
    labels = []
    fig = plt.figure(figsize=(12, 11))
    for i in np.arange(3):
        data = np.loadtxt(f"./{folder}/data/cm/{traj_label}_t{i+1}_o1_3x.csv", dtype="float", delimiter=",", skiprows=1)
        plot_idx = np.arange(0, data.shape[0] - step, step)
        for j in np.arange(4):
            c = all_colors[j]
            pts = data[:, (j * 2):(j * 2) + 2]
            lines = []
            label = r"$\textbf{" + f"Trial {i + 1}" + r"}$" + "\n" + r"$\textbf{" + f"{order[j]}" + "}$"
            line1, = plt.plot(pts[plot_idx, 0], pts[plot_idx, 1], color=c, linestyle=styles[i])
            lines.append(line1)
            legends.append(lines[-1])
            labels.append(label)

    plt.xlabel(f"X (cm)")
    plt.ylabel(f"Y (cm)")
    plt.grid(which='major', alpha=0.75)
    plt.grid(which='minor', alpha=0.25)
    plt.xlim([-4.5, 12])
    plt.ylim([-5.5, 5.5])
    plt.gca().set_aspect("equal")
    plt.minorticks_on()

    # plt.legend(legends, labels)
    width_scale = 0.7
    box = plt.gca().get_position()
    plt.gca().set_position([box.x0, box.y0, box.width * width_scale, box.height])
    plt.legend(legends, labels, loc="center left", bbox_to_anchor=(1, 0.5))

    plt.title(r"$\textbf{" + f"Step Size {step_size} cm for {step_total} cm Total Movement" + "}$")
    fig.suptitle(r"$\textbf{X and Y Positions for " + name + "Gait Over 3 Steps}$", fontsize=18, y=0.75)

    plt.savefig(f"./{folder}/plots/all_trials/{traj_label}.png", bbox_inches="tight", pad_inches=0.05)
    # plt.show( )
    plt.close()


def ocrl_plot_all_traj(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names, step=20):
    for orientation in orientations:
        for i, traj_label in enumerate(traj_labels):
            name = traj_names[i]
            fig = plt.figure(figsize=(11, 8))
            plots = []
            if orientation == 1:
                if traj_label == "A" or traj_label == "B":
                    order = ["Front Right", "Front Left", "Back Left", "Back Right"]
                else:
                    order = ["Front", "Left", "Back", "Right"]
            elif orientation == 2:
                if traj_label == "A" or traj_label == "B":
                    order = ["Back Right", "Front Right", "Front Left", "Back Left"]
                else:
                    order = ["Right", "Front", "Left", "Back"]
            for j, version in enumerate(versions):
                step_size = sizes_steps[j]
                step_total = total_steps[j]
                id = f"Traj{traj_label}{version}"
                num = 221 + j
                ax = fig.add_subplot(num)
                ax, legends, labels = ocrl_plot_helper(folder, ax, id, step_size, step_total, step, trials, iters, orientation, order)

                ax.annotate(r"$\textbf{(" + version + ")}$",
                        xy=(0, 1), xycoords='axes fraction',
                        xytext=(+0.5, -0.5), textcoords='offset fontsize',
                        fontsize='medium', verticalalignment='top',
                        bbox=dict(facecolor='1.0', edgecolor='none', pad=3.0))
                plots.append(ax)

            width_scale = 0.7
            for ax in plots:
                box = ax.get_position()
                ax.set_position([box.x0, box.y0, box.width * width_scale, box.height])
            plots[1].legend(legends, labels, loc="center left", bbox_to_anchor=(1, -0.29))

            fig.suptitle(r"$\textbf{X and Y Positions for " + name + "Gait Over 3 Steps}$", fontsize=18)
            fig.subplots_adjust(wspace=0.3, hspace=0.15, right=0.8, top=0.9)
            plt.savefig(f"./{folder}/plots/all_versions/rw_ocrl_traj{traj_label}_o{orientation}.png", bbox_inches="tight", pad_inches=0.05, dpi=300)
            plt.close()


def ocrl_plot_helper(folder, ax, id, step_size, step_total, step, trials, iters, orientation, order):
    styles = ["dashed", "solid", "dotted"]
    legends = []
    labels = []

    for i in trials:
        data = np.loadtxt(f"./{folder}/data/cm/{id}_t{i}_o{orientation}_{iters}x.csv", dtype="float", delimiter=",", skiprows=1)
        plot_idx = np.arange(0, data.shape[0] - step, step)
        for j in np.arange(4):
            c = all_colors[j]
            pts = data[:, (j * 2):(j * 2) + 2]
            lines = []
            label = r"$\textbf{" + f"Trial {i}" + "}$" + "\n" + r"$\textbf{" + f"{order[j]}" + "}$"
            line1, = ax.plot(pts[plot_idx, 0], pts[plot_idx, 1], color=c, alpha=1.0, linestyle=styles[i-1])
            lines.append(line1)
            legends.append(lines[-1])
            labels.append(label)

    ax.set_xlabel(r"$\textbf{X (cm)}$")
    ax.set_ylabel(r"$\textbf{Y (cm)}$")
    ax.set_xlim([-4.5, 12])
    ax.set_ylim([-5.5, 5.5])

    ax.set_title(r"$\textbf{" + f"Step Size {step_size} cm" + "}$" + "\n" + r"$\textbf{" + f"{step_total} cm Total Movement" + "}$")
    ax.grid(which='major', alpha=0.75)
    ax.grid(which='minor', alpha=0.25)
    ax.set_aspect("equal")
    ax.minorticks_on()

    return ax, legends, labels


if __name__ == "__main__":
    folder = "1_ocrl_trials"

    trials = [1, 2, 3]
    orientations = [1]
    traj_labels = ["A", "B", "C", "D"]
    # traj_labels = ["C", "D"]
    iters = 3
    versions = ["a", "b", "c", "d"]
    sizes_steps = ["1.0", "1.5", "2.0", "2.5"]
    total_steps = ["3.0", "4.5", "6.0", "7.5"]
    traj_names = ["Walk ", "Amble ", "S-", "C-"]
    # traj_names = ["S-", "C-"]

    ocrl_plot_all_traj(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names, step=20)
    # plot_traj_wrapper(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names,
    #                    step=20)


