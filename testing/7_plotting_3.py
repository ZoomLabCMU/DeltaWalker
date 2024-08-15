import numpy as np
import matplotlib.pyplot as plt
import sys


all_colors = ['tab:green', 'tab:red', 'tab:orange', 'tab:blue', 'tab:purple']
plt.rc('text', usetex=True)
plt.rc('font', family='serif', size=12, weight='bold')
plt.rcParams['text.latex.preamble'] = r'\boldmath'


def ocrl_plot_all_traj(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names, step=20):
    styles = ["dashed", "solid", "dotted"]
    legends = []
    labels = []

    for orientation in orientations:
        for i, traj_label in enumerate(traj_labels):
            name = traj_names[i]
            fig = plt.figure(figsize=(10,6))
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
                labels = []

                step_size = sizes_steps[j]
                step_total = total_steps[j]
                id = f"Traj{traj_label}{version}"

                for trial in trials:
                    data = np.loadtxt(f"./{folder}/data/cm/{id}_t{trial}_o{orientation}_{iters}x.csv", dtype="float",
                                      delimiter=",", skiprows=1)
                    plot_idx = np.arange(0, data.shape[0] - step, step)

                    for j in np.arange(4):
                        c = all_colors[j]
                        pts = data[:, (j * 2):(j * 2) + 2]
                        lines = []
                        label = r"$\textbf{" + f"Trial {trial}" + "}$" + "\n" + r"$\textbf{" + f"{order[j]}" + "}$"
                        line1, = plt.plot(pts[plot_idx, 0], pts[plot_idx, 1], color=c, alpha=1.0,
                                         linestyle=styles[trial - 1])
                        lines.append(line1)
                        legends.append(lines[-1])
                        labels.append(label)

                plt.xlabel(r"$\textbf{X (cm)}$")
                plt.ylabel(r"$\textbf{Y (cm)}$")
                plt.xlim([-4.5, 12])
                plt.ylim([-5.5, 5.5])

                plt.title(
                    r"$\textbf{" + f"Step Size {step_size} cm" + "}$" + "\n" + r"$\textbf{" + f"{step_total} cm Total Movement" + "}$")
                plt.grid(which='major', alpha=0.75)
                plt.grid(which='minor', alpha=0.25)
                plt.gca().set_aspect("equal")
                plt.minorticks_on()

                width_scale = 0.8
                box = plt.gca().get_position()
                plt.gca().set_position([box.x0, box.y0, box.width * width_scale, box.height])
                plt.gca().legend(legends, labels, loc="center left", bbox_to_anchor=(1.05, 0.5))

                fig.suptitle(r"$\textbf{X and Y Positions for " + name + "Gait Over 3 Steps}$", fontsize=18)
                # fig.subplots_adjust(wspace=0.3, hspace=0.15, right=0.8, top=0.9)
                plt.savefig(f"./{folder}/plots/all_versions/rw_ocrl_traj{traj_label}_o{orientation}.png", bbox_inches="tight", pad_inches=0.05, dpi=300)
                plt.close()


if __name__ == "__main__":
    folder = "3_ocrl_further_studies"

    trials = [1, 2, 3]
    orientations = [1]
    traj_labels = ["A", "C"]
    versions = ["e"]
    iters = 1
    sizes_steps = ["2"]
    total_steps = ["6"]
    traj_names = ["Walk ", "S-"]
    print(folder)
    ocrl_plot_all_traj(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps,
                       traj_names, step=20)

