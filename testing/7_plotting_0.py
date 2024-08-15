import numpy as np
import matplotlib.pyplot as plt
import sys


all_colors = ['tab:green', 'tab:red', 'tab:orange', 'tab:blue', 'tab:purple']
plt.rc('text', usetex=True)
plt.rc('font', family='serif', size=12, weight='bold')
plt.rcParams['text.latex.preamble'] = r'\boldmath'


def plot_versions_side(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names, step=20):
    sublabels = ['a', 'b', 'c', 'd']
    styles = ["dashed", "solid", "dotted"]
    for orientation in orientations:
        if orientation == 1:
            order = ["Front Right", "Front Left", "Back Left", "Back Right"]
            ylims = [-5.5, 5.5]
        elif orientation == 2:
            order = ["Back Right", "Front Right", "Front Left", "Back Left"]
            ylims = [-4, 7]
        for i, traj_label in enumerate(traj_labels):
            name = traj_names[i]
            fig = plt.figure(figsize=(11, 8))
            plots = []

            for j, version in enumerate(versions):
                sublabel = sublabels[j]
                step_size = sizes_steps[j]
                step_total = total_steps[j]
                num = 221 + j
                ax = fig.add_subplot(num)

                data = np.loadtxt(f"./{folder}/data/cm/Traj{traj_label}{version}_t1_o{orientation}_{iters}x.csv", dtype="float", delimiter=",", skiprows=1)

                plot_idx = np.arange(0, data.shape[0] - step, step)
                for k in np.arange(4):
                    c = all_colors[k]
                    pts = data[:, (k * 2):(k * 2) + 2]
                    ax.plot(pts[plot_idx, 0], pts[plot_idx, 1], color=c, linestyle=styles[j])

                ax.set_xlabel(r"$\textbf{X (cm)}$")
                ax.set_ylabel(r"$\textbf{Y (cm)}$")
                ax.set_xlim([-4.5, 12])
                ax.set_ylim(ylims)

                ax.set_title(r"$\textbf{" f"Version {version}" + "}$")
                ax.grid(which='major', alpha=0.75)
                ax.grid(which='minor', alpha=0.25)
                ax.set_aspect("equal")
                ax.minorticks_on()

                ax.annotate(r"$\textbf{(" + sublabel + ")}$",
                        xy=(0, 1), xycoords='axes fraction',
                        xytext=(+0.5, -0.5), textcoords='offset fontsize',
                        fontsize='medium', verticalalignment='top',
                        bbox=dict(facecolor='1.0', edgecolor='none', pad=3.0))
                plots.append(ax)

            ax = fig.add_subplot(224)
            sublabel = sublabels[len(versions)]
            legends = []
            labels = []
            for j, version in enumerate(versions):
                step_size = sizes_steps[j]
                step_total = total_steps[j]
                data = np.loadtxt(f"./{folder}/data/cm/Traj{traj_label}{version}_t1_o{orientation}_{iters}x.csv",
                                  dtype="float", delimiter=",", skiprows=1)

                plot_idx = np.arange(0, data.shape[0] - step, step)
                for k in np.arange(4):
                    c = all_colors[k]
                    lines = []
                    label = r"$\textbf{" + f"Version {version}" + "}$" + "\n" + r"$\textbf{" + f"{order[k] }" + "}$"
                    pts = data[:, (k * 2):(k * 2) + 2]
                    line1, = ax.plot(pts[plot_idx, 0], pts[plot_idx, 1], color=c, linestyle=styles[j])
                    lines.append(line1)
                    legends.append(lines[-1])
                    labels.append(label)

                ax.set_xlabel(r"$\textbf{X (cm)}$")
                ax.set_ylabel(r"$\textbf{Y (cm)}$")
                ax.set_xlim([-4.5, 12])
                ax.set_ylim(ylims)

                ax.set_title(r"$\textbf{All Versions}$")
                ax.grid(which='major', alpha=0.75)
                ax.grid(which='minor', alpha=0.25)
                ax.set_aspect("equal")
                ax.minorticks_on()

                ax.annotate(r"$\textbf{(" + sublabel + ")}$",
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

            fig.suptitle(r"$\textbf{X and Y Positions for " + name + " Gait Over 3 Steps}$" + "\n" + r"$\textbf{" + f"Step Size {step_size} cm for {step_total} cm Total Movement" + "}$", fontsize=18)
            fig.subplots_adjust(wspace=0.3, hspace=0.15, right=0.8, top=0.9)
            plt.savefig(f"./{folder}/plots/all_versions/rw_hw_traj{traj_label}_o{orientation}.png", bbox_inches="tight", pad_inches=0.05, dpi=300)
            # plt.show()
            plt.close()


def plot_versions_single_unlabeled(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names, step=20):
    sublabels = ['a', 'b', 'c', 'd']
    # styles = ["dashed", "solid", "dotted"]
    for orientation in orientations:
        for i, traj_label in enumerate(traj_labels):
            name = traj_names[i]
            if orientation == 1:
                ylims = [-5.5, 5.5]
                if traj_label == "A" or traj_label == "B":
                    order = ["Front Right", "Front Left", "Back Left", "Back Right"]
                else:
                    order = ["Front", "Left", "Back", "Right"]
            elif orientation == 2:
                ylims = [-4, 7]
                if traj_label == "A" or traj_label == "B":
                    order = ["Back Right", "Front Right", "Front Left", "Back Left"]
                else:
                    order = ["Right", "Front", "Left", "Back"]

            for j, version in enumerate(versions):
                fig = plt.figure(figsize=(8, 5))
                sublabel = sublabels[j]
                step_size = sizes_steps[j]
                step_total = total_steps[j]

                data = np.loadtxt(f"./{folder}/data/cm/Traj{traj_label}{version}_t1_o{orientation}_{iters}x.csv", dtype="float", delimiter=",", skiprows=1)

                plot_idx = np.arange(0, data.shape[0] - step, step)
                legends = []
                labels = []
                for k in np.arange(4):
                    c = all_colors[k]
                    lines = []
                    label = r"$\textbf{" + f"{order[k]}" + "}$"
                    pts = data[:, (k * 2):(k * 2) + 2]
                    line1, = plt.plot(pts[plot_idx, 0], pts[plot_idx, 1], color=c, linestyle="solid")
                    lines.append(line1)
                    legends.append(lines[-1])
                    labels.append(label)

                plt.xlabel(r"$\textbf{X (cm)}$")
                plt.ylabel(r"$\textbf{Y (cm)}$")
                plt.xlim([-4.5, 12])
                plt.ylim(ylims)

                plt.title(r"$\textbf{" + f"Step Size {step_size} cm for {step_total} cm Total Movement" + "}$")
                plt.grid(which='major', alpha=0.75)
                plt.grid(which='minor', alpha=0.25)
                plt.gca().set_aspect("equal")
                plt.minorticks_on()

                # width_scale = 0.7
                # box = plt.gca().get_position()
                # plt.gca().set_position([box.x0, box.y0, box.width * width_scale, box.height])
                plt.legend(legends, labels) #, loc="center left", bbox_to_anchor=(1, 0.5))

                fig.suptitle(r"$\textbf{X and Y Positions for " + name + "Gait Over 3 Steps}$", fontsize=18)
                fig.subplots_adjust(wspace=0.3, hspace=0.15, right=0.8, top=0.9)
                plt.savefig(f"./{folder}/plots/single_versions_no_label/rw_hw_traj{traj_label}{version}_o{orientation}.png", bbox_inches="tight", pad_inches=0.05, dpi=300)
                # plt.show()
                plt.close()


def plot_versions_single_labeled(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names, step=20):
    sublabels = ['a', 'b', 'c', 'd']
    # styles = ["dashed", "solid", "dotted"]
    for orientation in orientations:
        if orientation == 1:
            order = ["Front Right", "Front Left", "Back Left", "Back Right"]
            ylims = [-5.5, 5.5]
        elif orientation == 2:
            order = ["Back Right", "Front Right", "Front Left", "Back Left"]
            ylims = [-4, 7]
        for i, traj_label in enumerate(traj_labels):
            name = traj_names[i]
            for j, version in enumerate(versions):
                fig = plt.figure(figsize=(8, 5))
                sublabel = sublabels[j]
                step_size = sizes_steps[j]
                step_total = total_steps[j]

                data = np.loadtxt(f"./{folder}/data/cm/Traj{traj_label}{version}_t1_o{orientation}_{iters}x.csv", dtype="float", delimiter=",", skiprows=1)

                plot_idx = np.arange(0, data.shape[0] - step, step)
                legends = []
                labels = []
                for k in np.arange(4):
                    c = all_colors[k]
                    lines = []
                    label = r"$\textbf{" + f"{order[k]}" + "}$"
                    pts = data[:, (k * 2):(k * 2) + 2]
                    line1, = plt.plot(pts[plot_idx, 0], pts[plot_idx, 1], color=c, linestyle="solid")
                    lines.append(line1)
                    legends.append(lines[-1])
                    labels.append(label)

                plt.xlabel(r"$\textbf{X (cm)}$")
                plt.ylabel(r"$\textbf{Y (cm)}$")
                plt.xlim([-4.5, 12])
                plt.ylim(ylims)

                plt.title(r"$\textbf{" + f"Step Size {step_size} cm for {step_total} cm Total Movement" + "}$")
                plt.grid(which='major', alpha=0.75)
                plt.grid(which='minor', alpha=0.25)
                plt.gca().set_aspect("equal")
                plt.minorticks_on()

                # width_scale = 0.7
                # box = plt.gca().get_position()
                # plt.gca().set_position([box.x0, box.y0, box.width * width_scale, box.height])
                plt.legend(legends, labels) #, loc="center left", bbox_to_anchor=(1, 0.5))

                fig.suptitle(r"$\textbf{X and Y Positions for " + name + " Gait V" + version + " Over 3 Steps}$", fontsize=18)
                fig.subplots_adjust(wspace=0.3, hspace=0.15, right=0.8, top=0.9)
                plt.savefig(f"./{folder}/plots/single_versions_yes_label/rw_hw_traj{traj_label}{version}_o{orientation}.png", bbox_inches="tight", pad_inches=0.05, dpi=300)
                # plt.show()
                plt.close()


if __name__ == "__main__":
    folder = "0_manual_design"
    trials = [1]
    orientations = [1, 2]
    traj_labels = ["A", "B", "C"]
    versions = ["0", "1", "2"]
    iters = 3
    sizes_steps = ["1.0", "1.0", "1.0"]
    total_steps = ["3.0", "3.0", "3.0"]
    traj_names = ["Walk ", "Amble ", "S-"]
    plot_versions_side(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names, step=20)
    plot_versions_single_unlabeled(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps, traj_names,
                       step=20)
    plot_versions_single_labeled(folder, trials, orientations, traj_labels, iters, versions, sizes_steps, total_steps,
                                   traj_names,
                                   step=20)

