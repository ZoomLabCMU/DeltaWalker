import math
import numpy as np
import matplotlib.pyplot as plt

PI = math.pi
all_colors = ['tab:green', 'tab:red', 'tab:orange', 'tab:blue', 'tab:purple']
plt.rc('text', usetex=True)
plt.rc('font', family='serif', size=12, weight='bold')
plt.rcParams['text.latex.preamble'] = r'\boldmath'


def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in degrees.
    """
    ox, oy = origin
    px, py = point
    angle = np.radians(angle)

    qx = ox + np.cos(angle) * (px - ox) - np.sin(angle) * (py - oy)
    qy = oy + np.sin(angle) * (px - ox) + np.cos(angle) * (py - oy)
    return [qx, qy]


def draw_traj_3d(name, path, all_pos, all_pos_fk, ee_poses, offset, z_off, show):
    d_pos = path.pos_to_deltas(all_pos)
    d_pos_fk = path.pos_to_deltas(all_pos_fk)
    d_pos_ee = path.pos_to_deltas(ee_poses)

    fig = plt.figure(figsize=(10, 6))
    ax1 = fig.add_subplot(221, projection='3d')
    ax2 = fig.add_subplot(222, projection='3d')
    ax3 = fig.add_subplot(223, projection='3d')
    ax4 = fig.add_subplot(224, projection='3d')
    plots = [ax1, ax2, ax3, ax4]

    tol = 0.005
    xlims = [(0.04-tol, 0.04+tol), (0-tol, 0+tol), (-0.04-tol, -0.04+tol), (0-tol, 0+tol)]
    ylims = [(0-tol, 0+tol), (0.04-tol, 0.04+tol), (0-tol, 0+tol), (-0.04-tol, -0.04+tol)]

    for i in np.arange(path.walker.num_feet):
        ax = plots[i]
        xyz = d_pos[i]
        xyz_fk = d_pos_fk[i]
        xyz_ee = d_pos_ee[i]
        subname = f"Delta {i + 1}"
        ax.plot(xyz[:, 0], xyz[:, 1], zs=xyz[:, 2] - offset,
                color="red", marker="o", linestyle="solid", label="Generated Trajectory")
        ax.plot(xyz_fk[:, 0], xyz_fk[:, 1], zs=xyz_fk[:, 2] - offset,
                color="blue", marker="o", linestyle="dashed", label="Actuator FK Trajectory")
        ax.plot(xyz_ee[:, 0], xyz_ee[:, 1], zs=xyz_ee[:, 2] - z_off,
                color="green", marker="o", linestyle="dotted", label="EE Trajectory")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

        ax.set_xlim3d(xlims[i])
        ax.set_ylim3d(ylims[i])
        ax.set_zlim3d(0, 0.025)

        ax.legend()
        ax.title.set_text(subname)

    savename = f"./results/images/3d/3dplot_{name}.png"
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
    if show:
        plt.show()
    plt.close()


def draw_traj_z(name, path, all_pos, all_pos_fk, ee_poses, offset, z_off, show, steps):
    d_pos = path.pos_to_deltas(all_pos)
    d_pos_fk = path.pos_to_deltas(all_pos_fk)
    d_pos_ee = path.pos_to_deltas(ee_poses)

    t = np.arange(steps)

    fig = plt.figure(figsize=(10, 6))
    ax1 = fig.add_subplot(221)
    ax2 = fig.add_subplot(222)
    ax3 = fig.add_subplot(223)
    ax4 = fig.add_subplot(224)
    plots = [ax1, ax2, ax3, ax4]

    for i in np.arange(path.walker.num_feet):
        ax = plots[i]
        xyz = d_pos[i]
        xyz_fk = d_pos_fk[i]
        xyz_ee = d_pos_ee[i]
        subname = f"Delta {i + 1}"
        ax.plot(t, xyz[:, 2] - offset,
                color="red", marker="o", linestyle="solid", label="Generated Trajectory")
        ax.plot(t, xyz_fk[:, 2] - offset,
                color="blue", marker="o", linestyle="dashed", label="Actuator FK Trajectory")
        ax.plot(t, xyz_ee[:, 2] - z_off,
                color="green", marker="o", linestyle="dotted", label="EE Trajectory")
        ax.set_xlabel("timestep")
        ax.set_ylabel("z")
        ax.set_ylim(0, 0.025)
        ax.legend()
        ax.title.set_text(subname)

    savename = f"./results/images/z_only/zplot_{name}.png"
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
    if show:
        plt.show()
    plt.close()


def draw_traj_xy(name, path, all_pos, all_pos_fk, ee_poses, show, steps):
    d_pos = path.pos_to_deltas(all_pos)
    d_pos_fk = path.pos_to_deltas(all_pos_fk)
    d_pos_ee = path.pos_to_deltas(ee_poses)

    alpha = np.linspace(0.0, 1.0, num=steps)

    fig = plt.figure(figsize=(10, 6))
    ax1 = fig.add_subplot(221)
    ax2 = fig.add_subplot(222)
    ax3 = fig.add_subplot(223)
    ax4 = fig.add_subplot(224)
    plots = [ax1, ax2, ax3, ax4]

    for i in np.arange(path.walker.num_feet):
        ax = plots[i]
        xyz = d_pos[i]
        xyz_fk = d_pos_fk[i]
        xyz_ee = d_pos_ee[i]
        subname = f"Delta {i + 1}"
        for j in np.arange(xyz.shape[0]):
            a = alpha[j]
            # line1, = ax.plot(xyz[j, 0], xyz[j, 1], color="red", alpha=a, marker="o", linestyle="solid")
            # line2, = ax.plot(xyz_fk[j, 0], xyz_fk[j, 1], color="blue", alpha=a, marker="o", linestyle="dashed")
            line3, = ax.plot(xyz_ee[j, 0], xyz_ee[j, 1], color="green", alpha=a, marker="o", linestyle="dotted")
        # ax.legend([line1, line2, line3], ["Generated Trajectory", "Actuator FK Trajectory", "EE Trajectory"])
        ax.legend([line3], ["EE Trajectory"])
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.title.set_text(subname)

    savename = f"./results/images/xy/xyplot_{name}.png"
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
    if show:
        plt.show()
    plt.close()


def draw_pos_3d(name, paths, steps, ee_pos_fk, com_pos_fk, ee_pos_in, com_pos_in):
    fig = plt.figure(figsize=(10, 6))

    ax = fig.add_subplot(projection='3d')

    for i in np.arange(paths.walker.num_feet + 1):
        if i < paths.walker.num_feet:
            # fk = ee_pos_fk[(i*3):(i*3)+3, :]
            # print("sth")
            gn = ee_pos_in[(i*3):(i*3)+3, :]
            # print(gn)
            key = f"Foot {i} Position"
        else:
            # fk = com_pos_fk
            gn = com_pos_in
            key = "Body COM Position"

        fk_label = "FK " + key
        gn_label = "Generated " + key
        # ax.plot(fk[:, 0], fk[:, 1], zs=fk[:, 2], marker="o", linestyle="solid", label=fk_label)
        ax.plot(gn[0, :], gn[1, :], zs=gn[2, :], marker="o", linestyle="solid", label=gn_label)

    ax.legend()
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.show()
    plt.close()


def draw_pos_2d(name, paths, steps, ee_pos_res, com_pos_res, ee_pos_des, com_pos_des):
    alpha = np.linspace(0.25, 1.0, num=steps-1)

    fig_w = 7
    fig_h = 7
    fig = plt.figure(figsize=(fig_w, fig_h))
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    # ax3 = fig.add_subplot(313)
    # plots = [ax1, ax2, ax3]

    fig.tight_layout(pad=3)

    colors_des = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']
    colors_res = ['tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']

    legends = []
    labels = []

    for i in np.arange(paths.walker.num_feet + 1):
        c_des = colors_des[i]
        c_res = colors_res[i]

        if i < paths.walker.num_feet:
            res = ee_pos_res[(i * 3):(i * 3) + 3, :]
            des = ee_pos_des[(i * 3):(i * 3) + 3, :]
            key = f"\nPosition \nFoot {i+1}"
        else:
            res = com_pos_res
            des = com_pos_des
            key = "\nPosition \nBody COM"

        res_label = "Actual " + key
        des_label = "Desired " + key

        lines = []

        for j in np.arange(steps-1):
            a = alpha[j]
            line1_res, = ax1.plot(res[0, j:j+2], res[1, j:j+2], color=c_res, alpha=a, marker=".", linestyle="solid")
            ax2.plot(res[0, j:j + 2], res[2, j:j + 2], color=c_res, alpha=a, marker=".", linestyle="solid")
            # ax3.plot(res[1, j:j + 2], res[2, j:j + 2], color=c_res, alpha=a, marker=".", linestyle="solid")
            # ax2.plot(res[0, j:j + 2], res[2, j:j + 2], color=c_res, alpha=a, marker=".", linestyle="solid")
            # ax3.plot(res[1, j:j + 2], res[2, j:j + 2], color=c_res, alpha=a, marker=".", linestyle="solid")
            lines.append(line1_res)

            line1_des = ax1.plot(des[0, j:j+2], des[1, j:j+2], color=c_des, alpha=a, marker=".", linestyle="solid")
            ax2.plot(des[0, j:j+2], des[2, j:j+2], color=c_des, alpha=a, marker=".", linestyle="solid")
            # ax3.plot(des[1, j:j+2], des[2, j:j+2], color=c_des, alpha=a, marker=".", linestyle="solid")
            lines.append(line1_des)

        legends.append(lines[-2])   # res
        legends.append(lines[-1][0])   # des
        labels.append(res_label)    # res
        labels.append(des_label)    # des

    box = ax1.get_position()
    ax1.set_position([box.x0, box.y0, box.width * 0.75, box.height])
    box = ax2.get_position()
    ax2.set_position([box.x0, box.y0, box.width * 0.75, box.height])
    # box = ax3.get_position()
    # ax3.set_position([box.x0, box.y0, box.width * 0.75, box.height])

    # ax1.legend()
    ax1.legend(legends, labels, loc='upper left', bbox_to_anchor=(1, 0.9), ncol=1)

    #
    # ax2.legend(legends, labels)
    # ax3.legend(legends, labels)

    ax1.set(xlabel="X", ylabel="Y")
    ax1.set_title("XY Plane")

    ax2.set(xlabel="X", ylabel="Z")
    ax2.set_title("XZ Plane")

    # ax3.set(xlabel="Y", ylabel="Z")
    # ax3.set_title("YZ Plane")

    savename = f"./results_julia/images/xy/xyplot_{name}.png"
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
    # if show:
    plt.show()
    plt.close()


def draw_ocrl_3d(save_name, name, paths, steps, ee_pos_res, com_pos_res, ee_pos_des, com_pos_des):
    alpha = np.linspace(0.25, 1.0, num=steps - 1)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # ax.view_init(elev=30, azim=45, roll=15)

    legends = []
    labels = []

    for i in np.arange(paths.walker.num_feet + 1):
        c = all_colors[i]

        if i < paths.walker.num_feet:
            res = ee_pos_res[(i * 3):(i * 3) + 3, :] * 100
            des = ee_pos_des[(i * 3):(i * 3) + 3, :] * 100
            key = f"Position Foot {i + 1}"
        else:
            res = com_pos_res * 100
            des = com_pos_des * 100
            key = "Position Body COM"

        res_label = "Actual " + key
        des_label = "Desired " + key

        lines = []

        for j in np.arange(steps - 1):
            line1_res, = ax.plot(res[0, j:j + 2], res[1, j:j + 2], zs=res[2, j:j + 2], color=c, linestyle="solid")
            lines.append(line1_res)

            line1_des, = ax.plot(des[0, j:j + 2], des[1, j:j + 2], zs=des[2, j:j + 2], color=c, linestyle="dotted")
            lines.append(line1_des)

        legends.append(lines[-2])  # res
        legends.append(lines[-1])  # des
        labels.append(res_label)  # res
        labels.append(des_label)  # des

    ax.set_xlabel(r"$\textbf{X (cm)}$")
    ax.set_ylabel(r"$\textbf{Y (cm)}$")
    ax.set_zlabel(r"$\textbf{Z (cm)}$")
    ax.set_xlim3d([-6, 6])
    ax.set_ylim3d([-6, 6])
    ax.set_zlim3d([0, 6])
    ax.set_title(name)
    # box = ax1.get_position()
    # ax1.set_position([box.x0, box.y0, box.width * 0.75, box.height])
    # box = ax2.get_position()
    # ax2.set_position([box.x0, box.y0, box.width * 0.75, box.height])
    # box = ax3.get_position()
    # ax3.set_position([box.x0, box.y0, box.width * 0.75, box.height])

    # ax1.legend()
    ax.legend(legends, labels, loc='upper left', bbox_to_anchor=(1, 0.9), ncol=1)

    #
    # ax2.legend(legends, labels)
    # ax3.legend(legends, labels)

    # ax1.set(xlabel="X", ylabel="Y")
    # ax1.set_title("XY Plane")
    #
    # ax2.set(xlabel="X", ylabel="Z")
    # ax2.set_title("XZ Plane")

    # ax3.set(xlabel="Y", ylabel="Z")
    # ax3.set_title("YZ Plane")

    savename = f"./results_julia/plots/3d/{save_name}.png"
    plt.savefig(savename, bbox_inches="tight", pad_inches=0.05)
    # if show:
    plt.show()
    plt.close()


def print_errs(ee_pos_res, com_pos_res, ee_pos_des, com_pos_des, num_feet):
    mse = np.zeros(num_feet + 1)
    for i in np.arange(num_feet + 1):
        if i < num_feet:
            ee_res = ee_pos_res[i*3:(i+1)*3, :]
            ee_des = ee_pos_des[i*3:(i+1)*3, :]
            all_errs = np.linalg.norm(ee_des - ee_res, axis=0)
        else:
            all_errs = np.linalg.norm(com_pos_des - com_pos_res, axis=0)
            print(len(all_errs), com_pos_res.shape[1])
        mse[i] = np.average(all_errs * 1000)
    rnd = np.round(mse, 1)
    print(f"{rnd[4]} & {rnd[0]} & {rnd[1]} & {rnd[2]} & {rnd[3]}")
    return mse