import numpy as np
import matplotlib.pyplot as plt

# rotate trajectory to be in each orientation


def get_rot(angle):
    rad = np.radians(angle)
    cos = np.round(np.cos(rad))
    sin = np.round(np.sin(rad))
    rz = np.array([[cos, -sin, 0],
                  [sin, cos, 0],
                  [0, 0, 1]])

    rotation = np.zeros((15, 15))
    for i in np.arange(5):
        rotation[i*3:(i+1)*3, i*3:(i+1)*3] = rz
    return rotation


def draw(steps, org_pos, rot_pos):
    alpha = np.linspace(0.25, 1.0, num=steps - 1)

    # fig_w = 7
    # fig_h = 7
    # fig = plt.figure(figsize=(fig_w, fig_h))
    # ax1 = fig.add_subplot(211)
    # ax2 = fig.add_subplot(212)
    # # ax3 = fig.add_subplot(313)
    # # plots = [ax1, ax2, ax3]
    #
    # fig.tight_layout(pad=3)

    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']

    legends = []
    labels = []

    for i in np.arange(5):
        c = colors[i]

        if i < 1:
            org = org_pos[(i * 3):(i * 3) + 3, :]
            rot = rot_pos[(i * 3):(i * 3) + 3, :]
            key = "\nPosition \nBody COM"
        else:
            org = org_pos[(i * 3):(i * 3) + 3, :]
            rot = rot_pos[(i * 3):(i * 3) + 3, :]
            key = f"\nPosition \nFoot {i}"

        org_label = "Org Pos " + key
        rot_label = "Rot Pos " + key

        lines = []

        for j in np.arange(steps - 1):
            a = alpha[j]
            line1_org, = plt.plot(org[0, j:j + 2], org[1, j:j + 2], color=c, alpha=a, marker=".", linestyle="solid")
            lines.append(line1_org)
            line1_rot, = plt.plot(rot[0, j:j + 2], rot[1, j:j + 2], color=c, alpha=a, marker="x", linestyle="solid")
            lines.append(line1_rot)

        legends.append(lines[-2])  # res
        legends.append(lines[-1])  # des
        labels.append(org_label)  # res
        labels.append(rot_label)  # des

    plt.legend(legends, labels, loc='upper left', bbox_to_anchor=(1, 0.9), ncol=1)

    plt.show()
    plt.close()


if __name__ == "__main__":
    angles = [90, 180, -90]
    foot_order = [1, 2, 3, 4]
    trajs = ["walk", "ambl", "mtri", "adjs"]
    steps = ["a", "b", "c", "d"]
    # orders = [1, 2, 3, 4]
    # steps = ["a"]

    load_path = "./orientation1/"

    for i, angle in enumerate(angles):
        R = get_rot(angle)
        foot_order.insert(0, foot_order.pop())
        for traj in trajs:
            # for order in orders:
            for step in steps:
                file_name = f"{traj}_{step}"
                data = np.loadtxt(f"{load_path}{file_name}.csv", skiprows=1, delimiter=',')[:15]

                num_steps = data.shape[1]
                heading = np.arange(num_steps)
                rotated = R @ data
                reordered = np.array(rotated)

                for j, foot in enumerate(foot_order):
                    reordered[(j+1)*3:(j+2)*3, :] = rotated[foot*3:(foot+1)*3, :]

                results = np.vstack((heading, reordered))
                np.savetxt(f"./orientation{i + 2}/{file_name}.csv", results, fmt='%.20f', delimiter=',')

                # draw(num_steps, data, reordered)

