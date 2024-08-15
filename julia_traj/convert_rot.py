import numpy as np
import matplotlib.pyplot as plt


# convert the adjs trajectory to be a rotation trajectory as an attempt to design a rotation in place trajectory

# delta 1 - rotate 90
# delta 2 - rotate 180
# delta 3 - rotate 270
# delta 4 - rotate 0

# all data has shape 15 x num_steps where each set of 3 rows is com, d1, d2, d3, d4

def rotate(pts, angle):
    offset = pts[:, 0].reshape((-1, 1))

    removed_offset = pts - offset

    cos = np.cos(angle)
    sin = np.sin(angle)
    rz = np.array([[cos, -sin, 0],
                   [sin, cos, 0],
                   [0, 0, 1]])

    rotated = rz @ removed_offset
    new_pts = rotated + offset

    return new_pts


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
    load_path = './orientation1/'
    base_file = 'adjs_'
    file_version = 'a'
    file_name = f'rots_mod'

    data = np.loadtxt(f"{load_path}{base_file}{file_version}.csv", skiprows=1, delimiter=',')[:15]
    steps = data.shape[1]

    offset_angles = [90, 180, 270, 0]
    theta = -15

    rot_version = np.zeros(data.shape)

    for i, offset in enumerate(offset_angles):
        angle = np.deg2rad(offset + theta)
        com_section = data[0:3, i*10:(i+1)*10]
        delta = data[(i+1)*3:(i+2)*3, :]

        com_rot = rotate(com_section, angle)
        delta_rot = rotate(delta, angle)

        rot_version[0:3, i*10:(i+1)*10] = com_rot
        rot_version[(i+1)*3:(i+2)*3, :] = delta_rot

    # last_point = rot_version[:, -1].reshape((-1, 1))
    # last_point[0:2] = 0
    first_point = rot_version[:, 0].reshape((-1, 1))

    draw(steps, data, rot_version)

    rot_version = np.hstack((rot_version, first_point))
    heading = np.arange(steps + 1)
    results = np.vstack((heading, rot_version))

    # np.savetxt(f"{load_path}{file_name}_2.csv", results, fmt='%.20f', delimiter=',')



