import sys
import numpy as np
import matplotlib.pyplot as plt

import kinematics_walker as k


def ranges():
    steps = 20
    middle_steps = 4
    motor1_range = np.linspace(0, 0.02, middle_steps, endpoint=True)
    motor2_range = np.linspace(0, 0.02, steps, endpoint=True)
    motor3_range = np.linspace(0, 0.02, steps, endpoint=True)

    return motor1_range, motor2_range, motor3_range


def ranges_full():
    steps = 10
    motor1_range = np.linspace(0, 0.02, steps, endpoint=True)
    motor2_range = np.linspace(0, 0.02, steps, endpoint=True)
    motor3_range = np.linspace(0, 0.02, steps, endpoint=True)

    return motor1_range, motor2_range, motor3_range


# Find WS all deltas
# def find_ws_all():
#     walker = DeltaWalkerKin(nonfloat=False)
#
#     print("CENTERS", walker.delta_centers)
#     # actuations = [[0, 0, 0], [0.02, 0.02, 0.02], [0.005, 0.005, 0.005], [0.01, 0.01, 0.01]]
#     # ee_pos = walker.go_to_FK(actuations)
#     # print("EEPOS", ee_pos)
#
#     motor1_range, motor2_range, motor3_range = ranges()
#
#     for dst1 in motor1_range:
#         x_solutions = [[], [], [], []]
#         y_solutions = [[], [], [], []]
#         z_solutions = [[], [], [], []]
#
#         for dst2 in motor2_range:
#             for dst3 in motor3_range:
#                 # print("# Distance: ", dst1, dst2, dst3)
#                 actuations = [[dst1, dst2, dst3], [dst1, dst2, dst3], [dst1, dst2, dst3], [dst1, dst2, dst3]]
#                 ee_pos = walker.go_to_FK(actuations)
#                 # print(ee_pos)
#                 for e in range(walker.num_finger):
#                     x_solutions[e].append(ee_pos[e][0])
#                     y_solutions[e].append(ee_pos[e][1])
#                     z_solutions[e].append(ee_pos[e][2])
#
#         x_solutions = np.array(x_solutions)
#         y_solutions = np.array(y_solutions)
#         z_solutions = np.array(z_solutions) # *-1
#         # z_solutions -= np.min(z_solutions)
#
#         print("Min/Max on x axis: ", np.min(x_solutions), np.max(x_solutions))
#         print("Min/Max on y axis: ", np.min(y_solutions), np.max(y_solutions))
#         print("Min/Max on z axis: ", np.min(z_solutions), np.max(z_solutions))
#
#         draw_ws_all(x_solutions, y_solutions, z_solutions)
#     return
#
#
# # Draws WS for each delta
# def draw_ws_all(x_solutions, y_solutions, z_solutions):
#     fig = plt.figure(figsize=(10, 10))
#
#     color_tracking = ["mediumorchid", "mediumturquoise", "mediumpurple", "cornflowerblue"]
#     ax1 = fig.add_subplot(221, projection='3d')
#
#     for i in range(len(x_solutions)):
#         ax1.scatter(x_solutions[i], y_solutions[i], z_solutions[i], color=color_tracking[i])
#     ax1.set_title("Working Space")
#     ax1.set_xlabel("X")
#     ax1.set_ylabel("Y")
#     ax1.set_zlabel("Z")
#
#     ax2 = fig.add_subplot(222, projection='3d')
#     for i in range(len(x_solutions)):
#         ax2.scatter(x_solutions[i], y_solutions[i], z_solutions[i], color = color_tracking[i])
#     ax2.set_title("Front")
#     ax2.set_xlabel("X")
#     ax2.set_ylabel("Y")
#     ax2.set_zlabel("Z")
#
#     ax3 = fig.add_subplot(223, projection='3d')
#     for i in range(len(x_solutions)):
#         ax3.scatter(x_solutions[i], y_solutions[i], z_solutions[i], color = color_tracking[i])
#     ax3.set_title("Side")
#     ax3.set_xlabel("X")
#     ax3.set_ylabel("Y")
#     ax3.set_zlabel("Z")
#
#     ax4 = fig.add_subplot(224, projection='3d')
#     for i in range(len(x_solutions)):
#         ax4.scatter(x_solutions[i], y_solutions[i], z_solutions[i], color=color_tracking[i])
#     ax4.set_title("Top")
#     ax4.set_xlabel("X")
#     ax4.set_ylabel("Y")
#     ax4.set_zlabel("Z")
#
#     # ax1.view_init(elev=0.0, azim=0.0)
#     ax2.view_init(elev=0.0, azim=-90.0)
#     ax3.view_init(elev=0.0, azim=0.0)
#     ax4.view_init(elev=90.0, azim=-90.0)
#
#     # xy_lim = 0.08
#     # z_lim = 0.05
#
#     # ax1.set_xlim3d(-xy_lim, xy_lim)
#     # ax1.set_ylim3d(-xy_lim, xy_lim)
#     # ax1.set_zlim3d(0, z_lim)
#     #
#     # ax2.set_xlim3d(-xy_lim, xy_lim)
#     # ax2.set_ylim3d(-xy_lim, xy_lim)
#     # ax2.set_zlim3d(0, z_lim)
#     #
#     # ax3.set_xlim3d(-xy_lim, xy_lim)
#     # ax3.set_ylim3d(-xy_lim, xy_lim)
#     # ax3.set_zlim3d(0, z_lim)
#     #
#     # ax4.set_xlim3d(-xy_lim, xy_lim)
#     # ax4.set_ylim3d(-xy_lim, xy_lim)
#     # ax4.set_zlim3d(0, z_lim)
#
#     plt.show()
#
#     return


def find_ws_one():
    delta = k.PDelta()

    # h1, h2, h3 = delta.go_to(0, 0, -0.05)
    # print(h1*1000, h2*1000, h3*1000)

    x, y, z = delta.go_to_distance(0, 0, 0)
    h1, h2, h3 = delta.go_to_position(0, 0, z)
    # h1, h2, h3 = delta.go_to_position(0, 0, z)
    print('xyz')
    print(x*1000, y*1000, z*1000)
    print('acts')
    print(h1*1000, h2*1000, h3*1000)

    motor1_range, motor2_range, motor3_range = ranges_full()

    x_solutions = []
    y_solutions = []
    z_solutions = []

    for dst1 in motor1_range:
        for dst2 in motor2_range:
            for dst3 in motor3_range:
                # print("# Distance: ", dst1, dst2, dst3)
                x0, y0, z0 = delta.go_to_distance(dst1, dst2, dst3)
                x_solutions.append(x0)
                y_solutions.append(y0)
                z_solutions.append(z0)

    x_solutions = np.array(x_solutions)
    y_solutions = np.array(y_solutions)
    z_solutions = np.array(z_solutions) # *-1
    # z_solutions -= np.min(z_solutions)

    print("Min/Max on x axis: ", np.min(x_solutions), np.max(x_solutions))
    print("Min/Max on y axis: ", np.min(y_solutions), np.max(y_solutions))
    print("Min/Max on z axis: ", np.min(z_solutions), np.max(z_solutions))

    draw_ws_one(x_solutions, y_solutions, z_solutions)


def data_for_cylinder_along_z(radius):
    z = np.linspace(-0.0455, -0.0555, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid)
    y_grid = radius*np.sin(theta_grid)
    return x_grid, y_grid, z_grid


def draw_ws_one(x_solutions, y_solutions, z_solutions):
    fig = plt.figure(figsize=(8, 8))

    ax1 = fig.add_subplot(projection='3d')

    scale = 100

    ax1.scatter(x_solutions*scale, y_solutions*scale, z_solutions*scale, label=r"$\textbf{Full}$")
    ax1.set_title(r"$\textbf{Workspace}$")
    ax1.set_xlabel(r"$\textbf{X (cm)}$")
    ax1.set_ylabel(r"$\textbf{Y (cm)}$")
    ax1.set_zlabel(r"$\textbf{Z (cm)}$")

    xy_lim = 0.03*scale
    ax1.set_xlim3d(-xy_lim, xy_lim)
    ax1.set_ylim3d(-xy_lim, xy_lim)
    ax1.set_zlim3d(-0.07*scale, -0.04*scale)

    xs, ys, zs = data_for_cylinder_along_z(0.0135)
    ax1.scatter(xs*scale, ys*scale, zs*scale, label=r"$\textbf{Restricted}$")

    # width_scale = 0.7
    # box = ax1.get_position()
    # ax1.set_position([box.x0, box.y0, box.width * width_scale, box.height])

    plt.legend()
    plt.savefig(f"./workspace.png", bbox_inches="tight", pad_inches=0.15, dpi=300)
    # plt.show()
    plt.close()


if __name__ == '__main__':
    # find_ws_all()
    find_ws_one()