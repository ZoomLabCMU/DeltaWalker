import taichi as ti
import time
import numpy as np
import sys

from DeltaWalker import DeltaWalkerEnv

# REF_PATH = ''  # TODO comment if necessary and replace '' with path to main folder
#  eg: "/User/NAME/Documents/deltawalker/"
# sys.path.append(REF_PATH)

import kinematics_walker as k

ti.init(arch=ti.gpu)


def teleop_xyz(walker, env, gui):
    cnt = 0

    print("Initializing...")
    # Initialize act distances to be 0.005 m extension and move it to that position
    start_act = 0.005
    traj = np.array([start_act, start_act, start_act,
                     start_act, start_act, start_act,
                     start_act, start_act, start_act,
                     start_act, start_act, start_act])
    env.move_joint_position([traj.tolist()], [1.0])

    # Get actuation amounts in correct format for FK
    act = [[traj[0], traj[1], traj[2]],
           [traj[3], traj[4], traj[5]],
           [traj[6], traj[7], traj[8]],
           [traj[9], traj[10], traj[11]]]

    # Get EE positions (4 rows of [x y z] - one for each delta)
    desired_pos = walker.go_to_FK(act)
    start_pos = np.copy(desired_pos)
    start_com = np.mean(np.array(start_pos), axis=0)[:2]

    # Teleoperation Loop
    print("Start teleoperating..")
    start = time.time()
    increment = 0.001
    while gui.running:
        for e in gui.get_events(gui.PRESS):
            # Shut down GUI------------------------------------------------------
            if e.key == gui.ESCAPE:
                gui.running = False
            # Increment foot position in one direction --------------------------
            # EE 1
            elif e.key == "q":
                desired_pos[0][0] -= increment  # -x
            elif e.key == "w":
                desired_pos[0][0] += increment  # +x
            elif e.key == "a":
                desired_pos[0][1] -= increment  # -y
            elif e.key == "s":
                desired_pos[0][1] += increment  # +y
            elif e.key == "z":
                desired_pos[0][2] -= increment  # -z
            elif e.key == "x":
                desired_pos[0][2] += increment  # +z
            # EE 2
            elif e.key == "e":
                desired_pos[1][0] -= increment  # -x
            elif e.key == "r":
                desired_pos[1][0] += increment  # +x
            elif e.key == "d":
                desired_pos[1][1] -= increment  # -y
            elif e.key == "f":
                desired_pos[1][1] += increment  # +y
            elif e.key == "c":
                desired_pos[1][2] -= increment  # -z
            elif e.key == "v":
                desired_pos[1][2] += increment  # +z
            # EE 3
            elif e.key == "t":
                desired_pos[2][0] -= increment  # -x
            elif e.key == "y":
                desired_pos[2][0] += increment  # +x
            elif e.key == "g":
                desired_pos[2][1] -= increment  # -y
            elif e.key == "h":
                desired_pos[2][1] += increment  # +y
            elif e.key == "b":
                desired_pos[2][2] -= increment  # -z
            elif e.key == "n":
                desired_pos[2][2] += increment  # +z
            # EE 4
            elif e.key == "u":
                desired_pos[3][0] -= increment  # -x
            elif e.key == "i":
                desired_pos[3][0] += increment  # +x
            elif e.key == "j":
                desired_pos[3][1] -= increment  # -y
            elif e.key == "k":
                desired_pos[3][1] += increment  # +y
            elif e.key == "m":
                desired_pos[3][2] -= increment  # -z
            elif e.key == "l":
                desired_pos[3][2] += increment  # +z

        # Get act positions based on desired position
        act = walker.go_to_IK(desired_pos)

        # Move to desired position based on act positions
        traj = np.array([act[0][0], act[0][1], act[0][2],
                         act[1][0], act[1][1], act[1][2],
                         act[2][0], act[2][1], act[2][2],
                         act[3][0], act[3][1], act[3][2]])
        env.move_joint_position([traj.tolist()], [1.0])

        # Update GUI
        draw_start_pos = np.array(start_pos)[:, 0:2] * 10 + 0.5
        draw_start_com = start_com.reshape((-1, 2)) * 10 + 0.5

        fk_pos = walker.go_to_FK(act)  # 4 x 3
        draw_fk_pos = np.array(fk_pos)[:, 0:2] * 10 + 0.5
        fk_com = np.mean(np.array(fk_pos), axis=0)[:2]
        draw_fk_com = fk_com.reshape((-1, 2)) * 10 + 0.5

        draw_desired_pos = np.array(desired_pos)[:, 0:2] * 10 + 0.5
        desired_com = np.mean(np.array(desired_pos), axis=0)[:2]
        draw_desired_com = desired_com.reshape((-1, 2)) * 10 + 0.5

        gui.circles(draw_start_pos, radius=20, color=0xFFAA33)
        gui.circles(draw_start_com, radius=20, color=0xFFAA33)
        gui.circles(draw_fk_pos, radius=15, color=0x9403fc)
        gui.circles(draw_fk_com, radius=15, color=0x9403fc)
        gui.circles(draw_desired_pos, radius=8, color=0x068587)
        gui.circles(draw_desired_com, radius=8, color=0x068587)
        gui.show()

        time.sleep(0.05)
        cnt += 1

    end = time.time()
    print("Time: ", end - start)


def teleop_deltas(walker, env, gui):
    cnt = 0

    print("Initializing...")
    # Initialize act distances to be 0.005 m extension and move it to that position
    start_act = 0.005
    traj = np.array([start_act, start_act, start_act,
                     start_act, start_act, start_act,
                     start_act, start_act, start_act,
                     start_act, start_act, start_act])
    env.move_joint_position([traj.tolist()], [1.0])

    # Get actuation amounts in correct format for FK
    act = [[traj[0], traj[1], traj[2]],
           [traj[3], traj[4], traj[5]],
           [traj[6], traj[7], traj[8]],
           [traj[9], traj[10], traj[11]]]

    # Get EE positions
    start_pos = walker.go_to_FK(act)
    start_com = np.mean(np.array(start_pos), axis=0)[:2]

    # Teleoperation Loop
    print("Start teleoperating..")
    start = time.time()
    increment = 0.001
    while gui.running:
        for e in gui.get_events(gui.PRESS):
            # Shut down GUI------------------------------------------------------
            if e.key == gui.ESCAPE:
                gui.running = False
            # Increment delta actuation in one direction --------------------------
            # Delta 1
            elif e.key == "q":
                act[0][0] += increment  # act0 extend
            elif e.key == "w":
                act[0][0] -= increment  # act0 retract
            elif e.key == "a":
                act[0][1] += increment  # act1 extend
            elif e.key == "s":
                act[0][1] -= increment  # act1 retract
            elif e.key == "z":
                act[0][2] += increment  # act2 extend
            elif e.key == "x":
                act[0][2] -= increment  # act2 retract
            # Delta 2
            elif e.key == "e":
                act[1][0] += increment  # act3 extend
            elif e.key == "r":
                act[1][0] -= increment  # act3 retract
            elif e.key == "d":
                act[1][1] += increment  # act4 extend
            elif e.key == "f":
                act[1][1] -= increment  # act4 retract
            elif e.key == "c":
                act[1][2] += increment  # act5 extend
            elif e.key == "v":
                act[1][2] -= increment  # act5 retract
            # Delta 3
            elif e.key == "t":
                act[2][0] += increment  # act6 extend
            elif e.key == "y":
                act[2][0] -= increment  # act6 retract
            elif e.key == "g":
                act[2][1] += increment  # act7 extend
            elif e.key == "h":
                act[2][1] -= increment  # act7 retract
            elif e.key == "b":
                act[2][2] += increment  # act8 extend
            elif e.key == "n":
                act[2][2] -= increment  # act8 retract
            # Delta 4
            elif e.key == "u":
                act[3][0] += increment  # act9 extend
            elif e.key == "i":
                act[3][0] -= increment  # act9 retract
            elif e.key == "j":
                act[3][1] += increment  # act10 extend
            elif e.key == "k":
                act[3][1] -= increment  # act10 retract
            elif e.key == "m":
                act[3][2] += increment  # act11 extend
            elif e.key == "l":
                act[3][2] -= increment  # act11 retract

        # Move to desired position based on act positions
        traj = np.array([act[0][0], act[0][1], act[0][2],
                         act[1][0], act[1][1], act[1][2],
                         act[2][0], act[2][1], act[2][2],
                         act[3][0], act[3][1], act[3][2]])
        env.move_joint_position([traj.tolist()], [1.0])

        # Update GUI
        draw_start_pos = np.array(start_pos)[:, 0:2] * 10 + 0.5
        draw_start_com = start_com.reshape((-1, 2)) * 10 + 0.5

        fk_pos = walker.go_to_FK(act)  # 4 x 3
        draw_fk_pos = np.array(fk_pos)[:, 0:2] * 10 + 0.5
        fk_com = np.mean(np.array(fk_pos), axis=0)[:2]
        draw_fk_com = fk_com.reshape((-1, 2)) * 10 + 0.5

        gui.circles(draw_start_pos, radius=20, color=0xFFAA33)
        gui.circles(draw_start_com, radius=20, color=0xFFAA33)
        gui.circles(draw_fk_pos, radius=15, color=0x9403fc)
        gui.circles(draw_fk_com, radius=15, color=0x9403fc)
        gui.show()

        time.sleep(0.05)
        cnt += 1

    end = time.time()
    print("Time: ", end - start)


if __name__ == '__main__':

    walker = k.DeltaWalkerKin()
    env = DeltaWalkerEnv('/dev/cu.usbmodem142401', 57600)
    gui = ti.GUI("EE pos visualizer", (1024, 1024))

    teleop_xyz(walker, env, gui)
    # teleop_deltas(walker, env, gui)


