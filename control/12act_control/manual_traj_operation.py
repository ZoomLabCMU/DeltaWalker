# import sys
# REF_PATH = ''  # TODO comment if necessary and replace '' with path to main folder
#  eg: "/User/NAME/Documents/deltawalker/"
# sys.path.append(REF_PATH)
from DeltaWalker import *
from trajectories import *
import helpers as h
import path_gen as g
import kinematics_walker as k
import ocrl_paths as o
from control_helpers import *


def switch(paths, case, step, cycle_steps, iters):
    # FB:   1 = q1, 2 = q2, 3 = q3, 4 = q4
    #       Note: We rotate the walker 45 degrees clockwise to see the robot move along one axis rather than between 2
    #       1 = +x, 2 = +y, 3 = -x, 4 = -y
    # FBS:  1 = +x, 2 = +y, 3 = -x, 4 = -y
    # fb v0 q1 - 4: Traj D 1-4
    # fb v1 q1 - 4: Traj E 1-4
    # fb v2 q1 - 4: Traj F 1-4
    # fbs v0 +- xy: Traj G 1-4
    # fbs v1 +- xy: Traj H 1-4
    # fbs v2 +- xy: Traj I 1-4
    step_part = step / np.sqrt(2)

    # These

    # FB V0 - ambl - previously noted as trajD
    if case == 0:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB0_t1_o1"
    elif case == 1:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB0_t1_o2"
    elif case == 2:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB0_t1_o3"
    elif case == 3:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB0_t1_o4"

    # FB V1 - ambl - previously noted as trajE
    elif case == 4:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB1_t1_o1"
    elif case == 5:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB1_t1_o2"
    elif case == 6:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB1_t1_o3"
    elif case == 7:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB1_t1_o4"

    # FB V2 - ambl - previously noted as trajF
    elif case == 8:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB2_t1_o1"
    elif case == 9:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB2_t1_o2"
    elif case == 10:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB2_t1_o3"
    elif case == 11:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB2_t1_o4"

    # FBS V0 - mtri  - previously noted as trajG
    elif case == 12:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v0(paths, 1, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC0_t1_o1"
    elif case == 13:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v0(paths, 2, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC0_t1_o2"
    elif case == 14:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v0(paths, 3, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC0_t1_o3"
    elif case == 15:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v0(paths, 4, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC0_t1_o4"

    # FBS V1 - mtri  - previously noted as trajH
    elif case == 16:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v1(paths, 1, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC1_t1_o1"
    elif case == 17:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v1(paths, 2, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC1_t1_o2"
    elif case == 18:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v1(paths, 3, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC1_t1_o3"
    elif case == 19:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v1(paths, 4, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC1_t1_o4"

    # FBS V2 - mtri - previously noted as trajI
    elif case == 20:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v2(paths, 1, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC2_t1_o1"
    elif case == 21:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v2(paths, 2, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC2_t1_o2"
    elif case == 22:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v2(paths, 3, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC2_t1_o3"
    elif case == 23:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v2(paths, 4, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC2_t1_o4"

    # FB VO - walk - previously noted as trajJ
    elif case == 24:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA0_t1_o1"
    elif case == 25:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA0_t1_o2"
    elif case == 26:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA0_t1_o3"
    elif case == 27:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA0_t1_o4"

    # FB V1 - walk - previously noted as trajK
    elif case == 28:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA1_t1_o1"
    elif case == 29:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA1_t1_o2"
    elif case == 30:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA1_t1_o3"
    elif case == 31:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA1_t1_o4"

    # FB V2 - walk - previously noted as trajL
    elif case == 32:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA2_t1_o1"
    elif case == 33:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA2_t1_o2"
    elif case == 34:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA2_t1_o3"
    elif case == 35:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA2_t1_o4"

    return all_pos, all_acts, all_pos_fk, steps, gait_name, traj_name


if __name__ == '__main__':
    walker = k.DeltaWalkerKin()
    paths = g.GenPaths(walker)

    print("Connecting to Robot")
    # TODO change usb port
    env = DeltaWalkerEnv('/dev/cu.usbmodem143101', 9600)
    print("Robot Connected")
    env.start()
    print("Robot Started")
    env.reset()
    print("Robot Reset")

    # TODO toggle to choose which trajectory
    case = 33

    # TODO toggle to choose how many iterations
    iters = 3

    # TODO toggle to choose how many steps to send the robot each segment
    cycle_steps = 3

    # TODO toggle to choose step size
    step = 0.01

    # Pulling the trajectory
    all_pos, all_acts, all_pos_fk, steps, gait_name, traj_name = switch(paths, case, step, cycle_steps, iters)
    motor_ranges = paths.acts_to_motors(all_acts, steps)
    savename = f"{traj_name}_{iters}x"

    print(savename)
    print("Steps ", steps)

    act_traj_all(env, motor_ranges, steps)

    recorder(savename, "0_manual_design")
    # observation_cam()
    print("Reset and Close")
    env.reset()
    env.close()
    print("Done")




