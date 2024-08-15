import time
from DeltaControl import DeltaRobotEnv
import numpy as np

ACT_HEIGHT = 0.01


def one_command(base_motion, act_id, act_bool):
    if act_bool[act_id]:
        print("Resetting", act_id)
        base_motion[0][act_id] = 0
    else:
        print("Actuating", act_id)
        base_motion[0][act_id] = ACT_HEIGHT
    act_bool[act_id] = not act_bool[act_id]

    return base_motion, act_bool


def one_delta(base_motion, act_bool, act_ids, mid_bool, delta_bool):
    if mid_bool:
        print("Including Middle")
        act_ids = [0] + act_ids
    else:
        print("Excluding Middle")

    if delta_bool:
        print("Actuating")
        for id in act_ids:
            base_motion[0][id] = ACT_HEIGHT
            act_bool[id] = True
    else:
        print('Resetting')
        for id in act_ids:
            base_motion[0][id] = 0
            act_bool[id] = False

    return base_motion, act_bool


def continuous_command(env):
    exit_command = False
    base_motion = [[0] * 9]

    act_bool = [False] * 9

    delta_commands = ["0_n_a", "0_n_r", "0_y_a", "0_y_r",
                      "1_n_a", "1_n_r", "1_y_a", "1_y_r",
                      "2_n_a", "2_n_r", "2_y_a", "2_y_r",
                      "3_n_a", "3_n_r", "3_y_a", "3_y_r"]

    delta_acts = [[1, 8], [2, 3], [4, 5], [6, 7]]

    skip_act = False

    while not exit_command:
        choice = input("Enter a command for actuation. \n")
        # choice = int(choice)

        if choice == 'exit':
            exit_command = True

        elif choice == 'reset':
            print("Resetting all")
            base_motion = [[0] * 9]
            act_bool = [False] * 9

        elif choice == 'all':
            print("Actuating all")
            base_motion = [[ACT_HEIGHT] * 9]
            act_bool = [True] * 9

        elif choice in np.arange(9).astype(str):
            act_id = int(choice)
            base_motion, act_bool = one_command(base_motion, act_id, act_bool)

        elif choice == 'invert_mid':
            print("Inverting Position")
            for i in np.arange(9):
                base_motion, act_bool = one_command(base_motion, i, act_bool)

        elif choice == 'invert_no_mid':
            print("Inverting Position Without Middle")
            for i in np.arange(1, 9):
                base_motion, act_bool = one_command(base_motion, i, act_bool)

        elif choice == 'odd_no_mid':
            print("Odd actuators only")
            for i in np.arange(1, 9, 2):
                base_motion, act_bool = one_command(base_motion, i, act_bool)

        elif choice == 'even_no_mid':
            print("Even actuators only")
            for i in np.arange(2, 9, 2):
                base_motion, act_bool = one_command(base_motion, i, act_bool)

        elif choice == 'odd_mid':
            print("Odd actuators only")
            base_motion, act_bool = one_command(base_motion, 0, act_bool)
            for i in np.arange(1, 9, 2):
                base_motion, act_bool = one_command(base_motion, i, act_bool)

        elif choice == 'even_mid':
            print("Even actuators only")
            for i in np.arange(0, 9, 2):
                base_motion, act_bool = one_command(base_motion, i, act_bool)

        elif choice in delta_commands:
            choice_split = choice.split("_")
            act_ids = delta_acts[int(choice_split[0])]
            mid_bool = True if choice_split[1] == 'y' else False
            delta_bool = True if choice_split[2] == 'a' else False
            base_motion, act_bool = one_delta(base_motion, act_bool, act_ids, mid_bool, delta_bool)

        else:
            skip_act = True

        if not exit_command:
            if skip_act:
                skip_act = False
            else:
                env.actuate_tridelta(base_motion)

    print("Exiting")
    return


def circle(env):
    base_motion = [[ACT_HEIGHT] * 9]
    base_motion[0][1] = 0
    env.actuate_tridelta(base_motion)
    for j in range(1):
        for i in range(9):
            base_motion[0][i] = ACT_HEIGHT
            base_motion[0][i + 1] = 0
            env.actuate_tridelta(base_motion)
    # env.reset()


def simple_walk(env):
    bot = 0
    mid = 0.009
    top = 0.018

    for i in np.arange(3):
        print("go to mid")
        base_motion = [[mid]*9]
        print(base_motion)
        env.actuate_tridelta(base_motion)

        print("go to step")
        base_motion = [[top, bot, bot, top, top, top, top, mid, mid]]
        print(base_motion)
        env.actuate_tridelta(base_motion)

        # print("back to mid")
        # base_motion = [[mid] * 9]
        # env.actuate_tridelta(base_motion)

        print(f"step {i} done")

    base_motion = [[mid] * 9]
    print(base_motion)
    env.actuate_tridelta(base_motion)


if __name__ == "__main__":
    print("initializing")
    env = DeltaRobotEnv()
    print("resetting")
    env.reset()
    print("done resetting")
    # continuous_command(env)
    # circle(env)
    # simple_walk(env)

    env.shut_down()