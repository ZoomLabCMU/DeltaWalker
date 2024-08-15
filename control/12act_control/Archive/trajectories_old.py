import numpy as np

none_ext = 0.019
base_ext = 0.015
full_ext = 0.01
part_ext = 0.0125

cycle_steps = 3


def one_step_no_shift(path):  # Only step delta 1, no shift back to 0
    x = 0.005
    y = 0.005

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    c0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    c1 = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    c2 = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    c3 = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)

    d1 = np.vstack((c0, c1, c2, c3))

    d2_0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    d2_1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 3, 1)
    d3_1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 3, 2)
    d4_1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 3, 3)

    d2 = np.vstack((d2_0, d2_1))
    d3 = np.vstack((d3_0, d3_1))
    d4 = np.vstack((d4_0, d4_1))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "one_step_no_shift"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def one_step_weight_shift(path):  # Only step delta 1 and shift back to 0
    x = 0.005
    y = 0.005

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    c0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    c1 = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    c2 = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    c3 = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    c4 = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)

    d1 = np.vstack((c0, c1, c2, c3, c4))

    d2_0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    d2_1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 4, 1)
    d3_1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 4, 2)
    d4_1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 4, 3)

    d2 = np.vstack((d2_0, d2_1))
    d3 = np.vstack((d3_0, d3_1))
    d4 = np.vstack((d4_0, d4_1))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "one_step_weight_shift"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def step_all(path):
    x = 0.005
    y = 0.005

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    all_pos_init = path.gen_all_traj_pos(home_none, home_full, cycle_steps)
    all_pos_step = path.gen_all_traj_pos(home_full, step_base, cycle_steps)
    all_pos_zero = path.gen_all_traj_pos(step_base, home_base, cycle_steps)

    all_pos = np.vstack((all_pos_init, all_pos_step, all_pos_zero))
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "step_all"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_no_shift(path):  # Walk
    x = 0.005
    y = 0.005

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    d1_c0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps*2, 0)
    d1_c1 = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d1_c2 = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d1_stay1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 2, 0)
    d1_stay2 = path.gen_one_traj_pos(step_base, step_base, cycle_steps * 2, 0)

    d2_c0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps*2, 1)
    d2_c1 = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d2_c2 = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d2_stay1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 2, 1)
    d2_stay2 = path.gen_one_traj_pos(step_base, step_base, cycle_steps * 2, 1)

    d3_c0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps*2, 2)
    d3_c1 = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d3_c2 = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d3_stay1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 2, 2)
    d3_stay2 = path.gen_one_traj_pos(step_base, step_base, cycle_steps * 2, 2)

    d4_c0 = path.gen_one_traj_pos(home_none, home_base, cycle_steps*2, 3)
    d4_c1 = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)
    d4_c2 = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)
    d4_stay1 = path.gen_one_traj_pos(home_base, home_base, cycle_steps * 2, 3)
    d4_stay2 = path.gen_one_traj_pos(step_base, step_base, cycle_steps * 2, 3)

    d1 = np.vstack((d1_c0, d1_c1, d1_c2, d1_stay2, d1_stay2, d1_stay2, d1_stay2))
    d2 = np.vstack((d2_c0, d2_stay1, d2_c1, d2_c2, d2_stay2, d2_stay2, d2_stay2))
    d3 = np.vstack((d3_c0, d3_stay1, d3_stay1, d3_c1, d3_c2, d3_stay2, d3_stay2))
    d4 = np.vstack((d4_c0, d4_stay1, d4_stay1, d4_stay1, d4_c1, d4_c2, d4_stay2))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_no_shift"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def traj_segments(path, delta):
    x = 0.005
    y = 0.005

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Vertical push to move the robot down in z
    push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, delta)

    # Vertical lift to move the robot up in z
    lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, delta)

    # Step in xy direction with down in z
    step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, delta)

    # Reset actuation to xy pos = 0, no change in z
    zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, delta)

    # Wait at base, no change in x y or z
    wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, delta)

    # Stay at step position, no change in x y or z
    stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, delta)

    return push, lift, step, zero, wait, stay


def full_traj_walk_v0_xy(path, num_iters=1):
    x = 0.01
    y = 0.01

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------

    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step))

    for i in np.arange(num_iters+1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v0_xy"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_v1_xy(path, num_iters=1):
    x = 0.01
    y = 0.01

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Stay at step, no change in x y or z
    d1_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    d2_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 1)
    d3_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 2)
    d4_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------
    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v1_xy"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_v2_xy(path, num_iters=1):
    x = 0.01
    y = 0.01

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Stay at step, no change in x y or z
    d1_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    d2_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 1)
    d3_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 2)
    d4_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------

    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_stay, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_stay, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_stay, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_stay, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_stay, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_stay, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_stay, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_stay))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v2_xy"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_v0_x(path, num_iters=1):
    x = 0.01
    y = 0

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------

    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step))

    for i in np.arange(num_iters+1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v0_x"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_v1_x(path, num_iters=1):
    x = 0.01
    y = 0

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Stay at step, no change in x y or z
    d1_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    d2_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 1)
    d3_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 2)
    d4_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------
    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v1_x"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_v2_x(path, num_iters=1):
    x = 0.01
    y = 0

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Stay at step, no change in x y or z
    d1_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    d2_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 1)
    d3_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 2)
    d4_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------

    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_stay, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_stay, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_stay, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_stay, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_stay, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_stay, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_stay, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_stay))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v2_x"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_v0_y(path, num_iters=1):
    x = 0
    y = 0.01

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------

    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step))

    for i in np.arange(num_iters+1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v0_y"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_v1_y(path, num_iters=1):
    x = 0
    y = 0.01

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Stay at step, no change in x y or z
    d1_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    d2_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 1)
    d3_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 2)
    d4_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------
    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v1_y"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def full_traj_walk_v2_y(path, num_iters=1):
    x = 0
    y = 0.01

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Stay at step, no change in x y or z
    d1_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    d2_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 1)
    d3_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 2)
    d4_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 3)

    # Trajectory Generation --------------------------------------------------------------------------------------------

    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_stay, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_stay, d2_zero, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lift, d3_step, d3_stay, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait, d3_wait))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_stay, d4_zero))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_stay, d1_stay, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_wait, d2_wait, d2_lift, d2_step, d2_stay, d2_stay, d2_zero, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lift, d3_step, d3_stay, d3_stay, d3_zero, d3_wait, d3_wait, d3_wait))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_stay, d4_stay))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d3 = np.vstack((d3, d3_wait))

        if i == 0:
            d4 = np.vstack((d4, d4_wait))
        else:
            d4 = np.vstack((d4, d4_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "full_traj_walk_v2_y"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def walk_fbs_v0_x(path, num_iters=1):
    x = 0.01
    y = 0

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    lean_none = [-x, -y, none_ext]
    lean_base = [-x, -y, base_ext]
    lean_full = [-x, -y, full_ext]
    lean_part = [-x, -y, part_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Stay at step, no change in x y or z
    d1_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    d2_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 1)
    d3_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 2)
    d4_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 3)

    # Lean forward with d3
    d3_lnfw = path.gen_one_traj_pos(home_base, lean_full, cycle_steps, 2)

    # Wait at lean forward position for d3
    d3_lnwt = path.gen_one_traj_pos(lean_full, lean_full, cycle_steps, 2)

    # Move from lean position to pre step for d3
    d3_lnst = path.gen_one_traj_pos(lean_full, home_none, cycle_steps, 2)

    # Trajectory Generation --------------------------------------------------------------------------------------------

    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_lift, d2_step, d2_zero, d2_wait, d2_wait, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lnfw, d3_lnwt, d3_lnwt, d3_lnwt, d3_lnst, d3_step, d3_zero))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_zero, d4_wait, d4_wait))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_lift, d2_step, d2_zero, d2_wait, d2_wait, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lnfw, d3_lnwt, d3_lnwt, d3_lnwt, d3_lnst, d3_step))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_zero, d4_wait))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d4 = np.vstack((d4, d4_wait))

        if i == 0:
            d3 = np.vstack((d3, d3_wait))
        else:
            d3 = np.vstack((d3, d3_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "walk_fbs_v0_x"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def walk_fbs_v1_x(path, num_iters=1):
    x = 0.01
    y = 0

    home_none = [0, 0, none_ext]
    home_base = [0, 0, base_ext]
    home_full = [0, 0, full_ext]

    step_none = [x, y, none_ext]
    step_base = [x, y, base_ext]
    step_full = [x, y, full_ext]

    lean_none = [-x, -y, none_ext]
    lean_base = [-x, -y, base_ext]
    lean_full = [-x, -y, full_ext]
    lean_part = [-x, -y, part_ext]

    # Trajectory Segments ----------------------------------------------------------------------------------------------

    # Vertical push to move the robot down in z
    d1_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4_push = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    # Vertical lift to move the robot up in z
    d1_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 0)
    d2_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 1)
    d3_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 2)
    d4_lift = path.gen_one_traj_pos(home_base, home_none, cycle_steps, 3)

    # Step in xy direction with down in z
    d1_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 0)
    d2_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 1)
    d3_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 2)
    d4_step = path.gen_one_traj_pos(home_none, step_base, cycle_steps, 3)

    # Reset actuation to xy pos = 0, no change in z
    d1_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 0)
    d2_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 1)
    d3_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 2)
    d4_zero = path.gen_one_traj_pos(step_base, home_base, cycle_steps, 3)

    # Wait at base, no change in x y or z
    d1_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 0)
    d2_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 1)
    d3_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 2)
    d4_wait = path.gen_one_traj_pos(home_base, home_base, cycle_steps, 3)

    # Stay at step, no change in x y or z
    d1_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 0)
    d2_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 1)
    d3_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 2)
    d4_stay = path.gen_one_traj_pos(step_base, step_base, cycle_steps, 3)

    # Lean forward with d3
    d3_lnfw = path.gen_one_traj_pos(home_base, lean_part, cycle_steps, 2)

    # Wait at lean forward position for d3
    d3_lnwt = path.gen_one_traj_pos(lean_part, lean_part, cycle_steps, 2)

    # Move from lean position to pre step for d3
    d3_lnst = path.gen_one_traj_pos(lean_part, home_none, cycle_steps, 2)

    # Trajectory Generation --------------------------------------------------------------------------------------------

    # d1 = np.vstack((d1_push, d1_lift, d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    # d2 = np.vstack((d2_push, d2_wait, d2_wait, d2_lift, d2_step, d2_zero, d2_wait, d2_wait, d2_wait, d2_wait))
    # d3 = np.vstack((d3_push, d3_wait, d3_wait, d3_lnfw, d3_lnwt, d3_lnwt, d3_lnwt, d3_lnst, d3_step, d3_zero))
    # d4 = np.vstack((d4_push, d4_wait, d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_zero, d4_wait, d4_wait))

    d1 = d1_push
    d2 = d2_push
    d3 = d3_push
    d4 = d4_push

    d1_traj = np.vstack((d1_step, d1_zero, d1_wait, d1_wait, d1_wait, d1_wait, d1_wait))
    d2_traj = np.vstack((d2_wait, d2_lift, d2_step, d2_zero, d2_wait, d2_wait, d2_wait))
    d3_traj = np.vstack((d3_wait, d3_lnfw, d3_lnwt, d3_lnwt, d3_lnwt, d3_lnst, d3_step))
    d4_traj = np.vstack((d4_wait, d4_wait, d4_wait, d4_lift, d4_step, d4_zero, d4_wait))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            d1 = np.vstack((d1, d1_lift))
        else:
            d1 = np.vstack((d1, d1_wait))

        d2 = np.vstack((d2, d2_wait))
        d4 = np.vstack((d4, d4_wait))

        if i == 0:
            d3 = np.vstack((d3, d3_wait))
        else:
            d3 = np.vstack((d3, d3_zero))

        if i < num_iters:
            d1 = np.vstack((d1, d1_traj))
            d2 = np.vstack((d2, d2_traj))
            d3 = np.vstack((d3, d3_traj))
            d4 = np.vstack((d4, d4_traj))

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = "walk_fbs_v1_x"

    return all_pos, all_acts, all_pos_fk, steps, gait_name


def get_all_combos(path, x, y):
    increment = -0.003
    none, base, part, full = np.round(np.arange(0.019, 0.009, increment), 4)
    none = 0.019
    base = 0.015
    part = 0.013
    full = 0.011

    deltas = [{}, {}, {}, {}]

    home_none = [0, 0, none]
    home_base = [0, 0, base]
    home_part = [0, 0, part]
    home_full = [0, 0, full]

    step_none = [x, y, none]
    step_base = [x, y, base]
    step_part = [x, y, part]
    step_full = [x, y, full]

    lean_none = [-x, -y, none]
    lean_base = [-x, -y, base]
    lean_part = [-x, -y, part]
    lean_full = [-x, -y, full]

    # Each movement is for 1 foot only
    for i in np.arange(4):
        dict = deltas[i]

        # Push - Push foot down to home base
        dict['push_A'] = path.gen_one_traj_pos(home_none, home_base, cycle_steps, i)  # home none -> home base

        # Lift - Lift foot up to home_none to prepare for step
        dict['lift_A'] = path.gen_one_traj_pos(home_base, home_none, cycle_steps, i)  # home base -> home none
        dict['lift_B'] = path.gen_one_traj_pos(lean_part, home_none, cycle_steps, i)  # lean part -> home none
        dict['lift_C'] = path.gen_one_traj_pos(lean_full, home_none, cycle_steps, i)  # lean full -> home none

        # Step - Step in xy direction while moving z down
        dict['step_A'] = path.gen_one_traj_pos(home_none, step_base, cycle_steps, i)  # Step down (dx, dy, -z)

        # Zero - Reset to home_base
        dict['zero_A'] = path.gen_one_traj_pos(step_base, home_base, cycle_steps, i)  # Reset on plane (dx, dy)

        # Wait - Wait at current position
        dict['wait_A'] = path.gen_one_traj_pos(home_base, home_base, cycle_steps, i)  # Wait at home_base
        dict['wait_B'] = path.gen_one_traj_pos(lean_part, lean_part, cycle_steps, i)  # Wait at lean_part
        dict['wait_C'] = path.gen_one_traj_pos(lean_full, lean_full, cycle_steps, i)  # Wait at lean_full
        dict['wait_D'] = path.gen_one_traj_pos(step_base, step_base, cycle_steps, i)  # Wait at step_base

        # Lean - Lean robot forward from home_base
        dict['lean_A'] = path.gen_one_traj_pos(home_base, lean_base, cycle_steps, i)  # Go to lean base
        dict['lean_B'] = path.gen_one_traj_pos(home_base, lean_part, cycle_steps, i)  # Go to lean part
        dict['lean_C'] = path.gen_one_traj_pos(home_base, lean_full, cycle_steps, i)  # Go to lean full

    return deltas


def walk_fbs_v2(path, orientation, num_iters):
    # order = [F, B, L, R]
    step = 0.01
    x = 0
    y = 0
    if orientation == 1:
        order = np.array([1, 3, 2, 4]) - 1
        x = step
        dir = "px"
    elif orientation == 2:
        order = np.array([2, 4, 3, 1]) - 1
        y = step
        dir = "py"
    elif orientation == 3:
        order = np.array([3, 1, 4, 2]) - 1
        x = -step
        dir = "nx"
    else:
        order = np.array([4, 2, 1, 3]) - 1
        y = -step
        dir = "ny"

    deltas = get_all_combos(path, x, y)

    dF_dict = deltas[order[0]]
    dB_dict = deltas[order[1]]
    dL_dict = deltas[order[2]]
    dR_dict = deltas[order[3]]

    dF = dF_dict['push_A']
    dB = dB_dict['push_A']
    dL = dL_dict['push_A']
    dR = dR_dict['push_A']

    dF_traj = np.vstack((dF_dict['step_A'], dF_dict['zero_A'], dF_dict['wait_A'], dF_dict['wait_A'], dF_dict['wait_A'], dF_dict['wait_A'], dF_dict['wait_A']))
    dB_traj = np.vstack((dB_dict['wait_A'], dB_dict['lean_C'], dB_dict['wait_C'], dB_dict['wait_C'], dB_dict['wait_C'], dB_dict['lift_C'], dB_dict['step_A']))
    dL_traj = np.vstack((dL_dict['wait_A'], dL_dict['lean_B'], dL_dict['lift_B'], dL_dict['step_A'], dL_dict['zero_A'], dL_dict['wait_A'], dL_dict['wait_A']))
    dR_traj = np.vstack((dR_dict['wait_A'], dR_dict['lean_B'], dR_dict['wait_B'], dR_dict['lift_B'], dR_dict['step_A'], dR_dict['zero_A'], dR_dict['wait_A']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dF = np.vstack((dF, dF_dict['lift_A']))
        else:
            dF = np.vstack((dF, dF_dict['wait_A']))

        dL = np.vstack((dL, dL_dict['wait_A']))
        dR = np.vstack((dR, dR_dict['wait_A']))

        if i == 0:
            dB = np.vstack((dB, dB_dict['wait_A']))
        else:
            dB = np.vstack((dB, dB_dict['zero_A']))

        if i < num_iters:
            dF = np.vstack((dF, dF_traj))
            dB = np.vstack((dB, dB_traj))
            dL = np.vstack((dL, dL_traj))
            dR = np.vstack((dR, dR_traj))

    if orientation == 1:
        all_pos = path.combine_traj_pos(dF, dL, dB, dR)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dR, dF, dL, dB)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dB, dR, dF, dL)
    else:
        all_pos = path.combine_traj_pos(dL, dB, dR, dF)

    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)

    gait_name = f"walk_fbs_v2_{dir}"

    return all_pos, all_acts, all_pos_fk, steps, gait_name

    # TODO fix this wahhh


def gen_dicts(path, x, y, cycle_steps, none=0.0195, base=0.015, part=0.0125, full=0.010):
    deltas = [{}, {}, {}, {}]

    home_none = [0, 0, none]
    home_base = [0, 0, base]
    home_part = [0, 0, part]
    home_full = [0, 0, full]

    step_none = [x, y, none]
    step_base = [x, y, base]
    step_part = [x, y, part]
    step_full = [x, y, full]

    lean_none = [-x, -y, none]
    lean_base = [-x, -y, base]
    lean_part = [-x, -y, part]
    lean_full = [-x, -y, full]

    # Each movement is for 1 foot only
    for i in np.arange(4):
        dict = deltas[i]

        # Push - Push foot down to home base
        dict['push_A'] = path.gen_one_traj_pos(home_none, home_base, cycle_steps, i)  # home none -> home base

        # Lift - Lift foot up to home_none to prepare for step
        dict['lift_A'] = path.gen_one_traj_pos(home_base, home_none, cycle_steps, i)  # home base -> home none
        dict['lift_B'] = path.gen_one_traj_pos(lean_part, home_none, cycle_steps, i)  # lean part -> home none
        dict['lift_C'] = path.gen_one_traj_pos(lean_full, home_none, cycle_steps, i)  # lean full -> home none

        # Step - Step in xy direction while moving z down
        dict['step_A'] = path.gen_one_traj_pos(home_none, step_base, cycle_steps, i)  # Step down (dx, dy, -z)

        # Zero - Reset to home_base
        dict['zero_A'] = path.gen_one_traj_pos(step_base, home_base, cycle_steps, i)  # Reset on plane (dx, dy)

        # Wait - Wait at current position
        dict['wait_A'] = path.gen_one_traj_pos(home_base, home_base, cycle_steps, i)  # Wait at home_base
        dict['wait_B'] = path.gen_one_traj_pos(lean_part, lean_part, cycle_steps, i)  # Wait at lean_part
        dict['wait_C'] = path.gen_one_traj_pos(lean_full, lean_full, cycle_steps, i)  # Wait at lean_full
        dict['wait_D'] = path.gen_one_traj_pos(step_base, step_base, cycle_steps, i)  # Wait at step_base

        # Lean - Lean robot forward from home_base
        dict['lean_A'] = path.gen_one_traj_pos(home_base, lean_base, cycle_steps, i)  # Go to lean base
        dict['lean_B'] = path.gen_one_traj_pos(home_base, lean_part, cycle_steps, i)  # Go to lean part
        dict['lean_C'] = path.gen_one_traj_pos(home_base, lean_full, cycle_steps, i)  # Go to lean full

    return deltas


def debug_path(path, cycle_steps=5, num_iters=1):
    deltas = gen_dicts(path, 0, 0, cycle_steps)
    d1 = deltas[0]
    d2 = deltas[1]
    d3 = deltas[2]
    d4 = deltas[3]

    d1_path = np.vstack((d1['push_A'], d1['wait_A'], d1['lift_A'], d1['step_A'], d1['wait_A'], d1['wait_A'], d1['wait_A'], d1['wait_A'], d1['wait_A'], d1['wait_A'], d1['wait_A']))
    d2_path = np.vstack((d2['push_A'], d2['wait_A'], d2['wait_A'], d2['wait_A'], d2['lift_A'], d2['step_A'], d2['wait_A'], d2['wait_A'], d2['wait_A'], d2['wait_A'], d2['wait_A']))
    d3_path = np.vstack((d3['push_A'], d3['wait_A'], d3['wait_A'], d3['wait_A'], d3['wait_A'], d3['wait_A'], d3['lift_A'], d3['step_A'], d3['wait_A'], d3['wait_A'], d3['wait_A']))
    d4_path = np.vstack((d4['push_A'], d4['wait_A'], d4['wait_A'], d4['wait_A'], d4['wait_A'], d4['wait_A'], d4['wait_A'], d4['wait_A'], d4['lift_A'], d4['step_A'], d4['wait_A']))

    all_pos = path.combine_traj_pos(d1_path, d2_path, d3_path, d4_path)
    all_acts, all_pos_fk, steps = path.gen_all_steps(all_pos)
    gait_name = f"debug"

    return all_pos, all_acts, all_pos_fk, steps, gait_name