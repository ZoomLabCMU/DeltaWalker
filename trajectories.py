import numpy as np


def gen_pushes(path):
    cycle_steps = 3
    home_none = [0, 0, 0.0195]
    home_base = [0, 0, 0.015]

    d1 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 0)
    d2 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 1)
    d3 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 2)
    d4 = path.gen_one_traj_pos(home_none, home_base, cycle_steps, 3)

    all_pos = path.combine_traj_pos(d1, d2, d3, d4)

    return all_pos, cycle_steps


def gen_dicts(path, x, y, cycle_steps, none=0.0195, base=0.015, part=0.013, full=0.011):
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
        dict['name'] = f"delta{i + 1}"

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


def ambl_v0(path, x, y, cycle_steps=3, num_iters=1):
    # order = [FL, FR, BL, BR]
    # x and y are the directionality components

    if y >= 0:
        if x >= 0:
            order = np.array([2, 1, 3, 4]) - 1
            orientation = 1
        else:
            order = np.array([3, 2, 4, 1]) - 1
            orientation = 2
    else:
        if x < 0:
            order = np.array([4, 3, 1, 2]) - 1
            orientation = 3
        else:
            order = np.array([1, 4, 2, 3]) - 1
            orientation = 4

    deltas = gen_dicts(path, x, y, cycle_steps)

    dFL = deltas[order[0]]
    dFR = deltas[order[1]]
    dBL = deltas[order[2]]
    dBR = deltas[order[3]]

    dFL_traj = np.vstack((dFL['push_A'], dFL['wait_A']))
    dFR_traj = np.vstack((dFR['push_A'], dFR['wait_A']))
    dBL_traj = np.vstack((dBL['push_A'], dBL['wait_A']))
    dBR_traj = np.vstack((dBR['push_A'], dBR['wait_A']))

    # Step order: FR, BL, FL, BR
    dFR_rep = np.vstack(
        (dFR['step_A'], dFR['zero_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A']))
    dBL_rep = np.vstack(
        (dBL['wait_A'], dBL['lift_A'], dBL['step_A'], dBL['zero_A'], dBL['wait_A'], dBL['wait_A'], dBL['wait_A']))
    dFL_rep = np.vstack(
        (dFL['wait_A'], dFL['wait_A'], dFL['wait_A'], dFL['lift_A'], dFL['step_A'], dFL['zero_A'], dFL['wait_A']))
    dBR_rep = np.vstack(
        (dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['lift_A'], dBR['step_A']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dFR_traj = np.vstack((dFR_traj, dFR['lift_A']))
        else:
            dFR_traj = np.vstack((dFR_traj, dFR['wait_A']))

        dBL_traj = np.vstack((dBL_traj, dBL['wait_A']))
        dFL_traj = np.vstack((dFL_traj, dFL['wait_A']))

        if i == 0:
            dBR_traj = np.vstack((dBR_traj, dBR['wait_A']))
        else:
            dBR_traj = np.vstack((dBR_traj, dBR['zero_A']))

        if i < num_iters:
            dFR_traj = np.vstack((dFR_traj, dFR_rep))
            dBL_traj = np.vstack((dBL_traj, dBL_rep))
            dFL_traj = np.vstack((dFL_traj, dFL_rep))
            dBR_traj = np.vstack((dBR_traj, dBR_rep))

    # order = [FL, FR, BL, BR]
    if orientation == 1:
        all_pos = path.combine_traj_pos(dFR_traj, dFL_traj, dBL_traj, dBR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dBR_traj, dFR_traj, dFL_traj, dBL_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dBL_traj, dBR_traj, dFR_traj, dFL_traj)
    else:
        all_pos = path.combine_traj_pos(dFL_traj, dBL_traj, dBR_traj, dFR_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)
    gait_name = f"mtri_v0_o{orientation}"
    save_name = f"B0_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


def ambl_v1(path, x, y, cycle_steps=3, num_iters=1):
    # order = [FL, FR, BL, BR]
    # x and y are the directionality components

    if y >= 0:
        if x >= 0:
            order = np.array([2, 1, 3, 4]) - 1
            orientation = 1
        else:
            order = np.array([3, 2, 4, 1]) - 1
            orientation = 2
    else:
        if x < 0:
            order = np.array([4, 3, 1, 2]) - 1
            orientation = 3
        else:
            order = np.array([1, 4, 2, 3]) - 1
            orientation = 4

    deltas = gen_dicts(path, x, y, cycle_steps)

    dFL = deltas[order[0]]
    dFR = deltas[order[1]]
    dBL = deltas[order[2]]
    dBR = deltas[order[3]]

    dFL_traj = np.vstack((dFL['push_A'], dFL['wait_A']))
    dFR_traj = np.vstack((dFR['push_A'], dFR['wait_A']))
    dBL_traj = np.vstack((dBL['push_A'], dBL['wait_A']))
    dBR_traj = np.vstack((dBR['push_A'], dBR['wait_A']))

    # Step order: FR, BL, FL, BR
    dFR_rep = np.vstack((dFR['step_A'], dFR['wait_D'], dFR['zero_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'],
                         dFR['wait_A'], dFR['wait_A']))
    dBL_rep = np.vstack((dBL['wait_A'], dBL['lift_A'], dBL['step_A'], dBL['wait_D'], dBL['zero_A'], dBL['wait_A'],
                         dBL['wait_A'], dBL['wait_A']))
    dFL_rep = np.vstack((dFL['wait_A'], dFL['wait_A'], dFL['wait_A'], dFL['lift_A'], dFL['step_A'], dFL['wait_D'],
                         dFL['zero_A'], dFL['wait_A']))
    dBR_rep = np.vstack((dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['lift_A'],
                         dBR['step_A'], dBR['wait_D']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dFR_traj = np.vstack((dFR_traj, dFR['lift_A']))
        else:
            dFR_traj = np.vstack((dFR_traj, dFR['wait_A']))

        dBL_traj = np.vstack((dBL_traj, dBL['wait_A']))
        dFL_traj = np.vstack((dFL_traj, dFL['wait_A']))

        if i == 0:
            dBR_traj = np.vstack((dBR_traj, dBR['wait_A']))
        else:
            dBR_traj = np.vstack((dBR_traj, dBR['zero_A']))

        if i < num_iters:
            dFR_traj = np.vstack((dFR_traj, dFR_rep))
            dBL_traj = np.vstack((dBL_traj, dBL_rep))
            dFL_traj = np.vstack((dFL_traj, dFL_rep))
            dBR_traj = np.vstack((dBR_traj, dBR_rep))

    # order = [FL, FR, BL, BR]
    if orientation == 1:
        all_pos = path.combine_traj_pos(dFR_traj, dFL_traj, dBL_traj, dBR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dBR_traj, dFR_traj, dFL_traj, dBL_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dBL_traj, dBR_traj, dFR_traj, dFL_traj)
    else:
        all_pos = path.combine_traj_pos(dFL_traj, dBL_traj, dBR_traj, dFR_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)
    gait_name = f"mtri_v1_o{orientation}"
    save_name = f"B1_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


def ambl_v2(path, x, y, cycle_steps=3, num_iters=1):
    # order = [FL, FR, BL, BR]
    # x and y are the directionality components

    if y >= 0:
        if x >= 0:
            order = np.array([2, 1, 3, 4]) - 1
            orientation = 1
        else:
            order = np.array([3, 2, 4, 1]) - 1
            orientation = 2
    else:
        if x < 0:
            order = np.array([4, 3, 1, 2]) - 1
            orientation = 3
        else:
            order = np.array([1, 4, 2, 3]) - 1
            orientation = 4

    deltas = gen_dicts(path, x, y, cycle_steps)

    dFL = deltas[order[0]]
    dFR = deltas[order[1]]
    dBL = deltas[order[2]]
    dBR = deltas[order[3]]

    dFL_traj = np.vstack((dFL['push_A'], dFL['wait_A']))
    dFR_traj = np.vstack((dFR['push_A'], dFR['wait_A']))
    dBL_traj = np.vstack((dBL['push_A'], dBL['wait_A']))
    dBR_traj = np.vstack((dBR['push_A'], dBR['wait_A']))

    # Step order: FR, BL, FL, BR
    dFR_rep = np.vstack((dFR['step_A'], dFR['wait_D'], dFR['wait_D'], dFR['zero_A'], dFR['wait_A'], dFR['wait_A'],
                         dFR['wait_A'], dFR['wait_A'], dFR['wait_A']))
    dBL_rep = np.vstack((dBL['wait_A'], dBL['lift_A'], dBL['step_A'], dBL['wait_D'], dBL['wait_D'], dBL['zero_A'],
                         dBL['wait_A'], dBL['wait_A'], dBL['wait_A']))
    dFL_rep = np.vstack((dFL['wait_A'], dFL['wait_A'], dFL['wait_A'], dFL['lift_A'], dFL['step_A'], dFL['wait_D'],
                         dFL['wait_D'], dFL['zero_A'], dFL['wait_A']))
    dBR_rep = np.vstack((dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['lift_A'],
                         dBR['step_A'], dBR['wait_D'], dBR['wait_D']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dFR_traj = np.vstack((dFR_traj, dFR['lift_A']))
        else:
            dFR_traj = np.vstack((dFR_traj, dFR['wait_A']))

        dBL_traj = np.vstack((dBL_traj, dBL['wait_A']))
        dFL_traj = np.vstack((dFL_traj, dFL['wait_A']))

        if i == 0:
            dBR_traj = np.vstack((dBR_traj, dBR['wait_A']))
        else:
            dBR_traj = np.vstack((dBR_traj, dBR['zero_A']))

        if i < num_iters:
            dFR_traj = np.vstack((dFR_traj, dFR_rep))
            dBL_traj = np.vstack((dBL_traj, dBL_rep))
            dFL_traj = np.vstack((dFL_traj, dFL_rep))
            dBR_traj = np.vstack((dBR_traj, dBR_rep))

    # order = [FL, FR, BL, BR]
    if orientation == 1:
        all_pos = path.combine_traj_pos(dFR_traj, dFL_traj, dBL_traj, dBR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dBR_traj, dFR_traj, dFL_traj, dBL_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dBL_traj, dBR_traj, dFR_traj, dFL_traj)
    else:
        all_pos = path.combine_traj_pos(dFL_traj, dBL_traj, dBR_traj, dFR_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)
    gait_name = f"mtri_v2_o{orientation}"
    save_name = f"B2_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


def mtri_v0(path, orientation, step=0.01, cycle_steps=3, num_iters=1):
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

    deltas = gen_dicts(path, x, y, cycle_steps)

    dF = deltas[order[0]]
    dB = deltas[order[1]]
    dL = deltas[order[2]]
    dR = deltas[order[3]]

    dF_traj = np.vstack((dF['push_A'], dF['wait_A']))
    dB_traj = np.vstack((dB['push_A'], dB['wait_A']))
    dL_traj = np.vstack((dL['push_A'], dL['wait_A']))
    dR_traj = np.vstack((dR['push_A'], dR['wait_A']))

    dF_rep = np.vstack((dF['step_A'], dF['zero_A'], dF['wait_A'], dF['wait_A'], dF['wait_A'], dF['wait_A'], dF['wait_A']))
    dB_rep = np.vstack((dB['wait_A'], dB['lean_C'], dB['wait_C'], dB['wait_C'], dB['wait_C'], dB['lift_C'], dB['step_A']))
    dL_rep = np.vstack((dL['wait_A'], dL['lift_A'], dL['step_A'], dL['zero_A'], dL['wait_A'], dL['wait_A'], dL['wait_A']))
    dR_rep = np.vstack((dR['wait_A'], dR['wait_A'], dR['wait_A'], dR['lift_A'], dR['step_A'], dR['zero_A'], dR['wait_A']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dF_traj = np.vstack((dF_traj, dF['lift_A']))
        else:
            dF_traj = np.vstack((dF_traj, dF['wait_A']))

        dL_traj = np.vstack((dL_traj, dL['wait_A']))
        dR_traj = np.vstack((dR_traj, dR['wait_A']))

        if i == 0:
            dB_traj = np.vstack((dB_traj, dB['wait_A']))
        else:
            dB_traj = np.vstack((dB_traj, dB['zero_A']))

        if i < num_iters:
            dF_traj = np.vstack((dF_traj, dF_rep))
            dB_traj = np.vstack((dB_traj, dB_rep))
            dL_traj = np.vstack((dL_traj, dL_rep))
            dR_traj = np.vstack((dR_traj, dR_rep))

    if orientation == 1:
        all_pos = path.combine_traj_pos(dF_traj, dL_traj, dB_traj, dR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dR_traj, dF_traj, dL_traj, dB_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dB_traj, dR_traj, dF_traj, dL_traj)
    else:
        all_pos = path.combine_traj_pos(dL_traj, dB_traj, dR_traj, dF_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)

    gait_name = f"mtri_v0_o{orientation}"
    save_name = f"C0_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


def mtri_v1(path, orientation, step=0.01, cycle_steps=3, num_iters=1):
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

    deltas = gen_dicts(path, x, y, cycle_steps)

    dF = deltas[order[0]]
    dB = deltas[order[1]]
    dL = deltas[order[2]]
    dR = deltas[order[3]]

    dF_traj = np.vstack((dF['push_A'], dF['wait_A']))
    dB_traj = np.vstack((dB['push_A'], dB['wait_A']))
    dL_traj = np.vstack((dL['push_A'], dL['wait_A']))
    dR_traj = np.vstack((dR['push_A'], dR['wait_A']))

    dF_rep = np.vstack((dF['step_A'], dF['zero_A'], dF['wait_A'], dF['wait_A'], dF['wait_A'], dF['wait_A'], dF['wait_A']))
    dB_rep = np.vstack((dB['wait_A'], dB['lean_B'], dB['wait_B'], dB['wait_B'], dB['wait_B'], dB['lift_B'], dB['step_A']))
    dL_rep = np.vstack((dL['wait_A'], dL['lift_A'], dL['step_A'], dL['zero_A'], dL['wait_A'], dL['wait_A'], dL['wait_A']))
    dR_rep = np.vstack((dR['wait_A'], dR['wait_A'], dR['wait_A'], dR['lift_A'], dR['step_A'], dR['zero_A'], dR['wait_A']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dF_traj = np.vstack((dF_traj, dF['lift_A']))
        else:
            dF_traj = np.vstack((dF_traj, dF['wait_A']))

        dL_traj = np.vstack((dL_traj, dL['wait_A']))
        dR_traj = np.vstack((dR_traj, dR['wait_A']))

        if i == 0:
            dB_traj = np.vstack((dB_traj, dB['wait_A']))
        else:
            dB_traj = np.vstack((dB_traj, dB['zero_A']))

        if i < num_iters:
            dF_traj = np.vstack((dF_traj, dF_rep))
            dB_traj = np.vstack((dB_traj, dB_rep))
            dL_traj = np.vstack((dL_traj, dL_rep))
            dR_traj = np.vstack((dR_traj, dR_rep))

    if orientation == 1:
        all_pos = path.combine_traj_pos(dF_traj, dL_traj, dB_traj, dR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dR_traj, dF_traj, dL_traj, dB_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dB_traj, dR_traj, dF_traj, dL_traj)
    else:
        all_pos = path.combine_traj_pos(dL_traj, dB_traj, dR_traj, dF_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)

    gait_name = f"mtri_v1_o{orientation}"
    save_name = f"C1_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


def mtri_v2(path, orientation, step=0.01, cycle_steps=3, num_iters=1):
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

    deltas = gen_dicts(path, x, y, cycle_steps)

    dF = deltas[order[0]]
    dB = deltas[order[1]]
    dL = deltas[order[2]]
    dR = deltas[order[3]]

    dF_traj = np.vstack((dF['push_A'], dF['wait_A']))
    dB_traj = np.vstack((dB['push_A'], dB['wait_A']))
    dL_traj = np.vstack((dL['push_A'], dL['wait_A']))
    dR_traj = np.vstack((dR['push_A'], dR['wait_A']))

    dF_rep = np.vstack((dF['step_A'], dF['zero_A'], dF['wait_A'], dF['wait_A'], dF['wait_A'], dF['wait_A'],
                        dF['wait_A']))
    dB_rep = np.vstack((dB['wait_A'], dB['lean_C'], dB['wait_C'], dB['wait_C'], dB['wait_C'], dB['lift_C'],
                        dB['step_A']))
    dL_rep = np.vstack((dL['wait_A'], dL['lean_B'], dL['lift_B'], dL['step_A'], dL['zero_A'], dL['wait_A'],
                        dL['wait_A']))
    dR_rep = np.vstack((dR['wait_A'], dR['lean_B'], dR['wait_B'], dR['lift_B'], dR['step_A'], dR['zero_A'],
                        dR['wait_A']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dF_traj = np.vstack((dF_traj, dF['lift_A']))
        else:
            dF_traj = np.vstack((dF_traj, dF['wait_A']))

        dL_traj = np.vstack((dL_traj, dL['wait_A']))
        dR_traj = np.vstack((dR_traj, dR['wait_A']))

        if i == 0:
            dB_traj = np.vstack((dB_traj, dB['wait_A']))
        else:
            dB_traj = np.vstack((dB_traj, dB['zero_A']))

        if i < num_iters:
            dF_traj = np.vstack((dF_traj, dF_rep))
            dB_traj = np.vstack((dB_traj, dB_rep))
            dL_traj = np.vstack((dL_traj, dL_rep))
            dR_traj = np.vstack((dR_traj, dR_rep))

    if orientation == 1:
        all_pos = path.combine_traj_pos(dF_traj, dL_traj, dB_traj, dR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dR_traj, dF_traj, dL_traj, dB_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dB_traj, dR_traj, dF_traj, dL_traj)
    else:
        all_pos = path.combine_traj_pos(dL_traj, dB_traj, dR_traj, dF_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)

    gait_name = f"mtri_v2_o{orientation}"
    save_name = f"C2_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


def walk_v0(path, x, y, cycle_steps=3, num_iters=1):
    # order = [FL, FR, BL, BR]
    # x and y are the directionality components

    if y >= 0:
        if x >= 0:
            order = np.array([2, 1, 3, 4]) - 1
            orientation = 1
        else:
            order = np.array([3, 2, 4, 1]) - 1
            orientation = 2
    else:
        if x < 0:
            order = np.array([4, 3, 1, 2]) - 1
            orientation = 3
        else:
            order = np.array([1, 4, 2, 3]) - 1
            orientation = 4

    deltas = gen_dicts(path, x, y, cycle_steps)

    dFL = deltas[order[0]]
    dFR = deltas[order[1]]
    dBL = deltas[order[2]]
    dBR = deltas[order[3]]

    dFL_traj = np.vstack((dFL['push_A'], dFL['wait_A']))
    dFR_traj = np.vstack((dFR['push_A'], dFR['wait_A']))
    dBL_traj = np.vstack((dBL['push_A'], dBL['wait_A']))
    dBR_traj = np.vstack((dBR['push_A'], dBR['wait_A']))

    # Step order: BL, FL, BR, FR
    dBL_rep = np.vstack(
        (dBL['step_A'], dBL['zero_A'], dBL['wait_A'], dBL['wait_A'], dBL['wait_A'], dBL['wait_A'], dBL['wait_A']))
    dFL_rep = np.vstack(
        (dFL['wait_A'], dFL['lift_A'], dFL['step_A'], dFL['zero_A'], dFL['wait_A'], dFL['wait_A'], dFL['wait_A']))
    dBR_rep = np.vstack(
        (dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['lift_A'], dBR['step_A'], dBR['zero_A'], dBR['wait_A']))
    dFR_rep = np.vstack(
        (dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['lift_A'], dFR['step_A']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dBL_traj = np.vstack((dBL_traj, dBL['lift_A']))
        else:
            dBL_traj = np.vstack((dBL_traj, dBL['wait_A']))

        dFL_traj = np.vstack((dFL_traj, dFL['wait_A']))
        dBR_traj = np.vstack((dBR_traj, dBR['wait_A']))

        if i == 0:
            dFR_traj = np.vstack((dFR_traj, dFR['wait_A']))
        else:
            dFR_traj = np.vstack((dFR_traj, dFR['zero_A']))

        if i < num_iters:
            dBL_traj = np.vstack((dBL_traj, dBL_rep))
            dFL_traj = np.vstack((dFL_traj, dFL_rep))
            dBR_traj = np.vstack((dBR_traj, dBR_rep))
            dFR_traj = np.vstack((dFR_traj, dFR_rep))

    # order = [FL, FR, BL, BR]
    if orientation == 1:
        all_pos = path.combine_traj_pos(dFR_traj, dFL_traj, dBL_traj, dBR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dBR_traj, dFR_traj, dFL_traj, dBL_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dBL_traj, dBR_traj, dFR_traj, dFL_traj)
    else:
        all_pos = path.combine_traj_pos(dFL_traj, dBL_traj, dBR_traj, dFR_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)
    gait_name = f"walk_v0_o{orientation}"
    save_name = f"A0_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


def walk_v1(path, x, y, cycle_steps=3, num_iters=1):
    # order = [FL, FR, BL, BR]
    # x and y are the directionality components

    if y >= 0:
        if x >= 0:
            order = np.array([2, 1, 3, 4]) - 1
            orientation = 1
        else:
            order = np.array([3, 2, 4, 1]) - 1
            orientation = 2
    else:
        if x < 0:
            order = np.array([4, 3, 1, 2]) - 1
            orientation = 3
        else:
            order = np.array([1, 4, 2, 3]) - 1
            orientation = 4

    deltas = gen_dicts(path, x, y, cycle_steps)

    dFL = deltas[order[0]]
    dFR = deltas[order[1]]
    dBL = deltas[order[2]]
    dBR = deltas[order[3]]

    dFL_traj = np.vstack((dFL['push_A'], dFL['wait_A']))
    dFR_traj = np.vstack((dFR['push_A'], dFR['wait_A']))
    dBL_traj = np.vstack((dBL['push_A'], dBL['wait_A']))
    dBR_traj = np.vstack((dBR['push_A'], dBR['wait_A']))

    # Step order: BL, FL, BR, FR
    dBL_rep = np.vstack((dBL['step_A'], dBL['wait_D'], dBL['zero_A'], dBL['wait_A'], dBL['wait_A'], dBL['wait_A'],
                         dBL['wait_A'], dBL['wait_A']))
    dFL_rep = np.vstack((dFL['wait_A'], dFL['lift_A'], dFL['step_A'], dFL['wait_D'], dFL['zero_A'], dFL['wait_A'],
                         dFL['wait_A'], dFL['wait_A']))
    dBR_rep = np.vstack((dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['lift_A'], dBR['step_A'],
                         dBR['wait_D'], dBR['zero_A'], dBR['wait_A']))
    dFR_rep = np.vstack((dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['lift_A'],
                         dFR['step_A'], dFR['wait_D']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dBL_traj = np.vstack((dBL_traj, dBL['lift_A']))
        else:
            dBL_traj = np.vstack((dBL_traj, dBL['wait_A']))

        dFL_traj = np.vstack((dFL_traj, dFL['wait_A']))
        dBR_traj = np.vstack((dBR_traj, dBR['wait_A']))

        if i == 0:
            dFR_traj = np.vstack((dFR_traj, dFR['wait_A']))
        else:
            dFR_traj = np.vstack((dFR_traj, dFR['zero_A']))

        if i < num_iters:
            dBL_traj = np.vstack((dBL_traj, dBL_rep))
            dFL_traj = np.vstack((dFL_traj, dFL_rep))
            dBR_traj = np.vstack((dBR_traj, dBR_rep))
            dFR_traj = np.vstack((dFR_traj, dFR_rep))

    # order = [FL, FR, BL, BR]
    if orientation == 1:
        all_pos = path.combine_traj_pos(dFR_traj, dFL_traj, dBL_traj, dBR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dBR_traj, dFR_traj, dFL_traj, dBL_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dBL_traj, dBR_traj, dFR_traj, dFL_traj)
    else:
        all_pos = path.combine_traj_pos(dFL_traj, dBL_traj, dBR_traj, dFR_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)
    gait_name = f"walk_v1_o{orientation}"
    save_name = f"A1_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


def walk_v2(path, x, y, cycle_steps=3, num_iters=1):
    # order = [FL, FR, BL, BR]
    # x and y are the directionality components

    if y >= 0:
        if x >= 0:
            order = np.array([2, 1, 3, 4]) - 1
            orientation = 1
        else:
            order = np.array([3, 2, 4, 1]) - 1
            orientation = 2
    else:
        if x < 0:
            order = np.array([4, 3, 1, 2]) - 1
            orientation = 3
        else:
            order = np.array([1, 4, 2, 3]) - 1
            orientation = 4

    deltas = gen_dicts(path, x, y, cycle_steps)

    dFL = deltas[order[0]]
    dFR = deltas[order[1]]
    dBL = deltas[order[2]]
    dBR = deltas[order[3]]

    dFL_traj = np.vstack((dFL['push_A'], dFL['wait_A']))
    dFR_traj = np.vstack((dFR['push_A'], dFR['wait_A']))
    dBL_traj = np.vstack((dBL['push_A'], dBL['wait_A']))
    dBR_traj = np.vstack((dBR['push_A'], dBR['wait_A']))

    # Step order: BL, FL, BR, FR
    dBL_rep = np.vstack((dBL['step_A'], dBL['wait_D'], dBL['wait_D'], dBL['zero_A'], dBL['wait_A'], dBL['wait_A'],
                         dBL['wait_A'], dBL['wait_A'], dBL['wait_A']))
    dFL_rep = np.vstack((dFL['wait_A'], dFL['lift_A'], dFL['step_A'], dFL['wait_D'], dFL['wait_D'], dFL['zero_A'],
                         dFL['wait_A'], dFL['wait_A'], dFL['wait_A']))
    dBR_rep = np.vstack((dBR['wait_A'], dBR['wait_A'], dBR['wait_A'], dBR['lift_A'], dBR['step_A'],
                         dBR['wait_D'], dBR['wait_D'], dBR['zero_A'], dBR['wait_A']))
    dFR_rep = np.vstack((dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['wait_A'], dFR['lift_A'],
                         dFR['step_A'], dFR['wait_D'], dFR['wait_D']))

    for i in np.arange(num_iters + 1):
        if i < num_iters:
            dBL_traj = np.vstack((dBL_traj, dBL['lift_A']))
        else:
            dBL_traj = np.vstack((dBL_traj, dBL['wait_A']))

        dFL_traj = np.vstack((dFL_traj, dFL['wait_A']))
        dBR_traj = np.vstack((dBR_traj, dBR['wait_A']))

        if i == 0:
            dFR_traj = np.vstack((dFR_traj, dFR['wait_A']))
        else:
            dFR_traj = np.vstack((dFR_traj, dFR['zero_A']))

        if i < num_iters:
            dBL_traj = np.vstack((dBL_traj, dBL_rep))
            dFL_traj = np.vstack((dFL_traj, dFL_rep))
            dBR_traj = np.vstack((dBR_traj, dBR_rep))
            dFR_traj = np.vstack((dFR_traj, dFR_rep))

    # order = [FL, FR, BL, BR]
    if orientation == 1:
        all_pos = path.combine_traj_pos(dFR_traj, dFL_traj, dBL_traj, dBR_traj)
    elif orientation == 2:
        all_pos = path.combine_traj_pos(dBR_traj, dFR_traj, dFL_traj, dBL_traj)
    elif orientation == 3:
        all_pos = path.combine_traj_pos(dBL_traj, dBR_traj, dFR_traj, dFL_traj)
    else:
        all_pos = path.combine_traj_pos(dFL_traj, dBL_traj, dBR_traj, dFR_traj)

    all_acts, all_pos_fk, all_com, steps = path.gen_all_steps(all_pos)

    gait_name = f"walk_v2_o{orientation}"
    save_name = f"A2_o{orientation}"

    return all_pos, all_acts, all_pos_fk, all_com, steps, gait_name, save_name


