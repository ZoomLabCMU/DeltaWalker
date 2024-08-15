import math
import numpy as np

PI = math.pi


class TrajOCRL:
    def __init__(self, walker, filepath, com_offset=0):
        self.walker = walker
        all_data = np.loadtxt(filepath, skiprows=1, delimiter=',')
        self.steps = all_data.shape[1]

        all_data[2, :] -= com_offset

        # Get offset for z to shift the coordinate system down (make delta origin plane 0 instead of ground) to be in world
        z_off = all_data[2, :]
        xyz_off = np.vstack((np.zeros(self.steps), np.zeros(self.steps), z_off))

        # print("offset z")
        # print(z_off)

        # Values are in world coordinates
        # Position values (XYZ)
        self.com_pos = all_data[0:3, :] - xyz_off
        self.ee1_pos = all_data[3:6, :] - xyz_off
        self.ee2_pos = all_data[6:9, :] - xyz_off
        self.ee3_pos = all_data[9:12, :] - xyz_off
        self.ee4_pos = all_data[12:15, :] - xyz_off

        # Velocity Values (XYZ)
        self.com_vel = all_data[15:18, :]
        self.ee1_vel = all_data[18:21, :]
        self.ee2_vel = all_data[21:24, :]
        self.ee3_vel = all_data[24:27, :]
        self.ee4_vel = all_data[27:30, :]

        self.all_ee_unshifted = all_data[3:15, :]
        self.all_ee_shifted = np.vstack((self.ee1_pos, self.ee2_pos, self.ee3_pos, self.ee4_pos))
        self.com_unshifted = all_data[0:3, :]

    def init_helper(self, pos, pt, steps):
        x0 = 0
        y0 = 0
        z0 = self.walker.offset + 0.02 + 0.005
        if pt == 0:     # Delta 1
            x0 = 0.04
        elif pt == 1:   # Delta 2
            y0 = 0.04
        elif pt == 2:   # Delta 3
            x0 = -0.04
        elif pt == 3:   # Delta 4
            y0 = -0.04
        elif pt == 4:   # COM
            z0 = 0

        xs = np.linspace(x0, pos[0], num=steps)
        ys = np.linspace(y0, pos[1], num=steps)
        zs = np.linspace(z0, pos[2], num=steps)
        return np.vstack((xs, ys, zs))

    def init_acts(self):
        steps = 5
        repeat = 2
        e1 = self.init_helper(self.ee1_pos[:, 0], 0, steps)
        e2 = self.init_helper(self.ee2_pos[:, 0], 1, steps)
        e3 = self.init_helper(self.ee3_pos[:, 0], 2, steps)
        e4 = self.init_helper(self.ee4_pos[:, 0], 3, steps)
        cs = self.init_helper(self.com_pos[:, 0], 4, steps)
        all_acts = []
        for i in np.arange(steps):
            COM_position = cs[:, i]
            ee_positions = [e1[:, i], e2[:, i], e3[:, i], e4[:, i]]
            acts = self.walker.go_to_IK(ee_positions, COM_position)
            all_acts.append(np.round(acts, 4))
        last_act = all_acts[-1]
        for i in np.arange(repeat):
            all_acts.append(last_act)

        return all_acts, steps+repeat

    def gen_acts_all(self):
        all_acts = []
        for i in np.arange(self.steps):
            # print(f"Step {i}")
            COM_position = self.com_pos[:, i]
            ee_positions = [self.ee1_pos[:, i], self.ee2_pos[:, i], self.ee3_pos[:, i], self.ee4_pos[:, i]]
            acts = self.walker.go_to_IK(ee_positions, COM_position)
            all_acts.append(np.round(acts, 4))
        return all_acts

    def acts_to_motors(self, all_acts):
        steps = len(all_acts)
        num_acts = self.walker.num_feet * 3
        motor_ranges = np.zeros((num_acts, steps))
        for i in np.arange(steps):
            motor_ranges[:, i] = all_acts[i].flatten()
        # print(motor_ranges)
        return motor_ranges

    def fk_estimates(self, motor_ranges, steps):
        ee_pos = np.zeros((self.walker.num_feet*3, steps))
        com_pos = np.zeros((3, steps))
        for i in np.arange(steps):
            distances = motor_ranges[:, i].reshape((-1, 3))
            pos = self.walker.go_to_FK(distances)
            ee_pos[:, i] = np.array(pos).flatten()
            com_pos[:, i] = self.walker.world_COM
        return ee_pos, com_pos

    def plot_world_convert(self, ee_pos, com_pos):
        steps = ee_pos.shape[1]
        for i in np.arange(steps):
            es = ee_pos[:, i]
            min_z = np.min((es[2], es[5], es[8], es[11]))
            offset = np.array([0, 0, min_z])
            offset_all = np.hstack((offset, offset, offset, offset))
            ee_pos[:, i] -= offset_all
            com_pos[:, i] -= offset
        return ee_pos, com_pos


