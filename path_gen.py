import math
import numpy as np

PI = math.pi


class GenPaths:
    def __init__(self, walker):
        self.walker = walker

    def gen_all_traj_pos(self, start, end, steps):
        x0, y0, z0 = start
        x1, y1, z1 = end

        disp_x = np.linspace(x0, x1, num=steps).reshape((-1, 1))
        disp_y = np.linspace(y0, y1, num=steps).reshape((-1, 1))
        zs = np.linspace(z0, z1, num=steps).reshape((-1, 1))
        disp_xy = np.hstack((disp_x, disp_y))

        all_pos = np.zeros((steps, 3*self.walker.num_feet))
        for delta in np.arange(self.walker.num_feet):
            positions = np.zeros((steps, 3))
            for i in np.arange(steps):
                xy = self.walker.disp_to_world_one(disp_xy[i, :], delta)
                z = zs[i] + self.walker.offset
                positions[i, :] = np.round(np.hstack((xy, z)), 4)
            all_pos[:, delta*3:(delta + 1)*3] = positions
        # print(all_pos)
        return all_pos

    def gen_one_traj_pos(self, start, end, steps, delta):
        x0, y0, z0 = start
        x1, y1, z1 = end

        disp_x = np.linspace(x0, x1, num=steps).reshape((-1, 1))
        disp_y = np.linspace(y0, y1, num=steps).reshape((-1, 1))
        zs = np.linspace(z0, z1, num=steps).reshape((-1, 1))
        disp_xy = np.hstack((disp_x, disp_y))

        positions = np.zeros((steps, 3))

        for i in np.arange(steps):
            xy = self.walker.disp_to_world_one(disp_xy[i, :], delta)
            z = zs[i] + self.walker.offset
            positions[i, :] = np.round(np.hstack((xy, z)), 4)

        return positions

    def gen_one_step(self, pos):
        acts = np.round(self.walker.go_to_IK(pos), 4)
        pos_fk = np.round(self.walker.go_to_FK(acts), 4)
        pos_com = np.round(self.walker.world_COM, 4)
        # pos_fk = np.array(pos)
        return acts, pos_fk, pos_com

    def combine_traj_pos(self, d1, d2, d3, d4):
        return np.hstack((d1, d2, d3, d4))

    def gen_all_steps(self, positions):
        steps = np.shape(positions)[0]
        all_acts = []
        all_pos_fk = np.zeros((steps, 3*self.walker.num_feet))
        all_com = np.zeros((steps, 3))
        for i in np.arange(steps):
            # print(f"STEP {i}")
            pos = positions[i, :].reshape((-1, 3))
            acts, pos_fk, pos_com = self.gen_one_step(pos)
            all_acts.append(acts)
            all_pos_fk[i, :] = pos_fk.flatten()
            all_com[i, :] = pos_com
        return all_acts, all_pos_fk, all_com, steps

    def pos_to_deltas(self, positions):
        deltas = []
        for i in np.arange(self.walker.num_feet):
            deltas.append(positions[:, i*3:(i+1)*3])
        return deltas

    def acts_to_motors(self, all_acts, steps):
        num_acts = self.walker.num_feet * 3
        motor_ranges = np.zeros((num_acts, steps))
        for i in np.arange(steps):
            motor_ranges[:, i] = all_acts[i].flatten()
        return motor_ranges

