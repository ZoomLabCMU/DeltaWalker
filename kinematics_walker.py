import time
import math
import numpy as np
from kinematics_delta import PDelta
from helpers import *

PI = math.pi


# TODO make sure i have it documented somewhere
#  Here the world frame is centered with the origin at the the centroid of the base triangle
#  Z is negative downwards atm - make z positive downwards ?
#  When passing in values for movement in trajectories/GenPaths the values are displacements relative
#  to the specific delta in x, y, and z

class DeltaWalkerKin():
    def __init__(self, nonfloat=True, num_feet=4, center_len=0.04, ee_len=0.006, base_len=0.02, leg_len=0.05):
        self.nonfloat = nonfloat  # bool for floating vs not for deciding if to update centers
        # true = nonfloating environment, false = floating environment

        # Z is positive upwards and the walker origin is at the center of the base positions

        self.num_feet = num_feet
        self.center_len = center_len
        self.ee_len = ee_len
        self.base_len = base_len
        self.leg_len = leg_len

        # Walk Mode
        self.init_center = [0, self.center_len]
        self.angles = np.linspace(-90, 270, self.num_feet, endpoint=False)

        # World Frame Values
        self.world_COM = np.zeros(3)        # Walker COM xyz initialized at the world frame origin
        self.world_feet_pos = []            # Foot center positions
        self.world_base_pos = []            # Base center positions

        # Robot Frame Values
        self.robot_feet_pos = []
        self.robot_base_pos = []

        # Walker feet objects
        self.walker_feet = []

        # Previous Positions
        self.world_COM_prev = np.zeros(3)
        self.world_feet_pos_prev = []
        self.world_base_pos_prev = []
        self.robot_feet_pos_prev = []

        # Initialize the feet
        init_centers = []
        for i in range(self.num_feet):
            center = rotate([0, 0], self.init_center, self.angles[i])
            foot = PDelta(ee_len=self.ee_len, base_len=self.base_len, leg_len=self.leg_len)
            self.walker_feet.append(foot)

            foot_pos = [center[0], center[1], foot.p0[2]]
            base_pos = [center[0], center[1], foot.b0[2]]

            # Initialize current positions
            self.world_feet_pos.append(foot_pos)
            self.world_base_pos.append(base_pos)
            self.robot_feet_pos.append(foot_pos)
            self.robot_base_pos.append(base_pos)

            # Initialize previous positions
            self.world_feet_pos_prev.append(foot_pos)
            self.world_base_pos_prev.append(base_pos)
            self.robot_feet_pos_prev.append(foot_pos)

            init_centers.append([foot.p0[0], foot.p0[1], foot.p0[2]])

        # Feet are in the negative z axis and offset is used to account for z axis transformation
        feet_init_center = np.mean(np.array(self.world_feet_pos), axis=0)
        self.offset = np.round(feet_init_center[2] - 0.02, 4)
        print("OFFSET", self.offset)

        # TODO check signs and offsets - should z be positive or negative downwards

    # Used purely for plotting purposes
    def go_to_FK(self, distances):
        # distances are actuation distances, and it has to be in shape (num_feet x 3)
        new_feet_world = []
        new_feet_robot = []
        new_base_world = []
        for i in range(self.num_feet):
            x, y, z = self.walker_feet[i].go_to_distance(distances[i][0], distances[i][1], distances[i][2])
            # rotate to the world coordinate
            # TODO Check this update
            x_t = x + self.world_COM[0]
            y_t = y + self.world_COM[1]
            x_r, y_r = rotate([0,0], [x_t, y_t], self.angles[i])

            new_feet_robot.append([x_r, y_r, z])
            new_feet_world.append([x_r + self.world_COM[0], y_r + self.world_COM[1], z])
            disp = self.walker_feet[i].b0 - self.walker_feet[i].p0
            new_base_world.append([x_r + self.world_COM[0] - disp[0], y_r + self.world_COM[1] - disp[1], 0])

        if self.nonfloat:
            self.update_positions(new_feet_world, new_feet_robot, new_base_world)
        return new_feet_world

    def check_oob(self, a, act, foot):
        if a < 0:
            print(f"Delta {foot} Act {act} OOB below 0.00 {a}")
        elif a > 0.02:
            print(f"Delta {foot} Act {act} OOB above 0.02 {a}")

    def go_to_IK(self, ee_positions, COM_position=None):
        # Take desired ee position in the world frame
        # Take desired COM position in the world frame optionally
        actuations = []

        if COM_position is None:
            COM_offset = np.array([self.world_COM[0], self.world_COM[1], 0])
        else:
            COM_offset = np.array([COM_position[0], COM_position[1], 0])

        new_feet_world = []
        new_base_world = []
        new_feet_robot = []

        for i in range(self.num_feet):
            # Get desired world position
            ee_wor_pos = ee_positions[i]
            # Convert desired position into robot frame
            ee_rob_pos = ee_wor_pos - COM_offset
            # Convert desired position into delta i frame
            ee_del_pos = rotate([0, 0], [ee_rob_pos[0], ee_rob_pos[1]], -1 * self.angles[i])
            pos_x = ee_del_pos[0] - self.init_center[0]
            pos_y = ee_del_pos[1] - self.init_center[1]

            # Calculate actuation amounts
            a1, a2, a3 = self.walker_feet[i].go_to_position(pos_x, pos_y, ee_rob_pos[2])
            # TODO determine if should clip limits here or in the kinematics_delta
            self.check_oob(a1, 1, i+1)
            self.check_oob(a2, 2, i+1)
            self.check_oob(a3, 3, i+1)
            # print(f"act {i+1}: {a1}, {a2}, {a3}")
            a1 = np.clip(a1, 0, 0.02)
            a2 = np.clip(a2, 0, 0.02)
            a3 = np.clip(a3, 0, 0.02)
            actuations.append([a1, a2, a3])

            # Update positions
            # TODO determine if this form of updating the base positions is valid
            new_feet_world.append(ee_wor_pos)
            new_feet_robot.append(ee_rob_pos)
            disp = self.walker_feet[i].b0 - self.walker_feet[i].p0     # displacement from foot to base
            new_base_world.append(ee_wor_pos - disp)

        # Update positions
        if self.nonfloat:
            self.update_positions(new_feet_world, new_feet_robot, new_base_world, COM_position)
        return actuations

    def update_positions(self, new_feet_world, new_feet_robot, new_base_world, COM_position=None):
        self.world_COM_prev = self.world_COM
        self.world_feet_pos_prev = self.world_feet_pos
        self.world_base_pos_prev = self.world_base_pos
        self.robot_feet_pos_prev = self.robot_feet_pos

        self.world_feet_pos = new_feet_world
        self.world_base_pos = new_base_world
        self.robot_feet_pos = new_feet_robot

        if COM_position is None:
            self.world_COM = np.mean(np.array(self.world_base_pos), axis=0)
        else:
            self.world_COM = COM_position
        return

    # def update_centers(self):
    #     curr_centers = []
    #     for i in range(self.num_feet):
    #         curr_centers.append([self.walker_feet[i].p0[0], self.walker_feet[i].p0[1], self.walker_feet[i].p0[2]])
    #     curr_centers = np.mean(np.array(curr_centers), axis=0)
    #     self.walker_center = curr_centers[:2]
    #     return

    # TODO For Jennifer
    #  maybe also update rotations (is the delta still along the xy axes or is it at an angle?)
    # def update_centers_WIP(self, new_feet_pos, new_base_pos):
    #     self.feet_pos = new_feet_pos
    #     self.base_pos = new_base_pos
    #
    #     self.feet_COM = np.mean(np.array(self.feet_pos), axis=0)
    #     self.feet_center = self.feet_COM[:2]
    #
    #     self.walker_COM = np.mean(np.array(self.base_pos), axis=0)
    #     self.walker_center = self.walker_COM[:2]
    #
    #     print(f"CENTER {self.walker_center}")
    #     return

    # def ee_to_world(self):
    #     pos = self.feet_pos

    # def world_to_ee_all(self, displacement):
    #     pos = self.feet_pos[:2] + self.walker_center + displacement.flatten()
    #     print("feet")
    #     print(pos)
    #     return pos
    #
    # def world_to_ee_one(self, displacement, delta):
    #     pos = self.feet_pos[delta][:2] + self.walker_center + displacement.flatten()
    #     print(f"foot {delta}")
    #     print(pos)
    #     return pos

    def disp_to_world_all(self, displacement):
        pos = self.world_feet_pos[:2] + displacement.flatten()
        return pos

    def disp_to_world_one(self, displacement, delta):
        pos = self.world_feet_pos[delta][:2] + displacement.flatten()
        return pos


