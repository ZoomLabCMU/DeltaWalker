import numpy as np
import os
import time
from scipy.spatial.transform import Rotation as R
import pickle
import matplotlib.pyplot as plt
import socket

from Prismatic_Delta import Prismatic_Delta
from DeltaRobotAgent import DeltaArrayAgent

BUFFER_SIZE = 1

class DeltaRobotEnv():
    def __init__(self):
        self.rot_30 = np.pi/6
        self.low_z = 9
        self.high_z = 5.5

        """ Delta Robots Vars """
        self.NUM_MOTORS = 12
        self.to_be_moved = []
        u_p = 0.6 # cm
        u_b = 2.0 # 1.2 + 0.25 # cm
        l = 4.4 # cm
        self.Delta = Prismatic_Delta(u_p, u_b, l)
        ip_addr = "172.26.177.218" # "192.168.1.17"
        self.delta_comm = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.delta_comm.connect((ip_addr, 80))
        self.delta_comm.settimeout(0.1)
        self.delta_agent = DeltaArrayAgent(self.delta_comm, 0)
        self.delta_robots = self.delta_agent.robot_pos_ctrl_dict.keys()

    def move_tridelta(self, active_ids, trajs):
        assert(len(active_ids) == len(trajs))
        for i, idx in enumerate(active_ids):
            if idx in self.delta_robots:
                self.delta_agent.save_joint_positions(idx, trajs[i])
        self.delta_agent.move_useful()
        self.wait_until_done()
        print("Done Moving")

    def actuate_tridelta(self, trajs):
        self.delta_agent.move_actuators(trajs)
        self.wait_until_done()
        print("Done Actuating")

    def wait_until_done(self):
        done_moving = False
        while not done_moving:
            try:
                received = self.delta_agent.esp01.recv(BUFFER_SIZE)
                ret = received.decode().strip()
                if ret == "A":
                    done_moving = True
                    # time.sleep(0.001)
            except Exception as e:
                # print(e)
                pass
        # time.sleep(0.001)
        return

    def reset(self):
        self.delta_agent.reset()
        self.wait_until_done()
        print("Done Resetting")

    def shut_down(self):
        self.delta_comm.shutdown(socket.SHUT_RDWR)
        self.delta_comm.close()

if __name__ == "__main__":
    env = DeltaRobotEnv()
    env.reset()
    base_motion = [[0.015] * 9]
    env.actuate_tridelta(base_motion)
    time.sleep(2.0)
    env.reset()
