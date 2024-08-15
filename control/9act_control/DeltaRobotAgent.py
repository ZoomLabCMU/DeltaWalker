import delta_trajectory_pb2 as delta_trajectory_pb2
import numpy as np
# from serial import Serial
from Prismatic_Delta import Prismatic_Delta

NUM_MOTORS = 9
BUFFER_SIZE = 1
u_p = 0.6 # cm
u_b = 2.0 #1.2 + 0.25 # cm
l = 4.3 # cm

Delta = Prismatic_Delta(u_p, u_b, l)

class DeltaArrayAgent:
    def __init__(self, ser, robot_id):
        self.esp01 = ser
        self.delta_message = delta_trajectory_pb2.DeltaMessage()
        self.delta_message.id = robot_id
        self.delta_message.request_joint_pose = False
        self.delta_message.request_done_state = False
        self.delta_message.reset = False
        self.min_joint_pos = 0.001 # 1.0 mm
        self.max_joint_pos = 0.019 # 19.0 mm

        self.done_moving = False
        self.current_joint_positions = [0.05]*NUM_MOTORS
        self.robot_pos_ctrl_dict = {1: [], 2: [], 3: []}

    # Generate reset point for useless robots
    def reset(self):
        for j in range(20):
            pts = np.array([self.min_joint_pos, self.min_joint_pos, self.min_joint_pos])
            _ = [self.delta_message.trajectory.append(pts[i%3]) for i in range(12)]
        self.send_proto_cmd()
        del self.delta_message.trajectory[:]

    def save_joint_positions(self, idx, traj):
        """ If given trajectory is less than 20, pad with last point """
        assert isinstance(traj, list), "Trajectory must be a list"
        assert idx <= 3, "Robot ID must be between 1 and 3"
        traj_len = len(traj)

        if traj_len == 20:
            self.robot_pos_ctrl_dict[idx] = np.array([np.clip(np.array(Delta.IK(pos))*0.01, self.min_joint_pos, self.max_joint_pos) for pos in traj])
        else:
            self.robot_pos_ctrl_dict[idx] = [np.clip(np.array(Delta.IK(pos))*0.01, self.min_joint_pos, self.max_joint_pos) for pos in traj]
            self.robot_pos_ctrl_dict[idx] += [np.clip(np.array(Delta.IK(traj[-1]))*0.01, self.min_joint_pos, self.max_joint_pos) for i in range(20-traj_len)]
            self.robot_pos_ctrl_dict[idx] = np.array(self.robot_pos_ctrl_dict[idx])

    # Move useful robots
    def move_useful(self):
        """ This function is called over entire array of active robots. Do not call this individually in a loop if there is going to be some
        intense compute after this function is called, since that will cause delays in starting to make the deltas move.

        This fn also moves only those individual robots already saved using save_joint_positions(), and makes others go to zero position.
        """
        zeros = np.array([np.array([self.min_joint_pos, self.min_joint_pos, self.min_joint_pos]) for i in range(20)])
        final_jt_pos = []
        for i in self.robot_pos_ctrl_dict:
            if len(self.robot_pos_ctrl_dict[i])!= 20:
                final_jt_pos.append(zeros)
            else:
                final_jt_pos.append(self.robot_pos_ctrl_dict[i])

        final_jt_pos.append(zeros)

        final_jt_pos = np.hstack(final_jt_pos)
        assert final_jt_pos.shape == (20, 12), "Final joint positions must be of shape (20, 12)"
        for j in range(20):
            _ = [self.delta_message.trajectory.append(final_jt_pos[j][i]) for i in range(12)]
        self.send_proto_cmd()
        del self.delta_message.trajectory[:]

    def move_actuators(self, actuator_trajs):
        """
        Directly control the acutation of each motors by passing the values (acutation heights in meter)
        size: N x 9
        Ex: actuator_trajs = [[0.005, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015]]
        """
        final_jt_pos = []
        for traj in actuator_trajs:
            cur_traj = traj.copy()
            cur_traj += [self.min_joint_pos for i in range(12-len(traj))]
            final_jt_pos.append(cur_traj)
        final_jt_pos += [final_jt_pos[len(actuator_trajs)-1] for i in range(20-len(actuator_trajs))]
        final_jt_pos = np.array(final_jt_pos)

        assert final_jt_pos.shape == (20, 12), "Final joint positions must be of shape (20, 12)"
        for j in range(20):
            _ = [self.delta_message.trajectory.append(final_jt_pos[j][i]) for i in range(12)]
        self.send_proto_cmd()
        del self.delta_message.trajectory[:]


    def stop(self):
        self.esp01.send()

    def send_proto_cmd(self, ret_expected = False):
        serialized = self.delta_message.SerializeToString()
        self.esp01.send(b'\xa6~~'+ serialized + b'\xa7~~\r\n')
        if ret_expected:
            done_moving = self.esp01.recv(BUFFER_SIZE)
            print(done_moving, type(done_moving))
            return done_moving


    def move_joint_trajectory(self, desired_trajectory):
        desired_trajectory = np.clip(desired_trajectory,self.min_joint_pos,self.max_joint_pos)
        # Add joint positions to delta_message protobuf
        for i in range(20):
            _ = [self.delta_message.trajectory.append(desired_trajectory[i, j]) for j in range(12)]
        # self.send_proto_cmd()
        # print(self.delta_message)
        del self.delta_message.trajectory[:]

    def close(self):
        self.esp01.close()

    def get_done_state(self):
        _ = [[self.delta_message.trajectory.append(0) for j in 12] for i in range(20)]
        self.delta_message.request_done_state = True
        done_moving = self.send_proto_cmd(ret_expected = True)
        del self.delta_message.trajectory[:]
        self.delta_message.request_joint_pose = False
        self.delta_message.request_done_state = False
        self.delta_message.reset = False
        return done_moving

    def get_joint_positions(self):
        _ = [self.delta_message.joint_pos.append(0.5) for i in range(12)]
        self.delta_message.request_done_state = True
        self.current_joint_positions = self.send_proto_cmd(True)
        del self.delta_message.joint_pos[:]
        self.delta_message.request_joint_pose = False
        self.delta_message.request_done_state = False
        self.delta_message.reset = False
        # print(self.current_joint_positions)
