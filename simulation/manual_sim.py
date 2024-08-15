import pybullet as p
import pybullet_data
import os
import recorder
from trajectories import *

import helpers as h
import path_gen as g
import kinematics_walker as k

LEG_LEN = 0.05
FOOT_H = 0.018


class DeltaSim:
    def __init__(self, urdf_file, pos, orientation):
        self.robotPos = pos  # [0, 0, 0.0]
        self.robotScale = 1
        self.robot = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/deltas/' + urdf_file,
                                self.robotPos,
                                p.getQuaternionFromEuler(orientation),  # [0, 0, 0]
                                useFixedBase=0,
                                globalScaling=self.robotScale)

        self.jointNameToID = {}
        self.linkNameToID = {}
        self.revoluteID = []
        self.prismaticID = []
        self.motorName = []

        for j in range(p.getNumJoints(self.robot)):
            info = p.getJointInfo(self.robot, j)
            # print(info)
            jointID = info[0]
            jointName = info[1].decode('UTF-8')
            jointType = info[2]
            if jointName[0:12] == "motor_joint_":
                self.motorName.append(jointName)
            if (jointType == p.JOINT_PRISMATIC):
                # print("prismatic: ", jointName)
                self.jointNameToID[jointName] = jointID
                self.linkNameToID[info[12].decode('UTF-8')] = jointID
                self.prismaticID.append(jointID)
            if (jointType == p.JOINT_REVOLUTE):
                # print("revolute: ", jointName)
                self.jointNameToID[jointName] = jointID
                self.linkNameToID[info[12].decode('UTF-8')] = jointID
                self.revoluteID.append(jointID)

        # print("jointNameToID: ", self.jointNameToID)
        # print("linkNameToID: ", self.linkNameToID)
        # print("revoluteID: ", self.revoluteID)
        # print("prismaticID: ", self.prismaticID)
        # print("motorName: ", self.motorName)

        offset = [0.04, 0.04, .01 + 0.04 + LEG_LEN]

        ballposA = [self.robotPos[0] + offset[0], self.robotPos[1], self.robotPos[2] - offset[2]]
        self.sphereA = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/feet/foot.urdf',
                                  ballposA,
                                  p.getQuaternionFromEuler([0, np.pi, -np.pi / 2]),
                                  globalScaling=self.robotScale)  # green - A

        ballposB = [self.robotPos[0], self.robotPos[1] + offset[1], self.robotPos[2] - offset[2]]
        self.sphereB = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/feet/foot.urdf',
                                  ballposB,
                                  p.getQuaternionFromEuler([0, np.pi, 0]),
                                  globalScaling=self.robotScale)  # red - B

        ballposC = [self.robotPos[0] - offset[0], self.robotPos[1], self.robotPos[2] - offset[2]]
        self.sphereC = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/feet/foot.urdf',
                                  ballposC,
                                  p.getQuaternionFromEuler([0, np.pi, np.pi / 2]),
                                  globalScaling=self.robotScale)  # orange - C

        ballposD = [self.robotPos[0], self.robotPos[1] - offset[1], self.robotPos[2] - offset[2]]
        self.sphereD = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/feet/foot.urdf',
                                  ballposD,
                                  p.getQuaternionFromEuler([0, np.pi, np.pi]),
                                  globalScaling=self.robotScale)  # blue - D

        # print("SPHERE INFO")
        # print(p.getJointInfo(self.sphereD))

        def get_a_set(center, top, topl, topr, right_angle, left_angle):
            bottom_right = h.rotate(center, top, right_angle);
            bottom_rightl = h.rotate(center, topl, right_angle);
            bottom_rightr = h.rotate(center, topr, right_angle)
            bottom_left = h.rotate(center, top, left_angle);
            bottom_leftl = h.rotate(center, topl, left_angle);
            bottom_leftr = h.rotate(center, topr, left_angle)
            return top, topl, topr, bottom_right, bottom_rightl, bottom_rightr, bottom_left, bottom_leftl, bottom_leftr

        all_spheres = [self.sphereA, self.sphereB, self.sphereC, self.sphereD]
        all_set_angle = [180, 180, 180, 180]
        all_constraint_position = []

        # connect links of delta to the end effector (foot)
        for i in range(len(all_spheres)):
            # set 1
            center = [0.0, 0.0]
            delta_offset = 0.006
            topP = [0.0, delta_offset]
            topPl = [-0.005, delta_offset]
            topPr = [0.005, delta_offset]

            set_angle = all_set_angle[i]

            topP = h.rotate(center, topP, set_angle)
            topPl = h.rotate(center, topPl, set_angle)
            topPr = h.rotate(center, topPr, set_angle)

            right_angle = 120
            left_angle = -120
            topP, topPl, topPr, bottom_right, bottom_rightl, bottom_rightr, bottom_left, bottom_leftl, bottom_leftr = get_a_set(
                center, topP, topPl, topPr, right_angle, left_angle)

            # from top left leg, clockwise circle
            constraint_position = [[[topPl[0], topPl[1], 0.0], [topPr[0], topPr[1], 0.0]],
                                   [[bottom_rightl[0], bottom_rightl[1], 0.0],
                                    [bottom_rightr[0], bottom_rightr[1], 0.0]],
                                   [[bottom_leftl[0], bottom_leftl[1], 0.0], [bottom_leftr[0], bottom_leftr[1], 0.0]]]
            all_constraint_position.append(constraint_position)

        A_parent_links = ['parallel_0_', 'parallel_2_', 'parallel_1_']
        B_parent_links = ['parallel_3_', 'parallel_5_', 'parallel_4_']
        C_parent_links = ['parallel_6_', 'parallel_8_', 'parallel_7_']
        D_parent_links = ['parallel_9_', 'parallel_11_', 'parallel_10_']

        all_parent_links = [A_parent_links, B_parent_links, C_parent_links, D_parent_links]

        for i in range(len(all_parent_links)):
            parent_links = all_parent_links[i]
            constraint_position = all_constraint_position[i]
            sphere = all_spheres[i]
            for j in range(len(parent_links)):
                # print(constraint_position[j])
                p.createConstraint(parentBodyUniqueId=self.robot,
                                   parentLinkIndex=self.linkNameToID[parent_links[j] + 'leg1'],
                                   childBodyUniqueId=sphere,
                                   childLinkIndex=-1,
                                   jointType=p.JOINT_POINT2POINT,
                                   jointAxis=[0, 0, 0],
                                   parentFramePosition=[0, 0, LEG_LEN],
                                   childFramePosition=constraint_position[j][0])
                p.createConstraint(parentBodyUniqueId=self.robot,
                                   parentLinkIndex=self.linkNameToID[parent_links[j] + 'leg2'],
                                   childBodyUniqueId=sphere,
                                   childLinkIndex=-1,
                                   jointType=p.JOINT_POINT2POINT,
                                   jointAxis=[0, 0, 0],
                                   parentFramePosition=[0, 0, LEG_LEN],
                                   childFramePosition=constraint_position[j][1])

        motorID = []
        for motor_name in self.motorName:
            motorID.append(self.jointNameToID[motor_name])
        for i in self.revoluteID:
            if i in motorID:
                pass
            else:
                # set friction of the other joint to 0.001
                p.setJointMotorControl2(self.robot, i, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0.001)
        p.setGravity(0, 0, -9.81)

        print(motorID)
        print(self.jointNameToID)


def reset(delta):
    for motor_name in delta.motorName:
        if delta.jointNameToID[motor_name] in delta.prismaticID:
            p.setJointMotorControl2(bodyIndex=delta.robot, jointIndex=delta.jointNameToID[motor_name],
                                    controlMode=p.POSITION_CONTROL, targetPosition=0.0, targetVelocity=0.0,
                                    force=45)
        if delta.jointNameToID[motor_name] in delta.revoluteID:
            p.setJointMotorControl2(bodyIndex=delta.robot, jointIndex=delta.jointNameToID[motor_name],
                                    controlMode=p.POSITION_CONTROL, targetPosition=-1.5, targetVelocity=0.0,
                                    force=25)
    for pp in range(100):
        p.stepSimulation()
    com_pos = np.array(p.getBasePositionAndOrientation(delta.robot)[0]) - np.array([0, 0, 0.04 + FOOT_H + 0.01])
    # print(com_pos)


def draw_ws(rec, cam1, cam2, delta, motor_ranges, steps, record_video, one_view):
    ee_positions1 = np.zeros((steps, 3))
    ee_positions2 = np.zeros((steps, 3))
    ee_positions3 = np.zeros((steps, 3))
    ee_positions4 = np.zeros((steps, 3))
    com_positions = np.zeros((steps, 3))
    cnt = 0
    pose_err_threshold = 1e-6

    acts = motor_ranges.shape[0]
    for i in range(steps):
        for j in np.arange(acts):
            motor_pos = motor_ranges[j, i]
            joint_id = f"motor_joint_{j}"
            p.setJointMotorControl2(bodyIndex=delta.robot, jointIndex=delta.jointNameToID[joint_id],
                                    controlMode=p.POSITION_CONTROL, targetPosition=motor_pos, targetVelocity=0.0,
                                    force=45, maxVelocity=0.5)
            
        ee_last_pos1 = np.asarray(p.getBasePositionAndOrientation(delta.sphereA)[0])
        ee_last_pos2 = np.asarray(p.getBasePositionAndOrientation(delta.sphereB)[0])
        ee_last_pos3 = np.asarray(p.getBasePositionAndOrientation(delta.sphereC)[0])
        ee_last_pos4 = np.asarray(p.getBasePositionAndOrientation(delta.sphereD)[0])

        for pp in range(100):
            p.stepSimulation()
            # time.sleep(5)
            # check the stability
            try:
                ee_pos1 = np.asarray(p.getBasePositionAndOrientation(delta.sphereA)[0])
                ee_pos2 = np.asarray(p.getBasePositionAndOrientation(delta.sphereB)[0])
                ee_pos3 = np.asarray(p.getBasePositionAndOrientation(delta.sphereC)[0])
                ee_pos4 = np.asarray(p.getBasePositionAndOrientation(delta.sphereD)[0])
                pose_err1 = np.abs(ee_last_pos1[0] - ee_pos1[0]) + np.abs(ee_last_pos1[1] - ee_pos1[1]) + np.abs(
                    ee_last_pos1[2] - ee_pos1[2])
                pose_err2 = np.abs(ee_last_pos2[0] - ee_pos2[0]) + np.abs(ee_last_pos2[1] - ee_pos2[1]) + np.abs(
                    ee_last_pos2[2] - ee_pos2[2])
                pose_err3 = np.abs(ee_last_pos3[0] - ee_pos3[0]) + np.abs(ee_last_pos3[1] - ee_pos3[1]) + np.abs(
                    ee_last_pos3[2] - ee_pos3[2])
                pose_err4 = np.abs(ee_last_pos4[0] - ee_pos4[0]) + np.abs(ee_last_pos4[1] - ee_pos4[1]) + np.abs(
                    ee_last_pos4[2] - ee_pos4[2])

                pose_err = pose_err1 + pose_err2 + pose_err3 + pose_err4
                if pose_err <= pose_err_threshold:
                    # print("Pose err is ", pose_err1, " ", pose_err2, " ", pose_err3, " ", pose_err4, "at iter #", pp)
                    break
                else:
                    # print("Pose err is ", pose_err, "at iter #", pp)
                    pass
                ee_last_pos1 = ee_pos1.copy()
                ee_last_pos2 = ee_pos2.copy()
                ee_last_pos3 = ee_pos3.copy()
                ee_last_pos4 = ee_pos4.copy()
            except:
                print("SIM ERR! Pose err is ", pose_err1 + pose_err2 + pose_err3 + pose_err4, "at iter #", pp)
                break
        ee_pos1 = p.getBasePositionAndOrientation(delta.sphereA)[0]
        ee_pos2 = p.getBasePositionAndOrientation(delta.sphereB)[0]
        ee_pos3 = p.getBasePositionAndOrientation(delta.sphereC)[0]
        ee_pos4 = p.getBasePositionAndOrientation(delta.sphereD)[0]

        ee_positions1[cnt, 0] = ee_pos1[0]
        ee_positions1[cnt, 1] = ee_pos1[1]
        ee_positions1[cnt, 2] = ee_pos1[2] - FOOT_H

        ee_positions2[cnt, 0] = ee_pos2[0]
        ee_positions2[cnt, 1] = ee_pos2[1]
        ee_positions2[cnt, 2] = ee_pos2[2] - FOOT_H

        ee_positions3[cnt, 0] = ee_pos3[0]
        ee_positions3[cnt, 1] = ee_pos3[1]
        ee_positions3[cnt, 2] = ee_pos3[2] - FOOT_H

        ee_positions4[cnt, 0] = ee_pos4[0]
        ee_positions4[cnt, 1] = ee_pos4[1]
        ee_positions4[cnt, 2] = ee_pos4[2] - FOOT_H

        com_pos = np.array(p.getBasePositionAndOrientation(delta.robot)[0]) - np.array([0, 0, 0.04 + FOOT_H + 0.01])
        com_positions[cnt, 0] = com_pos[0]
        com_positions[cnt, 1] = com_pos[1]
        com_positions[cnt, 2] = com_pos[2]

        # print(ee_positions4[cnt,2])

        # print(cnt)
        # print("EE 1: ", ee_positions1[cnt, :])
        # print("EE 2: ", ee_positions2[cnt, :])
        # print("EE 3: ", ee_positions3[cnt, :])
        # print("EE 4: ", ee_positions4[cnt, :])

        if record_video:
            view1_img, _ = cam1.get_image()
            view2_img, _ = cam2.get_image()
            if one_view:
                rec.capture(view1_img.copy())
            else:
                rec.capture(view1_img.copy(), view2_img.copy())
        cnt += 1
    return ee_positions1, ee_positions2, ee_positions3, ee_positions4, com_positions


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
        traj_name = "trajB0_o1"
    elif case == 1:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB0_o2"
    elif case == 2:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB0_o3"
    elif case == 3:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB0_o4"

    # FB V1 - ambl - previously noted as trajE
    elif case == 4:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB1_o1"
    elif case == 5:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB1_o2"
    elif case == 6:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB1_o3"
    elif case == 7:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB1_o4"

    # FB V2 - ambl - previously noted as trajF
    elif case == 8:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB2_o1"
    elif case == 9:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB2_o2"
    elif case == 10:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB2_o3"
    elif case == 11:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = ambl_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajB2_o4"

    # FBS V0 - mtri  - previously noted as trajG
    elif case == 12:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v0(paths, 1, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC0_o1"
    elif case == 13:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v0(paths, 2, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC0_o2"
    elif case == 14:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v0(paths, 3, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC0_o3"
    elif case == 15:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v0(paths, 4, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC0_o4"

    # FBS V1 - mtri  - previously noted as trajH
    elif case == 16:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v1(paths, 1, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC1_o1"
    elif case == 17:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v1(paths, 2, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC1_o2"
    elif case == 18:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v1(paths, 3, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC1_o3"
    elif case == 19:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v1(paths, 4, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC1_o4"

    # FBS V2 - mtri - previously noted as trajI
    elif case == 20:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v2(paths, 1, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC2_o1"
    elif case == 21:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v2(paths, 2, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC2_o2"
    elif case == 22:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v2(paths, 3, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC2_o3"
    elif case == 23:
        all_pos, all_acts, all_pos_fk, steps, gait_name = mtri_v2(paths, 4, step=step, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajC2_o4"

    # FB VO - walk - previously noted as trajJ
    elif case == 24:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA0_o1"
    elif case == 25:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA0_o2"
    elif case == 26:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA0_o3"
    elif case == 27:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v0(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA0_o4"

    # FB V1 - walk - previously noted as trajK
    elif case == 28:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA1_o1"
    elif case == 29:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA1_o2"
    elif case == 30:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA1_o3"
    elif case == 31:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v1(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA1_o4"

    # FB V2 - walk - previously noted as trajL
    elif case == 32:
        x = step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA2_o1"
    elif case == 33:
        x = -step_part
        y = step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA2_o2"
    elif case == 34:
        x = -step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA2_o3"
    elif case == 35:
        x = step_part
        y = -step_part
        all_pos, all_acts, all_pos_fk, steps, gait_name = walk_v2(paths, x, y, cycle_steps=cycle_steps, num_iters=iters)
        traj_name = "trajA2_o4"

    return all_pos, all_acts, all_pos_fk, steps, gait_name, traj_name


if __name__ == '__main__':
    # TODO use this to toggle if the simulation floats or not
    #  (should generally leave as true unless trying out a new trajectory)
    nonfloat = True
    if nonfloat:
        label = "nonfloat"
        z_off = 0
    else:
        label = "float"
        z_off = 0

    delta_urdf = f"12act_{label}.urdf"

    # kinematics = k.DeltaWalkerKin(nonfloat=nonfloat, leg_len=LEG_LEN)
    # floating delta but nonfloat delta kinematics
    kinematics = k.DeltaWalkerKin(nonfloat=not nonfloat, leg_len=LEG_LEN)
    paths = g.GenPaths(kinematics)

    # TODO use this to toggle if the simulation should run or not (should generally leave as true)
    vis = True

    # TODO use this to change which trajectory you're using
    case = 35

    # TODO use this to toggle how many iterations of a step trajectory to create
    iters = 1

    # TODO use this to toggle how many intermediate steps are created for each segment

    # TODO use this to toggle the desired step size
    cycle_steps = 5
    step = 0.01

    all_pos, all_acts, all_pos_fk, all_pos_com, steps, gait_name, save_name = switch(paths, case, step, cycle_steps, iters)
    motor_ranges = paths.acts_to_motors(all_acts, steps)

    push_acts, push_steps = gen_pushes(paths)
    motor_pushes = paths.acts_to_motors(push_acts, push_steps)

    name = f"{gait_name}_{label}_iters{iters}"
    video_name = f"traj{save_name}.mp4"

    if vis:
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        p.setTimeStep(1 / 1000)
        planeID = p.loadURDF('plane.urdf')

        # p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=0, cameraPitch=-89,
        #                              cameraTargetPosition=[-0, 0.045, 0.01], physicsClientId=physicsClient)

        # TODO you can change the viewing angles here
        #  (pitch=-89 for top view, pitch=89 for bottom view)
        p.resetDebugVisualizerCamera(cameraDistance=0.2, cameraYaw=0, cameraPitch=0,
                                     cameraTargetPosition=[0, 0, 0.02], physicsClientId=physicsClient)

        delta1 = DeltaSim(delta_urdf, [0, 0.0, .01 + 0.04 + LEG_LEN + FOOT_H + 0.02], [0, np.pi, np.pi])
        # base height + motor height + leg length + foot height + actuation limits

        # TODO use this to toggle if the video should be recorded (generally leave as true)
        record_video = True
        # TODO use this to decide if the video should record one or two views (generally leave as 1 so true)
        one_view = True
        if record_video:
            cam1 = recorder.Camera(p, cameraDistance=0.2, cameraYaw=0, cameraPitch=-20,
                                   cameraTargetPosition=[-0, 0, 0.01], cameraResolution=[640, 480])
            cam2 = recorder.Camera(p, cameraDistance=0.3, cameraYaw=0, cameraPitch=-89,
                                   cameraTargetPosition=[-0, 0, 0.0], cameraResolution=[640, 480])

            view1_img, _ = cam1.get_image()
            view2_img, _ = cam2.get_image()

            save_dir = os.path.join(".", 'results', 'videos')
            os.makedirs(save_dir, exist_ok=True)
            video_path = os.path.join(save_dir, video_name)
            if one_view:
                rec = recorder.video_recorder_one_cam(view1_img.shape, path=video_path, fps=5)
            else:
                rec = recorder.video_recorder(view1_img.shape, view2_img.shape, path=video_path, fps=5)
        print(f"{name}, {steps} steps")
        # input('robot loaded. press enter to continue')
        reset(delta1)
        print("robot reset")

        ee1, ee2, ee3, ee4, com_pos = draw_ws(rec, cam1, cam2, delta1, motor_ranges, steps, record_video, one_view)
        # ee1 has shape (num steps, 3)

        print(com_pos.T[2, :])

        ee_poses = np.hstack((ee1, ee2, ee3, ee4))

        step_ids = np.arange(steps)
        positions = np.vstack((step_ids, com_pos.T, ee_poses.T))
        np.savetxt(f"./results/positions/traj{save_name}.csv", positions, fmt="%.4f", delimiter=",")
        fk_poses = np.vstack((step_ids, all_pos_com.T, all_pos_fk.T))
        np.savetxt(f"./results/gen_traj/traj{save_name}.csv", fk_poses, fmt="%.4f", delimiter=",")

        if record_video:
            rec.release()
        print("DONE MOVING STUFF")
