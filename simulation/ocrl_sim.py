import pybullet as p
import pybullet_data
import os
import recorder
from trajectories import *

import helpers as h
import kinematics_walker as k
import ocrl_paths as o

LEG_LEN = 0.05
FOOT_H = 0.018
np.set_printoptions(precision=5)


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

        offset = [0.04, 0.04, .01 + 0.04 + LEG_LEN]

        print(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/feet/foot.urdf')
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
    print(com_pos)
    return com_pos


def draw_ws(rec, cam1, cam2, delta, motor_ranges, steps, record_video, one_view):
    ee_positions1 = np.zeros((steps, 3))
    ee_positions2 = np.zeros((steps, 3))
    ee_positions3 = np.zeros((steps, 3))
    ee_positions4 = np.zeros((steps, 3))
    com_positions = np.zeros((steps, 3))

    cnt = 0
    pose_err_threshold = 1e-5

    acts = motor_ranges.shape[0]

    for i in range(steps):
        for j in np.arange(acts):
            motor_pos = motor_ranges[j, i]
            joint_id = f"motor_joint_{j}"
            p.setJointMotorControl2(bodyIndex=delta.robot, jointIndex=delta.jointNameToID[joint_id],
                                    controlMode=p.POSITION_CONTROL, targetPosition=motor_pos, targetVelocity=0.0,
                                    force=45, maxVelocity=0.15)

        ee_last_pos1 = np.asarray(p.getBasePositionAndOrientation(delta.sphereA)[0])
        ee_last_pos2 = np.asarray(p.getBasePositionAndOrientation(delta.sphereB)[0])
        ee_last_pos3 = np.asarray(p.getBasePositionAndOrientation(delta.sphereC)[0])
        ee_last_pos4 = np.asarray(p.getBasePositionAndOrientation(delta.sphereD)[0])

        for pp in range(100):
            p.stepSimulation()
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
                    print("Pose err is net ", pose_err, " e1 ", pose_err1, " e2 ", pose_err2, " e3 ", pose_err3, " e4 ", pose_err4, "at iter #", pp)
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

        com_pos = np.array(p.getBasePositionAndOrientation(delta.robot)[0]) - np.array([0, 0, FOOT_H + 0.04 + 0.01])
        com_positions[cnt, 0] = com_pos[0]
        com_positions[cnt, 1] = com_pos[1]
        com_positions[cnt, 2] = com_pos[2]

        if record_video:
            view1_img, _ = cam1.get_image()
            view2_img, _ = cam2.get_image()
            if one_view:
                rec.capture(view1_img.copy())
            else:
                rec.capture(view1_img.copy(), view2_img.copy())
        cnt += 1
    return ee_positions1, ee_positions2, ee_positions3, ee_positions4, com_positions


def switch(trajectory, version, orientation, kinematics):
    path_header = f'../julia_traj/orientation{orientation}/'
    if trajectory == 0:
        traj = 'walk'
        label = 'A'
        name = "Walk"
    elif trajectory == 1:
        traj = 'ambl'
        label = 'B'
        name = "Amble"
    elif trajectory == 2:
        traj = 'mtri'
        label = 'C'
        name = "Modified Tripedal"
    elif trajectory == 3:
        traj = 'adjs'
        label = 'D'
        name = "Adjacent Sequential"

    if version == 0:
        step = 'a'
        step_size = "1.0 cm"
    elif version == 1:
        step = 'b'
        step_size = "1.5 cm"
    elif version == 2:
        step = 'c'
        step_size = "2.0 cm"
    elif version == 3:
        step = 'd'
        step_size = "2.5 cm"
    elif version == 4:
        step = 'e'
        step_size = "1.0 cm"

    file_name = f"{traj}_{step}"
    save_name = f"traj{label}{step}_o{orientation}"
    title = f"{name} for Step Size {step_size}"

    filepath = path_header + file_name + '.csv'
    paths_ik = o.TrajOCRL(kinematics, filepath=filepath, com_offset=0.02)
    paths_fk = o.TrajOCRL(kinematics, filepath=filepath, com_offset=0.02)
    return paths_ik, paths_fk, file_name, save_name, title


if __name__ == '__main__':
    # TODO use this to toggle if the simulation floats or not
    #  (should generally leave as true unless trying out a new trajectory)
    nonfloat = True
    if nonfloat:
        label = "nonfloat"
        z_off = 0
        init_off = 0.005
    else:
        label = "float"
        z_off = 0
        init_off = 0.02

    delta_urdf = f"12act_{label}.urdf"

    kinematics = k.DeltaWalkerKin(nonfloat=nonfloat, leg_len=LEG_LEN)

    # TODO use this to change which trajectory you're using
    trajectory = 3
    step = 3

    # TODO select direction AND FIX SAVE LOCATION TO MATCH IT
    orientation = 1

    # Retrieve the trajectory
    paths_ik, paths_fk, file_name, save_name, title = switch(trajectory, step, orientation, kinematics)

    # TODO use this to toggle if the simulation should run or not (should generally leave as true)
    vis = True

    # Initialization
    init_acts, init_steps = paths_ik.init_acts()
    init_motors = paths_ik.acts_to_motors(init_acts)

    # Trajectory
    all_acts = paths_ik.gen_acts_all()  # 3D list shape steps x 4 x 3
    # One act generated has shape 4x3 (num feet x 3)
    # All acts is a list with steps# elements and each element is one act
    motor_ranges = paths_ik.acts_to_motors(all_acts)  # 2D numpy array with shape 12 (num acts total) x num steps

    # FK Positions
    ee_pos_fk, com_pos_fk = paths_fk.fk_estimates(motor_ranges, len(all_acts))
    ee_pos_fk, com_pos_fk = paths_fk.plot_world_convert(ee_pos_fk, com_pos_fk)

    # ee_pos_fk is 2D numpy array with shape 12 (xyz for each delta) x num steps
    # com_pos_fk is 2D numpy array with shape 3 (xyz for COM) x num steps

    # Desired Trajectory Unshifted
    ee_pos_in = paths_ik.all_ee_unshifted  # 2D numpy array with shape 12 (xyz for each delta) x num steps
    com_pos_in = paths_ik.com_unshifted  # 2D numpy array with shape 3 (xyz for COM) x num steps

    # Number of steps in trajectory
    steps = paths_ik.steps

    name = f"{file_name}_{label}"
    video_name = f"{save_name}.mp4"

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
        p.resetDebugVisualizerCamera(cameraDistance=0.2, cameraYaw=20, cameraPitch=0,
                                     cameraTargetPosition=[0, 0, 0.02], physicsClientId=physicsClient)

        delta1 = DeltaSim(delta_urdf, [0, 0.0, .01 + 0.04 + LEG_LEN + FOOT_H + init_off], [0, np.pi, np.pi])
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

            save_dir = os.path.join(".", 'results_julia', 'videos')
            os.makedirs(save_dir, exist_ok=True)
            video_path = os.path.join(save_dir, video_name)
            if one_view:
                rec = recorder.video_recorder_one_cam(view1_img.shape, path=video_path, fps=5)
            else:
                rec = recorder.video_recorder(view1_img.shape, view2_img.shape, path=video_path, fps=5)

        com_init = reset(delta1)
        # input('robot reset. press enter to continue')
        print(f"robot reset, COM position is {com_init}")

        ee1, ee2, ee3, ee4, com_pos = draw_ws(rec, cam1, cam2, delta1, init_motors, init_steps, False, True)
        # print("SIM INITED")

        ee1, ee2, ee3, ee4, com_pos = draw_ws(rec, cam1, cam2, delta1, motor_ranges, steps, record_video, one_view)
        # ee1 has shape (num steps, 3)

        ee_poses = np.hstack((ee1, ee2, ee3, ee4))

        if record_video:
            rec.release()
        print("DONE MOVING STUFF")

        step_ids = np.arange(steps)
        positions = np.vstack((step_ids, com_pos.T, ee_poses.T))
        np.savetxt(f"./results_julia/positions/{save_name}.csv", positions, fmt="%.4f", delimiter=",")
