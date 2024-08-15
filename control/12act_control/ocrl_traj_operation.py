# import sys
# REF_PATH = ''  # TODO comment if necessary and replace '' with path to main folder
#  eg: "/User/NAME/Documents/deltawalker/"
# sys.path.append(REF_PATH)
from DeltaWalker import *
import kinematics_walker as k
import ocrl_paths as o
from control_helpers import *


def switch(trajectory, version, orientation, kinematics):
    path_header = f'../../julia_traj/orientation{orientation}/'
    if trajectory == 0:
        traj = 'walk'  # Walk
        label = 'A'
    elif trajectory == 1:
        traj = 'ambl'  # Amble
        label = 'B'
    elif trajectory == 2:
        traj = 'mtri'  # S-Gait
        label = 'C'
    elif trajectory == 3:
        traj = 'adjs'  # C-Gait
        label = 'D'
    elif trajectory == 4:
        traj = 'rots'  # Rotation in place
        label = 'E'
        path_header = f'../../julia_traj/rotations/'

    if version == 0:
        step = 'a'  # Step size 1.0 cm or rotation 15 degrees
    elif version == 1:
        step = 'b'  # Step size 1.5 cm or rotation 20 degrees
    elif version == 2:
        step = 'c'  # Step size 2.0 cm or rotation 30 degrees
    elif version == 3:
        step = 'd'  # Step size 2.5 cm or rotation 40 degrees
    elif version == 4:
        step = 'e'  # Move 6 cm in 3 steps as 1 iteration or rotation -15 degrees
    elif version == 5:
        step = 'dance'  # Dancing trajectory for fun - associated with rotation

    file_name = f"{traj}_{step}"
    save_name = f"traj{label}{step}"

    filepath = path_header + file_name + '.csv'
    paths_ik = o.TrajOCRL(kinematics, filepath=filepath, com_offset=0.02)
    paths_fk = o.TrajOCRL(kinematics, filepath=filepath, com_offset=0.02)
    return paths_ik, paths_fk, file_name, save_name


# Recorder for teleoprated random commands
def recorder_interactable(savename, env):
    cap = cv2.VideoCapture(0)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # 1920
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  # 1080
    size = (w, h)
    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')

    all_frames = []
    num_frames = 0
    start = time.time()

    # Live camera feed
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 0)

        # Store the frame
        all_frames.append(frame)

        # Display the frame
        cv2.imshow('Camera Feed', frame)

        num_frames += 1

        if cv2.waitKey(1) & 0xFF == ord('1'):
            print("walk")
            teleop(0, 2, orientation, env)  # walk 2 cm

        elif cv2.waitKey(1) & 0xFF == ord('2'):
            print("rotate")
            teleop(4, 0, orientation, env)  # rotate 15 3x

        elif cv2.waitKey(1) & 0xFF == ord('3'):
            print("mtri")
            teleop(2, 2, orientation, env)  # mtri 2 cm

        elif cv2.waitKey(1) & 0xFF == ord('4'):
            print("dance")
            teleop(4, 5, orientation, env)

        # Check for the 'q' key to quit
        elif cv2.waitKey(1) & 0xFF == ord('q'):
            end = time.time()
            print("Recording Stopped")
            break

    # Release the camera and video and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

    print("Saving Video")

    # Calculate frame rate
    elapsed_time = end - start
    fps = num_frames/elapsed_time

    # Write video frames at frame rate
    out = cv2.VideoWriter(f'./{savename}.avi', fourcc, fps, size)
    for i in np.arange(num_frames):
        out.write(all_frames[i])

    out.release()
    print("Video Saved")

    return


def teleop(trajectory, version, orientation, env):
    walker = k.DeltaWalkerKin()
    # Retrieve the trajectory
    paths_ik, paths_fk, file_name, save_name = switch(trajectory, version, orientation, walker)

    init_acts, init_steps = paths_ik.init_acts()
    init_motors = paths_ik.acts_to_motors(init_acts)

    # Generate the actuation amounts
    all_acts = paths_ik.gen_acts_all()  # 3D list shape steps x 4 x 3
    # One act generated has shape 4x3 (num feet x 3)
    # All acts is a list with steps # elements and each element is one act

    # Generate the motor ranges
    motor_ranges = paths_ik.acts_to_motors(all_acts)  # 2D numpy array with shape 12 (num acts total) x num steps

    # Get the given IK positions
    ee_pos_in = paths_ik.all_ee_shifted  # 2D numpy array with shape 12 (xyz for each delta) x num steps
    com_pos_in = paths_ik.com_pos  # 2D numpy array with shape 3 (xyz for COM) x num steps

    # Get the number of steps
    steps = paths_ik.steps

    # Get the FK estimates
    ee_pos_fk, com_pos_fk = paths_fk.fk_estimates(motor_ranges, steps)
    # ee_pos_fk is 2D numpy array with shape 12 (xyz for each delta) x num steps
    # com_pos_fk is 2D numpy array with shape 3 (xyz for COM) x num steps

    # Stack the motor ranges for the total number of iterations
    motor_ranges, steps = stack_ranges(motor_ranges, steps, iters)
    print(motor_ranges[6])
    motor_ranges = np.hstack((init_motors, motor_ranges))
    steps = steps + init_steps
    act_traj_all(env, motor_ranges, steps)
    return


if __name__ == '__main__':
    walker = k.DeltaWalkerKin()

    # TODO change usb port

    print("Connecting to Robot")
    # TODO change usb port
    env = DeltaWalkerEnv('/dev/cu.usbmodem143401', 9600)
    print("Robot Connected")
    env.start()
    print("Robot Started")
    env.reset()
    print("Robot Reset")

    # TODO toggle to choose which trajectory
    trajectory = 0
    version = 2

    # TODO toggle to denote which trial number
    trial = 1       # 1 - 3

    # TODO toggle to choose how many times it repeats
    iters = 3

    # TODO select direction AND FIX SAVE LOCATION TO MATCH IT
    orientation = 1
    # orientation = quadrant for ambl and walk (1 = q1, 2 = q2, 3 = q3, 4 = q4)
    # orientation = direction for mtri (1 = px, 2 = py, 3 = nx, 4 = ny)

    # Retrieve the trajectory
    paths_ik, paths_fk, file_name, save_name = switch(trajectory, version, orientation, walker)

    init_acts, init_steps = paths_ik.init_acts()
    init_motors = paths_ik.acts_to_motors(init_acts)

    # Generate the actuation amounts
    all_acts = paths_ik.gen_acts_all()  # 3D list shape steps x 4 x 3
    # One act generated has shape 4x3 (num feet x 3)
    # All acts is a list with steps # elements and each element is one act

    # Generate the motor ranges
    motor_ranges = paths_ik.acts_to_motors(all_acts)  # 2D numpy array with shape 12 (num acts total) x num steps

    # Get the given IK positions
    ee_pos_in = paths_ik.all_ee_shifted  # 2D numpy array with shape 12 (xyz for each delta) x num steps
    com_pos_in = paths_ik.com_pos  # 2D numpy array with shape 3 (xyz for COM) x num steps

    # Get the number of steps
    steps = paths_ik.steps

    # Get the FK estimates
    ee_pos_fk, com_pos_fk = paths_fk.fk_estimates(motor_ranges, steps)
    # ee_pos_fk is 2D numpy array with shape 12 (xyz for each delta) x num steps
    # com_pos_fk is 2D numpy array with shape 3 (xyz for COM) x num steps

    # Stack the motor ranges for the total number of iterations
    motor_ranges, steps = stack_ranges(motor_ranges, steps, iters)
    print(motor_ranges[6])
    motor_ranges = np.hstack((init_motors, motor_ranges))
    steps = steps + init_steps

    savename = f"{save_name}_t{trial}_o{orientation}_{iters}x"
    print(savename)
    print(f"Steps {steps}")
    savefolder = "1_ocrl_trials"  # TODO change this based on desired folder

    # Send the trajectories to the robot
    act_traj_all(env, motor_ranges, steps)
    recorder(savename, savefolder)

    # Comment out previous two lines and comment this in for teleop segments
    # recorder_interactable("demo", env)

    # observation_cam()
    print("Reset and Close")
    env.reset()
    env.close()
    print("Done")

