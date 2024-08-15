import numpy as np
import time
import cv2


def act_traj_steps(env, motor_ranges, steps):
    for i in np.arange(steps):
        traj = motor_ranges[:, i] * 1000
        env.move_joint_position([traj.tolist()], [1.0])
        time.sleep(0.25)
    return


def act_traj_all(env, motor_ranges, steps):
    durations = np.ones(steps)
    all_traj = []
    for i in np.arange(steps):
        traj = motor_ranges[:, i] * 1000
        all_traj.append(traj.tolist())
    env.move_joint_position(all_traj, durations)
    return


def slow_reset(end_traj, steps=2, end_pos=0.01):
    # end traj = 1d numpy array with len 12
    # want to return motor ranges where each column is a step of the trajectory
    num_motors = len(end_traj)
    motor_ranges = np.zeros((num_motors, steps))
    for i in np.arange(num_motors):
        motor_ranges[i, :] = np.linspace(end_traj[i], end_pos, num=steps)
    return motor_ranges, steps


def move(zero=False):
    steps = 1
    act = 0.01
    if zero:
        motor_ranges = np.zeros((12, steps))
    else:
        motor_ranges = act * np.ones((12, steps))
        # motor_ranges[0, 0] = 0.02
        motor_ranges = np.array([0.02, 0.00, 0.00,
                                 0.00, 0.00, 0.00,
                                 0.00, 0.00, 0.00,
                                 0.00, 0.00, 0.00])
    motor_ranges = motor_ranges.reshape((12, -1))
    return motor_ranges, steps


def stack_ranges(motor_ranges, steps, iters):
    full_range = motor_ranges.copy()
    for _ in np.arange(iters-1):
        full_range = np.hstack((full_range, motor_ranges))
    full_steps = steps * iters
    return full_range, full_steps


def recorder(savename, savefolder):
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

        # Check for the 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
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
    out = cv2.VideoWriter(f'../../testing/{savefolder}/videos/raw/{savename}.avi', fourcc, fps, size)
    for i in np.arange(num_frames):
        out.write(all_frames[i])

    out.release()
    print("Video Saved")

    return


def observation_cam():
    cap = cv2.VideoCapture(0)
    start = time.time()

    # Live camera feed
    print("Start Camera")
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 0)

        # Display the frame
        cv2.imshow('Camera Feed', frame)

        # Check for the 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            end = time.time()
            print("End Livecam")
            break

    # Release the camera and video and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    return

