import numpy as np

# adds reset to untwist the robot (all motors sent back to the first position)
if __name__ == "__main__":
    load_path = './rotations/'
    file_name = 'rots_'
    file_version = ['a', 'b', 'c', 'd', 'e']

    for version in file_version:
        data = np.loadtxt(f"{load_path}original/{file_name}{version}.csv", skiprows=1, delimiter=',')[:15]
        reset = data[:, 0].reshape((-1, 1))
        mod_data = np.hstack((data, reset))

        steps = mod_data.shape[1]
        heading = np.arange(steps)
        results = np.vstack((heading, mod_data))
        np.savetxt(f"{load_path}{file_name}{version}.csv", results, fmt='%.20f', delimiter=',')
