import math
import numpy as np

# Constants
PI = math.pi
SIN30 = 0.5
COS30 = np.sqrt(3)/2
TAN30 = 1/np.sqrt(3)
TAN60 = np.sqrt(3)


# Kinematics Reference: https://www.ri.cmu.edu/app/uploads/2021/12/Masters-Thesis-Final-Version.pdf
class PDelta():
    def __init__(self, ee_len=0.006, base_len=0.02, leg_len=0.05):
        # Delta parameters as defined by CAD
        #   ee_len: distance from platform centroid to the side of the triangle (inradius)
        #   base_len: distance from base centroid to the vertex of the triangle (circumradius)
        #   leg_len: length of the delta leg

        # Key for number labels of points on each triangle
        #   0: Origin
        #   1: Bottom
        #   2: Top Right
        #   3: Top Left

        # Delta parameters as defined by kinematics reference
        self.s_b = base_len * 2 * np.sqrt(3)        # base side length
        self.s_p = ee_len * 2 * np.sqrt(3)          # platform side length
        self.l = leg_len
        self.u_b = base_len                         # base circumradius
        self.u_p = ee_len                           # platform circumradius

        # Base Joint Locations (these values should never change)
        self.b0 = np.zeros(3)
        self.b1 = self.b0 + np.array([0, -self.u_b, 0])
        self.b2 = self.b0 + np.array([self.u_b * COS30, self.u_b * SIN30, 0])
        self.b3 = self.b0 + np.array([-self.u_b * COS30, self.u_b * SIN30, 0])

        # Motor Actuation Amounts
        self.h1 = 0                 # Motor 1
        self.h2 = 0                 # Motor 2
        self.h3 = 0                 # Motor 3

        # Knee Joint positions
        self.k1 = np.zeros(3)       # Knee 1
        self.k2 = np.zeros(3)       # Knee 2
        self.k3 = np.zeros(3)       # Knee 3
        self.update_knee_positions()

        # Platform Joint Locations
        self.p0 = np.zeros(3)
        self.p1 = np.zeros(3)
        self.p2 = np.zeros(3)
        self.p3 = np.zeros(3)
        self.update_platform_corners()

        self.b0_prev = np.zeros(3)
        self.h1_prev = 0
        self.h2_prev = 0
        self.h3_prev = 0
        self.p0_prev = np.zeros(3)

        self.go_home()

        # TODO do i need to store previous positions?
        # TODO do i need to tell it to just go to a fixed distance away as home?
        # TODO update functions
        #   walker calls go_to for IK and go_to_distance for FK

    # REPORTING
    def report_distance(self):
        print('AT DISTANCE {:0.2f}, {:0.2f}, {:0.2f}'.format(self.t1, self.t2, self.t3))

    def report_position(self):
        print('AT POS {:0.2f}, {:0.2f}, {:0.2f}'.format(self.x0, self.y0, self.z0))

    # UPDATING PARAMETERS
    def update_platform_corners(self):
        self.p1 = self.p0 + np.array([0, -self.u_p, 0])
        self.p2 = self.p0 + np.array([self.u_p * COS30, self.u_p * SIN30, 0])
        self.p3 = self.p0 + np.array([-self.u_p * COS30, self.u_p * SIN30, 0])
        return

    def update_knee_positions(self):
        self.k1 = self.b1 - np.array([0, 0, self.h1])
        self.k2 = self.b2 - np.array([0, 0, self.h2])
        self.k3 = self.b3 - np.array([0, 0, self.h3])
        return

    def update_prev_positions(self):
        self.b0_prev = self.b0
        self.h1_prev = self.h1
        self.h2_prev = self.h2
        self.h3_prev = self.h3
        self.p0_prev = self.p0
        return

    # MOVE FUNCTIONS
    def go_home(self):
        self.h1, self.h2, self.h3 = 0, 0, 0
        self.update_knee_positions()
        self.go_to_distance(0, 0, 0)
        # self.go_to_position(0, 0, -0.04)
        return

    # Given a position, use IK to calculate the motor actuation amounts
    def go_to_position(self, x, y, z):
        self.update_prev_positions()
        self.p0 = np.array([x, y, z])
        self.update_platform_corners()
        state = self.delta_calc_inverse()
        if state == 0:
            pass
        else:
            # TODO fix this section
            print("Err in IK!")
            self.go_home()
        return self.h1, self.h2, self.h3

    # Given motor actuation amounts, use IK to calculate the EE position
    def go_to_distance(self, dst_1, dst_2, dst_3):
        self.update_prev_positions()
        p0_prev = self.p0

        state = self.delta_calc_forward(dst_1, dst_2, dst_3)
        if state == 0:
            self.h1, self.h2, self.h3 = dst_1, dst_2, dst_3
            self.update_knee_positions()
        else:
            # TODO fix this section
            print("Err in FK!")
            self.go_home()

        return self.p0[0], self.p0[1], self.p0[2]

    # Calculate inverse kinematics based on EE position
    # Returns a 0 if the position is feasible and -1 otherwise
    def delta_calc_inverse(self):
        # Calculate XY squared dist from the end effector center to prismatic actuator axis
        d1 = np.sum(np.power(self.p1[0:2] - self.b1[0:2], 2))
        d2 = np.sum(np.power(self.p2[0:2] - self.b2[0:2], 2))
        d3 = np.sum(np.power(self.p3[0:2] - self.b3[0:2], 2))

        # Check if the end effector center is too far to be feasible
        l_squared = np.power(self.l, 2)
        if d1 > l_squared or d2 > l_squared or d3 > l_squared:
            return -1

        # IK calculation of the motor actuation amounts
        self.h1 = -1*(self.p0[2] + np.sqrt(l_squared - d1))
        self.h2 = -1*(self.p0[2] + np.sqrt(l_squared - d2))
        self.h3 = -1*(self.p0[2] + np.sqrt(l_squared - d3))

        self.update_knee_positions()

        return 0

    # Calculate FK based on given motor actuation amounts
    # Returns a 0 if the position is feasible and -1 otherwise
    def delta_calc_forward(self, dst_1, dst_2, dst_3):
        # Find corners of a new triangle

        t = (self.s_b - self.s_p) * TAN30 / 2  # equivalent to u_b - u_p

        # Corner 1 (x1 = 0)
        y1 = -1 * t
        z1 = -1 * dst_1

        # Corner 2
        x2 = t * COS30
        y2 = t * SIN30
        z2 = -1 * dst_2

        # Corner 3
        x3 = -t * COS30
        y3 = t * SIN30
        z3 = -1 * dst_3

        # Detn Value
        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        # x = (a1*z + b1)/dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        # y = (a2*z + b2)/dnm
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # a*z^2 + b*z + c = 0
        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - self.l * self.l)

        # discriminant
        d = b * b - (4.0 * a * c)
        if d < 0:
            return -1  # non-existing point

        z0 = -(0.5 * (b + np.sqrt(d)) / a)
        x0 = (a1 * z0 + b1) / dnm
        y0 = (a2 * z0 + b2) / dnm

        self.p0 = np.array([x0, y0, z0])
        self.update_platform_corners()

        return 0
