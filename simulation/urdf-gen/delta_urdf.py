import numpy as np
from odio_urdf import *
import delta_materials

import matplotlib.pyplot as plt

umass = 0.001  # 1 g
uinertia = [1e-6, 0.0, 0.0, 1e-6, 0.0, 1e-6]
# origin: x, y, z, r, p, y


def create_origin(origin):
    xyz = tuple(origin[:3])
    rpy = tuple(origin[3:])
    return Origin(xyz, rpy=rpy)


def create_inertia(uninertia):
    return Inertia(uninertia[0], ixy=uninertia[1], ixz=uninertia[2], iyy=uninertia[3], iyz=uninertia[4], izz=uninertia[5])


def base_link(origin, geometry, material):
    # Base approximately 250 g
    link = Link(
        Inertial(Mass(value=umass*250), create_inertia(uinertia)),
        Visual(create_origin(origin), Geometry(geometry), Material(material)),
        Collision(create_origin(origin), Geometry(geometry), Material(material)),
        name="base_link",
    )
    return link


def base_joint(origin):
    joint = Joint(
        Parent(link="world"),
        Child(link="base_link"),
        create_origin(origin),
        name="base_joint",
        type="fixed",
    )
    return joint


def motor_link(N, origin, geometry, material):
    # Motors are 21g
    N = str(N)
    link = Link(
        Inertial(Mass(value=umass*21), create_inertia(uinertia)),
        Visual(create_origin(origin),Geometry(geometry),Material(material)),
        Collision(create_origin(origin), Geometry(geometry),Material(material)),
        name="motor_link_"+N
    )
    return link


def motor_joint(N, type, origin, limit=None, axis=None):
    N = str(N)
    joint = Joint(
        Axis(axis),
        Limit(lower=limit[0], upper=limit[1], effort=100, velocity=1),
        Parent(link="base_link"),
        Child(link= "motor_link_"+N),
        create_origin(origin),
        name="motor_joint_"+N,
        type=type
    )
    return joint


def leg_link(leg_name, origin, geometry, material):
    link = Link(
        Inertial(Mass(value=umass*5), create_inertia(uinertia)),
        Visual(create_origin(origin),Geometry(geometry),Material(material)),
        Collision(create_origin(origin), Geometry(geometry),Material(material)),
        name = leg_name
    )
    return link


def leg_joint(name, parent, child, origin, limit=None, axis=None):
    joint = Joint(
        Axis(axis),
        Limit(lower=limit[0], upper=limit[1], effort=100, velocity=1),
        Parent(link = parent),
        Child(link = child),
        create_origin(origin),
        name = name,
        type = "revolute"
    )
    return joint


def knee_link(N, origin, geometry, material):
    N = str(N)
    link = Link(
        Inertial(Mass(value=umass*2), create_inertia(uinertia)),
        Visual(create_origin(origin),Geometry(geometry),Material(material)),
        Collision(create_origin(origin), Geometry(geometry),Material(material)),
        name = "parallel_"+N+"_knee"
    )
    return link


def knee_joint(N, parent, origin, limit=None, axis=None):
    N = str(N)
    joint = Joint(
        Axis(axis),
        Limit(lower=limit[0], upper=limit[1], effort=100, velocity=1),
        Parent(link = parent),
        Child(link = "parallel_"+N+"_knee"),
        create_origin(origin),
        name = "parallel_"+N+"_kneejoint",
        type = "revolute"
    )
    return joint


def bar_link(N, origin, geometry, material):
    N = str(N)
    link = Link(
        Inertial(Mass(value=umass*7), create_inertia(uinertia)),
        Visual(create_origin(origin),Geometry(geometry),Material(material)),
        Collision(create_origin(origin), Geometry(geometry),Material(material)),
        name = "bar_link_"+N
    )
    return link


def bar_joint(N, parent, child, origin):
    N = str(N)
    joint = Joint(
        Parent(link = parent),
        Child(link = child),
        create_origin(origin),
        name = "bar_joint_"+N,
        type = "fixed"
    )
    return joint


def parallel_leg(N, origin, parent):
    leg_len = 0.05
    leg_lw = 0.003
    # include a knee joint, a knee bar, two leg joints, two parallel legs
    N = str(N)
    kknee_joint = knee_joint(N, parent, origin, limit=[-np.pi, np.pi], axis=[1, 0, 0])
    kknee_link = knee_link(N, [0, 0, 0, 0, 0.5*np.pi, 0], Cylinder(radius=0.0025, length=0.015), "Red")
    leg_joint1 = leg_joint("parallel_"+N+"_legjoint1", "parallel_"+N+"_knee", "parallel_"+N+"_leg1", [-0.005, 0, 0, 0, 0, 0], limit=[-0.5*np.pi, 0.5*np.pi], axis=[0, 1, 0])
    leg_joint2 = leg_joint("parallel_"+N+"_legjoint2", "parallel_"+N+"_knee", "parallel_"+N+"_leg2", [0.005, 0, 0, 0, 0, 0], limit=[-0.5*np.pi, 0.5*np.pi], axis=[0, 1, 0])
    leg1 = leg_link("parallel_"+N+"_leg1", [0, 0, leg_len/2, 0, 0, 0], Box(size=[leg_lw, leg_lw, leg_len]), "BananaYellow")
    leg2 = leg_link("parallel_"+N+"_leg2", [0, 0, leg_len/2, 0, 0, 0], Box(size=[leg_lw, leg_lw, leg_len]), "White")

    return(Group(kknee_joint, kknee_link, leg_joint1, leg_joint2, leg1, leg2))


def double_parallel_leg(N, origin, parent):
    # include a knee joint, a knee bar, 2 x (two leg joints, two parallel legs) in parallel
    N = str(N)
    kknee_joint = knee_joint(N, parent, origin, limit=[-np.pi,np.pi], axis=[1,0,0])
    kknee_link = knee_link(N, [0,0,0,0,0.5*np.pi,0], Cylinder(radius=0.001,length=0.03), "Red")
    leg_joint1 = leg_joint("parallel_"+N+"_legjoint1", "parallel_"+N+"_knee", "parallel_"+N+"1_leg1", [-0.012, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg_joint2 = leg_joint("parallel_"+N+"_legjoint2", "parallel_"+N+"_knee", "parallel_"+N+"1_leg2", [-0.002, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg1 = leg_link("parallel_"+N+"1_leg1", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")
    leg2 = leg_link("parallel_"+N+"1_leg2", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")

    leg_joint3 = leg_joint("parallel_"+N+"_legjoint3", "parallel_"+N+"_knee", "parallel_"+N+"2_leg1", [0.002, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg_joint4 = leg_joint("parallel_"+N+"_legjoint4", "parallel_"+N+"_knee", "parallel_"+N+"2_leg2", [0.012, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg3 = leg_link("parallel_"+N+"2_leg1", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")
    leg4 = leg_link("parallel_"+N+"2_leg2", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")

    return(Group(kknee_joint, kknee_link, leg_joint1, leg_joint2, leg1, leg2, leg_joint3, leg_joint4, leg3, leg4))


def twelve_act_float():
    # Base information
    base_rad = 0.07
    base_height = 0.01

    # Actuator information
    motor_limit = [0, 0.02]
    motor_axis = [0, 0, 1]
    motor_height = 0.04
    motor_wid = 0.01
    motor_geometry = Box(size=[motor_wid, motor_wid, motor_height])
    motor_material = "BananaYellow"

    # Delta Info
    foot_rad = 0.04
    delta_rad = 0.02
    a = foot_rad + delta_rad * 0.5
    b = delta_rad * np.sqrt(3) / 2
    c = foot_rad - delta_rad
    motor_link_origin = [0, 0, motor_height / 2, 0, 0, 0]

    delta = Group(
        # base
        base_joint([0, 0, 0, 0, 0, 0]),  # yes world link - floating
        base_link([0, 0, base_height / 2, 0, 0, 0], Cylinder(radius=base_rad, length=base_height), "Grey"),

        # act 0 - for delta 1 (+x of the center) - mid
        motor_joint(0, "prismatic", [c, 0, base_height, 0, 0, np.radians(90)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(0, motor_link_origin, motor_geometry, "Green"),

        # act 1 - for delta 1 (+x of the center) - cw left act
        motor_joint(1, "prismatic", [a, b, base_height, 0, 0, np.radians(-30)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(1, motor_link_origin, motor_geometry, "Green"),

        # act 2 - for delta 1 (+x of the center) - cw right act
        motor_joint(2, "prismatic", [a, -b, base_height, 0, 0, np.radians(-150)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(2, motor_link_origin, motor_geometry, "Green"),

        # act 3 - for delta 2 (-y of the center) - mid
        motor_joint(3, "prismatic", [0, -c, base_height, 0, 0, 0], limit=motor_limit,
                    axis=motor_axis),
        motor_link(3, motor_link_origin, motor_geometry, "Red"),

        # act 4 - for delta 2 (-y of the center) - cw left act
        motor_joint(4, "prismatic", [b, -a, base_height, 0, 0, np.radians(-120)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(4, motor_link_origin, motor_geometry, "Red"),

        # act 5 - for delta 2 (-y of the center) - cw right act
        motor_joint(5, "prismatic", [-b, -a, base_height, 0, 0, np.radians(120)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(5, motor_link_origin, motor_geometry, "Red"),

        # act 6 - for delta 3 (-x of the center) - mid
        motor_joint(6, "prismatic", [-c, 0, base_height, 0, 0, np.radians(-90)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(6, motor_link_origin, motor_geometry, "Orange"),

        # act 7 - for delta 3 (-x of the center) - cw left act
        motor_joint(7, "prismatic", [-a, -b, base_height, 0, 0, np.radians(150)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(7, motor_link_origin, motor_geometry, "Orange"),

        # act 8 - for delta 3 (-x of the center) - cw right act
        motor_joint(8, "prismatic", [-a, b, base_height, 0, 0, np.radians(30)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(8, motor_link_origin, motor_geometry, "Orange"),

        # act 9 - for delta 4 (+y of the center) - mid
        motor_joint(9, "prismatic", [0, c, base_height, 0, 0, np.radians(180)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(9, motor_link_origin, motor_geometry, "Blue"),

        # act 10 - for delta 4 (+y of the center) - cw left act
        motor_joint(10, "prismatic", [-b, a, base_height, 0, 0, np.radians(60)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(10, motor_link_origin, motor_geometry, "Blue"),

        # act 11 - for delta 4 (+y of the center) - cw right act
        motor_joint(11, "prismatic", [b, a, base_height, 0, 0, np.radians(-60)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(11, motor_link_origin, motor_geometry, "Blue"),

        # Legs for separate actuators
        # Delta 1
        parallel_leg(0, [0, 0, motor_height, 0, 0, 0], 'motor_link_0'),
        parallel_leg(1, [0, 0, motor_height, 0, 0, 0], 'motor_link_1'),
        parallel_leg(2, [0, 0, motor_height, 0, 0, 0], 'motor_link_2'),

        # Delta 2
        parallel_leg(3, [0, 0, motor_height, 0, 0, 0], 'motor_link_3'),
        parallel_leg(4, [0, 0, motor_height, 0, 0, 0], 'motor_link_4'),
        parallel_leg(5, [0, 0, motor_height, 0, 0, 0], 'motor_link_5'),

        # Delta 3
        parallel_leg(6, [0, 0, motor_height, 0, 0, 0], 'motor_link_6'),
        parallel_leg(7, [0, 0, motor_height, 0, 0, 0], 'motor_link_7'),
        parallel_leg(8, [0, 0, motor_height, 0, 0, 0], 'motor_link_8'),

        # Delta 4
        parallel_leg(9, [0, 0, motor_height, 0, 0, 0], 'motor_link_9'),
        parallel_leg(10, [0, 0, motor_height, 0, 0, 0], 'motor_link_10'),
        parallel_leg(11, [0, 0, motor_height, 0, 0, 0], 'motor_link_11'),
    )
    name = "12act_float"
    return delta, name


def twelve_act_nonfloat():
    # Base information
    base_rad = 0.07
    base_height = 0.01

    # Actuator information
    motor_limit = [0, 0.1]
    motor_axis = [0, 0, 1]
    motor_height = 0.04
    motor_wid = 0.01
    motor_geometry = Box(size=[motor_wid, motor_wid, motor_height])
    motor_material = "BananaYellow"

    # Delta Info
    foot_rad = 0.04
    delta_rad = 0.02
    a = foot_rad + delta_rad * 0.5
    b = delta_rad * np.sqrt(3) / 2
    c = foot_rad - delta_rad
    motor_link_origin = [0, 0, motor_height / 2, 0, 0, 0]

    delta = Group(
        # base
        base_link([0, 0, base_height / 2, 0, 0, 0], Cylinder(radius=base_rad, length=base_height), "Grey"),

        # act 0 - for delta 1 (+x of the center) - mid
        motor_joint(0, "prismatic", [c, 0, base_height, 0, 0, np.radians(90)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(0, motor_link_origin, motor_geometry, "Green"),

        # act 1 - for delta 1 (+x of the center) - cw left act
        motor_joint(1, "prismatic", [a, b, base_height, 0, 0, np.radians(-30)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(1, motor_link_origin, motor_geometry, "Green"),

        # act 2 - for delta 1 (+x of the center) - cw right act
        motor_joint(2, "prismatic", [a, -b, base_height, 0, 0, np.radians(-150)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(2, motor_link_origin, motor_geometry, "Green"),

        # act 3 - for delta 2 (-y of the center) - mid
        motor_joint(3, "prismatic", [0, -c, base_height, 0, 0, 0], limit=motor_limit,
                    axis=motor_axis),
        motor_link(3, motor_link_origin, motor_geometry, "Red"),

        # act 4 - for delta 2 (-y of the center) - cw left act
        motor_joint(4, "prismatic", [b, -a, base_height, 0, 0, np.radians(-120)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(4, motor_link_origin, motor_geometry, "Red"),

        # act 5 - for delta 2 (-y of the center) - cw right act
        motor_joint(5, "prismatic", [-b, -a, base_height, 0, 0, np.radians(120)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(5, motor_link_origin, motor_geometry, "Red"),

        # act 6 - for delta 3 (-x of the center) - mid
        motor_joint(6, "prismatic", [-c, 0, base_height, 0, 0, np.radians(-90)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(6, motor_link_origin, motor_geometry, "Orange"),

        # act 7 - for delta 3 (-x of the center) - cw left act
        motor_joint(7, "prismatic", [-a, -b, base_height, 0, 0, np.radians(150)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(7, motor_link_origin, motor_geometry, "Orange"),

        # act 8 - for delta 3 (-x of the center) - cw right act
        motor_joint(8, "prismatic", [-a, b, base_height, 0, 0, np.radians(30)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(8, motor_link_origin, motor_geometry, "Orange"),

        # act 9 - for delta 4 (+y of the center) - mid
        motor_joint(9, "prismatic", [0, c, base_height, 0, 0, np.radians(180)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(9, motor_link_origin, motor_geometry, "Blue"),

        # act 10 - for delta 4 (+y of the center) - cw left act
        motor_joint(10, "prismatic", [-b, a, base_height, 0, 0, np.radians(60)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(10, motor_link_origin, motor_geometry, "Blue"),

        # act 11 - for delta 4 (+y of the center) - cw right act
        motor_joint(11, "prismatic", [b, a, base_height, 0, 0, np.radians(-60)], limit=motor_limit,
                    axis=motor_axis),
        motor_link(11, motor_link_origin, motor_geometry, "Blue"),

        # Legs for separate actuators
        # Delta 1
        parallel_leg(0, [0, 0, motor_height, 0, 0, 0], 'motor_link_0'),
        parallel_leg(1, [0, 0, motor_height, 0, 0, 0], 'motor_link_1'),
        parallel_leg(2, [0, 0, motor_height, 0, 0, 0], 'motor_link_2'),

        # Delta 2
        parallel_leg(3, [0, 0, motor_height, 0, 0, 0], 'motor_link_3'),
        parallel_leg(4, [0, 0, motor_height, 0, 0, 0], 'motor_link_4'),
        parallel_leg(5, [0, 0, motor_height, 0, 0, 0], 'motor_link_5'),

        # Delta 3
        parallel_leg(6, [0, 0, motor_height, 0, 0, 0], 'motor_link_6'),
        parallel_leg(7, [0, 0, motor_height, 0, 0, 0], 'motor_link_7'),
        parallel_leg(8, [0, 0, motor_height, 0, 0, 0], 'motor_link_8'),

        # Delta 4
        parallel_leg(9, [0, 0, motor_height, 0, 0, 0], 'motor_link_9'),
        parallel_leg(10, [0, 0, motor_height, 0, 0, 0], 'motor_link_10'),
        parallel_leg(11, [0, 0, motor_height, 0, 0, 0], 'motor_link_11'),
    )
    name = "12act_nonfloat"
    return delta, name


if __name__ == "__main__":
    # RUN COMMAND
    # python ./urdf-gen/delta_urdf.py > ./urdfs/deltas/12act_float.urdf (yes world link)
    # Float -> yes world link (delta will stay where it is initalized at and float)

    # bot, name = twelve_act_float()
    # delta = Robot(
    #     delta_materials.materials,
    #     Link("world"),
    #     bot,
    #     name=name
    # )

    # RUN COMMAND
    # python ./urdf-gen/delta_urdf.py > ./urdfs/deltas/12act_nonfloat.urdf
    # Nonfloat -> no world link (delta will be subject to gravity)

    bot, name = twelve_act_nonfloat()
    delta = Robot(
        delta_materials.materials,
        bot,
        name=name
    )

    print(delta)
