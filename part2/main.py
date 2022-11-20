from __future__ import print_function

import os
import sys
import argparse
import numpy as np
from pddl_parser.PDDL import PDDL_Parser


sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['../padm-project-2022f', '../padm-project-2022f/ss-pybullet', '../padm-project-2022f/src']) #'../padm-project-2022f/pddlstream'

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def steer(start_pose, end_pose, world, tool_link, ik_joints, visualize=False):
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                return False
            if visualize:
                # Conf is a list with the position (float) of each joint
                set_joint_positions(world.robot, ik_joints, conf)
    return conf

def rand_position(start_pose):
    return multiply(start_pose, Pose(Point(z=1.0)))

def get_gripper_position(world):
    # ee = end effector
    world_from_base = get_link_pose(world.robot, link_from_name(world.robot, PANDA_INFO.base_link))
    world_from_ee = get_link_pose(world.robot, link_from_name(world.robot, PANDA_INFO.ee_link))

    # outputs the position AND the rotation as tuples with what is presumed to be xyz and quaternions q1-4
    # ( (position1, position2, position3), (q1, q2, q3, q4))
    return world_from_ee

def get_gripper_position_from_conf(conf):
    return False
    fk_fn = 0#UKNOWN FUNCTION! TRY TO FIND THE FK FUNCTION FOR THE ROBOT
    #mypos = compute_forward_kinematics(fk_fn, conf)
    # outputs the position AND the rotation as tuples with what is presumed to be xyz and quaternions q1-4
    # ( (position1, position2, position3), (q1, q2, q3, q4))
    return mypos



def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    wait_for_user()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    print("Going to use IK to go from a sample start state to a goal state\n")



    for i in range(2):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(world.robot, world.arm_joints, conf)
        wait_for_user()
        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
        start_pose = get_link_pose(world.robot, tool_link)
        end_pose = rand_position(start_pose)
        output_config = steer(start_pose, end_pose, world, tool_link, ik_joints, visualize=True)
        print(f"Position to point {get_gripper_position(world)}")
        if output_config:
            print(f"Movement {output_config} goes to point {get_gripper_position(world)} ok")

        else:
            print("Error! movement failed!")
            wait_for_user()

    
    print("Going to operate the base without collision checking")
    for i in range(100):
        goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        if (i % 30 == 0):
            wait_for_user()
    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
