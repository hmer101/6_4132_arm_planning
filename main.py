from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import math
import time
from pddl_parser.PDDL import PDDL_Parser

import utils

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])

import gitmodules
__import__('padm-project-2022f') 

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions, get_distance, get_angle

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, get_body_name

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


# def find_base_goal(world):
#     # Get indigo_drawer_top location - front left point
#     print("TEST")
#     #print(world.get_body('sugar_box0'))
#     #print(world.get_body('sugar_box0'))
#     #print(world.all_bodies)
    
#     #print("DRAWER ABB: ")
#     drawer_surface =  compute_surface_aabb(world, 'indigo_drawer_top')
#     #stove_surface = compute_surface_aabb(world, 'front_right_stove')
#     #print(stove_surface)

#     #drawer_body = 0 #world.get_body('kitchen_part_right')
#     #drawer_link = link_from_name(KITCHEN_BODY,'indigo_drawer_top')
#     #drawer_pose = get_link_pose(KITCHEN_BODY, drawer_link)

#     #print(drawer_pose)


#     print((drawer_surface.lower[0],drawer_surface.upper[1], (drawer_surface.lower[2]+drawer_surface.upper[2])/2)) # Also has all drawer verticies so if have location of drawer, can get absolute position
#     # Set goal pose to be to the left of the drawer by the base's width to prevent collision when opening the drawer

#     return ()


# ACTION FUNCTIONS
def action_navigate(world):
    base_goal_pos = utils.get_base_goal_position(world)
    goal_ang_turn_by = utils.get_base_goal_ang(world, base_goal_pos)
    ang_start = get_joint_positions(world.robot, world.base_joints)[2]

    # Turn to goal angle
    new_ang = get_joint_positions(world.robot, world.base_joints)[2]
    while abs(new_ang-ang_start) < abs(goal_ang_turn_by):
        base_goal_pose = utils.rotate_base(world, np.sign(float(goal_ang_turn_by))*0.05)
        set_joint_positions(world.robot, world.base_joints, base_goal_pose)
        new_ang = get_joint_positions(world.robot, world.base_joints)[2]
        time.sleep(0.1)

    # Move to goal position
    while not utils.base_at_goal(world, base_goal_pos):
        goal_pos_lin = translate_linearly(world, 0.01)
        set_joint_positions(world.robot, world.base_joints, goal_pos_lin)
        time.sleep(0.01)


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
    
    print('Kitchen joints', [get_joint_name(world.kitchen, joint) for joint in world.kitchen_joints])

    action_navigate(world)
    wait_for_user()
    #print(base_goal_pos)
    #find_base_goal(world)
    
    # print("Going to use IK to go from a sample start state to a goal state\n")
    # for i in range(2):
    #     print('Iteration:', i)
    #     conf = sample_fn()
    #     set_joint_positions(world.robot, world.arm_joints, conf)
    #     wait_for_user()
    #     ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    #     start_pose = get_link_pose(world.robot, tool_link)
    #     end_pose = rand_position(start_pose)
    #     output_config = steer(start_pose, end_pose, world, tool_link, ik_joints, visualize=True)
    #     if conf is not None:
    #         print(f"Movement {conf} is ok")
    #     else:
    #         print("Error! movement failed!")
    #         wait_for_user()

if __name__ == '__main__':
    main()
