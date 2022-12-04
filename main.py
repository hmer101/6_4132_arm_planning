from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import math
import time
import rrt
from pddl_parser.PDDL import PDDL_Parser

import utils
import execute_plan as ex_plan

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])

import gitmodules
__import__('padm-project-2022f') 

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions, get_distance, get_angle, quat_combination, sub_inverse_kinematics, get_configuration, single_collision, link_pairs_collision, get_links

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics
from pybullet_tools.transformations import quaternion_from_euler, euler_from_quaternion

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, get_body_name, set_tool_pose, surface_from_name



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

def rand_position(start_pose):
    return multiply(start_pose, Pose(Point(z=1.0)))



# if __name__ == '__main__':
#     main()


def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    obj_pos_meat = utils.get_pose_obj_goal(world, 'potted_meat_can1') 
    obj_pos_sugar = utils.get_pose_obj_goal(world, 'sugar_box0')

    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    ex_plan.action_navigate(world)

    # Set up RRT
    handle_pose_closed = utils.get_handle_position(world, is_open=False)
    start_config = get_joint_positions(world.robot, world.arm_joints)
    conf_handle_closed = utils.get_goal_config(world, start_config, handle_pose_closed)

    # Run RRT
    config_path = rrt.rrt_arm_wrapper(start_config, conf_handle_closed, world.robot, world.arm_joints)
    end_conf = utils.move(world, config_path)

    wait_for_user()
    world.destroy()

    ## COLLISION DETECTION TESTING
    # Good for testing self-collision
    #set_joint_positions(world.robot, [world.arm_joints[1]], [3]) 

    # Test collisions with other objects
    #set_joint_positions(world.robot, [world.arm_joints[1]], [2.5])
    #set_joint_positions(world.robot, [world.arm_joints[3]], [0])
    # config_test = get_joint_positions(world.robot, world.arm_joints)
    # result = rrt.detect_collision(world.robot, config_test)


#     def clone_world(client=None, exclude=[]):
#     visual = has_gui(client)
#     mapping = {}
#     for body in get_bodies():
#         if body not in exclude:
#             new_body = clone_body(body, collision=True, visual=visual, client=client)
#             mapping[body] = new_body
#     return mapping


# def body_collision(body1, body2, max_distance=MAX_DISTANCE): # 10000
#     # TODO: confirm that this doesn't just check the base link
#     return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
#                                   physicsClientId=CLIENT)) != 0 # getContactPoints`

# def pairwise_collision(body1, body2, **kwargs):
#     if isinstance(body1, tuple) or isinstance(body2, tuple):
#         body1, links1 = expand_links(body1)
#         body2, links2 = expand_links(body2)
#         return any_link_pair_collision(body1, links1, body2, links2, **kwargs)
#     return body_collision(body1, body2, **kwargs)

# #def single_collision(body, max_distance=1e-3):
# #    return len(p.getClosestPoints(body, max_distance=max_distance)) != 0

# def single_collision(body1, **kwargs):
#     for body2 in get_bodies():
#         if (body1 != body2) and pairwise_collision(body1, body2, **kwargs):
#             return True
#     return False

# def link_pairs_collision(body1, links1, body2, links2=None, **kwargs):
#     if links2 is None:
#         links2 = get_all_links(body2)
#     for link1, link2 in product(links1, links2):
#         if (body1 == body2) and (link1 == link2):
#             continue
#         if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
#             return True
#     return False


if __name__ == '__main__':
    main()
