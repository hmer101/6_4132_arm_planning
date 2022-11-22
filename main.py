from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import math
import time
from rrt import rrt, TreeNode
from pddl_parser.PDDL import PDDL_Parser

import utils

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])

import gitmodules
__import__('padm-project-2022f') 

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions, get_distance, get_angle, quat_combination, sub_inverse_kinematics, get_configuration

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

#def get_goal_sample_fn(body, joints, custom_limits={}, **kwargs):


def steer(start_pose, end_pose, world, tool_link, ik_joints, visualize=False):
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                return False
            if visualize:
                # Conf is a list with the position (float) of each joint
                set_joint_positions(world.robot, ik_joints, conf)
    return conf


# Take in only last and new config, use inverse kinematics to get close to new config
# def steer2(world, config_last, config_new):
#     #start_pose = #get_gripper_position_from_conf(config_last)
#     end_pose = get_gripper_position_from_conf(config_new)
#     tool_link = link_from_name(world.robot, 'panda_hand')

#     # Attempt to simulate movement from the last config to the new config
#     for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
#         conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
#         if conf is None:
#             return False
#         # Conf is a list with the position (float) of each joint
#         set_joint_positions(world.robot, ik_joints, conf)

#         # If no collisions, add this conf to 
    

    # for i in range(2):
    #     print('Iteration:', i)
    #     conf = sample_fn()
    #     set_joint_positions(world.robot, world.arm_joints, conf)
    #     wait_for_user()
    #     ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    #     start_pose = get_link_pose(world.robot, tool_link)
    #     end_pose = multiply(start_pose, Pose(Point(z=1.0)))
    #     for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
    #         conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
    #         if conf is None:
    #             print('Failure!')
    #             wait_for_user()
    #             break
    #         set_joint_positions(world.robot, ik_joints, conf)
    
    return conf


def rand_position(start_pose):

    return multiply(start_pose, Pose(Point(z=1.0)))


# def get_link_position(world, link_name):
#     # ee = end effector
#     link = link_from_name(world.robot, link_name)
#     link_pose = get_link_pose(world.robot, link)

#     return link_pose

# def get_gripper_position(world):
#     return get_link_position(world, 'panda_hand')

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
    print('Arm Joints num:', [joint for joint in world.arm_joints])
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    print('Kitchen joints', [get_joint_name(world.kitchen, joint) for joint in world.kitchen_joints])
    action_navigate(world)

    #print("Going to use IK to go from a sample start state to a goal state\n")


    start_pose = get_link_pose(world.robot, tool_link)
    end_pose = rand_position(start_pose)
    #conf = get_conf_from_gripper_pos(end_pose, start_pose, world)

    # Setup RRT
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    start = TreeNode(get_joint_positions(world.robot, world.arm_joints))
    sample = TreeNode(sample_fn())

    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    #set_joint_positions(world.robot, ik_joints, sample.config)
    goal_sample = utils.goal_sampling
    distance_fn = get_distance # Give distance of one config away from other  #dist_test = distance_fn(start.config,sample.config)
    extend_fn = (last.config, s)# Function to generate a new configuration based on a new sample and the closest configuration
    #collision_fn = # Function to figure out if the new configuration causes any collisions. Could incorporate into steer????

    # # Run RRT
    # rrt(start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
    #     goal_probability=.2, max_iterations=200000000, max_time=float('inf'))

    # param start: Start configuration - conf
    # :param distance_fn: Distance function - distance_fn(q1, q2)->float
    # :param sample_fn: Sample function - sample_fn()->conf
    # :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    # :param collision_fn: Collision function - collision_fn(q)->bool
    # :param max_iterations: Maximum number of iterations - int
    # :param max_time: Maximum runtime - float
    # :return: Path [q', ..., q"] or None if unable to find a solution



    # from src/utils: are_confs_close(conf1, conf2, tol=1e-8)

    #from pybullet_tools/utils
    # class ConfSaver(Saver):
    # def __init__(self, body): #, joints):
    #     self.body = body
    #     self.conf = get_configuration(body)

    # def apply_mapping(self, mapping):
    #     self.body = mapping.get(self.body, self.body)

    # def restore(self):
    #     set_configuration(self.body, self.conf)

    # def __repr__(self):
    #     return '{}({})'.format(self.__class__.__name__, self.body)


#     def get_joint_position(body, joint):
#         return get_joint_state(body, joint).jointPosition
    
#     def set_joint_position(body, joint, value):
#         p.resetJointState(body, joint, value, targetVelocity=0, physicsClientId=CLIENT)

#     def set_joint_positions(body, joints, values):
#     assert len(joints) == len(values)
#     for joint, value in zip(joints, values):
#         set_joint_position(body, joint, value)

#     def get_configuration(body):
#         return get_joint_positions(body, get_movable_joints(body))

#     def set_configuration(body, values):
#         set_joint_positions(body, get_movable_joints(body), values)

#     def get_full_configuration(body):
#         # Cannot alter fixed joints
#         return get_joint_positions(body, get_joints(body))

#     def clone_world(client=None, exclude=[]):
#     visual = has_gui(client)
#     mapping = {}
#     for body in get_bodies():
#         if body not in exclude:
#             new_body = clone_body(body, collision=True, visual=visual, client=client)
#             mapping[body] = new_body
#     return mapping


#     def get_collision_data(body, link=BASE_LINK):
#     # TODO: try catch
#     return [CollisionShapeData(*tup) for tup in p.getCollisionShapeData(body, link, physicsClientId=CLIENT)]


#     def get_closest_points(body1, body2, link1=None, link2=None, max_distance=MAX_DISTANCE):
#     assert (link1 is None) and (link2 is None)
#     return [CollisionInfo(*info) for info in p.getClosestPoints(
#         bodyA=body1, bodyB=body2, #linkIndexA=link1, linkIndexB=link2,
#         distance=max_distance, physicsClientId=CLIENT)]

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



# def sub_inverse_kinematics(robot, first_joint, target_link, target_pose, **kwargs):
#     solutions = plan_cartesian_motion(robot, first_joint, target_link, [target_pose], **kwargs)
#     if solutions:
#         return solutions[0]
#     return None


    # def plan_cartesian_motion(robot, first_joint, target_link, waypoint_poses,
    #                       max_iterations=200, custom_limits={}, **kwargs):
    # # TODO: fix stationary joints
    # # TODO: pass in set of movable joints and take least common ancestor
    # # TODO: update with most recent bullet updates
    # # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
    # # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py
    # # TODO: plan a path without needing to following intermediate waypoints

    # lower_limits, upper_limits = get_custom_limits(robot, get_movable_joints(robot), custom_limits)
    # selected_links = get_link_subtree(robot, first_joint) # TODO: child_link_from_joint?
    # selected_movable_joints = prune_fixed_joints(robot, selected_links)
    # assert(target_link in selected_links)
    # selected_target_link = selected_links.index(target_link)
    # sub_robot = clone_body(robot, links=selected_links, visual=False, collision=False) # TODO: joint limits
    # sub_movable_joints = get_movable_joints(sub_robot)
    # #null_space = get_null_space(robot, selected_movable_joints, custom_limits=custom_limits)
    # null_space = None

    # solutions = []
    # for target_pose in waypoint_poses:
    #     for iteration in range(max_iterations):
    #         sub_kinematic_conf = inverse_kinematics_helper(sub_robot, selected_target_link, target_pose, null_space=null_space)
    #         if sub_kinematic_conf is None:
    #             remove_body(sub_robot)
    #             return None
    #         set_joint_positions(sub_robot, sub_movable_joints, sub_kinematic_conf)
    #         if is_pose_close(get_link_pose(sub_robot, selected_target_link), target_pose, **kwargs):
    #             set_joint_positions(robot, selected_movable_joints, sub_kinematic_conf)
    #             kinematic_conf = get_configuration(robot)
    #             if not all_between(lower_limits, kinematic_conf, upper_limits):
    #                 #movable_joints = get_movable_joints(robot)
    #                 #print([(get_joint_name(robot, j), l, v, u) for j, l, v, u in
    #                 #       zip(movable_joints, lower_limits, kinematic_conf, upper_limits) if not (l <= v <= u)])
    #                 #print("Limits violated")
    #                 #wait_for_user()
    #                 remove_body(sub_robot)
    #                 return None
    #             #print("IK iterations:", iteration)
    #             solutions.append(kinematic_conf)
    #             break
    #     else:
    #         remove_body(sub_robot)
    #         return None
    # remove_body(sub_robot)
    # return solutions
    

if __name__ == '__main__':
    main()
