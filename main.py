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



#import os
# import sys
# import argparse
# import numpy as np

# sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

# from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
# from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

# from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
# from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

# from src.world import World
# from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
#     ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
#     BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
#     STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly


def min_example():
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
    sample_fn = rrt.get_sample_fn(world.robot, world.arm_joints)
    print("Going to use IK to go from a sample start state to a goal state\n")
    
    # start_pose = get_link_pose(world.robot, tool_link)
    # end_pose = multiply(start_pose, Pose(Point(z=1.0)))
    

    # joint_poses_init = get_joint_positions(world.robot, world.arm_joints)
    # print(joint_poses_init)

    # set_tool_pose(world, end_pose)
    # joint_poses_final = get_joint_positions(world.robot, world.arm_joints)
    # print(joint_poses_final)
    # #ipdb - interactive debugging

    # test = 1
    
    for i in range(2):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(world.robot, world.arm_joints, conf)
        wait_for_user()
        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
        start_pose = get_link_pose(world.robot, tool_link)
        end_pose = multiply(start_pose, Pose(Point(z=1.0)))
        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                print('Failure!')
                wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)
    print("Going to operate the base without collision checking")
    for i in range(100):
        goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        if (i % 30 == 0):
            wait_for_user()
    wait_for_user()
    world.destroy()

# if __name__ == '__main__':
#     main()





def main():
    #min_example()

    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    obj_pos_meat = utils.get_pose_obj_goal(world, 'potted_meat_can1') 
    obj_pos_sugar = utils.get_pose_obj_goal(world, 'sugar_box0')

    # wait_for_user()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    action_navigate(world)

    #set_joint_positions(world.robot, world.arm_joints, conf)
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_pose = get_link_pose(world.robot, tool_link)


    handle_pose_closed = utils.get_handle_position(world, is_open=False)
    start_config = get_joint_positions(world.robot, world.arm_joints)
    conf_handle_closed = utils.get_goal_config(world, start_config, handle_pose_closed)

    end_conf = utils.move(world, [conf_handle_closed], sleep_time=0.005)

    print("Did first move")
    wait_for_user()
    utils.open_drawer(world)

    # surface_name = 'indigo_drawer_top'
    # surface = surface_from_name(surface_name)
    # print(f"SURFACE TYPE: {type(surface)}")
    

    # # Test "in hand"
    # item_in_hand = surface #world.body_from_name['potted_meat_can1']
    # utils.move(world, [conf_goal], item_in_hand, sleep_time=0.005)

    # # Visualise moving to goal
    # print(f"\n\nFound goal config! = {conf_goal}")
    # for conf in utils.interpolate_configs(start_config, conf_goal):
    #     set_joint_positions(world.robot, ik_joints, conf)
    #     set_pose(item_in_hand, get_link_pose(world.robot, tool_link))
    #     time.sleep(0.005)
    # print("At goal config")
    # wait_for_user()




    '''
    print("Did first move")
    wait_for_user()
    utils.open_drawer(world)
    print("Did 2nd move")

    wait_for_user()


    

    has_broken = False
    for i in range(100):
        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if rrt.goal_test_pos(pose[0], end_pose[0], radius=radius):
                radius-=radius_step
                print(f"Sucess! Trying new radius={radius}")
                
                start_pose = get_link_pose(world.robot, tool_link)
                joint_poses_initial = get_joint_positions(world.robot, world.arm_joints)
                set_joint_positions(world.robot, ik_joints, joint_poses_initial)
                break
                

            if conf is None:
                print(f'Failure at radius={radius}')
                has_broken = True
                set_joint_positions(world.robot, ik_joints, joint_poses_initial)
                #wait_for_user()
                break
            
            set_joint_positions(world.robot, ik_joints, conf)
        if has_broken:
            has_broken=False
            continue
        print(f"success at {i}")
        wait_for_user()


    joint_poses_final = get_joint_positions(world.robot, world.arm_joints)
    print(joint_poses_final)

    wait_for_user()'''


    ## COLLISION DETECTION TESTING
    # Good for testing self-collision
    #set_joint_positions(world.robot, [world.arm_joints[1]], [3]) 

    # Test collisions with other objects
    #set_joint_positions(world.robot, [world.arm_joints[1]], [2.5])
    #set_joint_positions(world.robot, [world.arm_joints[3]], [0])
    # config_test = get_joint_positions(world.robot, world.arm_joints)
    # result = rrt.detect_collision(world.robot, config_test)


    # # Setup RRT
    # sample_fn = rrt.get_sample_fn(world.robot, world.arm_joints)
    # start = rrt.TreeNode(get_joint_positions(world.robot, world.arm_joints))
    # goal_sample = None   #utils.goal_sampling # TO FIX!!!
    # distance_fn = get_distance # Give distance of one config away from other  #dist_test = distance_fn(start.config,sample.config)
    # extend_fn = rrt.extend # Function to generate a new configuration based on a new sample and the closest configuration
    # goal_test_fn = rrt.goal_test_pos
    # collision_fn = rrt.detect_collision #lambda conf, obstacles: False # Function to figure out if the new configuration causes any collisions. Could incorporate into steer????

    # goal_pose = rand_position(get_link_pose(world.robot, tool_link))
    # obstacles = None

    # # Run RRT
    # viable_config_path = rrt.rrt(world, world.robot, obstacles, start, goal_pose, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=goal_test_fn,
    #     goal_probability=.2, max_iterations=200000000, max_time=float('inf'))


    # Follow config path from RRT, interpolating to give a "nice" animation















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
