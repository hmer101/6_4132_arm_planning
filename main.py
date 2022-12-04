# MAIN.py

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

from pathlib import Path

#sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])
sys.path.insert(0,os.path.abspath(os.path.join(os.getcwd(), 'part1')))
from ff_planner import FF_Planner
from bfs_planner import BFS_Planner

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
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, get_body_name, set_tool_pose, surface_from_name, open_surface_joints



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
    print(sys.path)

    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    obj_pos_meat = utils.get_pose_obj_goal(world, 'potted_meat_can1')
    obj_pos_sugar = utils.get_pose_obj_goal(world, 'sugar_box0')
    global pos_counter
    pos_counter = tuple(obj_pos_meat)
    global pos_burner
    pos_burner = tuple(obj_pos_sugar)
    franka_name = 'franka'
    indigo_drawer_handle_name = 'indigo_drawer_handle'
    indigo_drawer_center_name = 'indigo_drawer'
    burner_name = 'burner'
    countertop_name = 'countertop'
    KITCHEN_BODY = 0
    item_in_hand = dict()
    is_open = dict()
    is_open[indigo_drawer_center_name] = False
    item_in_hand[franka_name] = None

    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    ex_plan.action_navigate(world)

    #set_joint_positions(world.robot, world.arm_joints, conf)
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_pose = get_link_pose(world.robot, tool_link)
    

    # Set up RRT
    handle_pose_closed = utils.get_handle_position(world, is_open=False)

    pos_indigo_drawer_center = utils.get_drawer_center_position(world)
    start_config = get_joint_positions(world.robot, world.arm_joints)
    global current_conf
    current_conf = start_config
    
    def get_robot(robot_name):
        if robot_name == 'franka':
            return world.robot

    def planner_get_pose(end_location_name):
        if end_location_name == burner_name:
            return pos_burner
            #return utils.get_surface_position(world, 'front_right')
        if end_location_name == indigo_drawer_handle_name:
            areuopen = is_open[indigo_drawer_center_name]
            return utils.get_handle_position(world, areuopen)
            #return get_drawer_pose(end_location_name) #TODO CHECK THIS FUNCTION
        if end_location_name == indigo_drawer_center_name:
            return pos_indigo_drawer_center
        if end_location_name == countertop_name:
            return pos_counter
            #return utils.get_surface_position(world, 'indigo_tmp')

    def get_drawer_link(drawer_name):
        return link_from_name(KITCHEN_BODY,drawer_name)

    def get_drawer_pose(drawer_name):
        return get_link_pose(KITCHEN_BODY, get_drawer_link(drawer_name))
    
    def get_surface(surface_name):
        return surface_from_name(indigo_drawer_center_name)

    def navigate(robot_name, start_location, end_location):
        robot = get_robot(robot_name)
        end_pose = planner_get_pose(end_location)
        if end_pose == None:
            print(f"Error! No end_pose found for {end_location}")
            wait_for_user()
        start_pose = planner_get_pose(start_location)
        global current_conf
        # TODO Check that the start_pose and current robot pose are close
        current_pose = utils.tool_pose_from_config(world.robot, current_conf)

        
        c_sq = 0
        for i in range(len(current_pose[0])):
            c_sq += (current_pose[0][i] - start_pose[0][i])**2
        if c_sq >= 0.21:
            print (f"ERROR! ROBOT NOT AT START POSE!\nPose={current_pose}\nStart={start_pose}")
            wait_for_user()
        else:
            print ("Navigate ok. Robot near start pose...")

        end_config = utils.get_goal_config(world, current_conf, end_pose, ik_time=0.1)

        
        if end_config == None:
            print ("ERROR! No end config found! Exiting program")
            wait_for_user()
        save = get_joint_positions(world.robot, world.arm_joints)
        print("End config found and shown.")
        set_joint_positions(world.robot, world.arm_joints, end_config)
        wait_for_user()
        set_joint_positions(world.robot, world.arm_joints, save)
        
        
        config_path = rrt.rrt_arm_wrapper(current_conf, end_config, world.robot, world.arm_joints)
        if config_path == None:
            print ("ERROR! No config_path found! Exiting program")
            wait_for_user()

        current_conf = utils.move(world, config_path, item_in_hand=item_in_hand[robot_name])

    def get_surface_name(drawer_name):
        if drawer_name == indigo_drawer_center_name:
            return 'indigo_drawer_top'

    def open_drawer(robot_name, drawer_name, drawer_handle_name):
        robot = get_robot(robot_name)
        #sname = get_surface_name(drawer_name)
        surface = get_surface('indigo_drawer_top')

        current_conf = utils.open_the_drawer(world,surface)
        
        is_open[drawer_name] = True

    def pick_up (robot_name, item_name):
        item_in_hand[robot_name] = item_name

    def place (robot_name, item_name, surface_name):
        item_in_hand[robot_name] = None

    # TODO
    def close_drawer (robot_name, drawer_name):
        robot = get_robot(robot_name)
        surface = get_surface(drawer_name)
        #utils.close_the_drawer(world, surface)
        is_open[drawer_name] = False

    def perform_action (name, params):
        a = name
        if a == 'open-drawer':
            open_drawer(params[0],params[1],params[2])
        if a == 'navigate':
            navigate(params[0],params[1],params[2])
        if a == 'pick-up':
            pick_up(params[0],params[1])
        if a == 'place':
            place(params[0],params[1],params[2])
        if a == 'close_drawer':
            close_drawer(params[0],params[1])

    # wait_for_user()
    
    dirname = os.path.abspath(os.path.join(os.getcwd(), 'part1'))
    domain = os.path.join(dirname,'domain.pddl') #dinner blocksworld.pddl domain.pddl
    problem = os.path.join(dirname,'problem.pddl') #pb1_dinner pb4_blocksworld.pddl problem.pddl
    ff_planner = FF_Planner(domain, problem)
    bfs_planner = BFS_Planner(frozenset(ff_planner.s_0), ff_planner.s_goal_pos, ff_planner.s_goal_neg, ff_planner.actions)
    plan = bfs_planner.solve()
    
    if type(plan) is list:
        print("Plan found:")
        for act in plan:
            print((act.name) + ' ' + ' '.join(act.parameters))
    else:
        print('No plan was found')
        exit(1)



    print('EXECUTING PLAN:')
    for act in plan:
        print('PERFORMING ACTION: ' + (act.name) + ' ' + ' '.join(act.parameters))
        perform_action(act.name, act.parameters)


    #conf_handle_closed = utils.get_goal_config(world, start_config, handle_pose_closed)
    
    # end_conf = utils.move(world, [conf_handle_closed], None, sleep_time=0.005)
    
    # print("Did first move")
    # wait_for_user()
    # utils.open_drawer(world)
    # print("Opened drawer")
    # wait_for_user()


    #utils.move(world, handle_pose_closed, item_in_hand=None, sleep_time=0.005)

    # Run RRT

    #########################
    # THIS IS GOOD BELOW:
    #########################
    #config_path = rrt.rrt_arm_wrapper(start_config, conf_meat_can, world.robot, world.arm_joints)
    #end_conf = utils.move(world, config_path)

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
