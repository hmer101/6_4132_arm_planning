# MAIN.py


from __future__ import print_function
import execute_plan as exec
import os
import sys
# import argparse
# import numpy as np
# import math
# import time
# import rrt
from pddl_parser.PDDL import PDDL_Parser

#import utils
import execute_plan as ex_plan

from pathlib import Path

#sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])
sys.path.insert(0,os.path.abspath(os.path.join(os.getcwd(), 'part1')))
from ff_planner import FF_Planner
from bfs_planner import BFS_Planner

# import gitmodules
# __import__('padm-project-2022f')

from pybullet_tools.utils import  wait_for_user, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed #set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
# from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions, get_distance, get_angle, quat_combination, sub_inverse_kinematics, get_configuration, single_collision, link_pairs_collision, get_links

# from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
# from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics
# from pybullet_tools.transformations import quaternion_from_euler, euler_from_quaternion

# from src.world import World
# from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
#     ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
#     BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
#     STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, get_body_name, set_tool_pose, surface_from_name, open_surface_joints



def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())
    exec.world._update_initial()
    exec.action_navigate(exec.world)

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
        exec.perform_action(act.name, act.parameters)
    print('Action plan completed. Press enter to exit.')
    wait_for_user()

if __name__ == '__main__':
    main()
