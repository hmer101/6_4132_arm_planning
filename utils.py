import numpy as np
import math
import os
import sys

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])
sys.path.extend('pybullet')
import gitmodules
__import__('padm-project-2022f') 

from pybullet_tools.utils import  link_from_name, multiply, Pose, Point, interpolate_poses, set_joint_positions
from pybullet_tools.utils import get_link_pose, get_joint_positions, get_distance, get_angle, clone_body
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import compute_surface_aabb

KITCHEN_BODY = 0


#### TO FIX/IMPLEMENT
# def steer(start_pose, end_pose, world, tool_link, ik_joints, visualize=False):
#     for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
#             conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
#             if conf is None:
#                 return False
#             if visualize:
#                 # Conf is a list with the position (float) of each joint
#                 set_joint_positions(world.robot, ik_joints, conf)
#     return conf

# TO FIX -> Don't use inverse kinematics???????
# def config_from_pose(robot_body, pose):
#     tool_link = link_from_name(robot_body, 'panda_hand')
#     ik_joints = get_ik_joints(robot_body, PANDA_INFO, tool_link)
#     out_config = next(closest_inverse_kinematics(robot_body, PANDA_INFO, tool_link, pose, max_time=0.05), None)

#     return out_config

# NOT YET WORKING!!!!
def config_from_tool_pose(robot_body, arm_joints, pose):
    # Clone body to find config from pose
    #test_body = clone_body(robot_body, links=arm_joints, collision=True, visual=False, client=None)
    
    tool_link = link_from_name(robot_body, 'panda_hand')
    ik_joints = get_ik_joints(robot_body, PANDA_INFO, tool_link)

    out_config = False

    return out_config


def interpolate_configs(start_config, end_config, config_step_size=0.01):
    return




#### ALL BELOW ARE TESTED AND WORKING

# Find the pose of the tool from a given config 
def tool_pose_from_config(robot_body, config):
    tool_link = link_from_name(robot_body, 'panda_hand')
    ik_joints = get_ik_joints(robot_body, PANDA_INFO, tool_link)

    # Get original configuration to allow resetting
    conf_orig = get_joint_positions(robot_body, ik_joints)

    # Set joint to new position and get tool pose
    set_joint_positions(robot_body, ik_joints, config)
    tool_pose = get_link_pose(robot_body, tool_link)

    # Reset to original config
    set_joint_positions(robot_body, ik_joints,conf_orig)

    return tool_pose


# Get the position of the indigo drawer handle in the world
def get_handle_position(world):
    # Get drawer centre pose
    drawer_link = link_from_name(KITCHEN_BODY,'indigo_drawer_top')
    drawer_pose = get_link_pose(KITCHEN_BODY, drawer_link)

    # Add drawer dimensions for handle pose
    drawer_surface =  compute_surface_aabb(world, 'indigo_drawer_top')
    handle_pose = (list(drawer_pose[0]), list(drawer_pose[1])) #Note not a deep copy as drawer pose thrown away
    handle_pose[0][0] = float(drawer_surface.upper[0]) #handle_pose[0][0] +
    handle_pose = (tuple(handle_pose[0]), tuple(handle_pose[1]))

    return handle_pose[0]



## FOR BASE MOVEMENT

# Get the position to move the base to to allow it to perform all actions
# This position is just left of the drawer's left edge
def get_base_goal_position(world):
    # Get drawer centre pose
    drawer_link = link_from_name(KITCHEN_BODY,'indigo_drawer_top')
    drawer_pose = get_link_pose(KITCHEN_BODY, drawer_link)

    # Add drawer and robot dimensions for left edge
    drawer_surface =  compute_surface_aabb(world, 'indigo_drawer_top')
    goal_position = list(drawer_pose[0]) # Note not a deep copy as drawer pose thrown away
    goal_position[0] = float(drawer_surface.upper[0]) + 0.2 #approximate robot width = 0.2                 
    goal_position[1] = float(drawer_surface.lower[1]) - 0.2 

    return tuple(goal_position[0:2])


# Find angle to rotate to to allow linear translation to goal
def get_base_goal_ang(world, goal_pos):
    # Get current pose
    x, y, theta = get_joint_positions(world.robot, world.base_joints)

    # Find angle to turn to to get to goal position linearly
    goal_pos = list(goal_pos)
    goal_pos.append(0)
    ang_turn_by = get_angle(tuple(goal_pos),(x,y,0))
    #ang_turn_to = get_angle(tuple(goal_pos),(x,y,0)) + theta

    return ang_turn_by


# Check if the base has reached within a specified radius of the goal position
def base_at_goal(world, goal_position, radius=0.35):
    # Get current robot distance from goal
    x, y, theta = get_joint_positions(world.robot, world.base_joints)
    dist_to_goal = get_distance((x,y),goal_position)

    # Return true if the robot is within the specified radius around the goal and false otherwise
    if abs(dist_to_goal) < radius:
        return True
    else:
        return False

# Find pose when rotated by a certain angle
def rotate_base(world, angle):
    x, y, theta = get_joint_positions(world.robot, world.base_joints)
    pos = np.array([x, y])
    goal_ang = (theta + angle)%(2*math.pi)  # Normalize goal angle
    goal_pose = np.append(pos, [goal_ang])
    return goal_pose
