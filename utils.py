import numpy as np
import math
import os
import sys

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])
sys.path.extend('pybullet')
import gitmodules
__import__('padm-project-2022f') 

from pybullet_tools.utils import  link_from_name, multiply, Pose, Point, interpolate_poses, set_joint_positions
from pybullet_tools.utils import get_link_pose, get_joint_positions, get_distance, get_angle
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import compute_surface_aabb

KITCHEN_BODY = 0


# Trys to move the arm to the goal. 
# Returns the output conf if it can, otherwise it returns None
# AKA Goal sampling
def goal_sampling(world, end_pose, reset_at_end = True):
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_pose = get_link_pose(world.robot, tool_link)
    #end_pose = multiply(start_pose, Pose(Point(z=1.0)))
    start_conf = get_joint_positions(world.robot, tool_link)
    conf = None
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        if conf is None:
            return None
        set_joint_positions(world.robot, ik_joints, conf)
    if reset_at_end:
        set_joint_positions(world.robot, ik_joints, start_conf)
    return conf


'''def config_from_pose(world, pose):
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    out_config = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
    return out_config'''


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
