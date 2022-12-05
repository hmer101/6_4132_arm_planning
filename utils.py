import numpy as np
import math
import os
import sys

# UTILS.py

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])
sys.path.extend('pybullet')
import gitmodules
__import__('padm-project-2022f') 

from pybullet_tools.utils import  link_from_name, multiply, Pose, Point, interpolate_poses, set_joint_positions, set_joint_position
from pybullet_tools.utils import get_link_pose, get_joint_position, get_joint_positions, get_distance, get_angle, clone_body, get_body_info, get_pose, set_pose
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics
from pybullet_tools.transformations import quaternion_from_euler, euler_from_quaternion
from src.world import World
from src.utils import compute_surface_aabb, open_surface_joints, surface_from_name, joint_from_name, Surface
import rrt
import time
KITCHEN_BODY = 0
D_LENGTH = 0.5


## HELPER FUNCTIONS
# converted this function from a generator to returning a list
def interpolate_configs(start_config, end_config, config_step_size=0.01):
    num_steps = int(math.ceil(get_distance(start_config, end_config)/config_step_size))
    configs = [ (1-(float(i)/num_steps))*np.array(start_config) + (float(i)/num_steps)*np.array(end_config) for i in range(num_steps) ]
    return configs


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


## FOR FINDING RRT GOAL POSES
def get_drawer_center_position(world):
    closed = get_handle_position(world, is_open=False)
    open = get_handle_position(world, is_open=True)
    xyz = []
    for i in range(3):
        xyz.append((open[0][i] + closed[0][i])/2)
    xyz[2] += 0.2
    return (xyz, open[1])

# Get the position of the indigo drawer handle in the world
def get_surface_position(world, surface_name):
    # Get drawer centre pose
    surface_link = link_from_name(KITCHEN_BODY,surface_name)
    surface_pose = get_link_pose(KITCHEN_BODY, surface_link)

    # Add drawer dimensions for handle pose
    surface_surface =  compute_surface_aabb(world, surface_name)
    # -math.pi, 0 , 0 = facing down
    # 0, -math.pi, 0 = facing down
    handle_euler = [math.pi,math.pi/2,0]
    handle_q = quaternion_from_euler(handle_euler[0], handle_euler[1], handle_euler[2]) #list(drawer_pose[1])
    handle_pose = (list(surface_pose[0]), handle_q) #Note not a deep copy as drawer pose thrown away
    handle_pose[0][0] = float(surface_pose.upper[0]) + 0.1 # back for not colliding with the object on the counter/burner
    handle_pose[0][2] = handle_pose[0][2] + 1.0 # up for countertop and burner
    handle_pose = (tuple(handle_pose[0]), tuple(handle_pose[1]))
    #handle_pose[0][]

    return handle_pose

def pose_error(p1,p2, to_print=True):
    e = []
    e[0] = []
    e[1] = []
    tots = []
    for i in range(len(p1)):
        for j in range(len(p2)):
            e[i][j] = p1[i][j] - p2[i][j]
        tots.append(sum(e[i])**0.5)
    if to_print:
        print(f'Errors={e}, totals={tots}')
    return tots


# Get the position of the indigo drawer handle in the world
def get_handle_position(world, is_open):
    # Get drawer centre pose
    drawer_link = link_from_name(KITCHEN_BODY,'indigo_drawer_top')
    drawer_pose = get_link_pose(KITCHEN_BODY, drawer_link)

    # Add drawer dimensions for handle pose
    drawer_surface =  compute_surface_aabb(world, 'indigo_drawer_top')
    handle_euler = [math.pi,math.pi/2,0]
    handle_q = quaternion_from_euler(handle_euler[0], handle_euler[1], handle_euler[2]) 
    handle_pose = (list(drawer_pose[0]), handle_q) #Note not a deep copy as drawer pose thrown away
    handle_pose[0][0] = drawer_surface.upper[0] + 0.2 # add to this value ot move 
    handle_pose[0][2] = handle_pose[0][2] - 0.0 
    #if is_open:
    #    handle_pose[0][0] += D_LENGTH #((0.4287344217300415, 1.0858064889907837, -0.6024341583251953), (0.7225275039672852, 0.36401623487472534, -0.5689088106155396, -0.14761091768741608))
    
    #handle_pose[1] = [0,0,0,1] 
    handle_pose = (tuple(handle_pose[0]), tuple(handle_pose[1]))

    return handle_pose


# Change the orientation in a pose to a new pose
def pose_change_orient(orig_pose, new_orient):
    new_pose = (list(orig_pose[0]), new_orient)

    return new_pose

# Get the pose of an object in the world, modifying the orientation for the gripper
def get_pose_obj_goal(world, object_name):
    obj_body = world.body_from_name[object_name]
    body_pose = get_pose(obj_body)

    # Replace gripper orientation with custom orientation determined by object
    gripper_orient = [0,0,1,0] # Edit this for custom end gripper orientation

    if object_name == 'potted_meat_can1':
        #gripper_orient = [0,0,0,1]
        gripper_euler = [math.pi,math.pi/2,0]
        gripper_orient = quaternion_from_euler(gripper_euler[0], gripper_euler[1], gripper_euler[2]) 
    elif object_name == 'sugar_box0':
        gripper_orient = [0,0,1,0]
    
    gripper_pose = pose_change_orient(body_pose, gripper_orient)

    return gripper_pose

# Get configuration from an end gripper pose
def get_goal_config(world, start_config, end_pose, goal_radius=0.01, pose_step_size = 0.025, visualize=False, ik_time=0.1):
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)

    # Get original configuration to allow resetting
    #print(f"Start Config={start_config}")
    start_pose = tool_pose_from_config(world.robot, start_config)
    # set the joints to the starting config
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=pose_step_size):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=ik_time), None)
            if rrt.goal_test_pos(pose[0], end_pose[0], radius=goal_radius):
                return conf
            if visualize:
                set_joint_positions(world.robot, ik_joints, conf)
    print("ERROR! NO GOAL CONFIG FOUND!!!")
    return None


# Open the drawer
def open_the_drawer(world, surface):
    
    # TODO Check that it is at the start pose before moving
    handle_pose_closed = get_handle_position(world, is_open=False)

    ee_start_config = get_joint_positions(world.robot, world.arm_joints)


    
    tool_link = link_from_name(world.robot, 'panda_hand')
    ee_start_pose = get_link_pose(world.robot, tool_link)

    ee_end_pose = ((ee_start_pose[0][0]+D_LENGTH, ee_start_pose[0][1], ee_start_pose[0][2]),ee_start_pose[1])
    goal_pose = ee_end_pose#get_handle_position(world, is_open=True)
    print(f"DRAWER_OPEN_POSE={goal_pose}")
    # move to the start config
    
    goal_conf = get_goal_config(world, ee_start_config, goal_pose, goal_radius=0.01, ik_time=0.1)

    surface_name = 'indigo_drawer_top'
    surface = surface_from_name(surface_name)
    
    item_in_hand = surface #world.body_from_name['potted_meat_can1']
    end_conf = move(world, [goal_conf], item_in_hand, sleep_time=0.005)

    item_in_hand = None
    return end_conf

def pose_offset(pose, x, y, z):
    return ((pose[0][0]+x,pose[0][1]+y,pose[0][2]+z), pose[1])

# Move the arm in the world by interpolating between end_confs
def move(world, end_confs, item_in_hand=None, sleep_time=0.005):
    tool_link = link_from_name(world.robot, 'panda_hand')
    start_conf = get_joint_positions(world.robot, world.arm_joints)
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)

    tool_init_pose = get_link_pose(world.robot, tool_link)
    tool_pose_current = None
    for confs in end_confs:
        for conf in interpolate_configs(start_conf, confs):
            time.sleep(sleep_time)

            # Set position of robot arm
            set_joint_positions(world.robot, ik_joints, conf)

            # Get current pose of robot hand
            tool_pose_current = get_link_pose(world.robot, tool_link)

            if type(item_in_hand) == Surface:
                drawer_joint = joint_from_name(world.kitchen,item_in_hand.joints[0])
                set_joint_position(int(KITCHEN_BODY), drawer_joint, tool_pose_current[0][0] - tool_init_pose[0][0])

            # Set position of other object in robot hand
            elif not item_in_hand == None:
                set_pose(item_in_hand, tool_pose_current)

    return get_joint_positions(world.robot, world.arm_joints)


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
    #offset = [0.4, -0.2]
    offset = [0.3, -0.1]
    goal_position[0] = float(drawer_surface.upper[0]) + offset[0] #approximate robot width = 0.2                 
    goal_position[1] = float(drawer_surface.lower[1]) + offset[1]

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
