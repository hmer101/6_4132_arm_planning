from random import random
import time
import utils

from pybullet_tools.utils import set_joint_positions, interval_generator, get_custom_limits, CIRCULAR_LIMITS, get_distance, link_from_name, get_joint_positions, single_collision

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

#REMOVE later
import sys
import os
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet'])

import gitmodules
__import__('padm-project-2022f') 


class TreeNode(object):

    def __init__(self, config, parent=None):
        self.parent = parent
        self.config = config

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]


def configs(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.config, nodes))


def irange(start, stop=None, step=1):
    if stop is None:
        stop = start
        start = 0
    while start < stop:
        yield start
        start += step

def argmin(function, sequence):
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))]

def elapsed_time(start_time):
    return time.time() - start_time



## RRT FUNCTIONS FOR ARM

# Trys to move the arm to the goal. 
# Returns the output conf if it can, otherwise it returns None
# AKA Goal sampling
# FIX SO NOT USING INVERSE KINEMATICS
# def goal_sampling(world, start_conf, goal_pose, reset_at_end = True):
#     tool_link = link_from_name(world.robot, 'panda_hand')
#     ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
#     set_joint_positions(world.robot, ik_joints, start_conf)
#     #end_pose = multiply(start_pose, Pose(Point(z=1.0)))
#     start_conf = get_joint_positions(world.robot, tool_link)
#     conf = None
#     for pose in interpolate_poses(start_pose, goal_pose, pos_step_size=0.01):
#         conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
#         if conf is None:
#             return None
#         set_joint_positions(world.robot, ik_joints, conf)
#     if reset_at_end:
#         set_joint_positions(world.robot, ik_joints, start_conf)
#     return conf

def goal_sampling(world, start_conf, goal_pose, max_time=0.2):
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    set_joint_positions(world.robot, ik_joints, start_conf)
    conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, goal_pose, max_time=max_time), None)
    set_joint_positions(world.robot, ik_joints, start_conf)
    return conf
    

# Returns a function for sampling the arm config space
def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

# Tests if the current position is within a certain radius of the desired position
def goal_test_pos(current_pos, goal_pos, radius=0.5):
    if get_distance(current_pos, goal_pos) <= radius:
        return True
    else:
        return False

# Extend function to extend the last configuration to the new configuration
def extend(config_last, config_new):
    return utils.interpolate_configs(config_last, config_new)
    #return [config_new]


# Function to allow collision detection between robot arm and the environment
def detect_collision(robot_body, config):
    collision = False
    
    tool_link = link_from_name(robot_body, 'panda_hand')
    ik_joints = get_ik_joints(robot_body, PANDA_INFO, tool_link)

    # Get original configuration to allow resetting
    conf_orig = get_joint_positions(robot_body, ik_joints)

    # Temporarily set to test configuration to test collisions
    set_joint_positions(robot_body, ik_joints, config)

    # Check collisions
    if single_collision(robot_body) == True:    # Collision with any bodies in world
        collision = True
    
    # SELF COLLISIONS
    #elif link_pairs_collision(robot_body, get_links(robot_body), robot_body, get_links(robot_body)): #pairwise_collision(robot_body, robot_body) == True: # Collision with self
     #   collision = True
        
    # Reset to original config
    set_joint_positions(robot_body, ik_joints,conf_orig)

    return collision 


## RRT


# Performs RRT from start to a goal pos
# Note that this returns the configuration sequence to traverse from the start to the goal but does not visualise this 
# (need to do something like set_joint_positions(interpolate_configs(configs)) on return)

# REMOVE: world, obstacles, goal_pose or goal_sample?
def rrt(world, robot_body, obstacles, start, goal_pose, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
        goal_probability=.2, max_iterations=20, max_time=float('inf'), visualize=False):
    """
    :param start: Start configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    start_time = time.time()
    if collision_fn(robot_body, start.config):
        return None
    # if not callable(goal_sample):
    #     g = goal_sample
    #     goal_sample = lambda: g
    nodes = [(start)]
    for i in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        goal = random() < goal_probability or i == 0

        # Sample. Try sampling from goal randomly - return a normal sample if unable to 
        #s = goal_sample() if goal else sample_fn()
        s = sample_fn()
        if goal:
            # Pick a random node: NOTE: TRY TO ONLY PICK CLOSEST NODES THAT HAVEN'T BEEN PICKED YET
            random_node = nodes[int(random()*len(nodes))]
            # Add dynamic goal sampling time (ex: number of times a node was selected increases goal sampling time)
            goal_sampling_time = 0.2

            test_conf = goal_sampling(world, random_node.config, goal_pose, max_time=goal_sampling_time)
            if test_conf == None:
                continue
            s = test_conf


        #         # 

        # Find closest config in tree to new sample config and extend from closest config to 
        last = argmin(lambda n: distance_fn(n.config, s), nodes)
        configs = extend_fn(last.config, s)

        valid = True
        for q in configs:
            if collision_fn(robot_body, q):
                valid = False
                break

            if visualize:
            # Temporarily visualise RRT functioning
                set_joint_positions(robot_body, world.arm_joints, last.config)

            if goal_test(utils.tool_pose_from_config(robot_body, last.config)[0],goal_pose[0]):
                return configs(last.retrace())
        
        
        if valid:
            last = TreeNode(configs[len(configs)-1], parent=last)
            nodes.append(last)


        # else:
        #     if goal:
        #         return configs(last.retrace())
    return None