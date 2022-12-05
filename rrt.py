# RRT.py

from random import random
import time
import utils

from pybullet_tools.utils import set_joint_positions, interval_generator, get_custom_limits, CIRCULAR_LIMITS, get_distance, link_from_name, get_joint_positions, single_collision, link_pairs_collision, get_links, get_all_links 
from src.utils import surface_from_name

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

import gitmodules
__import__('padm-project-2022f') 
KITCHEN_BODY = 0

class TreeNode(object):

    def __init__(self, config, parent=None):
        self.parent = parent
        self.config = config

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node.config)
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

# Returns a function for sampling the arm config space
def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

# Tests if the current position is within a certain radius of the desired position
def goal_test_pos(current_pos, goal_pos, radius):
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

    # Check collisions with kitchen
    if single_collision(robot_body) == True:    # Collision with any bodies in world
        collision = True
    # elif link_pairs_collision(robot_body, get_links(robot_body), int(KITCHEN_BODY)):
    #     collision = True


    # SELF COLLISIONS
    #elif link_pairs_collision(robot_body, get_links(robot_body), robot_body, get_links(robot_body)): #pairwise_collision(robot_body, robot_body) == True: # Collision with self
     #   collision = True
        
    # Reset to original config
    set_joint_positions(robot_body, ik_joints,conf_orig)

    return collision 


## RRT

# Wrapper to perform RRT on the arm from a start to an end config
def rrt_arm_wrapper(start_config, end_config, robot_body, arm_joints):
    start = start_config
    goal_sample = end_config
    distance_fn = get_distance
    sample_fn = get_sample_fn(robot_body, arm_joints)
    extend_fn = extend #utils.interpolate_configs
    collision_fn = detect_collision
    goal_test = goal_test_pos


    config_path = rrt(robot_body, start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test,
        goal_probability=.2)

    return config_path


# Performs RRT from start to a goal pos
# Note that this returns the configuration sequence to traverse from the start to the goal but does not visualize this 
def rrt(robot_body, start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
        goal_probability=.5, max_iterations=9*10**9, max_time=float('inf')):
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

    if collision_fn(robot_body, start):
        return None
    if not callable(goal_sample):
        g = goal_sample
        goal_sample = lambda: g
    nodes = [TreeNode(start)]
    nodes_goal_connection_tested = []

    for i in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        goal = random() < goal_probability or i == 0

        # Sample. Try sampling from goal randomly - return a normal sample if unable to 
        # s = goal_sample() if goal else sample_fn()
        s = sample_fn()

        # Find closest config in tree to new sample config and extend from closest config to 
        last = argmin(lambda n: distance_fn(n.config, s), nodes)
        
        # If goal sampling, check the nearest point to the goal that hasn't been checked yet 
        # (avoids an infeasible sample near the goal blocking other samples from connecting to the goal)
        if goal:
            s = goal_sample() 
            last = argmin(lambda n: distance_fn(n.config, s), set(nodes)-set(nodes_goal_connection_tested))
            nodes_goal_connection_tested.append(last)
         
        configs = extend_fn(last.config, s)

        valid = True

        for conf_count in range(len(configs)):
            q = configs[conf_count]

            # Test collision at every 10th point and at end (to stop collision testing appearing as often in visualization)
            if conf_count%10 == 0 or conf_count == len(configs)-1:
                if collision_fn(robot_body, q):
                    valid = False
                    break

            # If any config passes through the goal, return this config and parents
            if goal_test(utils.tool_pose_from_config(robot_body, q)[0],utils.tool_pose_from_config(robot_body, goal_sample())[0], 0.01):
                # Add q as final tree node
                last = TreeNode(q,parent = last) 
                nodes.append(last)
                
                return last.retrace()
        
        # Add the node to the tree if the path to it doesn't cause a collision, but also doesn't pass through the goal
        if valid:
            last = TreeNode(configs[len(configs)-1], parent=last)
            nodes.append(last)

    return None