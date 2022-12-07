import time
import numpy as np
import rrt

import utils
from pybullet_tools.utils import get_joint_positions, set_joint_positions
from src.utils import translate_linearly
__import__('padm-project-2022f')
from pybullet_tools.utils import set_pose, Pose, get_distance, Point, Euler, multiply, stable_z_on_aabb, wait_for_user, link_from_name
from pybullet_tools.utils import set_joint_positions, get_link_pose, get_joint_positions

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints

from src.world import World
from src.utils import COUNTERS, compute_surface_aabb, name_from_type, translate_linearly, surface_from_name
import math

from part3_traj_opt_kevin import Trajectory


UNIT_POSE2D = (0., 0., 0.)
GRIPPER_OFFSET_SPAM = (0.1, 0, 0.1)
GRIPPER_OFFSET_SUGAR = (0.15, 0.0, 0.15)
MAX_GOAL_RADIUS = 0.075
PRE_RENDER = False
DROP_DISTANCE = 0.15

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)


np.set_printoptions(precision=3, suppress=True)
world = World(use_gui=True)
sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
obj_pos_meat = utils.get_pose_obj_goal(world, 'potted_meat_can1')
obj_pos_sugar = utils.get_pose_obj_goal(world, 'sugar_box0')
pos_counter = utils.pose_offset(obj_pos_meat, GRIPPER_OFFSET_SPAM[0], GRIPPER_OFFSET_SPAM[1], GRIPPER_OFFSET_SPAM[2])
pos_burner = utils.pose_offset(obj_pos_sugar, GRIPPER_OFFSET_SUGAR[0], GRIPPER_OFFSET_SUGAR[1], GRIPPER_OFFSET_SUGAR[2])
franka_name = 'franka'
indigo_drawer_handle_name = 'indigo_drawer_handle'
indigo_drawer_center_name = 'indigo_drawer'
burner_name = 'burner'
countertop_name = 'countertop'
KITCHEN_BODY = 0
item_in_hand = dict()
item_in_holder = dict()
is_open = dict()
is_open[indigo_drawer_center_name] = False
item_in_hand[franka_name] = None
item_in_itemholder = {indigo_drawer_center_name:[]}

tool_link = link_from_name(world.robot, 'panda_hand')
ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
start_pose = get_link_pose(world.robot, tool_link)
handle_pose_closed = utils.get_handle_position(world, is_open=False)
pos_indigo_drawer_center = utils.pose_offset(utils.get_drawer_center_position(world), 0, 0, DROP_DISTANCE)
current_conf = get_joint_positions(world.robot, world.arm_joints)



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

def rand_position(start_pose):
    return multiply(start_pose, Pose(Point(z=1.0)))
    
def get_robot(robot_name):
    if robot_name == 'franka':
        return world.robot

def planner_get_pose(end_location_name, offset=(0,0,0)):
    pos = None
    if end_location_name == burner_name:
        pos= pos_burner
    if end_location_name == indigo_drawer_handle_name:
        areuopen = is_open[indigo_drawer_center_name]
        pos= utils.get_handle_position(world, areuopen)
        #return get_drawer_pose(end_location_name) #TODO CHECK THIS FUNCTION
    if end_location_name == indigo_drawer_center_name:
        pos= pos_indigo_drawer_center
    if end_location_name == countertop_name:
        pos= pos_counter
    if end_location_name == 'nowhere': # If the robot is nowhere of significance, just return its init config
        pos= utils.tool_pose_from_config(world.robot, get_joint_positions(world.robot, world.arm_joints))
        #return utils.get_surface_position(world, 'indigo_tmp')
    return utils.pose_offset(pos, offset[0], offset[1], offset[2])

def get_drawer_link(drawer_name):
    return link_from_name(KITCHEN_BODY,drawer_name)

def get_drawer_pose(drawer_name):
    return get_link_pose(KITCHEN_BODY, get_drawer_link(drawer_name))

def get_surface(surface_name):
    return surface_from_name(indigo_drawer_center_name)

def get_item_offset(robot_name):
    body = item_in_hand[robot_name]


def navigate(robot_name, start_location, end_location):
    robot = get_robot(robot_name)
    
    end_pose = planner_get_pose(end_location)

    if end_pose == None:
        print(f"Error! No end_pose found for {end_location}")
        wait_for_user()
    start_pose = planner_get_pose(start_location)
    current_pose = utils.tool_pose_from_config(world.robot, get_joint_positions(world.robot, world.arm_joints))

    if get_distance(current_pose[0], start_pose[0]) >= 0.21:
        print (f"ERROR! ROBOT NOT AT START POSE!\ntool_pose_from_config={current_pose}\nstart_pose={start_pose}")
        wait_for_user()

    end_config = utils.get_goal_config(world, get_joint_positions(world.robot, world.arm_joints), end_pose)

    if end_config == None:
        radius = 0.01
        while end_config == None and radius < MAX_GOAL_RADIUS:
            radius += 0.075/2-.01
            end_config = utils.get_goal_config(world, get_joint_positions(world.robot, world.arm_joints), end_pose, goal_radius=radius)
    
    if PRE_RENDER:
        save = get_joint_positions(world.robot, world.arm_joints)
        print("End config found and shown.")
        set_joint_positions(world.robot, world.arm_joints, end_config)
        wait_for_user()
        set_joint_positions(world.robot, world.arm_joints, save)
    
    
    

    #config_path = [get_joint_positions(world.robot, world.arm_joints), end_config]
    config_path = rrt.rrt_arm_wrapper(get_joint_positions(world.robot, world.arm_joints), end_config, world.robot, world.arm_joints)
    
    if config_path == None:
        print ("ERROR! No config_path found! Exiting program")
        wait_for_user()


    if end_location == indigo_drawer_handle_name and start_location == 'nowhere':
        steps = 10
        opt = Trajectory(world, steps, config_path[0], config_path[-1])
        init_config_path = opt.discretize(config_path)
        path = opt.optimize(init_config_path)
        config_path = [path]

    current_conf = utils.move(world, config_path, item_in_hand=item_in_hand[robot_name])

def get_surface_name(surface_name):
    if surface_name == indigo_drawer_center_name:
        return 'indigo_drawer_top'
    if surface_name == burner_name:
        return 'indigo_drawer_top'

def close_drawer (robot_name, drawer_name):
    #robot = get_robot(robot_name)
    surface = get_surface('indigo_drawer_top')
    current_conf = utils.close_the_drawer(world, surface, items_on_surface=item_in_itemholder[drawer_name])
    is_open[drawer_name] = False
def open_drawer(robot_name, drawer_name, drawer_handle_name):
    #robot = get_robot(robot_name)
    #sname = get_surface_name(drawer_name)
    surface = get_surface('indigo_drawer_top')
    current_conf = utils.open_the_drawer(world,surface, items_on_surface=item_in_itemholder[drawer_name])
    is_open[drawer_name] = True

def body_name_from_item_name(item_name):
    if item_name == 'spam_box':
        return 'potted_meat_can1'
    if item_name == 'sugar_box':
        return 'sugar_box0'

def body_from_item_name(item_name):
    return world.body_from_name[body_name_from_item_name(item_name)]

def pick_up (robot_name, item_name, item_holder_name):
    body = body_from_item_name(item_name)
    item_in_hand[robot_name] = body
    item_in_itemholder[item_holder_name] = []


def place (robot_name, item_name, surface_name):
    pose = None

    if surface_name == indigo_drawer_center_name:
        drawer_link = link_from_name(KITCHEN_BODY,'indigo_drawer_top')
        pose = get_link_pose(KITCHEN_BODY, drawer_link)
    else:
        gripper_pose = planner_get_pose(surface_name)
        pose = utils.pose_offset(gripper_pose, -GRIPPER_OFFSET_SPAM[0], -GRIPPER_OFFSET_SPAM[1], -GRIPPER_OFFSET_SPAM[2])
            
    
    item_in_itemholder[surface_name].append(body_from_item_name(item_name))
    set_pose(item_in_hand[robot_name], pose)
    #body_name = body_name_from_item_name(item_name)
    #their_surface_name = convert_surface_name(surface_name)
    #pose2d_on_surface(world, body_name, their_surface_name)
    item_in_hand[robot_name] = None

def convert_surface_name(surface_name):
    if surface_name == indigo_drawer_center_name:
        return 'indigo_drawer_bottom'




def perform_action (name, params):
    a = name
    if a == 'open-drawer':
        open_drawer(params[0],params[1],params[2])
    if a == 'navigate':
        navigate(params[0],params[1],params[2])
    if a == 'pick-up':
        pick_up(params[0],params[1],params[2])
    if a == 'place':
        place(params[0],params[1],params[2])
    if a == 'close-drawer':
        close_drawer(params[0],params[1])
