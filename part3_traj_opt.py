from pydrake.solvers import MathematicalProgram
import numpy as np
import math
import matplotlib.pyplot as plt
import pydrake.solvers
from pybullet_tools.utils import get_link_pose, get_joint_position, get_joint_positions
from utils import interpolate_configs #all_poses_from_config
from part2_rrt import detect_collision
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits


class Trajectory():
    def __init__(self, world, steps, start, goal):
        self.prog = MathematicalProgram()
        self.c_len = 7
        self.N = steps
        self.vars = self.prog.NewContinuousVariables(self.N, self.c_len, "config")
        self.world = world
        self.start = start
        self.goal = goal

    def discretize(self, init_config_path):
        large_path = []
        intermediate_steps = np.lcm(len(init_config_path), self.N)
        for i in range(len(init_config_path)-1):
            start = init_config_path[i]
            end = init_config_path[i+1]
            large_path.extend(interpolate_configs(start, end, num_steps=intermediate_steps))
        return large_path

    # Returns 1 once it has a collision, or (hopefully) returns 0
    ####################################
    def has_collision(self, config):
        return detect_collision(self.world.robot, config)


    def optimize(self, init_config_path):


        lower_limits, upper_limits = get_custom_limits(self.world.robot, self.world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)
        
        # Set up mathematical program with variables
        q = self.prog.NewContinuousVariables(self.N,self.c_len, "arm_configs")
        radii = (abs(np.array(upper_limits)-np.array(lower_limits)))*0.001 # Radius to be off by as a % of the full range of motion

        # Set up constraints
        for joint_num in range(len(lower_limits)):
            # Joint limits
            self.prog.AddBoundingBoxConstraint(lower_limits[joint_num], upper_limits[joint_num], q[:,joint_num])
            # Start config
            self.prog.AddConstraint(q[0,joint_num] == self.start[joint_num])
            # End config
            self.prog.AddConstraint(abs(q[self.N-1,joint_num]-self.goal[joint_num]) <= radii[joint_num])

        # NEED TO ADD OBSTACLE COLLISION STUFF
        # Detect collision for vars=[q0|q1|...|qn]
        ####################################
        # for i in range(self.N):
        #     self.prog.AddConstraint(self.has_collision(q[i]), lb=np.zeros(self.N), ub=np.zeros(self.N), vars=q)

        # Cost function
        for cost_num in range(self.N-1):
                self.prog.AddCost(sum((q[cost_num+1,:]-q[cost_num,:])**2)) # DOES THIS SUM WORK?


        result = pydrake.solvers.Solve(self.prog)

        return result.GetSolution(q)

