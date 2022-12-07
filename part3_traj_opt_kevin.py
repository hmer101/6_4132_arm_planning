from pydrake.solvers import MathematicalProgram
import numpy as np
import math
import matplotlib.pyplot as plt
import pydrake.solvers
from pybullet_tools.utils import get_link_pose, get_joint_position, get_joint_positions
from utils import all_poses_from_config, interpolate_configs
from rrt import detect_collision

class Trajectory():
    def __init__(self, world, steps, start, goal):
        self.prog = MathematicalProgram()
        self.c_len = 7 #len(config_comfortable)
        #self.config_comfortable = config_comfortable
        self.N = steps
        self.vars = self.prog.NewContinuousVariables(self.N, self.c_len, "config")
        self.world = world #PLEASE pass by refrence...
        self.start = start
        self.goal = goal
        #self.init_config = init_config

    def discretize(self, init_config_path):
        large_path = []
        intermediate_steps = np.lcm(len(init_config_path), self.N)
        for i in range(len(init_config_path)-1):
            start = init_config_path[i]
            end = init_config_path[i+1]
            large_path.extend(interpolate_configs(start, end, num_steps=intermediate_steps))
        return large_path

    def optimize(self, init_config_path):


        # Reduces the deviation from the arms "comfortable" config
        ####################################
        # self.prog.AddQuadraticErrorCost(
        #     Q=np.array(np.diag([x for x in range(self.c_len)])), # weight each joint slightly differently
        #     x_desired=np.array(self.config_comfortable),
        #     vars=self.vars
        # )

        # Reduces the deviation from the arms "comfortable" config
        ####################################
        # self.prog.AddQuadraticErrorCost(
        #     Q=np.array(np.diag([x for x in range(self.c_len)])), # weight each joint slightly differently
        #     x_desired=np.array(self.config_comfortable),
        #     vars=self.vars
        # )

        # Constrains the joint positions to their min and max values
        ####################################
        # self.prog.AddConstraint(
        #     lambda config: all_poses_from_config(self.world.robot, config),
        #     lb=[0, 0],
        #     ub=[np.inf, np.inf],
        #     vars=self.vars)
        
        # Detects collisions
        ####################################
        # self.prog.AddConstraint(
        #     lambda config: detect_collision(self.world.robot, config),
        #     lb=np.NINF,
        #     ub=0
        #     vars=self.vars
        # )

        

        # Returns 1 once it has a collision, or (hopefully) returns 0
        ####################################
        def has_collision(configs):
            for config in configs:
                if detect_collision(self.world.robot, config):
                    return 1
            return 0
        

        # Total config space distance traveled, sums over config1, config2, ...
        ####################################
        def config_dist(configs):
            return sum([c_dist(configs[i+1],configs[i]) for i in range(self.N-1)])
            
        # Root-Square error between 2 configs 
        ####################################
        def c_dist(c1, c2):
            return (sum([(c1[i+1]-c2[i])**2 for i in range(self.c_len)]))**0.5

        
        
        def v_to_a(in_vars):
            return np.reshape(np.array(in_vars), (self.c_len, self.N))

        def error_start(config):
            return c_dist(config, self.start)
        def error_goal(config):
            return c_dist(config, self.goal)
        
            configs = v_to_a(in_vars)
            return c_dist(configs[len(configs)], self.goal) + c_dist(configs[0], self.goal)


        # Detect collision for vars=[q0|q1|...|qn]
        ####################################
        self.prog.AddConstraint(has_collision, lb=np.zeros(self.N), ub=np.zeros(self.N), vars=self.vars)

        # Sets an allowable max error from the start and end goals, can/should searate between start and end
        self.prog.AddConstraint(error_start, lb=-0.001, ub=0.001, vars=self.vars[0])
        self.prog.AddConstraint(error_goal,  lb=-0.001, ub=0.001, vars=self.vars[self.N-1])

        

        


        # Minimize the travel cost
        ###################################
        self.prog.AddCost(lambda z: config_dist(z), vars = self.vars)



        result = pydrake.solvers.Solve(self.prog, np.array(init_config_path))

        return result



# def main():

# if __name__ == '__main__':
#     main()