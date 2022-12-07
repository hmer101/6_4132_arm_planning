from pydrake.solvers import MathematicalProgram
import numpy as np
import matplotlib.pyplot as plt
import pydrake.solvers
from pybullet_tools.utils import get_link_pose, get_joint_position, get_joint_positions
from utils import all_poses_from_config
from rrt import detect_collision

class Trajectory():
    def __init__(self, world, steps, init_config):
        self.prog = MathematicalProgram()
        self.c_len = 7 #len(config_comfortable)
        #self.config_comfortable = config_comfortable
        self.N = steps
        self.vars = self.prog.NewContinuousVariables(self.c_len)
        self.world = world #PLEASE pass by refrence...
        self.init_config = init_config


    def optimize(self, config_init):


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
        def has_collision(in_vars):
            # goes from vars=[q0|q1|...|qn] to [q1, q2, ..., qn]
            configs = np.reshape(in_vars, (self.c_len, self.N))
            for config in configs:
                if detect_collision(self.world.robot, config):
                    return 1
            return 0
        

        # Total config space distance traveled, sums over config1, config2, ...
        ####################################
        def config_dist(in_vars):
            configs = np.reshape(in_vars, (self.c_len, self.N))
            return sum([c_dist(configs[i+1],configs[i]) for i in range(self.N-1)])
            
        # Root-Square error between 2 configs 
        ####################################
        def c_dist(c1, c2):
            return (sum([(c1[i+1]-c2[i])**2 for i in range(self.c_len)]))**0.5


        

        # Detect collision for vars=[q0|q1|...|qn]
        ####################################
        self.prog.AddConstraint(
            lambda config_vector: has_collision(config_vector),
            lb=np.NINF,
            ub=0
            vars=self.vars
        )

        # Minimize the travel cost
        ###################################
        self.prog.AddCost(lambda z: config_dist(z), vars = self.vars)



        result = pydrake.solvers.Solve(self.prog, np.array(config_init))

        return result



# def main():

# if __name__ == '__main__':
#     main()