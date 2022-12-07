from pydrake.solvers import MathematicalProgram
import numpy as np
import matplotlib.pyplot as plt
import pydrake.solvers
from pybullet_tools.utils import get_link_pose, get_joint_position, get_joint_positions
from utils import all_poses_from_config

class Trajectory():
    def __init__(self, world, config_comfortable=(0,0,0,0,0,0,0)):
        self.prog = MathematicalProgram()
        self.c_len = len(config_comfortable)
        self.config_comfortable = config_comfortable
        self.vars = self.prog.NewContinuousVariables(self.c_len)
        self.world = world #PLEASE pass by refrence...


    def optimize(self, config_init):

        #def cost_fun(config):
        #    return (sum([(config[i]-self.config_comfortable[i])**2 for i in range(len(self.c_len))]))**0.5


        # Reduces the deviation from the arms "comfortable" config
        self.prog.AddQuadraticErrorCost(
            Q=np.array(np.diag([x for x in range(self.c_len)])), # weight each joint slightly differently
            x_desired=np.array(self.config_comfortable),
            vars=self.vars
        )

        # Constrains the joint positions to their min and max values
        self.prog.AddConstraint(
            lambda poses: all_poses_from_config(world.robot, config),
            lb=[0, 0],
            ub=[np.inf, np.inf],
            vars=self.vars)
        

        result = pydrake.solvers.Solve(self.prog, np.array(config_init)))



def main():

if __name__ == '__main__':
    main()