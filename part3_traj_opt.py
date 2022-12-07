from pydrake.solvers import MathematicalProgram
import numpy as np
import matplotlib.pyplot as plt

# TEMPORARY JUST TO GET JOINT LIMITS!!
import execute_plan as exec
__import__('padm-project-2022f')
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits

def main():
    lower_limits, upper_limits = get_custom_limits(exec.world.robot, exec.world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)
    #generator = interval_generator(lower_limits, upper_limits, **kwargs)

    # Set up mathematical program with variables
    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(7, "arm_config")
    #print(x)

    # Set up constraints
    # Joint limits
    for joint_num in range(len(lower_limits)):
        #prog.AddConstraint(-x[joint_num] <= -lower_limits[joint_num])
        #prog.AddConstraint(x[joint_num] <= upper_limits[joint_num])
        prog.AddBoundingBoxConstraint(lower_limits[joint_num], upper_limits[joint_num], x[joint_num])
        

    print(prog)
    # Start and goal configs


    # Collision avoidance



if __name__ == '__main__':
    main()