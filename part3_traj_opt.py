from pydrake.solvers import MathematicalProgram, Solve
import numpy as np
import matplotlib.pyplot as plt

# TEMPORARY JUST TO GET JOINT LIMITS!!
import execute_plan as exec
__import__('padm-project-2022f')
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, get_joint_positions, get_distance

def main():
    lower_limits, upper_limits = get_custom_limits(exec.world.robot, exec.world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)
    #generator = interval_generator(lower_limits, upper_limits, **kwargs)

    # Set up mathematical program with variables
    prog = MathematicalProgram()
    N = 100 # Number of steps
    q = prog.NewContinuousVariables(N,7, "arm_configs") # DOES SWAPPING ROWS AND COLS CHANGE???

    #print(x)

    # Set up constraints
    # Joint limits
    for joint_num in range(len(lower_limits)):
        #prog.AddConstraint(-x[joint_num] <= -lower_limits[joint_num])
        #prog.AddConstraint(x[joint_num] <= upper_limits[joint_num])
        prog.AddBoundingBoxConstraint(lower_limits[joint_num], upper_limits[joint_num], q[:,joint_num])

    # Start config (can combine into above loop)
    #conf_start = get_joint_positions(exec.world.robot, exec.world.arm_joints)
    conf_start= [0.01200158428400755, -0.5697816014289856, 5.6801487517077476e-05, -2.8105969429016113, -0.00025768374325707555, 3.0363450050354004, 0.7410701513290405]
    for start_state in range(len(conf_start)):
        #q[0,start_state] = conf_start[start_state] - DIRECTLY INITIALIZE TO START???
        prog.AddConstraint(q[0,start_state] == conf_start[start_state]) # TRY CONSTRAINT??

    # End gripper pos (can combine into above loop)
    conf_end = [-0.9530217678375428, 1.0807033846407885, 0.42455643206797666, -1.5866223390438225, 1.6336623076131151, 2.0654691687300155, 1.1994754910475203]
    radii = (abs(np.array(upper_limits)-np.array(lower_limits)))*0.001 # Radius to be off by as a % of the full range of motion

    for end_state in range(len(conf_end)):
        prog.AddConstraint(abs(q[N-1,end_state]-conf_end[end_state]) <= radii[end_state]) 

    # (try with end gripper pos from conf)
    #tool_pose_from_config(robot_body, config)
        

    # Collision avoidance

    # Cost function (can combine into above loop)
    # Use min squared distance between successive configs
    for cost_num in range(N-1):
        prog.AddCost(sum((q[cost_num+1,:]-q[cost_num,:])**2)) # DOES THIS SUM WORK?


    # Solve 
    result = Solve(prog)

    # print out the result.
    print("Success? ", result.is_success())
    # Print the solution to the decision variables.
    print('x* = ', result.GetSolution(q))
    # Print the optimal cost.
    print('optimal cost = ', result.get_optimal_cost())
    # Print the name of the solver that was called.
    print('solver is: ', result.get_solver_id().name())
    



if __name__ == '__main__':
    main()