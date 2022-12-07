# MAIN.py

from __future__ import print_function
import execute_plan as exec
import os
import sys
from pathlib import Path

sys.path.insert(0,os.path.abspath(os.path.join(os.getcwd(), 'part1')))
from ff_planner import FF_Planner

from pybullet_tools.utils import  wait_for_user, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed

def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())
    exec.world._update_initial()
    exec.action_navigate(exec.world)


    dirname = os.path.abspath(os.path.join(os.getcwd(), 'part1'))
    domain = os.path.join(dirname,'domain.pddl') #dinner blocksworld.pddl domain.pddl
    problem = os.path.join(dirname,'problem.pddl') #pb1_dinner pb4_blocksworld.pddl problem.pddl
    ff_planner = FF_Planner(domain, problem)
    plan = ff_planner.solve()

    if type(plan) is list:
        print("\nPlan found:")
        for act in plan:
            print((act.name) + ' ' + ' '.join(act.parameters))
    else:
        print('No plan was found')
        exit(1)

    print('\nEXECUTING PLAN:')
    for act in plan:
        print('\tPerforming action ' + (act.name) + ' ' + ' '.join(act.parameters))
        exec.perform_action(act.name, act.parameters)
    print('Action plan completed. Press enter to exit.')
    wait_for_user()

if __name__ == '__main__':
    main()
