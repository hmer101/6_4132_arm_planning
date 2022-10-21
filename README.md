# padm_proj1
Harvey and Kevin's PADM project 1 2022

## Setup
Follow the instructions here: https://gitlab.com/mit-mers/teaching/padm-project-2022f

Replace step 2 above with the instructions below:
To clone repository, make sure that the submodules are also cloned:
git clone --recurse-submodules https://github.com/hmer101/padm_proj1.git
cd cd padm-project-2022f

Continue from step 3


## Commands to run


Parse with PDDL Parser:

cd part1 \n
python3 -B -m pddl_parser.PDDL domain.pddl problem.pddl -v


PDDL Parser's inbuild planner on custom domain/problem:

cd part1 \n
python3 -B -m pddl_parser.planner domain.pddl problem.pddl -v


# Part 1
## Assumptions
- Robot can only hold a single item at a time
- The countertop, burner and drawer can each only hold a single item at a time (surfaces could be divided into multiple sections to allow each surface to hold multiple items, but this is not implemented yet).
- Draw is initially closed and must end closed


## Files
- domain.PDDL
- problem.PDDL
- planner.py
    - 

## Challenges and Alternative Approaches
- Recursive solver such that the function could be used for both the ff heuristic and the actual planner.
  - Infinite recursion is still a concern, but we are working out a way to determine an exit condition before the solver calls itself to prevent an infintie recursion.
  - Currently, our ff will only return values for "helpful" actions
- A graph plan was condsidered, where the action with the lowest ff heuristic was selected.
  - One issue is that our recursive solver resulted in an infinite loop.
- Our planner also has contingencies built in for when dead ends or a plateau are reached.

### Current Issues
- When running domain and problem with PDDL_parser's inbuild BFS planner, returns 'no plan was found' in 0.000566 seconds



