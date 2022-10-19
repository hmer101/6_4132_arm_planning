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

cd part1-pddl
python3 -B -m pddl_parser.PDDL domain.pddl problem.pddl -v


PDDL Parser's inbuild planner on custom domain/problem:

cd part1-pddl
python3 -B -m pddl_parser.planner domain.pddl problem.pddl -v



## Assumptions
- Robot can only hold a single item at a time
- The countertop, burner and drawer can each only hold a single item at a time (surfaces could be divided into multiple sections to allow each surface to hold multiple items, but this is not implemented yet).
- Draw is initially closed and must end closed


## Current Issues
- When running domain and problem with PDDL_parser's inbuild BFS planner, returns 'no plan was found' in 0.000566 seconds



