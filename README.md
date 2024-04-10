# 6.4132/16.413 Final Project
Harvey and Kevin's 'principles of autonomny and decision making' project 1 2022.

Please find the final project documentation in the file '6_4132_final_report'.


## Setup
Follow the instructions here: https://gitlab.com/mit-mers/teaching/padm-project-2022f

Replace step 2 above with the instructions below:
To clone repository, make sure that the submodules are also cloned:
git clone --recurse-submodules https://github.com/hmer101/padm_proj1.git
cd cd padm-project-2022f

Continue from step 3


## Commands to run


Parse with PDDL Parser:

cd part1
python3 -B -m pddl_parser.PDDL domain.pddl problem.pddl -v


PDDL Parser's inbuilt planner on custom domain/problem:

cd part1
python3 -B -m pddl_parser.planner domain.pddl problem.pddl -v

