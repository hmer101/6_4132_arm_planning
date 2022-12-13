# padm_proj1
Harvey and Kevin's PADM project 1 2022

Please find the final project documentation on Google Docs here: https://docs.google.com/document/d/1l4H05ZDN13Z1PVzFIdiocDltyawPMel44Kedhbwq78Y/edit?usp=sharing


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

