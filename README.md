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

cd part1
python3 -B -m pddl_parser.PDDL domain.pddl problem.pddl -v


PDDL Parser's inbuild planner on custom domain/problem:

cd part1
python3 -B -m pddl_parser.planner domain.pddl problem.pddl -v


# Part 1
## Assumptions
**Domain/Problem**
- Robot can only hold a single item at a time.
- The countertop, burner and drawer can each only hold a single item at a time (surfaces could be divided into multiple sections to allow each surface to hold multiple items, but this is not implemented yet).
- Draw is initially closed and must end closed.

**Planner**
- Only a single action can be taken at a time.
- 


## Files
Contained in part1 folder.

- domain.PDDL
- problem.PDDL
- planner.py
    - Syntax is based on interfacing with the pddl_parser's test planner
    - Solver uses a simplified fast forward planner that:
       a) Uses enforced hill climbing
       b) Resorts to BFS at plateaus in h_FF     
       c) Uses the related planning graph to generate h_FF
       d) Can only select a single action at each layer
       e) Resorts to A* search on EHC failure (when a dead-end is reached) - NOT YET IMPLEMENTED

## Challenges and Alternative Approaches
- A graph plan was condsidered, where the action with the lowest ff heuristic was selected.
- Our planner also has contingencies built in for when dead ends or a plateau are reached.
- Adding ability to deal with negative goals and negative preconditions of actions as our problem formulation contains 


### Current Issues
- FF_Planner gives strange results 
- Doesn't order by helpful actions


