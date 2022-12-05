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


PDDL Parser's inbuilt planner on custom domain/problem:

cd part1
python3 -B -m pddl_parser.planner domain.pddl problem.pddl -v


# Part 1
## Assumptions
**Domain/Problem**
- Robot can only hold a single item at a time.
- The countertop, burner and drawer can each only hold a single item at a time (surfaces could be divided into multiple sections to allow each surface to hold multiple items, but this is not implemented as not required).
- Draw is initially closed and must end closed.

**Planner**
- Only a single action can be taken at a time.

# Part 2
## Assumptions
 - When instructed to interact with an item up by the planner, the robot base is assumed to be within the arm's reach (ensured by selecting a static a base position to start with where all objects can be reached)
 - The gripper can grasp an object when it is within a certain small radius (e.g. 0.02 is used)

## Running
To run the file, navigate to the `part2` directory, and run `python3 main.py`

## Files
- part1
    - domain.PDDL
        - kitchen domain defined in PDDL
    - problem.PDDL
        - move_boxes_kitchen problem defined in PDDL
    - ff_planner.py
        - Contains the "main" function to run the planner
        - Syntax is based on interfacing with the pddl_parser's test planner
        - Solver uses a simplified fast forward planner that:
        a) Uses enforced hill climbing
        b) Resorts to BFS at plateaus in h_FF     
        c) Uses the relaxed planning graph to generate h_FF
        d) Can only select a single action at each layer
        e) Has a space to resort to A* search on EHC failure (when a dead-end is reached), but not implemented as this issue does not appear in our problem
    - bfs_planner.py
        - Uses breadth first search to solve the action planning problem
        - Called from ff_planner at a plateau to finish the action plan (could call to just get off the plateau but operates quickly)
- part2
    - rrt.py
        - Contains the core rrt solver and helper functions for different parts of rrt that can be substituted to allow the rrt solver to be used for different
        motion planning problems (e.g. moving the base and moving the arm)
        - Also contains a TreeNode class to store configs and parents in the RRT tree
        - Finally, contains wrappers for rrt (e.g. rrt_arm_wrapper) that simplifies calling rrt for a specific purpose (e.g. moving the arm)
    - utils.py 
        - Useful utility functions to interact with the world (e.g. hardcoded base movements, getting positions of items in the world, converting between poses and configs etc.)
    - execute_plan.py
        - Defines the functions that map between the action planner (from part 1) and the motion planner (from part 2)
        - Contains the "main" function to run the planner and the corresponding actions
- part3


## Challenges and Alternative Approaches
- A graph plan was considered, where the action with the lowest ff heuristic was selected.
- Our planner also has contingencies built in for when dead ends or a plateau are reached.
- Added ability to deal with negative goals and negative preconditions of actions as our problem formulation contains actions with negative preconditions. This is done by keeping a list of "removed" propositions when finding the relaxed action plan.

- Finding position of handle and position to move robot base to. "Hardcoded" these locations relative to drawer center.
- Base movement is "hardcoded" rather than using RRT. This allows no collision checking when base is moved - movement function and goal position designed for the kitchen such that no collisions will occur.


### Current Issues
- FF_Planner gives strange results in blocksworld pb4 - oscillates between pickup and putdown 
- FF doesn't order by helpful actions (implemented but not used)

- Collision checker visualizes when collision checking and so makes the simulation look spazzy
- RRT not RRT* so weird-looking motions

TODO part 2:
- Get countertop goal pose
- Link to action planner
    - Update action planner
    - "Put down" 
    - "Pick up"
    - Close drawer

- Clean up

Collision fxn
- Detect collisions with self (robot arm). Maybe have to do this by intersecting surfaces around links? Maybe some built-in functions to help? Look at current collision function for where self-collision checking fits in and methods already tried.
- Test if current collision method works with spam box and sugar bag



-> detect_collision CLIENT