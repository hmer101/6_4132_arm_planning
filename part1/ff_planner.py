# Looking at the folllowing repo:
# https://github.com/pucrs-automated-planning/pddl-parser

from pddl_parser.PDDL import PDDL_Parser
from pddl_parser.action import Action
from bfs_planner import BFS_Planner
import os

class FF_Planner():

    def __init__(self, domain_file, problem_file):
        self.parser = PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)

        # Parse initial and goal states
        self.s_0 = self.parser.state
        self.s_goal_pos = self.parser.positive_goals
        self.s_goal_neg = self.parser.negative_goals

        # Parse/ground actions
        self.actions = []
        for action in self.parser.actions:
            for act in action.groundify(self.parser.objects, self.parser.types):
                self.actions.append(act)


    # Solver uses a simplified fast forward planner that:
    #   a) Uses enforced hill climbing
    #   b) Resorts to BFS at plateaus in h_FF - NOT YET IMPLEMENTED     
    #   c) Uses the relaxed planning graph to generate h_FF
    #   d) Can only select a single action at each layer
    #   e) Resorts to A* search on EHC failure (when a dead-end is reached) - NOT YET IMPLEMENTED
    def solve(self):
        # Initialise solution
        state = set(self.s_0) 
        h_curr = float('inf')
        
        action_plan = []
        action_plan_relaxed = []

        # Continue performing actions while not in the goal state
        while not self.applicable(state, self.s_goal_pos, self.s_goal_neg):
            flag_next_found = False # True if a next possible action has been found
        
            # Find list of next possible actions (starting with helpful actions)
            actions_ordered = self.ordered_actions(state, action_plan_relaxed)

            # Search through ordered actions (which are already known to be possible)
            for a_dash in actions_ordered:
                # Try selected possible action
                next_state = self.act(state, a_dash) #True rather than relaxed?
                

                # Find heuristic of next state
                action_plan_relaxed, h_next = self.relaxed_plan(next_state)
                #print(f"\nh_next={h_next} for action {a_dash.name} to state: {next_state}")
                #print(("State:{}\nAction: {}, {}\nhFF: {}").format(next_state, a_dash.name,a_dash.parameters, h_next))

                # If heuristic of next state is lower than current state, take action - enforced hill climbing
                if h_next < h_curr:
                    #print(f"\n*****************\nh_next={h_next}<h_curr={h_curr}, taking action {a_dash.name} to {next_state}...\n****************")
                    h_curr = h_next 
                    state = next_state
                    action_plan.append(a_dash)
                    flag_next_found = True
                    break
            
            #print('---------------------------------------------------------\n')

            if len(actions_ordered) == 0: #and not self.applicable(state, self.s_goal_pos, self.s_goal_neg) - incase goal is dead end?
                # Reached a dead end - try A* search (unimplemented currently)
                #action_plan = A_star(self.s_0, self.s_goal_pos, self.s_goal_neg, self.actions)
                print('DEAD END REACHED \n')
                action_plan = []
                break
            elif not flag_next_found:
                #print('PLATEAU REACHED \n')
                # Plateau reached - perform BFS to get out
                bfs_planner = BFS_Planner(frozenset(state), self.s_goal_pos, self.s_goal_neg, self.actions)
                action_plan_bfs = bfs_planner.solve()

                # Append BFS plan to FF plan
                for a_bfs in action_plan_bfs:
                    action_plan.append(a_bfs)

                break

        return action_plan

    # returns if the current state satisfies positive and negative conditions
    def applicable(self, state, positive, negative):
        return set(positive).issubset(set(state)) and set(negative).isdisjoint(set(state))

    # returns if the action is possible
    def possible_action(self, action, state):
        return action.positive_preconditions.issubset(state) and action.negative_preconditions.isdisjoint(state)

    # returns if the action is possible in relaxed
    # Positive preconditions are present in the state and negative preconditions that haven't been removed are not present in the start/target state (i.e. negative preconditions are satisfied)
    def possible_action_relaxed(self, action, state_pos, start_state, state_removed):
        return action.positive_preconditions.issubset(state_pos) and ((action.negative_preconditions - state_removed).isdisjoint(start_state))


    # returns all possible actions for a given state. NO combinations of actions to prevent mutexes
    def possible_actions(self, state):
        return [a for a in self.actions if self.possible_action(a,state)]

    # returns all possible actions in a given state in the relaxed graph
    def possible_actions_relaxed(self, state_pos, start_state, state_removed):
        return [a for a in self.actions if self.possible_action_relaxed(a, state_pos, start_state, state_removed)]


    # Performs an action on a state and returns the resulting state (replaced apply in BFS example planner)
    def act(self, state, action):
        s_next = state
        s_next = s_next.difference(action.del_effects)
        s_next = s_next.union(action.add_effects)

        return s_next

    # Performs a relaxed action on a state and returns the resulting positive and negative parts of the state
    def act_relaxed(self, state_pos, state_neg, action):
        s_next_pos = state_pos
        s_next_neg = state_neg

        s_next_pos = s_next_pos.union(action.add_effects)
        s_next_neg = s_next_neg.union(action.del_effects)

        return s_next_pos, s_next_neg


    # Returns an ordered list of actions possible from 'state' to be searched through - starting with helpful actions
    def ordered_actions(self, state, action_plan_relaxed):
        actions_ordered = []
        possible_actions = self.possible_actions(state)
        
        # Find helpful actions  - UNCOMMENT AND TEST (note action_plan_relaxed may now have multiple actions per layer -> only take first layer and search through actions in that layer)
        # Helpful actions only exist if a relaxed plan has been generated
        # if len(action_plan_relaxed) != 0:
        #     action_plan_relaxed_next = action_plan_relaxed[0] # THIS LINE NEEDS TO BE FIXED
            
        #     # The first action in the relaxed plan is always helpful (if possible)
        #     if self.possible_action(action_plan_relaxed_next, state):
        #         actions_ordered.append(action_plan_relaxed_next) 

        #     next_state_relaxed = self.act(state,action_plan_relaxed_next, False) # Finds the next state generated using the relaxed plan

        #     # TEST! - LIKELY A MISTAKE HERE
        #     # Find all other possible actions that could lead to some facts in this next state
        #     for a_dash in possible_actions:
        #         next_state_test = self.act(state,a_dash,False)
                
        #         # If any facts in the next_state_relaxed are also created by a_dash, a_dash is helpful
        #         if (not next_state_relaxed.isdisjoint(next_state_test)) and (a_dash != action_plan_relaxed_next):
        #             actions_ordered.append(a_dash)

        # Add all other actions ("not helpful" ones)
        for a_dash in possible_actions:
            if actions_ordered.count(a_dash) == 0:
                actions_ordered.append(a_dash)

        return actions_ordered


    # Finds the relaxed plan and heuristic from the target to goal state
    def relaxed_plan(self, target):
        # Generate relaxed plan
        relaxed_act_record = list()
        state = target
        state_removed = set()


        # Keep adding action layers to relaxed graph whilst the goal state hasn't be reached
        # Need to remove the predicates present in the target and in the negative goals 
        state_need_removing = set(self.s_goal_neg) & target

        while not (set(self.s_goal_pos).issubset(state) and state_need_removing.issubset(state_removed)):
            #actions_poss = self.possible_actions(state)   # Find possible actions in current state
            actions_poss = self.possible_actions_relaxed(state, target, state_removed)
            #actions_poss = []

            # WILL THIS HELP??
            # Only add possible actions that actually add something (prevents repeat actions)
            # for test_act in actions_poss_full:
            #     if not test_act.add_effects.issubset(state) or not test_act.del_effects.issubset(state_removed)):
            #         actions_poss.append(test_act)
            
            relaxed_act_record.append(actions_poss) 

            # Generate next state in the relaxed plan by applying all possible (relaxed) actions
            next_state = state
            next_state_removed = state_removed

            for action in actions_poss:
                next_state, next_state_removed = self.act_relaxed(next_state, next_state_removed, action)

            state = next_state
            state_removed = next_state_removed

        # Extract plan and generate heuristic by moving back through action layers
        back_state = set(self.s_goal_pos)
        back_state_remove = state_need_removing
        plan_relaxed = []

        # If the relaxed plan is empty (i.e. the target is already at the goal)
        if relaxed_act_record is None:
            return [], 0
        
        for act_num in range(len(relaxed_act_record)-1, -1,-1):
            # Search through actions that produce back_state
            acts = relaxed_act_record[act_num]
            back_state_next = set()
            back_state_remove_next = set()
            next_acts = list()
            
            for action in acts:
                # If actions produce states that are desired and are not already present in the target
                if not action.add_effects.isdisjoint(back_state) or not action.del_effects.isdisjoint(back_state_remove):
                    # If an action produces effects that are present in the desired state, add it to the actions to take
                    next_acts.append(action)

                    # Remove the effects present in back_state and back_state_removed that are produced by the action just added
                    for add_eff in action.add_effects:
                        try:
                            back_state.remove(add_eff)
                        except: # Add effect not present in the back state
                            pass

                    for del_eff in action.del_effects:
                        try:
                            back_state_remove.remove(del_eff)
                        except: # Delete effect not present in the back state remove
                            pass

                    # Add the required preconditions of the action to the next state to search for
                    for add_precond in action.positive_preconditions:
                        back_state_next.add(add_precond)

                    # Add the required negative preconditions of the action that aren't satisfied in the target i.e. need to be satisfied before this action can be taken
                    for del_precond in (action.negative_preconditions & target):
                       back_state_remove_next.add(del_precond)

                # If all actions required to produce this state have been found
                if len(back_state) == 0 and len(back_state_remove) == 0:
                    break
            
            # Record actions to take in this step and move to next state
            plan_relaxed.append(next_acts)
            back_state = back_state_next
            back_state_remove = back_state_remove_next

        # Flip relaxed plan to be in forward direction
        plan_relaxed.reverse()        

        # Calculate heuristic by counting the number of actions in the relaxed plan
        #h_ff = sum([len(act_layer) for act_layer in plan_relaxed])
        h_ff = len(plan_relaxed)

        return plan_relaxed, h_ff



# -----------------------------------------------
# Main - copied from planner.py from pddl_parser
# -----------------------------------------------
if __name__ == '__main__':
    import sys, time
    start_time = time.time()

    # Run on default domain and problem - for project
    dirname = os.path.dirname(__file__)
    domain = os.path.join(dirname,'domain.pddl') #dinner blocksworld.pddl domain.pddl
    problem = os.path.join(dirname,'problem.pddl') #pb1_dinner pb4_blocksworld.pddl problem.pddl
    verbose = False
    debug = False

    # If arguments are given, replace problem to run on
    if len(sys.argv) > 1:
        domain = sys.argv[1]
        problem = sys.argv[2]
        verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'

    # Solve problem using action planner
    planner = FF_Planner(domain, problem)
    plan = planner.solve()
    print('Time: ' + str(time.time() - start_time) + 's')
    if type(plan) is list:
        print('plan:')
        for act in plan:
            print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
    else:
        print('No plan was found')
        exit(1)

        