# Looking at the folllowing repo:
# https://github.com/pucrs-automated-planning/pddl-parser

from pddl_parser.PDDL import PDDL_Parser
from pddl_parser.action import Action
import os

class Planner():

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
    def solve(self, state_curr, h_max=float('inf'), relaxed=False):
        # Initialise solution
        state = state_curr #self.s_0 # NEED TO CHANGE when calling for relaxed??? - see below for relaxed -> does this solve?
        h_curr = h_max #float('inf') # HOW USE H_MAX TO PRUNE/SET MAX DEPTH??????? - see below for relaxed -> does this solve?
        
        # If this is a recursive call to solve the relaxed graph, retain present state and h
        #if relaxed:
         #   state = state_curr
          #  h_curr = h_max
        
        # DO WE NEED BOTH OF THESE????
        action_plan = []
        action_plan_relaxed = []
        #num_actions_poss = 0 # Only need if not using ordered_actions list - remember to switch on reset

        # Continue performing actions while not in the goal state
        i = 0
        while not self.applicable(state, self.s_goal_pos, self.s_goal_neg):
            flag_next_found = False # True if a next possible action has been found
            #num_actions_poss = 0    # Reset number of actions possible in this state
        
            # Find list of next possible actions (starting with helpful actions)
            actions_ordered = self.ordered_actions(state, action_plan_relaxed)

            #print("ACTIONS ORDERED", (actions_ordered[i] for i in range(len(actions_ordered))))
            if i == 1:
                return actions_ordered

            # Search through ordered actions (which are already known to be possible)
            for a_dash in actions_ordered:
                # Try selected possible action
                next_state = self.act(state, a_dash, relaxed) #True rather than relaxed?
                

                # Find heuristic of next state
                action_plan_relaxed = self.solve(next_state, h_curr, True)
                h_next = len(action_plan_relaxed)

                # If heuristic of next state is lower than current state, take action
                if h_next < h_curr:
                    h_curr = h_next 
                    state = next_state
                    action_plan.append(a_dash)
                    flag_next_found = True
                    break


            if len(actions_ordered) == 0:
                # Reached a dead end - try A* search (unimplemented currently)
                #action_plan = A_star(self.s_0, self.s_goal_pos, self.s_goal_neg, self.actions)
                print('DEAD END REACHED \n')
                action_plan = []
                break
            elif not flag_next_found:
                # Plateau reached - perform BFS to get out (unimplemented currently)
                #state, acts_off_plateau = BFS(state, self.s_goal_pos, self.s_goal_neg, h_curr)
                #action_plan.append(acts_off_plateau)
                print('PLATEAU REACHED \n')
                break # REMOVE BREAK ONCE IMPLEMENTED

            i = i+1

        return action_plan

    # returns if the current state satisfies positive and negative conditions
    def applicable(self, state, positive, negative):
        return set(positive).issubset(set(state)) and set(negative).isdisjoint(set(state))

    # returns if the action is possible
    def possible_action(self, action, state):
        return action.positive_preconditions.issubset(state) and action.negative_preconditions.isdisjoint(state)

    # returns all possible actions for a given state. NO combinations of actions to prevent mutexes for now
    def possible_actions(self, state):
        return [a for a in self.parser.actions if self.possible_action(a,state)]

    # Performs an action on a state and returns the resulting state (replaced apply in BFS example planner)
    def act(self, state, action, relaxed=False):
        s_next = state
        
        # Allows this function to be used for ff heuristic
        if not relaxed:
            s_next.difference(action.del_effects)
        
        s_next = s_next.union(action.add_effects)
        return s_next

    # Returns an ordered list of actions possible from 'state' to be searched through - starting with helpful actions
    def ordered_actions(self, state, action_plan_relaxed):
        actions_ordered = []
        possible_actions = self.possible_actions(state)
        
        # Find helpful actions  - UNCOMMENT AND TEST
        # Helpful actions only exist if a relaxed plan has been generated
        # if len(action_plan_relaxed) != 0:
        #     action_plan_relaxed_next = action_plan_relaxed[0]
            
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


# -----------------------------------------------
# Main - copied from planner.py from pddl_parser
# -----------------------------------------------
if __name__ == '__main__':
    import sys, time
    start_time = time.time()

    # Run on default domain and problem - for project
    dirname = os.path.dirname(__file__)
    domain = os.path.join(dirname,'dinner.pddl')
    problem = os.path.join(dirname,'pb1.pddl')
    verbose = True

    # If arguments are given, replace problem to run on
    if len(sys.argv) > 1:
        domain = sys.argv[1]
        problem = sys.argv[2]
        verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'

    # Solve problem using action planner
    planner = Planner(domain, problem)
    plan = planner.solve(planner.s_0)
    print('Time: ' + str(time.time() - start_time) + 's')
    if type(plan) is list:
        print('plan:')
        for act in plan:
            print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
    else:
        print('No plan was found')
        exit(1)


'''
        Useful functions from planner on github:
                 parser.actions #likely a set
                 action.groundify(parser.objects, parser.types) #another set of actions
                
                parser.objects
                parser.types

                act.positive_preconditions
                    positive_preconditions.issubset(state)
                act.negative_preconditions
                    negative_preconditions.isdisjoint(state)
                


                positive = act.add_effects
                    state.difference(negative)
                        state.difference(something).union(positive)
                negative = act.del_effects

                act.difference(negative).union(positive)


'''
