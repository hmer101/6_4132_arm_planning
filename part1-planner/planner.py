# Looking at the folllowing repo:
# https://github.com/pucrs-automated-planning/pddl-parser

from pddl_parser.PDDL import pddl_parser
from pddl_parser.action import Action


class Planner():

    def __init__(self, domain_file, problem_file):
        self.parser = pddl_parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)
        
    def solve(self, relaxed=False):
        s = parser.state
        g = parser.positive_goals
        not_g = parser.negative_goals
        noop = "noop"

        plan = [(s, noop)]
        
        i = 0
        while i < 10000: # (counter just for now to prevent infinite loop)
            
            # Checks that the goal has/has not been acheived
            if s == g and s != not_g:
                return plan

            # Finds ff for every possible action in a list of tuples 
            # [(a1, ff(a1)), (a2, ff(a2), ...)]
            outcomes = [(a,self.ff(s, a)) for a in self.possible_actions(s)]
            
            # Chooses the oucome with the highest ff (looking at index 1)
            if not relaxed:
                a_selection, ff = min(outcomes, key=lambda x: x[1])
            else:
                a_selection = ____ #SOMETHING, idk what though
            
            # Performing the action to get to the next state
            s_next = self.act(s, a_selection, relaxed=relaxed)

            # Iterate
            s = s_next
            plan.append( (s, a_selection) )

            # There is currently no backup or any A*
            # Currently only DFS, no A* (need to add BFS at a plateau)


    # returns if the action is possible
    def possible_action(action, state):
        return action.positive_preconditions.issubset(state) and action.negative_preconditions.isdisjoint(state)

    # returns all possible actions for a given state. NO combinations of actions to prevent mutexes for now
    def possible_actions(state):
        return [a for a in self.parser.actions if self.possible_action(a,state)]

    # 
    def ff(state, action):
        return self.solve(s_init, goal, not_goal, relaxed=True)


    # returns all possible sets of actions
    #
    #     DO WE ASSUME IT CAN ONLY DO ONE ACTION AT A TIME? SEEMS REASONABLE
    #     AND SIMPLIFIES CODE, REDUCES RUNTIME SIGNIFICANTLY. LEFT OUT FOR NOW
    #
    def action_combinations(state):
        return 


    def act(self, state, action, relaxed=False):
        s_next = state
        
        # Allows this function to be used for ff heuristic
        if not relaxed:
            s_next.difference(action.negative_preconditions)
        
        s_next.union(action.positive_preconditions)
        return s_next



new_state = self.apply(state, act.add_effects, act.del_effects)
state.difference(negative).union(positive)



# -----------------------------------------------
# Main - copied from planner.py from pddl_parser
# -----------------------------------------------
if __name__ == '__main__':
    import sys, time
    start_time = time.time()
    domain = sys.argv[1]
    problem = sys.argv[2]
    verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'
    planner = Planner()
    plan = planner.solve(domain, problem)
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

