# Looking at the folllowing repo:
# https://github.com/pucrs-automated-planning/pddl-parser

from pddl_parser.PDDL import PDDL_Parser
from pddl_parser.action import Action
import os


class Node():
    def __init__(self, state, parent):
        self.state = state
        self.parent = parent
    

class BFS_Planner():
    def __init__(self, s_0, positive_goals, negative_goals, actions):
        # Parse initial and goal states
        self.s_0 = s_0
        self.s_goal_pos = positive_goals
        self.s_goal_neg = negative_goals

        # Parse/ground actions
        self.actions = actions


    # Solver uses a BFS Planner:
    def solve(self):
        # Exit condition
        if self.at_goal(self.s_0):
            return []

        # Initialise solution
        q_curr = [(self.s_0, [])]
        q_next = []
        visited=set([self.s_0])
        max_iter = 1000
        i = 0

        # Continue performing actions while not at max iteration
        while i < max_iter:
            i += 1

            # iterate thru the q
            for state, action_plan in q_curr:

                actions = self.possible_actions(state)

                for a in actions:

                    # Obtain the next next state
                    next_s = self.act(state, a)
                    if next_s in visited:
                        continue
                    visited.add(next_s)
                    next_action_plan = [b for b in action_plan]#.append(a)
                    next_action_plan.append(a)
                    
                    if self.at_goal(next_s):
                        return next_action_plan
                    
                    q_next.append((next_s, next_action_plan))
        
            q_curr = q_next
            q_next = []

        print ("No solution found")
        return None

                    
    




    # Returns if it is at the goal
    def at_goal(self, state):
        return self.applicable(state, self.s_goal_pos, self.s_goal_neg)

    # returns if the current state satisfies positive and negative conditions
    def applicable(self, state, positive, negative):
        return set(positive).issubset(set(state)) and set(negative).isdisjoint(set(state))

    # given a state and an action, returns if said action is allowed
    def possible_action(self, action, state):
        out = action.positive_preconditions.issubset(state) and action.negative_preconditions.isdisjoint(state)
        return out
        
    # returns all possible actions for a given state. NO combinations of actions to prevent mutexes for now
    def possible_actions(self, state):
        out = [a for a in self.actions if self.possible_action(a,state)]
        return out


    # Performs a single action on the state, returns the resulting state (replaced apply in BFS example planner)
    def act(self, state, action, relaxed=False):
        s_next = state
        # Allows this function to be used for ff heuristic
        if not relaxed:
            s_next = s_next.difference(action.del_effects)
        s_next = s_next.union(action.add_effects)

        return s_next



# -----------------------------------------------
# Main - copied from planner.py from pddl_parser
# -----------------------------------------------
if __name__ == '__main__':
    import sys, time
    start_time = time.time()

    # Run on default domain and problem - for project
    dirname = os.path.dirname(__file__)
    domain = os.path.join(dirname,'domain.pddl')
    problem = os.path.join(dirname,'problem.pddl')
    verbose = False
    debug = False


    # Seperate parser from BFS planner
    parser = PDDL_Parser()
    parser.parse_domain(domain)
    parser.parse_problem(problem)
    # Parse initial and goal states
    s_0 = parser.state
    s_goal_pos = parser.positive_goals
    s_goal_neg = parser.negative_goals
    # Parse/ground actions
    actions = []
    for action in parser.actions:
        for act in action.groundify(parser.objects, parser.types):
            actions.append(act)




    # If arguments are given, replace problem to run on
    if len(sys.argv) > 1:
        domain = sys.argv[1]
        problem = sys.argv[2]
        verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'

    # Solve problem using action planner
    planner = BFS_Planner(s_0, s_goal_pos, s_goal_neg, actions)
    plan = planner.solve()
    print('Time: ' + str(time.time() - start_time) + 's')


    if not debug:
        if type(plan) is list:
            print('plan:')
            for act in plan:
                
                print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
                pass
        else:
            print('No plan was found')
            exit(1)

    else: 
        for i in range(50):
            print()

        for i in range(30):
            print('-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------')

        if type(plan) is list:
            print("Found plan: \n")
            state = planner.s_0

            for a in plan:
                print(f'Is the next action possible? ={planner.possible_action(a,state)}')
                print(f'State={state}')
                print(f'Action={a}')
                state = planner.act(state, a)
                print()
        else:
            print('No plan was found')
            exit(1)
        print (len(plan))




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



