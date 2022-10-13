# Looking at the folllowing repo:
# https://github.com/pucrs-automated-planning/pddl-parser

from .PDDL import pddl_parser
from pddl_parser.action import Action


class Planner():
        
    def solve(self, domain, problem):

        # Initalizing up the basic parser
        parser = pddl_parser()
        parser.parse_domain(domain)
        parser.parse_problem(problem)

        s = parser.state
        g = parser.positive_goals
        #goal_not = parser.negative_goals


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

