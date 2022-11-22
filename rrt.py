from random import random
import time

class TreeNode(object):

    def __init__(self, config, parent=None):
        self.parent = parent
        self.config = config

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]


def configs(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.config, nodes))


def irange(start, stop=None, step=1):
    if stop is None:
        stop = start
        start = 0
    while start < stop:
        yield start
        start += step

def argmin(function, sequence):
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))]

def elapsed_time(start_time):
    return time.time() - start_time


def rrt(start, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
        goal_probability=.2, max_iterations=20, max_time=float('inf')): #goal_sample - START WITHOUT GOAL SAMPLING
    """
    :param start: Start configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    start_time = time.time()
    if collision_fn(start):
        return None
    # if not callable(goal_sample):
    #     g = goal_sample
    #     goal_sample = lambda: g
    nodes = [(start)]
    for i in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        #goal = random() < goal_probability or i == 0
        #s = goal_sample() if goal else 
        s = sample_fn()

        last = argmin(lambda n: distance_fn(n.config, s), nodes)
        for q in extend_fn(last.config, s):
            if collision_fn(q):
                break
            last = TreeNode(q, parent=last)
            nodes.append(last)
            if goal_test(last.config):
                return configs(last.retrace())
        # else:
        #     if goal:
        #         return configs(last.retrace())
    return None