




class Node:
    def __init__(self, node, parent=None):
        self.node = node
        self.parent = parent

    def extract_path(self):
        path = []
        node_i = self.node
        while node_i is not None:
            path.append(node_i)
            node_i = node_i.parent
        return node_i


def distance(node1, node2):
    out=0
    for i in range(len(node1)):
        out += (node1[i]-node2[i])**2
    return out**0.5

def rand_node():


def rrt (start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False, goal_probability=.2, max_iterations=100000, max_time=float("inf")):


def solver (start, goal_sample):
    return rrt(start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn)