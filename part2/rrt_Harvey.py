# Write your RRT code here (this cell and below)!
# Note that you can write your code in separate .py files
# and import them here later if you prefer

import numpy as np


# Node class for building a tree
class Node:
    def __init__(self, position, parent=None):
        self._pos = position     # Position tuple (x,y) 
        self._parent = parent    # Pointer to parent node

    @property
    def pos(self):
        return self._pos 
    
    @property
    def parent(self):
        return self._parent
    
    @parent.setter
    def parent(self, node_parent):
        self._parent = node_parent

# Function to generate a random 2D position tuple (x,y) within given bounds.
# Sampling is biased towards a goal 
def sample_point(bounds, sample_number, end_region):        
    GOAL_SAMPLING_INTERVAL = 20 # Can change so it updates based on bounds size??? 
    GOAL_SAMPLING_RADIUS = 0 #3    # Can change so expands as sample_number increases???
    
    x_rand = (0,0) # Randomly generated position tuple
    
    # Sample from the goal region if interval is reached
    if sample_number % GOAL_SAMPLING_INTERVAL == 0 and sample_number>0:        
        # Sample randomly within expanded goal region
        goal_sample_poly = end_region.buffer(GOAL_SAMPLING_RADIUS, resolution=3) # Expand goal region for sampling within 
        
        bounds_poly = Polygon([(bounds[0],bounds[1]),(bounds[0],bounds[3]),(bounds[2],bounds[3]),(bounds[2],bounds[1])]) # Create polygon to represent bounds
        goal_sample_poly = goal_sample_poly.intersection(bounds_poly)         # Trim goal sampling region to within bounds
        
        # Extract the point values that define the perimeter of the goal polygon
        xx, yy = goal_sample_poly.exterior.coords.xy

        # Convert to lists and then to limits (assumes square goal region)
        xx = xx.tolist()
        yy = yy.tolist()
        
        goal_sample_x = [min(xx), max(xx)]
        goal_sample_y = [min(yy), max(yy)]
        
        # Sample randomly within expanded goal 
        x_pos_rand = np.random.uniform(goal_sample_x[0],goal_sample_x[1],1)
        y_pos_rand = np.random.uniform(goal_sample_y[0],goal_sample_y[1],1)
        x_rand = (float(x_pos_rand), float(y_pos_rand))   
                              
    # Sample from the free space uniformly
    else:
        x_pos_rand = np.random.uniform(bounds[0],bounds[2],1)
        y_pos_rand = np.random.uniform(bounds[1],bounds[3],1)
        x_rand = (float(x_pos_rand), float(y_pos_rand))
    
    return x_rand

# Takes in two (x,y) position tuples and returns euclidian distance
def euclid_dist(pos1, pos2):
    return ((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)**0.5


# Return the node in the tree that is nearest to the location x_loc ((x,y) tuple)
def nearest_neighbor(tree_nodes, x_loc):
    dist_min = float('inf')
    x_nearest = None
    
    # Search through all nodes in the tree
    for next_node in tree_nodes:
        next_dist = euclid_dist(next_node.pos, x_loc)
        
        # If this node is closer to the x_rand node than the current minimum, update the 'nearest neighbor'
        if next_dist < dist_min:
            x_nearest = next_node
            dist_min = next_dist
            
    return x_nearest

# Steer function to generate new point for tree in the direction of a point x_goal
# Uses straight line connection - currently not limited distance
def steer(x_start, x_goal):
    x_new = x_goal
    return x_goal

# Check if the straight line path between points is free/has no collisions
def path_free(x_start, x_end, radius, env):
    # Line that connects the start to the end
    line = LineString([x_start, x_end])

    # "Buffer" the line by the radius amount
    expanded_line = line.buffer(radius, resolution=3)

    # Check collisions between the expanded line and each obstacle
    for i, obs in enumerate(env.obstacles):
        # Returns false if a collision with any object is detected
        if expanded_line.intersects(obs):
            return False
    
    # No collision with any object is detected - return true
    return True

# Extract path from tree
def extract_path(tree_nodes,node_at_goal):
    path = []
    next_node = node_at_goal
    
    # Backtrack up parent chain
    while next_node != None:
        path.append(next_node.pos)
        next_node = next_node.parent
    
    # Flip path to start at start
    path.reverse()
    
    return path

# Add the radius of the robot as a buffer to the bounds
def buffer_bounds(bounds, radius):
    # Generate bounds with a buffer for the robot radius (0 won't work for all negative bounds)
    bounds_list = list(bounds)
    bounds_with_rad = []
    
    # Add or subtract the radius to/from the bounds depending on their sign
    for bound in bounds_list:
        if bound >= 0:
            bounds_with_rad.append(bound - radius)
        else:
            bounds_with_rad.append(bound + radius)
            
    return tuple(bounds_with_rad)

def display_plan(env, bounds, start, end, tree, plan, radius):
    # Plot obstacles and bounds
    ax = plot_environment(env, bounds=bounds)
    
    # Plot start 
    start_point = Point(start)
    start_ball = start_point.buffer(0.3, resolution=3)
    plot_poly(ax, start_ball,'red')

    # Plot end
    plot_poly(ax, end,'green')
    
    # Plot tree
    num_nodes_in_tree = 0
    
    for node in tree:
        # Plot line that connects the this node to parent
        try:
            line = LineString([node.pos, node.parent.pos])
            plot_line(ax, line)
        except: # Node is start node so parent is null
            pass
        
        # Update nodes in tree counter
        num_nodes_in_tree = num_nodes_in_tree+1
         
    # Plot plan
    prev_point = plan[0]
    num_nodes_in_plan = 1 # Start at 1 to include start point
    plan_length = 0 
    
    for i in range(1,len(plan)):
        # Line that connects the prev_point to next_point
        line = LineString([prev_point, plan[i]])

        # "Buffer" the line by the radius amount
        expanded_line = line.buffer(radius, resolution=3)
        
        # Draw the original line
        #plot_line(ax, line)
        # Draw the expanded line
        plot_poly(ax, expanded_line, 'magenta', alpha=0.2)
        
        # Update trackers
        num_nodes_in_plan = num_nodes_in_plan +1
        plan_length = plan_length + euclid_dist(prev_point, plan[i])
        
        # Update previous point
        prev_point = plan[i]
    
    # Title/annotate solution
    plt.title("Tree nodes: {}, Path length: {} ({} nodes)  ".format(num_nodes_in_tree,round(plan_length,2),num_nodes_in_plan))

    #show the number of nodes in the tree, the number of nodes in the solution path and the path length (you could put this in the plot title, as in the examples above)
    
# Function to use a rapidly exploring random tree for path planning
def rrt(bounds, environment, start_pose, radius, end_region):
    node_initial = Node(start_pose,None)
    tree_nodes = [node_initial] # MIGHT HAVE TO CHANGE LIST DATASTRUCTURE FOR LARGER CASE
    
    # Add the radius of the robot as a buffer on the bounds
    bounds_with_rad = buffer_bounds(bounds, radius)
    
    N = int(1e6) # Maximum number of samples to take
    
    # Sample until desired number of samples reached
    for sample_num in range(N):
        # Generate new random sample point
        x_rand = sample_point(bounds_with_rad, sample_num, end_region)
        
        # Find closest neighbor already in tree
        x_nearest = nearest_neighbor(tree_nodes, x_rand)
        
        # Create new point in direction of x_rand
        x_new = steer(x_nearest, x_rand)
        
        #print(x_new)
        
        # If new point is in free space and no collision on path, add point to tree
        if path_free(x_nearest.pos, x_new, radius, environment):
            tree_new_node = Node(x_new, x_nearest)
            tree_nodes.append(tree_new_node)
            
            # Return path and output tree and graph if at goal
            if end_region.contains(Point(x_new)):
                path = extract_path(tree_nodes, tree_new_node)
                
                # Display output as a pretty graph
                display_plan(environment, bounds, start_pose, end_region, tree_nodes, path, radius)
                
                return path
     
    # Failed to find valid path
    return None
