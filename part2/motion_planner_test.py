from motion_planners_caelan.rrt import rrt


def wrap_sample_fn(sample_fn):
    samples = []

    def new_sample_fn(*args, **kwargs):
        q = sample_fn(*args, **kwargs)
        samples.append(q)
        return q

    return new_sample_fn, samples


def get_sample_fn(region, obstacles=[], only_cfree=True, **kwargs): #, check_collisions=False):
    # TODO: additional rejection function
    # TODO: Gaussian sampling for narrow passages
    collision_fn = get_collision_fn(region, obstacles)
    lower, upper = region
    generator = interval_generator(lower, upper, **kwargs)

    def region_gen():
        #area = np.product(upper - lower) # TODO: sample_fn proportional to area
        for q in generator:
            #q = sample_box(region)
            if only_cfree and collision_fn(q):
                continue
            return q # TODO: sampling with state (e.g. deterministic sampling)

    return region_gen



##############################################################


def wrap_extend_fn(extend_fn):
    roadmap = []

    def new_extend_fn(q1, q2, *args, **kwargs):
        raise NotImplementedError()

    return new_extend_fn, roadmap


def get_extend_fn(circular={}, step_size=STEP_SIZE, norm=INF):
    #difference_fn = get_difference
    difference_fn = get_difference_fn(circular=circular)
    def fn(q1, q2):
        # steps = int(np.max(np.abs(np.divide(difference_fn(q2, q1), resolutions))))
        # steps = int(np.linalg.norm(np.divide(difference_fn(q2, q1), resolutions), ord=norm))
        steps = int(np.linalg.norm(np.array(difference_fn(q2, q1)) / step_size, ord=norm))
        num_steps = steps + 1
        q = q1
        for i in range(num_steps):
            q = (1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q
            q = [wrap_interval(v, circular.get(i, UNBOUNDED_LIMITS)) for i, v in enumerate(q)]
            q = np.array(q) # tuple
            yield q
    return fn


def get_wrapped_extend_fn(environment, obstacles=[], **kwargs):
    collision_fn = get_collision_fn(environment, obstacles)
    roadmap = []

    def extend_fn(q1, q2):
        path = [q1]
        for q in sample_line(segment=(q1, q2), **kwargs):
            yield q
            if collision_fn(q):
                path = None
            if path is not None:
                roadmap.append((path[-1], q))
                path.append(q)

    return extend_fn, roadmap



####################################################


def wrap_collision_fn(collision_fn):
    colliding = []
    cfree = []
    # TODO: KDTree for hyperspheres
    # TODO: Signed Distance Function (SDF)

    def new_collision_fn(q, *args, **kwargs):
        result = collision_fn(q, *args, **kwargs)
        if result:
            colliding.append(q)
        else:
            cfree.append(q)
        return result

    return new_collision_fn, colliding, cfree


def get_collision_fn(environment, obstacles):

    def collision_fn(q):
        #time.sleep(1e-3)
        if not contains(q, environment):
            return True
        if point_collides(q, obstacles):
            return True
        return False

    return collision_fn





if __name__ == '__main__':
    
    # seed = args.seed
    # if seed is None:
    #     #seed = random.randint(0, sys.maxsize)
    #     seed = random.randint(0, 10**3-1)
    # print('Seed:', seed)
    # random.seed(seed)
    # np.random.seed(seed)

    # #########################

    # start, goal, regions, obstacles = problem1()
    # #obstacles = []
    # environment = regions['env']
    # if isinstance(goal, str) and (goal in regions):
    #     goal = get_box_center(regions[goal])
    # else:
    #     goal = np.array([1., 1.])

    # title = args.algorithm
    # if args.smooth:
    #     title += '+shortcut'

    
    
    rrt(start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
        goal_probability=.2, max_iterations=RRT_ITERATIONS, max_time=INF)
    
    #print("test")