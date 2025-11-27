from cbs_mapf.planner import Planner

GRID_SIZE = 30  # must be >= largest coordinate
ROBOT_RADIUS = 10

static_obstacles = [(3, 3), (5, 10)]
starts  = [(331, 165), (1530, 165), (315, 951), (1530, 951)]
goals   = [(540, 565), (835, 565), (1080, 565), (1370, 565)]

planner = Planner(grid_size=GRID_SIZE, robot_radius=ROBOT_RADIUS, static_obstacles=static_obstacles)
paths = planner.plan(starts, goals)
print(paths.shape)
