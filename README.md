# ROS-Project
Grid-Based Path Planning using A* in ROS (Noetic)
A ROS 1 package that plans a collision-free path across a 2D occupancy grid using the A* search algorithm and visualizes the whole process in RViz: obstacles, the planned path, and an animated marker that travels from the start to the goal.
![Demo](docs/demo.png)
Features
A* search with a Euclidean heuristic on an 8-connected grid (straight and diagonal moves)
Diagonal moves cost √2, straight moves cost 1, so path lengths stay geometrically accurate
Default map: three full-width walls with alternating gaps, forcing a slalom path
Publishes the result as `nav_msgs/Path` and a `visualization_msgs/MarkerArray`
Animated "robot" marker that follows the path and loops back to the start
Start, goal, grid size and obstacles are configurable through ROS parameters
One launch file starts the planner and RViz (with a saved RViz config)
How it works
The environment is a 30 x 30 grid. Blocked cells are stored in a set for O(1) lookup.
A* keeps a priority queue ordered by `f = g + h`, where `g` is the cost from the start and `h` is the straight-line distance to the goal.
Each expansion checks the 8 neighbours, skipping cells that are out of bounds or blocked.
When the goal is reached, the path is rebuilt by walking the `came_from` links backwards.
The node publishes the path and markers at 10 Hz and moves the agent marker one waypoint at a time.
Package structure
```
grid_path_planner/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── path_planning.launch      # starts the planner node + RViz
├── rviz/
│   └── config.rviz               # saved RViz displays (Grid, Path, MarkerArray)
├── scripts/
│   └── astar_planner.py          # A* + ROS node
└── docs/
    └── demo.png                  # screenshot
```
Requirements
Ubuntu 20.04 (native or WSL2)
ROS 1 Noetic
Python 3 (comes with Noetic)
Installation
```bash
# 1. Create a catkin workspace (skip if you already have one)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. Clone this repository
git clone https://github.com/<your-username>/grid_path_planner.git

# 3. Make the script executable and build
chmod +x grid_path_planner/scripts/astar_planner.py
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Usage
```bash
roslaunch grid_path_planner path_planning.launch
```
RViz opens with the grid, obstacles (grey), start (blue), goal (red), the planned path (green) and the moving agent (orange).
To run only the planner without RViz:
```bash
roscore &
rosrun grid_path_planner astar_planner.py
```
Parameters
Set in `launch/path_planning.launch`:
Parameter	Default	Description
`grid_width`	`30`	Number of cells along x
`grid_height`	`30`	Number of cells along y
`resolution`	`1.0`	Size of one cell in metres
`start`	`[0, 0]`	Start cell `[x, y]`
`goal`	`[29, 29]`	Goal cell `[x, y]`
`obstacles`	slalom walls	List of blocked cells, e.g. `[[5, 5], [5, 6]]`
Published topics
Topic	Type	Content
`/planned_path`	`nav_msgs/Path`	Waypoints from start to goal
`/planning_markers`	`visualization_msgs/MarkerArray`	Obstacles, path line, start, goal, moving agent
Tuning the animation
In `scripts/astar_planner.py`, `ticks_per_step` controls the agent speed (lower is faster) and `pause_ticks` controls how long it waits at the goal before looping.
Troubleshooting (WSL)
RViz opens then closes: run `export LIBGL_ALWAYS_SOFTWARE=1` before launching.
Package not found: run `source ~/catkin_ws/devel/setup.bash` in every new terminal.
Grid and path look misaligned: the grid display needs an offset of `15; 15; 0`. This is already saved in `rviz/config.rviz`.
Possible extensions
Dynamic replanning when obstacles change
Comparison with Dijkstra, Greedy Best-First and RRT
Loading the map from a `nav_msgs/OccupancyGrid` or a `.pgm` map file
Author
Kedar S Ajai, B.Tech Robotics and Automation, Adi Shankara Institute of Engineering and Technology, Kalady (APJ Abdul Kalam Technological University).
License
MIT. See LICENSE.
