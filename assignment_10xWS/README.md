# Custom DWA Planner for TurtleBot3

A simple implementation of the Dynamic Window Approach (DWA) algorithm for TurtleBot3 navigation in ROS2.

## What is DWA?

DWA (Dynamic Window Approach) is a local path planning algorithm that:
- Generates multiple trajectory predictions based on velocity samples
- Evaluates each trajectory using a cost function (goal distance, obstacle clearance, speed)
- Selects the best velocity command to reach the goal while avoiding obstacles

## How to Use

### 1. Set TurtleBot3 Model
```bash
export TURTLEBOT3_MODEL=burger
```

### 2. Launch TurtleBot3 World in Gazebo
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 3. Build and Source the Workspace
```bash
cd ~/Documents/assignment_10xWS
colcon build
source install/setup.bash
```

### 4. Launch DWA Planner with RViz
```bash
ros2 launch custom_dwa_planner dwa_planner.launch.py
```

You can set custom goal coordinates:
```bash
ros2 launch custom_dwa_planner dwa_planner.launch.py goal_x:=3.0 goal_y:=2.0
```

### 5. Set Goal in RViz (Optional)
- Click "2D Goal Pose" button in RViz
- Click on the map to set a new goal position

## Implementation Details

**DWA Algorithm** (`custom_dwa_planner/dwa_algorithm.py`):
- Samples velocity space (linear and angular velocities)
- Predicts trajectories for each velocity pair
- Scores trajectories based on heading, distance to goal, and obstacle clearance
- Returns the best velocity command

**ROS2 Node** (`scripts/dwa_planner_node.py`):
- Subscribes to `/odom` for robot position
- Subscribes to `/scan` for laser data (obstacles)
- Publishes velocity commands to `/cmd_vel`
- Visualizes planned trajectory in RViz

## Parameters

Edit `config/dwa_params.yaml` to tune the planner:
- `max_speed`: Maximum linear velocity (default: 0.22 m/s)
- `max_yaw_rate`: Maximum angular velocity (default: 40 degrees/s)
- `robot_radius`: Robot footprint radius (default: 0.2 m)
- `goal_tolerance`: Distance to consider goal reached (default: 0.3 m)

---

**Author**: Fayas
