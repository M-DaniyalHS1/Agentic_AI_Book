---
sidebar_position: 4
---

# Nav2 Navigation

## Nav2 Architecture

Nav2 is the standard navigation stack for ROS 2:

```
┌─────────────────────────────────────────────────────┐
│                    Nav2 Stack                        │
├─────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │
│  │   Planner   │  │  Controller │  │  Recovery  │  │
│  │  (Global)   │  │  (Local)    │  │  Behaviors │  │
│  └─────────────┘  └─────────────┘  └────────────┘  │
│         │                │                │         │
│         └────────────────┴────────────────┘         │
│                          │                          │
│              ┌───────────┴───────────┐              │
│              │   Costmap 2D/3D       │              │
│              │   (Obstacle Layer)    │              │
│              └───────────────────────┘              │
└─────────────────────────────────────────────────────┘
```

## Key Components

### 1. Planner Server
- Computes global path from A to B
- Algorithms: A*, Dijkstra, RRT, Theta*

### 2. Controller Server
- Follows the path
- Avoids dynamic obstacles
- Algorithms: DWA, TEB, MPPI

### 3. Costmap
- Obstacle layers
- Inflation layers
- Static map layer

### 4. Recovery Server
- Behaviors when stuck
- Spin, backup, wait

## Configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    # MPPI Controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle"]
      
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true

costmap:
  ros__parameters:
    global_frame: "map"
    robot_base_frame: "base_link"
    update_frequency: 5.0
    publish_frequency: 2.0
    width: 100
    height: 100
    resolution: 0.05
```

## Python API

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

navigator = BasicNavigator()

# Set initial pose
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.position.z = 0.0
initial_pose.pose.orientation.w = 1.0
navigator.setInitialPose(initial_pose)

# Navigate to goal
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 5.0
goal_pose.pose.position.y = 3.0
navigator.goToPose(goal_pose)

# Wait for completion
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    print(f'Distance remaining: {feedback.distance_remaining}')

# Get result
result = navigator.getResult()
print(f'Navigation completed: {result}')
```

## Lab Exercise 3.4

Implement autonomous navigation:
1. Configure Nav2 for your robot
2. Create a map of your environment
3. Navigate to multiple waypoints
4. Handle recovery behaviors

## Module 3 Complete! 🎉

You've learned about Isaac Sim, synthetic data, VSLAM, and Nav2. Continue to Module 4 for Vision-Language-Action systems.
