---
sidebar_position: 1
---

# Gazebo Simulation

## Overview

Gazebo is the most widely used open-source robotics simulator. It provides:
- Accurate physics simulation (ODE, Bullet, DART)
- High-quality 3D graphics
- Sensor simulation
- ROS 2 integration

## Installation

```bash
# Ubuntu
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify
gz sim --version
```

## Starting a Simulation

```bash
# Empty world
gz sim empty.sdf

# With ROS 2 bridge
gz sim -r empty.sdf
```

## Spawning a Robot

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class SpawnRobot(Node):
    def __init__(self):
        super().__init__('spawn_robot')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
    
    async def spawn(self, urdf_path, x=0.0, y=0.0, z=0.0):
        req = SpawnEntity.Request()
        req.name = 'humanoid'
        req.xml = open(urdf_path).read()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        
        future = self.client.call_async(req)
        await future
        return future.result()

async def main():
    rclpy.init()
    node = SpawnRobot()
    result = await node.spawn('humanoid.urdf')
    print(f'Spawned: {result}')
```

## Lab Exercise 2.1

Create a Gazebo world with:
- A humanoid robot
- A table
- Some objects to manipulate

## Next Steps

Learn about Unity simulation for higher fidelity graphics.
