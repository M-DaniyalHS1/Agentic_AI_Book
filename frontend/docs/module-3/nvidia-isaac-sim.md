---
sidebar_position: 1
---

# NVIDIA Isaac Sim

## What is Isaac Sim?

NVIDIA Isaac Sim is a powerful robotics simulation platform built on Omniverse, providing:
- Photorealistic environments
- GPU-accelerated physics (PhysX)
- Domain randomization
- Synthetic data generation
- ROS/ROS 2 bridges

## Installation

```bash
# Requires NVIDIA GPU with RTX support
# Download from NVIDIA Developer website
# Follow installation guide for your platform
```

## Key Features

### 1. Photorealistic Rendering
- Ray-traced graphics
- Physically-based materials (MDL)
- HDRI lighting

### 2. GPU-Accelerated Physics
- PhysX 5 for rigid body dynamics
- Soft body simulation
- Fluid simulation
- Particle systems

### 3. Domain Randomization
Randomize environment parameters for robust training:
- Lighting conditions
- Textures and colors
- Object positions
- Camera parameters

## Python API Example

```python
from omni.isaac.kit import SimulationApp

# Start simulation
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicSphere

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add objects
sphere = world.scene.add(
    DynamicSphere(
        prim_path="/World/sphere",
        name="sphere",
        position=[0, 0, 1],
    )
)

# Run simulation
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

## ROS 2 Bridge

```python
from omni.isaac.ros2_bridge import ROS2Bridge

bridge = ROS2Bridge()

# Create ROS 2 publisher
camera_pub = bridge.create_publisher(
    topic_name="/camera/image_raw",
    message_type="sensor_msgs/msg/Image"
)
```

## Lab Exercise 3.1

Create an Isaac Sim scene with:
1. A humanoid robot
2. Office environment
3. Randomized lighting
4. ROS 2 bridge for camera data

## Next Steps

Learn about synthetic data generation for AI training.
