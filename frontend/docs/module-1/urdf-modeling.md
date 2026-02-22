---
sidebar_position: 4
---

# URDF Modeling

## What is URDF?

URDF (Unified Robot Description Format) is an XML format for describing a robot's physical properties:
- Geometry (links)
- Joints between links
- Visual, collision, and inertial properties
- Sensors and actuators

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Joints -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="leg_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
</robot>
```

## Lab Exercise 1.2

Create a simple humanoid robot URDF with:
- Base link (torso)
- Two legs
- Two arms
- Head

## Visualization

Use RViz to visualize your URDF:

```bash
rviz2
```

In RViz:
1. Add → RobotModel
2. Set Fixed Frame to `base_link`
3. Load your URDF

## Next Steps

Continue to Module 2 for simulation environments.
