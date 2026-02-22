---
sidebar_position: 4
---

# Sensor Simulation

## Types of Sensors

### 1. Cameras
- RGB cameras
- Depth cameras (RGB-D)
- Stereo cameras
- Event cameras

### 2. LiDAR
- 2D scanning LiDAR
- 3D solid-state LiDAR
- Flash LiDAR

### 3. IMU (Inertial Measurement Unit)
- Accelerometer
- Gyroscope
- Magnetometer

### 4. Force/Torque Sensors
- Joint torque sensors
- Foot force sensors
- Wrist F/T sensors

## Gazebo Sensor Configuration

```xml
<link name="camera_link">
  <sensor name="camera" type="camera">
    <pose>0 0 0 0 0 0</pose>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
  </sensor>
</link>
```

## LiDAR Configuration

```xml
<sensor name="lidar" type="ray">
  <pose>0 0 0.5 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30</max>
    </range>
  </ray>
</sensor>
```

## Lab Exercise 2.4

Add sensors to your robot:
1. RGB camera for vision
2. LiDAR for navigation
3. IMU for balance
4. Force sensors in feet

## Summary

- Sensors provide perception capabilities
- Simulation allows testing without hardware
- Sensor noise can be modeled for realism

## Module 2 Complete! 🎉

You've learned about digital twins and simulation. Continue to Module 3 for AI-powered robot brains.
