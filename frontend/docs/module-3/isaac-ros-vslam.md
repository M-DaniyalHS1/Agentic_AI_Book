---
sidebar_position: 3
---

# Isaac ROS VSLAM

## What is VSLAM?

Visual Simultaneous Localization and Mapping (VSLAM) enables robots to:
- Build a map of unknown environments
- Track their position within the map
- Use only camera inputs

## Isaac ROS GEMs

NVIDIA provides GPU-accelerated ROS 2 packages (GEMs):

```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-*
```

### Key Packages

| Package | Purpose |
|---------|---------|
| `isaac_ros_vslam` | Visual SLAM |
| `isaac_ros_depth_image_proc` | Depth processing |
| `isaac_ros_tensor_rt` | TensorRT inference |
| `isaac_ros_dnn_inference` | DNN models |

## VSLAM Node Configuration

```yaml
/isaac_ros_vslam:
  ros__parameters:
    use_sim_time: false
    
    # Camera calibration
    calibration_file: "/path/to/camera.yaml"
    
    # SLAM parameters
    slam_type: 2  # STEREO_SLAM
    enable_localization: true
    enable_mapping: true
    
    # Optimization
    optimization_mode: 1  # ACCURACY
    max_keyframes: 100
```

## Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_vslam',
            executable='isaac_ros_vslam_node',
            name='isaac_ros_vslam',
            parameters=[{'calibration_file': '/path/to/calib.yaml'}],
            remappings=[
                ('left/image_rect', '/stereo/left/image_rect'),
                ('right/image_rect', '/stereo/right/image_rect'),
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'vslam.rviz']
        )
    ])
```

## Lab Exercise 3.3

Implement VSLAM for a mobile robot:
1. Set up stereo cameras
2. Configure Isaac ROS VSLAM
3. Navigate unknown environment
4. Build and visualize map

## Next Steps

Learn about Nav2 for autonomous navigation.
