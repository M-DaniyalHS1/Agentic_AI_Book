---
sidebar_position: 3
---

# Python Control with rclpy

## Overview

`rclpy` is the official ROS 2 client library for Python. It allows you to create ROS 2 nodes, publishers, subscribers, services, and actions using Python.

## Installation

```bash
# On Ubuntu with ROS 2 installed
source /opt/ros/humble/setup.bash
pip3 install rclpy

# Verify installation
python3 -c "import rclpy; print(rclpy.__version__)"
```

## Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node started')
    
    def spin_once(self):
        # Do something periodically
        pass

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
```

## Lab Exercise 1.1

Create a node that publishes a message every second.

### Solution

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
```

## Next Steps

Learn about URDF modeling for robot descriptions.
