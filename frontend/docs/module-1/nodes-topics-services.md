---
sidebar_position: 2
---

# Nodes, Topics, Services, and Actions

## Topics: Publish-Subscribe Pattern

Topics are used for continuous data streams like sensor readings or control commands.

### Example: Publishing Camera Images

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher_ = self.create_publisher(Image, '/camera/raw', 10)
        self.timer = self.create_timer(0.033, self.publish_image)  # 30 Hz
    
    def publish_image(self):
        # Capture and publish image
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        # ... populate image data
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = CameraDriver()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
```

### Example: Subscribing to Topics

```python
class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/raw',
            self.image_callback,
            10
        )
    
    def image_callback(self, msg):
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}')
        # Process image...
```

## Services: Request-Response Pattern

Services are used for computations that complete once and return a result.

### Example: Computing Inverse Kinematics

```python
from humanoid_interfaces.srv import ComputeIK

class IKService(Node):
    def __init__(self):
        super().__init__('ik_service')
        self.server = self.create_service(
            ComputeIK,
            '/compute_ik',
            self.compute_ik_callback
        )
    
    def compute_ik_callback(self, request, response):
        # Compute joint angles for target pose
        response.success = True
        response.joint_angles = [0.1, -0.2, 0.3, 0.0, 0.0, 0.0]
        return response
```

## Actions: Long-Running Tasks

Actions are used for tasks that take time and provide feedback.

### Example: Navigate to Goal

```python
from humanoid_interfaces.action import NavigateToPose
from rclpy.action import ActionServer, GoalResponse, CancelResponse

class NavigationAction(Node):
    def __init__(self):
        super().__init__('navigation_action')
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
    
    async def execute_callback(self, goal):
        # Execute navigation with feedback
        feedback = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        
        for progress in self.navigate(goal.target_pose):
            feedback.current_progress = progress
            self.action_server.publish_feedback(feedback)
        
        result.success = True
        return result
```

## When to Use Each

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Topic** | Continuous streams | Sensor data, control commands |
| **Service** | Quick computations | IK solving, path planning |
| **Action** | Long-running tasks | Navigation, manipulation |

## Summary

- **Topics**: Async, many-to-many, continuous data
- **Services**: Sync, request-response, quick tasks
- **Actions**: Async with feedback, long-running tasks
