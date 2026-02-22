---
sidebar_position: 3
---

# Natural Language to ROS Actions

## Action Grounding

Action grounding connects abstract language commands to concrete robot behaviors:

```
Natural Language → Semantic Parsing → Action Selection → Parameter Grounding → ROS 2 Execution
```

## Architecture

```python
class LanguageActionGrounding:
    def __init__(self):
        self.action_registry = {
            "navigate": self.navigate_action,
            "pick": self.pick_action,
            "place": self.place_action,
            "push": self.push_action,
            "open": self.open_action,
            "close": self.close_action,
        }
        self.object_detector = ObjectDetector()
        self.map_server = MapServer()
    
    def execute_command(self, command: str):
        # Parse command
        parsed = self.parse_command(command)
        
        # Select action
        action_fn = self.action_registry.get(parsed["action"])
        if not action_fn:
            raise ValueError(f"Unknown action: {parsed['action']}")
        
        # Ground parameters
        grounded_params = self.ground_parameters(parsed["parameters"])
        
        # Execute
        return action_fn(**grounded_params)
    
    def parse_command(self, command: str):
        """Use LLM to parse command into structured action"""
        # Implementation using LLM
        pass
    
    def ground_parameters(self, params: dict):
        """Ground abstract references to concrete values"""
        grounded = {}
        for key, value in params.items():
            if value.startswith("the_"):
                # Resolve object reference
                obj_type = value[4:]  # Remove "the_" prefix
                grounded[key] = self.object_detector.find_object(obj_type)
            else:
                grounded[key] = value
        return grounded
    
    def navigate_action(self, target):
        """Execute navigation to target"""
        from geometry_msgs.msg import PoseStamped
        
        goal = PoseStamped()
        goal.pose.position.x = target.x
        goal.pose.position.y = target.y
        goal.pose.position.z = target.z
        
        # Publish to Nav2
        self.nav_publisher.publish(goal)
```

## ROS 2 Action Servers

```python
from humanoid_interfaces.action import ManipulateObject
from rclpy.action import ActionClient
from rclpy.node import Node

class ManipulationCommander(Node):
    def __init__(self):
        super().__init__('manipulation_commander')
        self.action_client = ActionClient(
            self,
            ManipulateObject,
            '/manipulate_object'
        )
    
    def send_manipulation_command(self, object_name, action_type):
        """Send manipulation command via action server"""
        goal_msg = ManipulateObject.Goal()
        goal_msg.object_name = object_name
        goal_msg.action_type = action_type  # "pick", "place", "push"
        
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        return future
    
    def execute_language_command(self, parsed_command):
        """Execute a parsed language command"""
        action_type = parsed_command["action"]
        object_name = parsed_command["object"]
        
        future = self.send_manipulation_command(object_name, action_type)
        
        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected')
                return
            
            self.get_logger().info('Goal accepted')
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            return result_future
        
        return future
```

## Semantic Scene Understanding

```python
class SemanticSceneGraph:
    """Maintains a semantic understanding of the environment"""
    
    def __init__(self):
        self.objects = {}  # id -> Object
        self.relationships = []  # (subject, predicate, object)
    
    def add_object(self, obj_id, obj_type, pose, attributes):
        self.objects[obj_id] = {
            "type": obj_type,
            "pose": pose,
            "attributes": attributes,
            "affordances": self.get_affordances(obj_type)
        }
    
    def get_affordances(self, obj_type):
        """Return possible actions for an object type"""
        affordance_map = {
            "cup": ["grasp", "pour", "place"],
            "door": ["open", "close", "push", "pull"],
            "drawer": ["open", "close", "pull"],
            "ball": ["grasp", "throw", "roll"],
        }
        return affordance_map.get(obj_type, [])
    
    def query(self, description: str):
        """Find objects matching a description"""
        # "the red cup on the table"
        # Parse and match against scene graph
        pass
```

## Lab Exercise 4.3

Build a language-controlled robot:
1. Implement action grounding
2. Create ROS 2 action servers
3. Build semantic scene graph
4. Test with natural language commands

## Next Steps

Learn about multimodal perception integration.
