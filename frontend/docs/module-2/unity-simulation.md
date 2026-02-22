---
sidebar_position: 2
---

# Unity Simulation

## Why Unity?

Unity provides:
- Photorealistic rendering
- Advanced lighting and shadows
- Machine Learning Agents Toolkit (ML-Agents)
- Perception package for synthetic data

## Installation

```bash
# Install Unity Hub
# Download Unity 2022 LTS
# Install ROS-TCP-Connector
git clone https://github.com/Unity-Technologies/ROS-TCP-Connector.git
```

## Basic Unity Scene Setup

1. Create a new 3D scene
2. Add a ground plane
3. Import robot URDF using URDF Importer
4. Add ROS-TCP-Endpoint

## C# Script for Robot Control

```csharp
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    public string topicName = "/cmd_vel";
    private TwistMsg velocityCommand;
    
    void Start()
    {
        velocityCommand = new TwistMsg();
    }
    
    void OnTwistReceived(TwistMsg msg)
    {
        velocityCommand = msg;
    }
    
    void FixedUpdate()
    {
        float linear = (float)velocityCommand.linear.x;
        float angular = (float)velocityCommand.angular.z;
        
        transform.Translate(Vector3.forward * linear * Time.fixedDeltaTime);
        transform.Rotate(Vector3.up * angular * Time.fixedDeltaTime);
    }
}
```

## Lab Exercise 2.2

Create a Unity scene with:
- A humanoid robot
- Lighting and materials
- Camera sensors
- ROS-TCP connection

## Next Steps

Learn about physics and collision detection.
