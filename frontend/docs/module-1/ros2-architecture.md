---
sidebar_position: 1
---

# ROS 2 Architecture

## Overview

ROS 2 follows a **distributed architecture** where independent processes (nodes) communicate through well-defined interfaces.

## Core Components

### Nodes
A **node** is a process that performs computation. Examples:
- Sensor driver node (reads camera data)
- Planning node (computes robot trajectory)
- Control node (sends motor commands)

### Topics
**Topics** are named buses for streaming data. They follow a publish-subscribe pattern:
- Publishers send messages to topics
- Subscribers receive messages from topics
- Communication is **asynchronous** and **many-to-many**

### Services
**Services** provide synchronous request-response communication:
- Client sends a request
- Server processes and returns a response
- Used for one-off computations

### Actions
**Actions** are for long-running tasks with feedback:
- Goal → Execution → Result
- Provides continuous feedback during execution
- Can be cancelled mid-execution

## Communication Diagram

```
┌──────────────┐     Topic: /camera    ┌──────────────┐
│   Camera     │ ────────────────────→ │  Perception  │
│   (Publisher)│                        │  (Subscriber)│
└──────────────┘                        └──────────────┘
                                               │
                                               │ Service: /plan_path
                                               ↓
┌──────────────┐                        ┌──────────────┐
│    Action    │ ←──── /cmd_vel ─────── │  Navigation  │
│   (Action)   │     (Topic)            │   (Server)   │
└──────────────┘                        └──────────────┘
```

## Next Steps

Learn how to implement these patterns in Python using rclpy.
