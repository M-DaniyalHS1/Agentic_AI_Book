---
sidebar_position: 3
---

# Physics and Gravity

## Physics Engines in Robotics Simulation

### ODE (Open Dynamics Engine)
- Default in Gazebo Classic
- Fast, stable for simple scenarios
- Less accurate for complex contacts

### DART (Dynamic Animation and Robotics Toolkit)
- Default in Gazebo Sim
- More accurate physics
- Better for humanoid robots

### Bullet
- Used in PyBullet
- Good for manipulation tasks
- Machine learning friendly

## Configuring Physics

```xml
<physics type="dart" time_step="0.001">
  <gravity>0 0 -9.81</gravity>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Center of Mass and Stability

For humanoid robots, stability is critical:

```python
def check_stability(robot_state):
    """Check if robot is within stability margin"""
    com = robot_state.center_of_mass
    zmp = robot_state.zero_moment_point
    
    # ZMP must be within support polygon
    support_polygon = get_foot_positions(robot_state)
    
    if point_in_polygon(zmp, support_polygon):
        return True
    return False
```

## Lab Exercise 2.3

Simulate a humanoid robot:
1. Standing still (balance)
2. Walking forward
3. Recovering from a push

## Next Steps

Learn about sensor simulation.
