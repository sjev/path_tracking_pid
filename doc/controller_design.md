# Path Tracking PID Controller Design

## Table of Contents
1. [Introduction](#introduction)
2. [Design Philosophy](#design-philosophy)
3. [Core Architecture](#core-architecture)
4. [Mathematical Foundation](#mathematical-foundation)
5. [Control Modes](#control-modes)
6. [Robot Models](#robot-models)
7. [Advanced Features](#advanced-features)
8. [Implementation Details](#implementation-details)
9. [Tuning Guidelines](#tuning-guidelines)

## Introduction

This document describes the design and implementation of a sophisticated path tracking controller for mobile robots. Unlike classic **pure pursuit controllers that use constant velocity**, this controller implements a **dual-loop PID architecture with dynamic velocity regulation** that provides robust path following capabilities for both differential drive and tricycle steering robots.

### Key Features
- **Dynamic velocity control** with acceleration limiting (vs constant velocity in pure pursuit)
- Dual-loop control (lateral + angular) for independent tuning
- Multiple tracking modes (carrot-following vs base-link)
- Model Predictive Control (MPC) for velocity regulation
- Support for differential drive and tricycle steering models
- Anti-collision integration with costmaps
- Smooth goal approach with automatic deceleration
- Dynamic reconfiguration for real-time tuning

## Design Philosophy

### Why Dual-Loop Control?

The controller separates path tracking into two independent control problems:

1. **Lateral Control**: Minimizes perpendicular distance to the path
2. **Angular Control**: Aligns robot heading with path direction

This separation provides several advantages:
- **Decoupled tuning**: Each loop can be tuned independently
- **Better stability**: Avoids coupling between position and orientation errors
- **Flexible behavior**: Different tracking strategies for different scenarios

### Design Rationale

```python
# Classic Pure Pursuit approach (constant velocity)
def pure_pursuit_control(robot_pose, path, lookahead_distance):
    target_point = find_lookahead_point(robot_pose, path, lookahead_distance)
    curvature = compute_curvature(robot_pose, target_point, lookahead_distance)
    
    # Constant velocity (simple but limited)
    linear_vel = CONSTANT_VELOCITY  # e.g., 1.0 m/s
    angular_vel = curvature * linear_vel
    return linear_vel, angular_vel

# Our dual-loop approach with dynamic velocity (superior)
def dual_loop_control(lateral_error, angular_error, current_pose, path):
    # Independent control loops for steering
    lateral_effort = pid_lateral.compute(lateral_error)
    angular_effort = pid_angular.compute(angular_error)
    
    # Dynamic velocity control (key difference from pure pursuit)
    forward_velocity = velocity_controller.compute(current_pose, path)
    
    # Combine based on robot kinematics
    return combine_control_efforts(forward_velocity, lateral_effort, angular_effort)
```

## Core Architecture

### System Overview

```
Global Plan → Path Processing → Dual-Loop Controller → Robot Commands
     ↓              ↓                    ↓                   ↓
   Waypoints    Pose Projection    PID Control         Velocity/Steering
```

### Main Components

1. **Path Processor** (`calculations.cpp`): Computes path geometry
2. **Pose Tracker** (`controller.cpp:237-330`): Finds closest point on path
3. **Dual PID Controller** (`controller.cpp:357-616`): Generates control efforts
4. **Kinematic Converter** (`controller.cpp:641-690`): Maps to robot commands
5. **Velocity Regulator** (`controller.cpp:417-533`): Manages forward speed

### Control Flow Pseudocode

```python
def control_loop(current_pose, global_plan, odometry, dt):
    # 1. Path Processing
    reference_pose = find_pose_on_plan(current_pose, global_plan)

    # 2. Error Calculation
    if config.track_base_link:
        control_pose = current_pose
    else:  # Carrot following
        control_pose = get_control_point_pose(current_pose, config.l)

    error = control_pose.inverse() * reference_pose
    lateral_error = error.y  # Perpendicular distance to path
    angular_error = error.theta  # Heading misalignment

    # 3. PID Control
    lateral_effort = pid_lateral.compute(lateral_error, dt)
    angular_effort = pid_angular.compute(angular_error, dt)

    # 4. Forward Velocity Control
    forward_velocity = velocity_controller(current_pose, reference_pose, dt)

    # 5. Command Generation
    if robot_type == "differential_drive":
        cmd_vel = generate_diff_drive_cmd(forward_velocity, lateral_effort, angular_effort)
    elif robot_type == "tricycle":
        cmd_vel = generate_tricycle_cmd(forward_velocity, lateral_effort, angular_effort)

    return cmd_vel
```

## Mathematical Foundation

### Error Coordinate System

The controller uses a **path-relative coordinate system** where:
- **x-axis**: Along the path (tangential)
- **y-axis**: Perpendicular to the path (lateral)
- **θ**: Angular deviation from path direction

```python
def compute_path_relative_error(robot_pose, path_segment):
    """
    Transform robot pose into path-relative coordinates
    """
    # Path frame orientation
    path_direction = atan2(
        path_segment.end.y - path_segment.start.y,
        path_segment.end.x - path_segment.start.x
    )

    # Create path frame transform
    path_frame = Transform(path_segment.start, path_direction)

    # Express robot pose in path frame
    error = path_frame.inverse() * robot_pose

    return {
        'lateral': error.y,      # Cross-track error
        'angular': error.theta   # Heading error
    }
```

### PID Control Laws

#### Lateral Control
```python
def lateral_pid_control(lateral_error, dt):
    # Filter error to reduce noise
    filtered_error = lowpass_filter.update(lateral_error, dt)

    # PID computation
    proportional = config.Kp_lat * filtered_error
    integral = config.Ki_lat * integral_filter.update(filtered_error, dt)
    derivative = config.Kd_lat * derivative_filter.update(filtered_error, dt)

    control_effort = proportional + integral + derivative
    return clamp(control_effort, -100.0, 100.0)
```

#### Angular Control
```python
def angular_pid_control(angular_error, dt):
    # Normalize angle to [-π, π]
    normalized_error = normalize_angle(angular_error)
    filtered_error = lowpass_filter.update(normalized_error, dt)

    # PID computation with feedforward
    proportional = config.Kp_ang * filtered_error
    integral = config.Ki_ang * integral_filter.update(filtered_error, dt)
    derivative = config.Kd_ang * derivative_filter.update(filtered_error, dt)

    # Feedforward term based on path curvature
    if config.feedforward_ang:
        feedforward = turning_radius_inverse * forward_velocity
    else:
        feedforward = 0.0

    control_effort = proportional + integral + derivative + feedforward
    return clamp(control_effort, -100.0, 100.0)
```

### Forward Velocity Control

**Key Difference from Pure Pursuit**: Unlike classic pure pursuit controllers that use constant velocity, this controller implements **dynamic velocity regulation** with multiple sophisticated features:

The forward velocity controller implements **acceleration-limited tracking** with end-phase detection:

```python
def forward_velocity_control(current_vel, target_vel, distance_to_goal, dt):
    # End-phase detection: start decelerating when close to goal
    deceleration_distance = compute_stopping_distance(current_vel, target_end_vel)

    if distance_to_goal <= deceleration_distance:
        target_vel = target_end_vel  # Switch to end velocity

    # Acceleration limiting
    vel_error = target_vel - current_vel
    if target_vel > current_vel:
        max_accel = config.target_x_acc
    else:
        max_accel = config.target_x_decc

    accel = clamp(vel_error / dt, -max_accel, max_accel)
    new_velocity = current_vel + accel * dt

    # Minimum velocity to ensure goal reaching
    if distance_to_goal > 0 and abs(new_velocity) < config.abs_minimum_x_vel:
        new_velocity = copysign(config.abs_minimum_x_vel, target_vel)

    return new_velocity

# Comparison with Classic Pure Pursuit:
def pure_pursuit_velocity():
    return CONSTANT_VELOCITY  # Simple but lacks smoothness and safety
```

## Comparison with Pure Pursuit Controllers

### Key Differences

| Aspect | Pure Pursuit | This PID Controller |
|--------|-------------|-------------------|
| **Velocity Control** | Constant velocity | Dynamic with acceleration limiting |
| **Goal Approach** | Abrupt stop or external logic | Smooth deceleration with end-phase detection |
| **Safety** | External collision avoidance needed | Integrated obstacle-aware velocity scaling |
| **Smoothness** | Depends on path quality | Built-in acceleration/deceleration limits |
| **Tuning Complexity** | Simple (lookahead + velocity) | More complex but more capable |
| **Real-world Deployment** | Often needs additional layers | Production-ready with safety features |

### Velocity Control Comparison

```python
# Pure Pursuit: Simple but limited
def pure_pursuit_velocity_control():
    return CONSTANT_VELOCITY  # e.g., always 1.0 m/s

# This Controller: Sophisticated and safe
def pid_velocity_control(current_vel, target_vel, distance_to_goal, obstacles):
    # 1. Acceleration limiting for smooth motion
    if target_vel > current_vel:
        accel = min(config.target_x_acc, (target_vel - current_vel) / dt)
    else:
        accel = max(-config.target_x_decc, (target_vel - current_vel) / dt)
    
    # 2. End-phase deceleration
    stopping_distance = compute_stopping_distance(current_vel, target_end_vel)
    if distance_to_goal <= stopping_distance:
        target_vel = target_end_vel
    
    # 3. Obstacle-aware velocity scaling
    if obstacles_detected:
        target_vel *= safety_scale_factor
    
    # 4. MPC velocity regulation (optional)
    if config.use_mpc:
        target_vel = min(target_vel, mpc_velocity_limit)
    
    return current_vel + accel * dt
```

### When to Use Each Approach

**Pure Pursuit is suitable for**:
- Research and simulation environments
- Simple scenarios with external velocity planning
- Cases where simplicity is prioritized over performance

**This PID Controller is suitable for**:
- Real-world robot deployments
- Production systems requiring safety and smoothness
- Complex environments with obstacles
- Applications requiring precise goal approach behavior

### Configuring for Pure Pursuit-like Behavior

If you prefer constant velocity behavior similar to pure pursuit:

```yaml
# Near-instant acceleration (mimics constant velocity)
target_x_acc: 1000.0
target_x_decc: 1000.0

# Same cruise and end velocity
target_x_vel: 1.0
target_end_x_vel: 1.0

# Disable advanced features
use_mpc: false
anti_collision: false

# Pure lateral control (similar to pure pursuit)
feedback_lat: true
feedback_ang: false
feedforward_ang: true  # Use path curvature like pure pursuit
```

## Control Modes

### Carrot-Following Mode (Default)

**Concept**: Track a point ahead on the path at distance `l` from the robot.

```python
def carrot_following_control(robot_pose, global_plan, carrot_distance):
    # Compute carrot position
    carrot_pose = get_control_point_pose(robot_pose, carrot_distance)

    # Find reference on path for carrot
    reference_pose = find_pose_on_plan(carrot_pose, global_plan)

    # Compute error between carrot and reference
    error = carrot_pose.inverse() * reference_pose

    return error
```

**Advantages**:
- Stable following of straight paths
- Predictive behavior around curves
- Smooth motion characteristics

**Parameters**:
- `l > 0`: Forward carrot (standard operation)
- `l < 0`: Backward carrot (reverse driving)

### Base-Link Tracking Mode

**Concept**: Directly track the robot's base_link position on the path.

```python
def base_link_tracking_control(robot_pose, global_plan):
    # Find closest point on path to robot
    reference_pose = find_pose_on_plan(robot_pose, global_plan)

    # Add carrot offset to reference (lookahead for control)
    reference_with_carrot = get_control_point_pose(reference_pose, config.l)

    # Compute error
    error = robot_pose.inverse() * reference_with_carrot

    return error
```

**Advantages**:
- Precise path following
- Better for narrow passages
- Reduced steady-state error

**Use Cases**:
- High precision requirements
- Docking maneuvers
- Confined spaces

## Robot Models

### Differential Drive Model

**Kinematics**: The robot has two independently driven wheels.

```python
def differential_drive_command(forward_vel, lateral_effort, angular_effort):
    # For non-holonomic robots, lateral control affects angular velocity
    cmd_vel = Twist()
    cmd_vel.linear.x = forward_vel
    cmd_vel.linear.y = 0  # Non-holonomic constraint

    # Combine lateral and angular control
    # Lateral effort creates rotation to reduce cross-track error
    cmd_vel.angular.z = copysign(1.0, config.l) * lateral_effort + angular_effort

    # Apply velocity limits
    cmd_vel.angular.z = clamp(cmd_vel.angular.z, -config.max_yaw_vel, config.max_yaw_vel)

    # Respect minimum turning radius
    if config.min_turning_radius > 0:
        max_angular_vel = abs(forward_vel) / config.min_turning_radius
        cmd_vel.angular.z = clamp(cmd_vel.angular.z, -max_angular_vel, max_angular_vel)

    return cmd_vel
```

### Tricycle Steering Model

**Kinematics**: The robot has a steered front wheel and driven rear wheels.

```python
def tricycle_model_control(forward_vel, lateral_effort, angular_effort):
    # First compute desired base_link velocity
    base_cmd = differential_drive_command(forward_vel, lateral_effort, angular_effort)

    # Convert to steering commands using inverse kinematics
    steering_cmd = inverse_kinematics(base_cmd, wheel_base, wheel_offset)

    # Apply steering constraints
    steering_cmd.angle = clamp(steering_cmd.angle, -max_steering_angle, max_steering_angle)
    steering_cmd.speed = clamp(steering_cmd.speed, -max_speed, max_speed)

    # Apply acceleration limits
    steering_cmd = apply_steering_limits(steering_cmd, previous_cmd, dt)

    # Convert back to base_link commands for consistency
    return forward_kinematics(steering_cmd, wheel_base, wheel_offset)

def inverse_kinematics(cmd_vel, wheel_base, wheel_offset):
    """Convert base_link velocity to steering wheel commands"""
    # Kinematic transformation matrix (computed during initialization)
    steering_vel = inverse_matrix @ [cmd_vel.linear.x, cmd_vel.angular.z]

    return {
        'speed': hypot(steering_vel[0], steering_vel[1]),
        'angle': atan2(steering_vel[1], steering_vel[0])
    }
```

## Advanced Features

### Model Predictive Control (MPC)

**Purpose**: Dynamically adjust velocity to maintain lateral error bounds over a prediction horizon.

```python
def mpc_velocity_regulation(target_velocity, current_pose, odom):
    """
    Use MPC to find maximum safe velocity that keeps lateral error bounded
    """
    max_velocity = target_velocity
    velocity_candidates = [target_velocity]

    # Bisection search for optimal velocity
    for iteration in range(max_optimization_iterations):
        # Simulate forward in time
        predicted_pose = current_pose
        predicted_twist = odom
        max_lateral_error = 0

        for step in range(max_prediction_steps):
            # Run controller with candidate velocity
            result = controller.update(
                candidate_velocity, target_end_velocity,
                predicted_pose, predicted_twist, simulation_dt
            )

            # Update plant model
            predicted_pose = simulate_robot_motion(
                predicted_pose, result.velocity_command, simulation_dt
            )
            predicted_twist = result.velocity_command

            # Track maximum lateral error
            max_lateral_error = max(max_lateral_error, abs(controller.tracking_error_lat))

            # Early termination if error exceeds bounds
            if max_lateral_error > config.mpc_max_error_lat:
                break

        # Adjust velocity based on constraint satisfaction
        if max_lateral_error <= config.mpc_max_error_lat:
            # Can increase velocity
            max_velocity = candidate_velocity
            candidate_velocity = (candidate_velocity + target_velocity) / 2
        else:
            # Must decrease velocity
            candidate_velocity = (candidate_velocity + max_velocity) / 2

    return abs(max_velocity)
```

### Feedforward Control

**Angular Feedforward**: Uses precomputed path curvature to anticipate required angular velocity.

```python
def compute_feedforward_angular_velocity(path_segment_index, forward_velocity):
    """
    Feedforward angular velocity based on path curvature
    """
    curvature = turning_radius_inverse[path_segment_index]
    feedforward_angular_vel = curvature * forward_velocity

    return feedforward_angular_vel

def precompute_path_curvature(global_plan):
    """
    Precompute inverse turning radius for each path segment
    """
    deltas = deltas_of_plan(global_plan)
    curvatures = []

    for delta in deltas:
        dx, dy = delta.translation.x, delta.translation.y
        distance_squared = dx*dx + dy*dy

        if distance_squared < epsilon:
            curvature = infinity  # Straight line
        else:
            curvature = (2 * dy) / distance_squared  # Inverse radius

        curvatures.append(curvature)

    return curvatures
```

### Anti-Collision Integration

**Velocity Scaling**: Reduce velocity based on costmap obstacles.

```python
def compute_collision_velocity_limit(robot_pose, forward_velocity, costmap):
    """
    Project robot footprint along predicted trajectory and find velocity limit
    """
    # Generate prediction steps
    prediction_steps = []
    for i in range(num_prediction_steps):
        step_time = i * prediction_dt
        predicted_pose = simulate_motion(robot_pose, forward_velocity, step_time)
        prediction_steps.append(predicted_pose)

    # Project footprint along trajectory
    projected_footprint = project_footprint(robot_footprint, prediction_steps)

    # Find maximum cost in projected area
    max_cost = 0
    for point in projected_footprint:
        cost = costmap.get_cost(point.x, point.y)
        max_cost = max(max_cost, cost)

    # Scale velocity based on cost
    if max_cost >= lethal_cost:
        return 0.0  # Full stop
    elif max_cost >= inscribed_cost:
        # Linear scaling between inscribed and lethal cost
        scale_factor = (lethal_cost - max_cost) / (lethal_cost - inscribed_cost)
        return forward_velocity * scale_factor
    else:
        return forward_velocity  # No reduction needed
```

## Implementation Details

### State Management

The controller maintains internal state for:
- **Current velocities**: For acceleration limiting
- **Filter states**: For PID derivative and integral terms
- **Path tracking**: Current position along global plan
- **End phase detection**: To trigger goal approach behavior

```python
class ControllerState:
    def __init__(self):
        # Velocity state
        self.current_x_vel = 0.0
        self.current_yaw_vel = 0.0

        # Path tracking
        self.current_global_plan_index = 0
        self.end_phase_enabled = False
        self.end_reached = False

        # PID filter states
        self.error_lat_filter = SecondOrderLowpass()
        self.error_ang_filter = SecondOrderLowpass()
        self.error_integral_lat = Integral()
        self.error_integral_ang = Integral()
        self.error_deriv_lat = Derivative()
        self.error_deriv_ang = Derivative()

        # Tracking error (for diagnostics)
        self.tracking_error_lat = 0.0
        self.tracking_error_ang = 0.0
```

### Numerical Stability

**Key considerations**:
1. **Angle normalization**: All angular errors normalized to [-π, π]
2. **Integral windup protection**: Clamp integral terms to prevent windup
3. **Velocity limiting**: Respect physical constraints
4. **Smooth transitions**: Acceleration limiting prevents jerky motion

### Error Handling

**Robustness features**:
- **Plan validation**: Reject plans with non-monotonic waypoints
- **Large error detection**: Reset state when tracking error is excessive
- **Timeout protection**: Disable controller if no updates received
- **Graceful degradation**: Reduce performance rather than fail completely

## Tuning Guidelines

### Basic PID Tuning

#### Lateral Control
1. **Start with proportional gain** (`Kp_lat`):
   ```
   Kp_lat = 1.0  # Begin with unit gain
   Ki_lat = 0.0  # Disable integral initially
   Kd_lat = 0.0  # Disable derivative initially
   ```

2. **Increase Kp until oscillation**, then reduce by 50%
3. **Add derivative term** (`Kd_lat`) to dampen oscillations
4. **Add small integral term** (`Ki_lat`) to eliminate steady-state error

#### Angular Control
Similar process, but typically requires higher gains:
```
Kp_ang = 2.0  # Start higher than lateral
```

### Advanced Parameter Tuning

#### Carrot Distance (`l`)
- **Straight paths**: Larger `l` (1.0-2.0m) for stability
- **Curved paths**: Smaller `l` (0.3-0.8m) for precision
- **Rule of thumb**: `l ≈ 2 × robot_length`

#### Velocity Parameters
```yaml
target_x_vel: 1.0      # Maximum forward velocity
target_x_acc: 0.5      # Acceleration limit
target_x_decc: 1.0     # Deceleration limit (can be higher)
target_end_x_vel: 0.0  # Final goal velocity
```

#### Filter Parameters
```yaml
lowpass_cutoff: 2.0    # Cutoff frequency (Hz)
lowpass_damping: 1.0   # Damping ratio
```

### Robot-Specific Tuning

#### Differential Drive
- Focus on `min_turning_radius` for kinematic constraints
- Tune `max_yaw_vel` based on wheel separation

#### Tricycle Steering
- Configure `max_steering_angle` based on physical limits
- Tune `max_steering_yaw_vel` for smooth steering motion
- Set `wheel_base` and `wheel_offset` accurately

### Debugging and Diagnostics

**Key debug topics**:
- `/debug`: PID terms and internal state
- `/feedback`: High-level controller status
- `/visualization_path`: Current path being followed

**Common issues**:
1. **Oscillation**: Reduce proportional gain or add derivative term
2. **Steady-state error**: Add small integral term
3. **Sluggish response**: Increase proportional gain
4. **Overshoot**: Add derivative term or reduce gains

This comprehensive design document provides the foundation for understanding, implementing, and tuning the path tracking PID controller for robust mobile robot navigation.
