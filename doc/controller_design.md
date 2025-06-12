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
- **Angular feedforward** control using path curvature (lateral feedforward not implemented)
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

The controller computes error in the **carrot frame** (or base_link frame in base-link tracking mode):
- **x-axis**: Along the carrot/robot orientation (forward)
- **y-axis**: Perpendicular to carrot/robot orientation (lateral)
- **θ**: Angular deviation between carrot and reference pose

**Important**: The controller computes **two different errors**:

```python
def compute_control_error(carrot_pose, reference_pose):
    """
    Error used for PID control (carrot frame)
    """
    error = carrot_pose.inverse() * reference_pose
    return {
        'lateral': error.y,      # Lateral error for control
        'angular': error.theta   # Angular error for control
    }

def compute_tracking_error(robot_pose, path_segment):
    """
    Error used for diagnostics (path frame)
    """
    # Path frame orientation
    path_direction = atan2(
        path_segment.end.y - path_segment.start.y,
        path_segment.end.x - path_segment.start.x
    )

    # Create path frame transform
    path_frame = Transform(path_segment.start, path_direction)

    # Express robot pose in path frame
    tracking_error = -(path_frame.inverse() * robot_pose.origin)

    return {
        'lateral': tracking_error.y,      # True cross-track error
        'angular': control_error.theta    # Reused from control error
    }
```

### PID Control Laws

#### Lateral Control
```python
def lateral_pid_control(lateral_error, dt):
    # IMPORTANT: All gains (Kp, Ki, Kd) must have the same sign for stability
    assert same_sign(config.Kp_lat, config.Ki_lat, config.Kd_lat)

    # Filter error using second-order lowpass (configurable cutoff/damping)
    filtered_error = second_order_lowpass.filter(lateral_error, dt)

    # PID computation (no feedforward - not implemented)
    proportional = config.Kp_lat * filtered_error
    integral = config.Ki_lat * integral_filter.filter(filtered_error, dt)  # With windup protection
    derivative = config.Kd_lat * derivative_filter.filter(filtered_error, dt)

    # Note: Lateral feedforward is NOT implemented (always 0.0)
    control_effort = proportional + integral + derivative
    return clamp(control_effort, -100.0, 100.0)
```

#### Angular Control
```python
def angular_pid_control(angular_error, dt):
    # IMPORTANT: All gains (Kp, Ki, Kd) must have the same sign for stability
    assert same_sign(config.Kp_ang, config.Ki_ang, config.Kd_ang)

    # Normalize angle to [-π, π]
    normalized_error = normalize_angle(angular_error)
    filtered_error = second_order_lowpass.filter(normalized_error, dt)

    # PID computation with feedforward
    proportional = config.Kp_ang * filtered_error

    # ⚠️ BUG IN IMPLEMENTATION: integral incorrectly uses lateral error
    # Should be: integral_filter.filter(filtered_error, dt)
    # Actually is: integral_filter.filter(lateral_error, dt)  # WRONG!
    integral = config.Ki_ang * integral_filter.filter(filtered_error, dt)  # Corrected here
    derivative = config.Kd_ang * derivative_filter.filter(filtered_error, dt)

    # Feedforward term based on path curvature (angular only)
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
def forward_velocity_control(current_vel, target_vel, target_end_vel, distance_to_goal, dt, carrot_l):
    # End-phase detection: complex distance calculation with conservative buffer
    if target_vel > current_vel:
        time_to_end = abs((target_end_vel - current_vel) / config.target_x_acc)
    else:
        time_to_end = abs((target_end_vel - current_vel) / config.target_x_decc)

    # Trapezoidal profile distance + conservative buffer
    deceleration_distance = (current_vel + target_end_vel) * 0.5 * time_to_end + target_vel * 2.0 * dt

    # Direction check prevents end-phase when robot faces wrong way
    in_direction_of_goal = is_in_direction_of_target(current_pose, goal_position, target_vel)

    if distance_to_goal <= deceleration_distance and in_direction_of_goal:
        target_vel = target_end_vel  # Switch to end velocity

    # Acceleration limiting: use minimum of desired and configured limit
    acc_desired = (target_vel - current_vel) / dt
    if target_vel > current_vel:
        max_accel = config.target_x_acc
    else:
        max_accel = config.target_x_decc

    # Take minimum magnitude, preserve sign from configured limit
    acc_magnitude = min(abs(acc_desired), abs(max_accel))
    accel = copysign(acc_magnitude, max_accel)
    new_velocity = current_vel + accel * dt

    # Minimum velocity: sign depends on carrot direction, not target
    min_vel = copysign(config.abs_minimum_x_vel, carrot_l)
    if (end_phase_enabled and abs(target_end_vel) <= abs(min_vel) and
        abs(new_velocity) <= abs(min_vel)):
        new_velocity = min_vel

    # Goal reaching: exact velocity matching at goal
    if distance_to_goal == 0.0 or velocity_within_tolerance(new_velocity, target_end_vel):
        return target_end_vel  # Force exact end velocity

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

# This Controller: Sophisticated and safe (simplified version)
def pid_velocity_control(current_vel, target_vel, distance_to_goal, obstacles):
    # 1. End-phase detection with direction checking
    end_phase_distance = compute_trapezoidal_stopping_distance(current_vel, target_end_vel)
    if distance_to_goal <= end_phase_distance and facing_goal_direction():
        target_vel = target_end_vel

    # 2. Acceleration limiting: minimum of desired and configured
    acc_desired = (target_vel - current_vel) / dt
    max_accel = config.target_x_acc if target_vel > current_vel else config.target_x_decc
    accel = copysign(min(abs(acc_desired), abs(max_accel)), max_accel)

    # 3. Obstacle-aware velocity scaling (via update_with_limits)
    if obstacles_detected:
        target_vel = min(target_vel, obstacle_velocity_limit)

    # 4. MPC velocity regulation (optional)
    if config.use_mpc:
        target_vel = min(target_vel, mpc_velocity_limit)

    new_vel = current_vel + accel * dt

    # 5. Minimum velocity and goal reaching logic
    if goal_conditions_met():
        return target_end_vel
    elif end_phase_and_low_velocity():
        return copysign(config.abs_minimum_x_vel, carrot_direction)

    return new_vel
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

### Carrot-Following Mode (`track_base_link: false`)

**Concept**: Track a point ahead on the path at distance `l` from the robot.

```python
def carrot_following_control(robot_pose, global_plan, carrot_distance):
    # Extend global plan with carrot distance beyond goal
    extended_plan = global_plan.copy()
    carrot_extension = get_control_point_pose(global_plan[-1], carrot_distance)
    extended_plan.append(carrot_extension)

    # Compute carrot position
    carrot_pose = get_control_point_pose(robot_pose, carrot_distance)

    # Find reference on extended path for carrot
    reference_pose = find_pose_on_plan(carrot_pose, extended_plan)

    # IMPORTANT: Error always computed in carrot frame
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

### Base-Link Tracking Mode (Default: `track_base_link: true`)

**Concept**: Use robot's base_link to find path reference, but still maintain lookahead control.

```python
def base_link_tracking_control(robot_pose, global_plan, carrot_distance):
    # Find closest point on path to robot base_link (not carrot)
    reference_pose = find_pose_on_plan(robot_pose, global_plan)

    # Add carrot offset to the found reference (creates lookahead)
    reference_with_carrot = get_control_point_pose(reference_pose, carrot_distance)

    # Compute carrot position (still needed for error frame)
    carrot_pose = get_control_point_pose(robot_pose, carrot_distance)

    # IMPORTANT: Error still computed in carrot frame (not base_link frame!)
    error = carrot_pose.inverse() * reference_with_carrot

    return error
```

**Advantages**:
- More responsive to path changes (uses current robot position)
- Better for paths with sharp turns
- Reduced lag in path following

**Important Note**: Despite the name "base-link tracking", the error is **still computed in the carrot frame**, not the base_link frame. The key difference is which pose is used to find the reference on the path.

### Key Differences Summary

| Aspect | Carrot Mode | Base-Link Mode |
|--------|-------------|----------------|
| **Reference Finding** | Carrot pose → path | Robot pose → path |
| **Plan Extension** | Yes (carrot beyond goal) | No |
| **Error Frame** | Carrot frame | Carrot frame (same!) |
| **Lookahead** | Built-in via carrot | Added to reference |
| **Responsiveness** | Smooth, predictive | More responsive |

### Control Loop Configuration

The controller supports selective enabling of control loops:

```yaml
# Default configuration (lateral-only control)
feedback_lat: true      # Enable lateral PID
feedback_ang: false     # Disable angular PID
feedforward_ang: false  # Disable angular feedforward
```

**Lateral-Only Mode** (default): Only lateral PID active, similar to pure pursuit where lateral error creates angular velocity.

**Dual-Loop Mode**: Both lateral and angular PID active for more sophisticated control.

**Use Cases**:
- **Base-link mode**: High precision requirements, docking maneuvers, sharp turn following
- **Carrot mode**: Smooth path following, stable navigation on straight paths

## Robot Models

The controller supports three robot models with different kinematic constraints:

### Holonomic Robot Model

**Kinematics**: Robot can move and rotate independently in all directions (e.g., omniwheels, mecanum wheels).

```python
def holonomic_command(forward_vel, lateral_effort, angular_effort):
    # Direct control in all directions
    cmd_vel = Twist()
    cmd_vel.linear.x = forward_vel      # Forward/backward
    cmd_vel.linear.y = lateral_effort   # Direct lateral control
    cmd_vel.angular.z = angular_effort  # Direct angular control

    # Apply velocity limits
    cmd_vel.angular.z = clamp(cmd_vel.angular.z, -config.max_yaw_vel, config.max_yaw_vel)

    return cmd_vel
```

**Note**: Holonomic mode is marked as "unmaintained" in the code with potential y-direction bugs.

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
    # Sign depends on carrot direction: forward carrot (l>0) vs backward (l<0)
    cmd_vel.angular.z = copysign(1.0, config.l) * lateral_effort + angular_effort

    # Apply velocity limits
    cmd_vel.angular.z = clamp(cmd_vel.angular.z, -config.max_yaw_vel, config.max_yaw_vel)

    # Respect minimum turning radius
    if config.min_turning_radius > 0:
        max_angular_vel = abs(forward_vel) / config.min_turning_radius
        cmd_vel.angular.z = clamp(cmd_vel.angular.z, -max_angular_vel, max_angular_vel)

    # Apply yaw acceleration limiting
    yaw_acc = clamp((cmd_vel.angular.z - current_yaw_vel) / dt, -config.max_yaw_acc, config.max_yaw_acc)
    cmd_vel.angular.z = current_yaw_vel + yaw_acc * dt

    return cmd_vel
```

### Tricycle Steering Model

**Kinematics**: Robot with steered front wheel and driven rear wheels. Supports asymmetric wheel placement.

```python
def tricycle_model_control(forward_vel, lateral_effort, angular_effort):
    # 1. Compute desired base_link velocity (same as differential drive)
    base_cmd = differential_drive_command(forward_vel, lateral_effort, angular_effort)

    # 2. Convert to steering wheel commands
    steering_cmd = inverse_kinematics(base_cmd)

    # 3. Handle reverse motion (flip steering angle by π)
    if base_cmd.linear.x < 0.0 and steering_cmd.speed > 0.0:
        steering_cmd.speed = -steering_cmd.speed
        if steering_cmd.angle > 0:
            steering_cmd.angle -= π
        else:
            steering_cmd.angle += π

    # 4. Apply steering angle constraints
    steering_cmd.angle = clamp(steering_cmd.angle, -config.max_steering_angle, config.max_steering_angle)

    # 5. Apply steering velocity and acceleration limits
    steering_yaw_vel = clamp(
        (steering_cmd.angle - previous_steering_angle) / dt,
        -config.max_steering_yaw_vel, config.max_steering_yaw_vel)

    steering_angle_acc = clamp(
        (steering_yaw_vel - previous_steering_yaw_vel) / dt,
        -config.max_steering_yaw_acc, config.max_steering_yaw_acc)

    new_steering_yaw_vel = previous_steering_yaw_vel + steering_angle_acc * dt
    steering_cmd.angle = previous_steering_angle + new_steering_yaw_vel * dt

    # 6. Apply speed constraints and acceleration limiting
    steering_cmd.speed = clamp(steering_cmd.speed, -config.max_steering_x_vel, config.max_steering_x_vel)

    speed_acc = clamp(
        (steering_cmd.speed - previous_steering_speed) / dt,
        -config.max_steering_x_acc, config.max_steering_x_acc)

    steering_cmd.speed = previous_steering_speed + speed_acc * dt

    # 7. Convert back to base_link commands and update state
    return forward_kinematics(steering_cmd)

def inverse_kinematics(cmd_vel):
    """Convert base_link velocity to steered wheel velocity components"""
    # Transform using precomputed matrix based on wheel position
    x_alpha = inverse_matrix[0][0] * cmd_vel.linear.x + inverse_matrix[0][1] * cmd_vel.angular.z
    y_alpha = inverse_matrix[1][0] * cmd_vel.linear.x + inverse_matrix[1][1] * cmd_vel.angular.z

    # Convert to polar coordinates (speed and steering angle)
    return {
        'speed': hypot(x_alpha, y_alpha),
        'angle': atan2(y_alpha, x_alpha)
    }

def setup_kinematics_matrices(wheel_x, wheel_y):
    """Setup transformation matrices based on steered wheel position"""
    distance = hypot(wheel_x, wheel_y)
    wheel_theta = atan2(wheel_y, wheel_x)

    # Inverse kinematics matrix
    inverse_matrix = [
        [1, -distance * sin(wheel_theta)],
        [0, -distance * cos(wheel_theta)]
    ]

    # Forward kinematics matrix (matrix inverse)
    det = inverse_matrix[0][0] * inverse_matrix[1][1] - inverse_matrix[0][1] * inverse_matrix[1][0]

    forward_matrix = [
        [inverse_matrix[1][1] / det, -inverse_matrix[0][1] / det],
        [-inverse_matrix[1][0] / det, inverse_matrix[0][0] / det]
    ]

    return inverse_matrix, forward_matrix
```

### Robot Model Comparison

| Feature | Holonomic | Differential Drive | Tricycle Steering |
|---------|-----------|-------------------|-------------------|
| **Lateral Control** | Direct `cmd_vel.linear.y` | Via angular velocity | Via angular velocity |
| **Constraints** | None (full 3DOF) | Non-holonomic | Non-holonomic + steering limits |
| **Acceleration Limits** | Yaw only | Yaw only | Yaw + steering angle + steering speed |
| **Reverse Handling** | Standard | Standard | Special (flip steering angle) |
| **Complexity** | Simple | Medium | High (state tracking required) |
| **Use Cases** | Omniwheels, mecanum | Most mobile robots | Car-like robots, forklifts |
| **Status** | Unmaintained (bugs) | Well-tested | Production-ready |

## Advanced Features

### Model Predictive Control (MPC)

**Purpose**: Dynamically adjust velocity to maintain lateral error bounds over a prediction horizon using bisection optimization.

```python
def mpc_velocity_regulation(target_velocity, current_pose, odom):
    """
    Use MPC to find maximum safe velocity that keeps lateral error bounded
    """
    # Save controller state for clean simulation
    saved_state = controller_state.copy()

    candidate_velocity = target_velocity
    prev_velocity = 0.0
    optimization_iter = 0

    while optimization_iter <= config.mpc_max_vel_optimization_iterations:
        # Reset state for clean simulation
        controller_state = saved_state.copy()
        predicted_pose = current_pose
        predicted_twist = odom

        # Forward simulation loop
        for step in range(config.mpc_max_fwd_iterations):
            # Run controller with candidate velocity
            result = controller.update(
                candidate_velocity, config.target_end_x_vel,
                predicted_pose, predicted_twist, config.mpc_simulation_sample_time
            )

            # Simple unicycle plant model
            theta = predicted_pose.rotation.yaw
            predicted_pose.x += result.linear.x * cos(theta) * config.mpc_simulation_sample_time
            predicted_pose.y += result.linear.x * sin(theta) * config.mpc_simulation_sample_time
            predicted_pose.yaw += result.angular.z * config.mpc_simulation_sample_time
            predicted_twist = result.velocity_command

            # Check error bounds
            if abs(controller_state.tracking_error_lat) >= config.mpc_max_error_lat:
                # Error exceeded - need to reduce velocity
                optimization_iter += 1
                prev_velocity, candidate_velocity = candidate_velocity, \
                    candidate_velocity - abs(prev_velocity - candidate_velocity) / 2
                break
        else:
            # Completed simulation within bounds
            if abs(controller_state.tracking_error_lat) <= config.mpc_max_error_lat and \
               abs(candidate_velocity) < abs(target_velocity):
                # Can try to increase velocity
                optimization_iter += 1
                prev_velocity, candidate_velocity = candidate_velocity, \
                    candidate_velocity + abs(target_velocity - candidate_velocity) / 2
            else:
                break  # Found good velocity

    # Apply minimum velocity constraint
    final_velocity = max(abs(candidate_velocity), config.mpc_min_x_vel)

    # Restore controller state
    controller_state = saved_state

    return final_velocity
```

### Feedforward Control

**Angular Feedforward**: Uses precomputed path curvature to anticipate required angular velocity.

**Note**: Lateral feedforward is configurable but **NOT implemented** - it always returns 0.0 (see `controller.cpp:551`).

```python
def compute_feedforward_angular_velocity(path_segment_index, computed_forward_velocity):
    """
    Feedforward angular velocity based on path curvature
    IMPORTANT: Uses computed forward velocity, not target velocity
    """
    curvature = turning_radius_inverse[path_segment_index]
    feedforward_angular_vel = curvature * computed_forward_velocity  # control_effort_long_

    return feedforward_angular_vel

def precompute_path_curvature(global_plan):
    """
    Precompute inverse turning radius for each path segment using specific formula
    """
    deltas = deltas_of_plan(global_plan)  # Pose-to-pose transforms
    curvatures = []

    for delta in deltas:
        dx, dy = delta.translation.x, delta.translation.y
        distance_squared = dx*dx + dy*dy

        if distance_squared < FLT_EPSILON:
            curvature = infinity  # Straight line segment
        else:
            # Specific curvature formula from implementation
            curvature = (2 * dy) / distance_squared

        curvatures.append(curvature)

    curvatures.append(0.0)  # Zero curvature at goal

    return curvatures
```

### Anti-Collision Integration

**Velocity Scaling**: Integrated costmap-based obstacle avoidance with footprint projection.

```python
def compute_collision_velocity_limit(config, costmap, robot_footprint):
    """
    Project robot footprint along predicted trajectory and compute velocity limit
    """
    if not config.anti_collision:
        return INFINITY  # No collision avoidance

    # Generate prediction steps along forward trajectory
    prediction_steps = []
    for i in range(num_prediction_steps):
        step_distance = i * config.collision_look_ahead_resolution
        predicted_pose = robot_pose + forward_direction * step_distance
        prediction_steps.append(predicted_pose)

    # Project robot footprint along all prediction steps
    projected_footprint = project_footprint_union(robot_footprint, prediction_steps)

    # Find maximum cost in projected footprint area
    max_cost = 0
    for footprint_point in projected_footprint:
        cost = costmap.get_cost(footprint_point.x, footprint_point.y)
        max_cost = max(max_cost, cost)

    # Apply velocity scaling based on obstacle cost
    if max_cost >= LETHAL_OBSTACLE:
        return 0.0  # Complete stop for lethal obstacles
    elif config.obstacle_speed_reduction:
        # Linear velocity reduction based on cost
        reduction_factor = max_cost / LETHAL_OBSTACLE
        velocity_limit = config.max_x_vel * (1.0 - reduction_factor)
        return velocity_limit
    else:
        return INFINITY  # No speed reduction, just stop for lethal

def integrate_velocity_limits(target_vel, external_limit, obstacle_limit, mpc_limit):
    """
    Combine all velocity limits (called in update_with_limits)
    """
    # Take minimum of all active limits
    max_vel = abs(target_vel)
    max_vel = min(max_vel, external_limit)      # External system limits
    max_vel = min(max_vel, obstacle_limit)      # Obstacle-based limits
    max_vel = min(max_vel, mpc_limit)          # MPC-based limits

    # Preserve sign
    return copysign(max_vel, target_vel)
```

## Implementation Details

### ⚠️ Known Implementation Bugs

**Bug #1 - Critical: Angular Integral Error (`controller.cpp:402`)**
The angular integral term incorrectly uses lateral error instead of angular error:

```cpp
// WRONG (current implementation):
auto error_integral_ang = controller_state_.error_integral_ang.filter(error_lat_filtered, dt.toSec());

// SHOULD BE:
auto error_integral_ang = controller_state_.error_integral_ang.filter(error_ang_filtered, dt.toSec());
```

**Bug #2 - Syntax Error: Invalid Tracking Error Assignment (`controller.cpp:395-396`)**
Invalid syntax with stray comma and unused `dt.toSec()`:

```cpp
// WRONG (syntax error):
controller_state_.tracking_error_ang = angles::normalize_angle(tf2::getYaw(error.getRotation())),
dt.toSec();

// SHOULD BE:
controller_state_.tracking_error_ang = angles::normalize_angle(tf2::getYaw(error.getRotation()));
```

These bugs can affect controller stability and prevent compilation.

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
