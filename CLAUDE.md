# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

### Build
```bash
catkin_make
```

### Testing
```bash
# Unit tests
catkin run_tests path_tracking_pid

# Integration tests
rostest path_tracking_pid test_path_tracking_pid.test
```

### Launch
```bash
roslaunch path_tracking_pid path_tracking_pid_mbf.launch
```

### Code Quality
```bash
# Linting is integrated into CMake build via roslint
```

## Architecture Overview

This is a ROS C++ package implementing a PID-based path tracking controller for mobile robots. It serves as a local planner plugin for move_base_flex navigation framework.

### Core Components

- **Controller** (`controller.hpp/cpp`): Main PID control logic with dual loops (lateral + angular)
- **Local Planner Plugin** (`path_tracking_pid_local_planner.hpp/cpp`): move_base_flex plugin interface
- **Control Details** (`details/`): Low-level components (derivative, integral, filters, FIFO arrays)
- **Calculations** (`calculations.hpp/cpp`): Mathematical utilities for path tracking
- **Visualization** (`visualization.hpp/cpp`): RViz markers and debug visualization

### Key Features

- **Dual Control Modes**: Carrot-following vs direct base_link tracking
- **Model Support**: Differential drive and tricycle steering models
- **MPC Integration**: Optional Model Predictive Control for velocity regulation
- **Anti-collision**: Costmap integration for obstacle avoidance
- **Dynamic Reconfiguration**: Real-time parameter tuning

### Build System

- **C++ Standard**: C++17 with strict warnings (`-Wall -Wextra -Wpedantic -Werror`)
- **ROS Version**: Tested on Melodic and Noetic
- **Plugin System**: Exports mbf_costmap_core plugin via `path_tracking_pid_plugin.xml`

### Testing Structure

- **Unit Tests**: `/test/unittests/` - Google Test framework for mathematical components
- **Integration Tests**: Root `/test/` - Python-based rostest scenarios (preemption, acceleration, turns)
- **Main Test Suite**: `test_path_tracking_pid.test` - comprehensive scenario testing

### Configuration

- **Static Parameters**: `/param/path_tracking_pid.yaml`
- **Dynamic Parameters**: `cfg/Pid.cfg` - accessible via rqt_reconfigure
- **Custom Messages**: `PidDebug.msg`, `PidFeedback.msg`

### Development Notes

- The controller supports both feedback and feedforward control modes
- Tricycle model requires steered wheel frame configuration
- MPC feature uses predictive path analysis for velocity regulation
- Anti-collision uses costmap gradients for safe velocity scaling