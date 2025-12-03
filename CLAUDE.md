# CLAUDE.md - AI Assistant Guide for R2D10 Robot Project

## Project Overview

**R2D10** is an autonomous navigation robot project built using ROS 2 (Robot Operating System 2) and Gazebo simulation. The project implements a differential drive robot with LiDAR sensing capabilities for SLAM (Simultaneous Localization and Mapping) and autonomous navigation in simulated environments.

**Key Capabilities:**
- Differential drive mobile robot with caster wheel
- LiDAR sensor for environment scanning
- Ultrasonic sensors (forward, left, right)
- SLAM integration using slam_toolbox
- Gazebo simulation with custom world environments
- RViz2 visualization
- Teleoperation support

## Repository Structure

```
/home/user/group-10-robot-/
├── CMakeLists.txt              # CMake build configuration for r2d10 package
├── package.xml                 # ROS 2 package manifest for r2d10
├── README.md                   # User-facing documentation
├── r2d10_bridge.yaml          # ROS-Gazebo bridge configuration
│
├── src/                        # Source code packages
│   ├── cmd_vel_bridge/        # Python package for velocity command bridging
│   │   ├── cmd_vel_bridge/
│   │   │   ├── __init__.py
│   │   │   └── cmd_vel_bridge.py  # Twist to TwistStamped converter node
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── test/              # Python test files (pep257, flake8, copyright)
│   └── r2d10/                 # Currently empty/placeholder
│
├── launch/                     # ROS 2 launch files
│   ├── updated_launchfile.py  # Basic robot spawn in RViz (mostly commented out)
│   └── world.launch.py        # Main launch file: Gazebo world + robot spawn
│
├── urdf/                       # Robot description files
│   ├── assembly_3.urdf        # Robot URDF (Unified Robot Description Format)
│   ├── assembly_3.urdf.xacro  # Xacro version with macros
│   └── materials.xacro        # Material definitions
│
├── meshes/                     # 3D STL mesh files for robot components
│   ├── base_link.stl
│   ├── caster_1.stl
│   ├── left_wheel_1.stl
│   ├── right_wheel_1.stl
│   ├── lidar_1.stl
│   ├── ultrasonic_forward_1.stl
│   ├── ultrasonic_left_1.stl
│   └── ultrasonic_right_1.stl
│
├── worlds/                     # Gazebo simulation worlds
│   ├── church.sdf             # Main world file (14.5KB, complex environment)
│   └── models/                # Custom Gazebo models
│       ├── actor/
│       ├── bookshelf/
│       ├── cricket ball/
│       ├── floor materials/
│       ├── office chair/
│       ├── office desk/
│       └── trashbin/
│
├── rviz/                       # RViz configuration files
│   └── default.rviz           # Default RViz visualization config
│
├── build/                      # Build artifacts (gitignored)
├── install/                    # Install space (gitignored)
└── log/                        # Build logs (gitignored)
```

## Technology Stack

### Core Technologies
- **ROS 2**: Robot Operating System 2 (likely Humble or Iron distribution)
- **Gazebo**: Physics simulation environment (Gazebo Harmonic/Garden with gz-sim)
- **Python 3**: Primary programming language for nodes
- **C++**: For performance-critical components (configured but not currently used)
- **CMake**: Build system via ament_cmake

### ROS 2 Packages Used
- `rclpy` - Python client library for ROS 2
- `rclcpp` - C++ client library for ROS 2
- `geometry_msgs` - Standard geometry messages (Twist, TwistStamped)
- `nav_msgs` - Navigation messages (Odometry)
- `sensor_msgs` - Sensor messages (LaserScan)
- `robot_state_publisher` - Publishes robot state from URDF
- `joint_state_publisher` - Publishes joint states
- `ros_gz_sim` - ROS 2 integration with Gazebo
- `ros_gz_bridge` - Bridge between ROS 2 and Gazebo topics
- `slam_toolbox` - SLAM implementation
- `teleop_twist_keyboard` - Keyboard teleoperation
- `tf2_ros` - Transform library
- `xacro` - XML macro language for URDF
- `rviz2` - 3D visualization tool

## Key Components

### 1. Robot Model (URDF)

**File**: `urdf/assembly_3.urdf` and `assembly_3.urdf.xacro`

The robot consists of:
- **base_link**: Main body (mass: 37.82 kg)
- **left_wheel_1** and **right_wheel_1**: Differential drive wheels (black material)
- **caster_1**: Passive caster wheel for stability (mass: 0.07 kg)
- **lidar_1**: LiDAR sensor for environment scanning
- **ultrasonic_forward_1**, **ultrasonic_left_1**, **ultrasonic_right_1**: Ultrasonic sensors

All meshes are STL files scaled by 0.001 (millimeters to meters).

**Materials**:
- Silver (RGB: 0.7, 0.7, 0.7) for body and sensors
- Black (RGB: 0, 0, 0) for wheels

### 2. ROS-Gazebo Bridge Configuration

**File**: `r2d10_bridge.yaml`

Bridges the following topics between ROS 2 and Gazebo:

| ROS Topic | Gazebo Topic | Message Type | Direction |
|-----------|--------------|--------------|-----------|
| `/cmd_vel` | `cmd_vel` | geometry_msgs/Twist ↔ gz.msgs.Twist | ROS→GZ |
| `/odom` | `odometry` | nav_msgs/Odometry ↔ gz.msgs.Odometry | GZ→ROS |
| `/scan` | `scan` | sensor_msgs/LaserScan ↔ gz.msgs.LaserScan | GZ→ROS |
| `/clock` | `clock` | rosgraph_msgs/Clock ↔ gz.msgs.Clock | GZ→ROS |

### 3. Velocity Command Bridge Node

**File**: `src/cmd_vel_bridge/cmd_vel_bridge/cmd_vel_bridge.py`

**Purpose**: Converts `Twist` messages to `TwistStamped` messages.

- **Subscribes to**: `/cmd_vel_raw` (Twist)
- **Publishes to**: `/cmd_vel` (TwistStamped)
- **Adds**: timestamp and `base_link` frame_id

This is necessary when teleoperation outputs Twist but Gazebo expects TwistStamped.

### 4. Launch Files

#### `launch/world.launch.py` (Primary Launch File)

**What it does**:
1. Loads the `church.sdf` world into Gazebo
2. Reads robot URDF from `urdf/assembly_3.urdf`
3. Spawns robot at position (0, 0, 0) with a 4-second delay
4. Sets Gazebo model path environment variable

**Commented out but available**:
- `robot_state_publisher` node
- `joint_state_publisher` node
- RViz2 launch

**Usage**: `ros2 launch r2d10 world.launch.py`

#### `launch/updated_launchfile.py` (Alternate/Deprecated)

Mostly commented out. Originally designed to:
- Launch empty Gazebo world
- Spawn robot in RViz
- Start robot_state_publisher and joint_state_publisher

Currently only launches empty Gazebo world without robot.

### 5. World Environment

**File**: `worlds/church.sdf`

A complex indoor environment (14.5 KB file).

**Known Issue**: The world is too GPU-intensive for systems without dedicated GPUs (e.g., mini PCs). According to README, the project should use simpler models from Gazebo library instead.

## Development Workflows

### Standard Development Cycle

1. **Navigate to workspace**: `cd ~/ros2_ws`
2. **Make code changes** in the repository
3. **Build**: `colcon build` or `colcon build --packages-select r2d10 cmd_vel_bridge`
4. **Source**: `source install/setup.bash`
5. **Test**: Run launch files or individual nodes
6. **Commit**: Follow git workflow (see below)

### Running the Robot

#### Option 1: Robot in RViz Only (Simple)

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch r2d10 launchfile2.py
```

Note: `launchfile2.py` is not in the repository - likely exists in install space or is a user-created variant.

#### Option 2: Robot in Gazebo World (Full Simulation)

**Known Issue**: Robot only loads into empty world due to GPU limitations with complex furniture models.

**Terminal 1** - Launch Gazebo world:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch r2d10 worldlaunch2.py
```

Note: `worldlaunch2.py` is not in the repository - likely refers to `world.launch.py`.

**Terminal 2** - Start SLAM:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**Terminal 3** - Teleoperation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 4** - Static transform (map→odom):
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

**Terminal 5** - ROS-Gazebo bridge:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

**Terminal 6** - Robot state publisher:
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat ~/ros2_ws/install/r2d10/share/r2d10/urdf/assembly_3.urdf)"
```

**Terminal 7** - Joint state publisher:
```bash
ros2 run joint_state_publisher joint_state_publisher
```

**Terminal 8** - Odometry to TF converter:
```bash
cd ~/Downloads
python3 odom_to_tf.py
```

Note: `odom_to_tf.py` is not in the repository - external script.

### Simplified Launch (In Development)

A new launch file `slam_complete_launch` is being developed to consolidate all terminals into one launch file. Status: needs RViz fixes and debugging.

## Build System

### Package: r2d10

- **Type**: ament_cmake (C++/CMake-based)
- **Build tool**: ament_cmake
- **Dependencies**: rclcpp, geometry_msgs, std_msgs, robot_state_publisher, ros_gz_sim, xacro, ros2launch

**CMakeLists.txt** installs:
- meshes/
- urdf/
- rviz/
- worlds/
- launch/

### Package: cmd_vel_bridge

- **Type**: ament_python (Python-based)
- **Dependencies**: rclpy, geometry_msgs
- **Entry point**: `cmd_vel_bridge = cmd_vel_bridge.cmd_vel_bridge:main`

## Important Conventions

### File Naming
- Launch files: `*.launch.py` or `*launchfile.py`
- URDF files: `*.urdf` (compiled) or `*.urdf.xacro` (source with macros)
- World files: `*.sdf` (Gazebo SDF format)
- Mesh files: `*.stl` (STereoLithography format)

### ROS 2 Conventions
- **Package names**: lowercase with underscores (e.g., `cmd_vel_bridge`)
- **Node names**: lowercase with underscores (e.g., `cmd_vel_bridge`)
- **Topic names**: lowercase with underscores, prefixed with `/` (e.g., `/cmd_vel`)
- **Frame names**: lowercase with underscores (e.g., `base_link`, `odom`, `map`)

### Code Style
- **Python**: Follow PEP 257 (docstrings), PEP 8, and Flake8 standards
- **C++**: Wall, Wextra, Wpedantic enabled
- **Indentation**: Spaces (verify in existing files - typically 4 for Python, 2 for launch files)

### Git Conventions
- Commit messages: Descriptive, imperative mood (e.g., "Add details about new launch file in README")
- Branch: `main` is the primary branch
- Recent commits show frequent README updates and frame file removals

## Common Issues and Solutions

### 1. Furniture Models Too Complex

**Problem**: Complex furniture models (chair, coffee table, bookshelf) from Gazebo database are too GPU-intensive for mini PCs.

**Solution**:
- Use simplified shape models (cubes, spheres, cylinders)
- Assignment requirements: at least 1 chair, 1 coffee table, 1 bookshelf, 1 sphere, 1 cube
- Can still use objects from Gazebo database but prefer simplified versions

**Recommendation for AI**:
- When adding objects to worlds, use basic geometric shapes or optimized models
- Prefer `<geometry><box>`, `<geometry><sphere>`, `<geometry><cylinder>` over complex meshes

### 2. URDF File Not Found

**Error**: "ERROR: URDF file not found at: {robot_path}"

**Cause**: Robot URDF not installed or package not built

**Solution**:
```bash
cd ~/ros2_ws
colcon build --packages-select r2d10
source install/setup.bash
```

### 3. Launch Files Not in Repository

**Issue**: README references `launchfile2.py`, `worldlaunch2.py`, `slam_complete_launch` which don't exist in source.

**Explanation**: These are likely:
- Generated during build process
- User-created variants not committed
- Renamed versions of existing launch files

**AI Action**: When asked about these, check both source and install directories, or ask user for clarification.

### 4. Multiple Terminal Requirement

**Problem**: Full simulation requires 8 separate terminal windows.

**Status**: Team is working on `slam_complete_launch` to consolidate all nodes into one launch file.

**AI Recommendation**: When creating launch files, use `ComposableNode` or `Node` actions to group related functionality.

## Git Workflow

### Committing Changes

```bash
cd ~/ros2_ws
git add .
git commit -m "Descriptive message about changes"
git branch --set-upstream-to=origin/main main  # First time only
git pull --rebase
git push
```

### Current Branch

- **Main branch**: `main`
- **Development branch** (for AI): `claude/claude-md-mioovmfgkw00a44d-018bxrEbPZfipHe6kGU557D3`

**IMPORTANT for AI assistants**:
- Always develop on the designated Claude branch
- Push to the Claude branch, NOT main
- Only merge to main via pull request when requested

### Recent Activity

Recent commits focus on:
- README updates and improvements
- Removing unnecessary frame files
- Adding documentation for launch files and Gazebo usage
- Merging local changes with remote

## AI Assistant Guidelines

### When Making Changes

1. **Always read files before modifying** - Never propose changes to code you haven't seen
2. **Preserve existing patterns** - Match indentation, naming conventions, and code style
3. **Test locally** - Suggest commands to verify changes work
4. **Update documentation** - If you change functionality, update README.md or this file
5. **Commit with clear messages** - Explain what changed and why

### File Operations Priority

1. **Read**: Use Read tool for viewing file contents
2. **Edit**: Use Edit tool for modifying existing files (preferred over Write)
3. **Write**: Only for new files when absolutely necessary
4. **Avoid**: Don't use bash cat/echo/sed for file operations

### Launch File Development

When creating or modifying launch files:
- Use `generate_launch_description()` function
- Import from `launch`, `launch_ros.actions`, `ament_index_python.packages`
- Handle file not found errors gracefully (see existing launch files)
- Use `TimerAction` for delayed spawning (robot needs Gazebo to be ready)
- Set `use_sim_time:=true` for simulation nodes

### URDF Development

When modifying robot description:
- Edit `.urdf.xacro` source file, not compiled `.urdf`
- Maintain inertial properties (mass, inertia matrix)
- Keep mesh file paths as `package://r2d10/meshes/filename.stl`
- Scale meshes consistently (current: 0.001 for mm→m conversion)
- Test collision geometry matches visual geometry

### Adding Dependencies

**For Python packages** (cmd_vel_bridge):
- Add to `package.xml`: `<depend>package_name</depend>`
- Add to `setup.py`: Update `install_requires` list

**For CMake packages** (r2d10):
- Add to `package.xml`: `<depend>package_name</depend>`
- Add to `CMakeLists.txt`: `find_package(package_name REQUIRED)`

### Testing Changes

Always suggest verification commands:
```bash
# Build specific package
colcon build --packages-select r2d10

# Check for compilation errors
colcon build --packages-select r2d10 --event-handlers console_direct+

# Source and test
source install/setup.bash
ros2 launch r2d10 world.launch.py

# Check topic list
ros2 topic list

# Echo a topic
ros2 topic echo /scan

# Check TF tree
ros2 run tf2_tools view_frames
```

### World File Modifications

When editing `worlds/church.sdf`:
- Use simple geometric shapes for new objects
- Avoid high-polygon meshes
- Test on low-end hardware if possible
- Consider providing "lite" and "full" world variants

### Common Tasks

**Adding a new sensor**:
1. Add link and joint to URDF
2. Add mesh file to `meshes/` if needed
3. Add Gazebo sensor plugin to URDF
4. Add sensor topic to bridge configuration (`r2d10_bridge.yaml`)
5. Update launch file to bridge new topics
6. Test in Gazebo and RViz

**Creating a new node**:
1. Add Python file to `src/cmd_vel_bridge/cmd_vel_bridge/` or create new package
2. Update `setup.py` with entry point
3. Add to launch file
4. Build and test

**Debugging Gazebo issues**:
- Check Gazebo logs: `~/.gz/sim/log/`
- Verify topic publishing: `ros2 topic list`, `ros2 topic hz /topic_name`
- Check bridge: `ros2 run ros_gz_bridge parameter_bridge --help`
- Use verbose mode: `gz sim -v 4 world.sdf`

## Assignment Requirements (Context)

The project must include in the world:
- At least 1 chair
- At least 1 coffee table
- 1 bookshelf
- At least 1 sphere
- At least 1 cube

Objects can be from Gazebo database, but simplified models are recommended for performance.

## Useful ROS 2 Commands

```bash
# List all packages
ros2 pkg list

# Find package installation path
ros2 pkg prefix r2d10

# List nodes
ros2 node list

# Get node info
ros2 node info /node_name

# List topics
ros2 topic list

# Get topic info
ros2 topic info /cmd_vel

# Echo topic
ros2 topic echo /scan

# Publish to topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# List services
ros2 service list

# List parameters
ros2 param list

# Get parameter value
ros2 param get /node_name param_name

# View TF tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo source_frame target_frame

# Launch file
ros2 launch package_name launch_file.py

# Run node
ros2 run package_name executable_name
```

## Gazebo Commands

```bash
# Launch Gazebo with world
gz sim world.sdf

# Launch with verbosity
gz sim -v 4 world.sdf

# List Gazebo topics
gz topic -l

# Echo Gazebo topic
gz topic -e -t /topic_name

# List models
gz model -l

# Get model info
gz model -m model_name -i
```

## Project Status and TODOs

### Completed
- ✅ Basic robot URDF with meshes
- ✅ Differential drive configuration
- ✅ LiDAR and ultrasonic sensors modeled
- ✅ Gazebo world with objects
- ✅ ROS-Gazebo bridge configuration
- ✅ Velocity command bridge node
- ✅ Basic launch files
- ✅ RViz configuration

### In Progress
- 🔄 Consolidated launch file (slam_complete_launch)
- 🔄 Optimizing world for low-end hardware
- 🔄 RViz integration fixes

### Known Issues
- ⚠️ Complex furniture models cause performance issues
- ⚠️ Robot only loads into empty world (not church.sdf) on mini PCs
- ⚠️ Multiple terminal windows required for full simulation
- ⚠️ Some launch file references in README don't match repository

### Suggested Improvements (For AI)
- Create lightweight world variants
- Implement navigation goal system
- Add autonomous navigation capabilities
- Create launch file configurations for different hardware profiles
- Add IMU sensor integration
- Implement obstacle avoidance
- Add camera sensor for visual SLAM
- Create unit tests for nodes
- Add CI/CD pipeline

## Contact and Maintenance

- **Package maintainer**: group10 (group10@todo.todo)
- **License**: TODO (not yet declared)
- **ROS 2 Distribution**: Not specified (recommend documenting)

---

**Last Updated**: 2025-12-02
**Repository**: jdynros2/group-10-robot-
**For AI Assistants**: This document should be updated whenever significant changes are made to the codebase structure, workflows, or conventions.
