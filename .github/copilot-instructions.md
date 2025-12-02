# Copilot / AI Agent Instructions for this repository

Purpose: help an AI coding assistant become productive quickly with this ROS2 workspace.

- **Big picture:** This is a ROS2 workspace that provides a simulated robot package `r2d10` (C++/URDF assets + launch files) and a small Python ROS node package `cmd_vel_bridge` (in `src/cmd_vel_bridge`) that adapts teleop Twist -> TwistStamped for the simulator. Simulation integration uses `ros_gz_sim` and a ROS↔Gazebo bridge configured in `src/r2d10/r2d10_bridge.yaml`.

- **Important locations:**
  - `CMakeLists.txt` (workspace/package top-level): shows this package uses `ament_cmake`, `rclcpp`, `ros_gz_sim` and installs `meshes`, `urdf`, `rviz`, `worlds`, `launch`.
  - `src/r2d10/` : package share with `urdf/`, `meshes/`, `rviz/`, `worlds/`, and `launch/` (multiple launch files). See `src/r2d10/README.md` for developer notes and run steps.
  - `src/r2d10/launch/` : multiple launch files (e.g. `world.launch.py`, `updated_launchfile.py`, `launchfile2.py`). These orchestrate `ros_gz_sim` and spawning.
  - `src/r2d10/r2d10_bridge.yaml` : mapping of ROS topics ↔ Gazebo topics (cmd_vel, odom, scan, clock). Use this to understand bridging direction and message types.
  - `src/cmd_vel_bridge/` : Python package with `cmd_vel_bridge/cmd_vel_bridge.py` — converts `/cmd_vel_raw` (Twist) to `/cmd_vel` (TwistStamped).

- **Build / run (concrete):**
  - Build workspace from repository root:

    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

  - Launch robot in RViz (example from `src/r2d10/README.md`):

    ```bash
    ros2 launch r2d10 launchfile2.py
    ```

  - Launch robot in the Gazebo world (example):

    ```bash
    ros2 launch r2d10 worldlaunch2.py
    # or use the main world launcher:
    ros2 launch r2d10 world.launch.py
    ```

  - Single-launch entry (repo provides `launch/updated_launchfile.py` and `src/r2d10/launch/updated_launchfile.py`): these include `ros_gz_sim` and spawn/rviz nodes. If URDF not found the launch prints an error.

- **Topic / runtime conventions discovered:**
  - Teleop publishes to `/cmd_vel_raw` (Twist). `cmd_vel_bridge` subscribes to `/cmd_vel_raw` and republishes a `TwistStamped` on `/cmd_vel` with `frame_id='base_link'` and current time.
  - Bridging config located at `src/r2d10/r2d10_bridge.yaml` defines:
    - `cmd_vel` (ROS -> GZ)
    - `odom`, `scan`, `clock` (GZ -> ROS)
  - Some launch flows rely on `ros_gz_sim` `create` executable to spawn robots using either `--file <urdf>` or `-topic robot_description`.

- **Edit / extend guidance (where to change things):**
  - URDF: `src/r2d10/urdf/assembly_3.urdf` (spawned by launch files). If changes cause "URDF file not found" errors check path resolution in launch files (`get_package_share_directory('r2d10')`).
  - Bridge mappings: update `src/r2d10/r2d10_bridge.yaml` to add/remove topics or change directions.
  - cmd_vel adaptors: modify `src/cmd_vel_bridge/cmd_vel_bridge/cmd_vel_bridge.py` to adjust stamping, frames or topic names.

- **Debug / common pitfalls** (based on repo notes):
  - URDF/model path errors: launch files will print a FileNotFoundError when URDF missing — confirm `install/share/r2d10/urdf/...` after `colcon build` and `source install/setup.bash`.
  - Gazebo/world complexity: `src/r2d10/README.md` notes heavy models can cause simulator failures on low-GPU machines. Use `-r empty.sdf` or simplified models for debugging.
  - Multiple terminals historically required; many launch files try to consolidate processes — prefer the single combined launch when possible.

- **Testing / linting signals:**
  - `cmd_vel_bridge` includes test extras in `setup.py` and mentions `pytest` in `extras_require`. There are `test/` folders under `src/cmd_vel_bridge/` — run `pytest` from the package directory after sourcing the environment or use `colcon test` (if tests are registered).

- **When writing code changes** — be conservative and follow these repo patterns:
  - Use package share paths obtained with `get_package_share_directory('r2d10')` inside launch code.
  - Keep the bridge topic names and message types consistent with `r2d10_bridge.yaml` unless intentionally changing bridge behavior — update the YAML and any launch/README references together.
  - For Python ROS nodes, use `rclpy` patterns similar to `cmd_vel_bridge` (node init, subscriptions, publishers, `get_clock().now().to_msg()` for header stamping).

If anything here is unclear or you want the file to emphasize different areas (more examples, CI steps, or contributor conventions), tell me which sections to expand and I'll iterate.
