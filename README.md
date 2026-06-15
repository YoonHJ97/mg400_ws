# mg400_ws

Controlling the MG400 Robot Arm Using ROS2.

This repository is a collection of ROS2 packages (packages live at the repo
root). Clone it into the `src/` folder of a colcon workspace.

## Packages

| Package | Type | Description |
|---|---|---|
| `mg400_pkg` | ament_python | MG400 nodes: motion (`move_node`), MoveTo service (`move_service_server`), status publisher (`mg400_status_node`) and the Dobot TCP API. |
| `mg400_pkg_msgs` | ament_cmake | Interfaces (`MoveTo.srv`). |
| `mg400_bringup` | ament_cmake | Launch + config to bring the arm up. |

> Nav2 / AMR integration nodes live on the `amr-integration` branch.

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YoonHJ97/mg400_ws.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run

```bash
# Status publisher (joint_states / tcp_pose / robot_mode)
ros2 launch mg400_bringup mg400.launch.py

# Override robot IP
ros2 launch mg400_bringup mg400.launch.py robot_ip:=192.168.1.10

# Also start the MoveTo service server
ros2 launch mg400_bringup mg400.launch.py enable_service:=true
```
