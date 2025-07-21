# kinova_interface

ROS2 code for teleoperating a Kinova Gen3 7 DOF arm via ROS2.

## Connecting to the arm and the space mouse

[Official readme](https://github.com/Kinovarobotics/ros2_kortex/tree/ros2_dev?tab=readme-ov-file#gen-3-robots)

We'll use the BDAI container to interact with the Kinova arm and the space mouse

```bash

$ bdai docker start -sp -p maple --devices  # Enter the container

$ bdai build-packages manipulation_runtime -n -1

$ sudo apt update && sudo apt install -y ros-humble-kortex-bringup  # Must be done every time

$ ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.10 robot_controller:=twist_controller


```

### Controlling with the keyboard or space mouse

Now that our arm is connected to ROS, we can send Twist commands to it from either the keyboard or space mouse.

```bash
# Connect to the existing Docker container started above
$ bdai docker shell  

# Control with the keyboard
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/twist_controller/commands

# Or with space mouse
$ ros2 run bdai_spacenav bdai_spacenav_node --ros-args --remap /spacenav/twist:=/twist_controller/commands
```

## With xvisio controller

The xvisio controller ("Thor handle") runs in its own container, built from [Andy Park's repo](https://github.com/robodreamer/xvisiotech_camera_examples).

```bash
$ cd ~/xvisiotech_camera_examples
$ ./scripts/start_container.sh
$ source ./scripts/setup_ros2_driver.sh
$ sudo chmod -R 777 /dev/bus/usb/  # Yes, this is jank, but necessary to avoid permission errors
$ ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py
```

### xvisio coordinate frames

The coordinate frames published from xvisio to ROS do not follow ROS conventions:

- +x is right
- +y is down
- +z is forward

Therefore it's right-handed but needs to be rotated to align with ROS conventions.

## kinova_interface node

We need a simple ROS2 node to translate data from HIDs into Twist messages for the Kinova arm.

Subscribes to:
- Pose or Twist messages, e.g. `/xv_sdk/slam/pose` (PoseStamped)
- Joy messages for button states, e.g. `/spacenav/joy`