## go2_urdf
This repository contains the urdf model of go2.

## How to use with go2_robot (replace only go2_description)

Follow the upstream setup, then swap only this package.

1) Setup upstream workspace

```bash
# Create workspace and clone upstream (humble branch)
mkdir -p ~/go2_ws/src && cd ~/go2_ws/src
git clone https://github.com/Unitree-Go2-Robot/go2_robot.git -b humble

# Install dependencies
sudo apt update && sudo apt install ros-dev-tools -y
vcs import < go2_robot/dependencies.repos

cd ~/go2_ws
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

2) Replace only go2_description with this one

```bash
cd ~/go2_ws/src
rm -rf go2_robot/src/go2_description
# Copy this repository's go2_description into the upstream tree
cp -R <PATH_TO_THIS_REPO>/src/go2_description go2_robot/src/
```

3) Build and source

```bash
cd ~/go2_ws
colcon build --symlink-install
source install/setup.bash
```

4) Launch examples (from upstream bringup)

```bash
ros2 launch go2_bringup go2.launch.py rviz:=True
```

Reference: `https://github.com/Unitree-Go2-Robot/go2_robot`

## scripts

### `scripts/go2_lowstate_to_joint.py`

- Subscribes: `/lowstate` (`unitree_go/LowState`)
- Publishes: `/joint_states` (`sensor_msgs/JointState`)
- Joint order (name and arrays):
  - `[FL_hip_joint, FL_thigh_joint, FL_calf_joint, FR_hip_joint, FR_thigh_joint, FR_calf_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint, RR_hip_joint, RR_thigh_joint, RR_calf_joint]`
- Motor index mapping (from `LowState.motor_state` to the above order):
  - `[3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]` (matches `go2_driver.cpp`)

Run directly (example):

```bash
python3 ~/go2_ws/src/go2_description/scripts/go2_lowstate_to_joint.py
```

Or include it in a launch file as a separate process alongside `robot_state_publisher` and `rviz2`.

### Joint names, order, and config alignment

- Joint names in URDF (12 total):
  - FL_hip_joint, FL_thigh_joint, FL_calf_joint
  - FR_hip_joint, FR_thigh_joint, FR_calf_joint
  - RL_hip_joint, RL_thigh_joint, RL_calf_joint
  - RR_hip_joint, RR_thigh_joint, RR_calf_joint

- Expected JointState.name order (must match go2_driver.cpp and ros_control):
```text
["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
 "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
 "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
 "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"]
```

- Driver publishing order and LowState index mapping (`src/go2_driver/src/go2_driver/go2_driver.cpp`):
  - name order: FL, FR, RL, RR (each: hip, thigh, calf)
  - position mapping from `LowState.motor_state` indices:
    - FL: [3, 4, 5]
    - FR: [0, 1, 2]
    - RL: [9, 10, 11]
    - RR: [6, 7, 8]

- ros2_control config alignment:
  - File: `src/go2_description/config/ros_control/ros_control.yaml`
  - `joint_trajectory_controller.ros__parameters.joints` must keep the same order as `JointState.name` above.

- Important:
  - `JointState.name`, `position`, `velocity`, `effort` arrays must have the same length (12) and the same ordering.
  - A missing comma or mismatched lengths/order will cause `robot_state_publisher` to ignore messages as invalid.
