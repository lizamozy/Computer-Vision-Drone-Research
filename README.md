# Computer-Vision-Drone-Research

## Overview
**Complete the PX4 steps before running any of the programs below**
* simple.py: Sanity Script to make sure that two drones can fly at once in one Gazebo simluation
* sqaure_mission: Script to fly two drones in a square mission
* reported_position.py: Script to get the PX4 Position and Gazebo position of the leader drone, flys them in a sqaure mission, and outputs a graph to show the difference between the two positions
* drone_vision.py: Script to calculate estimated position of leader drone from camera footage from the follower drone

## Two PX4 x500_depth Drones in One Gazebo Gz World – Quick Steps
### 1. Prerequisites (done once)
- Ubuntu 22.04 with PX4-Autopilot cloned.
- Gazebo Gz (Harmonic) installed via PX4 setup script.
- ROS 2 Humble (for camera feed) and ros_gz_bridge installed.
- Build PX4 SITL at least once:
* `cd ~/PX4-Autopilot`
* `make px4_sitl`
### 2. Start Drone 1 (rear drone, instance 1)
Open Terminal 1:
* `cd ~/PX4-Autopilot`
* `PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL=x500_depth \
./build/px4_sitl_default/bin/px4 -i 1`
*Wait for the pxh> prompt, then in that shell:
* `param set NAV_DLL_ACT 0`
This starts Gazebo Gz with the default world and spawns model x500_depth_1.
### 3. Start Drone 2 (front drone, instance 2)
Open Terminal 2:
*`cd ~/PX4-Autopilot`
*`PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL=x500_depth \
PX4_GZ_MODEL_POSE="3,0" \
./build/px4_sitl_default/bin/px4 -i 2`
*In the second pxh> shell:
*`param set NAV_DLL_ACT 0`
Now the world contains two drones: x500_depth_1 at the origin and
x500_depth_2 about 3 m in front along the +X axis.
### 4. Arm and Take Off (per drone)
In each pxh> shell (Terminal 1 and Terminal 2):
*`commander arm`
*`commander takeoff`
Make sure the Gazebo simulation is unpaused so both vehicles lift off.
### 5. Camera Topics (optional, for ROS 2)
- Each drone publishes its RGB camera over Gz topics, for example:
`/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image`
`/world/default/model/x500_depth_2/link/camera_link/sensor/IMX214/image`
To bridge the rear drone camera to ROS 2 run the below in a fresh terminal:
*`ros2 run ros_gz_bridge parameter_bridge \
"/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image@"\`

### Visualize in another fresh terminal using `rqt_image_view` or `RViz2`.

### Then run any of the python scripts in the repo after all of these steps are completed
* do not run more than one program at once as MAVSDK can get confused on ports being busy 
** To run `reported_position.py` user needs to create additional ros bridge to gather the gazebo world GPS data and run this bridge in another seperate terminal **
