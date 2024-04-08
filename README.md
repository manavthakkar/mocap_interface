# MoCap ROS2 Interface

This ROS2 package can be used to get the pose and odometry information of a specific robot present in the working area of the motion capture system. It builds on top of the [qualisys_driver](https://github.com/MOCAP4ROS2-Project/mocap4ros2_qualisys/tree/main) package. 

## Installation

Navigate the terminal to the `ros2_ws/src` and run the following command to download the qualisys repo recursively:
```bash
git clone --recursive https://github.com/MOCAP4ROS2-Project/mocap4ros2_qualisys.git
```

Install dependencies:
```bash
vcs import < mocap4ros2_qualisys/dependency_repos.repos
```

Compiling workspace:
```bash
cd .. && colcon build --symlink-install
```

Source workspace:
```bash
source install/setup.bash
```

Setup your qualisys configuration. Change host_name to `"192.168.1.20"` in the following file :
```bash
ros2_ws/src/mocap4ros2_qualisys/qualisys_driver/config/qualisys_driver_params.yaml
```

## Example:

To get the pose information of a specific frame/robot present in the working area, run the following command (You should be connected to the ITL Forschung local network). Change the `id` and `robot_name` according to the requirement:
```bash
ros2 launch mocap_interface startup.launch.py id:=11 robot_name:=/laura
```
For e.g. in this case, the pose information of laura robot having `id:=11` (obtained from the qualisys MoCap software) is published on the `/laura/pose` topic in the form of `PoseStamped` messages. Also, odometry is published on the `/laura/odometry` topic.


## To track another robot simultaneously:

To add another robot: 
```bash
ros2 launch mocap_interface add_robot.launch.py id:=13 robot_name:=/jackal
```
This will publish the pose and odometry to `/jackal/pose` and `/jackal/odometry` topics respectively.

Follow the same process for each new robot you want to add.