# VINS-Fusion Monocular + IMU Quick Start

Follow the terminal order below to run the EuRoC dataset (`MH_01_easy.bag`). Activate the same ROS environment in every shell.

## Terminal 1 – RViz
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch vins vins_rviz.launch
```
Keep RViz open to visualize the estimator output.

## Terminal 2 – Main VINS Node
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml
```
This starts the monocular + IMU estimator.

## Terminal 3 – Loop Fusion (optional)
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml
```
Launch only if you need loop-closure.

## Terminal 4 – Play the Dataset
```bash
rosbag play /home/ardyseto/Downloads/MH_01_easy.bag
```
Stream the EuRoC bag that contains monocular and IMU data.
