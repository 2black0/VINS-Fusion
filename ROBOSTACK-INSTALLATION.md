# Install VINS-Fusion with RoboStack (ROS Noetic)

This document explains how to deploy VINS-Fusion on Ubuntu 22.04 using RoboStack ROS 1 (Noetic) on conda.

## 1. Create and Prepare the Environment
```bash
conda create -n ros_env -c conda-forge -c robostack-noetic ros-noetic-desktop
conda activate ros_env
conda install -c conda-forge compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```

> After activation, run once:
> ```bash
> rosdep init    # only if this machine has never run rosdep before
> rosdep update
> ```

## 2. Create the ROS Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
cd ..
```

## 3. Build with catkin_make
RoboStack ships CMake 4.1 (compatibility for <3.5 removed) and OpenCV 4.11 (legacy C-API macros disabled). The repository has been patched to work in this environment:

- all packages now compile with C++14 (`-std=c++14`)
- OpenCV C-API compatibility headers are included wherever `CV_*` macros are still used
- the Ceres `FindSuiteSparse.cmake` script accepts SuiteSparse v7.x from conda-forge
- the workspace toplevel `CMakeLists.txt` should be copied (or symlinked) from `$CONDA_PREFIX/share/catkin/cmake/toplevel.cmake`

Prepare the catkin entry point and build:
```bash
cd ~/catkin_ws/src/VINS-Fusion
cp $CONDA_PREFIX/share/catkin/cmake/toplevel.cmake CMakeLists.txt
cd ..
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

> For a clean rebuild:
> ```bash
> rm -rf build devel
> catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5
> ```

## 4. Verify roscore and RViz
Use two terminals, each with `conda activate ros_env`:

**Terminal 1**
```bash
roscore
```

**Terminal 2**
```bash
rviz
```

## 5. Compatibility Notes
- The conda packages `ceres-solver` 2.1.0 and SuiteSparse 7.10 work with the patches in this repo; adjust `FindSuiteSparse.cmake` if you use another version.
- Always run ROS commands inside the `ros_env` environment so RoboStack dependencies are visible.
- For EuRoC monocular + IMU execution instructions, see `Run-Tutorial.md`.
