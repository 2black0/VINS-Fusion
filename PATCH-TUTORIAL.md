# PATCH Tutorial

This document explains how to apply the `local-changes.patch` file to the original VINS-Fusion repository and how to update it when needed.

## Patch contents
The patch contains the following updates:
- Build configuration tweaks (CMake) so the project compiles with the RoboStack toolchain on Ubuntu 22.04.
- Fixes for code that relies on legacy OpenCV APIs and related configuration changes.
- Additional documentation (`ROBOSTACK-INSTALLATION.md` and `RUN-TUTORIAL.md`).
- A `CMakeLists.txt` symlink that points to catkin’s `toplevel.cmake` inside the conda environment.

The patch was generated from base commit `be55a937a57436548ddfb1bd324bc1e9a9e828e0` (HEAD at the time this document was written). Make sure the target repository is at that commit before applying the patch.

## How to apply the patch
1. **Prepare a fresh checkout of the original VINS-Fusion repository**
   ```bash
   git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
   cd VINS-Fusion
   git checkout be55a937a57436548ddfb1bd324bc1e9a9e828e0
   ```
2. **Copy `local-changes.patch`** into the root of that repository.
3. **Apply the patch**
   ```bash
   git apply local-changes.patch
   ```
4. **Verify the result**
   ```bash
   git status
   ```
   All modifications should now appear as unstaged changes.

> Note: the patch creates a `CMakeLists.txt` symlink that points to an absolute conda path, e.g. `/home/ardyseto/miniconda3/envs/ros_env/share/catkin/cmake/toplevel.cmake`. Adjust the symlink after applying the patch so it targets `toplevel.cmake` on your machine (`$CONDA_PREFIX/share/catkin/cmake/toplevel.cmake`).

## Reverting changes
If you want to discard the applied changes before committing:
```bash
git reset --hard HEAD
```

## Regenerating the patch
When you modify the project and need to refresh the patch:
1. Ensure the updates are present in your working tree.
2. Stage everything (including new files) with `git add -A`.
3. Produce the new patch:
   ```bash
   git diff --cached > local-changes.patch
   ```
4. Restore the staging area if necessary:
   ```bash
   git reset
   ```
