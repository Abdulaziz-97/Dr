# Troubleshooting Guide - CMake Build Error Fix

## Issue: Duplicate Target Error

If you encountered this error during build:
```
CMake Error: add_custom_target cannot create target "ament_cmake_python_symlink_bodyguard_drone"
because another target with the same name already exists.
```

**Solution:** The CMakeLists.txt has been fixed. Follow these steps:

## Fix Instructions

### Step 1: Clean the Build
```bash
cd ~/Dr/bodyguard_drone_ws
rm -rf build/ install/ log/
```

### Step 2: Rebuild
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Step 3: Source and Test
```bash
source install/setup.bash
ros2 pkg list | grep bodyguard_drone
```

You should see `bodyguard_drone` in the list.

### Step 4: Verify Nodes
```bash
ros2 pkg executables bodyguard_drone
```

You should see all 7 nodes listed.

## What Was Fixed

The issue was in `CMakeLists.txt`:
- **Removed**: `ament_python_install_package(${PROJECT_NAME})`
- **Reason**: This was conflicting with `setup.py` which already handles Python package installation

## Continue Installation

After successful build, continue with:
```bash
./quick_start.sh headless
```

## Path Warning Fix (Optional)

If you see warnings about scripts not in PATH:
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

This adds Python scripts (ultralytics, whisper, transformers) to your PATH.

---

**If you still have issues after this fix, please check the logs:**
```bash
cat log/latest_build/bodyguard_drone/stderr.log
```
