NOTICE
======

`eys3d_camera` is © 2026 eYs3D Microelectronics Corp., released under
Apache-2.0 (see `LICENSE`).

## ROS 1 dependencies

Declared in `eys3d_camera/package.xml`:

| Component | License |
|---|---|
| `actionlib`, `actionlib_msgs`, `catkin`, `diagnostic_msgs`, `diagnostic_updater`, `dynamic_reconfigure`, `geometry_msgs`, `message_generation`, `message_runtime`, `nodelet`, `pluginlib`, `roscpp`, `roslib`, `sensor_msgs`, `std_msgs`, `std_srvs`, `tf2`, `tf2_ros`, `topic_tools` | BSD-3-Clause |
| `yaml-cpp` | MIT |
| Boost (pulled in by `roscpp`; used directly for the `dynamic_reconfigure` server mutex) | BSL-1.0 |

## Bundled binaries and headers

| File | Origin | License |
|---|---|---|
| `eys3d_camera/eSPDI/libeSPDI_*.so`, `eSPDI/eSPDI*.h` | eYs3D Microelectronics Corp. | proprietary, redistributed with this driver |
| `eys3d_camera/selfk/lib/*/libeys3d_selfk_*.a`, `selfk/include/` | eYs3D Microelectronics Corp. | proprietary, redistributed with this driver |
| `eys3d_camera/eSPDI/turbojpeg.h` | libjpeg-turbo | BSD-3-Clause |
| `eys3d_camera/eSPDI/OpenCL/*/lib/libOpenCL.so.1` | ocl-icd loader | BSD-2-Clause |
