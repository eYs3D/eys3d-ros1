# eYs3D 立体深度相机 ROS 1 驱动程序

[![ROS 1](https://img.shields.io/badge/ROS%201-Melodic%20%7C%20Noetic-blue)](https://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`eys3d_camera` 是 eYs3D 立体深度相机的官方 ROS 1 驱动程序，发布彩色影像、
深度影像与点云，遵循 REP-103 frame tree，并直接构建在随附的 eSPDI SDK 之上
——没有中间封装层。支持 ROS Melodic 与 Noetic。

### 支持的相机

| 型号 | Product code | USB | 状态 |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量产 |
| **R77** | YX8072 | USB 2.0 | 量产 |
| **G62** | YX8081 | USB 2.0 | 量产 |

---

## 安装

```bash
sudo apt install ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-nodelet \
                 ros-$ROS_DISTRO-dynamic-reconfigure \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-rviz
```

这些都不在 `ros-$ROS_DISTRO-ros-base` 内。per-model launch 默认
`urdf:=true` 与 `rviz:=true`，缺少 `robot_state_publisher`、`xacro`、`rviz`
会无法启动；加上 `urdf:=false rviz:=false` 则不需要。

驱动程序以 GCC 7.5 及以上构建，与随附 eSPDI SDK 自身的工具链一致，因此
Melodic（Ubuntu 18.04）与 Noetic（Ubuntu 20.04）均受支持。更旧的发行版
无法支持：SDK 二进制文件本身就需要 GCC 7 的 C++ runtime。

将 package 放入 catkin workspace 的 `src/` 后构建：

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```


### 设备权限

驱动以普通用户身份打开相机。若打开设备时出现权限错误，安装随附的
udev 规则，让 eSPDI SDK 能访问该 USB 设备：

```bash
sudo cp $(rospack find eys3d_camera)/udev/99-eys3d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

然后重新插拔相机。此规则授予对 eYs3D 设备（USB vendor `3438`）的访问权限；
或者将用户加入 `video` 组后重新登录。

---

## 快速开始

每个型号都有对应的 launch 快捷方式，自动应用该机型的默认 video mode：

```bash
roslaunch eys3d_camera G100P.launch     # G100+：L'+D 1280x720 interleave，SDK 每流 30 fps
roslaunch eys3d_camera R77.launch       # R77： L'+D 1280x920 彩色 + 640x460 深度 @ 30 fps
roslaunch eys3d_camera G62.launch       # G62： L'+D 640x480 @ 25 fps
```

要切换 mode 用 `mode_id:=<n>`；各机型完整 mode 列表位于
`launch/video_modes/<MODEL>.yaml`。

### 发布的 Topic

以 `camera_name:=G100P_1`（各机型默认值）为例：

| Topic | 类型 | 说明 |
|---|---|---|
| `/G100P_1/left_color/image_raw` | `sensor_msgs/Image` | 左眼彩色影像，固定 `rgb8`（YUYV 与 MJPEG 来源均 inline 解码；灰阶传感模组以 R=G=B 输出）|
| `/G100P_1/right_color/image_raw` | `sensor_msgs/Image` | 右眼彩色影像；仅在 video mode 带有 L\|R 并排输出时发布 |
| `/G100P_1/depth/image_raw` | `sensor_msgs/Image` (`16UC1`，mm，REP-118) | 深度（毫米） |
| `/G100P_1/depth/points` | `sensor_msgs/PointCloud2` | XYZ float32，已转换为 ROS 基底轴（米）；当 `colored_pointcloud:=true` 时改为 XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 相机内参，每张 Image 一张（`header.stamp` 相同） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 健康状态，1 Hz |

### 配合 image_pipeline（image_proc、depth_image_proc 等）

每条流会把 `<stream>/image_raw` 与 `<stream>/camera_info` 发布为同层
sibling，这正是 `image_transport::CameraSubscriber` 解析的排列方式。因此
`image_proc`、`depth_image_proc`、`stereo_image_proc` 与 `camera_calibration`
都能自动把影像与其 `CameraInfo` 配对——指向该流的 namespace 即可直接工作：

```bash
ROS_NAMESPACE=/G100P_1/left_color rosrun image_proc image_proc
```

驱动本身已经能发布配准好的 XYZRGB 点云（`colored_pointcloud:=true`），
因此只有在需要 `depth_image_proc` 的特定变体时才需要用它。

**不支持主机端标定。** 没有 `camera_info_url` 参数、没有
`camera_info_manager`、也没有 `set_camera_info` service，所以
`rosrun camera_calibration cameracalibrator.py` 可以显示与计算标定结果，
但无法存回本驱动。标定值是从模组自身的 flash 读取的，漂移要靠
[自我标定](#自我标定) 修正，而非主机端文件。

### CameraInfo 与畸变系数

每一份 `camera_info` 描述的都是它自己 topic 上发布的那张图像。

深度在所有模式下都是已校正的；彩色则在目录名称带撇号的模式下已校正
（`L'+D`、`L'+R'+D`）。那些 topic 上的 `K` 是 `P` 的左 3×3、`D` 为零
（`plumb_bob`）、`R` 为单位矩阵 —— 相机已经移除畸变，没有东西需要还原。

`L+R` 与 `L+R+D` 模式在彩色 topic 上发布的是原始传感器图像。那里的
`K` 与 `D` 是发布分辨率下的原厂镜头模型、`R` 是校正旋转，`image_proc`
可以正常校正它们。请读 `distortion_model` 而不要假设系数个数：driver 会
报告 `rational_polynomial` 八个系数或 `plumb_bob` 五个，依该台模组内
存储的标定数据而定。

两种情况下 `P` 都是投影矩阵，也是位姿估计（AprilTag、PnP、SLAM）应该
取用的内参来源。立体对的右相机 `P[3]` 是 `-fx × 基线`，单位为米。

---

## 配置

### 切换 video mode

```bash
roslaunch eys3d_camera G100P.launch mode_id:=7    # L'+D 640x480 interleave
roslaunch eys3d_camera R77.launch   mode_id:=4    # 纯深度 640x460 @ 30 fps
roslaunch eys3d_camera G62.launch   mode_id:=3    # L'+D 320x240 @ 30 fps
```

节点启动时会打印完整 mode 列表。`mode_id:=-1`（默认）会按协商到的 USB
链路速度自动选用该机型的 signature mode。

### Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `camera_name` | `<MODEL>_1` | ROS namespace 与 frame-id 前缀 |
| `mode_id` | `-1` | video mode 列表中的索引；`-1` = 自动 |
| `dev_serial_number` | `""` | 通过序列号子串绑定 |
| `usb_port` | `""` | 通过 USB 拓扑路径绑定，例如 `2-3:1.0` |
| `depth_near_mm` / `depth_far_mm` | `-1` | 深度裁切上下限（mm）；`-1` = 该型号默认 |
| `ir_value` | `-1` | `-1` = 该型号默认（G100+/R77 = 3、G62 = 60）；`0` = 关闭 |
| `colored_pointcloud` | `false` | 从最近一张彩色帧取色，发布 XYZRGB |
| `spatial_filter` | `false` | 启用视差域边缘感知 IIR 滤波器 |
| `temporal_filter` | `false` | 启用时间滤波器 |
| `hole_filling` | `0` | `0`=关闭、`1`=fill_from_left、`2`=farthest_from_around、`3`=nearest_from_around |
| `filter_profile` | `default` | 滤波器调校配置档（`cfg/filter_profiles/<name>.yaml`）|
| `selfcal_enable` | `false` | 开放自我标定 action 与 commit service |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` 频率；低于 `0.001` 即关闭 |
| `urdf` | `true` | 通过 `robot_state_publisher` 发布相机模型 |
| `rviz` | `true` | 是否以该机型的随附布局打开 RViz |
| `rviz_config` | `""` | 自定义 `.rviz` 布局路径；留空则使用随附布局 |
| `output` | `screen` | 节点输出去向：`screen` 或 `log` |

### 运行中调整（dynamic_reconfigure）

影像与时间滤波器设置可在驱动运行中调整：

```bash
rosrun rqt_reconfigure rqt_reconfigure          # GUI
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera ir_value 5
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera auto_exposure false
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera exposure_time_step -8
```

可调整项：`ir_value`、`auto_exposure`、`exposure_time_step`（`[-13, 3]`，
仅在 `auto_exposure` 关闭时生效）、`auto_white_balance`、
`power_line_frequency`（1 = 50 Hz、2 = 60 Hz）、四个 `temporal_filter_*`
数值，以及 `selfcal_profile`。

所有 eYs3D 模组出厂默认自动曝光与自动白平衡均为开启。驱动程序
**继承固件开机时的设置**，只写回 launch 明确覆盖的项目，所以在 ROS 之外
调整过的设置能跨重启保留。IR 是例外：投射器开机后默认关闭，因此
`ir_value` 一定会应用。

### 后处理滤波器

三组可选滤波器，默认全关，且彼此独立运作。

| 滤波器 | 切换方式 |
|---|---|
| `spatial_filter` | 仅 launch |
| `temporal_filter` | launch + 运行中（dynamic_reconfigure）|
| `hole_filling` | 仅 launch（`2` 为建议起始模式）|

调校值（`alpha` / `delta` / `magnitude` / `holes_fill` / `persistence`）
放在 `cfg/filter_profiles/<name>.yaml`。要自定义 profile，复制
`default.yaml` 并修改字段，然后传入 `filter_profile:=<name>`。

```bash
roslaunch eys3d_camera G100P.launch spatial_filter:=true temporal_filter:=true
roslaunch eys3d_camera G100P.launch spatial_filter:=true hole_filling:=2
roslaunch eys3d_camera G100P.launch spatial_filter:=true filter_profile:=indoor
```

---

## 运行中的流控制

```bash
# 停止发布，但相机仍在 USB 上传输。驱动 CPU 降到接近零；
# 恢复后下一帧就会出现。适合短暂中断。
rosservice call /G100P_1/pause "data: true"
rosservice call /G100P_1/pause "data: false"

# 完全释放 USB 管线。节点仍存活、topic 仍保持 advertised；
# 重新打开需要数秒，视机型而定。可释出 USB 带宽。
rosservice call /G100P_1/standby "data: true"
rosservice call /G100P_1/standby "data: false"

# 通过 USB 复位相机（重新枚举）。节点会停止数据流、发出复位，
# 然后由 watchdog 自动重连。用于在不重启节点的情况下救回卡死的相机。
rosservice call /G100P_1/hw_reset
```

两个流控制都是彩色 + 深度成对作用：不支持单条流切换，因为
interleave 模式需要 V4L2 流的两半同时工作。

### 热插拔自动恢复

1 Hz 的 watchdog 从打开起就监看当前模式会跑的每一条流。送出过帧的流
静默 3 秒判定断开;打开后 10 秒内一帧都没送出的流同样判定断开,不论
另一条是否正常。断开后驱动每 2 秒轮询一次,按绑定的序列号 / USB port
重新打开,重新发布 static TF,并重新应用 IR 与 CT/PU 设置。

> **RViz 与改名后的相机。** 随附布局在每个 topic 路径、`robot_description`
> 与 Fixed Frame 都写着该机型的默认 `camera_name`。若改用其他名称，布局会
> 按实际名称改写后从临时目录打开，各面板与默认情况一样正常显示。若要改用
> 自己的布局，传 `rviz_config:=<path>`，该文件会原样使用。

### 多台相机

驱动以机型的 USB PID 选择设备（G100+ = `0x0181`、R77 = `0x0180`、
G62 = `0x0183`）。**同型号**的两台相机必须各自绑定：

```bash
roslaunch eys3d_camera G100P.launch camera_name:=G100P_left  dev_serial_number:=ABC123
roslaunch eys3d_camera G100P.launch camera_name:=G100P_right usb_port:=2-3:1.0

roslaunch eys3d_camera dual_G100P.launch          # 示例封装
roslaunch eys3d_camera G100P_plus_R77.launch      # 混合机型无需绑定
```

---

## Nodelet（零拷贝）

这个驱动**本身就是** nodelet —— 独立可执行文件 `eys3d_camera_node` 只是把
它加载到自己的 process。改为加载到共享的 manager，该 manager 内的其他
nodelet 就能以驱动发布时的同一个 `shared_ptr` 收到影像与点云消息，没有
序列化、也没有拷贝：

```bash
roslaunch eys3d_camera G100P_nodelet.launch
rosrun nodelet nodelet load my_pkg/MyNodelet /eys3d_nodelet_manager
```

请将 manager 的 `num_worker_threads` 保持在 4 以上：仅这个驱动本身，就需要
在其中一个 callback 卡在 SDK 调用时，让 service、timer、action 与
reconfigure 的 callback 仍能继续推进。

---

## 自我标定

可选的 in-stream 自我标定，可重新对齐双目、救回标定已漂移模组的深度
填充率。逐次 launch 启用：

```bash
roslaunch eys3d_camera G100P.launch selfcal_enable:=true
```

运行时相机必须**正在传输含深度的 mode，并对着工作距离内正常、有纹理的
场景**——标定器是靠测量深度覆盖率工作的，因此需要有效的深度作为依据。

```bash
# 跑一次 session（约 20-30 秒收敛，之后有一段简短的复检）。
rostopic pub --once /G100P_1/selfcal/run/goal \
  eys3d_camera/SelfCalActionGoal "goal: {auto_commit_shift_px: 0.25}"

# 或用正规的 action client 操作：
rosrun actionlib axclient.py /G100P_1/selfcal/run

# 将保留下来的结果写入 flash（只有在 auto-commit 没触发时才需要）。
# 请检查响应的 success 字段——若没有保留中的结果会是 false。
rosservice call /G100P_1/selfcal/commit
```

一次 run 的结局同时取决于标定器的 outcome **以及一段实测 A/B 复检**
——在同一场景下比较新对齐与运行前对齐的深度填充率：

- **确认变好** —— 复检确认的 `SUCCESS` 会生效（`applied: true`）。若
  `auto_commit_shift_px` 为正值且 `cy_shift_px` 达到该值，这次 run 会
  **写入** flash（`committed: true`）；否则只是**保留在线上**但为易失性
  ——调用 `selfcal/commit` 才能撑过断电。
- **已是最优**（`NO_CHANGE`）—— 没有任何变更、应用或写入。
- **变差 / 无法验证 / 失败** —— 相机会**回退**（`reverted: true`）。
  此处若为 `reverted: false`，表示回退本身失败，相机停在被否决的对齐上，
  要断电才会恢复；`message` 会说明。

`cy_shift_px` 是直接从硬件读回的实测垂直位移量，也是 auto-commit 阀门的
比较对象。步进固定为 **0.25 px**，修正量上限为 **5.0 px**。

> `auto_commit_shift_px` **必须为正值才会 auto-commit。** ROS 1 的
> message 字段没有默认值，省略的字段会以 `0.0` 送达；阀门将 `<= 0` 视为
> 永不 commit，因此省略的情况就是安全的那一边。若确实需要 auto-commit，
> 建议值为 `0.25`——也就是一个标定步进。

### Action Result

| 字段 | 含义 |
|---|---|
| `outcome` | `SUCCESS` / `NO_CHANGE` / `INSUFFICIENT_INPUT`（有效深度不足）/ `TIMEOUT`（时限内未收敛）/ `FAILED` |
| `cy_shift_px` | 实测的垂直 cy 位移量（像素）——auto-commit 的判断依据 |
| `recheck_verdict` | A/B 复检结果：`improved` / `worse` / `inconclusive` / `skipped` |
| `recheck_ratio_before` / `_after` | 运行前 / 收敛后对齐的填充率 |
| `applied` / `reverted` / `committed` | 这次 run 在相机上留下了什么 |
| `message` | 人类可读的摘要 |

`commit` 是唯一会写入 flash 的步骤；原厂标定会保留为备份，永远不会被
覆盖。已保留但未 commit 的结果只存在于相机的寄存器中，因此断电或
`hw_reset` 就会清回已存储的标定值。这让「在线标定、永不写 flash」成为
一个安全的默认流程。

有几点值得知道：

- **重复运行会叠加。** 在 commit 或断电之前重跑，是叠在上一次保留的结果
  之上，而不是叠在 flash 上。要干净的基准请先 commit 或断电。
- **不符合条件或已在运行 → 是 rejected，不是 failed。** 不支持自我标定的
  模组，以及第二个并发的 run，都会被直接拒绝——action client 会看到
  goal 被 rejected 且没有 result；原因请看节点 log。
- **session 无法中断。** cancel 会被拒绝，而控制类 service
  （`pause` / `standby` / `hw_reset`）在运行期间也会拒绝。它很短，而且
  结果变差时会自动回退。
- **每个 process 一次只能跑一个 session。** 共用同一个 nodelet manager
  的相机只能一台一台标定；位于不同 process 的相机则互不影响。

---

## Diagnostics（`/diagnostics`）

每秒一条 `DiagnosticArray`。每一条内含 5 个 `DiagnosticStatus`（每
task 一个），名称为 `"<node_name>: <task>"` —— 去掉开头斜线的节点名称，
例如 `G100P_1/eys3d_camera: device`；模块序列号放在另一个独立的
`hardware_id` 字段。整体健康状态由 `device` task 承载：

| `level` | `message` | 含义 |
|---|---|---|
| `OK` | `streaming` | 每条配置中的流都在发送 |
| `OK` | `streaming (paused — publish gated by operator)` | `pause` 生效中 |
| `OK` | `standby (USB pipe closed by operator)` | `standby` 生效中 |
| `ERROR` | `no frames flowing on any configured stream` | 每条配置中的流都低于预期速率的一半 |
| `ERROR` | `camera disconnected; Linux device node not present` | USB 断线；watchdog 会在设备回插时自动重连 |

各 task 的 `values` 键值对：

**`device`** — 连接与识别：

| Key | 说明 |
|---|---|
| `connection_state` | `streaming` 或 `disconnected` |
| `device_present` | `true` / `false`，与 `connection_state` 同一个状态的布尔形式 |
| `reconnect_attempts` | 当前这次断线的重连次数；成功一次后归零 |
| `usb_port` | open 时解析到的 sysfs 接口路径（如 `2-3:1.0`）|
| `serial_number` | SDK 上报的模块序列号 |
| `actual_fps` | `APC_OpenDevice2` 协商到的 fps。interleave mode 不会除以二 —— 每串流速率是该值的一半 |
| `stream_state` | `Active` / `Paused` / `Standby` — `pause` / `standby` service 控制的运行时状态 |

**`color`** 与 **`depth`** — 每条串流吞吐：

| Key | 说明 |
|---|---|
| `input_fps` | 过去 1 秒从 SDK 收到的帧数。与订阅者状态无关 — 用于判断相机 / USB 健康度 |
| `publish_fps` | 过去 1 秒实际发布到 topic 的帧数。无订阅者时为 0；有订阅者且 driver 跟得上时 ≈ `input_fps`；持续低于代表 driver 落后 |
| `input_total` | 自 open 起累计从 SDK 收到的帧数 |
| `publish_total` | 自 open 起累计发布的帧数 |
| `input_dropped` | 累计 SDK 端掉帧数（由 serial-number 跳跃检测）|
| `decode_avg_ms` | （仅 `color`）过去 1 秒 color 解码平均耗时。当下 1 秒内有解码过 frame 才会出现 |
| `decode_max_ms` | （仅 `color`）至今观察到的最长 color 解码耗时。曾经解码过 frame 才会出现 |

两者的摘要为 `streaming`、`input rate below 50% of expected`（WARN）、
`not configured (D-only mode)` 或 `standby`。

**`pointcloud`** — 点云投影 + 后处理计数：

| Key | 说明 |
|---|---|
| `compute_status` | `active`（有订阅者拉点云）、`idle (no /depth/points subscriber)` 或 `idle (never run ; no subscriber since start)`。断线时不会出现该字段，该 task 只带 summary |
| `publish_fps` | 过去 1 秒点云发布帧数。无订阅者时为 0 |
| `compute_avg_ms` | 过去 1 秒点云计算平均耗时（`active` 时出现）|
| `compute_max_ms` | 至今观察到的最长点云计算耗时 |
| `publish_total` | 累计发布的点云数 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各后处理阶段的累计调用次数 |

**`thermal`**：

| Key | 说明 |
|---|---|
| `temperature_c` | 芯片温度（°C）。仅在机型具备传感器且读取成功时发布；否则不会出现该字段，原因写在该任务的 summary |

由 timer 以 `diagnostics_rate_hz` 驱动 Updater，无论有没有订阅者都会
发布。hot-path 的 atomic counter 独立累计，所以随时连接监控都能看到自
open 起的累计值。

判读方法：

| `input_fps` | `publish_fps` | 含义 |
|---|---|---|
| ≈ 预期 | 0 | 无订阅者，driver idle |
| ≈ 预期 | ≈ input | driver 跟得上 |
| ≈ 预期 | < input | 有订阅者，但发布路径落后（解码太慢，或订阅端跟不上）|
| 0 | 0 | 相机 / USB 没在发送数据 — 查看 `device.connection_state` |

```bash
rosrun rqt_robot_monitor rqt_robot_monitor
rostopic echo /diagnostics
```

内置监控工具：自动订阅所有 stream，并把 SDK / Pub / Rx 三条 rate
与解码 / 计算耗时并列显示：

```bash
rosrun eys3d_camera perf_monitor            # 自动检测 namespace
rosrun eys3d_camera perf_monitor --ns /G100P_1 --interval 0.5
```

每一行按流显示：**SDK**（相机送来的帧）、**Pub**（驱动实际发出的帧）、
**Rx**（monitor 收到的帧）。

---

## Frame ID

两层 TF tree，以 latched 方式发布于 `/tf_static`：

```
<camera_name>_link                      ROS 基底轴（x 前、y 左、z 上）
├── <camera_name>_left_color_frame      y 轴 +baseline/2
│   └── <camera_name>_left_color_optical_frame     REP-103 光学坐标
├── <camera_name>_right_color_frame     y 轴 -baseline/2
│   └── <camera_name>_right_color_optical_frame
├── <camera_name>_depth_frame           y 轴 +baseline/2
│   └── <camera_name>_depth_optical_frame
└── <camera_name>_points_frame          y 轴 +baseline/2
```

影像消息标记 `_optical_frame` 叶节点；`PointCloud2` 标记
`<camera_name>_points_frame`，该 frame 已经是基底轴。

---

## 疑难排解

| 症状 | 处理方式 |
|---|---|
| 打开相机失败显示 "device busy"，或打开后始终收不到帧 | 有其他 process 占用 `/dev/videoN`。第二个本驱动的实例通常不会报错,而是打开后饿死,每秒记录一次 `delivered no frame within 10 s of open`。用 `lsof /dev/video*` 找出占用者;若是残留的驱动就停掉它。 |
| 打开时出现权限错误 | 安装 udev 规则（见[设备权限](#设备权限)）或加入 `video` 组。 |
| RViz Image 面板空白 | 将 Fixed Frame 设为 `<camera_name>_link`，并确认 topic 与当前的 `camera_name` 相符。 |
| 没有 `right_color` topic | 只有 wide-color 模式会在单一端点带 L\|R。请检查 mode 列表中的 `split_lr`。 |
| 深度全为零 | 落在 `depth_near_mm` / `depth_far_mm` 之外的像素会被归零,请确认范围涵盖你的工作距离。近平面若等于或超过远平面会被直接拒绝:节点记录 `depth_near_mm must be < depth_far_mm` 后进入空闲,topic 仍会 advertise 但不会有数据。 |
| 自我标定 goal 被 rejected | 要么该模组没有 user calibration slot，要么已有 session 在运行。原因请看节点 log。 |

---

## 许可

Apache-2.0 —— 见 [LICENSE](../LICENSE) 与 [NOTICE.md](../NOTICE.md)。
