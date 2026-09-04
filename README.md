# ROS 1 Driver for eYs3D Stereo Depth Cameras

[![ROS 1](https://img.shields.io/badge/ROS%201-Melodic%20%7C%20Noetic-blue)](https://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

**Language:** [English](README.md) · [日本語](docs/README.ja.md) · [繁體中文](docs/README.zh-TW.md) · [简体中文](docs/README.zh-CN.md)

`eys3d_camera` is the official ROS 1 driver for eYs3D stereo depth cameras.
It publishes color, depth, and point cloud topics with a standard REP-103
frame tree, and is built directly on the bundled eSPDI SDK — no wrapper
layer. Supports ROS Melodic and Noetic.

### Supported Devices

| Module | Product code | USB | Status |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | Production |
| **R77** | YX8072 | USB 2.0 | Production |
| **G62** | YX8081 | USB 2.0 | Production |

---

## Installation

```bash
sudo apt install ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-nodelet \
                 ros-$ROS_DISTRO-dynamic-reconfigure \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-rviz
```

None ship with `ros-$ROS_DISTRO-ros-base`. The per-model launches default to
`urdf:=true` and `rviz:=true` and will not start without
`robot_state_publisher`, `xacro` and `rviz`. Pass `urdf:=false rviz:=false` to
run without them.

The driver builds with GCC 7.5 or newer, matching the bundled eSPDI SDK's
own toolchain, so both Melodic (Ubuntu 18.04) and Noetic (Ubuntu 20.04) are
supported. Older distros are out of reach: the SDK binary itself needs a
GCC 7 C++ runtime.

Place the package under your catkin workspace `src/` and build:

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```


### Device Permissions

The driver opens the camera as a normal user. If device open fails with a
permission error, install the bundled udev rule so the eSPDI SDK can reach
the USB device:

```bash
sudo cp $(rospack find eys3d_camera)/udev/99-eys3d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then replug the camera. The rule grants access to eYs3D devices (USB vendor
`3438`); alternatively, add your user to the `video` group and re-login.

---

## Quick Start

Each model has a launch shortcut that opens its default mode:

```bash
roslaunch eys3d_camera G100P.launch     # G100+: L'+D 1280x720 interleave, SDK 30 fps per stream
roslaunch eys3d_camera R77.launch       # R77:   L'+D 1280x920 color + 640x460 depth @ 30 fps
roslaunch eys3d_camera G62.launch       # G62:   L'+D 640x480 @ 25 fps
```

Override the mode with `mode_id:=<n>`; the full per-model catalogue is in
`launch/video_modes/<MODEL>.yaml`.

### Published Topics

With `camera_name:=G100P_1` (the per-model default):

| Topic | Type | Description |
|---|---|---|
| `/G100P_1/left_color/image_raw` | `sensor_msgs/Image` | Left color image, always `rgb8` (YUYV and MJPEG sources are decoded inline; grayscale-source modules deliver R=G=B) |
| `/G100P_1/right_color/image_raw` | `sensor_msgs/Image` | Right color image; published only in wide-color modes that carry L\|R on one endpoint |
| `/G100P_1/depth/image_raw` | `sensor_msgs/Image` (`16UC1`, mm, REP-118) | Depth in millimetres |
| `/G100P_1/depth/points` | `sensor_msgs/PointCloud2` | XYZ float32 in ROS base axes (metres); XYZRGB when `colored_pointcloud:=true` |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics, one per matching Image frame (same `header.stamp`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Health metrics, 1 Hz |

### Using image_pipeline (image_proc, depth_image_proc, …)

Each stream publishes `<stream>/image_raw` and `<stream>/camera_info` as
siblings, which is the layout `image_transport::CameraSubscriber` resolves.
`image_proc`, `depth_image_proc`, `stereo_image_proc` and
`camera_calibration` therefore pair the image with its `CameraInfo`
automatically — point them at the stream namespace and they just work:

```bash
ROS_NAMESPACE=/G100P_1/left_color rosrun image_proc image_proc
```

The driver already publishes a registered XYZRGB cloud itself
(`colored_pointcloud:=true`), so `depth_image_proc` is only needed when you
want one of its specific variants.

**Host-side calibration is not supported.** There is no `camera_info_url`
parameter, no `camera_info_manager`, and no `set_camera_info` service, so
`rosrun camera_calibration cameracalibrator.py` can display and compute a
calibration but cannot store it against this driver. Calibration is read
from the module's own flash, and drift is corrected by
[self-calibration](#self-calibration) rather than by a host-side file.

### CameraInfo and Distortion Coefficients

Each `camera_info` describes the image published on its own topic.

Depth is rectified in every mode, and color is rectified in every mode whose
catalogue name carries an apostrophe (`L'+D`, `L'+R'+D`). On those topics `K`
is the left 3x3 of `P`, `D` is zero (`plumb_bob`) and `R` is the identity: the
camera has already removed the distortion, so there is nothing left to undo.

The `L+R` and `L+R+D` modes publish the raw sensor image on the color topics.
There `K` and `D` are the factory lens model at the published resolution and
`R` is the rectification rotation, so `image_proc` rectifies them normally.
Read `distortion_model` rather than assuming a length: the driver reports
`rational_polynomial` with eight coefficients or `plumb_bob` with five,
following the calibration stored in that individual module.

`P` is the projection matrix in both cases and is the right source of
intrinsics for pose estimation (AprilTag, PnP, SLAM). For a stereo pair,
`P[3]` of the right camera is `-fx * baseline` in metres.

---

## Configuration

### Switch Video Mode

```bash
roslaunch eys3d_camera G100P.launch mode_id:=7    # L'+D 640x480 interleave
roslaunch eys3d_camera R77.launch   mode_id:=4    # D-only 640x460 @ 30 fps
roslaunch eys3d_camera G62.launch   mode_id:=3    # L'+D 320x240 @ 30 fps
```

The node prints the full mode table on startup. `mode_id:=-1` (the default)
auto-selects the model's signature mode for the negotiated USB link.

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `camera_name` | `<MODEL>_1` | ROS namespace + frame-id prefix |
| `mode_id` | `-1` | Mode index in the per-model catalogue; `-1` = auto |
| `dev_serial_number` | `""` | Bind by serial-number substring |
| `usb_port` | `""` | Bind by USB topology, e.g. `2-3:1.0` |
| `depth_near_mm` / `depth_far_mm` | `-1` | Depth cutoffs in mm; `-1` = per-model default |
| `ir_value` | `-1` | `-1` = per-model default (G100+/R77 = 3, G62 = 60); `0` = off |
| `colored_pointcloud` | `false` | Publish XYZRGB sampled from the latest color frame |
| `spatial_filter` | `false` | Enable the disparity-domain edge-aware IIR filter |
| `temporal_filter` | `false` | Enable the temporal filter |
| `hole_filling` | `0` | `0`=off, `1`=fill_from_left, `2`=farthest_from_around, `3`=nearest_from_around |
| `filter_profile` | `default` | Filter tuning profile (`cfg/filter_profiles/<name>.yaml`) |
| `selfcal_enable` | `false` | Expose the self-calibration action + commit service |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` rate; below `0.001` disables it |
| `urdf` | `true` | Publish the camera model via `robot_state_publisher` |
| `rviz` | `true` | Open RViz on the bundled layout for this model |
| `rviz_config` | `""` | Path to your own `.rviz` layout; empty uses the bundled one |
| `output` | `screen` | Node output sink: `screen` or `log` |

### Runtime Controls (dynamic_reconfigure)

Image and temporal-filter settings are tunable while the driver runs:

```bash
rosrun rqt_reconfigure rqt_reconfigure          # GUI
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera ir_value 5
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera auto_exposure false
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera exposure_time_step -8
```

Tunable: `ir_value`, `auto_exposure`, `exposure_time_step` (`[-13, 3]`,
applies only while `auto_exposure` is off), `auto_white_balance`,
`power_line_frequency` (1 = 50 Hz, 2 = 60 Hz), the four
`temporal_filter_*` values, and `selfcal_profile`.

All eYs3D modules ship with auto-exposure and auto white balance on. The
driver **inherits the firmware's boot state** and writes back only what the
launch explicitly overrides, so settings you made outside ROS survive a
restart. IR is the exception: the projector boots off, so `ir_value` is
always applied.

### Post-Processing Filters

Three optional filters, all off by default and independent of each other.

| Filter | Toggled by |
|---|---|
| `spatial_filter` | launch only |
| `temporal_filter` | launch + runtime (dynamic_reconfigure) |
| `hole_filling` | launch only (`2` is the recommended starting mode) |

Tuning values (`alpha` / `delta` / `magnitude` / `holes_fill` /
`persistence`) live in `cfg/filter_profiles/<name>.yaml`. Copy
`default.yaml` and edit fields to author a profile, then pass
`filter_profile:=<name>`.

```bash
roslaunch eys3d_camera G100P.launch spatial_filter:=true temporal_filter:=true
roslaunch eys3d_camera G100P.launch spatial_filter:=true hole_filling:=2
roslaunch eys3d_camera G100P.launch spatial_filter:=true filter_profile:=indoor
```

---

## Runtime Stream Control

```bash
# Stop publishing but keep the camera streaming on USB. Driver CPU drops to
# near zero; resume appears on the next frame. For short interruptions.
rosservice call /G100P_1/pause "data: true"
rosservice call /G100P_1/pause "data: false"

# Release the USB pipe entirely. The node stays alive and its topics stay
# advertised; the reopen takes a few seconds depending on the model.
# Frees USB bandwidth.
rosservice call /G100P_1/standby "data: true"
rosservice call /G100P_1/standby "data: false"

# Reset the camera over USB (re-enumerate). The node stops the streams,
# issues the reset, and the watchdog reconnects automatically. Use to
# recover a wedged camera without restarting the node.
rosservice call /G100P_1/hw_reset
```

Both stream controls act on the color + depth pair together: per-stream
toggling is unsupported because interleave modes need both halves of the
V4L2 stream active at once.

### Hot-plug Auto-recovery

A 1 Hz watchdog watches each stream the active mode runs, from the open.
A stream that has delivered a frame and then goes silent for 3 s declares a
disconnect; so does one that has delivered no frame within 10 s of the open,
whether or not the other stream is running. On disconnect the driver polls
every 2 s, reopens by the pinned serial / USB port, re-publishes the static
TF, and re-applies the IR and CT/PU settings.

> **RViz and a renamed camera.** The bundled layouts name each model's
> default `camera_name` in every topic path, in `robot_description` and in the
> Fixed Frame. Launched under a different name, the layout is rewritten for
> the name in use and opened from the temporary directory, so the panels fill
> in as they do by default. Pass `rviz_config:=<path>` to open your own layout
> instead; that one is used exactly as written.

### Multiple Cameras

The driver selects a device by the model's USB PID (G100+ = `0x0181`,
R77 = `0x0180`, G62 = `0x0183`). Two cameras of the *same* model must be
pinned individually:

```bash
roslaunch eys3d_camera G100P.launch camera_name:=G100P_left  dev_serial_number:=ABC123
roslaunch eys3d_camera G100P.launch camera_name:=G100P_right usb_port:=2-3:1.0

roslaunch eys3d_camera dual_G100P.launch          # example wrapper
roslaunch eys3d_camera G100P_plus_R77.launch      # mixed models need no pinning
```

---

## Nodelet (zero-copy)

The driver *is* a nodelet — the standalone `eys3d_camera_node` executable
loads it into its own process. Load it into a shared manager instead and
any other nodelet in that manager receives the image and point-cloud
messages as the very `shared_ptr` the driver published, with no
serialisation and no copy:

```bash
roslaunch eys3d_camera G100P_nodelet.launch
rosrun nodelet nodelet load my_pkg/MyNodelet /eys3d_nodelet_manager
```

Keep the manager's `num_worker_threads` at 4 or above: the driver alone
needs its service, timer, action, and reconfigure callbacks to make
progress while another is blocked in an SDK call.

---

## Self-Calibration

Optional in-stream self-calibration re-aligns the stereo pair to recover
depth fill rate on a module whose calibration has drifted. Enable it per
launch:

```bash
roslaunch eys3d_camera G100P.launch selfcal_enable:=true
```

Run it with the camera **streaming a depth mode and pointed at a normal,
textured scene at working distance** — the calibrator measures depth
coverage, so it needs valid depth to work from.

```bash
# Run one session (~20-30 s to converge, then a brief re-check).
rostopic pub --once /G100P_1/selfcal/run/goal \
  eys3d_camera/SelfCalActionGoal "goal: {auto_commit_shift_px: 0.25}"

# Or drive it from a proper action client:
rosrun actionlib axclient.py /G100P_1/selfcal/run

# Persist a kept result to flash (only needed when auto-commit did not fire).
# Check the response's success field — it is false if there is nothing kept.
rosservice call /G100P_1/selfcal/commit
```

The run resolves itself on the calibrator's outcome **plus a live A/B
re-check** that measures depth fill-rate at the new vs. the pre-run
alignment on the same scene:

- **verified better** — a `SUCCESS` the re-check confirms is live
  (`applied: true`). If `auto_commit_shift_px` is positive and
  `cy_shift_px` reaches it, the run **commits** to flash
  (`committed: true`); otherwise it is **kept live** but volatile — call
  `selfcal/commit` to keep it past a power-cycle.
- **already optimal** (`NO_CHANGE`) — nothing changed, applied, or written.
- **worse / unverifiable / failed** — the camera is **rolled back**
  (`reverted: true`). `reverted: false` here means the rollback itself
  failed and the camera is on the rejected alignment until it is
  power-cycled; `message` says so.

`cy_shift_px` is the measured vertical shift read straight from the
hardware, and is what the auto-commit gate compares against. The step is
fixed at **0.25 px** and the correction is capped at **5.0 px**.

> `auto_commit_shift_px` **must be positive to auto-commit.** ROS 1 message
> fields have no default value, so an omitted field arrives as `0.0`; the
> gate treats `<= 0` as never-commit so that the omitted case is the safe
> one. `0.25` — one calibration step — is the recommended value when you do
> want auto-commit.

### Action Result

| Field | Meaning |
|---|---|
| `outcome` | `SUCCESS` / `NO_CHANGE` / `INSUFFICIENT_INPUT` (not enough valid depth) / `TIMEOUT` (did not converge in time) / `FAILED` |
| `cy_shift_px` | measured vertical cy shift in pixels — the auto-commit gate |
| `recheck_verdict` | A/B re-check: `improved` / `worse` / `inconclusive` / `skipped` |
| `recheck_ratio_before` / `_after` | fill-rate at the pre-run / converged alignment |
| `applied` / `reverted` / `committed` | what the run left on the camera |
| `message` | human-readable summary |

`commit` is the only step that writes flash; the factory calibration is
kept as a backup and never overwritten. A kept-but-not-committed result
lives only in the camera's registers, so a power-cycle or `hw_reset` clears
it back to the stored calibration. This makes "calibrate live, never write
flash" a safe default workflow.

A couple of things worth knowing:

- **Repeated runs can stack.** Re-running before a commit or power-cycle
  builds on the last kept result, not on flash. Commit or power-cycle first
  for a clean baseline.
- **Not eligible or already running → rejected, not failed.** A module that
  does not support self-calibration, and a second concurrent run, are both
  refused outright — the action client sees the goal rejected with no
  result; check the node log for the reason.
- **A session cannot be interrupted.** Cancels are refused, and the control
  services (`pause` / `standby` / `hw_reset`) decline while one is running.
  It is short and auto-reverts if the result is worse.
- **One session per process.** Cameras sharing a nodelet manager calibrate
  one at a time; cameras in separate processes are unaffected.

---

## Diagnostics (/diagnostics)

`/diagnostics` carries one `DiagnosticArray` per second. Each round
emits five `DiagnosticStatus` entries — one per task — named
`"<node_name>: <task>"` — the node name without its leading slash, so
`G100P_1/eys3d_camera: device`. The module serial rides the separate
`hardware_id` field. The `device` task carries the overall health
summary:

| `level` | `message` | Meaning |
|---|---|---|
| `OK` | `streaming` | Every configured stream is delivering |
| `OK` | `streaming (paused — publish gated by operator)` | `pause` is in effect |
| `OK` | `standby (USB pipe closed by operator)` | `standby` is in effect |
| `ERROR` | `no frames flowing on any configured stream` | Every configured stream is below half its expected rate |
| `ERROR` | `camera disconnected; Linux device node not present` | USB lost; the watchdog will reconnect when the device returns |

Per-task `values` (KeyValue pairs):

**`device`** — connection + identity:

| Key | Description |
|---|---|
| `connection_state` | `streaming` or `disconnected` |
| `device_present` | `true` / `false`; the same state `connection_state` reports, as a boolean |
| `reconnect_attempts` | Attempts in the disconnect being recovered from; back to 0 once one succeeds |
| `usb_port` | sysfs interface path resolved at open (`2-3:1.0` style) |
| `serial_number` | Module SN reported by the SDK |
| `actual_fps` | FPS that `APC_OpenDevice2` negotiated. Not halved for interleave modes — the per-stream rate is half this |
| `stream_state` | `Active` / `Paused` / `Standby` — runtime tier reported by `pause` / `standby` services |

**`color`** and **`depth`** — per-stream throughput:

| Key | Description |
|---|---|
| `input_fps` | Frames per second received from the camera SDK over the past second. Independent of subscribers — measures camera / USB health |
| `publish_fps` | Frames per second actually emitted to the topic over the past second. Zero when no subscribers; matches `input_fps` when subscribers exist and the driver keeps up. A persistent gap indicates the driver is behind |
| `input_total` | Cumulative frames received from the SDK since open |
| `publish_total` | Cumulative frames published since open |
| `input_dropped` | Cumulative SDK-side drop counter (detected via serial-number gaps) |
| `decode_avg_ms` | (color only) Mean color decode time during the past second. Reported only when at least one frame was decoded in that period |
| `decode_max_ms` | (color only) Longest color decode time observed so far. Reported only when at least one frame has been decoded |

Their summary is `streaming`, `input rate below 50% of expected` (WARN),
`not configured (D-only mode)`, or `standby`.

**`pointcloud`** — reprojection + post-processing counters:

| Key | Description |
|---|---|
| `compute_status` | `active` (a subscriber is pulling clouds), `idle (no /depth/points subscriber)`, or `idle (never run ; no subscriber since start)`. Absent while disconnected, when the task carries only its summary |
| `publish_fps` | Clouds published per second over the past second. Zero when no subscriber |
| `compute_avg_ms` | Mean compute time per cloud during the past second (present when `active`) |
| `compute_max_ms` | Longest compute time observed so far |
| `publish_total` | Cumulative number of clouds published |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | Cumulative invocation counters for each enabled post-processing stage |

**`thermal`**:

| Key | Description |
|---|---|
| `temperature_c` | On-die thermal sensor (°C). Present only when the module has the sensor and the read succeeds; otherwise the key is absent and the task summary carries the reason |

A timer drives the Updater at `diagnostics_rate_hz` and publishes
whether or not anything is subscribed. The hot-path atomic counters
tick independently of it, so attaching a monitor at any moment shows
the cumulative values from open.

How to read it:

| `input_fps` | `publish_fps` | Meaning |
|---|---|---|
| ≈ expected | 0 | No subscribers; driver is idle |
| ≈ expected | ≈ input | Driver keeps up |
| ≈ expected | < input | Subscribers present, publish path is behind (decode too slow, or a subscriber can't keep up) |
| 0 | 0 | Camera / USB not delivering — check `device.connection_state` |

```bash
rosrun rqt_robot_monitor rqt_robot_monitor
rostopic echo /diagnostics
```

A bundled monitor subscribes to every stream and prints SDK / Pub / Rx
rates per stream alongside decode and compute timings:

```bash
rosrun eys3d_camera perf_monitor            # auto-detect namespace
rosrun eys3d_camera perf_monitor --ns /G100P_1 --interval 0.5
```

Each line shows, per stream: **SDK** (frames from the camera), **Pub**
(frames the driver emitted), **Rx** (frames the monitor received).

---

## Frame IDs

Two-layer TF tree, published latched on `/tf_static`:

```
<camera_name>_link                      ROS base axes (x fwd, y left, z up)
├── <camera_name>_left_color_frame      +baseline/2 on y
│   └── <camera_name>_left_color_optical_frame     REP-103 optical
├── <camera_name>_right_color_frame     -baseline/2 on y
│   └── <camera_name>_right_color_optical_frame
├── <camera_name>_depth_frame           +baseline/2 on y
│   └── <camera_name>_depth_optical_frame
└── <camera_name>_points_frame          +baseline/2 on y
```

Image messages stamp the `_optical_frame` leaves; `PointCloud2` stamps
`<camera_name>_points_frame`, which is already in base axes.

---

## Troubleshooting

| Symptom | Resolution |
|---|---|
| Camera open fails with "device busy", or opens and then never delivers a frame | Another process holds `/dev/videoN`. A second copy of this driver usually opens without an error and starves instead, logging `delivered no frame within 10 s of open` once a second. Find the holder with `lsof /dev/video*`; if it is a stale driver, stop it. |
| Permission error on open | Install the udev rule (see [Device Permissions](#device-permissions)) or join the `video` group. |
| RViz Image panel is blank | Set Fixed Frame to `<camera_name>_link` and confirm the topic matches the active `camera_name`. |
| No `right_color` topic | Only wide-color modes carry L\|R on one endpoint. Check `split_lr` in the mode catalogue. |
| Depth is all zeros | Pixels outside `depth_near_mm` / `depth_far_mm` are zeroed, so check the band covers your working distance. A near plane at or past the far plane is refused outright: the node logs `depth_near_mm must be < depth_far_mm` and idles, with the topics advertised but nothing published. |
| Self-calibration goal rejected | Either the module has no user calibration slot, or a session is already running. The node log gives the reason. |

---

## License

Apache-2.0 — see [LICENSE](LICENSE) and [NOTICE.md](NOTICE.md).
