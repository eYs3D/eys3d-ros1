# eYs3D 立體深度相機 ROS 1 驅動程式

[![ROS 1](https://img.shields.io/badge/ROS%201-Melodic%20%7C%20Noetic-blue)](https://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`eys3d_camera` 是 eYs3D 立體深度相機的官方 ROS 1 驅動程式，發布彩色影像、
深度影像與點雲，遵循 REP-103 frame tree，並直接建構在隨附的 eSPDI SDK 之上
——沒有中間包裝層。支援 ROS Melodic 與 Noetic。

### 支援的相機

| 型號 | Product code | USB | 狀態 |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量產 |
| **R77** | YX8072 | USB 2.0 | 量產 |
| **G62** | YX8081 | USB 2.0 | 量產 |

---

## 安裝

```bash
sudo apt install ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-nodelet \
                 ros-$ROS_DISTRO-dynamic-reconfigure \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-rviz
```

這些都不在 `ros-$ROS_DISTRO-ros-base` 內。per-model launch 預設
`urdf:=true` 與 `rviz:=true`，缺少 `robot_state_publisher`、`xacro`、`rviz`
會無法啟動；加上 `urdf:=false rviz:=false` 則不需要。

驅動程式以 GCC 7.5 以上建置，與隨附 eSPDI SDK 本身的工具鏈一致，因此
Melodic（Ubuntu 18.04）與 Noetic（Ubuntu 20.04）皆支援。更舊的發行版無法
支援：SDK 二進位檔本身就需要 GCC 7 的 C++ runtime。

將 package 放入 catkin workspace 的 `src/` 後建置：

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```


### 裝置權限

驅動以一般使用者身分開啟相機。若開啟裝置時出現權限錯誤，安裝隨附的
udev 規則，讓 eSPDI SDK 能存取該 USB 裝置：

```bash
sudo cp $(rospack find eys3d_camera)/udev/99-eys3d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

然後重新插拔相機。此規則授予對 eYs3D 裝置（USB vendor `3438`）的存取權限；
或將使用者加入 `video` 群組後重新登入。

---

## 快速開始

每個型號都有對應的 launch 捷徑，自動套用該機型的預設 video mode：

```bash
roslaunch eys3d_camera G100P.launch     # G100+：L'+D 1280x720 interleave，SDK 每串流 30 fps
roslaunch eys3d_camera R77.launch       # R77： L'+D 1280x920 彩色 + 640x460 深度 @ 30 fps
roslaunch eys3d_camera G62.launch       # G62： L'+D 640x480 @ 25 fps
```

要切換 mode 用 `mode_id:=<n>`；各機型完整 mode 列表位於
`launch/video_modes/<MODEL>.yaml`。

### 發布的 Topic

以 `camera_name:=G100P_1`（各機型預設值）為例：

| Topic | 型別 | 說明 |
|---|---|---|
| `/G100P_1/left_color/image_raw` | `sensor_msgs/Image` | 左眼彩色影像，固定 `rgb8`（YUYV 與 MJPEG 來源皆 inline 解碼；灰階感測模組以 R=G=B 輸出）|
| `/G100P_1/right_color/image_raw` | `sensor_msgs/Image` | 右眼彩色影像；僅在 video mode 帶有 L\|R 並排輸出時發布 |
| `/G100P_1/depth/image_raw` | `sensor_msgs/Image` (`16UC1`，mm，REP-118) | 深度（毫米） |
| `/G100P_1/depth/points` | `sensor_msgs/PointCloud2` | XYZ float32，已轉換為 ROS 基底軸（公尺）；當 `colored_pointcloud:=true` 時改為 XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 相機內參，每張 Image 一張（`header.stamp` 相同） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 健康狀態，1 Hz |

### 搭配 image_pipeline（image_proc、depth_image_proc 等）

每條串流會把 `<stream>/image_raw` 與 `<stream>/camera_info` 發布為同層
sibling，這正是 `image_transport::CameraSubscriber` 解析的排列方式。因此
`image_proc`、`depth_image_proc`、`stereo_image_proc` 與 `camera_calibration`
都能自動把影像與其 `CameraInfo` 配對——指向該串流的 namespace 即可直接運作：

```bash
ROS_NAMESPACE=/G100P_1/left_color rosrun image_proc image_proc
```

驅動本身已經能發布配準好的 XYZRGB 點雲（`colored_pointcloud:=true`），
因此只有在需要 `depth_image_proc` 的特定變體時才需要用它。

**不支援主機端校正。** 沒有 `camera_info_url` 參數、沒有
`camera_info_manager`、也沒有 `set_camera_info` service，所以
`rosrun camera_calibration cameracalibrator.py` 可以顯示與計算校正結果，
但無法存回本驅動。校正值是從模組自己的 flash 讀取的，飄移要靠
[自我校正](#自我校正) 修正，而非主機端檔案。

### CameraInfo 與畸變係數

每一份 `camera_info` 描述的都是它自己 topic 上發布的那張影像。

深度在所有模式下都是已校正的；彩色則在目錄名稱帶撇號的模式下已校正
（`L'+D`、`L'+R'+D`）。那些 topic 上的 `K` 是 `P` 的左 3×3、`D` 為零
（`plumb_bob`）、`R` 為單位矩陣 —— 相機已經移除畸變，沒有東西需要還原。

`L+R` 與 `L+R+D` 模式在彩色 topic 上發布的是原始感測器影像。那裡的
`K` 與 `D` 是發布解析度下的原廠鏡頭模型、`R` 是校正旋轉，`image_proc`
可以正常校正它們。請讀 `distortion_model` 而不要假設係數個數：driver 會
回報 `rational_polynomial` 八個係數或 `plumb_bob` 五個，依該台模組內
儲存的校正資料而定。

兩種情況下 `P` 都是投影矩陣，也是姿態估計（AprilTag、PnP、SLAM）應該
取用的內參來源。立體對的右相機 `P[3]` 是 `-fx × 基線`，單位為公尺。

---

## 設定

### 切換 video mode

```bash
roslaunch eys3d_camera G100P.launch mode_id:=7    # L'+D 640x480 interleave
roslaunch eys3d_camera R77.launch   mode_id:=4    # 純深度 640x460 @ 30 fps
roslaunch eys3d_camera G62.launch   mode_id:=3    # L'+D 320x240 @ 30 fps
```

節點啟動時會印出完整 mode 列表。`mode_id:=-1`（預設）會依協商到的 USB
連線速度自動選用該機型的 signature mode。

### Launch 參數

| 參數 | 預設值 | 說明 |
|---|---|---|
| `camera_name` | `<MODEL>_1` | ROS namespace 與 frame-id 前綴 |
| `mode_id` | `-1` | video mode 列表中的索引；`-1` = 自動 |
| `dev_serial_number` | `""` | 以序號子字串綁定 |
| `usb_port` | `""` | 以 USB 拓樸路徑綁定，例如 `2-3:1.0` |
| `depth_near_mm` / `depth_far_mm` | `-1` | 深度裁切上下限（mm）；`-1` = 該型號預設 |
| `ir_value` | `-1` | `-1` = 該型號預設（G100+/R77 = 3、G62 = 60）；`0` = 關閉 |
| `colored_pointcloud` | `false` | 從最近一張彩色幀取色，發布 XYZRGB |
| `spatial_filter` | `false` | 啟用視差域邊緣感知 IIR 濾波器 |
| `temporal_filter` | `false` | 啟用時間濾波器 |
| `hole_filling` | `0` | `0`=關閉、`1`=fill_from_left、`2`=farthest_from_around、`3`=nearest_from_around |
| `filter_profile` | `default` | 濾波器調校設定檔（`cfg/filter_profiles/<name>.yaml`）|
| `selfcal_enable` | `false` | 開放自我校正 action 與 commit service |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` 頻率；低於 `0.001` 即關閉 |
| `urdf` | `true` | 透過 `robot_state_publisher` 發布相機模型 |
| `rviz` | `true` | 是否以該機型的隨附版面開啟 RViz |
| `rviz_config` | `""` | 自訂 `.rviz` 版面路徑；留空則使用隨附版面 |
| `output` | `screen` | 節點輸出去向：`screen` 或 `log` |

### 執行中調整（dynamic_reconfigure）

影像與時間濾波器設定可在驅動執行中調整：

```bash
rosrun rqt_reconfigure rqt_reconfigure          # GUI
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera ir_value 5
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera auto_exposure false
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera exposure_time_step -8
```

可調整項目：`ir_value`、`auto_exposure`、`exposure_time_step`（`[-13, 3]`，
僅在 `auto_exposure` 關閉時生效）、`auto_white_balance`、
`power_line_frequency`（1 = 50 Hz、2 = 60 Hz）、四個 `temporal_filter_*`
數值，以及 `selfcal_profile`。

所有 eYs3D 模組出廠預設自動曝光與自動白平衡皆為開啟。驅動程式
**承襲韌體開機時的設定**，只寫回 launch 明確覆寫的項目，所以在 ROS 之外
調整過的設定能跨重啟保留。IR 是例外：投射器開機後預設為關閉，因此
`ir_value` 一定會套用。

### 後處理濾波器

三組可選濾波器，預設全關，且彼此獨立運作。

| 濾波器 | 切換方式 |
|---|---|
| `spatial_filter` | 僅 launch |
| `temporal_filter` | launch + 執行中（dynamic_reconfigure）|
| `hole_filling` | 僅 launch（`2` 為建議起始模式）|

調校值（`alpha` / `delta` / `magnitude` / `holes_fill` / `persistence`）
放在 `cfg/filter_profiles/<name>.yaml`。要自訂 profile，複製
`default.yaml` 並修改欄位，然後傳入 `filter_profile:=<name>`。

```bash
roslaunch eys3d_camera G100P.launch spatial_filter:=true temporal_filter:=true
roslaunch eys3d_camera G100P.launch spatial_filter:=true hole_filling:=2
roslaunch eys3d_camera G100P.launch spatial_filter:=true filter_profile:=indoor
```

---

## 執行中的串流控制

```bash
# 停止發布，但相機仍在 USB 上串流。驅動 CPU 降到接近零；
# 恢復後下一張 frame 就會出現。適合短暫中斷。
rosservice call /G100P_1/pause "data: true"
rosservice call /G100P_1/pause "data: false"

# 完全釋放 USB 管線。節點仍存活、topic 仍保持 advertised；
# 重新開啟需要數秒，視機型而定。可釋出 USB 頻寬。
rosservice call /G100P_1/standby "data: true"
rosservice call /G100P_1/standby "data: false"

# 透過 USB 重置相機（重新列舉）。節點會停止串流、發出重置，
# 然後由 watchdog 自動重連。用於在不重啟節點的情況下救回卡死的相機。
rosservice call /G100P_1/hw_reset
```

兩個串流控制都是彩色 + 深度成對作用：不支援單一串流切換，因為
interleave 模式需要 V4L2 串流的兩半同時作用。

### 熱插拔自動復原

1 Hz 的 watchdog 從開啟起就監看目前模式會跑的每一條串流。送出過 frame
的串流靜默 3 秒判定斷線;開啟後 10 秒內一張都沒送出的串流同樣判定斷線,
不論另一條是否正常。斷線後驅動每 2 秒輪詢一次,依綁定的序號 / USB port
重新開啟,重新發布 static TF,並重新套用 IR 與 CT/PU 設定。

> **RViz 與改名後的相機。** 隨附版面在每個 topic 路徑、`robot_description`
> 與 Fixed Frame 都寫著該機型的預設 `camera_name`。若改用其他名稱，版面會
> 依實際名稱改寫後從暫存目錄開啟，各面板一如預設情況正常顯示。若要改用
> 自己的版面，傳 `rviz_config:=<path>`，該檔案會原樣使用。

### 多台相機

驅動以機型的 USB PID 選擇裝置（G100+ = `0x0181`、R77 = `0x0180`、
G62 = `0x0183`）。**同型號**的兩台相機必須各自綁定：

```bash
roslaunch eys3d_camera G100P.launch camera_name:=G100P_left  dev_serial_number:=ABC123
roslaunch eys3d_camera G100P.launch camera_name:=G100P_right usb_port:=2-3:1.0

roslaunch eys3d_camera dual_G100P.launch          # 範例包裝
roslaunch eys3d_camera G100P_plus_R77.launch      # 混合機型不需綁定
```

---

## Nodelet（零複製）

這個驅動**本身就是** nodelet —— 獨立執行檔 `eys3d_camera_node` 只是把它
載入自己的 process。改為載入共用的 manager，該 manager 內的其他 nodelet
就能以驅動發布時的同一個 `shared_ptr` 收到影像與點雲訊息，沒有序列化、
也沒有複製：

```bash
roslaunch eys3d_camera G100P_nodelet.launch
rosrun nodelet nodelet load my_pkg/MyNodelet /eys3d_nodelet_manager
```

請將 manager 的 `num_worker_threads` 保持在 4 以上：光是這個驅動，就需要
在其中一個 callback 卡在 SDK 呼叫時，讓 service、timer、action 與
reconfigure 的 callback 仍能繼續推進。

---

## 自我校正

選用的 in-stream 自我校正，可重新對齊雙目、救回校正已飄移模組的深度
填充率。逐次 launch 啟用：

```bash
roslaunch eys3d_camera G100P.launch selfcal_enable:=true
```

執行時相機必須**正在串流含深度的 mode，並對著工作距離內一般、有紋理的
場景**——校正器是靠量測深度覆蓋率運作的，因此需要有效的深度作為依據。

```bash
# 跑一次 session（約 20-30 秒收斂，之後有一段簡短的複檢）。
rostopic pub --once /G100P_1/selfcal/run/goal \
  eys3d_camera/SelfCalActionGoal "goal: {auto_commit_shift_px: 0.25}"

# 或用正規的 action client 操作：
rosrun actionlib axclient.py /G100P_1/selfcal/run

# 將保留下來的結果寫入 flash（只有在 auto-commit 沒觸發時才需要）。
# 請檢查回應的 success 欄位——若沒有保留中的結果會是 false。
rosservice call /G100P_1/selfcal/commit
```

一次 run 的結局同時取決於校正器的 outcome **以及一段實測 A/B 複檢**
——在同一場景下比較新對齊與執行前對齊的深度填充率：

- **確認變好** —— 複檢確認的 `SUCCESS` 會生效（`applied: true`）。若
  `auto_commit_shift_px` 為正值且 `cy_shift_px` 達到該值，這次 run 會
  **寫入** flash（`committed: true`）；否則只是**保留在線上**但為揮發性
  ——呼叫 `selfcal/commit` 才能撐過斷電。
- **已是最佳**（`NO_CHANGE`）—— 沒有任何變更、套用或寫入。
- **變差 / 無法驗證 / 失敗** —— 相機會**回退**（`reverted: true`）。
  此處若為 `reverted: false`，代表回退本身失敗，相機停在被否決的對齊上，
  要斷電才會恢復；`message` 會說明。

`cy_shift_px` 是直接從硬體讀回的實測垂直位移量，也是 auto-commit 閘門的
比較對象。步進固定為 **0.25 px**，修正量上限為 **5.0 px**。

> `auto_commit_shift_px` **必須為正值才會 auto-commit。** ROS 1 的
> message 欄位沒有預設值，省略的欄位會以 `0.0` 送達；閘門將 `<= 0` 視為
> 永不 commit，因此省略的情況就是安全的那一邊。若確實需要 auto-commit，
> 建議值為 `0.25`——也就是一個校正步進。

### Action Result

| 欄位 | 意義 |
|---|---|
| `outcome` | `SUCCESS` / `NO_CHANGE` / `INSUFFICIENT_INPUT`（有效深度不足）/ `TIMEOUT`（時限內未收斂）/ `FAILED` |
| `cy_shift_px` | 實測的垂直 cy 位移量（像素）——auto-commit 的判斷依據 |
| `recheck_verdict` | A/B 複檢結果：`improved` / `worse` / `inconclusive` / `skipped` |
| `recheck_ratio_before` / `_after` | 執行前 / 收斂後對齊的填充率 |
| `applied` / `reverted` / `committed` | 這次 run 在相機上留下了什麼 |
| `message` | 人類可讀的摘要 |

`commit` 是唯一會寫入 flash 的步驟；原廠校正會保留為備份，永遠不會被
覆蓋。已保留但未 commit 的結果只存在於相機的暫存器中，因此斷電或
`hw_reset` 就會清回已儲存的校正值。這讓「線上校正、永不寫 flash」成為
一個安全的預設流程。

有幾點值得知道：

- **重複執行會疊加。** 在 commit 或斷電之前重跑，是疊在上一次保留的結果
  之上，而不是疊在 flash 上。要乾淨的基準請先 commit 或斷電。
- **不符資格或已在執行 → 是 rejected，不是 failed。** 不支援自我校正的
  模組，以及第二個並行的 run，都會被直接拒絕——action client 會看到
  goal 被 rejected 且沒有 result；原因請看節點 log。
- **session 無法中斷。** cancel 會被拒絕，而控制類 service
  （`pause` / `standby` / `hw_reset`）在執行期間也會拒絕。它很短，而且
  結果變差時會自動回退。
- **每個 process 一次只能跑一個 session。** 共用同一個 nodelet manager
  的相機只能一台一台校正；位於不同 process 的相機則互不影響。

---

## Diagnostics（`/diagnostics`）

每秒一筆 `DiagnosticArray`。每一筆內含 5 個 `DiagnosticStatus`（每
task 一個），名稱為 `"<node_name>: <task>"` —— 去掉開頭斜線的節點名稱，
例如 `G100P_1/eys3d_camera: device`；模組序號放在另一個獨立的
`hardware_id` 欄位。整體健康狀態由 `device` task 承載：

| `level` | `message` | 意義 |
|---|---|---|
| `OK` | `streaming` | 每條設定中的串流都有在送 |
| `OK` | `streaming (paused — publish gated by operator)` | `pause` 生效中 |
| `OK` | `standby (USB pipe closed by operator)` | `standby` 生效中 |
| `ERROR` | `no frames flowing on any configured stream` | 每條設定中的串流都低於預期速率的一半 |
| `ERROR` | `camera disconnected; Linux device node not present` | USB 斷線；watchdog 會在裝置回插時自動重連 |

各 task 的 `values` 鍵值對：

**`device`** — 連線與識別：

| Key | 說明 |
|---|---|
| `connection_state` | `streaming` 或 `disconnected` |
| `device_present` | `true` / `false`，與 `connection_state` 同一個狀態的布林形式 |
| `reconnect_attempts` | 目前這次斷線的重連次數；成功一次後歸零 |
| `usb_port` | open 時解析到的 sysfs 介面路徑（如 `2-3:1.0`）|
| `serial_number` | SDK 回報的模組序號 |
| `actual_fps` | `APC_OpenDevice2` 協商到的 fps。interleave mode 不會除以二 —— 每串流速率是這個值的一半 |
| `stream_state` | `Active` / `Paused` / `Standby` — `pause` / `standby` service 控制的執行時狀態 |

**`color`** 與 **`depth`** — 每條串流吞吐：

| Key | 說明 |
|---|---|
| `input_fps` | 過去 1 秒從 SDK 收到的幀數。與訂閱者狀態無關 — 用於判斷相機 / USB 健康度 |
| `publish_fps` | 過去 1 秒實際發布到 topic 的幀數。沒訂閱者時為 0；有訂閱者且 driver 跟得上時 ≈ `input_fps`；持續低於代表 driver 落後 |
| `input_total` | 自 open 起累積從 SDK 收到的幀數 |
| `publish_total` | 自 open 起累積發布的幀數 |
| `input_dropped` | 累積 SDK 端掉幀數（由 serial-number 不連續偵測）|
| `decode_avg_ms` | （僅 `color`）過去 1 秒 color 解碼平均耗時。當下 1 秒內有解碼過 frame 才會出現 |
| `decode_max_ms` | （僅 `color`）至今觀察到的最長 color 解碼耗時。曾經解碼過 frame 才會出現 |

兩者的摘要為 `streaming`、`input rate below 50% of expected`（WARN）、
`not configured (D-only mode)` 或 `standby`。

**`pointcloud`** — 點雲投影 + 後處理計數：

| Key | 說明 |
|---|---|
| `compute_status` | `active`（有訂閱者拉點雲）、`idle (no /depth/points subscriber)` 或 `idle (never run ; no subscriber since start)`。斷線時不會出現此欄位，該 task 只帶 summary |
| `publish_fps` | 過去 1 秒點雲發布幀數。沒訂閱者時為 0 |
| `compute_avg_ms` | 過去 1 秒點雲計算平均耗時（`active` 時出現）|
| `compute_max_ms` | 至今觀察到的最長點雲計算耗時 |
| `publish_total` | 累積發布的點雲數 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各後處理階段的累積套用次數 |

**`thermal`**：

| Key | 說明 |
|---|---|
| `temperature_c` | 晶片溫度（°C）。僅在機型具備感測器且讀取成功時發布；否則不會出現此欄位，原因寫在該任務的 summary |

由 timer 以 `diagnostics_rate_hz` 驅動 Updater，無論有沒有訂閱者都會
發布。hot-path 的 atomic counter 獨立累計，所以隨時掛上監控都能看到自
open 起的累積值。

判讀方式：

| `input_fps` | `publish_fps` | 意義 |
|---|---|---|
| ≈ 預期 | 0 | 沒有訂閱者，driver idle |
| ≈ 預期 | ≈ input | driver 跟得上 |
| ≈ 預期 | < input | 有訂閱者，但發布路徑落後（解碼太慢，或訂閱端跟不上）|
| 0 | 0 | 相機 / USB 沒在送資料 — 看 `device.connection_state` |

```bash
rosrun rqt_robot_monitor rqt_robot_monitor
rostopic echo /diagnostics
```

內附監控工具：自動訂閱所有 stream，並把 SDK / Pub / Rx 三條 rate
跟解碼 / 計算耗時並列顯示：

```bash
rosrun eys3d_camera perf_monitor            # 自動偵測 namespace
rosrun eys3d_camera perf_monitor --ns /G100P_1 --interval 0.5
```

每一行按串流顯示：**SDK**（相機送來的 frame）、**Pub**（驅動實際發出的
frame）、**Rx**（monitor 收到的 frame）。

---

## Frame ID

兩層 TF tree，以 latched 方式發布於 `/tf_static`：

```
<camera_name>_link                      ROS 基底軸（x 前、y 左、z 上）
├── <camera_name>_left_color_frame      y 軸 +baseline/2
│   └── <camera_name>_left_color_optical_frame     REP-103 光學座標
├── <camera_name>_right_color_frame     y 軸 -baseline/2
│   └── <camera_name>_right_color_optical_frame
├── <camera_name>_depth_frame           y 軸 +baseline/2
│   └── <camera_name>_depth_optical_frame
└── <camera_name>_points_frame          y 軸 +baseline/2
```

影像訊息標記 `_optical_frame` 葉節點；`PointCloud2` 標記
`<camera_name>_points_frame`，該 frame 已經是基底軸。

---

## 疑難排解

| 症狀 | 處理方式 |
|---|---|
| 開啟相機失敗顯示 "device busy"，或開啟後始終收不到影格 | 有其他 process 佔用 `/dev/videoN`。第二個本驅動的實例通常不會報錯,而是開起來後餓死,每秒記錄一次 `delivered no frame within 10 s of open`。用 `lsof /dev/video*` 找出佔用者;若是殘留的驅動就停掉它。 |
| 開啟時出現權限錯誤 | 安裝 udev 規則（見[裝置權限](#裝置權限)）或加入 `video` 群組。 |
| RViz Image 面板空白 | 將 Fixed Frame 設為 `<camera_name>_link`，並確認 topic 與當前的 `camera_name` 相符。 |
| 沒有 `right_color` topic | 只有 wide-color 模式會在單一端點帶 L\|R。請檢查 mode 列表中的 `split_lr`。 |
| 深度全為零 | 落在 `depth_near_mm` / `depth_far_mm` 之外的像素會被歸零,請確認範圍涵蓋你的工作距離。近平面若等於或超過遠平面會被直接拒絕:節點記錄 `depth_near_mm must be < depth_far_mm` 後進入閒置,topic 仍會 advertise 但不會有資料。 |
| 自我校正 goal 被 rejected | 不是該模組沒有 user calibration slot，就是已有 session 在執行。原因請看節點 log。 |

---

## 授權

Apache-2.0 —— 見 [LICENSE](../LICENSE) 與 [NOTICE.md](../NOTICE.md)。
