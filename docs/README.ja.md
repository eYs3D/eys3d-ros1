# eYs3D ステレオ深度カメラ用 ROS 1 ドライバ

[![ROS 1](https://img.shields.io/badge/ROS%201-Melodic%20%7C%20Noetic-blue)](https://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`eys3d_camera` は eYs3D ステレオ深度カメラ用の公式 ROS 1 ドライバです。
カラー画像・深度画像・点群を標準 REP-103 frame tree で配信します。同梱の
eSPDI SDK の上に直接実装されており、ラッパ層はありません。ROS Melodic と
Noetic に対応します。

### 対応カメラ

| 型番 | Product code | USB | 状態 |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量産 |
| **R77** | YX8072 | USB 2.0 | 量産 |
| **G62** | YX8081 | USB 2.0 | 量産 |

---

## インストール

```bash
sudo apt install ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-nodelet \
                 ros-$ROS_DISTRO-dynamic-reconfigure \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-rviz
```

いずれも `ros-$ROS_DISTRO-ros-base` には含まれません。機種別 launch は
`urdf:=true` と `rviz:=true` がデフォルトのため、`robot_state_publisher`、
`xacro`、`rviz` が無いと起動しません。`urdf:=false rviz:=false` を渡せば
不要です。

本ドライバは GCC 7.5 以降でビルドします。これは同梱の eSPDI SDK 自身の
ツールチェーンと一致するため、Melodic（Ubuntu 18.04）と Noetic
（Ubuntu 20.04）の両方に対応します。より古いディストリビューションは
対象外です: SDK バイナリ自体が GCC 7 の C++ ランタイムを必要とします。

パッケージを catkin workspace の `src/` に置いてビルドします:

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```


### デバイス権限

ドライバは一般ユーザ権限でカメラを開きます。デバイスのオープンが権限
エラーで失敗する場合は、同梱の udev ルールをインストールして eSPDI SDK が
USB デバイスにアクセスできるようにします:

```bash
sudo cp $(rospack find eys3d_camera)/udev/99-eys3d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

その後カメラを挿し直してください。このルールは eYs3D デバイス
（USB vendor `3438`）へのアクセスを許可します。あるいはユーザを `video`
グループに追加して再ログインしても構いません。

---

## クイックスタート

各機種にはデフォルトモードを開く launch ショートカットがあります:

```bash
roslaunch eys3d_camera G100P.launch     # G100+: L'+D 1280x720 interleave、SDK 各ストリーム 30 fps
roslaunch eys3d_camera R77.launch       # R77:   L'+D 1280x920 カラー + 640x460 深度 @ 30 fps
roslaunch eys3d_camera G62.launch       # G62:   L'+D 640x480 @ 25 fps
```

モードの変更は `mode_id:=<n>`。機種別の全モード一覧は
`launch/video_modes/<MODEL>.yaml` にあります。

### 配信される Topic

`camera_name:=G100P_1`（機種別デフォルト）の場合:

| Topic | 型 | 説明 |
|---|---|---|
| `/G100P_1/left_color/image_raw` | `sensor_msgs/Image` | 左カラー画像、常に `rgb8`（YUYV と MJPEG はインラインでデコード、グレースケール系モジュールは R=G=B で出力）|
| `/G100P_1/right_color/image_raw` | `sensor_msgs/Image` | 右カラー画像。1 エンドポイントに L\|R を載せる wide-color モードでのみ配信 |
| `/G100P_1/depth/image_raw` | `sensor_msgs/Image` (`16UC1`, mm, REP-118) | 深度(ミリメートル) |
| `/G100P_1/depth/points` | `sensor_msgs/PointCloud2` | ROS 基準軸の XYZ float32(メートル)。`colored_pointcloud:=true` では XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 内部パラメータ。対応する Image 1 枚につき 1 つ(`header.stamp` は同一) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ヘルス情報、1 Hz |

### image_pipeline との併用（image_proc、depth_image_proc など）

各ストリームは `<stream>/image_raw` と `<stream>/camera_info` を兄弟階層
として配信します。これは `image_transport::CameraSubscriber` が解決する
レイアウトそのものです。したがって `image_proc`、`depth_image_proc`、
`stereo_image_proc`、`camera_calibration` は画像と `CameraInfo` を自動的に
対応付けます。ストリームの namespace を指すだけで動作します:

```bash
ROS_NAMESPACE=/G100P_1/left_color rosrun image_proc image_proc
```

ドライバ自身が既にレジストレーション済みの XYZRGB 点群を配信できる
（`colored_pointcloud:=true`）ため、`depth_image_proc` が必要になるのは
その固有のバリエーションを使いたい場合だけです。

**ホスト側キャリブレーションには対応していません。** `camera_info_url`
パラメータも `camera_info_manager` も `set_camera_info` service もないため、
`rosrun camera_calibration cameracalibrator.py` は表示と計算はできても、
本ドライバに対して保存することはできません。キャリブレーション値は
モジュール自身の flash から読み出され、ずれはホスト側ファイルではなく
[セルフキャリブレーション](#セルフキャリブレーション) で補正します。

### CameraInfo と歪み係数

各 `camera_info` は、その topic で配信される画像そのものを記述します。

深度はすべてのモードで補正済みです。カラーはカタログ名にアポストロフィが
付くモード（`L'+D`、`L'+R'+D`）で補正済みです。それらの topic では `K` は
`P` の左 3×3、`D` はゼロ（`plumb_bob`）、`R` は単位行列になります。カメラが
すでに歪みを取り除いているため、戻すものが残っていません。

`L+R` と `L+R+D` モードはカラー topic に生のセンサー画像を配信します。
そこでの `K` と `D` は配信解像度における工場出荷時のレンズモデル、`R` は
補正回転で、`image_proc` がそのまま補正できます。係数の個数を仮定せず
`distortion_model` を読んでください。ドライバは `rational_polynomial` 8 個
または `plumb_bob` 5 個を、その個体に保存された校正値に従って報告します。

いずれの場合も `P` が投影行列であり、姿勢推定（AprilTag、PnP、SLAM）で
使うべき内部パラメータです。ステレオ対では右カメラの `P[3]` が
`-fx × ベースライン`（メートル）です。

---

## 設定

### ビデオモードの切り替え

```bash
roslaunch eys3d_camera G100P.launch mode_id:=7    # L'+D 640x480 interleave
roslaunch eys3d_camera R77.launch   mode_id:=4    # 深度のみ 640x460 @ 30 fps
roslaunch eys3d_camera G62.launch   mode_id:=3    # L'+D 320x240 @ 30 fps
```

ノードは起動時に全モード表を出力します。`mode_id:=-1`（デフォルト）は
ネゴシエートされた USB リンクに対する機種の signature モードを自動選択
します。

### Launch 引数

| 引数 | デフォルト | 説明 |
|---|---|---|
| `camera_name` | `<MODEL>_1` | ROS namespace と frame-id の接頭辞 |
| `mode_id` | `-1` | 機種別カタログ内のモード番号。`-1` = 自動 |
| `dev_serial_number` | `""` | シリアル番号の部分一致でバインド |
| `usb_port` | `""` | USB トポロジでバインド（例 `2-3:1.0`）|
| `depth_near_mm` / `depth_far_mm` | `-1` | 深度のクリップ範囲(mm)。`-1` = 機種別デフォルト |
| `ir_value` | `-1` | `-1` = 機種別デフォルト(G100+/R77 = 3、G62 = 60)。`0` = オフ |
| `colored_pointcloud` | `false` | 最新カラーフレームから色をサンプルして XYZRGB を配信 |
| `spatial_filter` | `false` | 視差領域のエッジ保持 IIR フィルタを有効化 |
| `temporal_filter` | `false` | 時間フィルタを有効化 |
| `hole_filling` | `0` | `0`=オフ、`1`=fill_from_left、`2`=farthest_from_around、`3`=nearest_from_around |
| `filter_profile` | `default` | フィルタ調整プロファイル(`cfg/filter_profiles/<name>.yaml`) |
| `selfcal_enable` | `false` | セルフキャリブレーションの action と commit service を公開 |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` のレート。`0.001` 未満で無効化 |
| `urdf` | `true` | `robot_state_publisher` でカメラモデルを配信 |
| `rviz` | `true` | 機種に対応する同梱レイアウトで RViz を起動するか |
| `rviz_config` | `""` | 独自の `.rviz` レイアウトのパス。空なら同梱レイアウト |
| `output` | `screen` | ノード出力先: `screen` または `log` |

### 実行中の調整（dynamic_reconfigure）

画像設定と時間フィルタは、ドライバの実行中に変更できます:

```bash
rosrun rqt_reconfigure rqt_reconfigure          # GUI
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera ir_value 5
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera auto_exposure false
rosrun dynamic_reconfigure dynparam set /G100P_1/eys3d_camera exposure_time_step -8
```

変更可能な項目: `ir_value`、`auto_exposure`、`exposure_time_step`
（`[-13, 3]`、`auto_exposure` がオフのときのみ有効）、
`auto_white_balance`、`power_line_frequency`（1 = 50 Hz、2 = 60 Hz）、
4 つの `temporal_filter_*` 値、および `selfcal_profile`。

eYs3D の全モジュールは自動露出と自動ホワイトバランスが有効な状態で出荷
されます。ドライバは**ファームウェアの起動時状態を引き継ぎ**、launch で
明示的に上書きされたものだけを書き戻すため、ROS の外で行った設定は再起動
後も保持されます。IR だけは例外で、プロジェクタは起動時オフのため
`ir_value` は必ず適用されます。

### 後処理フィルタ

3 つのオプションフィルタがあり、いずれもデフォルトはオフで、互いに独立
して動作します。

| フィルタ | 切り替え方法 |
|---|---|
| `spatial_filter` | launch のみ |
| `temporal_filter` | launch + 実行中（dynamic_reconfigure）|
| `hole_filling` | launch のみ（`2` が推奨の開始モード）|

調整値（`alpha` / `delta` / `magnitude` / `holes_fill` / `persistence`）は
`cfg/filter_profiles/<name>.yaml` にあります。`default.yaml` をコピーして
フィールドを編集すればプロファイルを作成でき、`filter_profile:=<name>` で
指定します。

```bash
roslaunch eys3d_camera G100P.launch spatial_filter:=true temporal_filter:=true
roslaunch eys3d_camera G100P.launch spatial_filter:=true hole_filling:=2
roslaunch eys3d_camera G100P.launch spatial_filter:=true filter_profile:=indoor
```

---

## 実行中のストリーム制御

```bash
# 配信を停止するが、カメラは USB 上でストリーミングを続ける。ドライバの
# CPU 使用率はほぼゼロになり、再開は次のフレームから。短い中断向け。
rosservice call /G100P_1/pause "data: true"
rosservice call /G100P_1/pause "data: false"

# USB パイプを完全に解放する。ノードは生存し topic も advertise された
# まま。再オープンには機種に応じて数秒かかる。USB 帯域を空けられる。
rosservice call /G100P_1/standby "data: true"
rosservice call /G100P_1/standby "data: false"

# USB 経由でカメラをリセット(再列挙)する。ノードはストリームを停止し、
# リセットを発行し、ウォッチドッグが自動的に再接続する。ノードを再起動
# せずにハングしたカメラを復旧させる用途。
rosservice call /G100P_1/hw_reset
```

どちらのストリーム制御もカラー + 深度のペアに同時に作用します。ストリーム
個別の切り替えには対応していません。interleave モードでは V4L2 ストリームの
両方が同時にアクティブである必要があるためです。

### ホットプラグ自動復旧

1 Hz のウォッチドッグは、オープン以降、現在のモードが動かす各ストリームを
監視します。フレームを出したストリームが 3 秒無音になった場合、およびオープン
から 10 秒以内に 1 フレームも出さないストリームがある場合は、もう一方が流れて
いても切断と判定します。切断後は 2 秒ごとにポーリングし、固定したシリアル /
USB ポートで再オープンし、static TF を再配信し、IR と CT/PU 設定を再適用します。

> **RViz とカメラ名の変更。** 同梱レイアウトは、各 topic のパス、
> `robot_description`、Fixed Frame のいずれにも機種のデフォルト
> `camera_name` を書き込んでいます。別の名前で起動した場合はレイアウトが
> その名前に書き換えられ、一時ディレクトリから開かれるため、各パネルは
> デフォルトのときと同じように表示されます。独自のレイアウトを使うには
> `rviz_config:=<path>` を渡してください。そのファイルはそのまま使われます。

### 複数カメラ

ドライバは機種の USB PID でデバイスを選択します（G100+ = `0x0181`、
R77 = `0x0180`、G62 = `0x0183`）。**同一機種**のカメラ 2 台は個別に
バインドする必要があります:

```bash
roslaunch eys3d_camera G100P.launch camera_name:=G100P_left  dev_serial_number:=ABC123
roslaunch eys3d_camera G100P.launch camera_name:=G100P_right usb_port:=2-3:1.0

roslaunch eys3d_camera dual_G100P.launch          # サンプルのラッパ
roslaunch eys3d_camera G100P_plus_R77.launch      # 機種混在ならバインド不要
```

---

## Nodelet（ゼロコピー）

このドライバは**それ自体が nodelet** です。スタンドアロンの実行ファイル
`eys3d_camera_node` は、それを自身のプロセスにロードしているだけです。
代わりに共有 manager にロードすれば、その manager 内の他の nodelet は、
ドライバが配信したまさにその `shared_ptr` として画像と点群のメッセージを
受け取れます。シリアライズもコピーもありません:

```bash
roslaunch eys3d_camera G100P_nodelet.launch
rosrun nodelet nodelet load my_pkg/MyNodelet /eys3d_nodelet_manager
```

manager の `num_worker_threads` は 4 以上に保ってください。このドライバ
単体でも、いずれかが SDK 呼び出しでブロックしている間に service、timer、
action、reconfigure の各コールバックが進行できる必要があります。

---

## セルフキャリブレーション

オプションのストリーム内セルフキャリブレーションは、キャリブレーションが
ずれたモジュールのステレオペアを再整列させ、深度の充填率を回復させます。
launch ごとに有効化します:

```bash
roslaunch eys3d_camera G100P.launch selfcal_enable:=true
```

実行時はカメラが**深度を含むモードでストリーミングしており、作動距離に
ある通常のテクスチャのあるシーンに向いている**必要があります。
キャリブレータは深度のカバレッジを測定するため、有効な深度が得られている
ことが前提です。

```bash
# セッションを 1 回実行(収束まで約 20-30 秒、その後に短い再チェック)。
rostopic pub --once /G100P_1/selfcal/run/goal \
  eys3d_camera/SelfCalActionGoal "goal: {auto_commit_shift_px: 0.25}"

# あるいは正式な action client から操作:
rosrun actionlib axclient.py /G100P_1/selfcal/run

# 保持された結果を flash に書き込む(auto-commit が発火しなかった場合のみ必要)。
# レスポンスの success フィールドを確認してください。保持結果がなければ false です。
rosservice call /G100P_1/selfcal/commit
```

実行の結末は、キャリブレータの outcome **と実測の A/B 再チェック**の両方で
決まります。再チェックは同一シーン上で、新しい整列と実行前の整列の深度
充填率を比較します:

- **改善が確認された** —— 再チェックが裏付けた `SUCCESS` は反映されます
  （`applied: true`）。`auto_commit_shift_px` が正の値で `cy_shift_px` が
  それに達した場合、その実行は flash に**書き込みます**
  （`committed: true`）。そうでなければ**ライブで保持**されるだけの
  揮発状態です。電源断をまたいで残すには `selfcal/commit` を呼びます。
- **既に最適**（`NO_CHANGE`）—— 変更も適用も書き込みも行われません。
- **悪化 / 検証不能 / 失敗** —— カメラは**ロールバック**されます
  （`reverted: true`）。ここが `reverted: false` の場合はロールバック
  自体が失敗しており、電源を入れ直すまで却下されたアライメントのまま
  です。`message` がその旨を伝えます。

`cy_shift_px` はハードウェアから直接読み出した実測の垂直方向シフト量で、
auto-commit の判定対象そのものです。ステップは **0.25 px** 固定、補正量の
上限は **5.0 px** です。

> `auto_commit_shift_px` は **auto-commit させるには正の値である必要が
> あります。** ROS 1 の message フィールドにはデフォルト値がないため、
> 省略されたフィールドは `0.0` として届きます。ゲートは `<= 0` を
> 「commit しない」と扱うので、省略した場合が安全側になります。
> auto-commit を実際に使いたい場合の推奨値は `0.25`、つまり
> キャリブレーション 1 ステップ分です。

### Action Result

| フィールド | 意味 |
|---|---|
| `outcome` | `SUCCESS` / `NO_CHANGE` / `INSUFFICIENT_INPUT`(有効な深度が不足) / `TIMEOUT`(時間内に収束せず) / `FAILED` |
| `cy_shift_px` | 実測の垂直 cy シフト量(ピクセル) —— auto-commit の判定基準 |
| `recheck_verdict` | A/B 再チェック: `improved` / `worse` / `inconclusive` / `skipped` |
| `recheck_ratio_before` / `_after` | 実行前 / 収束後の整列での充填率 |
| `applied` / `reverted` / `committed` | その実行がカメラに何を残したか |
| `message` | 人が読める要約 |

flash に書き込むのは `commit` だけです。工場出荷時のキャリブレーションは
バックアップとして保持され、上書きされることはありません。保持されたが
commit されていない結果はカメラのレジスタ上にのみ存在するため、電源断や
`hw_reset` で保存済みキャリブレーションに戻ります。これにより
「ライブで校正し、flash には書かない」という運用が安全なデフォルトに
なります。

知っておくとよい点:

- **繰り返し実行すると積み重なります。** commit または電源断の前に再実行
  すると、flash ではなく直前に保持された結果の上に積まれます。きれいな
  基準から始めるには、先に commit するか電源を入れ直してください。
- **対象外または実行中の場合は failed ではなく rejected です。**
  セルフキャリブレーションに対応しないモジュール、および 2 つ目の同時実行は
  いずれも即座に拒否されます。action client には result なしで goal が
  rejected として見えます。理由はノードのログを確認してください。
- **セッションは中断できません。** cancel は拒否され、制御系 service
  （`pause` / `standby` / `hw_reset`）も実行中は拒否します。実行は短時間で、
  結果が悪化した場合は自動的にロールバックされます。
- **1 プロセスにつき 1 セッションです。** nodelet manager を共有する
  カメラは 1 台ずつ校正されます。別プロセスのカメラは影響を受けません。

---

## Diagnostics（`/diagnostics`）

毎秒 1 件 `DiagnosticArray` を配信します。1 件あたり 5 つの
`DiagnosticStatus`（タスクごとに 1 つ）を含み、名前は
`"<node_name>: <task>"` 形式 —— 先頭のスラッシュを除いたノード名で、
例えば `G100P_1/eys3d_camera: device`。モジュールのシリアルは別の
`hardware_id` フィールドに入ります。全体の健全性は `device` タスクが
運びます:

| `level` | `message` | 意味 |
|---|---|---|
| `OK` | `streaming` | 設定された各ストリームが配信中 |
| `OK` | `streaming (paused — publish gated by operator)` | `pause` が有効 |
| `OK` | `standby (USB pipe closed by operator)` | `standby` が有効 |
| `ERROR` | `no frames flowing on any configured stream` | 設定された全ストリームが期待レートの半分未満 |
| `ERROR` | `camera disconnected; Linux device node not present` | USB 切断、ウォッチドッグがデバイス復帰時に自動再接続 |

タスクごとの `values` キーバリュー:

**`device`** — 接続 + 識別情報:

| Key | 説明 |
|---|---|
| `connection_state` | `streaming` または `disconnected` |
| `device_present` | `true` / `false`、`connection_state` と同じ状態の真偽値 |
| `reconnect_attempts` | 復旧中の切断における試行回数。成功すると 0 に戻る |
| `usb_port` | open 時に解決した sysfs インタフェースパス（例 `2-3:1.0`）|
| `serial_number` | SDK が返したモジュールシリアル |
| `actual_fps` | `APC_OpenDevice2` がネゴシエートした fps。interleave mode でも半分にはならず、ストリーム単体のレートはこの半分 |
| `stream_state` | `Active` / `Paused` / `Standby` — `pause` / `standby` サービスが制御する実行時状態 |

**`color`** と **`depth`** — ストリーム別スループット:

| Key | 説明 |
|---|---|
| `input_fps` | 直近 1 秒で SDK から受信したフレーム数。購読者の有無に依存せず、カメラ / USB 健全性の指標 |
| `publish_fps` | 直近 1 秒で実際にトピックへ発行したフレーム数。購読者なしなら 0、追従できていれば `input_fps` と一致、継続的に下回る場合はドライバ側が遅れている |
| `input_total` | open 以降に SDK から受信した累積フレーム数 |
| `publish_total` | open 以降に発行した累積フレーム数 |
| `input_dropped` | SDK 側の累積ドロップ数（シリアル番号の不連続から検出） |
| `decode_avg_ms` | （`color` のみ）直近 1 秒の color デコード平均時間。直近 1 秒内にデコードが発生した場合のみ出力 |
| `decode_max_ms` | （`color` のみ）これまで観測した最長 color デコード時間。一度でもデコードが発生した場合のみ出力 |

サマリは `streaming`、`input rate below 50% of expected`（WARN）、
`not configured (D-only mode)`、`standby` のいずれかです。

**`pointcloud`** — 投影 + 後処理カウンタ:

| Key | 説明 |
|---|---|
| `compute_status` | `active`（購読者あり）、`idle (no /depth/points subscriber)`、`idle (never run ; no subscriber since start)`。切断中はこのキー自体が現れず、タスクは summary のみを運ぶ |
| `publish_fps` | 直近 1 秒の点群発行数。購読者なしで 0 |
| `compute_avg_ms` | 直近 1 秒の点群計算平均時間（`active` のとき出力） |
| `compute_max_ms` | これまで観測した最長点群計算時間 |
| `publish_total` | 累積発行点群数 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各後処理段の累積適用回数 |

**`thermal`**:

| Key | 説明 |
|---|---|
| `temperature_c` | チップ温度（°C）。センサーを備えたモデルで読み取りに成功した場合のみ発行され、それ以外はキー自体が現れず、理由はタスクの summary に入る |

タイマーが `diagnostics_rate_hz` で Updater を駆動し、購読者の有無に
かかわらず発行します。ホットパスの atomic カウンタはそれとは独立に
動作するため、後から監視を接続しても open 以降の累積値を確認できます。

判読方法:

| `input_fps` | `publish_fps` | 意味 |
|---|---|---|
| ≈ 期待値 | 0 | 購読者なし、ドライバはアイドル |
| ≈ 期待値 | ≈ input | ドライバが追従できている |
| ≈ 期待値 | < input | 購読者あり、配信パスが遅れている（デコードが遅い、または購読側が追従できていない）|
| 0 | 0 | カメラ / USB がデータを送出していない — `device.connection_state` を確認 |

```bash
rosrun rqt_robot_monitor rqt_robot_monitor
rostopic echo /diagnostics
```

同梱モニタは全ストリームを自動購読し、SDK / Pub / Rx の各レートと
デコード / 計算所要時間を並べて表示します:

```bash
rosrun eys3d_camera perf_monitor            # namespace 自動検出
rosrun eys3d_camera perf_monitor --ns /G100P_1 --interval 0.5
```

各行はストリームごとに **SDK**（カメラから届いたフレーム）、**Pub**
（ドライバが送出したフレーム）、**Rx**（モニタが受信したフレーム）を
表示します。

---

## Frame ID

2 層の TF tree を `/tf_static` に latched で配信します:

```
<camera_name>_link                      ROS 基準軸(x 前、y 左、z 上)
├── <camera_name>_left_color_frame      y 軸 +baseline/2
│   └── <camera_name>_left_color_optical_frame     REP-103 光学座標
├── <camera_name>_right_color_frame     y 軸 -baseline/2
│   └── <camera_name>_right_color_optical_frame
├── <camera_name>_depth_frame           y 軸 +baseline/2
│   └── <camera_name>_depth_optical_frame
└── <camera_name>_points_frame          y 軸 +baseline/2
```

画像メッセージは `_optical_frame` の葉に、`PointCloud2` は既に基準軸である
`<camera_name>_points_frame` にスタンプされます。

---

## トラブルシューティング

| 症状 | 対処 |
|---|---|
| カメラのオープンが "device busy" で失敗する、またはオープンしても一度もフレームが来ない | 他のプロセスが `/dev/videoN` を掴んでいます。本ドライバの二つ目のインスタンスは多くの場合エラーを出さずにオープンし、そのまま飢餓状態になって `delivered no frame within 10 s of open` を毎秒記録します。`lsof /dev/video*` で掴んでいるものを特定し、古いドライバなら停止してください。 |
| オープン時に権限エラー | udev ルールをインストールする（[デバイス権限](#デバイス権限)を参照）か、`video` グループに参加する。 |
| RViz の Image パネルが空白 | Fixed Frame を `<camera_name>_link` に設定し、topic が現在の `camera_name` と一致しているか確認する。 |
| `right_color` topic がない | 1 エンドポイントに L\|R を載せるのは wide-color モードだけです。モードカタログの `split_lr` を確認してください。 |
| 深度がすべてゼロ | `depth_near_mm` / `depth_far_mm` の範囲外のピクセルはゼロになるため、作業距離を含む範囲か確認してください。近側が遠側以上の場合は設定自体が拒否され、ノードは `depth_near_mm must be < depth_far_mm` を記録してアイドルに入ります。topic は advertise されますが何も publish されません。 |
| セルフキャリブレーションの goal が rejected | モジュールに user calibration slot がないか、既にセッションが実行中です。理由はノードのログにあります。 |

---

## ライセンス

Apache-2.0 —— [LICENSE](../LICENSE) と [NOTICE.md](../NOTICE.md) を参照。
