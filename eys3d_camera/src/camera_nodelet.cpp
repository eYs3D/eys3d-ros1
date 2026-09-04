#include "eys3d_camera/camera_nodelet.hpp"

#include <csignal>
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <sensor_msgs/image_encodings.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>

namespace eys3d_camera {

namespace {

std::string join_frame(const std::string& prefix, const std::string& leaf) {
    if (prefix.empty()) return leaf;
    return prefix + "_" + leaf;
}

// Normalise the launch-supplied model token. Trims whitespace, accepts a
// small set of casual aliases (G100+, G100Plus), and otherwise returns
// the original string for the YAML loader and PID lookup to handle.
std::string normalize_model(std::string s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = s.find_last_not_of(" \t\r\n");
    s = s.substr(first, last - first + 1);
    if (s == "G100+" || s == "G100Plus" || s == "g100p" || s == "G100p") return "G100P";
    if (s == "G100+i" || s == "g100pi" || s == "G100pi")                return "G100Pi";
    if (s == "r77")                                                     return "R77";
    if (s == "g62")                                                     return "G62";
    return s;
}

}  // namespace

CameraNodelet::CameraNodelet() : device_(std::make_unique<EspdiDevice>()) {}

void CameraNodelet::onInit() {
    // Multi-threaded handles: the watchdog timer, the self-cal tick, the
    // service handlers and the action server all have to make progress
    // while the others are blocked in an SDK call.
    nh_  = getMTNodeHandle();
    pnh_ = getMTPrivateNodeHandle();

    read_params();

    tf_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();

    VideoMode vm;
    std::string err;
    if (!load_video_mode(vm, err)) {
        NODELET_ERROR("Video mode lookup failed: %s; node will idle", err.c_str());
        return;
    }
    NODELET_INFO("Selected mode %d: %s", vm.id, vm.name.c_str());

    // The video-mode catalogue declares whether the wire frame is wide
    // L|R. LR modes always split; non-LR modes never do.
    split_color_ = vm.color_split;

    // Topic names are unprefixed — the launch sets the ROS namespace to
    // camera_name, so the fully-qualified form is /<camera_name>/<leaf>.
    const std::string color_topic       = "left_color";
    const std::string color_right_topic = "right_color";
    const std::string depth_topic       = "depth";
    const std::string points_topic      = "depth/points";

    // Publishers are gated by the active video mode, camera_info included.
    // <stream>/image_raw beside <stream>/camera_info is the layout
    // image_transport::CameraSubscriber resolves, so image_proc and friends
    // pair the two without a remap.
    if (vm.has_color) {
        // espdi_device decodes both YUYV and MJPEG inputs to rgb8 inline, so
        // the publish path emits a uniform sensor_msgs/Image with
        // encoding=rgb8 regardless of the source wire format.
        pub_color_ = nh_.advertise<sensor_msgs::Image>(color_topic + "/image_raw", 5);
        pub_color_info_ =
            nh_.advertise<sensor_msgs::CameraInfo>(color_topic + "/camera_info", 5);
        if (split_color_) {
            pub_color_right_ =
                nh_.advertise<sensor_msgs::Image>(color_right_topic + "/image_raw", 5);
            pub_color_right_info_ =
                nh_.advertise<sensor_msgs::CameraInfo>(color_right_topic + "/camera_info", 5);
        }
    }
    if (vm.has_depth) {
        pub_depth_  = nh_.advertise<sensor_msgs::Image>(depth_topic + "/image_raw", 5);
        pub_depth_info_ =
            nh_.advertise<sensor_msgs::CameraInfo>(depth_topic + "/camera_info", 5);
        pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>(points_topic, 5);
    }

    // build_device_config() throws on an out-of-range depth-range or filter
    // value. Catch it and idle like any other config error rather than
    // letting the exception escape into the nodelet loader.
    DeviceConfig initial_cfg;
    try {
        initial_cfg = build_device_config(vm);
    } catch (const std::exception& e) {
        NODELET_ERROR("Invalid configuration: %s; node will idle", e.what());
        return;
    }
    {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        cached_cfg_ = initial_cfg;
    }

    // A failed open is not fatal: the node still builds its full surface and
    // enters the watchdog reconnect loop. Every device_ accessor below is
    // safe on a closed handle.
    const bool device_opened = device_->open(cached_cfg_);
    if (!device_opened) {
        NODELET_ERROR(
            "EspdiDevice::open() failed. Check that no other process holds the device "
            "(lsof /dev/video*), the model PID matches the connected camera, and the "
            "USB cable supports the required bandwidth. The watchdog will keep retrying.");
    } else {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        cached_calib_ = device_->calibration();
    }

    if (device_opened) publish_static_tf();

    // Five tasks, one DiagnosticStatus each per array; the diagnostic stack
    // aggregates max(level). diagnostics_rate_hz below 0.001 disables it.
    // The timer starts with the rest of the callback surface, below.
    double diag_hz = 1.0;
    pnh_.param("diagnostics_rate_hz", diag_hz, 1.0);
    const bool diagnostics_on = diag_hz >= 0.001;
    // ~diagnostic_period is the standard ROS 1 knob and wins when set;
    // diagnostics_rate_hz is this driver's own.
    double diag_period = diagnostics_on ? 1.0 / diag_hz : 1.0;
    if (pnh_.hasParam("diagnostic_period")) {
        double standard_period = diag_period;
        pnh_.getParam("diagnostic_period", standard_period);
        if (standard_period > 0.0) diag_period = standard_period;
    }
    if (diagnostics_on) {
        updater_ = std::make_unique<diagnostic_updater::Updater>(nh_, pnh_, getName());
        // Pin the device serial as hardware_id — immutable after open().
        const auto sn = device_->serial_number();
        updater_->setHardwareID(sn.empty() ? "eys3d_camera" : sn);
        updater_->add("device",     this, &CameraNodelet::diagnose_device);
        updater_->add("color",      this, &CameraNodelet::diagnose_color);
        updater_->add("depth",      this, &CameraNodelet::diagnose_depth);
        updater_->add("pointcloud", this, &CameraNodelet::diagnose_pc);
        updater_->add("thermal",    this, &CameraNodelet::diagnose_thermal);
        // Self-calibration is an on-demand action, fully described by the
        // selfcal/run feedback + result; it is not a continuous-stream health
        // subsystem, so it gets no /diagnostics task.
        prev_stats_ = device_->stats();
        prev_stats_wall_ = ros::Time::now();
    }

    // Callbacks are wired only for channels the active mode produces;
    // unused callbacks no-op when the matching publisher is inactive.
    ColorFrameCb color_cb = [this](FrameBuffer&& f) {
        if (split_color_) publish_split_color(std::move(f));
        else              on_color(std::move(f));
    };
    DepthFrameCb depth_cb = [this](FrameBuffer&& f) { on_depth(std::move(f)); };
    PointCloudCb pc_cb    = [this](std::vector<uint8_t>&& xyz_bytes, uint32_t n,
                                   uint32_t point_step, uint64_t ts) {
        on_point_cloud(std::move(xyz_bytes), n, point_step, ts);
    };

    // Install the subscriber gates BEFORE start() spawns the fetch threads so
    // the threads observe a fully-initialised gate on the very first frame;
    // setting them after start() is a data race on std::function.
    device_->set_pc_gate([this]() {
        return pub_points_ && pub_points_.getNumSubscribers() > 0;
    });
    device_->set_color_gate([this]() {
        const bool left_subs  = pub_color_  && pub_color_.getNumSubscribers() > 0;
        const bool right_subs = pub_color_right_ &&
                                pub_color_right_.getNumSubscribers() > 0;
        // Keep the color path running whenever colored_pointcloud is on and a
        // pointcloud subscriber exists, so the cloud paints from a current
        // color frame.
        bool pc_needs_color = false;
        {
            std::lock_guard<std::mutex> lk(cfg_mtx_);
            pc_needs_color = cached_cfg_.colored_pointcloud;
        }
        pc_needs_color = pc_needs_color && pub_points_ &&
                         pub_points_.getNumSubscribers() > 0;
        return left_subs || right_subs || pc_needs_color;
    });
    device_->set_depth_gate([this]() {
        return pub_depth_ && pub_depth_.getNumSubscribers() > 0;
    });

    // Set whether or not the open succeeded: they describe what the active
    // mode runs, and the watchdog needs them to watch a stream that only
    // arrives on a later reconnect.
    color_configured_ = vm.has_color;
    depth_configured_ = vm.has_depth;
    color_rectified_  = color_is_rectified(vm.depth_data_type);

    if (device_opened) {
        device_->start(ColorFrameCb(vm.has_color ? color_cb : ColorFrameCb{}),
                       DepthFrameCb(vm.has_depth ? depth_cb : DepthFrameCb{}),
                       PointCloudCb(vm.has_depth ? pc_cb    : PointCloudCb{}));
    } else {
        // The initial open failed: enter the reconnect loop rather than
        // stream. The watchdog polls kDisconnected and drives try_reconnect(),
        // which publishes the static TF and re-applies CT/PU once the device
        // appears.
        conn_state_.store(ConnState::kDisconnected);
    }

    configured_ = true;

    // ---- Callback surface, last ----------------------------------------
    //
    // A nodelet's callback queues are served by the manager's thread pool
    // while onInit() runs, so everything below goes live the moment it is
    // created; it is built only now, with the device open and configured_ set.
    //
    //   <cam>/pause    (SetBool)  gate publishing; USB keeps streaming
    //   <cam>/standby  (SetBool)  release the USB pipe, then reopen
    //   <cam>/hw_reset (Empty)    firmware USB self-reset, then reconnect
    //
    // Both stream controls act on the colour + depth pair together:
    // interleave modes need both halves of the V4L2 stream active at once.

    // Runtime image / filter controls. Seeds itself from the live CT/PU
    // state, so it must come up after the device is open.
    setup_reconfigure();

    srv_pause_    = nh_.advertiseService("pause",    &CameraNodelet::handle_pause,    this);
    srv_standby_  = nh_.advertiseService("standby",  &CameraNodelet::handle_standby,  this);
    srv_hw_reset_ = nh_.advertiseService("hw_reset", &CameraNodelet::handle_hw_reset, this);

    // Self-calibration, exposed only when selfcal_enable is set.
    //   <cam>/selfcal/run    (action)  — run one session to completion and resolve
    //                               it (revert / keep / commit)
    //   <cam>/selfcal/commit (Trigger) — persist a kept result to the user slot
    bool selfcal_enable = false;
    pnh_.param("selfcal_enable", selfcal_enable, false);
    if (selfcal_enable) {
        act_selfcal_run_ = std::make_unique<SelfCalServer>(
            nh_, "selfcal/run",
            boost::bind(&CameraNodelet::selfcal_handle_goal, this, _1),
            boost::bind(&CameraNodelet::selfcal_handle_cancel, this, _1),
            /*auto_start=*/false);
        act_selfcal_run_->start();
        srv_selfcal_commit_ =
            nh_.advertiseService("selfcal/commit", &CameraNodelet::handle_selfcal_commit, this);
    }

    if (diagnostics_on) {
        // ROS 1's Updater has no internal timer — drive it here. The timer
        // period is the authority (see diag_period above), so force_update()
        // rather than update(): the latter would re-gate on the Updater's own
        // next_time_ and drop a tick whenever the timer fires a hair early.
        diagnostics_timer_ = nh_.createTimer(
            ros::Duration(diag_period),
            [this](const ros::TimerEvent&) { updater_->force_update(); });
        NODELET_INFO("/diagnostics rate: %.2f Hz", 1.0 / diag_period);
    }

    // 1 Hz watchdog: detect USB disconnect and drive the reconnect loop.
    rebaseline_watchdog();
    watchdog_timer_ = nh_.createTimer(ros::Duration(1.0),
                                      &CameraNodelet::watchdog_tick, this);

    const char* color_topic_label =
        split_color_ ? "left_color/image_raw + right_color/image_raw"
                     : "left_color/image_raw";
    if (device_opened) {
        NODELET_INFO(
            "eys3d_camera '%s' running. Topics under '%s' (%s, depth/image_raw, depth/points).",
            camera_name_.c_str(), nh_.getNamespace().c_str(), color_topic_label);
    } else {
        NODELET_WARN(
            "eys3d_camera '%s' started without a device; waiting for the camera "
            "to appear under '%s'.",
            camera_name_.c_str(), nh_.getNamespace().c_str());
    }
}

CameraNodelet::~CameraNodelet() {
    // fprintf, not NODELET_INFO: the standalone node destroys the nodelet
    // after ros::shutdown(), where rosconsole drops every message. These two
    // lines bracket a teardown that can block, so they have to survive it.
    fprintf(stderr, "[%s] ~CameraNodelet()\n", getName().c_str());
    // Drop the timers and callback surfaces first so no callback thread can
    // enter pause() / standby() / hw_reset() / reconfigure against a device
    // that is mid-shutdown.
    watchdog_timer_.stop();
    diagnostics_timer_.stop();
    selfcal_run_timer_.stop();
    srv_pause_.shutdown();
    srv_standby_.shutdown();
    srv_hw_reset_.shutdown();
    srv_selfcal_commit_.shutdown();
    reconf_srv_.reset();

    // Abort an in-flight self-cal goal so the client's result callback fires
    // instead of hanging on a silently-destroyed goal.
    {
        std::lock_guard<std::mutex> lk(selfcal_goal_mtx_);
        if (selfcal_run_goal_) {
            SelfCalResult res;
            res.outcome = "FAILED";
            res.message = "node shutting down; self-calibration aborted";
            selfcal_run_goal_->setAborted(res, res.message);
            selfcal_run_goal_.reset();
        }
    }
    act_selfcal_run_.reset();
    updater_.reset();
    if (device_) device_->stop();
    fprintf(stderr, "[%s] ~CameraNodelet() done\n", getName().c_str());
}

void CameraNodelet::read_params() {
    // Identity + frames. Frame ids default to <camera_name>_<leaf>, ensuring
    // TF uniqueness in multi-camera setups; individual names can be
    // overridden to slot into an existing TF tree.
    pnh_.param<std::string>("camera_name", camera_name_, "eys3d_camera");
    pnh_.param<std::string>("dm_quality_cfg_dir", dm_quality_cfg_dir_, "");

    pnh_.param<std::string>("base_frame",        base_frame_,        "");
    pnh_.param<std::string>("left_color_frame",  left_color_frame_,  "");
    pnh_.param<std::string>("right_color_frame", right_color_frame_, "");
    pnh_.param<std::string>("depth_frame",       depth_frame_,       "");
    pnh_.param<std::string>("points_frame",      points_frame_,      "");
    if (base_frame_.empty())        base_frame_        = join_frame(camera_name_, "link");
    if (left_color_frame_.empty())  left_color_frame_  = join_frame(camera_name_, "left_color_frame");
    if (right_color_frame_.empty()) right_color_frame_ = join_frame(camera_name_, "right_color_frame");
    if (depth_frame_.empty())       depth_frame_       = join_frame(camera_name_, "depth_frame");
    if (points_frame_.empty())      points_frame_      = join_frame(camera_name_, "points_frame");
    left_color_optical_frame_  = join_frame(camera_name_, "left_color_optical_frame");
    right_color_optical_frame_ = join_frame(camera_name_, "right_color_optical_frame");
    depth_optical_frame_       = join_frame(camera_name_, "depth_optical_frame");
}

bool CameraNodelet::load_video_mode(VideoMode& out, std::string& err) {
    std::string dir;
    pnh_.param<std::string>("video_modes_dir", dir, "");
    if (dir.empty()) {
        const std::string share = ros::package::getPath("eys3d_camera");
        if (share.empty()) {
            err = "cannot resolve the eys3d_camera package path";
            return false;
        }
        dir = share + "/launch/video_modes";
    }
    std::string model_raw;
    pnh_.param<std::string>("model", model_raw, "G100P");
    const std::string model = normalize_model(model_raw);
    int mode_id = -1;
    pnh_.param("mode_id", mode_id, -1);

    const auto modes = load_video_modes(dir, model);
    if (modes.empty()) {
        err = "no modes loaded from " + dir + "/" + model + ".yaml "
              "(accepted models: G100P, G100Pi, R77, G62)";
        return false;
    }
    NODELET_INFO("%s", format_mode_table(model, modes).c_str());

    const auto info = load_model_info(dir, model);
    if (!info) {
        err = "model header missing or invalid in " + model + ".yaml";
        return false;
    }
    model_info_ = *info;

    // mode_id < 0 = auto: probe the negotiated USB link and pick the model's
    // signature default mode for it. An explicit mode_id is instead validated
    // against the link at open time.
    if (mode_id < 0) {
        if (model_info_.signature_mode.empty()) {
            err = "mode_id=auto: no signature_mode entry in " + model + ".yaml";
            return false;
        }
        DeviceConfig probe_cfg;
        pnh_.param<std::string>("dev_serial_number", probe_cfg.serial_number, "");
        pnh_.param<std::string>("usb_port",          probe_cfg.usb_port,      "");
        probe_cfg.expected_pid = model_info_.pid;
        const auto usb = device_->probe_usb_type(probe_cfg);

        // Fall back to the lowest-bandwidth signature (USB2 when declared):
        // it opens on either link, and open() validates the real one.
        auto it = model_info_.signature_mode.begin();
        if (usb) {
            const auto exact = model_info_.signature_mode.find(*usb);
            if (exact != model_info_.signature_mode.end()) {
                it = exact;
                NODELET_INFO(
                    "mode_id=auto -> %d (signature mode for the negotiated USB%d link)",
                    it->second, *usb);
            } else {
                NODELET_WARN(
                    "mode_id=auto: no signature_mode entry for a USB%d link in %s.yaml; "
                    "falling back to mode %d (USB%d signature)",
                    *usb, model.c_str(), it->second, it->first);
            }
        } else {
            NODELET_WARN(
                "mode_id=auto: could not probe the USB link type (camera not attached?); "
                "falling back to mode %d (USB%d signature)",
                it->second, it->first);
        }
        mode_id = it->second;
    }

    const auto found = find_mode(modes, mode_id);
    if (!found) {
        err = "mode_id=" + std::to_string(mode_id) + " not found in " + model + ".yaml";
        return false;
    }

    out = *found;
    return true;
}

DeviceConfig CameraNodelet::build_device_config(const VideoMode& vm) const {
    ros::NodeHandle pnh = pnh_;
    DeviceConfig cfg;
    cfg.color_width      = vm.color_width;
    cfg.color_height     = vm.color_height;
    cfg.color_format     = vm.color_format;
    cfg.depth_width      = vm.depth_width;
    cfg.depth_height     = vm.depth_height;
    cfg.depth_data_type  = vm.depth_data_type;
    cfg.zd_index         = vm.zd_index;
    cfg.framerate        = vm.framerate;
    cfg.interleave       = vm.interleave;
    cfg.mode_usb         = vm.usb;
    pnh.param("depth_near_mm", cfg.depth_near_mm, -1);
    pnh.param("depth_far_mm",  cfg.depth_far_mm,  -1);
    // Z14 depth is a 14-bit distance in mm: the far plane cannot exceed
    // 2^14 - 1, and near must sit in front of far. A launch value past that
    // is a config error, caught here before the device opens.
    if (cfg.depth_far_mm > 16383)
        throw std::invalid_argument("depth_far_mm exceeds the 16383 mm Z14 limit");
    if (cfg.depth_near_mm > 16383)
        throw std::invalid_argument("depth_near_mm exceeds the 16383 mm Z14 limit");
    // Order near against the far plane actually applied; depth_far_mm may be
    // -1, meaning the per-model default.
    const int effective_far =
        cfg.depth_far_mm > 0 ? cfg.depth_far_mm : model_info_.depth_far_mm;
    if (cfg.depth_near_mm > 0 && effective_far > 0 &&
        cfg.depth_near_mm >= effective_far)
        throw std::invalid_argument("depth_near_mm must be < depth_far_mm");

    pnh.param("ir_value", cfg.ir_value, -1);
    pnh.param("colored_pointcloud",        cfg.colored_pointcloud,        false);
    pnh.param("spatial_filter",            cfg.spatial_filter_enabled,    false);
    pnh.param("spatial_filter_alpha",      cfg.spatial_filter_alpha,      0.5);
    pnh.param("spatial_filter_delta",      cfg.spatial_filter_delta,      20);
    pnh.param("spatial_filter_magnitude",  cfg.spatial_filter_magnitude,  2);
    pnh.param("spatial_filter_holes_fill", cfg.spatial_filter_holes_fill, 0);
    pnh.param("temporal_filter",             cfg.temporal_filter_enabled,     false);
    pnh.param("temporal_filter_alpha",       cfg.temporal_filter_alpha,       0.4);
    pnh.param("temporal_filter_delta",       cfg.temporal_filter_delta,       20);
    pnh.param("temporal_filter_persistence", cfg.temporal_filter_persistence, 3);
    pnh.param("hole_filling",                cfg.hole_filling,                0);
    // Drives the wide-YUYV split decode in the device layer.
    cfg.split_color = split_color_;
    pnh.param<std::string>("dev_serial_number", cfg.serial_number, "");
    pnh.param<std::string>("usb_port",          cfg.usb_port,      "");
    std::string model_raw;
    pnh.param<std::string>("model", model_raw, "G100P");
    cfg.model = normalize_model(model_raw);
    // Per-model constants from the catalogue header.
    cfg.expected_pid    = model_info_.pid;
    cfg.mono            = model_info_.mono;
    cfg.ir_default      = model_info_.ir_default;
    cfg.default_near_mm = model_info_.depth_near_mm;
    cfg.default_far_mm  = model_info_.depth_far_mm;

    pnh.param("selfcal_enable", cfg.selfcal_enable, false);
    pnh.param<std::string>("selfcal_config_dir", cfg.selfcal_config_dir, "");
    // Empty selfcal_config_dir resolves to the in-package profiles, mirroring
    // how video_modes_dir falls back to the package share.
    if (cfg.selfcal_enable && cfg.selfcal_config_dir.empty()) {
        const std::string share = ros::package::getPath("eys3d_camera");
        // Leave empty on failure; start_selfcal() reports a missing profile.
        if (!share.empty()) cfg.selfcal_config_dir = share + "/config/selfcal";
    }

    // Range-check the filter tuning launch values: the device layer would
    // otherwise silently clamp them (a spatial_filter_magnitude of 99 becomes
    // 5 with no feedback). Throwing here is caught by onInit(), which logs and
    // idles rather than running a configuration the operator did not ask for.
    auto check_int = [](const char* name, long v, long lo, long hi) {
        if (v < lo || v > hi)
            throw std::invalid_argument(
                std::string(name) + " must be in [" + std::to_string(lo) + ", " +
                std::to_string(hi) + "]");
    };
    auto check_unit = [](const char* name, double v) {
        if (v < 0.0 || v > 1.0)
            throw std::invalid_argument(std::string(name) + " must be in [0.0, 1.0]");
    };
    check_int ("spatial_filter_delta",        cfg.spatial_filter_delta,        1, 4095);
    check_int ("spatial_filter_magnitude",    cfg.spatial_filter_magnitude,    1, 5);
    check_int ("spatial_filter_holes_fill",   cfg.spatial_filter_holes_fill,   0, 255);
    check_int ("temporal_filter_delta",       cfg.temporal_filter_delta,       1, 4095);
    check_int ("temporal_filter_persistence", cfg.temporal_filter_persistence, 0, 8);
    check_int ("hole_filling",                cfg.hole_filling,                0, 3);
    check_unit("spatial_filter_alpha",        cfg.spatial_filter_alpha);
    check_unit("temporal_filter_alpha",       cfg.temporal_filter_alpha);
    return cfg;
}

ros::Time CameraNodelet::stamp_from_hw_us(uint64_t hw_us) {
    if (hw_us == 0) return ros::Time::now();
    if (!time_anchor_set_.load(std::memory_order_acquire)) {
        const auto now_t = ros::Time::now();
        int64_t expected = 0;
        if (hw_anchor_us_.compare_exchange_strong(expected, static_cast<int64_t>(hw_us))) {
            ros_anchor_ns_.store(static_cast<int64_t>(now_t.toNSec()),
                                 std::memory_order_release);
            time_anchor_set_.store(true, std::memory_order_release);
            return now_t;
        }
        // CAS lost: the other fetch thread is mid-init. Spin until it has
        // published ros_anchor_ns_ — otherwise reading it below could return 0
        // and stamp this frame near the epoch, violating REP-117.
        while (!time_anchor_set_.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }
    const int64_t hw_anchor  = hw_anchor_us_.load(std::memory_order_acquire);
    const int64_t ros_anchor = ros_anchor_ns_.load(std::memory_order_acquire);
    const int64_t delta_ns = (static_cast<int64_t>(hw_us) - hw_anchor) * 1000;
    ros::Time t;
    t.fromNSec(static_cast<uint64_t>(ros_anchor + delta_ns));
    return t;
}

EspdiDevice::Calibration CameraNodelet::snapshot_calib() const {
    std::lock_guard<std::mutex> lk(calib_mtx_);
    return cached_calib_;
}

sensor_msgs::CameraInfo CameraNodelet::build_camera_info(
    const std::string& frame_id, const ros::Time& stamp,
    int width, int height, const EspdiDevice::Calibration& calib,
    InfoStream stream) const {
    sensor_msgs::CameraInfo ci;
    ci.header.stamp = stamp;
    ci.header.frame_id = frame_id;
    ci.width  = static_cast<uint32_t>(width);
    ci.height = static_cast<uint32_t>(height);
    // Everything stays zeroed when the rectify log did not load: the message
    // definition lets a client read K[0] == 0.0 as an uncalibrated camera, so
    // neither branch below may write an identity R or a zeroed D instead.
    if (calib.valid) {
        const bool left_eye  = stream != InfoStream::kRightColor;
        const bool rectified = stream == InfoStream::kDepth || color_rectified_;
        const auto& lens = left_eye ? calib.left : calib.right;
        // fx, fy, cx, cy are pixels of the raster the matrix is at; Tx and Ty
        // are those pixels times the log's baseline, which is millimetres.
        const double ps = EspdiDevice::raster_scale(calib.out_height, height);
        for (size_t i = 0; i < 12; ++i) ci.P[i] = lens.P[i];
        for (int i : {0, 2, 3, 5, 6, 7}) ci.P[i] *= ps;
        // Tx = -fx' * B and Ty = -fy' * B, in metres.
        ci.P[3] /= 1000.0;
        ci.P[7] /= 1000.0;

        if (rectified) {
            // The published image is the rectified one, so it is its own raw
            // image: K is P's left 3x3, R is the identity, and the five
            // coefficients are zero rather than absent -- image_geometry
            // classifies an empty D as UNKNOWN and throws on rectifyImage().
            const double k_from_p[9] = {ci.P[0], ci.P[1], ci.P[2],
                                        ci.P[4], ci.P[5], ci.P[6],
                                        ci.P[8], ci.P[9], ci.P[10]};
            for (size_t i = 0; i < 9; ++i) ci.K[i] = k_from_p[i];
            const double eye[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            for (size_t i = 0; i < 9; ++i) ci.R[i] = eye[i];
            ci.distortion_model = "plumb_bob";
            ci.D.assign(5, 0.0);
        } else {
            // Raw color: the factory lens model, scaled to the raster it is
            // published at. D acts on normalised coordinates and is invariant
            // under scale. The coefficient count follows the calibration --
            // eight are stored, and a five-coefficient lens leaves k4..k6 zero.
            const double ks = EspdiDevice::raster_scale(calib.in_height, height);
            for (size_t i = 0; i < 9; ++i) ci.K[i] = lens.K[i];
            for (int i : {0, 2, 4, 5}) ci.K[i] *= ks;
            for (size_t i = 0; i < 9; ++i) ci.R[i] = lens.R[i];
            const bool eight = std::any_of(lens.D.begin() + 5, lens.D.end(),
                                           [](double v) { return v != 0.0; });
            ci.distortion_model = eight ? "rational_polynomial" : "plumb_bob";
            ci.D.assign(lens.D.begin(), lens.D.begin() + (eight ? 8 : 5));
        }
        warn_if_off_raster(ci, stream);
    }
    return ci;
}

// A principal point far from the centre of the frame means the intrinsics
// belong to a raster other than the one being published. Both K and P are
// checked: the raw form scales them by independent factors. The bound admits a
// real principal point, which sits within a few percent of centre.
void CameraNodelet::warn_if_off_raster(const sensor_msgs::CameraInfo& ci,
                                       InfoStream stream) const {
    constexpr double kMaxOffCentre = 0.25;
    const double w = static_cast<double>(ci.width);
    const double h = static_cast<double>(ci.height);
    if (w <= 0.0 || h <= 0.0) return;
    const auto off = [](double c, double extent) {
        return std::abs(c / extent - 0.5) > kMaxOffCentre;
    };
    const bool bad_k = off(ci.K[2], w) || off(ci.K[5], h);
    const bool bad_p = off(ci.P[2], w) || off(ci.P[6], h);
    if (!bad_k && !bad_p) return;
    auto& latch = off_raster_warned_[static_cast<size_t>(stream)];
    if (latch.exchange(true, std::memory_order_relaxed)) return;
    NODELET_WARN("%s: principal point K (%.2f, %.2f) P (%.2f, %.2f) is not "
                 "centred in the %ux%u image; the intrinsics do not describe "
                 "this raster",
                 ci.header.frame_id.c_str(), ci.K[2], ci.K[5],
                 ci.P[2], ci.P[6], ci.width, ci.height);
}

void CameraNodelet::on_color(FrameBuffer&& f) {
    if (!pub_color_ || pub_color_.getNumSubscribers() == 0) return;
    NODELET_INFO_ONCE("on_color: first frame (frame=%d, %dx%d, %zu bytes, rgb8)",
                      f.frame_number, f.width, f.height, f.data.size());
    const auto stamp = stamp_from_hw_us(f.hw_timestamp_us);

    sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = left_color_optical_frame_;
    msg->height = static_cast<uint32_t>(f.height);
    msg->width  = static_cast<uint32_t>(f.width);
    msg->encoding = sensor_msgs::image_encodings::RGB8;
    msg->step = static_cast<uint32_t>(f.width) * 3;
    msg->is_bigendian = 0;
    msg->data = std::move(f.data);
    pub_color_.publish(msg);

    const auto calib = snapshot_calib();
    pub_color_info_.publish(build_camera_info(left_color_optical_frame_, stamp,
                                              f.width, f.height, calib,
                                              InfoStream::kLeftColor));
}

void CameraNodelet::publish_split_color(FrameBuffer&& f) {
    // Wide-color modes pack L|R into one wide raster. Two cases:
    //
    //   Pre-split (YUYV wide, default):  espdi_device decoded the wide YUYV
    //     directly into f.data (left) and f.data_right (right). Both buffers
    //     are moved straight into Image messages — no per-pixel copy here.
    //
    //   Wide intermediate (MJPEG wide):  espdi_device decoded the whole wide
    //     raster into f.data and left f.data_right empty. The half-width
    //     buffers are produced by row-by-row memcpy.
    if (!pub_color_) return;
    const bool publish_left  = pub_color_.getNumSubscribers() > 0;
    const bool publish_right = pub_color_right_ &&
                               pub_color_right_.getNumSubscribers() > 0;
    if (!publish_left && !publish_right) return;

    const auto stamp = stamp_from_hw_us(f.hw_timestamp_us);
    const bool pre_split = !f.data_right.empty();

    // In the pre-split path f.width is already the per-side width; in the
    // wide-intermediate path it is the full wide width and must be halved.
    const int half_w = pre_split ? f.width : (f.width / 2);
    if (!pre_split && (f.width % 2 != 0)) {
        NODELET_WARN_ONCE(
            "split_color requested but frame width %d is odd; falling back to L-only",
            f.width);
        on_color(std::move(f));
        return;
    }
    constexpr int bytes_per_pixel = 3;
    const size_t out_step = static_cast<size_t>(half_w) * bytes_per_pixel;
    const size_t half_bytes = out_step * static_cast<size_t>(f.height);

    auto fill_header = [&](sensor_msgs::Image& m, const std::string& frame) {
        m.header.stamp = stamp;
        m.header.frame_id = frame;
        m.height = static_cast<uint32_t>(f.height);
        m.width  = static_cast<uint32_t>(half_w);
        m.encoding = sensor_msgs::image_encodings::RGB8;
        m.is_bigendian = 0;
        m.step = static_cast<uint32_t>(out_step);
    };

    sensor_msgs::ImagePtr left;
    if (publish_left) {
        left = boost::make_shared<sensor_msgs::Image>();
        fill_header(*left, left_color_optical_frame_);
    }
    sensor_msgs::ImagePtr right;
    if (publish_right) {
        right = boost::make_shared<sensor_msgs::Image>();
        fill_header(*right, right_color_optical_frame_);
    }

    if (pre_split) {
        // Buffers are already half-width; transfer ownership.
        if (left)  left->data  = std::move(f.data);
        if (right) right->data = std::move(f.data_right);
    } else {
        // Wide intermediate: slice row-by-row into half-width buffers.
        const size_t in_step = static_cast<size_t>(f.width) * bytes_per_pixel;
        if (left)  left->data.resize(half_bytes);
        if (right) right->data.resize(half_bytes);
        const uint8_t* src = f.data.data();
        uint8_t* lp = left  ? left->data.data()  : nullptr;
        uint8_t* rp = right ? right->data.data() : nullptr;
        if (lp && rp) {
            for (int r = 0; r < f.height; ++r) {
                std::memcpy(lp + r * out_step, src + r * in_step,            out_step);
                std::memcpy(rp + r * out_step, src + r * in_step + out_step, out_step);
            }
        } else if (lp) {
            for (int r = 0; r < f.height; ++r) {
                std::memcpy(lp + r * out_step, src + r * in_step, out_step);
            }
        } else {
            for (int r = 0; r < f.height; ++r) {
                std::memcpy(rp + r * out_step, src + r * in_step + out_step, out_step);
            }
        }
    }

    if (left)  pub_color_.publish(left);
    if (right) pub_color_right_.publish(right);

    const auto calib = snapshot_calib();
    if (publish_left) {
        pub_color_info_.publish(build_camera_info(left_color_optical_frame_, stamp,
                                                  half_w, f.height, calib,
                                                  InfoStream::kLeftColor));
    }
    if (publish_right && pub_color_right_info_) {
        pub_color_right_info_.publish(build_camera_info(right_color_optical_frame_,
                                                        stamp, half_w, f.height,
                                                        calib,
                                                        InfoStream::kRightColor));
    }
}

void CameraNodelet::on_depth(FrameBuffer&& f) {
    if (!pub_depth_) return;
    NODELET_INFO_ONCE("on_depth: first frame (frame=%d, %dx%d, %zu bytes)",
                      f.frame_number, f.width, f.height, f.data.size());

    if (!dm_quality_applied_.exchange(true) && !dm_quality_cfg_dir_.empty()) {
        NODELET_INFO("First depth frame in; applying DM_Quality cfg from '%s'",
                     dm_quality_cfg_dir_.c_str());
        device_->apply_dm_quality_register_setting_async(dm_quality_cfg_dir_);
    }

    if (pub_depth_.getNumSubscribers() == 0) return;

    const auto stamp = stamp_from_hw_us(f.hw_timestamp_us);

    sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = depth_optical_frame_;
    msg->height = static_cast<uint32_t>(f.height);
    msg->width  = static_cast<uint32_t>(f.width);
    msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    msg->is_bigendian = 0;
    msg->step = static_cast<uint32_t>(f.width) * 2;
    msg->data = std::move(f.data);
    pub_depth_.publish(msg);

    const auto calib = snapshot_calib();
    pub_depth_info_.publish(build_camera_info(depth_optical_frame_, stamp,
                                              f.width, f.height, calib,
                                              InfoStream::kDepth));
}

void CameraNodelet::on_point_cloud(std::vector<uint8_t>&& xyz_bytes,
                                   uint32_t valid_points,
                                   uint32_t point_step,
                                   uint64_t hw_timestamp_us) {
    if (!pub_points_ || valid_points == 0 || xyz_bytes.empty()) return;
    const float* xyz = reinterpret_cast<const float*>(xyz_bytes.data());
    NODELET_INFO_ONCE(
        "on_point_cloud: first cloud (%u valid points, sample xyz=[%.3f, %.3f, %.3f] m, "
        "point_step=%u)",
        valid_points, xyz[0], xyz[1], xyz[2], point_step);
    NODELET_DEBUG_THROTTLE(10.0,
                           "pointcloud: %u valid points, sample xyz=[%.3f, %.3f, %.3f] m",
                           valid_points, xyz[0], xyz[1], xyz[2]);

    auto msg = boost::make_shared<sensor_msgs::PointCloud2>();
    msg->header.stamp = stamp_from_hw_us(hw_timestamp_us);
    msg->header.frame_id = points_frame_;
    msg->height = 1;
    msg->width  = valid_points;
    msg->is_dense     = false;
    msg->is_bigendian = false;

    auto set_field = [](sensor_msgs::PointField& fld, const char* name,
                        uint32_t offset, uint8_t datatype) {
        fld.name = name;
        fld.offset = offset;
        fld.datatype = datatype;
        fld.count = 1;
    };
    const bool colored = (point_step == 16);
    msg->fields.resize(colored ? 4 : 3);
    set_field(msg->fields[0], "x", 0, sensor_msgs::PointField::FLOAT32);
    set_field(msg->fields[1], "y", 4, sensor_msgs::PointField::FLOAT32);
    set_field(msg->fields[2], "z", 8, sensor_msgs::PointField::FLOAT32);
    if (colored) {
        // `rgb` as float32 with the bits 0x00RRGGBB is the PCL convention.
        set_field(msg->fields[3], "rgb", 12, sensor_msgs::PointField::FLOAT32);
    }

    msg->point_step = point_step;
    msg->row_step   = msg->point_step * msg->width;

    // Take ownership of the buffer (the PC thread sized it to
    // valid_points * point_step before invoking the callback).
    msg->data = std::move(xyz_bytes);

    pub_points_.publish(msg);
}

void CameraNodelet::publish_static_tf() {
    // Two-layer TF tree. Position links sit in ROS-base orientation so
    // robot-frame math does not need to unwind an optical rotation; optical
    // leaves carry the (-pi/2, 0, -pi/2) RPY that image_geometry,
    // depth_image_proc, and the RViz Image display expect.
    bool calib_valid = false;
    double baseline_mm = 0.0;
    {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        calib_valid = cached_calib_.valid;
        baseline_mm = cached_calib_.baseline_mm;
    }
    if (!calib_valid) {
        NODELET_WARN("publish_static_tf: calibration not valid; TF tree will use baseline=0");
    }
    const double half_baseline_m = calib_valid ? baseline_mm * 0.5e-3 : 0.0;

    tf2::Quaternion q_opt;
    q_opt.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);

    const auto stamp = ros::Time::now();
    auto make_pos_tf = [&](const std::string& parent, const std::string& child, double y_m) {
        geometry_msgs::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = 0.0;
        t.transform.translation.y = y_m;
        t.transform.translation.z = 0.0;
        t.transform.rotation.w = 1.0;
        return t;
    };
    auto make_opt_tf = [&](const std::string& parent, const std::string& child) {
        geometry_msgs::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.rotation.x = q_opt.x();
        t.transform.rotation.y = q_opt.y();
        t.transform.rotation.z = q_opt.z();
        t.transform.rotation.w = q_opt.w();
        return t;
    };

    std::vector<geometry_msgs::TransformStamped> tfs;
    tfs.reserve(7);
    // Layer 1 — base → position-only links (ROS-base orientation).
    tfs.push_back(make_pos_tf(base_frame_, left_color_frame_,  +half_baseline_m));
    tfs.push_back(make_pos_tf(base_frame_, right_color_frame_, -half_baseline_m));
    tfs.push_back(make_pos_tf(base_frame_, depth_frame_,       +half_baseline_m));
    tfs.push_back(make_pos_tf(base_frame_, points_frame_,      +half_baseline_m));
    // Layer 2 — position links → optical leaves (rotation only).
    tfs.push_back(make_opt_tf(left_color_frame_,  left_color_optical_frame_));
    tfs.push_back(make_opt_tf(right_color_frame_, right_color_optical_frame_));
    tfs.push_back(make_opt_tf(depth_frame_,       depth_optical_frame_));

    // /tf_static is latched, so any subscriber that joins later receives the
    // cached value immediately — no /tf re-stamping needed.
    tf_static_->sendTransform(tfs);
    NODELET_INFO("Published static TF (%zu links) under %s, baseline=%.2f mm",
                 tfs.size(), base_frame_.c_str(), baseline_mm);
}

}  // namespace eys3d_camera

PLUGINLIB_EXPORT_CLASS(eys3d_camera::CameraNodelet, nodelet::Nodelet)
