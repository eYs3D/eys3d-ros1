#ifndef EYS3D_CAMERA__CAMERA_NODELET_HPP_
#define EYS3D_CAMERA__CAMERA_NODELET_HPP_

// API stability note
// ------------------
// CameraNodelet is registered as the nodelet "eys3d_camera/CameraNodelet"
// and is also what the standalone eys3d_camera_node executable runs, so
// both entry points execute identical code. Stable across the 1.x series:
//
//   * published topics + types + frame IDs
//   * service names + types ("pause" and "standby" — both
//     std_srvs/SetBool; "hw_reset" — std_srvs/Empty; "selfcal/commit" —
//     std_srvs/Trigger)
//   * the selfcal/run action name and its SelfCal.action definition
//   * launch-time parameter names + types
//   * the dynamic_reconfigure parameter names (EYS3DCamera.cfg)
//   * the /diagnostics KeyValue keys listed in the README
//   * the nodelet plugin name above
//
// Everything else here — the private member layout, the types pulled in
// from espdi_device.hpp / video_modes.hpp, and the helpers — is
// implementation detail. Do NOT subclass, do NOT depend on the field
// layout, and do NOT take the address of any member.

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <actionlib/server/action_server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <eys3d_camera/EYS3DCameraConfig.h>
#include <eys3d_camera/SelfCalAction.h>

#include "eys3d_camera/espdi_device.hpp"
#include "eys3d_camera/video_modes.hpp"

namespace eys3d_camera {

class CameraNodelet : public nodelet::Nodelet {
public:
    CameraNodelet();
    ~CameraNodelet() override;

private:
    // nodelet entry point; performs the full device bring-up.
    void onInit() override;

    void read_params();
    // Loads the model's catalogue (mode entry + per-model header) from
    // launch/video_modes/<MODEL>.yaml; fills model_info_.
    bool load_video_mode(VideoMode& out, std::string& err_msg);
    DeviceConfig build_device_config(const VideoMode& vm) const;
    ModelInfo model_info_;

    void on_color(FrameBuffer&& f);
    void on_depth(FrameBuffer&& f);
    void on_point_cloud(std::vector<uint8_t>&& xyz_bytes,
                        uint32_t valid_points,
                        uint32_t point_step,
                        uint64_t hw_timestamp_us);

    ros::Time stamp_from_hw_us(uint64_t hw_us);
    // Which topic a CameraInfo belongs to. Selects the eye whose matrices are
    // read and, with color_rectified_, which of the two forms applies: depth is
    // rectified in every mode, color follows the mode's depth data type.
    enum class InfoStream { kLeftColor, kRightColor, kDepth, kCount };

    // One CameraInfo describing the image published on that topic and nothing
    // else. The rectified form carries K as P's left 3x3, D zero and R the
    // identity; the raw form carries the factory lens model.
    sensor_msgs::CameraInfo build_camera_info(
        const std::string& frame_id, const ros::Time& stamp,
        int width, int height, const EspdiDevice::Calibration& calib,
        InfoStream stream) const;
    void warn_if_off_raster(const sensor_msgs::CameraInfo& ci,
                            InfoStream stream) const;
    // One latch per stream: the three topics carry independent matrices at
    // independent rasters, so a shared latch would hide two of them. Cleared
    // on reconnect, which may land on a different calibration.
    mutable std::array<std::atomic<bool>,
                       static_cast<size_t>(InfoStream::kCount)> off_raster_warned_{};

    // Locked snapshot of cached_calib_ for per-frame CameraInfo publishes.
    EspdiDevice::Calibration snapshot_calib() const;

    // Wide-frame (L|R packed) slicer used when split_color is true. Yields
    // two per-half ROS messages from a single fetched FrameBuffer. The
    // sliced halves share a hw timestamp so consumers can synchronise them.
    void publish_split_color(FrameBuffer&& f);

    // Static TF: <cam>_link → {left/right/depth/points}_frame, with
    // left/depth/points placed at +baseline/2 on the Y (ROS-base left) axis
    // and right at -baseline/2. Sent once after open() reports a valid
    // calibration; latched by tf2 thereafter.
    void publish_static_tf();

    // ---- Runtime controls (dynamic_reconfigure) ----------------------
    //
    // The server's first callback carries the .cfg defaults; reconfigure_cb()
    // treats it as a seeding pass and reads the device instead of writing to
    // it, so the module's flash-persisted CT/PU state survives startup.
    using ReconfServer = dynamic_reconfigure::Server<EYS3DCameraConfig>;
    std::unique_ptr<ReconfServer> reconf_srv_;
    // dynamic_reconfigure::Server is not internally synchronised against
    // its own setCallback; it also owns the mutex the caller must hold to
    // call updateConfig() from inside the callback.
    boost::recursive_mutex reconf_mtx_;
    bool reconf_seeded_ = false;
    EYS3DCameraConfig reconf_cur_{};
    // Which CT/PU controls the launch set, sampled before the reconfigure
    // server exists: coming up, it writes its whole config to the parameter
    // server, after which hasParam() is true for every control.
    struct LaunchOverrides {
        bool auto_exposure        = false;
        bool auto_white_balance   = false;
        bool exposure_time_step   = false;
        bool power_line_frequency = false;
    } launch_set_;
    void setup_reconfigure();
    void reconfigure_cb(EYS3DCameraConfig& cfg, uint32_t level);
    // Reads the live CT/PU state out of the device and mirrors it into the
    // reconfigure server so rqt_reconfigure shows what the camera is
    // actually doing. Called at bring-up and after every reopen.
    void seed_reconfigure_from_device();
    // Re-sync from the camera's persisted CT/PU state after a reopen.
    // Those values live in the module's flash and survive a USB
    // re-enumerate, so the firmware — not the config — is the source of
    // truth. ir_value is re-applied separately by the callers because the
    // projector boots OFF on every open.
    void resync_ct_pu_from_device();

    // ---- Services ----------------------------------------------------
    ros::ServiceServer srv_pause_;
    ros::ServiceServer srv_standby_;
    ros::ServiceServer srv_hw_reset_;
    ros::ServiceServer srv_selfcal_commit_;
    bool handle_pause   (std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool handle_standby (std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool handle_hw_reset(std_srvs::Empty::Request& req,   std_srvs::Empty::Response& res);
    bool handle_selfcal_commit(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);

    // ---- Self-calibration action -------------------------------------
    //
    // The low-level ActionServer is used rather than SimpleActionServer
    // because a run that cannot start must be *rejected* (no Result at
    // all); SimpleActionServer can only accept and then abort, which
    // reaches the client as a very different thing.
    using SelfCalServer = actionlib::ActionServer<SelfCalAction>;
    using SelfCalGoalHandle = SelfCalServer::GoalHandle;
    std::unique_ptr<SelfCalServer> act_selfcal_run_;
    // Guards the goal handle against the action server's own callback
    // thread racing the tick timer that resolves it.
    std::mutex selfcal_goal_mtx_;
    std::unique_ptr<SelfCalGoalHandle> selfcal_run_goal_;
    ros::Timer selfcal_run_timer_;
    ros::Time selfcal_run_deadline_;
    float selfcal_run_auto_commit_shift_px_ = -1.0f;

    void selfcal_handle_goal(SelfCalGoalHandle gh);
    void selfcal_handle_cancel(SelfCalGoalHandle gh);
    void selfcal_run_tick(const ros::TimerEvent&);
    // Clears the active goal and stops the tick timer. Caller holds
    // selfcal_goal_mtx_.
    void selfcal_clear_goal_locked();

    std::atomic<bool> user_wants_standby_{false};

    // ---- Diagnostics -------------------------------------------------
    // ROS 1's Updater has no internal timer, so a wall timer drives it at
    // diagnostics_rate_hz.
    std::unique_ptr<diagnostic_updater::Updater> updater_;
    ros::Timer diagnostics_timer_;
    void diagnose_device  (diagnostic_updater::DiagnosticStatusWrapper& s);
    void diagnose_color   (diagnostic_updater::DiagnosticStatusWrapper& s);
    void diagnose_depth   (diagnostic_updater::DiagnosticStatusWrapper& s);
    void diagnose_pc      (diagnostic_updater::DiagnosticStatusWrapper& s);
    void diagnose_thermal (diagnostic_updater::DiagnosticStatusWrapper& s);
    // Per-tick derived rates. Refreshed by diagnose_device() (registered
    // first, so runs first) and reused by the other tasks.
    struct DiagSnapshot {
        double color_input_fps   = 0.0;
        double depth_input_fps   = 0.0;
        double color_publish_fps = 0.0;
        double depth_publish_fps = 0.0;
        double color_decode_avg_ms = 0.0;
        double pc_publish_fps    = 0.0;
        double pc_compute_avg_ms = 0.0;
        uint64_t color_decode_delta_count = 0;
        uint64_t pc_count_delta  = 0;
        EspdiDevice::Stats cur{};
    } diag_snap_;
    void refresh_diag_snapshot();
    // Previous totals + wall time, so per-second fps is delta / dt.
    EspdiDevice::Stats prev_stats_{};
    ros::Time prev_stats_wall_{};

    std::unique_ptr<EspdiDevice> device_;
    // cached_calib_ is read on the frame callbacks (fetch threads) and
    // written by open() / try_reconnect(), so the mutex guards against
    // torn reads of the float arrays inside Calibration.
    mutable std::mutex calib_mtx_;
    EspdiDevice::Calibration cached_calib_{};

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

    // Plain publishers, not image_transport: its publisher-side plugins load
    // into this process, where the SDK's exported libjpeg symbols interpose
    // on the system one and a compressed encode faults. camera_info rides its
    // own publisher on <stream>/camera_info, the sibling layout
    // image_transport::CameraSubscriber resolves.
    ros::Publisher pub_color_;
    ros::Publisher pub_color_right_;
    ros::Publisher pub_depth_;
    ros::Publisher pub_points_;
    ros::Publisher pub_color_info_;
    ros::Publisher pub_color_right_info_;
    ros::Publisher pub_depth_info_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // camera_name is the launch-supplied label (e.g. "G100P_1"), used as
    // the frame_id prefix for every frame this node publishes. The ROS
    // namespace comes from the launch file.
    std::string camera_name_;
    // Two-layer TF: <cam>_link (ROS base) → per-stream sensor frame →
    // _optical_frame leaf (REP-103). Image messages stamp the leaf;
    // PointCloud2 stamps <cam>_points_frame (already in base axes).
    std::string base_frame_;
    std::string left_color_frame_;
    std::string right_color_frame_;
    std::string depth_frame_;
    std::string points_frame_;
    std::string left_color_optical_frame_;
    std::string right_color_optical_frame_;
    std::string depth_optical_frame_;
    std::string dm_quality_cfg_dir_;
    bool split_color_ = false;

    std::atomic<bool> dm_quality_applied_{false};

    // Connection state machine driven by the 1 Hz watchdog timer.
    enum class ConnState { kStreaming, kDisconnected };
    std::atomic<ConnState> conn_state_{ConnState::kStreaming};
    ros::Timer watchdog_timer_;
    // Written by the watchdog timer and by the standby / hw_reset service
    // handlers, which the nodelet manager's thread pool can run at the same
    // time; rebaseline_watchdog() is the only way to set it.
    std::mutex watchdog_stats_mtx_;
    EspdiDevice::Stats watchdog_prev_stats_{};
    // Per-stream disconnect detection. Which streams the active mode runs is
    // fixed at construction; each is watched from the open, through the startup
    // grace until it delivers a frame and through its own silence counter after.
    bool color_configured_{false};
    bool depth_configured_{false};
    // Whether the active mode delivers color rectified. Fixed at construction
    // from the mode's depth data type; depth is rectified in every mode.
    bool color_rectified_{true};
    std::atomic<bool> color_armed_{false};
    std::atomic<bool> depth_armed_{false};
    std::atomic<int>  color_silent_seconds_{0};    // consecutive 1-s ticks with no colour frame
    std::atomic<int>  depth_silent_seconds_{0};    // consecutive 1-s ticks with no depth frame
    std::atomic<int>  startup_grace_seconds_{0};   // ticks elapsed before any frame seen
    // Reopens spent on a stream that has never delivered. Cleared once every
    // configured stream has armed, and not by the reopen itself -- that is what
    // bounds the loop.
    std::atomic<int>  cold_start_reopens_{0};
    static constexpr int kMaxColdStartReopens = 3;
    std::atomic<uint64_t> reconnect_attempts_{0};
    int reconnect_poll_counter_ = 0;   // throttles re-open attempts
    // Cached so the watchdog can re-issue open() + start() after a
    // disconnect. reconfigure_cb() mutates the temporal_filter_* fields
    // from the reconfigure thread; try_reconnect() reads the struct under
    // cfg_mtx_ so those writes do not tear the std::string members.
    mutable std::mutex cfg_mtx_;
    DeviceConfig cached_cfg_{};
    void watchdog_tick(const ros::TimerEvent&);
    void rebaseline_watchdog();
    // Stop and close the device, force Disconnected, and reset the watchdog
    // arming / grace state so the reconnect loop starts clean. The poll
    // cadence is left alone — see the note at the reset itself.
    void declare_disconnected();
    bool try_reconnect();

    std::atomic<bool> time_anchor_set_{false};
    std::atomic<int64_t> hw_anchor_us_{0};
    std::atomic<int64_t> ros_anchor_ns_{0};

    // Set once onInit() has brought the device up. The callback surface is
    // created after this point, so a handler can assume it is true; the
    // watchdog still checks it because its timer and the flag are set in the
    // same function. A configuration failure returns from onInit() before
    // any of that exists, leaving a node that advertises only its topics
    // and idles.
    bool configured_ = false;
};

}  // namespace eys3d_camera

#endif  // EYS3D_CAMERA__CAMERA_NODELET_HPP_
