// Control plane for the eYs3D camera nodelet: runtime services, the
// disconnect watchdog and reconnect loop, dynamic_reconfigure, the
// /diagnostics tasks, and the self-calibration action server. The frame
// data path lives in camera_nodelet.cpp.

#include "eys3d_camera/camera_nodelet.hpp"

#include <algorithm>
#include <cmath>

#include <diagnostic_msgs/DiagnosticStatus.h>

namespace eys3d_camera {

// ===================================================================
// Runtime stream-control services
// ===================================================================

bool CameraNodelet::handle_pause(std_srvs::SetBool::Request& req,
                                 std_srvs::SetBool::Response& res) {
    if (device_->selfcal_active()) {
        res.success = false;
        res.message = "refused: a self-calibration session is in progress";
        return true;
    }
    // pause() fails only when a session started between the guard above and
    // this call, so the refusal reads the same either way.
    const bool ok = device_->pause(req.data);
    res.success = ok;
    res.message = ok ? std::string("stream ") + (req.data ? "paused" : "resumed")
                     : std::string("refused: a self-calibration session is in progress");
    return true;
}

bool CameraNodelet::handle_standby(std_srvs::SetBool::Request& req,
                                   std_srvs::SetBool::Response& res) {
    if (device_->selfcal_active()) {
        res.success = false;
        res.message = "refused: a self-calibration session is in progress";
        return true;
    }
    // Gated on connected: close() leaves stream_state stale.
    const bool in_standby =
        device_->stream_state() == EspdiDevice::StreamState::Standby;
    if (conn_state_.load() == ConnState::kStreaming && req.data == in_standby) {
        res.success = true;
        res.message = std::string("standby already ") +
                      (req.data ? "entered" : "exited");
        return true;
    }
    // Hold the operator-suspended flag across the whole transition: the
    // reopen in standby(false) takes seconds, during which the stats read
    // exactly like a stalled stream.
    if (req.data) {
        user_wants_standby_.store(true, std::memory_order_release);
    } else {
        // Re-anchor and re-apply DM_Quality before the resume restarts the
        // fetch threads, so a firmware that resets either on STREAMON cannot
        // stamp the first post-resume frame in the past.
        for (auto& latch : off_raster_warned_) latch.store(false, std::memory_order_relaxed);
        dm_quality_applied_.store(false);
        hw_anchor_us_.store(0, std::memory_order_relaxed);
        ros_anchor_ns_.store(0, std::memory_order_relaxed);
        time_anchor_set_.store(false, std::memory_order_release);
    }
    const bool ok = device_->standby(req.data);
    res.success = ok;
    if (ok) {
        if (!req.data) {
            // Cleared on the way out, not on the way in: callbacks run on a
            // multi-threaded nodelet queue, so a tick can interleave with the
            // reopen below and read the pre-standby armed state and the
            // silence counters this block has not zeroed yet. The guard still
            // clears it if one of those calls throws.
            struct ClearOnExit {
                std::atomic<bool>& flag;
                ~ClearOnExit() { flag.store(false, std::memory_order_release); }
            } clear_on_exit{user_wants_standby_};
            // Re-apply the operator's current ir_value: the SDK reopen
            // restores cfg.ir_value (the launch-time value), while a runtime
            // override lives only in the FW register APC_CloseDevice cleared.
            int ir = -1;
            {
                boost::recursive_mutex::scoped_lock lk(reconf_mtx_);
                ir = reconf_cur_.ir_value;
            }
            device_->set_ir_value(ir);
            // Re-sync the flash-persisted CT/PU settings from the firmware.
            resync_ct_pu_from_device();
            // Refresh the watchdog baseline AND re-arm the startup grace so
            // the first tick after standby(false) does not fall through to
            // the zero-frame check before the SDK delivers the first
            // post-resume frame.
            rebaseline_watchdog();
            color_silent_seconds_.store(0, std::memory_order_relaxed);
            depth_silent_seconds_.store(0, std::memory_order_relaxed);
            startup_grace_seconds_.store(0, std::memory_order_relaxed);
            cold_start_reopens_.store(0, std::memory_order_relaxed);
            color_armed_.store(false, std::memory_order_relaxed);
            depth_armed_.store(false, std::memory_order_relaxed);
        }
        res.message = std::string("standby ") + (req.data ? "entered" : "exited");
    } else {
        // The handle is known closed, so go straight to the reconnect
        // loop instead of waiting out the watchdog.
        user_wants_standby_.store(false, std::memory_order_release);
        if (!req.data) declare_disconnected();
        res.message = req.data
            ? "standby(true) failed ; device is not open"
            : "standby(false) reopen failed ; device left closed";
    }
    return true;
}

bool CameraNodelet::handle_hw_reset(std_srvs::Empty::Request&,
                                    std_srvs::Empty::Response&) {
    // Empty service has no response field, so a refusal can only be logged;
    // re-enumerating the device would kill a running session.
    if (device_->selfcal_active()) {
        NODELET_WARN("hw_reset refused: a self-calibration session is in "
                     "progress (stop it first)");
        return true;
    }
    NODELET_WARN("hw_reset: resetting the camera over USB");
    // stop() (join the fetch threads while the link is still up),
    // reset_usb() (write the detach sequence; the host re-enumerates), then
    // close() (release the now-stale handle).
    device_->stop();
    device_->reset_usb();
    device_->close();
    // Drive the node to Disconnected and let the watchdog reopen by the
    // pinned serial / port. Clearing user_wants_standby_ lets the watchdog
    // resume if the reset was issued from Standby.
    user_wants_standby_.store(false, std::memory_order_release);
    conn_state_.store(ConnState::kDisconnected);
    rebaseline_watchdog();
    color_armed_.store(false, std::memory_order_relaxed);
    depth_armed_.store(false, std::memory_order_relaxed);
    startup_grace_seconds_.store(0, std::memory_order_relaxed);
    color_silent_seconds_.store(0, std::memory_order_relaxed);
    depth_silent_seconds_.store(0, std::memory_order_relaxed);
    reconnect_poll_counter_ = 0;
    return true;
}

bool CameraNodelet::handle_selfcal_commit(std_srvs::Trigger::Request&,
                                          std_srvs::Trigger::Response& res) {
    // Refuse while a run is in flight (would race the worker on the handle);
    // the manual commit persists a kept result afterwards.
    {
        std::lock_guard<std::mutex> lk(selfcal_goal_mtx_);
        if (selfcal_run_goal_) {
            res.success = false;
            res.message = "commit refused: a self-calibration run is in "
                          "progress; wait for it to finish";
            return true;
        }
    }
    res.success = device_->commit_selfcal();
    res.message = res.success
        ? "self-calibration committed to flash (G2 user slot)"
        : "commit refused: no committable result (see node log)";
    return true;
}

// ===================================================================
// Disconnect watchdog + reconnect loop
// ===================================================================

void CameraNodelet::declare_disconnected() {
    device_->stop();
    device_->close();
    conn_state_.store(ConnState::kDisconnected);
    color_armed_.store(false, std::memory_order_relaxed);
    depth_armed_.store(false, std::memory_order_relaxed);
    color_silent_seconds_.store(0, std::memory_order_relaxed);
    depth_silent_seconds_.store(0, std::memory_order_relaxed);
    startup_grace_seconds_.store(0, std::memory_order_relaxed);
    // Cadence not reset here: the standby handler may retry faster than the poll.
}

void CameraNodelet::rebaseline_watchdog() {
    const auto s = device_->stats();
    std::lock_guard<std::mutex> lk(watchdog_stats_mtx_);
    watchdog_prev_stats_ = s;
}

void CameraNodelet::watchdog_tick(const ros::TimerEvent&) {
    if (!device_ || !configured_) return;

    // Operator-requested Standby suppresses the watchdog: with no fetch
    // threads the stats never advance, so the zero-frame check would misread
    // the pause as a disconnect. Skip the tick, refresh the baseline, bail.
    if (user_wants_standby_.load(std::memory_order_acquire)) {
        rebaseline_watchdog();
        color_silent_seconds_.store(0, std::memory_order_relaxed);
        depth_silent_seconds_.store(0, std::memory_order_relaxed);
        return;
    }

    if (conn_state_.load() == ConnState::kStreaming) {
        const auto cur = device_->stats();
        uint64_t color_frames, depth_frames;
        {
            std::lock_guard<std::mutex> lk(watchdog_stats_mtx_);
            color_frames =
                cur.color_input_total - watchdog_prev_stats_.color_input_total;
            depth_frames =
                cur.depth_input_total - watchdog_prev_stats_.depth_input_total;
            watchdog_prev_stats_ = cur;
        }

        // Arm each stream the moment it delivers its first frame. A stream
        // the active mode never runs stays unarmed, so its permanent zero
        // count is never read as a stall. Frames since the baseline; the
        // lifetime total is never reset.
        if (color_frames > 0) color_armed_.store(true, std::memory_order_relaxed);
        if (depth_frames > 0) depth_armed_.store(true, std::memory_order_relaxed);
        const bool color_armed = color_armed_.load(std::memory_order_relaxed);
        const bool depth_armed = depth_armed_.load(std::memory_order_relaxed);

        // The SDK starts capture on the first fetch and leaves the stream
        // unstarted when that fails, retrying forever; only a reopen clears it.
        // 10 s, not the steady-state 3 s: R77 mode 1 at 7 fps over USB 2.0 takes
        // several seconds to deliver its first frame.
        const bool color_pending = color_configured_ && !color_armed;
        const bool depth_pending = depth_configured_ && !depth_armed;
        if (color_pending || depth_pending) {
            const int grace_seconds =
                startup_grace_seconds_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (grace_seconds >= 10) {
                // A standby() that landed between the top-of-tick guard and
                // here would also produce zero frames. Bail rather than tear
                // the SDK down.
                if (user_wants_standby_.load(std::memory_order_acquire)) return;
                const char* which = (color_pending && depth_pending) ? "colour+depth"
                                    : color_pending ? "colour" : "depth";
                // A reopen clears a stream the SDK failed to start, not a mode
                // the link cannot carry; past the cap the device stays open so
                // a stream that does work keeps running.
                if (cold_start_reopens_.load(std::memory_order_relaxed)
                        < kMaxColdStartReopens) {
                    cold_start_reopens_.fetch_add(1, std::memory_order_relaxed);
                    NODELET_ERROR("watchdog: %s delivered no frame within %d s of "
                                  "open; declaring camera disconnected",
                                  which, grace_seconds);
                    declare_disconnected();
                    return;
                }
                NODELET_ERROR_THROTTLE(
                    60.0,
                    "watchdog: %s has delivered no frame after %d reopens; "
                    "leaving the device open. Check that the active mode fits "
                    "the negotiated USB link",
                    which, kMaxColdStartReopens);
            }
            // Fall through: a stream that did arm is still watched below.
        } else {
            cold_start_reopens_.store(0, std::memory_order_relaxed);
        }

        // At least one stream is live. An armed stream that then goes silent
        // is a per-stream stall — e.g. depth wedges in firmware while colour
        // keeps flowing — which reconnect recovers even though the other
        // stream is still delivering frames.
        const bool color_silent = color_armed && color_frames == 0;
        const bool depth_silent = depth_armed && depth_frames == 0;
        // A stream that delivers resets only its own counter.
        const int color_silent_seconds =
            color_silent ? color_silent_seconds_.fetch_add(1, std::memory_order_relaxed) + 1
                         : (color_silent_seconds_.store(0, std::memory_order_relaxed), 0);
        const int depth_silent_seconds =
            depth_silent ? depth_silent_seconds_.fetch_add(1, std::memory_order_relaxed) + 1
                         : (depth_silent_seconds_.store(0, std::memory_order_relaxed), 0);

        const int silent_seconds = std::max(color_silent_seconds, depth_silent_seconds);
        if (silent_seconds >= 3) {
            if (user_wants_standby_.load(std::memory_order_acquire)) return;
            const bool color_stalled = color_silent_seconds >= 3;
            const bool depth_stalled = depth_silent_seconds >= 3;
            const char* which = (color_stalled && depth_stalled) ? "colour+depth"
                                : color_stalled ? "colour" : "depth";
            NODELET_ERROR("watchdog: %s stream silent for %d s; "
                          "declaring camera disconnected", which, silent_seconds);
            declare_disconnected();
        }
        return;
    }

    // kDisconnected: poll every 2 s for the device to come back.
    if (++reconnect_poll_counter_ < 2) return;
    reconnect_poll_counter_ = 0;
    const uint64_t attempts_now =
        reconnect_attempts_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (try_reconnect()) {
        NODELET_INFO("watchdog: reconnect succeeded after %lu attempt(s)",
                     static_cast<unsigned long>(attempts_now));
        rebaseline_watchdog();
        conn_state_.store(ConnState::kStreaming);
        color_armed_.store(false, std::memory_order_relaxed);
        depth_armed_.store(false, std::memory_order_relaxed);
        startup_grace_seconds_.store(0, std::memory_order_relaxed);
        color_silent_seconds_.store(0, std::memory_order_relaxed);
        depth_silent_seconds_.store(0, std::memory_order_relaxed);
        reconnect_attempts_.store(0, std::memory_order_relaxed);
    }
}

bool CameraNodelet::try_reconnect() {
    DeviceConfig cfg_snapshot;
    {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        cfg_snapshot = cached_cfg_;
    }
    if (!device_->open(cfg_snapshot)) return false;
    {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        cached_calib_ = device_->calibration();
    }
    // Re-publish the static TF from the (re-read) calibration. Latched, so
    // this is harmless on an ordinary reconnect and is the only place the
    // tree is published when the initial open failed and the device appeared
    // later.
    publish_static_tf();

    // Reinstall gates before start() — see onInit() for the rationale.
    device_->set_pc_gate([this]() {
        return pub_points_ && pub_points_.getNumSubscribers() > 0;
    });
    device_->set_color_gate([this]() {
        const bool left_subs  = pub_color_ && pub_color_.getNumSubscribers() > 0;
        const bool right_subs = pub_color_right_ &&
                                pub_color_right_.getNumSubscribers() > 0;
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

    // USB re-enumeration restarts the hw timestamp counter and power-cycles
    // the registers, so the anchor and the DM_Quality flag are reset before
    // start() respawns the fetch threads. Both anchor halves must go to zero
    // -- stamp_from_hw_us elects its initialiser by CAS-from-zero.
    for (auto& latch : off_raster_warned_) latch.store(false, std::memory_order_relaxed);
    dm_quality_applied_.store(false);
    hw_anchor_us_.store(0, std::memory_order_relaxed);
    ros_anchor_ns_.store(0, std::memory_order_relaxed);
    time_anchor_set_.store(false, std::memory_order_release);

    ColorFrameCb color_cb = [this](FrameBuffer&& f) {
        if (split_color_) publish_split_color(std::move(f));
        else              on_color(std::move(f));
    };
    DepthFrameCb depth_cb = [this](FrameBuffer&& f) { on_depth(std::move(f)); };
    PointCloudCb pc_cb    = [this](std::vector<uint8_t>&& xyz, uint32_t n,
                                   uint32_t step, uint64_t ts) {
        on_point_cloud(std::move(xyz), n, step, ts);
    };
    const bool has_color = static_cast<bool>(pub_color_);
    const bool has_depth = static_cast<bool>(pub_depth_);
    device_->start(has_color ? color_cb : ColorFrameCb{},
                   has_depth ? depth_cb : DepthFrameCb{},
                   has_depth ? pc_cb    : PointCloudCb{});

    // Re-apply IR — the projector boots OFF after every open().
    int ir = -1;
    {
        boost::recursive_mutex::scoped_lock lk(reconf_mtx_);
        ir = reconf_cur_.ir_value;
    }
    device_->set_ir_value(ir);
    // Re-sync the flash-persisted CT/PU settings from the firmware.
    resync_ct_pu_from_device();
    return true;
}

// ===================================================================
// Runtime controls (dynamic_reconfigure)
// ===================================================================

void CameraNodelet::setup_reconfigure() {
    // Sample before the server is constructed: it writes its whole config to
    // the parameter server, after which hasParam() is true for everything.
    launch_set_.auto_exposure        = pnh_.hasParam("auto_exposure");
    launch_set_.auto_white_balance   = pnh_.hasParam("auto_white_balance");
    launch_set_.exposure_time_step   = pnh_.hasParam("exposure_time_step");
    launch_set_.power_line_frequency = pnh_.hasParam("power_line_frequency");

    // Seed the in-memory mirror from the launch parameters so the first
    // callback compares against what open() actually applied rather than
    // against zeroes.
    pnh_.param("ir_value", reconf_cur_.ir_value, -1);
    pnh_.param("temporal_filter",             reconf_cur_.temporal_filter,             false);
    pnh_.param("temporal_filter_alpha",       reconf_cur_.temporal_filter_alpha,       0.4);
    pnh_.param("temporal_filter_delta",       reconf_cur_.temporal_filter_delta,       20);
    pnh_.param("temporal_filter_persistence", reconf_cur_.temporal_filter_persistence, 3);
    pnh_.param<std::string>("selfcal_profile", reconf_cur_.selfcal_profile,
                            "maintenance_dock");

    reconf_srv_ = std::make_unique<ReconfServer>(reconf_mtx_, pnh_);
    // setCallback fires the callback synchronously; reconfigure_cb() treats
    // that first invocation as the seeding pass.
    reconf_srv_->setCallback(
        boost::bind(&CameraNodelet::reconfigure_cb, this, _1, _2));
}

void CameraNodelet::reconfigure_cb(EYS3DCameraConfig& cfg, uint32_t /*level*/) {
    if (!device_) return;

    if (!reconf_seeded_) {
        // Seeding pass: write back only what the launch asked for, and read
        // the rest off the device. Values assigned to cfg here become the
        // server's published state.
        const auto state = device_->read_runtime_state();
        NODELET_INFO(
            "Camera CT/PU state on open: IR=%d (read_ok=%d), AE=%s (ok=%d), "
            "exposure_step=%d (ok=%d), AWB=%s (ok=%d), power_line=%d (ok=%d)",
            state.ir_value, state.ir_read_ok,
            state.auto_exposure ? "auto" : "manual", state.auto_exposure_read_ok,
            state.exposure_time_step, state.exposure_read_ok,
            state.auto_white_balance ? "auto" : "manual", state.awb_read_ok,
            state.power_line_frequency, state.plf_read_ok);

        // ir_value is always applied: the projector boots OFF on every open,
        // so there is no firmware state worth preserving. -1 = mode-resolved
        // default, 0 = off, positive = raw level.
        cfg.ir_value = reconf_cur_.ir_value;
        device_->set_ir_value(cfg.ir_value);

        if (launch_set_.auto_exposure) {
            device_->set_auto_exposure(cfg.auto_exposure);
        } else if (state.auto_exposure_read_ok) {
            cfg.auto_exposure = state.auto_exposure;
        }
        if (!model_info_.mono && launch_set_.auto_white_balance) {
            device_->set_auto_white_balance(cfg.auto_white_balance);
        } else if (state.awb_read_ok) {
            cfg.auto_white_balance = state.auto_white_balance;
        }
        if (launch_set_.exposure_time_step && !cfg.auto_exposure) {
            (void)device_->set_exposure_time_step(cfg.exposure_time_step);
        } else if (state.exposure_read_ok) {
            cfg.exposure_time_step = state.exposure_time_step;
        }
        if (launch_set_.power_line_frequency) {
            device_->set_power_line_frequency(cfg.power_line_frequency);
        } else if (state.plf_read_ok &&
                   (state.power_line_frequency == 1 || state.power_line_frequency == 2)) {
            cfg.power_line_frequency = state.power_line_frequency;
        }

        // The temporal filter was already configured through DeviceConfig at
        // open(); mirror the launch values rather than re-pushing them.
        cfg.temporal_filter             = reconf_cur_.temporal_filter;
        cfg.temporal_filter_alpha       = reconf_cur_.temporal_filter_alpha;
        cfg.temporal_filter_delta       = reconf_cur_.temporal_filter_delta;
        cfg.temporal_filter_persistence = reconf_cur_.temporal_filter_persistence;
        cfg.selfcal_profile             = reconf_cur_.selfcal_profile;

        reconf_cur_ = cfg;
        reconf_seeded_ = true;
        return;
    }

    // Steady state: push only what actually changed, so an unrelated tweak
    // does not replay every control as a USB write.
    if (cfg.ir_value != reconf_cur_.ir_value) {
        device_->set_ir_value(cfg.ir_value);
    }

    const bool ae_changed = cfg.auto_exposure != reconf_cur_.auto_exposure;
    if (ae_changed) device_->set_auto_exposure(cfg.auto_exposure);
    // exposure_time_step applies in manual mode only. Re-push it whenever
    // auto-exposure has just been turned off, so the operator's step lands
    // even if its own value did not change in this batch.
    if (!cfg.auto_exposure &&
        (ae_changed || cfg.exposure_time_step != reconf_cur_.exposure_time_step)) {
        (void)device_->set_exposure_time_step(cfg.exposure_time_step);
    }

    if (cfg.auto_white_balance != reconf_cur_.auto_white_balance) {
        if (model_info_.mono) {
            NODELET_WARN("auto_white_balance ignored: %s is a monochrome module",
                         cached_cfg_.model.c_str());
            cfg.auto_white_balance = reconf_cur_.auto_white_balance;
        } else {
            device_->set_auto_white_balance(cfg.auto_white_balance);
        }
    }

    if (cfg.power_line_frequency != reconf_cur_.power_line_frequency) {
        device_->set_power_line_frequency(cfg.power_line_frequency);
    }

    const bool tf_changed =
        cfg.temporal_filter             != reconf_cur_.temporal_filter ||
        cfg.temporal_filter_alpha       != reconf_cur_.temporal_filter_alpha ||
        cfg.temporal_filter_delta       != reconf_cur_.temporal_filter_delta ||
        cfg.temporal_filter_persistence != reconf_cur_.temporal_filter_persistence;
    if (tf_changed) {
        device_->set_temporal_filter(cfg.temporal_filter,
                                     cfg.temporal_filter_alpha,
                                     cfg.temporal_filter_delta,
                                     cfg.temporal_filter_persistence);
        // Keep the reconnect snapshot in step, so a reopen restores what the
        // operator last asked for rather than the launch-time values.
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        cached_cfg_.temporal_filter_enabled     = cfg.temporal_filter;
        cached_cfg_.temporal_filter_alpha       = cfg.temporal_filter_alpha;
        cached_cfg_.temporal_filter_delta       = cfg.temporal_filter_delta;
        cached_cfg_.temporal_filter_persistence = cfg.temporal_filter_persistence;
    }

    reconf_cur_ = cfg;
}

void CameraNodelet::seed_reconfigure_from_device() {
    if (!device_ || !reconf_srv_) return;
    // The firmware is the source of truth after a reopen: read its state and
    // mirror it into the server; skip fields the firmware did not report.
    const auto s = device_->read_runtime_state();
    EYS3DCameraConfig cfg;
    {
        boost::recursive_mutex::scoped_lock lk(reconf_mtx_);
        cfg = reconf_cur_;
    }
    // auto_exposure=false makes the steady-state path re-apply
    // exposure_time_step, so hold it back until the exposure value reads
    // back too.
    if (s.auto_exposure_read_ok && (s.auto_exposure || s.exposure_read_ok))
        cfg.auto_exposure = s.auto_exposure;
    if (s.awb_read_ok && !model_info_.mono)
        cfg.auto_white_balance = s.auto_white_balance;
    if (s.exposure_read_ok && !s.auto_exposure)
        cfg.exposure_time_step = s.exposure_time_step;
    if (s.plf_read_ok && (s.power_line_frequency == 1 || s.power_line_frequency == 2))
        cfg.power_line_frequency = s.power_line_frequency;

    boost::recursive_mutex::scoped_lock lk(reconf_mtx_);
    // Update the mirror first: updateConfig() publishes the new state
    // without invoking the callback, so the mirror must already agree or the
    // next real callback would replay these as fresh writes.
    reconf_cur_ = cfg;
    reconf_srv_->updateConfig(cfg);
}

void CameraNodelet::resync_ct_pu_from_device() {
    seed_reconfigure_from_device();
}

// ===================================================================
// /diagnostics
// ===================================================================

// Refresh per-tick derived rates from the device stats counter. Called once
// per Updater round by diagnose_device() (always first in registration
// order) so the four remaining tasks reuse the same numbers and the per-tick
// math runs only once.
void CameraNodelet::refresh_diag_snapshot() {
    const auto stamp = ros::Time::now();
    const auto cur = device_->stats();
    const double dt = std::max(1e-3, (stamp - prev_stats_wall_).toSec());

    diag_snap_.cur = cur;
    diag_snap_.color_input_fps =
        static_cast<double>(cur.color_input_total   - prev_stats_.color_input_total)   / dt;
    diag_snap_.depth_input_fps =
        static_cast<double>(cur.depth_input_total   - prev_stats_.depth_input_total)   / dt;
    diag_snap_.color_publish_fps =
        static_cast<double>(cur.color_publish_total - prev_stats_.color_publish_total) / dt;
    diag_snap_.depth_publish_fps =
        static_cast<double>(cur.depth_publish_total - prev_stats_.depth_publish_total) / dt;
    diag_snap_.color_decode_delta_count =
        cur.color_publish_total - prev_stats_.color_publish_total;
    const uint64_t color_decode_delta_sum =
        cur.color_decode_sum_us - prev_stats_.color_decode_sum_us;
    diag_snap_.color_decode_avg_ms =
        diag_snap_.color_decode_delta_count > 0
            ? color_decode_delta_sum / 1000.0 / diag_snap_.color_decode_delta_count
            : 0.0;
    diag_snap_.pc_count_delta = cur.pc_publish_total - prev_stats_.pc_publish_total;
    const uint64_t pc_sum_delta = cur.pc_compute_sum_us - prev_stats_.pc_compute_sum_us;
    diag_snap_.pc_publish_fps =
        diag_snap_.pc_count_delta > 0
            ? static_cast<double>(diag_snap_.pc_count_delta) / dt
            : 0.0;
    diag_snap_.pc_compute_avg_ms =
        diag_snap_.pc_count_delta > 0
            ? pc_sum_delta / 1000.0 / diag_snap_.pc_count_delta
            : 0.0;

    prev_stats_ = cur;
    prev_stats_wall_ = stamp;
}

void CameraNodelet::diagnose_device(diagnostic_updater::DiagnosticStatusWrapper& s) {
    // Always first in registration order — refresh the shared snapshot here
    // so the other tasks in this round reuse the same numbers.
    refresh_diag_snapshot();

    using DS = diagnostic_msgs::DiagnosticStatus;
    using SS = EspdiDevice::StreamState;
    const ConnState state = conn_state_.load();
    const auto stream_state = device_->stream_state();
    const int actual_fps = device_->actual_fps();
    const int half_expected = std::max(1, actual_fps / 2);

    if (state == ConnState::kDisconnected) {
        s.summary(DS::ERROR, "camera disconnected; Linux device node not present");
    } else if (stream_state == SS::Standby) {
        s.summary(DS::OK, "standby (USB pipe closed by operator)");
    } else if (stream_state == SS::Paused) {
        s.summary(DS::OK, "streaming (paused — publish gated by operator)");
    } else {
        // Aggregate liveness: ERROR only when every configured stream is
        // below 50% of expected; per-stream WARN is the colour/depth tasks'
        // job. A zero colour rate is normal in D-only modes, so colour counts
        // as dead only once it has delivered a frame.
        const bool color_configured = diag_snap_.cur.color_input_total > 0
            || diag_snap_.color_input_fps > 0.0;
        const bool color_dead = color_configured
            && diag_snap_.color_input_fps < 0.5 * half_expected;
        const bool depth_dead = diag_snap_.depth_input_fps < 0.5 * half_expected;
        if (color_dead && depth_dead) {
            s.summary(DS::ERROR, "no frames flowing on any configured stream");
        } else {
            s.summary(DS::OK, "streaming");
        }
    }

    s.add("connection_state",   state == ConnState::kStreaming ? "streaming" : "disconnected");
    s.add("device_present",     state == ConnState::kStreaming ? "true" : "false");
    s.add("reconnect_attempts", reconnect_attempts_.load(std::memory_order_relaxed));
    s.add("usb_port",           device_->usb_port().empty() ? "n/a" : device_->usb_port());
    s.add("serial_number",      device_->serial_number().empty() ? "n/a" : device_->serial_number());
    s.add("actual_fps",         device_->actual_fps());
    s.add("stream_state",
          (stream_state == SS::Active)  ? "Active"
        : (stream_state == SS::Paused)  ? "Paused"
                                        : "Standby");
}

// Per-stream liveness shared by the color and depth tasks. Returns the level
// + message for the input rate against the expected per-stream rate (half of
// actual_fps in interleave modes where both streams share one USB endpoint).
// `allow_zero` covers the color side in D-only modes where no color stream is
// configured.
namespace {
struct StreamHealth { unsigned char level; const char* message; };
StreamHealth classify_stream(double input_fps, int actual_fps,
                             EspdiDevice::StreamState stream_state,
                             bool allow_zero) {
    using DS = diagnostic_msgs::DiagnosticStatus;
    using SS = EspdiDevice::StreamState;
    if (stream_state == SS::Standby) return {DS::OK, "standby"};
    const int per_stream_expected = std::max(1, actual_fps / 2);
    if (allow_zero && input_fps == 0.0) {
        return {DS::OK, "not configured (D-only mode)"};
    }
    if (input_fps < 0.5 * per_stream_expected) {
        return {DS::WARN, "input rate below 50% of expected"};
    }
    return {DS::OK, "streaming"};
}
}  // namespace

void CameraNodelet::diagnose_color(diagnostic_updater::DiagnosticStatusWrapper& s) {
    using DS = diagnostic_msgs::DiagnosticStatus;
    if (conn_state_.load() == ConnState::kDisconnected) {
        s.summary(DS::OK, "(disconnected — see device task)");
        return;
    }
    const auto h = classify_stream(diag_snap_.color_input_fps,
                                   device_->actual_fps(),
                                   device_->stream_state(),
                                   /*allow_zero=*/true);
    s.summary(h.level, h.message);

    s.addf("input_fps",     "%.2f", diag_snap_.color_input_fps);
    s.addf("publish_fps",   "%.2f", diag_snap_.color_publish_fps);
    s.add ("input_total",   diag_snap_.cur.color_input_total);
    s.add ("input_dropped", diag_snap_.cur.color_input_dropped);
    s.add ("publish_total", diag_snap_.cur.color_publish_total);
    // Decode timings only when frames decoded in the window (i.e. a
    // subscriber is present).
    if (diag_snap_.color_decode_delta_count > 0) {
        s.addf("decode_avg_ms", "%.2f", diag_snap_.color_decode_avg_ms);
        s.addf("decode_max_ms", "%.2f", diag_snap_.cur.color_decode_max_us / 1000.0);
    }
}

void CameraNodelet::diagnose_depth(diagnostic_updater::DiagnosticStatusWrapper& s) {
    using DS = diagnostic_msgs::DiagnosticStatus;
    if (conn_state_.load() == ConnState::kDisconnected) {
        s.summary(DS::OK, "(disconnected — see device task)");
        return;
    }
    const auto h = classify_stream(diag_snap_.depth_input_fps,
                                   device_->actual_fps(),
                                   device_->stream_state(),
                                   /*allow_zero=*/false);
    s.summary(h.level, h.message);

    s.addf("input_fps",     "%.2f", diag_snap_.depth_input_fps);
    s.addf("publish_fps",   "%.2f", diag_snap_.depth_publish_fps);
    s.add ("input_total",   diag_snap_.cur.depth_input_total);
    s.add ("input_dropped", diag_snap_.cur.depth_input_dropped);
    s.add ("publish_total", diag_snap_.cur.depth_publish_total);
}

void CameraNodelet::diagnose_pc(diagnostic_updater::DiagnosticStatusWrapper& s) {
    using DS = diagnostic_msgs::DiagnosticStatus;
    if (conn_state_.load() == ConnState::kDisconnected) {
        s.summary(DS::OK, "(disconnected — see device task)");
        return;
    }
    const char* status_str;
    if (diag_snap_.pc_count_delta > 0) {
        status_str = "active";
        s.summary(DS::OK, status_str);
        s.addf("publish_fps",    "%.2f", diag_snap_.pc_publish_fps);
        s.addf("compute_avg_ms", "%.2f", diag_snap_.pc_compute_avg_ms);
        s.addf("compute_max_ms", "%.2f", diag_snap_.cur.pc_compute_max_us / 1000.0);
        s.add ("publish_total",  diag_snap_.cur.pc_publish_total);
    } else if (diag_snap_.cur.pc_publish_total > 0) {
        status_str = "idle (no /depth/points subscriber)";
        s.summary(DS::OK, status_str);
        s.addf("publish_fps",    "%.2f", 0.0);
        s.addf("compute_max_ms", "%.2f", diag_snap_.cur.pc_compute_max_us / 1000.0);
        s.add ("publish_total",  diag_snap_.cur.pc_publish_total);
    } else {
        status_str = "idle (never run ; no subscriber since start)";
        s.summary(DS::OK, status_str);
        s.addf("publish_fps",    "%.2f", 0.0);
    }
    s.add("compute_status",        status_str);
    s.add("spatial_filter_total",  diag_snap_.cur.spatial_filter_total);
    s.add("temporal_filter_total", diag_snap_.cur.temporal_filter_total);
    s.add("hole_fill_total",       diag_snap_.cur.hole_fill_total);
}

void CameraNodelet::diagnose_thermal(diagnostic_updater::DiagnosticStatusWrapper& s) {
    using DS = diagnostic_msgs::DiagnosticStatus;
    if (conn_state_.load() == ConnState::kDisconnected) {
        s.summary(DS::OK, "(disconnected — SDK handle released)");
        return;
    }
    // Pause the sensor read during a self-cal session: selfk's worker writes
    // cy on the shared handle without sdk_mtx, so a concurrent control read
    // could interleave two UVC transfers on one handle.
    if (device_->selfcal_active()) {
        s.summary(DS::OK, "paused (self-calibration in progress)");
        return;
    }
    // APC_GetTemperature is a USB roundtrip; this runs once per tick.
    const auto thermal = device_->read_temperature();
    if (!thermal.supported) {
        s.summary(DS::OK, "not supported on this model");
    } else if (!thermal.read_ok) {
        s.summary(DS::WARN, "read failed");
    } else {
        s.summary(DS::OK, "ok");
        s.addf("temperature_c", "%.2f", thermal.celsius);
    }
}

// ===================================================================
// Self-calibration action
// ===================================================================

void CameraNodelet::selfcal_clear_goal_locked() {
    selfcal_run_goal_.reset();
    selfcal_run_timer_.stop();
}

void CameraNodelet::selfcal_handle_goal(SelfCalGoalHandle gh) {
    SelfCalResult res;
    if (!device_->selfcal_available()) {
        NODELET_WARN("selfcal/run rejected: not available");
        res.outcome = "FAILED";
        res.message = "self-calibration is not available on this module";
        gh.setRejected(res, res.message);
        return;
    }

    std::lock_guard<std::mutex> lk(selfcal_goal_mtx_);
    if (device_->selfcal_active() || selfcal_run_goal_) {
        NODELET_WARN("selfcal/run rejected: a session is already running");
        res.outcome = "FAILED";
        res.message = "a self-calibration session is already running";
        gh.setRejected(res, res.message);
        return;
    }

    const auto goal = gh.getGoal();
    std::string profile = goal->profile;
    if (profile.empty()) {
        boost::recursive_mutex::scoped_lock rlk(reconf_mtx_);
        profile = reconf_cur_.selfcal_profile;
    }
    selfcal_run_auto_commit_shift_px_ = goal->auto_commit_shift_px;

    gh.setAccepted();

    if (!device_->start_selfcal(profile)) {
        res.outcome = "FAILED";
        res.message = "could not start self-calibration (see node log)";
        gh.setAborted(res, res.message);
        return;
    }
    selfcal_run_goal_ = std::make_unique<SelfCalGoalHandle>(gh);
    // The one-shot profile runs ~20-30 s, then the A/B re-check adds a few
    // more; give a generous deadline before declaring a timeout and
    // reverting.
    selfcal_run_deadline_ = ros::Time::now() + ros::Duration(60.0);
    selfcal_run_timer_ = nh_.createTimer(ros::Duration(0.25),
                                         &CameraNodelet::selfcal_run_tick, this);
    NODELET_INFO("selfcal/run started (profile='%s')", profile.c_str());
}

void CameraNodelet::selfcal_handle_cancel(SelfCalGoalHandle) {
    // A session runs to completion and cannot be interrupted; if the result
    // is worse it auto-reverts, so there is nothing a cancel needs to undo.
    // The goal is deliberately left active.
    NODELET_WARN("selfcal/run: cancel rejected — a session cannot be interrupted");
}

void CameraNodelet::selfcal_run_tick(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lk(selfcal_goal_mtx_);
    if (!selfcal_run_goal_) {
        selfcal_run_timer_.stop();
        return;
    }
    // A disconnect mid-run is recovered by the watchdog's reopen (which
    // reloads flash); fail the goal promptly instead of polling a dead
    // session.
    if (conn_state_.load() != ConnState::kStreaming) {
        SelfCalResult res;
        res.outcome = "FAILED";
        res.message = "camera disconnected during calibration; aborted";
        selfcal_run_goal_->setAborted(res, res.message);
        selfcal_clear_goal_locked();
        return;
    }
    const auto st = device_->selfcal_status();

    const bool terminal = st.state == "COMPLETED" || st.state == "STOPPED" ||
                          st.state == "ERROR";
    // After the SDK converges, the worker runs the A/B re-check before the
    // session is resolvable; surface that as a distinct feedback phase.
    const bool rechecking = terminal && !st.resolve_ready;

    SelfCalFeedback fb;
    fb.phase = rechecking ? "RECHECK" : st.phase;
    fb.progress = st.progress;
    fb.processed_frames = static_cast<uint32_t>(st.processed_frames);
    fb.valid_ratio_latest = st.valid_ratio_latest;
    selfcal_run_goal_->publishFeedback(fb);

    const bool timed_out = ros::Time::now() >= selfcal_run_deadline_;
    // Wait for the A/B re-check to finish (resolve_ready) before acting,
    // unless the deadline has passed.
    if (!(terminal && st.resolve_ready) && !timed_out) {
        return;
    }

    SelfCalResult res;
    // The driver's own wall-clock deadline can fire before the SDK ever
    // produces a result, leaving st.outcome empty; report that case as
    // TIMEOUT rather than shipping an empty outcome string.
    res.outcome = timed_out ? "TIMEOUT" : st.outcome;
    res.valid_ratio_first = st.valid_ratio_first;
    res.valid_ratio_latest = st.valid_ratio_latest;
    res.valid_ratio_delta = st.valid_ratio_delta;
    res.correction_level = st.correction_level;
    res.cy_shift_px = st.cy_shift_px;
    res.recheck_verdict = st.ab_verdict;
    res.recheck_ratio_before = st.ab_ratio_initial;
    res.recheck_ratio_after = st.ab_ratio_final;

    // Resolve on the SDK outcome plus the A/B re-check. Anything that is not
    // SUCCESS or NO_CHANGE (including a hard ERROR / empty outcome) is a
    // failure and reverts; only a SUCCESS reaches the keep/commit paths.
    const bool failed = timed_out ||
                        (st.outcome != "SUCCESS" && st.outcome != "NO_CHANGE");

    if (failed) {
        // Roll the registers back to the pre-session calibration.
        res.reverted = device_->revert_selfcal();
        const char* why = timed_out ? "timed out" : "calibration failed";
        res.message = res.reverted
            ? std::string(why) + "; reverted to the previous calibration"
            : std::string(why) + "; ROLLBACK ALSO FAILED — the camera is on the "
              "rejected alignment until it is power-cycled";
    } else if (st.outcome == "NO_CHANGE") {
        // Nothing changed; restore the pre-session cy (a safe no-op). Not
        // applied, reverted, or committed.
        device_->revert_selfcal();
        res.message = "calibration already optimal; no change made";
    } else if (st.ab_verdict == "worse") {
        // SUCCESS moved cy, but the live A/B re-check shows the new alignment
        // is actually worse on this scene. Roll it back.
        res.reverted = device_->revert_selfcal();
        res.message = res.reverted
            ? "the re-check showed the new alignment was worse; reverted"
            : "the re-check showed the new alignment was worse, but the rollback "
              "failed — the camera is on the rejected alignment until it is "
              "power-cycled";
    } else if (st.ab_verdict == "inconclusive") {
        // Could not certify an improvement (scene too unstable during the
        // re-check, or the difference was within noise). Be conservative.
        res.reverted = device_->revert_selfcal();
        res.message = res.reverted
            ? "could not verify an improvement (unstable scene during the "
              "re-check); reverted — hold the camera still and re-run"
            : "could not verify an improvement, and the rollback failed — the "
              "camera is on the rejected alignment until it is power-cycled";
    } else if (selfcal_run_auto_commit_shift_px_ > 0.0f && st.can_commit &&
               st.cy_shift_px >= selfcal_run_auto_commit_shift_px_) {
        // Verified better (or re-check skipped) and the cy shift is large
        // enough to persist automatically. The gate is strictly positive so
        // that an omitted goal field — which ROS 1 delivers as 0.0, having no
        // per-field defaults — means never-commit. See SelfCal.action.
        if (device_->commit_selfcal()) {
            res.applied = true;
            res.committed = true;
            res.message = "verified improved and committed to flash";
        } else {
            device_->keep_selfcal();
            res.applied = true;
            res.message = "verified improved and kept live; flash commit failed (see log)";
        }
    } else {
        // Verified better (or re-check skipped), kept live for review.
        device_->keep_selfcal();
        res.applied = true;
        res.message = "verified improved and kept live; call selfcal/commit to persist";
    }

    NODELET_INFO("selfcal/run resolved: outcome=%s recheck=%s(%.3f->%.3f) "
                 "reverted=%d committed=%d",
                 res.outcome.c_str(), res.recheck_verdict.c_str(),
                 res.recheck_ratio_before, res.recheck_ratio_after,
                 res.reverted ? 1 : 0, res.committed ? 1 : 0);

    // Abort when the session did not reach a usable answer, or when a rollback
    // it intended did not take; a deliberate revert is the designed outcome and
    // succeeds. res.outcome carries the detail either way.
    const bool meant_to_revert =
        (st.ab_verdict == "worse" || st.ab_verdict == "inconclusive");
    if (failed || (meant_to_revert && !res.reverted)) {
        selfcal_run_goal_->setAborted(res, res.message);
    } else {
        selfcal_run_goal_->setSucceeded(res, res.message);
    }
    selfcal_clear_goal_locked();
}

}  // namespace eys3d_camera
