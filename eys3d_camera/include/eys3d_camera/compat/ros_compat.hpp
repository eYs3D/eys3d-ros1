// ROS 1 compatibility shim for the shared eYs3D driver core.
//
// The device layer (espdi_device.cpp), the DSP helpers, and the small
// catalogue/lookup translation units are ROS-agnostic apart from logging:
// they touch rclcpp only through RCLCPP_* macros, rclcpp::Logger, and a
// steady-time rclcpp::Clock used to throttle hot-loop warnings. Mapping
// those onto rosconsole lets those files compile unmodified against
// roscpp.
//
// Only the logging surface is shimmed. Emulating the node, parameter,
// publisher and action APIs would hide real design differences, so
// camera_nodelet and selfcal_manager are written natively against
// roscpp instead.
#ifndef EYS3D_CAMERA_COMPAT_ROS_COMPAT_HPP
#define EYS3D_CAMERA_COMPAT_ROS_COMPAT_HPP

#include <ros/console.h>

#include <string>
#include <utility>

// rclcpp::Clock is constructed with a clock-source tag. Only the steady
// source is used (throttling must not jump when wall time is stepped);
// rosconsole's throttle keeps its own steady reference, so the tag is
// accepted and ignored.
enum rcl_clock_type_t {
    RCL_CLOCK_UNINITIALIZED = 0,
    RCL_ROS_TIME,
    RCL_SYSTEM_TIME,
    RCL_STEADY_TIME,
};

namespace rclcpp {

// Carries the logger name only. ROS 1 resolves a logger by name at the
// call site (ROSCONSOLE_NAME_PREFIX + "." + name) rather than holding a
// logger object, so the name is the entire state.
class Logger {
public:
    Logger() = default;
    explicit Logger(std::string name) : name_(std::move(name)) {}

    const std::string& name() const { return name_; }

private:
    std::string name_{"eys3d_camera"};
};

inline Logger get_logger(const std::string& name) { return Logger(name); }

// Accepts an existing Logger so `rclcpp::get_logger(logger())` compiles
// whether the callee hands back a name or an already-built logger.
inline Logger get_logger(const Logger& logger) { return logger; }

class Clock {
public:
    Clock() = default;
    explicit Clock(rcl_clock_type_t) {}
};

}  // namespace rclcpp

// The RCLCPP_* macros take the logger as their first argument; the ROS 1
// _NAMED forms take the name. Unwrap the logger and forward the rest.
#define EYS3D_ROS1_LOG_NAME(logger) ((logger).name())

#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG_NAMED(EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_INFO(logger, ...)  ROS_INFO_NAMED (EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_WARN(logger, ...)  ROS_WARN_NAMED (EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR_NAMED(EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_FATAL(logger, ...) ROS_FATAL_NAMED(EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)

#define RCLCPP_DEBUG_ONCE(logger, ...) ROS_DEBUG_ONCE_NAMED(EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_INFO_ONCE(logger, ...)  ROS_INFO_ONCE_NAMED (EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_WARN_ONCE(logger, ...)  ROS_WARN_ONCE_NAMED (EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_ERROR_ONCE(logger, ...) ROS_ERROR_ONCE_NAMED(EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)

// The RCLCPP_* throttle macros take (clock, milliseconds); ROS 1 takes
// (seconds). The clock argument is consumed and discarded — see the
// Clock note above.
#define EYS3D_ROS1_THROTTLE_PERIOD(clock, ms) (static_cast<void>(clock), (ms) / 1000.0)

#define RCLCPP_DEBUG_THROTTLE(logger, clock, ms, ...) \
    ROS_DEBUG_THROTTLE_NAMED(EYS3D_ROS1_THROTTLE_PERIOD(clock, ms), EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) \
    ROS_INFO_THROTTLE_NAMED (EYS3D_ROS1_THROTTLE_PERIOD(clock, ms), EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) \
    ROS_WARN_THROTTLE_NAMED (EYS3D_ROS1_THROTTLE_PERIOD(clock, ms), EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) \
    ROS_ERROR_THROTTLE_NAMED(EYS3D_ROS1_THROTTLE_PERIOD(clock, ms), EYS3D_ROS1_LOG_NAME(logger), __VA_ARGS__)

#endif  // EYS3D_CAMERA_COMPAT_ROS_COMPAT_HPP
