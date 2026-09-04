// Standalone entry point for the eYs3D camera driver.
//
// The driver itself is a nodelet; this executable loads that nodelet into
// its own process through nodelet::Loader, so the standalone and the
// managed (nodelet manager) deployments run byte-identical code. Only the
// zero-copy story differs: inside a shared manager, publishers hand
// subscribers the same shared_ptr instead of serialising.

#include <malloc.h>

#include <csignal>
#include <mutex>

#include <nodelet/loader.h>
#include <ros/ros.h>

namespace {
// roscpp installs a SIGINT handler but leaves SIGTERM at its default
// (immediate termination), which would skip every C++ destructor — the
// fetch threads would never be joined and the SDK handle never released.
// Re-raise SIGINT from SIGTERM so container runtimes (docker stop,
// systemctl stop, kubectl delete pod) reach the same clean shutdown path
// as Ctrl-C.
void install_sigterm_to_shutdown() {
    static std::once_flag once;
    std::call_once(once, [] {
        struct sigaction sa{};
        sa.sa_handler = [](int) { raise(SIGINT); };
        sigemptyset(&sa.sa_mask);
        sigaction(SIGTERM, &sa, nullptr);
    });
}
}  // namespace

int main(int argc, char** argv) {
    // Keep large per-frame allocations (image and point cloud, several MB
    // each) in glibc's heap so resizes hit the cached free list instead of an
    // mmap / munmap round trip on every publish.
    //   M_MMAP_MAX       = 0   — allocate from the heap, not mmap
    //   M_TRIM_THRESHOLD = -1  — never return heap segments to the OS
    // Only this executable sets them: under a shared nodelet manager the
    // process is not ours to tune.
    mallopt(M_MMAP_MAX, 0);
    mallopt(M_TRIM_THRESHOLD, -1);

    ros::init(argc, argv, "eys3d_camera");
    install_sigterm_to_shutdown();

    nodelet::Loader loader;
    nodelet::M_string remappings(ros::names::getRemappings());
    nodelet::V_string my_argv(argv + 1, argv + argc);

    if (!loader.load(ros::this_node::getName(), "eys3d_camera/CameraNodelet",
                     remappings, my_argv)) {
        ROS_FATAL("Failed to load the eys3d_camera/CameraNodelet nodelet");
        return 1;
    }

    ros::spin();
    return 0;
}
