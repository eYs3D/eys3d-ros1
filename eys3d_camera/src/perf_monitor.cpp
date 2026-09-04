// Live performance monitor for the eys3d_camera driver. Subscribes with
// topic_tools::ShapeShifter, which keeps the payload as opaque bytes, so the
// driver leaves its no-subscriber idle state.
//
//   SDK : frames received from the camera
//   Pub : frames the driver emitted to the topic
//   Rx  : frames this monitor received
//
// The decode, compute, dropped and temperature figures come from
// /diagnostics; this monitor only reformats them next to the rates it
// measures itself.
//
// Usage:
//   rosrun eys3d_camera perf_monitor                # auto-detect namespace
//   rosrun eys3d_camera perf_monitor --ns /G100P_1
//   rosrun eys3d_camera perf_monitor --interval 0.5

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <ctime>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <ros/master.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <diagnostic_msgs/DiagnosticArray.h>

namespace {

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Discover the first <ns>/left_color or <ns>/depth topic on the graph.
// Returns an empty string when nothing matches.
std::string autodetect_namespace() {
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) return {};
    for (const char* suffix : {"/left_color/image_raw", "/depth/image_raw"}) {
        const std::string suffix_s = suffix;
        for (const auto& t : topics) {
            if (t.datatype != "sensor_msgs/Image") continue;
            if (!ends_with(t.name, suffix_s)) continue;
            return t.name.substr(0, t.name.size() - suffix_s.size());
        }
    }
    return {};
}

// Bumped from the spinner threads, read from the print loop.
struct Counters {
    std::atomic<unsigned long> color{0};
    std::atomic<unsigned long> right_color{0};
    std::atomic<unsigned long> depth{0};
    std::atomic<unsigned long> points{0};
};

// Every /diagnostics KeyValue, flattened to "<task>.<key>", which is how the
// print formatter addresses each figure.
using DiagMap = std::map<std::string, std::string>;

std::string lookup_float(const DiagMap& d, const std::string& key) {
    const auto it = d.find(key);
    if (it == d.end()) return "  --";
    char buf[16];
    try {
        std::snprintf(buf, sizeof(buf), "%5.1f", std::stod(it->second));
        return buf;
    } catch (const std::exception&) {
        return it->second;
    }
}

std::string lookup_str(const DiagMap& d, const std::string& key) {
    const auto it = d.find(key);
    return it == d.end() ? std::string{"--"} : it->second;
}

std::string fmt_rx(double v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%5.1f", v);
    return buf;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "eys3d_perf_monitor",
              ros::init_options::AnonymousName);

    std::string ns;
    double interval = 1.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--ns" && i + 1 < argc)            ns = argv[++i];
        else if (a == "--interval" && i + 1 < argc) interval = std::atof(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::printf("usage: perf_monitor [--ns /NAMESPACE] [--interval SECONDS]\n");
            return 0;
        }
    }
    if (interval <= 0.0) interval = 1.0;

    ros::NodeHandle nh;

    if (ns.empty()) {
        ns = autodetect_namespace();
        if (ns.empty()) {
            std::fprintf(stderr,
                         "perf_monitor: no eys3d_camera topics found on the graph. "
                         "Is the driver running? Pass --ns to name it explicitly.\n");
            return 1;
        }
        std::printf("perf_monitor: auto-detected namespace '%s'\n", ns.c_str());
    }

    Counters rx;
    auto make_cb = [](std::atomic<unsigned long>* counter) {
        return [counter](const topic_tools::ShapeShifter::ConstPtr&) {
            counter->fetch_add(1, std::memory_order_relaxed);
        };
    };

    // Plain-count subscriptions: their presence is also what takes the
    // driver out of its no-subscriber idle state, which is the point.
    std::vector<ros::Subscriber> subs;
    subs.push_back(nh.subscribe<topic_tools::ShapeShifter>(
        ns + "/left_color/image_raw", 5, make_cb(&rx.color)));
    subs.push_back(nh.subscribe<topic_tools::ShapeShifter>(
        ns + "/right_color/image_raw", 5, make_cb(&rx.right_color)));
    subs.push_back(nh.subscribe<topic_tools::ShapeShifter>(
        ns + "/depth/image_raw", 5, make_cb(&rx.depth)));
    subs.push_back(nh.subscribe<topic_tools::ShapeShifter>(
        ns + "/depth/points", 5, make_cb(&rx.points)));

    DiagMap diag;
    // /diagnostics arrives on a spinner thread while the print loop reads it.
    std::mutex diag_mtx;
    ros::Subscriber diag_sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>(
        "/diagnostics", 10,
        [&diag, &diag_mtx](const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
            // The Updater names each status "<node_name>: <task>", e.g.
            // "G100P_1/eys3d_camera: color"; the task is what follows ": ".
            DiagMap flattened;
            for (const auto& st : msg->status) {
                const auto sep = st.name.find(": ");
                if (sep == std::string::npos) continue;
                const std::string task = st.name.substr(sep + 2);
                for (const auto& kv : st.values) {
                    flattened[task + "." + kv.key] = kv.value;
                }
            }
            std::lock_guard<std::mutex> lk(diag_mtx);
            diag = std::move(flattened);
        });

    std::printf("perf_monitor: watching '%s' every %.2f s "
                "(SDK = from camera, Pub = driver emitted, Rx = received here)\n",
                ns.c_str(), interval);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    unsigned long prev_color = 0, prev_right = 0, prev_depth = 0, prev_points = 0;
    ros::Time prev_wall = ros::Time::now();
    ros::Rate rate(1.0 / interval);
    while (ros::ok()) {
        rate.sleep();
        const ros::Time now = ros::Time::now();
        const double dt = std::max(1e-3, (now - prev_wall).toSec());
        prev_wall = now;

        const unsigned long c = rx.color.load(std::memory_order_relaxed);
        const unsigned long r = rx.right_color.load(std::memory_order_relaxed);
        const unsigned long d = rx.depth.load(std::memory_order_relaxed);
        const unsigned long p = rx.points.load(std::memory_order_relaxed);
        const double color_rx  = (c - prev_color)  / dt;
        const double right_rx  = (r - prev_right)  / dt;
        const double depth_rx  = (d - prev_depth)  / dt;
        const double points_rx = (p - prev_points) / dt;
        prev_color = c; prev_right = r; prev_depth = d; prev_points = p;

        DiagMap snap;
        {
            std::lock_guard<std::mutex> lk(diag_mtx);
            snap = diag;
        }

        const std::time_t now_t = std::time(nullptr);
        char ts[16];
        std::strftime(ts, sizeof(ts), "%H:%M:%S", std::localtime(&now_t));

        std::printf("=== eys3d_camera perf [%s] @ %s ===\n", ns.c_str(), ts);
        std::printf("  color | SDK %s \xe2\x86\x92 Pub %s \xe2\x86\x92 Rx %s"
                    "  | decode avg %s ms   max %s ms   dropped %s\n",
                    lookup_float(snap, "color.input_fps").c_str(),
                    lookup_float(snap, "color.publish_fps").c_str(),
                    fmt_rx(color_rx).c_str(),
                    lookup_float(snap, "color.decode_avg_ms").c_str(),
                    lookup_float(snap, "color.decode_max_ms").c_str(),
                    lookup_str(snap, "color.input_dropped").c_str());
        if (r > 0) {
            std::printf("  right | (split_color)                                 "
                        "                Rx %s\n", fmt_rx(right_rx).c_str());
        }
        std::printf("  depth | SDK %s \xe2\x86\x92 Pub %s \xe2\x86\x92 Rx %s"
                    "  |                                       "
                    "   dropped %s\n",
                    lookup_float(snap, "depth.input_fps").c_str(),
                    lookup_float(snap, "depth.publish_fps").c_str(),
                    fmt_rx(depth_rx).c_str(),
                    lookup_str(snap, "depth.input_dropped").c_str());
        std::printf("  pc    |              Pub %s \xe2\x86\x92 Rx %s"
                    "  | compute avg %s ms   max %s ms   total %s\n",
                    lookup_float(snap, "pointcloud.publish_fps").c_str(),
                    fmt_rx(points_rx).c_str(),
                    lookup_float(snap, "pointcloud.compute_avg_ms").c_str(),
                    lookup_float(snap, "pointcloud.compute_max_ms").c_str(),
                    lookup_str(snap, "pointcloud.publish_total").c_str());
        std::printf("  temperature_c %s\n",
                    lookup_str(snap, "thermal.temperature_c").c_str());
        std::fflush(stdout);
    }

    spinner.stop();
    return 0;
}
