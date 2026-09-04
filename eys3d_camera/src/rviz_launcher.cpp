// Starts RViz on a layout that addresses the camera actually running.
//
// The bundled layouts name each model's default camera (<MODEL>_1) in every
// topic path, in the robot_description parameter and in the Fixed Frame.
// Under a different camera_name this rewrites the layout into the temporary
// directory; the default name and an operator-supplied --config are opened
// untouched.
//
// Usage (from camera.launch):
//   rviz_launcher --default-layout=<path> --camera-name=<name>
//                 --default-name=<name> --config=[path]
//
// Each option must stay one token: roslaunch appends its own __name:= and
// __log:= after these, so a flag whose value came out empty would consume
// one of them. Everything not consumed here is forwarded to rviz, those two
// included, and the exec keeps this a single process for roslaunch to watch.

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string temp_dir() {
    const char* t = std::getenv("TMPDIR");
    return (t && *t) ? std::string(t) : std::string("/tmp");
}

// Rewrites every mention of default_name to camera_name. Returns the path to
// use, which is the template itself when the rewrite could not be done — a
// layout addressing the wrong camera still opens, where no layout at all
// drops the operator into a bare RViz.
std::string resolve_layout(const std::string& tmpl,
                           const std::string& default_name,
                           const std::string& camera_name) {
    std::ifstream in(tmpl);
    if (!in) {
        std::fprintf(stderr,
                     "rviz_launcher: cannot read '%s'; opening it as-is\n",
                     tmpl.c_str());
        return tmpl;
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    std::string layout = buf.str();

    for (std::string::size_type at = layout.find(default_name);
         at != std::string::npos;
         at = layout.find(default_name, at + camera_name.size())) {
        layout.replace(at, default_name.size(), camera_name);
    }

    // One deterministic path per camera name, rewritten on every launch, so
    // repeated runs neither accumulate files nor read a stale one.
    const std::string out = temp_dir() + "/eys3d_camera_" + camera_name + ".rviz";
    std::ofstream os(out, std::ios::trunc);
    if (!os) {
        std::fprintf(stderr,
                     "rviz_launcher: cannot write '%s'; opening the default "
                     "layout, whose topics and Fixed Frame name '%s'\n",
                     out.c_str(), default_name.c_str());
        return tmpl;
    }
    os << layout;
    os.close();
    if (!os) {
        std::fprintf(stderr, "rviz_launcher: failed writing '%s'; opening the "
                             "default layout instead\n", out.c_str());
        return tmpl;
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string default_layout, camera_name, default_name, config;
    std::vector<std::string> passthrough;

    const auto take = [](const std::string& arg, const char* flag,
                         std::string* out) {
        const std::string prefix = std::string(flag) + "=";
        if (arg.compare(0, prefix.size(), prefix) != 0) return false;
        *out = arg.substr(prefix.size());
        return true;
    };
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (take(a, "--default-layout", &default_layout)) continue;
        if (take(a, "--camera-name",    &camera_name))    continue;
        if (take(a, "--default-name",   &default_name))   continue;
        if (take(a, "--config",         &config))         continue;
        passthrough.push_back(a);
    }

    std::string layout = config;
    if (layout.empty()) {
        layout = default_layout;
        if (!layout.empty() && !camera_name.empty() &&
            !default_name.empty() && camera_name != default_name) {
            layout = resolve_layout(layout, default_name, camera_name);
        }
    }

    std::vector<char*> args;
    args.push_back(const_cast<char*>("rviz"));
    if (!layout.empty()) {
        args.push_back(const_cast<char*>("-d"));
        args.push_back(const_cast<char*>(layout.c_str()));
    }
    for (auto& a : passthrough) args.push_back(const_cast<char*>(a.c_str()));
    args.push_back(nullptr);

    execvp("rviz", args.data());
    std::perror("rviz_launcher: exec rviz");
    return 127;
}
