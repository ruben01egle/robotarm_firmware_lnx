#ifndef ROBOTARM_CONTROLLERS_TEST_HELPERS_HPP
#define ROBOTARM_CONTROLLERS_TEST_HELPERS_HPP

// Shared by the controller tests: the real URDF, and log handlers that keep the allocation count
// of malloc_counter.hpp meaningful while the controller logs.

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcutils/logging.h"

#include "malloc_counter.hpp"

namespace test_helpers
{

// URDF of the real robot, processed by xacro (same as robotarm_rbd/test/real_urdf.hpp, whose test
// directory is not installed)
inline std::string real_urdf()
{
    static const std::string cached = [] {
        const std::string share = ament_index_cpp::get_package_share_directory("robotarm_description");
        const std::string cmd = "xacro '" + share + "/urdf/robotarm.urdf.xacro' 2>/dev/null";
        FILE * pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("cannot run xacro");
        }
        std::string out;
        char buf[4096];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), pipe)) > 0) {
            out.append(buf, n);
        }
        if (pclose(pipe) != 0 || out.empty()) {
            throw std::runtime_error("xacro failed on " + share + "/urdf/robotarm.urdf.xacro");
        }
        return out;
    }();
    return cached;
}

// "prefix/name" -> {"prefix", "name"}, split at the last slash like ros2_control interface names
inline std::pair<std::string, std::string> split_interface_name(const std::string & full)
{
    const size_t slash = full.rfind('/');
    return {full.substr(0, slash), full.substr(slash + 1)};
}

// Wraps the installed log output handler and pauses the allocation counter while it runs. The
// logging backend allocates for every message that is actually printed (rosout publishes over DDS),
// in update() those are throttled warnings and the errors right before a stop, accepted as rare.
// Everything up to the log call, incl. the throttle check, is still counted. Installed once in
// main(), after rclcpp::init() has set up the logging backend.
class LogAllocationExclusion
{
public:
    static void install()
    {
        previous_ = rcutils_logging_get_output_handler();
        rcutils_logging_set_output_handler(&LogAllocationExclusion::handler);
    }

private:
    static void handler(
        const rcutils_log_location_t * location, int severity, const char * name,
        rcutils_time_point_value_t timestamp, const char * format, va_list * args)
    {
        malloc_counter::ScopedPause pause;
        if (previous_) {
            previous_(location, severity, name, timestamp, format, args);
        }
    }

    static inline rcutils_logging_output_handler_t previous_ = nullptr;
};

// Counts log messages whose format string contains a marker and forwards everything to the handler
// that was installed before. Forwarding keeps the real logging backend inside the measured window.
// One instance at a time, its state is static.
class LogMarkerCounter
{
public:
    explicit LogMarkerCounter(const char * marker)
    {
        marker_ = marker;
        hits_ = 0;
        previous_ = rcutils_logging_get_output_handler();
        rcutils_logging_set_output_handler(&LogMarkerCounter::handler);
    }
    ~LogMarkerCounter() {rcutils_logging_set_output_handler(previous_);}

    int hits() const {return hits_.load();}

private:
    static void handler(
        const rcutils_log_location_t * location, int severity, const char * name,
        rcutils_time_point_value_t timestamp, const char * format, va_list * args)
    {
        if (format && std::strstr(format, marker_)) {
            ++hits_;
        }
        if (previous_) {
            previous_(location, severity, name, timestamp, format, args);
        }
    }

    static inline const char * marker_ = nullptr;
    static inline std::atomic<int> hits_{0};
    static inline rcutils_logging_output_handler_t previous_ = nullptr;
};

}  // namespace test_helpers

#endif  // ROBOTARM_CONTROLLERS_TEST_HELPERS_HPP
