#ifndef MOTEUS_INTERFACE_TRANSPORT_TIMING_HPP
#define MOTEUS_INTERFACE_TRANSPORT_TIMING_HPP

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace moteus_interface::transport
{

inline uint64_t transport_dt_us(const struct timespec& start, const struct timespec& end) {
    const int64_t sec_diff  = static_cast<int64_t>(end.tv_sec)  - static_cast<int64_t>(start.tv_sec);
    const int64_t nsec_diff = static_cast<int64_t>(end.tv_nsec) - static_cast<int64_t>(start.tv_nsec);
    const int64_t total_ns  = sec_diff * 1000000000LL + nsec_diff;

    // Should never happen with CLOCK_MONOTONIC_RAW and correctly-ordered
    // start/end, but never let a negative diff wrap into a huge uint64_t.
    if (total_ns < 0) {
        return 0;
    }
    return static_cast<uint64_t>(total_ns) / 1000ULL;
}

// Per-cycle timing instrumentation for a Transport. Meant to be held as a
// plain (non-pointer) member on every Transport so all derived transports
// get it for free.
//
// Stages measured:
//   encode_us         : building the outgoing command buffer
//   write_syscall_us  : the ::write() call to the kernel/tty layer
//   roundtrip_us      : syscall return -> first reply byte seen
//                       (USB out + device firmware + bus + USB in - i.e.
//                       everything NOT under your code's control)
//   drain_us          : first reply byte -> all expected replies parsed
//
// Cost when disabled (default): one predicted-true bool branch per
// mark_*() call, no clock_gettime, no allocation. Effectively free.
//
// Cost when enabled: 4x clock_gettime() (~20-40ns each via vDSO) plus a
// struct write into a preallocated vector per cycle - low hundreds of ns
// total, negligible against a several-hundred-microsecond cycle. Safe to
// leave enabled during real reliability testing.
//
// IMPORTANT: only call set_enabled()/reset() from outside the RT loop
// (e.g. node startup, a service callback between runs). Both can
// allocate/free memory and must never be called from inside
// cycle()/write()/read().
class TransportTiming
{
public:
    explicit TransportTiming(rclcpp::Logger logger): logger_(logger) {}

    struct TimingSample {
        uint64_t encode_us;
        uint64_t write_syscall_us;
        uint64_t roundtrip_us;
        uint64_t drain_us;
        uint32_t expected_replies;
        uint32_t received_replies;
        bool     timed_out;
    };

    void set_enabled(bool enabled, size_t capacity = 20000) {
        enabled_ = enabled;
        log_.clear();
        if (enabled) {
            log_.reserve(capacity);   // one allocation, up front, off the RT path
            log_full_warned_ = false;
        } else {
            log_.shrink_to_fit();     // release the memory when not in use
        }
    }

    bool enabled() const { return enabled_; }

    void reset() { log_.clear(); }

    bool dump(const std::string& path) const {
        FILE* f = std::fopen(path.c_str(), "w");
        if (!f) {
            RCLCPP_ERROR(logger_, "TransportTiming: failed to open log path '%s': %s",
                         path.c_str(), std::strerror(errno));
            return false;
        }

        std::fprintf(f, "sample,encode_us,write_syscall_us,roundtrip_us,drain_us,total_us,"
                         "expected_replies,received_replies,timed_out\n");

        for (size_t i = 0; i < log_.size(); ++i) {
            const auto& s = log_[i];
            const uint64_t total = s.encode_us + s.write_syscall_us + s.roundtrip_us + s.drain_us;
            std::fprintf(f, "%zu,%lu,%lu,%lu,%lu,%lu,%u,%u,%d\n",
                         i,
                         static_cast<unsigned long>(s.encode_us),
                         static_cast<unsigned long>(s.write_syscall_us),
                         static_cast<unsigned long>(s.roundtrip_us),
                         static_cast<unsigned long>(s.drain_us),
                         static_cast<unsigned long>(total),
                         s.expected_replies, s.received_replies, s.timed_out ? 1 : 0);
        }

        std::fclose(f);
        RCLCPP_INFO(logger_, "TransportTiming: wrote %zu samples to '%s'", log_.size(), path.c_str());
        return true;
    }

    // ---- call these from the owning transport at each pipeline stage ----

    // Call right before you start building the outgoing tx buffer.
    inline void mark_write_start() {
        if (!enabled_) return;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &t0_);
    }

    // Call right after the tx buffer is fully built, before the ::write() syscall.
    inline void mark_encode_done() {
        if (!enabled_) return;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &t1_);
    }

    // Call right after the ::write() syscall has accepted all bytes.
    inline void mark_write_done() {
        if (!enabled_) return;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &t2_);
        have_write_timestamps_ = true;
    }

    // Call the first time (and only the first time) data is read back in a
    // given read() call. Pass a bool local to that read() invocation so
    // repeated partial reads within one cycle don't overwrite t3_.
    inline void mark_first_byte_if_needed(bool& have_first_byte_flag) {
        if (!enabled_ || have_first_byte_flag) return;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &t3_);
        have_first_byte_flag = true;
    }

    // Call once at the very end of read(), regardless of success/timeout,
    // to close out and record the sample for this write()+read() cycle.
    inline void record_cycle_end(bool have_first_byte, uint32_t expected_replies,
                                  uint32_t received_replies, bool timed_out) {
        if (!enabled_ || !have_write_timestamps_) {
            have_write_timestamps_ = false;
            return;
        }

        struct timespec t4;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &t4);

        TimingSample sample{};
        sample.encode_us        = transport_dt_us(t0_, t1_);
        sample.write_syscall_us = transport_dt_us(t1_, t2_);
        // If nothing ever came back (full timeout, 0 replies), measure
        // roundtrip out to t4 so it shows up as a large number instead of
        // silently reading as zero.
        sample.roundtrip_us     = have_first_byte ? transport_dt_us(t2_, t3_) : transport_dt_us(t2_, t4);
        sample.drain_us         = have_first_byte ? transport_dt_us(t3_, t4) : 0;
        sample.expected_replies = expected_replies;
        sample.received_replies = received_replies;
        sample.timed_out        = timed_out;

        // Bounded by capacity reserved in set_enabled(); never reallocates
        // on the RT path.
        if (log_.size() < log_.capacity()) {
            log_.push_back(sample);
        } else if (!log_full_warned_) {
            log_full_warned_ = true;
            RCLCPP_WARN(logger_, "TransportTiming: log full (%zu samples), no longer recording.", log_.capacity());
        }

        have_write_timestamps_ = false;
    }

private:
    rclcpp::Logger logger_;
    bool enabled_ = false;
    bool have_write_timestamps_ = false;
    bool log_full_warned_ = false;
    struct timespec t0_{}, t1_{}, t2_{}, t3_{};
    std::vector<TimingSample> log_;
};

}

#endif