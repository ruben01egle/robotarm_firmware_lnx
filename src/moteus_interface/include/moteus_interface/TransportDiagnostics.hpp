#ifndef MOTEUS_INTERFACE_TRANSPORT_DIAGNOSTICS_HPP
#define MOTEUS_INTERFACE_TRANSPORT_DIAGNOSTICS_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string_view>

#include "moteus.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "moteus_interface/TransportTiming.hpp"

namespace moteus_interface::transport
{

// Per-cycle bookkeeping of everything the fdcanusb sends back, so a missing
// reply can be told apart from an adapter-side rejection:
//   OK   : adapter accepted one "can send" (queued, NOT acked on the bus)
//   ERR  : adapter rejected one "can send" -> frame never reached the bus
//   rcv  : a frame received from the bus
//   stray: anything else (garbled/truncated line, malformed rcv)
//
// Normal mode: problems are accumulated and reported as one throttled
// summary line per window. Verbose mode additionally prints a full report
// (expected/replied IDs, arrival times, raw adapter lines) for every cycle
// that had a problem, plus every late ERR/rcv line as it is drained.
//
// No allocation after construction; safe to call from the RT loop. Logging
// only happens when something went wrong.
class TransportDiagnostics
{
public:
    enum class LineKind { Ok, Err, Rcv, Stray };

    static constexpr size_t kMaxIds = 128;
    static constexpr size_t kTextLen = 64;
    static constexpr size_t kRawLines = 32;
    static constexpr size_t kRawLineLen = 96;

    TransportDiagnostics(rclcpp::Logger logger, bool verbose, uint64_t period_us = 1000000)
        : logger_(logger), verbose_(verbose), period_us_(period_us) {}

    // ---- called from the owning transport ----

    // Call from write() before encoding. Records what this cycle expects back.
    void begin_cycle(const mjbots::moteus::CanFdFrame* frames, size_t size) {
        cycle_ = CycleStats{};
        cycle_.frames_sent = static_cast<uint32_t>(size);
        cycle_.batch_pos.fill(-1);
        raw_count_ = 0;
        raw_dropped_ = 0;
        for (size_t i = 0; i < size; ++i) {
            if (!frames[i].reply_required) continue;
            const size_t id = static_cast<size_t>(frames[i].destination) & 0x7f;
            set_bit(cycle_.expected, id);
            cycle_.batch_pos[id] = static_cast<int16_t>(i);
        }
        cycle_.active = true;
    }

    // Call right after the ::write() syscall accepted all bytes.
    void mark_write_done() {
        if (!verbose_) return;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &write_done_);
    }

    // A complete line read during this cycle. For Rcv pass the parsed source.
    // Returns true if the line is a reply that should be handed to the caller
    // (first reply from an expected ID). Duplicates, unexpected sources and
    // stale replies are counted as stale/dup and must be dropped.
    //
    // The adapter answers every "can send" with OK/ERR as soon as it queues
    // it, and a frame's reply needs a bus round trip after that. So the reply
    // to the frame at batch position p can only follow at least p+1 OK/ERR
    // lines; anything earlier is a late reply to the previous batch.
    bool on_line(LineKind kind, std::string_view line, int source = -1) {
        const uint32_t t_us = verbose_ ? since_write_us() : 0;
        if (verbose_) record_raw(t_us, line);

        switch (kind) {
            case LineKind::Ok:
                cycle_.ok++;
                return false;
            case LineKind::Err:
                cycle_.err++;
                copy_text(cycle_.last_err, line);
                return false;
            case LineKind::Stray:
                cycle_.stray++;
                copy_text(cycle_.last_stray, line);
                return false;
            case LineKind::Rcv:
                break;
        }

        const size_t id = static_cast<size_t>(source) & 0x7f;
        const bool any_expected = cycle_.expected[0] != 0 || cycle_.expected[1] != 0;
        const bool stale = cycle_.batch_pos[id] >= 0 &&
                           cycle_.ok + cycle_.err <= static_cast<uint32_t>(cycle_.batch_pos[id]);
        if (stale || test_bit(cycle_.replied, id) || (any_expected && !test_bit(cycle_.expected, id))) {
            cycle_.stale_dup++;
            if (cycle_.stale_dup_id < 0) cycle_.stale_dup_id = static_cast<int16_t>(id);
            return false;
        }
        set_bit(cycle_.replied, id);
        cycle_.arrival_us[id] = t_us;
        return true;
    }

    // A complete line that belongs to the previous cycle: drained before the
    // next write, or a partial line carried over and completed afterwards.
    void on_late_line(LineKind kind, std::string_view line, int source = -1) {
        switch (kind) {
            case LineKind::Ok:
                // trailing OKs after an early read() exit are normal
                return;
            case LineKind::Err:
                window_.err++;
                copy_text(window_.last_err, line);
                break;
            case LineKind::Stray:
                window_.stray++;
                copy_text(window_.last_stray, line);
                break;
            case LineKind::Rcv:
                window_.late[static_cast<size_t>(source) & 0x7f]++;
                window_.late_total++;
                break;
        }
        window_.has_problem = true;

        if (verbose_) {
            char text[kRawLineLen];
            copy_text(text, line);
            RCLCPP_WARN(logger_, "Transport: late line from previous cycle: '%s'", text);
        }
    }

    // Call once at the end of read(). Missing replies and an OK shortfall are
    // only judged on timeout: after an early exit trailing OKs may still be
    // in flight.
    void end_cycle(bool timed_out) {
        struct timespec now;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        end_cycle_at(timed_out, static_cast<uint64_t>(now.tv_sec) * 1000000ULL +
                                static_cast<uint64_t>(now.tv_nsec) / 1000ULL);
    }

    // Same as end_cycle() with an explicit monotonic timestamp (for tests).
    void end_cycle_at(bool timed_out, uint64_t now_us) {
        if (!cycle_.active) return;
        cycle_.active = false;
        cycle_count_++;

        if (window_start_us_ == 0) {
            window_start_us_ = now_us;
        }

        window_.cycles++;
        window_.err += cycle_.err;
        window_.stray += cycle_.stray;
        window_.stale_dup += cycle_.stale_dup;
        if (cycle_.err > 0) copy_text(window_.last_err, cycle_.last_err);
        if (cycle_.stray > 0) copy_text(window_.last_stray, cycle_.last_stray);

        uint32_t missing_count = 0;
        uint32_t ok_short = 0;
        if (timed_out) {
            window_.short_cycles++;
            for (size_t id = 0; id < kMaxIds; ++id) {
                if (test_bit(cycle_.expected, id) && !test_bit(cycle_.replied, id)) {
                    window_.missing[id]++;
                    missing_count++;
                }
            }
            if (cycle_.ok + cycle_.err < cycle_.frames_sent) {
                ok_short = cycle_.frames_sent - cycle_.ok - cycle_.err;
                window_.ok_short += ok_short;
            }
        }

        const bool cycle_problem = timed_out || cycle_.err > 0 || cycle_.stray > 0 || cycle_.stale_dup > 0;
        if (cycle_problem) {
            window_.has_problem = true;
            if (verbose_) report_cycle(timed_out, missing_count, ok_short);
        }

        maybe_report(now_us);
    }

    // ---- accessors (tests / introspection) ----
    uint32_t cycle_ok() const { return cycle_.ok; }
    uint32_t cycle_err() const { return cycle_.err; }
    uint32_t cycle_stray() const { return cycle_.stray; }
    uint32_t cycle_stale_dup() const { return cycle_.stale_dup; }
    uint32_t window_missing(size_t id) const { return window_.missing[id & 0x7f]; }
    uint32_t window_late(size_t id) const { return window_.late[id & 0x7f]; }
    uint32_t window_ok_short() const { return window_.ok_short; }
    uint32_t window_err() const { return window_.err; }
    uint32_t window_short_cycles() const { return window_.short_cycles; }
    uint32_t summaries_emitted() const { return summaries_emitted_; }

private:
    struct CycleStats {
        bool active = false;
        uint32_t frames_sent = 0;
        uint32_t ok = 0;
        uint32_t err = 0;
        uint32_t stray = 0;
        uint32_t stale_dup = 0;
        int16_t stale_dup_id = -1;
        uint64_t expected[2] = {0, 0};
        uint64_t replied[2] = {0, 0};
        std::array<int16_t, kMaxIds> batch_pos{};
        std::array<uint32_t, kMaxIds> arrival_us{};
        char last_err[kTextLen] = {};
        char last_stray[kTextLen] = {};
    };

    struct WindowStats {
        bool has_problem = false;
        uint32_t cycles = 0;
        uint32_t short_cycles = 0;
        uint32_t err = 0;
        uint32_t ok_short = 0;
        uint32_t stray = 0;
        uint32_t stale_dup = 0;
        uint32_t late_total = 0;
        std::array<uint32_t, kMaxIds> missing{};
        std::array<uint32_t, kMaxIds> late{};
        char last_err[kTextLen] = {};
        char last_stray[kTextLen] = {};
    };

    struct RawLine {
        uint32_t t_us;
        char text[kRawLineLen];
    };

    static void set_bit(uint64_t (&mask)[2], size_t id) { mask[id >> 6] |= (1ULL << (id & 63)); }
    static bool test_bit(const uint64_t (&mask)[2], size_t id) { return (mask[id >> 6] >> (id & 63)) & 1ULL; }

    template <size_t N>
    static void copy_text(char (&dst)[N], std::string_view src) {
        const size_t n = std::min(src.size(), N - 1);
        std::memcpy(dst, src.data(), n);
        dst[n] = '\0';
    }

    uint32_t since_write_us() const {
        struct timespec now;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        return static_cast<uint32_t>(transport_dt_us(write_done_, now));
    }

    void record_raw(uint32_t t_us, std::string_view line) {
        if (raw_count_ >= kRawLines) {
            raw_dropped_++;
            return;
        }
        raw_[raw_count_].t_us = t_us;
        copy_text(raw_[raw_count_].text, line);
        raw_count_++;
    }

    // Appends printf-style text to report_buf_, silently truncating when full.
    template <typename... Args>
    void append(size_t& pos, const char* fmt, Args... args) {
        if (pos >= sizeof(report_buf_) - 1) return;
        const int n = std::snprintf(report_buf_ + pos, sizeof(report_buf_) - pos, fmt, args...);
        if (n > 0) pos = std::min(pos + static_cast<size_t>(n), sizeof(report_buf_) - 1);
    }

    void report_cycle(bool timed_out, uint32_t missing_count, uint32_t ok_short) {
        size_t pos = 0;
        append(pos, "Transport cycle %lu %s: sent %u, OK %u, ERR %u, stray %u, stale/dup %u",
               static_cast<unsigned long>(cycle_count_), timed_out ? "TIMED OUT" : "anomaly",
               cycle_.frames_sent, cycle_.ok, cycle_.err, cycle_.stray, cycle_.stale_dup);
        if (ok_short > 0) append(pos, ", OK short %u", ok_short);

        append(pos, "%s", "\n  replied:");
        for (size_t id = 0; id < kMaxIds; ++id) {
            if (test_bit(cycle_.replied, id)) {
                append(pos, " id%zu@%d(+%uus)", id, cycle_.batch_pos[id], cycle_.arrival_us[id]);
            }
        }
        if (missing_count > 0) {
            append(pos, "%s", "\n  missing:");
            for (size_t id = 0; id < kMaxIds; ++id) {
                if (test_bit(cycle_.expected, id) && !test_bit(cycle_.replied, id)) {
                    append(pos, " id%zu@%d", id, cycle_.batch_pos[id]);
                }
            }
        }
        if (cycle_.stale_dup > 0) append(pos, "\n  first stale/dup source: id%d", cycle_.stale_dup_id);
        if (cycle_.err > 0) append(pos, "\n  last ERR: '%s'", cycle_.last_err);

        append(pos, "\n  raw (%zu lines%s):", raw_count_, raw_dropped_ > 0 ? ", truncated" : "");
        for (size_t i = 0; i < raw_count_; ++i) {
            append(pos, "\n    +%6uus  %s", raw_[i].t_us, raw_[i].text);
        }
        RCLCPP_WARN(logger_, "%s", report_buf_);
    }

    void maybe_report(uint64_t now_us) {
        const uint64_t window_us = now_us - window_start_us_;

        if (!window_.has_problem) {
            if (window_us >= period_us_) reset_window(now_us);
            return;
        }
        // first problem after a quiet period is reported immediately
        if (last_report_us_ != 0 && now_us - last_report_us_ < period_us_) {
            return;
        }

        size_t pos = 0;
        append(pos, "Transport: last %.2fs: %u/%u cycles timed out", static_cast<double>(window_us) / 1e6,
               window_.short_cycles, window_.cycles);
        if (window_.short_cycles > 0) {
            append(pos, "%s", " | missing:");
            for (size_t id = 0; id < kMaxIds; ++id) {
                if (window_.missing[id] > 0) append(pos, " id%zu=%u", id, window_.missing[id]);
            }
        }
        append(pos, " | ERR=%u", window_.err);
        if (window_.err > 0) append(pos, " (last: '%s')", window_.last_err);
        append(pos, " | OK short=%u | stray=%u", window_.ok_short, window_.stray);
        if (window_.stray > 0) append(pos, " (last: '%s')", window_.last_stray);
        append(pos, " | stale/dup=%u", window_.stale_dup);
        if (window_.late_total > 0) {
            append(pos, "%s", " | late:");
            for (size_t id = 0; id < kMaxIds; ++id) {
                if (window_.late[id] > 0) append(pos, " id%zu=%u", id, window_.late[id]);
            }
        }
        RCLCPP_WARN(logger_, "%s", report_buf_);

        summaries_emitted_++;
        last_report_us_ = now_us;
        reset_window(now_us);
    }

    void reset_window(uint64_t now_us) {
        window_ = WindowStats{};
        window_start_us_ = now_us;
    }

    rclcpp::Logger logger_;
    const bool verbose_;
    const uint64_t period_us_;

    CycleStats cycle_;
    WindowStats window_;
    uint64_t cycle_count_ = 0;
    uint64_t window_start_us_ = 0;
    uint64_t last_report_us_ = 0;
    uint32_t summaries_emitted_ = 0;

    struct timespec write_done_{};
    std::array<RawLine, kRawLines> raw_{};
    size_t raw_count_ = 0;
    size_t raw_dropped_ = 0;

    char report_buf_[4096] = {};
};

}

#endif
