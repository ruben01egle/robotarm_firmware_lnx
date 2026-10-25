#include <gtest/gtest.h>

#include <vector>

#include "moteus_interface/TransportDiagnostics.hpp"

using moteus_interface::transport::TransportDiagnostics;
using Kind = TransportDiagnostics::LineKind;

namespace
{

std::vector<mjbots::moteus::CanFdFrame> make_batch(std::initializer_list<int> ids)
{
    std::vector<mjbots::moteus::CanFdFrame> frames;
    for (int id : ids) {
        mjbots::moteus::CanFdFrame f;
        f.destination = static_cast<int8_t>(id);
        f.reply_required = true;
        frames.push_back(f);
    }
    return frames;
}

TransportDiagnostics make_diag()
{
    return TransportDiagnostics(rclcpp::get_logger("test_transport_diagnostics"), false, 1000000);
}

}

TEST(TransportDiagnostics, ClassifiesLines)
{
    auto diag = make_diag();
    auto frames = make_batch({1, 2});
    diag.begin_cycle(frames.data(), frames.size());

    EXPECT_FALSE(diag.on_line(Kind::Ok, "OK"));
    EXPECT_FALSE(diag.on_line(Kind::Err, "ERR something"));
    EXPECT_FALSE(diag.on_line(Kind::Stray, "a0 3f 50"));
    EXPECT_TRUE(diag.on_line(Kind::Rcv, "rcv 0100 00", 1));

    EXPECT_EQ(diag.cycle_ok(), 1u);
    EXPECT_EQ(diag.cycle_err(), 1u);
    EXPECT_EQ(diag.cycle_stray(), 1u);
    EXPECT_EQ(diag.cycle_stale_dup(), 0u);
}

TEST(TransportDiagnostics, DuplicateAndUnexpectedRepliesAreNotCounted)
{
    auto diag = make_diag();
    auto frames = make_batch({1, 2});
    diag.begin_cycle(frames.data(), frames.size());

    diag.on_line(Kind::Ok, "OK");
    EXPECT_TRUE(diag.on_line(Kind::Rcv, "rcv 0100 00", 1));
    EXPECT_FALSE(diag.on_line(Kind::Rcv, "rcv 0100 00", 1));  // duplicate
    EXPECT_FALSE(diag.on_line(Kind::Rcv, "rcv 0700 00", 7));  // not addressed
    EXPECT_EQ(diag.cycle_stale_dup(), 2u);
}

TEST(TransportDiagnostics, MissingOnlyJudgedOnTimeout)
{
    auto diag = make_diag();
    auto frames = make_batch({5, 6});

    diag.begin_cycle(frames.data(), frames.size());
    diag.on_line(Kind::Ok, "OK");
    diag.on_line(Kind::Rcv, "rcv 0500 00", 5);
    diag.end_cycle_at(false, 1000);
    EXPECT_EQ(diag.window_missing(6), 0u);
    EXPECT_EQ(diag.window_ok_short(), 0u);

    diag.begin_cycle(frames.data(), frames.size());
    diag.on_line(Kind::Ok, "OK");
    diag.on_line(Kind::Ok, "OK");
    diag.on_line(Kind::Rcv, "rcv 0500 00", 5);
    diag.end_cycle_at(true, 2000);
    // first problem is reported immediately, which resets the window
    EXPECT_EQ(diag.summaries_emitted(), 1u);
}

TEST(TransportDiagnostics, OkShortfallAndMissingAccumulate)
{
    auto diag = make_diag();
    auto frames = make_batch({1, 2, 3});

    // first problem cycle: reported immediately, window reset
    diag.begin_cycle(frames.data(), frames.size());
    diag.end_cycle_at(true, 1000);
    ASSERT_EQ(diag.summaries_emitted(), 1u);

    // within the throttle period: accumulated, not reported
    diag.begin_cycle(frames.data(), frames.size());
    diag.on_line(Kind::Ok, "OK");
    diag.on_line(Kind::Err, "ERR tx");
    diag.on_line(Kind::Rcv, "rcv 0100 00", 1);
    diag.end_cycle_at(true, 2000);

    EXPECT_EQ(diag.summaries_emitted(), 1u);
    EXPECT_EQ(diag.window_short_cycles(), 1u);
    EXPECT_EQ(diag.window_missing(1), 0u);
    EXPECT_EQ(diag.window_missing(2), 1u);
    EXPECT_EQ(diag.window_missing(3), 1u);
    EXPECT_EQ(diag.window_err(), 1u);
    EXPECT_EQ(diag.window_ok_short(), 1u);  // 3 sent, 1 OK + 1 ERR
}

TEST(TransportDiagnostics, ThrottlesToOneSummaryPerPeriod)
{
    auto diag = make_diag();
    auto frames = make_batch({1});

    uint64_t t = 1000;
    for (int i = 0; i < 1000; ++i, t += 4000) {  // 4 s at 250 Hz, every cycle failing
        diag.begin_cycle(frames.data(), frames.size());
        diag.end_cycle_at(true, t);
    }
    // immediate first report + one per elapsed second
    EXPECT_GE(diag.summaries_emitted(), 4u);
    EXPECT_LE(diag.summaries_emitted(), 5u);
}

TEST(TransportDiagnostics, QuietCyclesNeverReport)
{
    auto diag = make_diag();
    auto frames = make_batch({1, 2});

    uint64_t t = 1000;
    for (int i = 0; i < 1000; ++i, t += 4000) {
        diag.begin_cycle(frames.data(), frames.size());
        diag.on_line(Kind::Ok, "OK");
        diag.on_line(Kind::Ok, "OK");
        diag.on_line(Kind::Rcv, "rcv 0100 00", 1);
        diag.on_line(Kind::Rcv, "rcv 0200 00", 2);
        diag.end_cycle_at(false, t);
    }
    EXPECT_EQ(diag.summaries_emitted(), 0u);
}

TEST(TransportDiagnostics, LateRepliesAreCounted)
{
    auto diag = make_diag();
    auto frames = make_batch({1});

    diag.begin_cycle(frames.data(), frames.size());
    diag.on_line(Kind::Rcv, "rcv 0100 00", 1);
    diag.end_cycle_at(false, 1000);

    diag.on_late_line(Kind::Ok, "OK");  // trailing OK is normal
    EXPECT_EQ(diag.window_late(1), 0u);
    diag.on_late_line(Kind::Rcv, "rcv 0300 00", 3);
    EXPECT_EQ(diag.window_late(3), 1u);
}

// Sequence observed on hardware after a timeout: the previous batch's reply
// from id6 shows up right after the next write, before any OK of this batch.
TEST(TransportDiagnostics, ReplyBeforeItsOkIsStale)
{
    auto diag = make_diag();
    auto frames = make_batch({6, 1, 2, 3, 4, 5});
    diag.begin_cycle(frames.data(), frames.size());

    EXPECT_FALSE(diag.on_line(Kind::Rcv, "rcv 0600 00", 6));  // stale, 0 OKs so far
    for (int i = 0; i < 4; ++i) diag.on_line(Kind::Ok, "OK");
    EXPECT_TRUE(diag.on_line(Kind::Rcv, "rcv 0600 00", 6));   // fresh
    diag.on_line(Kind::Ok, "OK");
    EXPECT_TRUE(diag.on_line(Kind::Rcv, "rcv 0100 00", 1));
    EXPECT_EQ(diag.cycle_stale_dup(), 1u);
}

TEST(TransportDiagnostics, ReplyNeedsOkForItsBatchPosition)
{
    auto diag = make_diag();
    auto frames = make_batch({1, 2, 3});
    diag.begin_cycle(frames.data(), frames.size());

    diag.on_line(Kind::Ok, "OK");
    EXPECT_FALSE(diag.on_line(Kind::Rcv, "rcv 0300 00", 3));  // position 2 needs 3 OK/ERR
    diag.on_line(Kind::Ok, "OK");
    diag.on_line(Kind::Err, "ERR x");
    EXPECT_TRUE(diag.on_line(Kind::Rcv, "rcv 0300 00", 3));
}

TEST(TransportDiagnostics, CountsPartialLineAtTimeout)
{
    auto diag = make_diag();
    auto frames = make_batch({1});

    diag.begin_cycle(frames.data(), frames.size());
    diag.end_cycle_at(true, 1000, "rcv 0100 2701");
    EXPECT_EQ(diag.summaries_emitted(), 1u);  // reported and reset

    diag.begin_cycle(frames.data(), frames.size());
    diag.end_cycle_at(true, 2000, "rcv 01");
    diag.begin_cycle(frames.data(), frames.size());
    diag.end_cycle_at(true, 3000);
    EXPECT_EQ(diag.window_partial_at_timeout(), 1u);
}

TEST(TransportDiagnostics, NudgeReplyIsNotCountedAsError)
{
    auto diag = make_diag();
    auto frames = make_batch({1, 2});
    diag.begin_cycle(frames.data(), frames.size());

    diag.on_line(Kind::Ok, "OK");
    diag.on_line(Kind::Ok, "OK");
    EXPECT_TRUE(diag.on_line(Kind::Rcv, "rcv 0100 00", 1));
    diag.on_nudge_sent();
    EXPECT_FALSE(diag.on_line(Kind::Err, "ERR unknown command"));  // nudge answer
    EXPECT_TRUE(diag.on_line(Kind::Rcv, "rcv 0200 00", 2));        // released reply
    EXPECT_EQ(diag.cycle_err(), 0u);

    diag.end_cycle_at(false, 1000);
    EXPECT_EQ(diag.summaries_emitted(), 1u);  // nudged cycles are reported

    // a nudge answer arriving only in the next drain is consumed as well
    diag.begin_cycle(frames.data(), frames.size());
    diag.on_nudge_sent();
    diag.end_cycle_at(true, 2000);
    diag.on_late_line(Kind::Err, "ERR unknown command");
    EXPECT_EQ(diag.window_err(), 0u);
    EXPECT_EQ(diag.window_nudged(), 1u);
    EXPECT_EQ(diag.window_nudge_rescued(), 0u);
}
