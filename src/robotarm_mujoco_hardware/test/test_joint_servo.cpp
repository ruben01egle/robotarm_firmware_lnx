#include "robotarm_mujoco_hardware/JointServo.hpp"
#include "robotarm_mujoco_hardware/RobotarmMujocoSystem.hpp"
#include "robotarm_mujoco_hardware/ServoBridge.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>

using robotarm_mujoco_hardware::JointServo;
using robotarm_mujoco_hardware::ServoGains;

namespace {

ServoGains gains(double kp, double ki, double kd, double limit = 100.0)
{
    ServoGains g;
    g.kp = kp;
    g.ki = ki;
    g.kd = kd;
    g.effort_limit = limit;
    return g;
}

}  // namespace

TEST(JointServo, ProportionalAndDampingOnTheVelocityError)
{
    JointServo servo(gains(10.0, 0.0, 2.0));
    // 0.1 rad behind, 0.5 rad/s slower than commanded: 10·0.1 + 2·0.5
    EXPECT_DOUBLE_EQ(servo.update(1.1, 1.0, 1.0, 0.5, 0.002), 2.0);
    // at the target, moving with the commanded velocity: no torque
    EXPECT_DOUBLE_EQ(servo.update(1.0, 1.0, 1.0, 1.0, 0.002), 0.0);
}

TEST(JointServo, IntegralBuildsUpAndResets)
{
    JointServo servo(gains(0.0, 100.0, 0.0));
    EXPECT_DOUBLE_EQ(servo.update(0.1, 0.0, 0.0, 0.0, 0.01), 0.0);   // integral applies from the next cycle
    EXPECT_DOUBLE_EQ(servo.integral(), 0.1);
    EXPECT_DOUBLE_EQ(servo.update(0.1, 0.0, 0.0, 0.0, 0.01), 0.1);
    servo.reset();
    EXPECT_DOUBLE_EQ(servo.integral(), 0.0);
}

TEST(JointServo, ZeroPeriodDoesNotIntegrate)
{
    JointServo servo(gains(1.0, 100.0, 0.0));
    servo.update(0.1, 0.0, 0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(servo.integral(), 0.0);
}

TEST(JointServo, OutputIsClampedAndTheIntegralStopsWhileSaturated)
{
    JointServo servo(gains(1000.0, 1000.0, 0.0, 5.0));
    EXPECT_DOUBLE_EQ(servo.update(1.0, 0.0, 0.0, 0.0, 0.01), 5.0);
    EXPECT_DOUBLE_EQ(servo.update(-1.0, 0.0, 0.0, 0.0, 0.01), -5.0);
    EXPECT_DOUBLE_EQ(servo.integral(), 0.0);
}

TEST(JointServo, IntegralIsClampedToTheEffortLimit)
{
    JointServo servo(gains(0.0, 1000.0, 0.0, 5.0));
    for (int i = 0; i < 100; ++i) {
        servo.update(1.0, 0.0, 0.0, 0.0, 0.01);
    }
    EXPECT_DOUBLE_EQ(servo.integral(), 5.0);
}

class ServoGainsFile : public ::testing::Test
{
protected:
    std::string write(const std::string& content)
    {
        std::ofstream(path_) << content;
        return path_;
    }
    void TearDown() override { std::remove(path_.c_str()); }

    const std::string path_ = ::testing::TempDir() + "servo_gains_test.yaml";
};

TEST_F(ServoGainsFile, ReadsEveryJoint)
{
    const auto g = robotarm_mujoco_hardware::load_servo_gains(
        write("a: {kp: 1.0, ki: 2.0, kd: 3.0}\nb: {kp: 4, ki: 5, kd: 6}\n"), {"a", "b"});
    EXPECT_DOUBLE_EQ(g.at("a").kp, 1.0);
    EXPECT_DOUBLE_EQ(g.at("a").kd, 3.0);
    EXPECT_DOUBLE_EQ(g.at("b").ki, 5.0);
}

TEST_F(ServoGainsFile, RejectsMissingJointMissingGainAndNegativeGain)
{
    using robotarm_mujoco_hardware::load_servo_gains;
    EXPECT_THROW(load_servo_gains(write("a: {kp: 1, ki: 1, kd: 1}\n"), {"a", "b"}), std::runtime_error);
    EXPECT_THROW(load_servo_gains(write("a: {kp: 1, ki: 1}\n"), {"a"}), std::runtime_error);
    EXPECT_THROW(load_servo_gains(write("a: {kp: -1, ki: 1, kd: 1}\n"), {"a"}), std::runtime_error);
    EXPECT_THROW(load_servo_gains(path_ + ".missing", {"a"}), std::runtime_error);
}

TEST(ServoBridge, HandsTargetsAndResetsBetweenTheSides)
{
    using robotarm_mujoco_hardware::ServoBridge;
    using robotarm_mujoco_hardware::ServoTarget;
    auto& bridge = ServoBridge::instance();

    std::vector<std::string> joints;
    std::vector<ServoGains> g;
    std::vector<ServoTarget> targets;
    uint64_t version = 0;

    bridge.attach();
    EXPECT_TRUE(bridge.attached());
    bridge.configure({"a", "b"}, {gains(1, 0, 0), gains(2, 0, 0)});
    ASSERT_TRUE(bridge.read(joints, g, targets, version));
    ASSERT_EQ(joints.size(), 2u);
    EXPECT_DOUBLE_EQ(g[1].kp, 2.0);
    EXPECT_FALSE(targets[0].active);

    std::vector<ServoTarget> posted(2);
    posted[1].active = true;
    posted[1].position = 0.5;
    posted[1].velocity = 1.0;
    posted[1].time = 3.0;
    bridge.set_targets(posted);
    ASSERT_TRUE(bridge.read(joints, g, targets, version));
    EXPECT_TRUE(targets[1].active);
    EXPECT_DOUBLE_EQ(targets[1].position, 0.5);
    EXPECT_DOUBLE_EQ(targets[1].time, 3.0);

    // a wrong size is ignored instead of breaking the plugin's buffers
    bridge.set_targets(std::vector<ServoTarget>(3));
    ASSERT_TRUE(bridge.read(joints, g, targets, version));
    EXPECT_TRUE(targets[1].active);

    EXPECT_FALSE(bridge.take_reset());
    bridge.notify_reset();
    EXPECT_TRUE(bridge.take_reset());
    EXPECT_FALSE(bridge.take_reset());

    bridge.detach();
    EXPECT_FALSE(bridge.attached());
}
