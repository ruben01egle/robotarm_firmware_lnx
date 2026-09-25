#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "moteus_interface/IdentityTransmission.hpp"
#include "moteus_interface/DifferentialTransmission.hpp"

using namespace moteus_interface;
using namespace moteus_interface::transmission;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double EPS = 1e-12;

// plain storage standing in for the vectors MoteusInterface owns
struct JointData
{
    double cmd_pos = 0.0, cmd_vel = 0.0, cmd_eff = 0.0;
    double st_pos = 0.0, st_vel = 0.0, st_eff = 0.0;
    CommandMode mode = CommandMode::IDLE;
    ActiveInterfaces interfaces;

    JointPort port()
    {
        return JointPort{Handle{&cmd_pos, &cmd_vel, &cmd_eff}, Handle{&st_pos, &st_vel, &st_eff}, &mode, &interfaces};
    }
};

struct ActuatorData
{
    double cmd_pos = 0.0, cmd_vel = 0.0, cmd_eff = 0.0;
    double st_pos = 0.0, st_vel = 0.0, st_eff = 0.0;
    CommandMode mode = CommandMode::IDLE;
    double home = 0.0;

    ActuatorPort port()
    {
        return ActuatorPort{Handle{&cmd_pos, &cmd_vel, &cmd_eff}, Handle{&st_pos, &st_vel, &st_eff}, &mode, &home};
    }

    // pretend the motor reached exactly what was commanded
    void loop_back()
    {
        st_pos = cmd_pos;
        st_vel = cmd_vel;
        st_eff = cmd_eff;
    }
};

ActiveInterfaces pv(bool position, bool velocity, bool torque_ff)
{
    ActiveInterfaces a;
    a.position = position;
    a.velocity = velocity;
    a.torque_ff = torque_ff;
    return a;
}

ActiveInterfaces effort_only()
{
    ActiveInterfaces a;
    a.effort = true;
    return a;
}

ActiveInterfaces mixed()
{
    ActiveInterfaces a;
    a.position = true;
    a.effort = true;
    return a;
}

}


// ---------------------------------------------------------------------------------------------
// DifferentialTransmission
// ---------------------------------------------------------------------------------------------

class DifferentialTransmissionTest : public testing::Test
{
protected:
    JointData j1, j2;
    ActuatorData a, b;

    DifferentialTransmission make(double j1_offset = 0.0, double j2_offset = 0.0, bool joint1_uses_actuator_a = true)
    {
        return DifferentialTransmission(j1.port(), j2.port(), a.port(), b.port(), j1_offset, j2_offset, joint1_uses_actuator_a);
    }
};

TEST_F(DifferentialTransmissionTest, JointToActuator)
{
    auto t = make();
    j1.cmd_pos = 0.3;  j2.cmd_pos = 0.1;
    j1.cmd_vel = 2.0;  j2.cmd_vel = -1.0;
    j1.cmd_eff = 2.0;  j2.cmd_eff = 1.0;

    t.joint_to_actuator();

    // position/velocity: a = j1 + j2, b = j1 - j2
    EXPECT_NEAR(a.cmd_pos, 0.4, EPS);
    EXPECT_NEAR(b.cmd_pos, 0.2, EPS);
    EXPECT_NEAR(a.cmd_vel, 1.0, EPS);
    EXPECT_NEAR(b.cmd_vel, 3.0, EPS);
    // effort: a = (j1 + j2) / 2, b = (j1 - j2) / 2
    EXPECT_NEAR(a.cmd_eff, 1.5, EPS);
    EXPECT_NEAR(b.cmd_eff, 0.5, EPS);
}

TEST_F(DifferentialTransmissionTest, ActuatorToJoint)
{
    auto t = make();
    a.st_pos = 0.4;  b.st_pos = 0.2;
    a.st_vel = 1.0;  b.st_vel = 3.0;
    a.st_eff = 1.5;  b.st_eff = 0.5;

    t.actuator_to_joint();

    // position/velocity: j1 = (a + b) / 2, j2 = (a - b) / 2
    EXPECT_NEAR(j1.st_pos, 0.3, EPS);
    EXPECT_NEAR(j2.st_pos, 0.1, EPS);
    EXPECT_NEAR(j1.st_vel, 2.0, EPS);
    EXPECT_NEAR(j2.st_vel, -1.0, EPS);
    // effort: j1 = a + b, j2 = a - b
    EXPECT_NEAR(j1.st_eff, 2.0, EPS);
    EXPECT_NEAR(j2.st_eff, 1.0, EPS);
}

// joint_to_actuator and actuator_to_joint must be exact inverses, otherwise the joint states
// reported to controllers would not match what they commanded
TEST_F(DifferentialTransmissionTest, RoundTrip)
{
    auto t = make();
    const double values[][2] = {{0.0, 0.0}, {1.25, -0.5}, {-2.0, 3.5}, {0.8, 0.8}, {-0.8, 0.8}};

    for (const auto& v : values) {
        j1.cmd_pos = v[0];        j2.cmd_pos = v[1];
        j1.cmd_vel = 2.0 * v[0];  j2.cmd_vel = 2.0 * v[1];
        j1.cmd_eff = -v[0];       j2.cmd_eff = -v[1];

        t.joint_to_actuator();
        a.loop_back();
        b.loop_back();
        t.actuator_to_joint();

        EXPECT_NEAR(j1.st_pos, j1.cmd_pos, EPS);
        EXPECT_NEAR(j2.st_pos, j2.cmd_pos, EPS);
        EXPECT_NEAR(j1.st_vel, j1.cmd_vel, EPS);
        EXPECT_NEAR(j2.st_vel, j2.cmd_vel, EPS);
        EXPECT_NEAR(j1.st_eff, j1.cmd_eff, EPS);
        EXPECT_NEAR(j2.st_eff, j2.cmd_eff, EPS);
    }
}

// the mux fills inactive interfaces with kNaN (position) / 0 (velocity, torque) in joint space and
// relies on the transmission carrying these through unchanged -- this is what makes that valid
TEST_F(DifferentialTransmissionTest, InactiveDefaultsPropagate)
{
    auto t = make();
    j1.cmd_pos = kNaN;  j2.cmd_pos = kNaN;
    j1.cmd_vel = 0.0;  j2.cmd_vel = 0.0;
    j1.cmd_eff = 0.0;  j2.cmd_eff = 0.0;

    t.joint_to_actuator();

    EXPECT_TRUE(std::isnan(a.cmd_pos));
    EXPECT_TRUE(std::isnan(b.cmd_pos));
    EXPECT_EQ(a.cmd_vel, 0.0);
    EXPECT_EQ(b.cmd_vel, 0.0);
    EXPECT_EQ(a.cmd_eff, 0.0);
    EXPECT_EQ(b.cmd_eff, 0.0);
}

// documents the fail-safe direction: a single kNaN position poisons both actuators (-> hold),
// it never produces a finite position target from half the information
TEST_F(DifferentialTransmissionTest, SingleNaNPositionPoisonsBothActuators)
{
    auto t = make();
    j1.cmd_pos = 0.5;
    j2.cmd_pos = kNaN;

    t.joint_to_actuator();

    EXPECT_TRUE(std::isnan(a.cmd_pos));
    EXPECT_TRUE(std::isnan(b.cmd_pos));
}

TEST_F(DifferentialTransmissionTest, ValidateAcceptsIdenticalValidSets)
{
    auto t = make();

    EXPECT_TRUE(t.validate_mode_switch());                  // both empty

    j1.interfaces = pv(true, true, false);
    j2.interfaces = pv(true, true, false);
    EXPECT_TRUE(t.validate_mode_switch());

    j1.interfaces = pv(false, false, true);
    j2.interfaces = pv(false, false, true);
    EXPECT_TRUE(t.validate_mode_switch());

    j1.interfaces = effort_only();
    j2.interfaces = effort_only();
    EXPECT_TRUE(t.validate_mode_switch());
}

TEST_F(DifferentialTransmissionTest, ValidateRejectsDifferentSets)
{
    auto t = make();

    // one joint claimed, the other not
    j1.interfaces = pv(true, false, false);
    j2.interfaces = ActiveInterfaces{};
    EXPECT_FALSE(t.validate_mode_switch());

    // same group, different members
    j1.interfaces = pv(true, true, false);
    j2.interfaces = pv(true, false, false);
    EXPECT_FALSE(t.validate_mode_switch());

    // gravity compensation on only one joint of the pair
    j1.interfaces = pv(true, true, true);
    j2.interfaces = pv(true, true, false);
    EXPECT_FALSE(t.validate_mode_switch());

    // different groups
    j1.interfaces = pv(true, false, false);
    j2.interfaces = effort_only();
    EXPECT_FALSE(t.validate_mode_switch());
}

TEST_F(DifferentialTransmissionTest, ValidateRejectsIdenticalButMixedSets)
{
    auto t = make();
    j1.interfaces = mixed();
    j2.interfaces = mixed();
    EXPECT_FALSE(t.validate_mode_switch());
}

TEST_F(DifferentialTransmissionTest, ValidateDoesNotModifyAnything)
{
    auto t = make();
    j1.interfaces = pv(true, false, false);
    j2.interfaces = effort_only();
    a.mode = CommandMode::PVC;
    b.mode = CommandMode::PVC;

    (void)t.validate_mode_switch();

    EXPECT_EQ(j1.interfaces, pv(true, false, false));
    EXPECT_EQ(j2.interfaces, effort_only());
    EXPECT_EQ(a.mode, CommandMode::PVC);
    EXPECT_EQ(b.mode, CommandMode::PVC);
}

TEST_F(DifferentialTransmissionTest, PerformPropagatesModeToBothActuators)
{
    auto t = make();

    j1.mode = CommandMode::TC;
    j2.mode = CommandMode::TC;
    t.perform_mode_switch();
    EXPECT_EQ(a.mode, CommandMode::TC);
    EXPECT_EQ(b.mode, CommandMode::TC);

    j1.mode = CommandMode::IDLE;
    j2.mode = CommandMode::IDLE;
    t.perform_mode_switch();
    EXPECT_EQ(a.mode, CommandMode::IDLE);
    EXPECT_EQ(b.mode, CommandMode::IDLE);
}

TEST_F(DifferentialTransmissionTest, HomeWithoutOffsets)
{
    auto t = make();
    a.home = 0.1;   // raw encoder of joint1 (joint1_uses_actuator_a = true)
    b.home = 0.2;   // raw encoder of joint2

    t.home();

    EXPECT_NEAR(a.home, 0.3, EPS);    // j1 + j2
    EXPECT_NEAR(b.home, -0.1, EPS);   // j1 - j2
}

TEST_F(DifferentialTransmissionTest, HomeWithSwappedEncoders)
{
    auto t = make(0.0, 0.0, false);
    a.home = 0.1;   // now the raw encoder of joint2
    b.home = 0.2;   // now the raw encoder of joint1

    t.home();

    EXPECT_NEAR(a.home, 0.3, EPS);    // j1 (0.2) + j2 (0.1)
    EXPECT_NEAR(b.home, 0.1, EPS);    // j1 (0.2) - j2 (0.1)
}

TEST_F(DifferentialTransmissionTest, HomeAppliesOffsetsInRadians)
{
    // offsets are given in rad, raw readings and results in revolutions
    auto t = make(2.0 * M_PI * 0.05, 2.0 * M_PI * -0.1);
    a.home = 0.1;
    b.home = 0.2;

    t.home();

    // j1 = 0.1 - 0.05 = 0.05, j2 = 0.2 + 0.1 = 0.3
    EXPECT_NEAR(a.home, 0.35, EPS);
    EXPECT_NEAR(b.home, -0.25, EPS);
}

TEST_F(DifferentialTransmissionTest, HomeWrapsEachJointToHalfRevolution)
{
    auto t = make();
    a.home = 0.9;   // joint1 -> -0.1
    b.home = 0.2;

    t.home();

    EXPECT_NEAR(a.home, 0.1, EPS);    // -0.1 + 0.2
    EXPECT_NEAR(b.home, -0.3, EPS);   // -0.1 - 0.2
}


// ---------------------------------------------------------------------------------------------
// IdentityTransmission
// ---------------------------------------------------------------------------------------------

class IdentityTransmissionTest : public testing::Test
{
protected:
    JointData j;
    ActuatorData act;

    IdentityTransmission make(double offset = 0.0)
    {
        return IdentityTransmission(j.port(), act.port(), offset);
    }
};

TEST_F(IdentityTransmissionTest, CopiesCommandsAndStates)
{
    auto t = make();
    j.cmd_pos = 0.7;  j.cmd_vel = -1.2;  j.cmd_eff = 0.4;

    t.joint_to_actuator();
    EXPECT_EQ(act.cmd_pos, 0.7);
    EXPECT_EQ(act.cmd_vel, -1.2);
    EXPECT_EQ(act.cmd_eff, 0.4);

    act.st_pos = 0.1;  act.st_vel = 0.2;  act.st_eff = 0.3;
    t.actuator_to_joint();
    EXPECT_EQ(j.st_pos, 0.1);
    EXPECT_EQ(j.st_vel, 0.2);
    EXPECT_EQ(j.st_eff, 0.3);
}

TEST_F(IdentityTransmissionTest, InactiveDefaultsPropagate)
{
    auto t = make();
    j.cmd_pos = kNaN;  j.cmd_vel = 0.0;  j.cmd_eff = 0.0;

    t.joint_to_actuator();

    EXPECT_TRUE(std::isnan(act.cmd_pos));
    EXPECT_EQ(act.cmd_vel, 0.0);
    EXPECT_EQ(act.cmd_eff, 0.0);
}

TEST_F(IdentityTransmissionTest, ValidateChecksGroupsOnly)
{
    auto t = make();
    EXPECT_TRUE(t.validate_mode_switch());

    j.interfaces = pv(true, false, true);
    EXPECT_TRUE(t.validate_mode_switch());

    j.interfaces = effort_only();
    EXPECT_TRUE(t.validate_mode_switch());

    j.interfaces = mixed();
    EXPECT_FALSE(t.validate_mode_switch());
}

TEST_F(IdentityTransmissionTest, PerformPropagatesMode)
{
    auto t = make();
    j.mode = CommandMode::PVC;
    t.perform_mode_switch();
    EXPECT_EQ(act.mode, CommandMode::PVC);
}

TEST_F(IdentityTransmissionTest, HomeAppliesOffsetAndWraps)
{
    auto t = make(2.0 * M_PI * 0.25);
    act.home = 0.1;

    t.home();

    EXPECT_NEAR(act.home, -0.15, EPS);

    act.home = 0.9;
    auto t0 = make();
    t0.home();
    EXPECT_NEAR(act.home, -0.1, EPS);
}
