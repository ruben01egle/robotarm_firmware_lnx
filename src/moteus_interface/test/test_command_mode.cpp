#include <gtest/gtest.h>

#include "moteus_interface/CommandMode.hpp"

using moteus_interface::ActiveInterfaces;
using moteus_interface::CommandMode;
using moteus_interface::mode_of_active_interfaces;

namespace
{

ActiveInterfaces make(bool position, bool velocity, bool torque_ff, bool effort)
{
    ActiveInterfaces a;
    a.position = position;
    a.velocity = velocity;
    a.torque_ff = torque_ff;
    a.effort = effort;
    return a;
}

}

TEST(CommandMode, EmptySetIsIdle)
{
    CommandMode mode = CommandMode::TC;
    ASSERT_TRUE(mode_of_active_interfaces(ActiveInterfaces{}, mode));
    EXPECT_EQ(mode, CommandMode::IDLE);
}

TEST(CommandMode, EffortAloneIsTorqueControl)
{
    CommandMode mode = CommandMode::IDLE;
    ASSERT_TRUE(mode_of_active_interfaces(make(false, false, false, true), mode));
    EXPECT_EQ(mode, CommandMode::TC);
}

// all 16 combinations: any non-empty subset of {position, velocity, torque_ff} is PVC,
// anything that combines one of them with effort is invalid
TEST(CommandMode, AllCombinations)
{
    for (int bits = 0; bits < 16; ++bits) {
        const bool pos = bits & 1, vel = bits & 2, ff = bits & 4, eff = bits & 8;
        const ActiveInterfaces a = make(pos, vel, ff, eff);
        const bool any_pv = pos || vel || ff;

        SCOPED_TRACE(testing::Message() << "pos=" << pos << " vel=" << vel << " ff=" << ff << " eff=" << eff);

        EXPECT_EQ(a.valid(), !(any_pv && eff));

        CommandMode mode = CommandMode::IDLE;
        const bool ok = mode_of_active_interfaces(a, mode);
        if (any_pv && eff) {
            EXPECT_FALSE(ok);
        } else if (any_pv) {
            ASSERT_TRUE(ok);
            EXPECT_EQ(mode, CommandMode::PVC);
        } else if (eff) {
            ASSERT_TRUE(ok);
            EXPECT_EQ(mode, CommandMode::TC);
        } else {
            ASSERT_TRUE(ok);
            EXPECT_EQ(mode, CommandMode::IDLE);
        }
    }
}

TEST(CommandMode, InvalidSetLeavesModeUntouched)
{
    CommandMode mode = CommandMode::PVC;
    EXPECT_FALSE(mode_of_active_interfaces(make(true, false, false, true), mode));
    EXPECT_EQ(mode, CommandMode::PVC);
}

TEST(CommandMode, Equality)
{
    EXPECT_EQ(make(true, true, false, false), make(true, true, false, false));
    EXPECT_NE(make(true, true, false, false), make(true, false, false, false));
    EXPECT_NE(make(false, false, true, false), make(false, false, false, true));
}
