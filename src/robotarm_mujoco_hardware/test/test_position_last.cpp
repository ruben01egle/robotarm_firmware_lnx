#include "robotarm_mujoco_hardware/RobotarmMujocoSystem.hpp"

#include <gtest/gtest.h>

using robotarm_mujoco_hardware::position_last;
using Interfaces = std::vector<std::string>;

TEST(PositionLast, MovesPositionBehindVelocityOfEveryJoint)
{
    // claim order of JointStreamController / CartesianJogController
    const Interfaces start = {"axis1/position", "axis1/velocity", "axis2/position", "axis2/velocity"};
    const Interfaces expected = {"axis1/velocity", "axis2/velocity", "axis1/position", "axis2/position"};
    EXPECT_EQ(position_last(start), expected);
}

TEST(PositionLast, KeepsOtherInterfacesInOrder)
{
    const Interfaces start = {"axis2/effort", "axis1/effort", "axis1/velocity"};
    EXPECT_EQ(position_last(start), start);
}

TEST(PositionLast, MatchesOnlyTheInterfaceName)
{
    // a joint name containing "position" is not a position interface
    const Interfaces start = {"position_axis/velocity", "axis1/position", "axis1/velocity"};
    const Interfaces expected = {"position_axis/velocity", "axis1/velocity", "axis1/position"};
    EXPECT_EQ(position_last(start), expected);
}

TEST(PositionLast, EmptyStaysEmpty)
{
    EXPECT_TRUE(position_last({}).empty());
}
