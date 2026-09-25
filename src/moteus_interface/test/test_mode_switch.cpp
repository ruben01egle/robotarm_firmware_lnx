#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "moteus_interface/MoteusInterface.hpp"

// Exercises on_init, prepare/perform_command_mode_switch and the joint interface -> physical mux
// without hardware: the transport is only created in on_configure(), which is never called here.

namespace moteus_interface
{

namespace
{

using hardware_interface::return_type;
using Interfaces = std::vector<std::string>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double EPS = 1e-12;
constexpr double EFFORT_LIMIT = 2.0;

std::string joint_xml(const std::string& name)
{
    return
        "<joint name=\"" + name + "\">"
        "  <command_interface name=\"position\"/>"
        "  <command_interface name=\"velocity\"/>"
        "  <command_interface name=\"torque_ff\"/>"
        "  <command_interface name=\"effort\"/>"
        "  <state_interface name=\"position\"/>"
        "  <state_interface name=\"velocity\"/>"
        "  <state_interface name=\"effort\"/>"
        "  <param name=\"encoder_offset\">0.0</param>"
        "</joint>";
}

std::string urdf_joint_xml(const std::string& name, const std::string& parent, const std::string& child, double effort)
{
    return
        "<joint name=\"" + name + "\" type=\"revolute\">"
        "  <parent link=\"" + parent + "\"/><child link=\"" + child + "\"/>"
        "  <axis xyz=\"0 0 1\"/>"
        "  <limit lower=\"-3.0\" upper=\"3.0\" effort=\"" + std::to_string(effort) + "\" velocity=\"1.0\"/>"
        "</joint>";
}

// j1: identity transmission, j5/j6: differential transmission (like the wrist)
std::string make_urdf(double j1_effort = EFFORT_LIMIT)
{
    return
        "<?xml version=\"1.0\"?>"
        "<robot name=\"test_robot\">"
        "  <link name=\"base\"/><link name=\"l1\"/><link name=\"l5\"/><link name=\"l6\"/>"
        + urdf_joint_xml("j1", "base", "l1", j1_effort)
        + urdf_joint_xml("j5", "l1", "l5", EFFORT_LIMIT)
        + urdf_joint_xml("j6", "l5", "l6", EFFORT_LIMIT) +
        "  <ros2_control name=\"test_system\" type=\"system\">"
        "    <hardware><plugin>moteus_interface/MoteusInterface</plugin></hardware>"
        + joint_xml("j1") + joint_xml("j5") + joint_xml("j6") +
        "    <transmission name=\"t1\">"
        "      <plugin>identity</plugin>"
        "      <joint name=\"j1\" role=\"joint1\"/>"
        "      <actuator name=\"m1\" role=\"actuator1\"/>"
        "      <param name=\"m1.can_id\">1</param>"
        "    </transmission>"
        "    <transmission name=\"wrist\">"
        "      <plugin>differential</plugin>"
        "      <joint name=\"j5\" role=\"joint1\"/>"
        "      <joint name=\"j6\" role=\"joint2\"/>"
        "      <actuator name=\"wa\" role=\"actuator1\"/>"
        "      <actuator name=\"wb\" role=\"actuator2\"/>"
        "      <param name=\"wa.can_id\">5</param>"
        "      <param name=\"wb.can_id\">6</param>"
        "      <param name=\"joint1_encoder_actuator\">wa</param>"
        "      <param name=\"joint2_encoder_actuator\">wb</param>"
        "    </transmission>"
        "  </ros2_control>"
        "</robot>";
}

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

// friend of MoteusInterface: only the fixture's own member functions have access, so the tests
// below go through these helpers
class MoteusInterfaceTest : public testing::Test
{
protected:
    std::unique_ptr<MoteusInterface> hw;

    void SetUp() override
    {
        ASSERT_EQ(init(make_urdf()), hardware_interface::CallbackReturn::SUCCESS);
    }

    hardware_interface::CallbackReturn init(const std::string& urdf)
    {
        hw = std::make_unique<MoteusInterface>();
        hardware_interface::HardwareComponentInterfaceParams params;
        params.hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf).at(0);
        return hw->on_init(params);
    }

    return_type prepare(const Interfaces& start, const Interfaces& stop = {})
    {
        return hw->prepare_command_mode_switch(start, stop);
    }

    return_type perform(const Interfaces& start, const Interfaces& stop = {})
    {
        return hw->perform_command_mode_switch(start, stop);
    }

    // what the controller_manager does: prepare, and perform only if prepare succeeded
    return_type do_switch(const Interfaces& start, const Interfaces& stop = {})
    {
        const auto ret = prepare(start, stop);
        if (ret != return_type::OK) return ret;
        return perform(start, stop);
    }

    // one row of the spec's test table, on the identity joint j1, starting from a fresh interface
    return_type switch_from(const Interfaces& before, const Interfaces& start, const Interfaces& stop = {})
    {
        EXPECT_EQ(init(make_urdf()), hardware_interface::CallbackReturn::SUCCESS);
        EXPECT_EQ(do_switch(before), return_type::OK);
        return do_switch(start, stop);
    }

    size_t joint(const std::string& name) { return hw->joint_name_to_idx_.at(name); }
    size_t actuator(const std::string& name) { return hw->actuator_name_to_idx_.at(name); }

    CommandMode joint_mode(const std::string& name) { return hw->joints_[joint(name)].cmd_mode_; }
    ActiveInterfaces joint_interfaces(const std::string& name) { return hw->joints_[joint(name)].interfaces_; }
    CommandMode actuator_mode(const std::string& name) { return hw->actuators_[actuator(name)].cmd_mode_; }

    // exported buffers, written by controllers
    double& cmd_position(const std::string& name) { return hw->hw_commands_position_[joint(name)]; }
    double& cmd_velocity(const std::string& name) { return hw->hw_commands_velocity_[joint(name)]; }
    double& cmd_torque_ff(const std::string& name) { return hw->hw_commands_torque_ff_[joint(name)]; }
    double& cmd_effort(const std::string& name) { return hw->hw_commands_effort_[joint(name)]; }

    // joint physical layer
    double phys_position(const std::string& name) { return hw->joint_commands_position_[joint(name)]; }
    double phys_velocity(const std::string& name) { return hw->joint_commands_velocity_[joint(name)]; }
    double phys_torque(const std::string& name) { return hw->joint_commands_torque_[joint(name)]; }

    // actuator physical layer
    double act_position(const std::string& name) { return hw->actuator_commands_position_[actuator(name)]; }
    double act_velocity(const std::string& name) { return hw->actuator_commands_velocity_[actuator(name)]; }
    double act_torque(const std::string& name) { return hw->actuator_commands_torque_[actuator(name)]; }

    // the first two steps of make_cyclic_commands(), without building CAN frames
    void resolve_commands()
    {
        hw->joint_interface_to_joint_physical();
        for (const auto& t : hw->transmissions_) {
            t->joint_to_actuator();
        }
    }

    std::vector<std::string> exported_command_interface_names()
    {
        std::vector<std::string> names;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        for (const auto& ci : hw->export_command_interfaces()) {
            names.push_back(ci.get_name());
        }
#pragma GCC diagnostic pop
        return names;
    }
};


// ---------------------------------------------------------------------------------------------
// on_init / export
// ---------------------------------------------------------------------------------------------

TEST_F(MoteusInterfaceTest, StartsIdle)
{
    for (const auto& j : {"j1", "j5", "j6"}) {
        EXPECT_EQ(joint_mode(j), CommandMode::IDLE) << j;
        EXPECT_EQ(joint_interfaces(j), ActiveInterfaces{}) << j;
    }
    for (const auto& a : {"m1", "wa", "wb"}) {
        EXPECT_EQ(actuator_mode(a), CommandMode::IDLE) << a;
    }
}

TEST_F(MoteusInterfaceTest, ExportsFourCommandInterfacesPerJoint)
{
    const auto names = exported_command_interface_names();
    ASSERT_EQ(names.size(), 12u);
    for (const auto& j : {std::string("j1"), std::string("j5"), std::string("j6")}) {
        for (const auto& i : {"position", "velocity", "torque_ff", "effort"}) {
            EXPECT_NE(std::find(names.begin(), names.end(), j + "/" + i), names.end()) << j << "/" << i;
        }
    }
}

TEST_F(MoteusInterfaceTest, InitFailsWithoutPositiveEffortLimit)
{
    EXPECT_EQ(init(make_urdf(0.0)), hardware_interface::CallbackReturn::ERROR);
}


// ---------------------------------------------------------------------------------------------
// Validation rules (spec test table), identity joint
// ---------------------------------------------------------------------------------------------

TEST_F(MoteusInterfaceTest, SpecTable)
{
    EXPECT_EQ(switch_from({}, {"j1/effort"}), return_type::OK);
    EXPECT_EQ(switch_from({"j1/effort"}, {"j1/position"}), return_type::ERROR);
    EXPECT_EQ(switch_from({}, {"j1/position", "j1/velocity"}), return_type::OK);
    EXPECT_EQ(switch_from({"j1/position", "j1/velocity"}, {"j1/torque_ff"}), return_type::OK);
    EXPECT_EQ(switch_from({}, {"j1/torque_ff"}), return_type::OK);
    EXPECT_EQ(switch_from({"j1/position"}, {"j1/effort"}), return_type::ERROR);
    EXPECT_EQ(switch_from({"j1/torque_ff"}, {"j1/effort"}), return_type::ERROR);
    // switching between groups within one switch is fine, only the target set counts
    EXPECT_EQ(switch_from({"j1/effort"}, {"j1/position", "j1/velocity"}, {"j1/effort"}), return_type::OK);
    EXPECT_EQ(switch_from({"j1/position", "j1/torque_ff"}, {"j1/effort"}, {"j1/position", "j1/torque_ff"}), return_type::OK);
    // releasing everything -> idle
    EXPECT_EQ(switch_from({"j1/position", "j1/velocity"}, {}, {"j1/position", "j1/velocity"}), return_type::OK);
    // mixed in a single request
    EXPECT_EQ(switch_from({}, {"j1/velocity", "j1/effort"}), return_type::ERROR);
}

TEST_F(MoteusInterfaceTest, ModesFollowClaims)
{
    ASSERT_EQ(do_switch({"j1/torque_ff"}), return_type::OK);
    EXPECT_EQ(joint_mode("j1"), CommandMode::PVC);
    EXPECT_EQ(actuator_mode("m1"), CommandMode::PVC);

    ASSERT_EQ(do_switch({"j1/effort"}, {"j1/torque_ff"}), return_type::OK);
    EXPECT_EQ(joint_mode("j1"), CommandMode::TC);
    EXPECT_EQ(actuator_mode("m1"), CommandMode::TC);

    ASSERT_EQ(do_switch({}, {"j1/effort"}), return_type::OK);
    EXPECT_EQ(joint_mode("j1"), CommandMode::IDLE);
    EXPECT_EQ(actuator_mode("m1"), CommandMode::IDLE);
}

TEST_F(MoteusInterfaceTest, RejectedPrepareLeavesStateUnchanged)
{
    ASSERT_EQ(do_switch({"j1/position"}), return_type::OK);

    EXPECT_EQ(prepare({"j1/effort"}), return_type::ERROR);

    EXPECT_EQ(joint_interfaces("j1"), make(true, false, false, false));
    EXPECT_EQ(joint_mode("j1"), CommandMode::PVC);
    EXPECT_EQ(actuator_mode("m1"), CommandMode::PVC);
}

// another component may reject the switch after our prepare succeeded, then perform never comes
TEST_F(MoteusInterfaceTest, SuccessfulPrepareDoesNotCommit)
{
    EXPECT_EQ(prepare({"j1/position"}), return_type::OK);
    EXPECT_EQ(joint_interfaces("j1"), ActiveInterfaces{});
    EXPECT_EQ(joint_mode("j1"), CommandMode::IDLE);

    // evaluated against the committed (empty) state, not against the previous prepare
    EXPECT_EQ(prepare({"j1/effort"}), return_type::OK);
}

TEST_F(MoteusInterfaceTest, InterfacesOfOtherComponentsAreIgnored)
{
    EXPECT_EQ(do_switch({"other_joint/position", "some/nested/joint/effort"}), return_type::OK);
    for (const auto& j : {"j1", "j5", "j6"}) {
        EXPECT_EQ(joint_interfaces(j), ActiveInterfaces{}) << j;
    }

    // mixed with one of ours: ours is applied, the foreign one ignored
    EXPECT_EQ(do_switch({"other_joint/effort", "j1/position"}), return_type::OK);
    EXPECT_EQ(joint_interfaces("j1"), make(true, false, false, false));
}


// ---------------------------------------------------------------------------------------------
// Differential transmission: both joints must always claim identical sets
// ---------------------------------------------------------------------------------------------

TEST_F(MoteusInterfaceTest, DifferentialRejectsClaimOnOneJointOnly)
{
    EXPECT_EQ(do_switch({"j5/position"}), return_type::ERROR);
    EXPECT_EQ(do_switch({"j6/effort"}), return_type::ERROR);
    EXPECT_EQ(joint_interfaces("j5"), ActiveInterfaces{});
    EXPECT_EQ(joint_interfaces("j6"), ActiveInterfaces{});
}

TEST_F(MoteusInterfaceTest, DifferentialAcceptsIdenticalClaims)
{
    ASSERT_EQ(do_switch({"j5/position", "j5/velocity", "j6/position", "j6/velocity"}), return_type::OK);
    EXPECT_EQ(joint_mode("j5"), CommandMode::PVC);
    EXPECT_EQ(joint_mode("j6"), CommandMode::PVC);
    EXPECT_EQ(actuator_mode("wa"), CommandMode::PVC);
    EXPECT_EQ(actuator_mode("wb"), CommandMode::PVC);
}

TEST_F(MoteusInterfaceTest, DifferentialRejectsDifferingSetsInSameGroup)
{
    EXPECT_EQ(do_switch({"j5/position", "j5/velocity", "j6/position"}), return_type::ERROR);
}

TEST_F(MoteusInterfaceTest, DifferentialGravityCompensationNeedsBothJoints)
{
    ASSERT_EQ(do_switch({"j5/position", "j6/position"}), return_type::OK);

    EXPECT_EQ(do_switch({"j5/torque_ff"}), return_type::ERROR);
    EXPECT_EQ(do_switch({"j5/torque_ff", "j6/torque_ff"}), return_type::OK);
    EXPECT_EQ(joint_interfaces("j5"), make(true, false, true, false));
    EXPECT_EQ(joint_interfaces("j6"), make(true, false, true, false));
}

TEST_F(MoteusInterfaceTest, DifferentialRejectsReleasingOneJointOnly)
{
    ASSERT_EQ(do_switch({"j5/effort", "j6/effort"}), return_type::OK);

    EXPECT_EQ(do_switch({}, {"j6/effort"}), return_type::ERROR);
    EXPECT_EQ(actuator_mode("wa"), CommandMode::TC);
    EXPECT_EQ(actuator_mode("wb"), CommandMode::TC);

    EXPECT_EQ(do_switch({}, {"j5/effort", "j6/effort"}), return_type::OK);
    EXPECT_EQ(actuator_mode("wa"), CommandMode::IDLE);
    EXPECT_EQ(actuator_mode("wb"), CommandMode::IDLE);
}

TEST_F(MoteusInterfaceTest, DifferentialSwitchDoesNotTouchOtherTransmissions)
{
    ASSERT_EQ(do_switch({"j1/effort"}), return_type::OK);
    ASSERT_EQ(do_switch({"j5/position", "j6/position"}), return_type::OK);
    EXPECT_EQ(actuator_mode("m1"), CommandMode::TC);
}


// ---------------------------------------------------------------------------------------------
// perform: reset of newly claimed interfaces
// ---------------------------------------------------------------------------------------------

TEST_F(MoteusInterfaceTest, PerformResetsOnlyNewlyClaimedInterfaces)
{
    cmd_position("j1") = 1.0;
    cmd_velocity("j1") = 2.0;
    cmd_torque_ff("j1") = 3.0;
    cmd_effort("j1") = 4.0;

    ASSERT_EQ(do_switch({"j1/position", "j1/torque_ff"}), return_type::OK);

    EXPECT_TRUE(std::isnan(cmd_position("j1")));
    EXPECT_EQ(cmd_torque_ff("j1"), 0.0);
    EXPECT_EQ(cmd_velocity("j1"), 2.0);
    EXPECT_EQ(cmd_effort("j1"), 4.0);
}

TEST_F(MoteusInterfaceTest, PerformDoesNotResetStillClaimedInterfaces)
{
    ASSERT_EQ(do_switch({"j1/position"}), return_type::OK);
    cmd_position("j1") = 0.7;   // written by the running controller

    ASSERT_EQ(do_switch({"j1/torque_ff"}), return_type::OK);   // gravity comp joins

    EXPECT_EQ(cmd_position("j1"), 0.7);
}


// ---------------------------------------------------------------------------------------------
// Mux: joint interfaces -> joint physical -> actuator physical
// ---------------------------------------------------------------------------------------------

TEST_F(MoteusInterfaceTest, MuxIdleIgnoresAllBuffers)
{
    cmd_position("j1") = 1.0;
    cmd_velocity("j1") = 1.0;
    cmd_torque_ff("j1") = 1.0;
    cmd_effort("j1") = 1.0;

    resolve_commands();

    EXPECT_TRUE(std::isnan(phys_position("j1")));
    EXPECT_EQ(phys_velocity("j1"), 0.0);
    EXPECT_EQ(phys_torque("j1"), 0.0);
}

TEST_F(MoteusInterfaceTest, MuxPvcFillsInactiveMembers)
{
    ASSERT_EQ(do_switch({"j1/position", "j1/torque_ff"}), return_type::OK);
    cmd_position("j1") = 0.5;
    cmd_velocity("j1") = 9.0;   // not claimed
    cmd_torque_ff("j1") = 1.0;
    cmd_effort("j1") = 9.0;     // other group

    resolve_commands();

    EXPECT_EQ(phys_position("j1"), 0.5);
    EXPECT_EQ(phys_velocity("j1"), 0.0);
    EXPECT_EQ(phys_torque("j1"), 1.0);
    EXPECT_EQ(act_position("m1"), 0.5);
    EXPECT_EQ(act_velocity("m1"), 0.0);
    EXPECT_EQ(act_torque("m1"), 1.0);
}

TEST_F(MoteusInterfaceTest, MuxPvcWithoutPositionHolds)
{
    ASSERT_EQ(do_switch({"j1/velocity"}), return_type::OK);
    cmd_position("j1") = 0.5;   // not claimed
    cmd_velocity("j1") = 0.2;

    resolve_commands();

    EXPECT_TRUE(std::isnan(phys_position("j1")));
    EXPECT_EQ(phys_velocity("j1"), 0.2);
    EXPECT_EQ(phys_torque("j1"), 0.0);
}

TEST_F(MoteusInterfaceTest, MuxClampsTorqueFeedforwardToUrdfLimit)
{
    ASSERT_EQ(do_switch({"j1/torque_ff"}), return_type::OK);

    cmd_torque_ff("j1") = 5.0;
    resolve_commands();
    EXPECT_EQ(phys_torque("j1"), EFFORT_LIMIT);

    cmd_torque_ff("j1") = -5.0;
    resolve_commands();
    EXPECT_EQ(phys_torque("j1"), -EFFORT_LIMIT);

    cmd_torque_ff("j1") = 1.5;
    resolve_commands();
    EXPECT_EQ(phys_torque("j1"), 1.5);

    // kNaN is passed on and zeroed when the frame is built
    cmd_torque_ff("j1") = kNaN;
    resolve_commands();
    EXPECT_TRUE(std::isnan(phys_torque("j1")));
}

TEST_F(MoteusInterfaceTest, MuxTorqueControlUsesEffortOnly)
{
    ASSERT_EQ(do_switch({"j1/effort"}), return_type::OK);
    cmd_position("j1") = 0.3;
    cmd_velocity("j1") = 0.3;
    cmd_torque_ff("j1") = 0.3;
    cmd_effort("j1") = 1.5;

    resolve_commands();

    EXPECT_TRUE(std::isnan(phys_position("j1")));
    EXPECT_EQ(phys_velocity("j1"), 0.0);
    EXPECT_EQ(phys_torque("j1"), 1.5);
    EXPECT_EQ(act_torque("m1"), 1.5);
}

TEST_F(MoteusInterfaceTest, MuxThroughDifferential)
{
    ASSERT_EQ(do_switch({"j5/position", "j5/torque_ff", "j6/position", "j6/torque_ff"}), return_type::OK);
    cmd_position("j5") = 0.3;   cmd_position("j6") = 0.1;
    cmd_velocity("j5") = 9.0;   cmd_velocity("j6") = 9.0;   // not claimed
    cmd_torque_ff("j5") = 1.0;  cmd_torque_ff("j6") = 0.5;

    resolve_commands();

    EXPECT_NEAR(act_position("wa"), 0.4, EPS);
    EXPECT_NEAR(act_position("wb"), 0.2, EPS);
    EXPECT_EQ(act_velocity("wa"), 0.0);
    EXPECT_EQ(act_velocity("wb"), 0.0);
    EXPECT_NEAR(act_torque("wa"), 0.75, EPS);
    EXPECT_NEAR(act_torque("wb"), 0.25, EPS);
}

TEST_F(MoteusInterfaceTest, MuxThroughDifferentialWithoutPositionHoldsBothActuators)
{
    ASSERT_EQ(do_switch({"j5/torque_ff", "j6/torque_ff"}), return_type::OK);
    cmd_position("j5") = 0.3;   cmd_position("j6") = 0.1;   // not claimed

    resolve_commands();

    EXPECT_TRUE(std::isnan(act_position("wa")));
    EXPECT_TRUE(std::isnan(act_position("wb")));
}

}
