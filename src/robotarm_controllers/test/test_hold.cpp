// HoldController: realtime allocations and the hold of the activation pose.
//
// The controller is driven directly (no controller manager): own chained position/velocity/
// acceleration command and position state interfaces. Allocations are counted only inside update(),
// with the malloc interposer of malloc_counter.hpp. CounterDetectsAllocation is the negative control.

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface_params.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robotarm_controllers/HoldController.hpp"

#include "malloc_counter.hpp"
#include "test_helpers.hpp"

namespace
{

using hold_controller::HoldController;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using lifecycle_msgs::msg::State;

constexpr unsigned int update_rate = 1000;
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
const std::vector<std::string> joints = {"axis1", "axis2", "axis3", "axis4", "axis5", "axis6"};
const std::vector<double> pose = {0.3, -1.0, 1.2, 0.2, 0.9, -0.3};

class HoldTest : public ::testing::Test
{
protected:
    void TearDown() override
    {
        if (controller_ && controller_->get_lifecycle_state().id() == State::PRIMARY_STATE_ACTIVE) {
            controller_->get_node()->deactivate();
        }
        if (controller_) {
            controller_->release_interfaces();
        }
        controller_.reset();
    }

    // init and configure, returns the lifecycle state reached by configure
    uint8_t configure(const std::vector<rclcpp::Parameter> & params)
    {
        controller_ = std::make_shared<HoldController>();
        controller_interface::ControllerInterfaceParams p;
        p.controller_name = "hold_controller";
        p.robot_description = "";
        p.update_rate = update_rate;
        p.controller_manager_update_rate = update_rate;
        p.node_options = controller_->define_custom_node_options();
        p.node_options.parameter_overrides(params);
        EXPECT_EQ(controller_->init(p), controller_interface::return_type::OK);
        return controller_->configure().id();
    }

    std::vector<rclcpp::Parameter> default_params() const
    {
        return {rclcpp::Parameter("joints", joints), rclcpp::Parameter("chain_to", "ff_controller")};
    }

    // fresh interfaces, commands NaN like a downstream reference nobody wrote yet, measured position q
    void assign(const std::vector<double> & q)
    {
        cmd_.clear();
        state_.clear();
        std::vector<hardware_interface::LoanedCommandInterface> loaned_cmd;
        std::vector<hardware_interface::LoanedStateInterface> loaned_state;
        for (const auto & name : controller_->command_interface_configuration().names) {
            const auto [prefix, iface] = test_helpers::split_interface_name(name);
            cmd_.push_back(std::make_shared<CommandInterface>(prefix, iface, "double", "0.0"));
            (void)cmd_.back()->set_value(nan);
            loaned_cmd.emplace_back(cmd_.back(), nullptr);
        }
        for (const auto & name : controller_->state_interface_configuration().names) {
            const auto [prefix, iface] = test_helpers::split_interface_name(name);
            state_.push_back(std::make_shared<StateInterface>(prefix, iface, "double", "0.0"));
            loaned_state.emplace_back(state_.back(), nullptr);
        }
        controller_->assign_interfaces(std::move(loaned_cmd), std::move(loaned_state));
        set_state(q);
    }

    void set_state(const std::vector<double> & q)
    {
        for (size_t j = 0; j < state_.size(); ++j) {
            (void)state_[j]->set_value(q[j]);
        }
    }

    // command interface k of joint j: 0 position, 1 velocity, 2 acceleration
    double command(size_t j, size_t k) const {return cmd_[3 * j + k]->get_optional<double>().value();}

    bool update()
    {
        const auto ret = controller_->update(time_, period_);
        time_ += period_;
        return ret == controller_interface::return_type::OK;
    }

    std::shared_ptr<HoldController> controller_;
    std::vector<CommandInterface::SharedPtr> cmd_;
    std::vector<StateInterface::SharedPtr> state_;

    rclcpp::Time time_{0, 0, RCL_ROS_TIME};
    const rclcpp::Duration period_ = rclcpp::Duration::from_seconds(1.0 / update_rate);
};

TEST(MallocCounter, CounterDetectsAllocation)
{
    // negative control: must be seen, else the interposer is not active in this executable
    EXPECT_GT(malloc_counter::count_allocations([] {
            auto p = std::make_unique<double>(1.0);
            asm volatile("" : : "g"(p.get()) : "memory");
        }), 0u);
}

TEST_F(HoldTest, ConfigureRejectsMissingJointsOrChain)
{
    EXPECT_NE(configure({rclcpp::Parameter("chain_to", "ff_controller")}), State::PRIMARY_STATE_INACTIVE);
    EXPECT_NE(configure({rclcpp::Parameter("joints", joints)}), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(HoldTest, ClaimsChainedReferencesOfDownstream)
{
    ASSERT_EQ(configure(default_params()), State::PRIMARY_STATE_INACTIVE);
    const auto names = controller_->command_interface_configuration().names;
    ASSERT_EQ(names.size(), 3 * joints.size());
    EXPECT_EQ(names[0], "ff_controller/axis1/position");
    EXPECT_EQ(names[1], "ff_controller/axis1/velocity");
    EXPECT_EQ(names[2], "ff_controller/axis1/acceleration");
}

TEST_F(HoldTest, HoldsActivationPoseWithoutAllocating)
{
    ASSERT_EQ(configure(default_params()), State::PRIMARY_STATE_INACTIVE);
    assign(pose);
    ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);

    // the arm moves (pushed, sagging), the hold reference must not follow it
    std::vector<double> moved = pose;
    for (auto & q : moved) {q += 0.1;}
    set_state(moved);

    size_t allocations = 0;
    for (int c = 0; c < 100; ++c) {
        bool ok = false;
        allocations += malloc_counter::count_allocations([&] {ok = update();});
        ASSERT_TRUE(ok);
    }
    EXPECT_EQ(allocations, 0u);

    for (size_t j = 0; j < joints.size(); ++j) {
        EXPECT_DOUBLE_EQ(command(j, 0), pose[j]) << "joint " << j;
        EXPECT_DOUBLE_EQ(command(j, 1), 0.0) << "joint " << j;
        EXPECT_DOUBLE_EQ(command(j, 2), 0.0) << "joint " << j;
    }
}

TEST_F(HoldTest, ReactivationLatchesNewPose)
{
    ASSERT_EQ(configure(default_params()), State::PRIMARY_STATE_INACTIVE);
    assign(pose);
    ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(update());
    ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);

    std::vector<double> moved = pose;
    for (auto & q : moved) {q -= 0.2;}
    set_state(moved);
    ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(update());

    for (size_t j = 0; j < joints.size(); ++j) {
        EXPECT_DOUBLE_EQ(command(j, 0), moved[j]) << "joint " << j;
    }
}

TEST_F(HoldTest, RefusesActivationWithoutValidPosition)
{
    ASSERT_EQ(configure(default_params()), State::PRIMARY_STATE_INACTIVE);
    std::vector<double> q = pose;
    q[2] = nan;
    assign(q);
    EXPECT_NE(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
}

}  // namespace

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    test_helpers::LogAllocationExclusion::install();
    malloc_counter::prime_backtrace();
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
