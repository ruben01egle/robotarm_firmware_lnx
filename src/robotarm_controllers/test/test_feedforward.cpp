// FeedforwardController: realtime allocations, the inverse dynamics feedforward and its safety layers.
//
// The controller is driven directly (no controller manager): own position/velocity/torque_ff command
// and position state interfaces, and the exported reference interfaces are written like an upstream
// controller would. There is no physics, every cycle is a static evaluation, checked against an own
// RobotarmRbd from the same URDF: τ_ff = RNEA(q_ref, dq_ref, ddq_ref) incl. gravity, to 1e-9.
//
// Allocations are counted only inside update(), with the malloc interposer of malloc_counter.hpp.
// Log output is excluded (test_helpers::LogAllocationExclusion), the throttled warnings in update()
// are accepted. CounterDetectsAllocation is the negative control of this executable.
//
// The start ramp blends from 0 over 1 s, so most tests run settle() first. torque_rise_time is 0
// unless a test sets it.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "controller_interface/controller_interface_params.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robotarm_controllers/FeedforwardController.hpp"
#include "robotarm_rbd/RobotarmRbd.hpp"

#include "malloc_counter.hpp"
#include "test_helpers.hpp"

namespace
{

using feedforward_controller::FeedforwardController;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using lifecycle_msgs::msg::State;
using test_helpers::real_urdf;

constexpr unsigned int update_rate = 1000;
constexpr char controller_name[] = "ff_controller";
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
constexpr double inf = std::numeric_limits<double>::infinity();
constexpr double exact = 1e-9;       // Nm, torques that are fixed by design
constexpr int settle_cycles = 1100;  // > 1 s start ramp at 1 kHz

struct StepResult
{
    size_t allocations = 0;     // summed over all cycles
    int first_cycle = -1;       // first cycle that allocated, -1 if none
    bool ok = true;             // all updates returned OK
};

#define EXPECT_NO_RT_ALLOCATIONS(r) \
    EXPECT_EQ((r).allocations, 0u) << "update() allocated, first in cycle " << (r).first_cycle \
                                   << " (backtrace on stderr)"

#define EXPECT_VEC_NEAR(a, b, tol) \
    EXPECT_LT(((a) - (b)).cwiseAbs().maxCoeff(), (tol)) \
        << "\n  actual:   " << (a).transpose() << "\n  expected: " << (b).transpose()

class FeedforwardTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ASSERT_TRUE(rbd_.initialize(real_urdf(), robotarm_rbd::RobotarmRbd::Config{})) << rbd_.last_error();
        ASSERT_TRUE(rbd_.get_joint_limits(limits_));
        ASSERT_TRUE(rbd_.get_joint_names(names_));
        dof_ = rbd_.get_dof();
    }

    void TearDown() override
    {
        if (controller_ && controller_->get_lifecycle_state().id() == State::PRIMARY_STATE_ACTIVE) {
            controller_->get_node()->deactivate();
        }
        if (controller_) {
            controller_->release_interfaces();
        }
        refs_.clear();
        controller_.reset();
    }

    // init and configure, returns the lifecycle state reached by configure
    uint8_t configure(const std::vector<rclcpp::Parameter> & params = {})
    {
        controller_ = std::make_shared<FeedforwardController>();
        controller_interface::ControllerInterfaceParams p;
        p.controller_name = controller_name;
        p.robot_description = real_urdf();
        p.update_rate = update_rate;
        p.controller_manager_update_rate = update_rate;
        p.node_options = controller_->define_custom_node_options();
        p.node_options.parameter_overrides(params);
        EXPECT_EQ(controller_->init(p), controller_interface::return_type::OK);
        return controller_->configure().id();
    }

    // configure, export the reference interfaces, assign the hardware interfaces at q0, activate
    void start(const Eigen::VectorXd & q0, const std::vector<rclcpp::Parameter> & params = {})
    {
        ASSERT_EQ(configure(params), State::PRIMARY_STATE_INACTIVE);
        refs_ = controller_->export_reference_interfaces();
        ASSERT_EQ(refs_.size(), 3 * dof_);
        assign(q0);
        ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
    }

    // fresh hardware interfaces, commands 0, measured position q and torque 0
    void assign(const Eigen::VectorXd & q)
    {
        cmd_.clear();
        state_.clear();
        const auto cmd_names = controller_->command_interface_configuration().names;
        const auto state_names = controller_->state_interface_configuration().names;
        ASSERT_EQ(cmd_names.size(), 3 * dof_);
        ASSERT_EQ(state_names.size(), 2 * dof_);

        std::vector<hardware_interface::LoanedCommandInterface> loaned_cmd;
        std::vector<hardware_interface::LoanedStateInterface> loaned_state;
        for (const auto & name : cmd_names) {
            const auto [prefix, iface] = test_helpers::split_interface_name(name);
            cmd_.push_back(std::make_shared<CommandInterface>(prefix, iface, "double", "0.0"));
            loaned_cmd.emplace_back(cmd_.back(), nullptr);
        }
        for (const auto & name : state_names) {
            const auto [prefix, iface] = test_helpers::split_interface_name(name);
            state_.push_back(std::make_shared<StateInterface>(prefix, iface, "double", "0.0"));
            loaned_state.emplace_back(state_.back(), nullptr);
        }
        controller_->assign_interfaces(std::move(loaned_cmd), std::move(loaned_state));
        set_state(q);
    }

    void set_state(const Eigen::VectorXd & q)
    {
        for (size_t j = 0; j < dof_; ++j) {
            (void)state_[2 * j]->set_value(q[j]);
        }
    }

    void set_effort(const Eigen::VectorXd & tau_measured)
    {
        for (size_t j = 0; j < dof_; ++j) {
            (void)state_[2 * j + 1]->set_value(tau_measured[j]);
        }
    }

    // deactivate and activate again at q with measured torque tau_measured, ramp_start set before
    void reactivate(const Eigen::VectorXd & q, const Eigen::VectorXd & tau_measured,
                    const std::string & ramp_start)
    {
        ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
        controller_->release_interfaces();
        assign(q);
        set_effort(tau_measured);
        ASSERT_TRUE(controller_->get_node()->set_parameter(
            rclcpp::Parameter("ramp_start", ramp_start)).successful);
        ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
    }

    // the measured torque of a gravity compensating effort mode, off the model so it can't be mistaken
    Eigen::VectorXd effort_mode_torque(const Eigen::VectorXd & q)
    {
        return gravity(q) + Eigen::VectorXd::Constant(dof_, 0.2);
    }

    // writes the references like an upstream controller, an empty vector leaves NaN
    void set_reference(const Eigen::VectorXd & q_d, const Eigen::VectorXd & dq_d = {},
                       const Eigen::VectorXd & ddq_d = {})
    {
        for (size_t j = 0; j < dof_; ++j) {
            (void)refs_[3 * j + 0]->set_value(q_d.size() ? q_d[j] : nan);
            (void)refs_[3 * j + 1]->set_value(dq_d.size() ? dq_d[j] : nan);
            (void)refs_[3 * j + 2]->set_value(ddq_d.size() ? ddq_d[j] : nan);
        }
    }

    // runs n control cycles, counting allocations only inside update()
    StepResult step(int n)
    {
        StepResult r;
        for (int c = 0; c < n; ++c) {
            controller_interface::return_type ret = controller_interface::return_type::OK;
            const size_t a = malloc_counter::count_allocations([&] {ret = controller_->update(time_, period_);});
            if (a > 0 && r.first_cycle < 0) {
                r.first_cycle = c;
                malloc_counter::print_first_allocation_backtrace();
            }
            r.allocations += a;
            r.ok = r.ok && ret == controller_interface::return_type::OK;
            time_ += period_;
            if (!r.ok) {
                break;
            }
        }
        return r;
    }

    void settle() {ASSERT_TRUE(step(settle_cycles).ok);}

    // command interface k of every joint: 0 position, 1 velocity, 2 torque_ff
    Eigen::VectorXd command(size_t k) const
    {
        Eigen::VectorXd v(dof_);
        for (size_t j = 0; j < dof_; ++j) {
            v[j] = cmd_[3 * j + k]->get_optional<double>().value();
        }
        return v;
    }
    Eigen::VectorXd cmd_q() const {return command(0);}
    Eigen::VectorXd cmd_dq() const {return command(1);}
    Eigen::VectorXd tau() const {return command(2);}

    // own model, outside of any measured window: inverse dynamics incl. gravity
    Eigen::VectorXd ff(const Eigen::VectorXd & q, const Eigen::VectorXd & dq, const Eigen::VectorXd & ddq)
    {
        Eigen::VectorXd t(dof_);
        EXPECT_TRUE(rbd_.recursive_newton_euler(q, dq, ddq, t)) << rbd_.last_error();
        return t;
    }

    Eigen::VectorXd gravity(const Eigen::VectorXd & q)
    {
        return ff(q, zero(), zero());
    }

    Eigen::VectorXd zero() const {return Eigen::VectorXd::Zero(dof_);}

    double effort(size_t j) const {return limits_[j].effort;}

    // a regular pose away from singularities and joint limits (same as the impedance test)
    Eigen::VectorXd regular_pose() const
    {
        Eigen::VectorXd q(6);
        q << 0.3, -1.0, 1.2, 0.2, 0.9, -0.3;
        return q;
    }

    // a reference that moves and accelerates every joint, small enough to stay below the effort limits
    void moving_reference(Eigen::VectorXd & q, Eigen::VectorXd & dq, Eigen::VectorXd & ddq) const
    {
        q = regular_pose() + Eigen::VectorXd::Constant(dof_, 0.1);
        dq.resize(6);
        dq << 0.3, -0.2, 0.4, -0.5, 0.3, 0.6;
        ddq.resize(6);
        ddq << 1.0, -0.5, 0.8, 2.0, -1.5, 1.0;
    }

    std::shared_ptr<FeedforwardController> controller_;
    std::vector<CommandInterface::SharedPtr> cmd_;
    std::vector<StateInterface::SharedPtr> state_;
    std::vector<CommandInterface::SharedPtr> refs_;

    robotarm_rbd::RobotarmRbd rbd_;
    std::vector<robotarm_rbd::RobotarmRbd::Limits> limits_;
    std::vector<std::string> names_;
    size_t dof_ = 0;

    rclcpp::Time time_{0, 0, RCL_ROS_TIME};
    const rclcpp::Duration period_ = rclcpp::Duration::from_seconds(1.0 / update_rate);
};

// ---------------------------------------------------------------------------------------------------
// allocation counter

TEST(MallocCounter, CounterDetectsAllocation)
{
    // negative control: these must be seen, else the interposer is not active in this executable
    EXPECT_GT(malloc_counter::count_allocations([] {
            Eigen::VectorXd v(6);
            v.setZero();
        }), 0u);
    EXPECT_EQ(malloc_counter::count_allocations([] {
            malloc_counter::ScopedPause pause;
            auto p = std::make_unique<double>(1.0);
            asm volatile("" : : "g"(p.get()) : "memory");
        }), 0u);
}

// ---------------------------------------------------------------------------------------------------
// configuration

TEST_F(FeedforwardTest, ConfigureWithDefaults)
{
    EXPECT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(FeedforwardTest, ConfigureRejectsInvalidParameters)
{
    const std::vector<std::pair<std::string, rclcpp::Parameter>> cases = {
        {"torque scale 0", rclcpp::Parameter("torque_scale", 0.0)},
        {"torque scale above 1", rclcpp::Parameter("torque_scale", 1.5)},
        {"non-finite torque scale", rclcpp::Parameter("torque_scale", nan)},
        {"negative rise time", rclcpp::Parameter("torque_rise_time", -0.1)},
        {"non-finite rise time", rclcpp::Parameter("torque_rise_time", nan)},
        {"unknown ramp start", rclcpp::Parameter("ramp_start", "last")},
    };
    for (const auto & [what, param] : cases) {
        EXPECT_EQ(configure({param}), State::PRIMARY_STATE_UNCONFIGURED) << what;
        controller_.reset();
    }
}

TEST_F(FeedforwardTest, InterfaceLayout)
{
    start(regular_pose());
    const auto cmd_names = controller_->command_interface_configuration().names;
    const auto state_names = controller_->state_interface_configuration().names;
    for (size_t j = 0; j < dof_; ++j) {
        EXPECT_EQ(cmd_names[3 * j + 0], names_[j] + "/position");
        EXPECT_EQ(cmd_names[3 * j + 1], names_[j] + "/velocity");
        EXPECT_EQ(cmd_names[3 * j + 2], names_[j] + "/torque_ff");
        EXPECT_EQ(state_names[2 * j + 0], names_[j] + "/position");
        EXPECT_EQ(state_names[2 * j + 1], names_[j] + "/effort");

        const std::string prefix = std::string(controller_name) + "/" + names_[j];
        EXPECT_EQ(refs_[3 * j + 0]->get_name(), prefix + "/position");
        EXPECT_EQ(refs_[3 * j + 1]->get_name(), prefix + "/velocity");
        EXPECT_EQ(refs_[3 * j + 2]->get_name(), prefix + "/acceleration");
    }
    for (size_t k = 0; k < refs_.size(); ++k) {
        EXPECT_TRUE(std::isnan(refs_[k]->get_optional<double>().value())) << "reference " << k << " not NaN";
    }
}

// ---------------------------------------------------------------------------------------------------
// realtime

TEST_F(FeedforwardTest, UpdateDoesNotAllocate)
{
    // from the first cycle on: no references (ramp), then a streamed trajectory, then a rate limited
    // torque step (logs a throttled warning)
    const Eigen::VectorXd q0 = regular_pose();
    start(q0, {rclcpp::Parameter("torque_rise_time", 0.03)});

    StepResult r = step(300);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);

    StepResult total;
    for (int c = 0; c < 1500; ++c) {
        const double t = c * 1e-3;
        const double w = 2.0 * M_PI;
        const Eigen::VectorXd ones = Eigen::VectorXd::Ones(dof_);
        set_reference(q0 + 0.05 * std::sin(w * t) * ones, 0.05 * w * std::cos(w * t) * ones,
                      -0.05 * w * w * std::sin(w * t) * ones);
        const StepResult s = step(1);
        total.allocations += s.allocations;
        if (s.allocations > 0 && total.first_cycle < 0) {
            total.first_cycle = c;
        }
        total.ok = total.ok && s.ok;
    }
    EXPECT_TRUE(total.ok);
    EXPECT_NO_RT_ALLOCATIONS(total);

    set_reference(q0, zero(), Eigen::VectorXd::Constant(dof_, 50.0));
    r = step(300);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
}

// ---------------------------------------------------------------------------------------------------
// feedforward

TEST_F(FeedforwardTest, WithoutReferenceHoldsActivationPose)
{
    const Eigen::VectorXd q = regular_pose();
    start(q);
    settle();
    EXPECT_VEC_NEAR(cmd_q(), q, exact);
    EXPECT_VEC_NEAR(cmd_dq(), zero(), exact);
    EXPECT_VEC_NEAR(tau(), gravity(q), exact);
}

TEST_F(FeedforwardTest, FeedforwardIsInverseDynamicsOfReference)
{
    // τ = M(q_d)·ddq_d + C(q_d, dq_d)·dq_d + g(q_d), position and velocity passed through
    start(regular_pose());
    settle();

    Eigen::VectorXd q_d, dq_d, ddq_d;
    moving_reference(q_d, dq_d, ddq_d);
    set_reference(q_d, dq_d, ddq_d);
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(cmd_q(), q_d, exact);
    EXPECT_VEC_NEAR(cmd_dq(), dq_d, exact);
    EXPECT_VEC_NEAR(tau(), ff(q_d, dq_d, ddq_d), exact);
    // every term contributes, else the comparison above could not tell them apart
    ASSERT_GT((ff(q_d, dq_d, ddq_d) - ff(q_d, dq_d, zero())).norm(), 1e-3);
    ASSERT_GT((ff(q_d, dq_d, zero()) - gravity(q_d)).norm(), 1e-3);
}

TEST_F(FeedforwardTest, FeedforwardIgnoresMeasuredState)
{
    // the measured position is only read on activation, the commands depend on the reference alone
    const Eigen::VectorXd q = regular_pose();
    start(q);
    settle();
    Eigen::VectorXd q_d, dq_d, ddq_d;
    moving_reference(q_d, dq_d, ddq_d);
    set_reference(q_d, dq_d, ddq_d);
    ASSERT_TRUE(step(1).ok);
    const Eigen::VectorXd tau_before = tau();

    set_state(q + Eigen::VectorXd::Constant(dof_, 0.3));
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(cmd_q(), q_d, exact);
    EXPECT_VEC_NEAR(cmd_dq(), dq_d, exact);
    EXPECT_VEC_NEAR(tau(), tau_before, exact);
}

TEST_F(FeedforwardTest, NonFiniteReferenceHoldsLastPosition)
{
    // a written position is kept when the upstream stops writing (NaN) or writes garbage (inf),
    // non-finite velocity and acceleration count as 0
    start(regular_pose());
    settle();
    Eigen::VectorXd q_d, dq_d, ddq_d;
    moving_reference(q_d, dq_d, ddq_d);
    set_reference(q_d, dq_d, ddq_d);
    ASSERT_TRUE(step(1).ok);

    for (const double bad : {nan, inf, -inf}) {
        SCOPED_TRACE("reference " + std::to_string(bad));
        set_reference(Eigen::VectorXd::Constant(dof_, bad), Eigen::VectorXd::Constant(dof_, bad),
                      Eigen::VectorXd::Constant(dof_, bad));
        ASSERT_TRUE(step(1).ok);
        EXPECT_VEC_NEAR(cmd_q(), q_d, exact);
        EXPECT_VEC_NEAR(cmd_dq(), zero(), exact);
        EXPECT_VEC_NEAR(tau(), gravity(q_d), exact);
    }
}

TEST_F(FeedforwardTest, PartialReferenceUsesWrittenEntries)
{
    // only the acceleration written, position NaN: the held pose stays the reference
    const Eigen::VectorXd q = regular_pose();
    start(q);
    settle();
    Eigen::VectorXd q_d, dq_d, ddq_d;
    moving_reference(q_d, dq_d, ddq_d);
    for (size_t j = 0; j < dof_; ++j) {
        (void)refs_[3 * j + 2]->set_value(ddq_d[j]);
    }
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(cmd_q(), q, exact);
    EXPECT_VEC_NEAR(tau(), ff(q, zero(), ddq_d), exact);
}

// ---------------------------------------------------------------------------------------------------
// lifecycle

TEST_F(FeedforwardTest, StartRampBlendsFromZero)
{
    // linear from 0 over 1 s: cycle c commands c·dt / 1 s of the feedforward
    const Eigen::VectorXd q = regular_pose();
    start(q);
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), zero(), exact);
    EXPECT_VEC_NEAR(cmd_q(), q, exact) << "position must be held from the first cycle on";
    ASSERT_TRUE(step(500).ok);
    EXPECT_VEC_NEAR(tau(), Eigen::VectorXd(0.5 * gravity(q)), exact);
    settle();
    EXPECT_VEC_NEAR(tau(), gravity(q), exact);
}

TEST_F(FeedforwardTest, ReactivationDropsStaleReferencesAndRestartsRamp)
{
    // a reference from the previous activation must not pull the arm back after reactivating
    const Eigen::VectorXd q_old = regular_pose();
    start(q_old);
    set_reference(q_old);
    settle();

    ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    controller_->release_interfaces();
    Eigen::VectorXd q_new = q_old;
    q_new[0] += 0.2;
    q_new[2] -= 0.2;
    assign(q_new);
    ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);

    for (size_t k = 0; k < refs_.size(); ++k) {
        EXPECT_TRUE(std::isnan(refs_[k]->get_optional<double>().value())) << "reference " << k;
    }
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(cmd_q(), q_new, exact);
    EXPECT_VEC_NEAR(tau(), zero(), exact) << "ramp did not restart";
    settle();
    EXPECT_VEC_NEAR(tau(), gravity(q_new), exact);
}

TEST_F(FeedforwardTest, StartRampBlendsFromMeasuredTorque)
{
    // out of an effort mode: continues from the measured torque, no step in the first cycle even with
    // a rate limit, linear to the feedforward over 1 s
    const Eigen::VectorXd q = regular_pose();
    start(q, {rclcpp::Parameter("torque_rise_time", 0.03)});
    settle();
    const Eigen::VectorXd tau_measured = effort_mode_torque(q);
    reactivate(q, tau_measured, "measured");

    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), tau_measured, exact);
    ASSERT_TRUE(step(500).ok);
    EXPECT_VEC_NEAR(tau(), Eigen::VectorXd(0.5 * (tau_measured + gravity(q))), exact);
    settle();
    EXPECT_VEC_NEAR(tau(), gravity(q), exact);
}

TEST_F(FeedforwardTest, StartRampZeroIgnoresMeasuredTorque)
{
    // the drive PID holds the measured torque (STOPPED), taking it over would double it
    const Eigen::VectorXd q = regular_pose();
    start(q);
    settle();
    reactivate(q, effort_mode_torque(q), "zero");
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), zero(), exact);
}

TEST_F(FeedforwardTest, ActivationRefusesInvalidRampStart)
{
    const Eigen::VectorXd q = regular_pose();
    start(q);
    ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(controller_->get_node()->set_parameter(rclcpp::Parameter("ramp_start", "last")).successful);
    EXPECT_NE(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(FeedforwardTest, ActivationRefusesNonFiniteMeasuredTorque)
{
    const Eigen::VectorXd q = regular_pose();
    start(q);
    ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    controller_->release_interfaces();
    assign(q);
    Eigen::VectorXd tau_measured = gravity(q);
    tau_measured[1] = nan;
    set_effort(tau_measured);
    ASSERT_TRUE(controller_->get_node()->set_parameter(rclcpp::Parameter("ramp_start", "measured")).successful);
    EXPECT_NE(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(FeedforwardTest, NonFiniteActivationStateNeverReachesHardware)
{
    // a NaN position passes activation (only has_value() is checked), the first update must stop
    // before anything is written
    Eigen::VectorXd q = regular_pose();
    q[2] = nan;
    start(q);
    EXPECT_FALSE(step(1).ok);
    for (size_t k = 0; k < cmd_.size(); ++k) {
        EXPECT_EQ(cmd_[k]->get_optional<double>().value(), 0.0) << "command " << k << " was written";
    }
}

// ---------------------------------------------------------------------------------------------------
// torque post-processing

TEST_F(FeedforwardTest, TorqueRateIsLimited)
{
    const double rise = 0.05;
    const Eigen::VectorXd q = regular_pose();
    start(q, {rclcpp::Parameter("torque_rise_time", rise)});
    settle();

    // an acceleration step: the torque follows with at most effort / rise_time per joint
    set_reference(q, zero(), Eigen::VectorXd::Constant(dof_, 50.0));
    bool limited = false;
    Eigen::VectorXd prev = tau();
    for (int c = 0; c < 200; ++c) {
        const StepResult r = step(1);
        ASSERT_TRUE(r.ok);
        EXPECT_NO_RT_ALLOCATIONS(r);
        const Eigen::VectorXd now = tau();
        for (size_t j = 0; j < dof_; ++j) {
            const double max_step = effort(j) / rise * 1e-3;
            EXPECT_LE(std::abs(now[j] - prev[j]), max_step + 1e-12) << "joint " << j << " cycle " << c;
            limited = limited || std::abs(std::abs(now[j] - prev[j]) - max_step) < 1e-12;
        }
        prev = now;
    }
    EXPECT_TRUE(limited) << "the rate limit never acted, adjust the scenario";
}

TEST_F(FeedforwardTest, ZeroPeriodHoldsTheTorque)
{
    // in the sim, sim time can stand still for a cycle while the controller manager runs on (period 0):
    // with a rate limit, no time passed means no torque change, and the update must not fail
    const Eigen::VectorXd q = regular_pose();
    start(q, {rclcpp::Parameter("torque_rise_time", 0.05)});
    settle();

    set_reference(q, zero(), Eigen::VectorXd::Constant(dof_, 5.0));
    const Eigen::VectorXd before = tau();
    ASSERT_EQ(controller_->update(time_, rclcpp::Duration::from_seconds(0.0)),
              controller_interface::return_type::OK);
    EXPECT_EQ(tau(), before);

    // the next real cycle moves again
    ASSERT_TRUE(step(1).ok);
    EXPECT_GT((tau() - before).cwiseAbs().maxCoeff(), 0.0);
}

TEST_F(FeedforwardTest, TorqueIsClampedToScaledEffort)
{
    const double scale = 0.25;
    const Eigen::VectorXd q = regular_pose();
    start(q, {rclcpp::Parameter("torque_scale", scale)});
    settle();
    set_reference(q, zero(), Eigen::VectorXd::Constant(dof_, 1e3));
    ASSERT_TRUE(step(1).ok);

    bool saturated = false;
    for (size_t j = 0; j < dof_; ++j) {
        EXPECT_LE(std::abs(tau()[j]), scale * effort(j) + 1e-12) << "joint " << j;
        saturated = saturated || std::abs(std::abs(tau()[j]) - scale * effort(j)) < 1e-12;
    }
    EXPECT_TRUE(saturated) << "no joint reached its limit, adjust the scenario";
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
