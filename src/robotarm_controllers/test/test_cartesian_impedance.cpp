// CartesianImpedanceController: realtime allocations, the control law and its safety layers.
//
// The controller is driven directly (no controller manager): own effort command and
// position/velocity/effort state interfaces, and the exported reference interfaces are written like
// an upstream controller would. There is no physics, the measured state is set by the test. That
// makes every cycle a static evaluation of the control law, checked against an own RobotarmRbd from
// the same URDF:
//
//   - physical properties instead of a copy of the formulas where possible: the spring pulls
//     towards the reference and linearizes to Jᵀ·K·J, the damping dissipates, every mode of the
//     linearized closed loop has the configured damping ratio, the joint damping stays bounded
//     through a singularity, the spring wrench stays within its limits, no torque pushes a joint
//     into its limit
//   - exact torques where the law is fixed by design (gravity, feedforward, damping on the velocity
//     error), to 1e-9
//
// Allocations are counted only inside update(), with the malloc interposer of malloc_counter.hpp.
// Log output is excluded (test_helpers::LogAllocationExclusion), the throttled warnings in update()
// are accepted. CounterDetectsAllocation is the negative control of this executable.
//
// The start ramp blends from the measured effort (0 here) over 1 s, so most tests run settle()
// first. torque_rise_time is 0 unless a test sets it.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>

#include "controller_interface/controller_interface_params.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robotarm_controllers/CartesianImpedanceController.hpp"
#include "robotarm_rbd/RobotarmRbd.hpp"

#include "malloc_counter.hpp"
#include "test_helpers.hpp"

namespace
{

using cartesian_impedance_controller::CartesianImpedanceController;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using lifecycle_msgs::msg::State;
using test_helpers::real_urdf;

constexpr unsigned int update_rate = 1000;
constexpr char controller_name[] = "cartesian_impedance_controller";
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
constexpr double exact = 1e-9;       // Nm, torques that are fixed by design
constexpr int settle_cycles = 1100;  // > 1 s start ramp at 1 kHz

// test gains, not the robot's: stiff enough to give clear torques, small enough to stay below the
// effort limits in a regular pose
const std::vector<double> stiffness = {500.0, 500.0, 500.0, 10.0, 10.0, 10.0};
constexpr double damping_ratio = 0.7;
const std::vector<double> joint_damping = {1.0, 2.0, 1.0, 0.1, 0.1, 0.1};
const std::vector<double> no_joint_damping(6, 0.0);
const std::vector<double> limit_zone = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
constexpr double limit_push = 0.2;

// all parameters of a mode, overrides replace an entry of the same name or are added
std::vector<rclcpp::Parameter> make_params(const std::string & mode,
                                           const std::vector<rclcpp::Parameter> & overrides = {})
{
    std::vector<rclcpp::Parameter> p = {
        rclcpp::Parameter("mode", mode),
        rclcpp::Parameter("joint_damping", joint_damping),
        rclcpp::Parameter("joint_limit_zone", limit_zone),
        rclcpp::Parameter("joint_limit_push", limit_push),
    };
    if (mode == "gc") {
        p.emplace_back("hold_stiffness", std::vector<double>{20.0, 50.0, 30.0, 4.0, 1.0, 1.0});
        p.emplace_back("hold_breakaway_torque", std::vector<double>{1.0, 2.5, 1.5, 0.4, 0.1, 0.1});
    } else {
        p.emplace_back("cartesian_linear_stiffness", std::vector<double>(stiffness.begin(), stiffness.begin() + 3));
        p.emplace_back("cartesian_angular_stiffness", std::vector<double>(stiffness.begin() + 3, stiffness.end()));
        p.emplace_back("max_linear_stiffness", 1e6);    // above every stiffness a test sets
        p.emplace_back("max_angular_stiffness", 1e6);
        p.emplace_back("cartesian_damping_ratio", damping_ratio);
        p.emplace_back("max_spring_force", 1e6);    // no clamp unless a test sets one
        p.emplace_back("max_spring_moment", 1e6);
    }
    for (const auto & o : overrides) {
        bool replaced = false;
        for (auto & existing : p) {
            if (existing.get_name() == o.get_name()) {
                existing = o;
                replaced = true;
            }
        }
        if (!replaced) {
            p.push_back(o);
        }
    }
    return p;
}

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

class CartesianImpedanceTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ASSERT_TRUE(rbd_.initialize(real_urdf(), robotarm_rbd::RobotarmRbd::Config{})) << rbd_.last_error();
        ASSERT_TRUE(rbd_.get_joint_limits(limits_));
        ASSERT_TRUE(rbd_.get_tcp_link_name(tcp_));
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
    uint8_t configure(const std::vector<rclcpp::Parameter> & params)
    {
        controller_ = std::make_shared<CartesianImpedanceController>();
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
    void start(const Eigen::VectorXd & q0, const std::vector<rclcpp::Parameter> & params)
    {
        ASSERT_EQ(configure(params), State::PRIMARY_STATE_INACTIVE);
        refs_ = controller_->export_reference_interfaces();
        ASSERT_EQ(refs_.size(), 3 * dof_);
        assign(q0);
        ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
    }

    // fresh hardware interfaces at rest at q, measured effort 0
    void assign(const Eigen::VectorXd & q)
    {
        cmd_.clear();
        state_.clear();
        const auto cmd_names = controller_->command_interface_configuration().names;
        const auto state_names = controller_->state_interface_configuration().names;
        ASSERT_EQ(cmd_names.size(), dof_);
        ASSERT_EQ(state_names.size(), 3 * dof_);

        std::vector<hardware_interface::LoanedCommandInterface> loaned_cmd;
        std::vector<hardware_interface::LoanedStateInterface> loaned_state;
        for (const auto & name : cmd_names) {
            const auto [prefix, iface] = test_helpers::split_interface_name(name);
            cmd_.push_back(std::make_shared<CommandInterface>(prefix, iface, "double", "0.0"));
            loaned_cmd.emplace_back(cmd_.back(), nullptr);
        }
        // layout of the controller: per joint position, velocity, effort
        for (size_t i = 0; i < state_names.size(); ++i) {
            const auto [prefix, iface] = test_helpers::split_interface_name(state_names[i]);
            const double init = (i % 3 == 0) ? q[static_cast<Eigen::Index>(i / 3)] : 0.0;
            state_.push_back(std::make_shared<StateInterface>(prefix, iface, "double", std::to_string(init)));
            loaned_state.emplace_back(state_.back(), nullptr);
        }
        controller_->assign_interfaces(std::move(loaned_cmd), std::move(loaned_state));
        // std::to_string rounds to 6 digits, set the exact values
        set_state(q, Eigen::VectorXd::Zero(dof_));
    }

    void set_state(const Eigen::VectorXd & q, const Eigen::VectorXd & dq)
    {
        for (size_t j = 0; j < dof_; ++j) {
            (void)state_[3 * j + 0]->set_value(q[j]);
            (void)state_[3 * j + 1]->set_value(dq[j]);
        }
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

    Eigen::VectorXd tau() const
    {
        Eigen::VectorXd t(dof_);
        for (size_t j = 0; j < dof_; ++j) {
            t[j] = cmd_[j]->get_optional<double>().value();
        }
        return t;
    }

    // own model, outside of any measured window

    Eigen::VectorXd gravity(const Eigen::VectorXd & q)
    {
        const Eigen::VectorXd zero = Eigen::VectorXd::Zero(dof_);
        Eigen::VectorXd t(dof_);
        EXPECT_TRUE(rbd_.recursive_newton_euler(q, zero, zero, t)) << rbd_.last_error();
        return t;
    }

    // M(q)·ddq + C(q, dq)·dq, without gravity
    Eigen::VectorXd dynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & dq, const Eigen::VectorXd & ddq)
    {
        Eigen::VectorXd t(dof_);
        EXPECT_TRUE(rbd_.recursive_newton_euler(q, dq, ddq, t, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                Eigen::Vector3d::Zero())) << rbd_.last_error();
        return t;
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(const Eigen::VectorXd & q)
    {
        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, dof_);
        EXPECT_TRUE(rbd_.calculate_jacobian(q, tcp_, J)) << rbd_.last_error();
        return J;
    }

    // the TCP wrench a joint torque corresponds to, τ = Jᵀ·F (J is regular in the test poses)
    Eigen::Matrix<double, 6, 1> wrench(const Eigen::VectorXd & q, const Eigen::VectorXd & t)
    {
        const Eigen::MatrixXd Jt = jacobian(q).transpose();
        return Jt.colPivHouseholderQr().solve(t);
    }

    static Eigen::Matrix<double, 6, 6> diag6(const std::vector<double> & v)
    {
        return Eigen::Map<const Eigen::Matrix<double, 6, 1>>(v.data()).asDiagonal();
    }

    Eigen::MatrixXd mass_matrix(const Eigen::VectorXd & q)
    {
        Eigen::MatrixXd M(dof_, dof_);
        EXPECT_TRUE(rbd_.calculate_mass_matrix(q, M)) << rbd_.last_error();
        return M;
    }

    // Cartesian damping for damping_ratio in every mode, formulated independently of the controller:
    // generalized eigenproblem K·v = μ·Λ·v with Vᵀ·Λ·V = I and Vᵀ·K·V = diag(μ). Each mode is a unit
    // mass on a spring μ and needs 2ζ·√μ, back in Cartesian coordinates D = Λ·V·diag(2ζ·√μ)·Vᵀ·Λ.
    Eigen::Matrix<double, 6, 6> modal_damping(const Eigen::VectorXd & q)
    {
        Eigen::Matrix<double, 6, 6> Lambda;
        EXPECT_TRUE(rbd_.calculate_task_space_inertia(q, tcp_, Lambda)) << rbd_.last_error();
        const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> ges(diag6(stiffness), Lambda);
        const Eigen::Matrix<double, 6, 1> d = 2.0 * damping_ratio * ges.eigenvalues().cwiseMax(0.0).cwiseSqrt();
        const Eigen::Matrix<double, 6, 6> LV = Lambda * ges.eigenvectors();
        return LV * d.asDiagonal() * LV.transpose();
    }

    // joint space damping matrix the controller applies at q, measured column by column from its
    // torque for a small velocity of one joint (reference at q and at rest, so no spring acts).
    // Includes joint_damping, start the controller with no_joint_damping to get Jᵀ·D·J alone.
    Eigen::MatrixXd measured_joint_damping(const Eigen::VectorXd & q)
    {
        constexpr double v = 1e-2;   // rad/s, small enough to stay far below the effort limits
        Eigen::MatrixXd C(dof_, dof_);
        set_reference(q);
        for (size_t i = 0; i < dof_; ++i) {
            Eigen::VectorXd dq = Eigen::VectorXd::Zero(dof_);
            dq[i] = v;
            set_state(q, dq);
            EXPECT_TRUE(step(1).ok) << "update failed at q = " << q.transpose();
            C.col(i) = -(tau() - gravity(q) - dynamics(q, dq, Eigen::VectorXd::Zero(dof_))) / v;
        }
        set_state(q, Eigen::VectorXd::Zero(dof_));
        return C;
    }

    double effort(size_t j) const {return limits_[j].effort;}

    // a regular pose away from singularities and joint limits (same as the jog test)
    Eigen::VectorXd regular_pose() const
    {
        Eigen::VectorXd q(6);
        q << 0.3, -1.0, 1.2, 0.2, 0.9, -0.3;
        return q;
    }

    std::shared_ptr<CartesianImpedanceController> controller_;
    std::vector<CommandInterface::SharedPtr> cmd_;
    std::vector<StateInterface::SharedPtr> state_;
    std::vector<CommandInterface::SharedPtr> refs_;

    robotarm_rbd::RobotarmRbd rbd_;
    std::vector<robotarm_rbd::RobotarmRbd::Limits> limits_;
    std::string tcp_;
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

TEST_F(CartesianImpedanceTest, ConfigureAcceptsBothModes)
{
    EXPECT_EQ(configure(make_params("gc")), State::PRIMARY_STATE_INACTIVE);
    controller_.reset();
    EXPECT_EQ(configure(make_params("impedance")), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(CartesianImpedanceTest, ConfigureIgnoresParametersOfTheOtherMode)
{
    // gc with impedance parameters set, impedance without the hold: both only warn
    std::vector<rclcpp::Parameter> gc = make_params("gc", {
        rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>{500.0, 500.0, 500.0}),
        rclcpp::Parameter("max_spring_force", 10.0),
    });
    EXPECT_EQ(configure(gc), State::PRIMARY_STATE_INACTIVE);
    controller_.reset();
    EXPECT_EQ(configure(make_params("impedance")), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(CartesianImpedanceTest, ConfigureRejectsInvalidParameters)
{
    const std::vector<std::pair<std::string, std::vector<rclcpp::Parameter>>> cases = {
        {"unknown mode", make_params("gc", {rclcpp::Parameter("mode", "position")})},
        {"impedance without linear stiffness",
         make_params("impedance", {rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>{})})},
        {"angular stiffness with 2 entries",
         make_params("impedance", {rclcpp::Parameter("cartesian_angular_stiffness", std::vector<double>{1, 1})})},
        {"negative linear stiffness",
         make_params("impedance", {rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>{1, -1, 1})})},
        {"angular stiffness above limit",
         make_params("impedance", {rclcpp::Parameter("max_angular_stiffness", 5.0)})},
        {"unknown stiffness frame",
         make_params("impedance", {rclcpp::Parameter("cartesian_stiffness_frame", "tool")})},
        {"no linear stiffness limit", make_params("impedance", {rclcpp::Parameter("max_linear_stiffness", 0.0)})},
        {"no angular stiffness limit", make_params("impedance", {rclcpp::Parameter("max_angular_stiffness", 0.0)})},
        {"negative damping ratio", make_params("impedance", {rclcpp::Parameter("cartesian_damping_ratio", -0.1)})},
        {"non-finite damping ratio", make_params("impedance", {rclcpp::Parameter("cartesian_damping_ratio", nan)})},
        {"no spring force limit", make_params("impedance", {rclcpp::Parameter("max_spring_force", 0.0)})},
        {"no spring moment limit", make_params("impedance", {rclcpp::Parameter("max_spring_moment", 0.0)})},
        {"gc without hold", make_params("gc", {rclcpp::Parameter("hold_stiffness", std::vector<double>{})})},
        {"joint damping missing", make_params("gc", {rclcpp::Parameter("joint_damping", std::vector<double>{})})},
        {"limit zone missing", make_params("impedance", {rclcpp::Parameter("joint_limit_zone", std::vector<double>{})})},
        {"overlapping limit zones",
         make_params("gc", {rclcpp::Parameter("joint_limit_zone", std::vector<double>{2.0, 2.0, 2.0, 2.0, 2.0, 2.0})})},
        {"push above 1", make_params("gc", {rclcpp::Parameter("joint_limit_push", 1.5)})},
        {"torque scale 0", make_params("gc", {rclcpp::Parameter("torque_scale", 0.0)})},
        {"negative rise time", make_params("gc", {rclcpp::Parameter("torque_rise_time", -0.1)})},
    };
    for (const auto & [what, params] : cases) {
        EXPECT_EQ(configure(params), State::PRIMARY_STATE_UNCONFIGURED) << what;
        controller_.reset();
    }
}

TEST_F(CartesianImpedanceTest, ExportsThreeReferenceInterfacesPerJoint)
{
    start(regular_pose(), make_params("impedance"));
    std::vector<std::string> names;
    ASSERT_TRUE(rbd_.get_joint_names(names));
    for (size_t j = 0; j < dof_; ++j) {
        const std::string prefix = std::string(controller_name) + "/" + names[j];
        EXPECT_EQ(refs_[3 * j + 0]->get_name(), prefix + "/position");
        EXPECT_EQ(refs_[3 * j + 1]->get_name(), prefix + "/velocity");
        EXPECT_EQ(refs_[3 * j + 2]->get_name(), prefix + "/acceleration");
        EXPECT_TRUE(std::isnan(refs_[3 * j]->get_optional<double>().value())) << "not NaN after activation";
    }
}

// ---------------------------------------------------------------------------------------------------
// realtime

TEST_F(CartesianImpedanceTest, GcUpdateDoesNotAllocate)
{
    // the arm is moved through the state, so the hold anchors are dragged along and joint damping acts
    const Eigen::VectorXd q0 = regular_pose();
    start(q0, make_params("gc"));
    StepResult total;
    for (int c = 0; c < 1500; ++c) {
        const double t = c * 1e-3;
        Eigen::VectorXd dq = Eigen::VectorXd::Constant(dof_, 0.5 * std::cos(2.0 * M_PI * t));
        Eigen::VectorXd q = q0 + Eigen::VectorXd::Constant(dof_, 0.5 / (2.0 * M_PI) * std::sin(2.0 * M_PI * t));
        set_state(q, dq);
        const StepResult r = step(1);
        total.allocations += r.allocations;
        if (r.allocations > 0 && total.first_cycle < 0) {
            total.first_cycle = c;
        }
        total.ok = total.ok && r.ok;
    }
    EXPECT_TRUE(total.ok);
    EXPECT_NO_RT_ALLOCATIONS(total);
}

TEST_F(CartesianImpedanceTest, ImpedanceUpdateDoesNotAllocate)
{
    // from the first cycle on: no references, then a streamed trajectory the arm lags behind, then
    // a joint inside its limit zone and a rate limited torque step (logs a throttled warning)
    const Eigen::VectorXd q0 = regular_pose();
    start(q0, make_params("impedance", {rclcpp::Parameter("torque_rise_time", 0.05)}));

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
        set_state(q0 + 0.05 * std::sin(w * (t - 0.02)) * ones, 0.05 * w * std::cos(w * (t - 0.02)) * ones);
        const StepResult s = step(1);
        total.allocations += s.allocations;
        if (s.allocations > 0 && total.first_cycle < 0) {
            total.first_cycle = c;
        }
        total.ok = total.ok && s.ok;
    }
    EXPECT_TRUE(total.ok);
    EXPECT_NO_RT_ALLOCATIONS(total);

    Eigen::VectorXd q = q0;
    q[1] = limits_[1].max - 0.5 * limit_zone[1];
    set_state(q, Eigen::VectorXd::Zero(dof_));
    Eigen::VectorXd q_d = q;
    q_d[1] += 0.3;
    set_reference(q_d);
    r = step(300);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
}

// ---------------------------------------------------------------------------------------------------
// gc mode

TEST_F(CartesianImpedanceTest, GcAtRestCommandsGravity)
{
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("gc"));
    settle();
    // hold anchored at the activation pose, at rest: neither hold nor damping act
    EXPECT_VEC_NEAR(tau(), gravity(q), exact);
}

TEST_F(CartesianImpedanceTest, GcHoldResistsUntilBreakaway)
{
    // inside the band the hold acts like a spring towards the activation pose, beyond it the anchor
    // is dragged along and the hold torque stays at the breakaway torque
    const Eigen::VectorXd q0 = regular_pose();
    start(q0, make_params("gc"));
    settle();

    Eigen::VectorXd q = q0;
    q[1] += 0.01;                                    // axis2: band 2.5 / 50 = 0.05 rad
    set_state(q, Eigen::VectorXd::Zero(dof_));
    ASSERT_TRUE(step(1).ok);
    EXPECT_NEAR(tau()[1] - gravity(q)[1], -50.0 * 0.01, exact);

    q[1] = q0[1] + 0.2;                              // far beyond the band
    set_state(q, Eigen::VectorXd::Zero(dof_));
    ASSERT_TRUE(step(1).ok);
    EXPECT_NEAR(tau()[1] - gravity(q)[1], -2.5, exact);

    q[1] -= 0.02;                                    // back by less than the band: anchor stays
    set_state(q, Eigen::VectorXd::Zero(dof_));
    ASSERT_TRUE(step(1).ok);
    EXPECT_NEAR(tau()[1] - gravity(q)[1], -2.5 + 50.0 * 0.02, exact);
}

TEST_F(CartesianImpedanceTest, GcIgnoresReferences)
{
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("gc"));
    settle();
    set_reference(q + Eigen::VectorXd::Constant(dof_, 0.3), Eigen::VectorXd::Ones(dof_), Eigen::VectorXd::Ones(dof_));
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), gravity(q), exact);
}

// ---------------------------------------------------------------------------------------------------
// impedance mode

TEST_F(CartesianImpedanceTest, ImpedanceWithoutReferenceHoldsActivationPose)
{
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance"));
    settle();
    EXPECT_VEC_NEAR(tau(), gravity(q), exact);

    // pushed away without a reference: the spring pulls back towards the activation pose
    Eigen::VectorXd q_pushed = q;
    q_pushed[0] += 0.01;
    set_state(q_pushed, Eigen::VectorXd::Zero(dof_));
    ASSERT_TRUE(step(1).ok);
    EXPECT_LT(tau()[0] - gravity(q_pushed)[0], -1e-3);
}

TEST_F(CartesianImpedanceTest, SpringLinearizesToJacobianStiffness)
{
    // small offset: τ - g = -Jᵀ·K·e with e ≈ J·(q - q_d), an independent formulation of the
    // pose error (log map) to first order
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance"));
    settle();

    Eigen::VectorXd dq_offset(6);
    dq_offset << 1.0, -0.5, 0.8, -1.2, 0.6, 1.5;
    dq_offset *= 1e-5;
    set_reference(q + dq_offset);
    ASSERT_TRUE(step(1).ok);

    const auto J = jacobian(q);
    const Eigen::VectorXd expected = J.transpose() * diag6(stiffness) * J * dq_offset;
    const Eigen::VectorXd actual = tau() - gravity(q);
    EXPECT_LT((actual - expected).norm(), 1e-3 * expected.norm())
        << "\n  actual:   " << actual.transpose() << "\n  expected: " << expected.transpose();
}

TEST_F(CartesianImpedanceTest, StiffnessCanBeSetAtRuntime)
{
    // same linearization as above, with the stiffness changed while active
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance"));
    settle();

    const std::vector<double> k_new = {1000.0, 200.0, 800.0, 5.0, 20.0, 15.0};
    auto node = controller_->get_node();
    ASSERT_TRUE(node->set_parameters_atomically({
        rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>(k_new.begin(), k_new.begin() + 3)),
        rclcpp::Parameter("cartesian_angular_stiffness", std::vector<double>(k_new.begin() + 3, k_new.end())),
    }).successful);

    Eigen::VectorXd dq_offset(6);
    dq_offset << 1.0, -0.5, 0.8, -1.2, 0.6, 1.5;
    dq_offset *= 1e-5;
    set_reference(q + dq_offset);
    const StepResult r = step(1);
    ASSERT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);

    const auto J = jacobian(q);
    const Eigen::VectorXd expected = J.transpose() * diag6(k_new) * J * dq_offset;
    const Eigen::VectorXd actual = tau() - gravity(q);
    EXPECT_LT((actual - expected).norm(), 1e-3 * expected.norm())
        << "\n  actual:   " << actual.transpose() << "\n  expected: " << expected.transpose();
}

TEST_F(CartesianImpedanceTest, StiffnessSetWhileInactiveIsUsedOnActivate)
{
    ASSERT_EQ(configure(make_params("impedance")), State::PRIMARY_STATE_INACTIVE);
    const std::vector<double> k_new = {1000.0, 200.0, 800.0, 5.0, 20.0, 15.0};
    auto node = controller_->get_node();
    ASSERT_TRUE(node->set_parameter(
        rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>(k_new.begin(), k_new.begin() + 3))).successful);
    ASSERT_TRUE(node->set_parameter(
        rclcpp::Parameter("cartesian_angular_stiffness", std::vector<double>(k_new.begin() + 3, k_new.end()))).successful);

    const Eigen::VectorXd q = regular_pose();
    refs_ = controller_->export_reference_interfaces();
    assign(q);
    ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
    settle();

    Eigen::VectorXd dq_offset(6);
    dq_offset << 1.0, -0.5, 0.8, -1.2, 0.6, 1.5;
    dq_offset *= 1e-5;
    set_reference(q + dq_offset);
    ASSERT_TRUE(step(1).ok);

    const auto J = jacobian(q);
    const Eigen::VectorXd expected = J.transpose() * diag6(k_new) * J * dq_offset;
    const Eigen::VectorXd actual = tau() - gravity(q);
    EXPECT_LT((actual - expected).norm(), 1e-3 * expected.norm())
        << "\n  actual:   " << actual.transpose() << "\n  expected: " << expected.transpose();
}

TEST_F(CartesianImpedanceTest, InvalidRuntimeStiffnessIsRejected)
{
    start(regular_pose(), make_params("impedance", {rclcpp::Parameter("max_linear_stiffness", 2000.0)}));
    auto node = controller_->get_node();
    const std::vector<std::pair<std::string, rclcpp::Parameter>> cases = {
        {"above limit", rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>{3000.0, 1.0, 1.0})},
        {"negative", rclcpp::Parameter("cartesian_angular_stiffness", std::vector<double>{1.0, -1.0, 1.0})},
        {"non-finite", rclcpp::Parameter("cartesian_angular_stiffness", std::vector<double>{1.0, nan, 1.0})},
        {"wrong length", rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>{1.0, 1.0})},
        {"unknown frame", rclcpp::Parameter("cartesian_stiffness_frame", "world")},
    };
    for (const auto & [what, param] : cases) {
        const auto result = node->set_parameter(param);
        EXPECT_FALSE(result.successful) << what;
        EXPECT_FALSE(result.reason.empty()) << what;
    }
    // the committed values are untouched
    EXPECT_EQ(node->get_parameter("cartesian_stiffness_frame").as_string(), "base");
    EXPECT_EQ(node->get_parameter("cartesian_linear_stiffness").as_double_array(),
              std::vector<double>(stiffness.begin(), stiffness.begin() + 3));
    EXPECT_EQ(node->get_parameter("cartesian_angular_stiffness").as_double_array(),
              std::vector<double>(stiffness.begin() + 3, stiffness.end()));
}

TEST_F(CartesianImpedanceTest, RuntimeStiffnessIsRejectedInGcMode)
{
    start(regular_pose(), make_params("gc"));
    auto node = controller_->get_node();
    EXPECT_FALSE(node->set_parameter(
        rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>{100.0, 100.0, 100.0})).successful);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("cartesian_stiffness_frame", "tcp")).successful);
}

TEST_F(CartesianImpedanceTest, StiffnessFrameCanBeSetAtRuntime)
{
    start(regular_pose(), make_params("impedance"));
    auto node = controller_->get_node();
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("cartesian_stiffness_frame", "tcp")).successful);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("cartesian_stiffness_frame", "base")).successful);
    const StepResult r = step(1);
    ASSERT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
}

TEST_F(CartesianImpedanceTest, TcpStiffnessLinearizesToRotatedJacobianStiffness)
{
    // anisotropic diagonal in the tcp frame: τ - g = Jᵀ·R6·K·R6ᵀ·J·δq, R from the own model at the reference
    const Eigen::VectorXd q = regular_pose();
    const std::vector<double> k = {1000.0, 200.0, 500.0, 5.0, 20.0, 12.0};
    start(q, make_params("impedance", {
        rclcpp::Parameter("cartesian_stiffness_frame", "tcp"),
        rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>(k.begin(), k.begin() + 3)),
        rclcpp::Parameter("cartesian_angular_stiffness", std::vector<double>(k.begin() + 3, k.end())),
    }));
    settle();

    Eigen::VectorXd dq_offset(6);
    dq_offset << 1.0, -0.5, 0.8, -1.2, 0.6, 1.5;
    dq_offset *= 1e-5;
    set_reference(q + dq_offset);
    ASSERT_TRUE(step(1).ok);

    Eigen::Isometry3d T_ref;
    ASSERT_TRUE(rbd_.calculate_link_transform(q + dq_offset, tcp_, T_ref)) << rbd_.last_error();
    const Eigen::Matrix3d R = T_ref.linear();
    ASSERT_GT((R - Eigen::Matrix3d::Identity()).norm(), 0.5) << "tcp too close to the base orientation";
    Eigen::Matrix<double, 6, 6> R6 = Eigen::Matrix<double, 6, 6>::Zero();
    R6.topLeftCorner<3, 3>() = R;
    R6.bottomRightCorner<3, 3>() = R;
    const Eigen::Matrix<double, 6, 6> K_base = R6 * diag6(k) * R6.transpose();

    const auto J = jacobian(q);
    const Eigen::VectorXd expected = J.transpose() * K_base * J * dq_offset;
    const Eigen::VectorXd expected_if_base = J.transpose() * diag6(k) * J * dq_offset;
    ASSERT_GT((expected - expected_if_base).norm(), 0.1 * expected.norm()) << "frames indistinguishable here";
    const Eigen::VectorXd actual = tau() - gravity(q);
    EXPECT_LT((actual - expected).norm(), 1e-3 * expected.norm())
        << "\n  actual:   " << actual.transpose() << "\n  expected: " << expected.transpose();
}

TEST_F(CartesianImpedanceTest, IsotropicStiffnessIsTheSameInBothFrames)
{
    // equal values per block: R·k·I·Rᵀ = k·I, switching the frame at runtime must not change the torque
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance", {rclcpp::Parameter("cartesian_stiffness_frame", "tcp")}));
    settle();

    Eigen::VectorXd offset(6);
    offset << 0.02, -0.01, 0.03, -0.02, 0.01, 0.02;
    set_reference(q + offset);
    ASSERT_TRUE(step(1).ok);
    const Eigen::VectorXd tau_tcp = tau();

    ASSERT_TRUE(controller_->get_node()->set_parameter(
        rclcpp::Parameter("cartesian_stiffness_frame", "base")).successful);
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), tau_tcp, 1e-9);
}

TEST_F(CartesianImpedanceTest, SpringPullsTowardsReference)
{
    // larger offsets along every joint: the spring torque does positive work towards the reference
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance"));
    settle();
    for (size_t j = 0; j < dof_; ++j) {
        for (const double sign : {1.0, -1.0}) {
            Eigen::VectorXd offset = Eigen::VectorXd::Zero(dof_);
            offset[j] = sign * 0.05;
            set_reference(q + offset);
            ASSERT_TRUE(step(1).ok);
            EXPECT_GT(offset.dot(tau() - gravity(q)), 0.0) << "joint " << j << " sign " << sign;
        }
    }
}

TEST_F(CartesianImpedanceTest, SpringWrenchIsClampedByNorm)
{
    // reference far away: the TCP wrench of the spring is limited to the configured norms
    const double f_max = 10.0;
    const double m_max = 1.0;
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance", {
        rclcpp::Parameter("max_spring_force", f_max),
        rclcpp::Parameter("max_spring_moment", m_max),
    }));
    settle();
    set_reference(q + Eigen::VectorXd::Constant(dof_, 0.3));
    ASSERT_TRUE(step(1).ok);

    const Eigen::Matrix<double, 6, 1> F = wrench(q, tau() - gravity(q));
    EXPECT_NEAR(F.head<3>().norm(), f_max, 1e-6);
    EXPECT_NEAR(F.tail<3>().norm(), m_max, 1e-6);
}

TEST_F(CartesianImpedanceTest, DampingDissipates)
{
    // at the reference pose, moving with a reference at rest: only the dampers act, and they take
    // energy out (τᵀ·dq < 0). Exact: -Jᵀ·D·J·dq - D_j·dq on top of gravity and Coriolis, with D from
    // the generalized eigenproblem (a different route than the controller's Cholesky + eigen
    // decomposition, so equal up to the rounding of the decompositions, not to 1e-9)
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance"));
    settle();

    Eigen::VectorXd dq(6);
    dq << 0.3, -0.2, 0.4, -0.5, 0.3, 0.6;
    set_reference(q);
    set_state(q, dq);
    ASSERT_TRUE(step(1).ok);

    const Eigen::VectorXd damping_torque = tau() - gravity(q) - dynamics(q, dq, Eigen::VectorXd::Zero(dof_));
    EXPECT_LT(damping_torque.dot(dq), 0.0) << "damping adds energy";

    const auto J = jacobian(q);
    const Eigen::Map<const Eigen::VectorXd> d_j(joint_damping.data(), dof_);
    const Eigen::VectorXd expected = -J.transpose() * modal_damping(q) * J * dq - d_j.cwiseProduct(dq);
    EXPECT_VEC_NEAR(damping_torque, expected, 1e-7);
}

TEST_F(CartesianImpedanceTest, EveryModeHasTheConfiguredDampingRatio)
{
    // the physical property the damping law is for: linearized around the reference, the closed loop
    // M·q̈ + C·q̇ + Jᵀ·K·J·q = 0 has the damping ratio ζ in every mode and in every pose, even though
    // the apparent mass changes with the pose. C is measured from the controller's torques, its
    // eigenvalues -ζω ± iω·√(1-ζ²) give ζ = -Re/|λ|. Off by the λ² regularization of Λ (~1e-3).
    Eigen::VectorXd q_far(6), q_folded(6);
    q_far << 0.0, -0.6, 0.6, -0.8, 1.2, 0.5;
    q_folded << -0.8, -1.5, 1.8, 1.0, -0.7, 1.2;
    start(regular_pose(), make_params("impedance", {rclcpp::Parameter("joint_damping", no_joint_damping)}));
    settle();

    for (const Eigen::VectorXd & q : {regular_pose(), q_far, q_folded}) {
        SCOPED_TRACE("q = " + std::to_string(q[0]) + " " + std::to_string(q[1]) + " " + std::to_string(q[2]));
        const Eigen::MatrixXd C = measured_joint_damping(q);
        const Eigen::MatrixXd M = mass_matrix(q);
        const auto J = jacobian(q);
        const Eigen::MatrixXd S = J.transpose() * diag6(stiffness) * J;

        const Eigen::Index n = static_cast<Eigen::Index>(dof_);
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * n, 2 * n);
        A.topRightCorner(n, n).setIdentity();
        A.bottomLeftCorner(n, n) = -M.ldlt().solve(S);
        A.bottomRightCorner(n, n) = -M.ldlt().solve(C);
        const Eigen::VectorXcd poles = Eigen::EigenSolver<Eigen::MatrixXd>(A).eigenvalues();
        for (Eigen::Index k = 0; k < poles.size(); ++k) {
            EXPECT_GT(std::abs(poles[k].imag()), 1e-6) << "mode does not oscillate, pole " << poles[k];
            EXPECT_NEAR(-poles[k].real() / std::abs(poles[k]), damping_ratio, 5e-3) << "pole " << poles[k];
        }
    }
}

TEST_F(CartesianImpedanceTest, DampingStaysBoundedThroughWristSingularity)
{
    // Λ grows without bound towards a singularity, but the motors only feel Jᵀ·D·J: in the singular
    // direction √Λ ~ 1/σ against J·J ~ σ², so the joint damping must neither blow up nor jump while
    // the wrist passes through axis5 = 0 (axis4 and axis6 aligned).
    Eigen::VectorXd q = regular_pose();
    q[4] = 0.0;
    Eigen::Matrix<double, Eigen::Dynamic, 6> j_inv(dof_, 6);
    robotarm_rbd::RobotarmRbd::SingularityInfo info;
    ASSERT_TRUE(rbd_.calculate_jacobian_inverse(q, tcp_, j_inv, &info)) << rbd_.last_error();
    ASSERT_LT(info.sigma_min(), 1e-6) << "axis5 = 0 is not singular in this pose, adjust the scenario";

    start(regular_pose(), make_params("impedance", {rclcpp::Parameter("joint_damping", no_joint_damping)}));
    settle();

    constexpr int samples = 41;
    constexpr double half_range = 0.2;   // rad around axis5 = 0
    std::vector<Eigen::MatrixXd> C(samples);
    double largest_at_ends = 0.0;
    double largest = 0.0;
    for (int k = 0; k < samples; ++k) {
        q[4] = -half_range + 2.0 * half_range * k / (samples - 1);
        C[k] = measured_joint_damping(q);
        const Eigen::MatrixXd C_sym = 0.5 * (C[k] + C[k].transpose());
        const double max_eig = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(C_sym).eigenvalues().maxCoeff();
        largest = std::max(largest, max_eig);
        if (k == 0 || k == samples - 1) {
            largest_at_ends = std::max(largest_at_ends, max_eig);
        }
    }
    EXPECT_LT(largest, 2.0 * largest_at_ends) << "joint damping grows towards the singularity";
    for (int k = 1; k < samples; ++k) {
        EXPECT_LT((C[k] - C[k - 1]).norm(), 0.1 * largest)
            << "joint damping jumps between axis5 = " << -half_range + 2.0 * half_range * (k - 1) / (samples - 1)
            << " and the next sample";
    }
}

TEST_F(CartesianImpedanceTest, DampingActsOnVelocityError)
{
    // following the reference exactly (pose and velocity): no damping, only the feedforward
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance"));
    settle();

    Eigen::VectorXd dq(6);
    dq << 0.3, -0.2, 0.4, -0.5, 0.3, 0.6;
    set_reference(q, dq);
    set_state(q, dq);
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), gravity(q) + dynamics(q, dq, Eigen::VectorXd::Zero(dof_)), exact);
}

TEST_F(CartesianImpedanceTest, AccelerationFeedforward)
{
    // at rest on the reference with a reference acceleration: τ = g + M·ddq_d
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance"));
    settle();

    Eigen::VectorXd ddq(6);
    ddq << 1.0, -0.5, 0.8, 2.0, -1.5, 1.0;
    set_reference(q, Eigen::VectorXd::Zero(dof_), ddq);
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), gravity(q) + dynamics(q, Eigen::VectorXd::Zero(dof_), ddq), exact);
}

TEST_F(CartesianImpedanceTest, ReactivationDropsStaleReferences)
{
    // a reference from the previous activation must not pull the arm back after reactivating
    const Eigen::VectorXd q_old = regular_pose();
    start(q_old, make_params("impedance"));
    settle();
    set_reference(q_old);
    ASSERT_TRUE(step(1).ok);

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
    settle();
    EXPECT_VEC_NEAR(tau(), gravity(q_new), exact);
}

TEST_F(CartesianImpedanceTest, PartialReferenceHoldsPosition)
{
    // only velocity and acceleration written, position NaN: the held pose stays the reference
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance"));
    settle();

    Eigen::VectorXd ddq(6);
    ddq << 1.0, -0.5, 0.8, 2.0, -1.5, 1.0;
    for (size_t j = 0; j < dof_; ++j) {
        (void)refs_[3 * j + 2]->set_value(ddq[j]);
    }
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), gravity(q) + dynamics(q, Eigen::VectorXd::Zero(dof_), ddq), exact);
}

// ---------------------------------------------------------------------------------------------------
// joint limit protection

TEST_F(CartesianImpedanceTest, LimitProtectionWinsAgainstTheSpring)
{
    // the reference pulls axis2 into its upper and axis5 into its lower limit. Outside the zone the
    // spring torque points into the limit, at and beyond the limit only the outward push is left
    const Eigen::VectorXd q0 = regular_pose();
    start(q0, make_params("impedance"));
    settle();

    struct Case { size_t joint; double limit; double into; };
    const Case cases[] = {
        {1, limits_[1].max, +1.0},
        {4, limits_[4].min, -1.0},
    };
    for (const Case & c : cases) {
        SCOPED_TRACE("joint " + std::to_string(c.joint));
        Eigen::VectorXd q_d = q0;
        q_d[c.joint] = c.limit + c.into * 0.2;
        set_reference(q_d);

        Eigen::VectorXd q = q0;
        q[c.joint] = c.limit - c.into * (limit_zone[c.joint] + 0.05);   // outside the zone
        set_state(q, Eigen::VectorXd::Zero(dof_));
        ASSERT_TRUE(step(1).ok);
        ASSERT_GT(c.into * (tau()[c.joint] - gravity(q)[c.joint]), 0.0) << "spring does not pull into the limit";

        for (const double beyond : {0.0, 0.05}) {
            q[c.joint] = c.limit + c.into * beyond;
            set_state(q, Eigen::VectorXd::Zero(dof_));
            ASSERT_TRUE(step(1).ok);
            EXPECT_NEAR(tau()[c.joint] - gravity(q)[c.joint], -c.into * limit_push * effort(c.joint), exact)
                << "beyond the limit by " << beyond;
        }
    }
}

TEST_F(CartesianImpedanceTest, LimitProtectionInGcMode)
{
    // activated at the limit, nothing else acts: only the push
    Eigen::VectorXd q = regular_pose();
    q[1] = limits_[1].max;
    start(q, make_params("gc"));
    settle();
    EXPECT_NEAR(tau()[1] - gravity(q)[1], -limit_push * effort(1), exact);
}

TEST_F(CartesianImpedanceTest, LimitProtectionLeavesTorqueAwayFromTheLimit)
{
    // in the zone, a reference pulling away from the limit is not scaled, only the push adds to it
    const Eigen::VectorXd q0 = regular_pose();
    start(q0, make_params("impedance"));
    settle();

    Eigen::VectorXd q = q0;
    q[1] = limits_[1].max - 0.5 * limit_zone[1];    // half way into the zone, s = 0.5
    Eigen::VectorXd q_d = q;
    q_d[1] -= 0.05;                                 // away from the upper limit
    set_reference(q_d);
    set_state(q, Eigen::VectorXd::Zero(dof_));
    ASSERT_TRUE(step(1).ok);
    const double with_zone = tau()[1] - gravity(q)[1];

    // same spring, push removed
    const double push = -0.5 * limit_push * effort(1);
    ASSERT_LT(with_zone - push, 0.0) << "spring does not pull away from the limit";

    // the spring part matches the same offset outside the zone (stiffness barely changes over 0.05 rad)
    Eigen::VectorXd q_out = q0;
    Eigen::VectorXd q_d_out = q0;
    q_d_out[1] -= 0.05;
    set_reference(q_d_out);
    set_state(q_out, Eigen::VectorXd::Zero(dof_));
    ASSERT_TRUE(step(1).ok);
    const double spring_out = tau()[1] - gravity(q_out)[1];
    EXPECT_NEAR(with_zone - push, spring_out, 0.2 * std::abs(spring_out)) << "torque away from the limit was scaled";
}

// ---------------------------------------------------------------------------------------------------
// torque post-processing

TEST_F(CartesianImpedanceTest, TorqueRateIsLimited)
{
    const double rise = 0.05;
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance", {rclcpp::Parameter("torque_rise_time", rise)}));
    settle();

    // a reference jump: the torque follows with at most effort / rise_time per joint
    Eigen::VectorXd q_d = q;
    q_d[0] += 0.3;
    q_d[1] -= 0.2;
    set_reference(q_d);
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

TEST_F(CartesianImpedanceTest, ZeroPeriodHoldsTheTorque)
{
    // in the sim, sim time can stand still for a cycle while the controller manager runs on (period 0):
    // no time passed, so the torque must not change, and the update must not fail
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance", {rclcpp::Parameter("torque_rise_time", 0.05)}));
    settle();

    Eigen::VectorXd q_d = q;
    q_d[0] += 0.3;
    set_reference(q_d);
    const Eigen::VectorXd before = tau();
    ASSERT_EQ(controller_->update(time_, rclcpp::Duration::from_seconds(0.0)),
              controller_interface::return_type::OK);
    EXPECT_EQ(tau(), before);

    // the next real cycle moves again
    ASSERT_TRUE(step(1).ok);
    EXPECT_GT((tau() - before).cwiseAbs().maxCoeff(), 0.0);
}

TEST_F(CartesianImpedanceTest, TorqueIsClampedToScaledEffort)
{
    const double scale = 0.25;
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("impedance", {
        rclcpp::Parameter("torque_scale", scale),
        rclcpp::Parameter("cartesian_linear_stiffness", std::vector<double>{1e5, 1e5, 1e5}),
        rclcpp::Parameter("cartesian_angular_stiffness", std::vector<double>{1e3, 1e3, 1e3}),
    }));
    settle();
    set_reference(q + Eigen::VectorXd::Constant(dof_, 0.3));
    ASSERT_TRUE(step(1).ok);

    bool saturated = false;
    for (size_t j = 0; j < dof_; ++j) {
        EXPECT_LE(std::abs(tau()[j]), scale * effort(j) + 1e-12) << "joint " << j;
        saturated = saturated || std::abs(std::abs(tau()[j]) - scale * effort(j)) < 1e-12;
    }
    EXPECT_TRUE(saturated) << "no joint reached its limit, adjust the scenario";
}

TEST_F(CartesianImpedanceTest, StartRampBlendsFromMeasuredEffort)
{
    // measured effort 0 at activation: the first command is 0, full torque after the ramp
    const Eigen::VectorXd q = regular_pose();
    start(q, make_params("gc"));
    ASSERT_TRUE(step(1).ok);
    EXPECT_VEC_NEAR(tau(), Eigen::VectorXd::Zero(dof_), exact);
    ASSERT_TRUE(step(500).ok);
    const Eigen::VectorXd half = tau();
    EXPECT_GT(half.norm(), 0.3 * gravity(q).norm());
    EXPECT_LT(half.norm(), 0.7 * gravity(q).norm());
    settle();
    EXPECT_VEC_NEAR(tau(), gravity(q), exact);
}

TEST_F(CartesianImpedanceTest, NonFiniteStateStops)
{
    for (const std::string mode : {"gc", "impedance"}) {
        SCOPED_TRACE(mode);
        Eigen::VectorXd q = regular_pose();
        start(q, make_params(mode));
        ASSERT_TRUE(step(10).ok);
        q[2] = nan;
        set_state(q, Eigen::VectorXd::Zero(dof_));
        EXPECT_FALSE(step(1).ok);
        TearDown();
    }
}

// ---------------------------------------------------------------------------------------------------
// the wrench convention the README documents: Jᵀ·F equals the RNEA with the wrench at the TCP

TEST_F(CartesianImpedanceTest, JacobianTransposeMatchesRneaWrench)
{
    const Eigen::VectorXd q = regular_pose();
    Eigen::Isometry3d T;
    ASSERT_TRUE(rbd_.calculate_link_transform(q, tcp_, T)) << rbd_.last_error();
    Eigen::Matrix<double, 6, 1> F;
    F << 3.0, -2.0, 5.0, 0.4, -0.3, 0.2;      // base frame, force on the environment

    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(dof_);
    Eigen::VectorXd with_wrench(dof_);
    ASSERT_TRUE(rbd_.recursive_newton_euler(q, zero, zero, with_wrench,
                                            T.linear().transpose() * F.head<3>(),
                                            T.linear().transpose() * F.tail<3>())) << rbd_.last_error();
    EXPECT_VEC_NEAR(with_wrench - gravity(q), Eigen::VectorXd(jacobian(q).transpose() * F), exact);
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
