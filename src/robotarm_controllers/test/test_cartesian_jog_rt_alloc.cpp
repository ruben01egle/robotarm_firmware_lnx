// CartesianJogController::update() runs in the realtime thread of the controller manager and must not
// allocate, from the very first cycle after activation on.
//
// The controller is driven directly (no controller manager): own state/command interfaces with ideal
// tracking (the commanded position/velocity is fed back as the measured state), twist commands are
// delivered over the real subscription outside of the measured window. Allocations are counted with
// the malloc interposer of malloc_counter.hpp, which also sees allocations in ruckig, rclcpp
// logging, realtime_tools etc., not only Eigen's.
//
// CounterDetectsAllocation is the negative control: without it, a counter that is silently not
// linked in would let all other checks pass.
//
// Log output is excluded from counting (LogAllocationExclusion): the logging backend allocates for
// every message that is actually printed (rosout publishes over DDS). In update() those are throttled
// warnings and the errors right before a stop, accepted as rare. Everything up to the log call, incl.
// the throttle check, is still counted.
//
// The tool frame and increment tests also check the motion itself (direction, distance, continuity),
// because those can't be seen in the allocation count.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/controller_interface_params.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "robotarm_interface/msg/cartesian_increment.hpp"

#include "robotarm_controllers/CartesianJogController.hpp"

#include "malloc_counter.hpp"

namespace
{

using cartesian_jog_controller::CartesianJogController;
using geometry_msgs::msg::TwistStamped;
using robotarm_interface::msg::CartesianIncrement;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;

constexpr unsigned int update_rate = 1000;
constexpr char controller_name[] = "cartesian_jog_controller";

// URDF of the real robot, processed by xacro (same as robotarm_rbd/test/real_urdf.hpp, whose test
// directory is not installed)
std::string real_urdf()
{
    static const std::string cached = [] {
        const std::string share = ament_index_cpp::get_package_share_directory("robotarm_description");
        const std::string cmd = "xacro '" + share + "/urdf/robotarm.urdf.xacro' 2>/dev/null";
        FILE * pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("cannot run xacro");
        }
        std::string out;
        char buf[4096];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), pipe)) > 0) {
            out.append(buf, n);
        }
        if (pclose(pipe) != 0 || out.empty()) {
            throw std::runtime_error("xacro failed on " + share + "/urdf/robotarm.urdf.xacro");
        }
        return out;
    }();
    return cached;
}

// empty frame_id is the base frame
TwistStamped make_twist(double vx, double vy, double vz, double wx, double wy, double wz,
                        const std::string & frame_id = "")
{
    TwistStamped t;
    t.header.frame_id = frame_id;
    t.twist.linear.x = vx;
    t.twist.linear.y = vy;
    t.twist.linear.z = vz;
    t.twist.angular.x = wx;
    t.twist.angular.y = wy;
    t.twist.angular.z = wz;
    return t;
}

TwistStamped make_twist(const Eigen::Vector3d & v, const Eigen::Vector3d & w, const std::string & frame_id)
{
    return make_twist(v.x(), v.y(), v.z(), w.x(), w.y(), w.z(), frame_id);
}

// one increment step, linear in m, angular in rad. Empty frame_id is the base frame
CartesianIncrement make_increment(double dx, double dy, double dz, double rx, double ry, double rz,
                                  const std::string & frame_id = "")
{
    CartesianIncrement inc;
    inc.header.frame_id = frame_id;
    inc.linear.x = dx;
    inc.linear.y = dy;
    inc.linear.z = dz;
    inc.angular.x = rx;
    inc.angular.y = ry;
    inc.angular.z = rz;
    return inc;
}

// Wraps the installed log output handler and pauses the allocation counter while it runs, see the
// file comment. Installed once in main(), after rclcpp::init() has set up the logging backend.
class LogAllocationExclusion
{
public:
    static void install()
    {
        previous_ = rcutils_logging_get_output_handler();
        rcutils_logging_set_output_handler(&LogAllocationExclusion::handler);
    }

private:
    static void handler(
        const rcutils_log_location_t * location, int severity, const char * name,
        rcutils_time_point_value_t timestamp, const char * format, va_list * args)
    {
        malloc_counter::ScopedPause pause;
        if (previous_) {
            previous_(location, severity, name, timestamp, format, args);
        }
    }

    static inline rcutils_logging_output_handler_t previous_ = nullptr;
};

// Counts log messages whose format string contains a marker and forwards everything to the handler
// that was installed before. Forwarding keeps the real logging backend inside the measured window.
class LogMarkerCounter
{
public:
    explicit LogMarkerCounter(const char * marker)
    {
        marker_ = marker;
        hits_ = 0;
        previous_ = rcutils_logging_get_output_handler();
        rcutils_logging_set_output_handler(&LogMarkerCounter::handler);
    }
    ~LogMarkerCounter() {rcutils_logging_set_output_handler(previous_);}

    int hits() const {return hits_.load();}

private:
    static void handler(
        const rcutils_log_location_t * location, int severity, const char * name,
        rcutils_time_point_value_t timestamp, const char * format, va_list * args)
    {
        if (format && std::strstr(format, marker_)) {
            ++hits_;
        }
        if (previous_) {
            previous_(location, severity, name, timestamp, format, args);
        }
    }

    static inline const char * marker_ = nullptr;
    static inline std::atomic<int> hits_{0};
    static inline rcutils_logging_output_handler_t previous_ = nullptr;
};

struct StepResult
{
    size_t allocations = 0;     // summed over all cycles
    int first_cycle = -1;       // first cycle that allocated, -1 if none
    bool ok = true;             // all updates returned OK
};

class CartesianJogRtAllocTest : public ::testing::Test
{
protected:
    void TearDown() override
    {
        if (controller_) {
            controller_->get_node()->deactivate();
            controller_->release_interfaces();
        }
        executor_.reset();
        controller_.reset();
        pub_node_.reset();
    }

    // init, configure, assign interfaces with the start pose q0, activate
    void start(const std::vector<double> & q0, const std::vector<rclcpp::Parameter> & overrides = {})
    {
        controller_ = std::make_shared<CartesianJogController>();

        controller_interface::ControllerInterfaceParams params;
        params.controller_name = controller_name;
        params.robot_description = real_urdf();
        params.update_rate = update_rate;
        params.controller_manager_update_rate = update_rate;
        params.node_options = controller_->define_custom_node_options();
        params.node_options.parameter_overrides(overrides);
        ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);

        ASSERT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

        const auto cmd_names = controller_->command_interface_configuration().names;
        const auto state_names = controller_->state_interface_configuration().names;
        ASSERT_EQ(cmd_names.size(), 2 * q0.size());
        ASSERT_EQ(state_names.size(), 2 * q0.size());

        std::vector<hardware_interface::LoanedCommandInterface> loaned_cmd;
        std::vector<hardware_interface::LoanedStateInterface> loaned_state;
        for (size_t i = 0; i < cmd_names.size(); ++i) {
            const auto [cprefix, cname] = split(cmd_names[i]);
            const auto [sprefix, sname] = split(state_names[i]);
            // layout of the controller: per joint position, then velocity
            const double init = (i % 2 == 0) ? q0[i / 2] : 0.0;
            cmd_.push_back(std::make_shared<CommandInterface>(cprefix, cname, "double", std::to_string(init)));
            state_.push_back(std::make_shared<StateInterface>(sprefix, sname, "double", std::to_string(init)));
            loaned_cmd.emplace_back(cmd_.back(), nullptr);
            loaned_state.emplace_back(state_.back(), nullptr);
        }
        controller_->assign_interfaces(std::move(loaned_cmd), std::move(loaned_state));

        ASSERT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

        // own kinematic model for frame names and FK, same source as the controller
        ASSERT_TRUE(rbd_.initialize(real_urdf(), robotarm_rbd::RobotarmRbd::Config{})) << rbd_.last_error();
        std::vector<std::string> links;
        ASSERT_TRUE(rbd_.get_link_names(links));
        ASSERT_TRUE(rbd_.get_tcp_link_name(tcp_frame_));
        base_frame_ = links[0];

        pub_node_ = std::make_shared<rclcpp::Node>("twist_publisher");
        const std::string topic = std::string("/") + controller_name + "/twist_cmds";
        twist_pub_ = pub_node_->create_publisher<TwistStamped>(topic, rclcpp::SystemDefaultsQoS());
        echo_sub_ = pub_node_->create_subscription<TwistStamped>(topic, rclcpp::SystemDefaultsQoS(),
                                                                 [this](TwistStamped::ConstSharedPtr) {++echo_count_;});
        const std::string inc_topic = std::string("/") + controller_name + "/increment_cmds";
        inc_pub_ = pub_node_->create_publisher<CartesianIncrement>(inc_topic, rclcpp::SystemDefaultsQoS());
        inc_echo_sub_ = pub_node_->create_subscription<CartesianIncrement>(
            inc_topic, rclcpp::SystemDefaultsQoS(), [this](CartesianIncrement::ConstSharedPtr) {++inc_echo_count_;});
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(controller_->get_node()->get_node_base_interface());
        executor_->add_node(pub_node_);
    }

    void send_twist(const TwistStamped & t) {publish_and_wait(twist_pub_, echo_count_, t);}
    void send_increment(const CartesianIncrement & inc) {publish_and_wait(inc_pub_, inc_echo_count_, inc);}

    // publishes over the real subscription and spins until it is delivered, outside of any
    // measured window
    template<class Msg>
    void publish_and_wait(const std::shared_ptr<rclcpp::Publisher<Msg>> & pub, const int & echo_count, const Msg & msg)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (pub->get_subscription_count() < 2) {   // controller + echo
            ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "subscription not matched";
            executor_->spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const int expected = echo_count + 1;
        pub->publish(msg);
        // the echo subscriber gets the same delivery as the controller, spin a bit more after it
        // arrived so the controller's callback has run too
        while (echo_count < expected) {
            ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "message not delivered";
            executor_->spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        for (int i = 0; i < 5; ++i) {
            executor_->spin_some();
        }
    }

    // runs n control cycles, counting allocations only inside update(). Between the cycles the
    // commanded position/velocity is fed back as measured state (ideal tracking).
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
            if (!r.ok) {
                break;
            }
            for (size_t i = 0; i < cmd_.size(); ++i) {
                (void)state_[i]->set_value(cmd_[i]->get_optional<double>().value());
            }
            time_ += period_;
        }
        return r;
    }

    double q_cmd(size_t joint) const {return cmd_[2 * joint]->get_optional<double>().value();}
    double dq_cmd(size_t joint) const {return cmd_[2 * joint + 1]->get_optional<double>().value();}

    double max_abs_dq_cmd() const
    {
        double m = 0.0;
        for (size_t j = 0; j < cmd_.size() / 2; ++j) {
            m = std::max(m, std::abs(dq_cmd(j)));
        }
        return m;
    }

    Eigen::VectorXd q_cmd_vec() const
    {
        Eigen::VectorXd q(cmd_.size() / 2);
        for (Eigen::Index j = 0; j < q.size(); ++j) {
            q[j] = q_cmd(j);
        }
        return q;
    }

    Eigen::VectorXd dq_cmd_vec() const
    {
        Eigen::VectorXd dq(cmd_.size() / 2);
        for (Eigen::Index j = 0; j < dq.size(); ++j) {
            dq[j] = dq_cmd(j);
        }
        return dq;
    }

    // FK of the commanded joint position, outside of any measured window
    Eigen::Isometry3d tcp_pose()
    {
        Eigen::Isometry3d T;
        EXPECT_TRUE(rbd_.calculate_link_transform(q_cmd_vec(), tcp_frame_, T)) << rbd_.last_error();
        return T;
    }

    static std::pair<std::string, std::string> split(const std::string & full)
    {
        const size_t slash = full.rfind('/');
        return {full.substr(0, slash), full.substr(slash + 1)};
    }

    // a regular pose away from singularities and joint limits
    static std::vector<double> regular_pose() {return {0.3, -1.0, 1.2, 0.2, 0.9, -0.3};}

    std::shared_ptr<CartesianJogController> controller_;
    std::vector<CommandInterface::SharedPtr> cmd_;
    std::vector<StateInterface::SharedPtr> state_;

    robotarm_rbd::RobotarmRbd rbd_;
    std::string base_frame_;
    std::string tcp_frame_;

    rclcpp::Node::SharedPtr pub_node_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<TwistStamped>::SharedPtr echo_sub_;
    int echo_count_ = 0;
    rclcpp::Publisher<CartesianIncrement>::SharedPtr inc_pub_;
    rclcpp::Subscription<CartesianIncrement>::SharedPtr inc_echo_sub_;
    int inc_echo_count_ = 0;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

    rclcpp::Time time_{0, 0, RCL_ROS_TIME};
    const rclcpp::Duration period_ = rclcpp::Duration::from_seconds(1.0 / update_rate);
};

#define EXPECT_NO_RT_ALLOCATIONS(r) \
    EXPECT_EQ((r).allocations, 0u) << "update() allocated, first in cycle " << (r).first_cycle \
                                   << " (backtrace on stderr)"

TEST(MallocCounter, CounterDetectsAllocation)
{
    // negative control: these must be seen, else the interposer is not active
    EXPECT_GT(malloc_counter::count_allocations([] {
            std::vector<int> v;
            v.reserve(16);
        }), 0u);
    EXPECT_GT(malloc_counter::count_allocations([] {
            Eigen::VectorXd v(6);
            v.setZero();
        }), 0u);
    EXPECT_GT(malloc_counter::count_allocations([] {
            auto p = std::make_unique<double>(1.0);
            // an unused new/delete pair may be elided under optimization (C++14), this keeps it
            asm volatile("" : : "g"(p.get()) : "memory");
        }), 0u);

    // and nothing is counted while disarmed
    EXPECT_EQ(malloc_counter::count_allocations([] {
            double x = 1.0;
            (void)x;
        }), 0u);
    std::vector<int> outside(16);
    EXPECT_EQ(malloc_counter::count(), 0u);

    // a paused section is excluded, counting continues after it (log exclusion relies on both)
    EXPECT_EQ(malloc_counter::count_allocations([] {
            malloc_counter::ScopedPause pause;
            auto p = std::make_unique<double>(1.0);
            asm volatile("" : : "g"(p.get()) : "memory");
        }), 0u);
    EXPECT_GT(malloc_counter::count_allocations([] {
            {
                malloc_counter::ScopedPause pause;
            }
            auto p = std::make_unique<double>(1.0);
            asm volatile("" : : "g"(p.get()) : "memory");
        }), 0u);
}

TEST_F(CartesianJogRtAllocTest, IdleUpdateDoesNotAllocate)
{
    start(regular_pose());
    const StepResult r = step(1000);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
}

TEST_F(CartesianJogRtAllocTest, JoggingDoesNotAllocate)
{
    start(regular_pose());
    send_twist(make_twist(0.05, -0.03, 0.02, 0.1, -0.2, 0.15));
    StepResult r = step(200);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    // sanity: the full ruckig / IK path ran (checked before the watchdog stops the arm)
    EXPECT_GT(max_abs_dq_cmd(), 1e-3);

    // watchdog timeout and braking to rest
    r = step(1800);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
}

TEST_F(CartesianJogRtAllocTest, NewCommandPickupDoesNotAllocate)
{
    // a fresh message in the realtime box is copied out by try_get() inside update()
    start(regular_pose());
    const TwistStamped cmds[] = {
        make_twist(0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
        make_twist(0.0, 0.05, 0.0, 0.0, 0.0, 0.2),
        make_twist(-0.05, 0.0, 0.03, 0.2, 0.0, 0.0),
        make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    };
    for (int round = 0; round < 3; ++round) {
        for (const TwistStamped & t : cmds) {
            SCOPED_TRACE("round " + std::to_string(round));
            send_twist(t);
            const StepResult r = step(200);
            EXPECT_TRUE(r.ok);
            EXPECT_NO_RT_ALLOCATIONS(r);
        }
    }
}

TEST_F(CartesianJogRtAllocTest, ToolFrameAndFrameSwitchDoNotAllocate)
{
    // alternating frames while moving runs the ruckig state transform inside update()
    start(regular_pose());
    const std::string frames[] = {tcp_frame_, base_frame_, tcp_frame_, ""};
    for (int round = 0; round < 2; ++round) {
        for (const std::string & frame : frames) {
            SCOPED_TRACE("round " + std::to_string(round) + " frame '" + frame + "'");
            send_twist(make_twist(0.04, -0.02, 0.03, 0.2, -0.1, 0.3, frame));
            const StepResult r = step(100);
            EXPECT_TRUE(r.ok);
            EXPECT_NO_RT_ALLOCATIONS(r);
            EXPECT_GT(max_abs_dq_cmd(), 1e-3);
        }
    }
}

TEST_F(CartesianJogRtAllocTest, ToolFrameTranslationMovesAlongToolAxis)
{
    start(regular_pose());
    const Eigen::Isometry3d T0 = tcp_pose();

    send_twist(make_twist(0.0, 0.0, 0.05, 0.0, 0.0, 0.0, tcp_frame_));
    EXPECT_TRUE(step(240).ok);
    EXPECT_TRUE(step(500).ok);    // watchdog zeroes the twist, ruckig brakes to rest

    const Eigen::Isometry3d T1 = tcp_pose();
    const Eigen::Vector3d dp = T1.translation() - T0.translation();
    const Eigen::Vector3d z_tool = T0.linear().col(2);
    ASSERT_GT(dp.norm(), 1e-3) << "arm did not move";
    // straight along +z of the tool, orientation unchanged
    EXPECT_GT(dp.dot(z_tool), 0.0);
    EXPECT_LT((dp - dp.dot(z_tool) * z_tool).norm(), 0.01 * dp.norm());
    EXPECT_LT(Eigen::AngleAxisd(T0.linear().transpose() * T1.linear()).angle(), 1e-3);
}

TEST_F(CartesianJogRtAllocTest, ToolFrameRotationTurnsAboutToolAxisAtTcp)
{
    start(regular_pose());
    const Eigen::Isometry3d T0 = tcp_pose();

    send_twist(make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.3, tcp_frame_));
    EXPECT_TRUE(step(240).ok);
    EXPECT_TRUE(step(500).ok);

    const Eigen::Isometry3d T1 = tcp_pose();
    // relative rotation in tool coordinates is a positive rotation about tool z
    const Eigen::AngleAxisd aa(T0.linear().transpose() * T1.linear());
    ASSERT_GT(aa.angle(), 0.01) << "tool did not rotate";
    EXPECT_GT(aa.axis().dot(Eigen::Vector3d::UnitZ()), 0.999);
    // the reference point of the twist is the TCP, so it stays in place
    EXPECT_LT((T1.translation() - T0.translation()).norm(), 1e-3);
}

TEST_F(CartesianJogRtAllocTest, FrameSwitchKeepsVelocityContinuous)
{
    // the same physical (pure translation) motion is commanded in the other frame at full speed. If
    // ruckig's state is transformed correctly, the switch is invisible in the joint velocities,
    // otherwise they jump by roughly the full jogging speed.
    start(regular_pose());
    const Eigen::Vector3d v_tool(0.03, 0.02, -0.04);
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

    send_twist(make_twist(v_tool, zero, tcp_frame_));
    EXPECT_TRUE(step(240).ok);      // reach steady state, within the watchdog timeout
    const Eigen::Matrix3d R = tcp_pose().linear();
    ASSERT_GT((R - Eigen::Matrix3d::Identity()).norm(), 0.1) << "tool frame too close to base for this test";

    // tool -> base
    Eigen::VectorXd dq_before = dq_cmd_vec();
    send_twist(make_twist(R * v_tool, zero, base_frame_));
    StepResult r = step(1);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    EXPECT_LT((dq_cmd_vec() - dq_before).norm(), 1e-3) << "joint velocity jumped on tool -> base";

    // base -> tool
    EXPECT_TRUE(step(100).ok);
    dq_before = dq_cmd_vec();
    send_twist(make_twist(v_tool, zero, tcp_frame_));
    r = step(1);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    EXPECT_LT((dq_cmd_vec() - dq_before).norm(), 1e-3) << "joint velocity jumped on base -> tool";
}

TEST_F(CartesianJogRtAllocTest, JogIntoJointLimitDoesNotAllocate)
{
    // axis2 starts inside the slow down zone of its upper limit (0.349 rad), then the arm is jogged
    // along each axis in both directions, so some of them drive axis2 into the limit
    std::vector<double> q0 = regular_pose();
    q0[1] = 0.30;
    start(q0);

    double q2_max = q0[1];
    for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {1.0, -1.0}) {
            double v[3] = {0.0, 0.0, 0.0};
            v[axis] = sign * 0.1;
            send_twist(make_twist(v[0], v[1], v[2], 0.0, 0.0, 0.0));
            const StepResult r = step(500);
            EXPECT_TRUE(r.ok);
            EXPECT_NO_RT_ALLOCATIONS(r) << " jogging axis " << axis << " sign " << sign;
            q2_max = std::max(q2_max, q_cmd(1));
        }
    }
    // sanity: the limit was approached, so position_limit_scale actually throttled the motion
    EXPECT_GT(q2_max, 0.349 - 0.02 - 0.01);
}

TEST_F(CartesianJogRtAllocTest, JointVelocityClippingDoesNotAllocate)
{
    // Close to the wrist singularity (axis5 = 0) the joint velocities from the damped inverse grow
    // quickly. With high cartesian limits ruckig cannot follow the pre-scaled target fast enough,
    // the joint space scaling in update() kicks in and logs "Joint space clipping" (throttled).
    // the throttle state is per call site and process wide: an earlier test that clipped (e.g. the
    // frame switch) suppresses the log for 1 s, wait it out so the marker below can be seen
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    std::vector<double> q0 = regular_pose();
    q0[4] = 0.15;
    start(q0, {
        rclcpp::Parameter("max_linear_velocity", 1.0),
        rclcpp::Parameter("max_linear_acceleration", 20.0),
        rclcpp::Parameter("max_linear_jerk", 500.0),
        rclcpp::Parameter("max_angular_velocity", 3.0),
        rclcpp::Parameter("max_angular_acceleration", 60.0),
        rclcpp::Parameter("max_angular_jerk", 1500.0),
    });

    LogMarkerCounter clipping("Joint space clipping");
    const TwistStamped cmds[] = {
        make_twist(0.0, 0.0, 0.0, 3.0, 0.0, 0.0),
        make_twist(0.0, 0.0, 0.0, 0.0, 3.0, 0.0),
        make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 3.0),
        make_twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    };
    for (const TwistStamped & t : cmds) {
        send_twist(t);
        const StepResult r = step(500);
        EXPECT_TRUE(r.ok);
        EXPECT_NO_RT_ALLOCATIONS(r);
        if (clipping.hits() > 0) {
            break;
        }
    }
    // sanity: the scenario really reached the clipping branch
    EXPECT_GT(clipping.hits(), 0) << "joint space clipping was never triggered, adjust the scenario";
}

// Distances of increment runs are checked to 1 %: x_ref advances by the twist the damped inverse
// realizes (J·J⁺·dx_ff). With jinv_method = svd all singular values in regular_pose() are above
// √2·λ, so nothing is damped and J·J⁺ = I. With ldlt and lambda = 0.01 it falls short of the step
// by ~0.5 % (with lambda = 0.001 below 1e-5 relative), the tolerance covers both.
constexpr double inc_tol = 0.01;

TEST_F(CartesianJogRtAllocTest, IncrementRunDoesNotAllocate)
{
    // one step: start of the run, position mode, corner pre-scaling, Finished and back to jog mode
    start(regular_pose());
    const Eigen::Isometry3d T0 = tcp_pose();

    send_increment(make_increment(0.001, 0.0, 0.0, 0.0, 0.0, 0.0));
    const StepResult r = step(1500);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);

    // 1 mm along base x, back at rest
    const Eigen::Vector3d dp = tcp_pose().translation() - T0.translation();
    EXPECT_NEAR(dp.x(), 0.001, inc_tol * 0.001);
    EXPECT_NEAR(dp.y(), 0.0, inc_tol * 0.001);
    EXPECT_NEAR(dp.z(), 0.0, inc_tol * 0.001);
    EXPECT_LT(max_abs_dq_cmd(), 1e-5);
}

TEST_F(CartesianJogRtAllocTest, IncrementsAddUpAndAxesOverlap)
{
    start(regular_pose());

    // three clicks on the same axis while the arm is still moving
    Eigen::Isometry3d T0 = tcp_pose();
    for (int click = 0; click < 3; ++click) {
        send_increment(make_increment(0.001, 0.0, 0.0, 0.0, 0.0, 0.0));
        const StepResult r = step(50);
        EXPECT_TRUE(r.ok);
        EXPECT_NO_RT_ALLOCATIONS(r);
    }
    StepResult r = step(1500);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    Eigen::Vector3d dp = tcp_pose().translation() - T0.translation();
    EXPECT_NEAR(dp.x(), 0.003, inc_tol * 0.003) << "steps did not add up";

    // a second axis while the first is moving: several corners in the pre-scaling
    T0 = tcp_pose();
    send_increment(make_increment(0.002, 0.0, 0.0, 0.0, 0.0, 0.0));
    EXPECT_TRUE(step(50).ok);
    send_increment(make_increment(0.0, -0.001, 0.0, 0.0, 0.0, 0.0));
    r = step(1500);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    dp = tcp_pose().translation() - T0.translation();
    EXPECT_NEAR(dp.x(), 0.002, inc_tol * 0.002);
    EXPECT_NEAR(dp.y(), -0.001, inc_tol * 0.002);
    EXPECT_NEAR(dp.z(), 0.0, inc_tol * 0.002);

    // rotation step in the tool frame: turns about tool z, the TCP stays in place
    T0 = tcp_pose();
    send_increment(make_increment(0.0, 0.0, 0.0, 0.0, 0.0, 0.05, tcp_frame_));
    r = step(1500);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    const Eigen::Isometry3d T1 = tcp_pose();
    const Eigen::AngleAxisd aa(T0.linear().transpose() * T1.linear());
    EXPECT_NEAR(aa.angle(), 0.05, inc_tol * 0.05);
    EXPECT_GT(aa.axis().dot(Eigen::Vector3d::UnitZ()), 0.999);
    // J·J⁺ also couples a little of the rotation into translation
    EXPECT_LT((T1.translation() - T0.translation()).norm(), 1e-4);
}

TEST_F(CartesianJogRtAllocTest, TwistEndsIncrementRunSmoothly)
{
    // a zero twist (stop) early in a run: velocity mode continues from the current state, so the
    // joint velocities stay smooth, and the arm brakes well before the end of the step.
    // The stop has to come while the run still speeds up (jerk > 0, first ~65 ms of a 3 mm step):
    // later, the rest of the planned profile already is the fastest stop and ends on the target.
    start(regular_pose());
    const Eigen::Isometry3d T0 = tcp_pose();

    send_increment(make_increment(0.003, 0.0, 0.0, 0.0, 0.0, 0.0));
    EXPECT_TRUE(step(29).ok);
    const Eigen::VectorXd dq_0 = dq_cmd_vec();
    EXPECT_TRUE(step(1).ok);
    const Eigen::VectorXd dq_1 = dq_cmd_vec();
    ASSERT_GT(max_abs_dq_cmd(), 1e-4) << "arm not moving when the stop arrives";

    send_twist(make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    StepResult r = step(1);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    // the change per cycle stays the same (no velocity jump): only the jerk flips, which changes
    // the velocity step by ~J⁺·2·j·dt², far below a jump of the velocity itself
    const Eigen::VectorXd change_before = dq_1 - dq_0;
    const Eigen::VectorXd change_after = dq_cmd_vec() - dq_1;
    EXPECT_LT((change_after - change_before).norm(), 1e-4) << "joint velocity jumped when the run ended";

    r = step(1500);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    const double dx = tcp_pose().translation().x() - T0.translation().x();
    EXPECT_GT(dx, 0.0001);
    EXPECT_LT(dx, 0.002) << "run was not ended by the twist";
    EXPECT_LT(max_abs_dq_cmd(), 1e-5);
}

TEST_F(CartesianJogRtAllocTest, IncrementIntoJointLimitAborts)
{
    // axis2 starts at its stop position in front of the upper limit (0.349 rad - 0.02 margin), so
    // every step that drives axis2 up is aborted by the pre-scaling, the others run normally
    std::vector<double> q0 = regular_pose();
    q0[1] = 0.3291;
    start(q0);

    LogMarkerCounter aborted("Increment aborted");     // one instance at a time, its state is static
    for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {1.0, -1.0}) {
            double d[3] = {0.0, 0.0, 0.0};
            d[axis] = sign * 0.003;
            send_increment(make_increment(d[0], d[1], d[2], 0.0, 0.0, 0.0));
            const StepResult r = step(800);
            EXPECT_TRUE(r.ok) << " step along axis " << axis << " sign " << sign;
            EXPECT_NO_RT_ALLOCATIONS(r) << " step along axis " << axis << " sign " << sign;
            EXPECT_LT(q_cmd(1), 0.3491) << "axis2 passed its upper limit";
        }
    }
    EXPECT_GT(aborted.hits(), 0) << "no step was aborted, adjust the scenario";
}

}  // namespace

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    LogAllocationExclusion::install();
    malloc_counter::prime_backtrace();
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
