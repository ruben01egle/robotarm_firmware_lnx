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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/controller_interface_params.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

#include "robotarm_controllers/CartesianJogController.hpp"

#include "malloc_counter.hpp"

namespace
{

using cartesian_jog_controller::CartesianJogController;
using geometry_msgs::msg::Twist;
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

Twist make_twist(double vx, double vy, double vz, double wx, double wy, double wz)
{
    Twist t;
    t.linear.x = vx;
    t.linear.y = vy;
    t.linear.z = vz;
    t.angular.x = wx;
    t.angular.y = wy;
    t.angular.z = wz;
    return t;
}

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

        pub_node_ = std::make_shared<rclcpp::Node>("twist_publisher");
        const std::string topic = std::string("/") + controller_name + "/twist_cmds";
        twist_pub_ = pub_node_->create_publisher<Twist>(topic, rclcpp::SystemDefaultsQoS());
        echo_sub_ = pub_node_->create_subscription<Twist>(topic, rclcpp::SystemDefaultsQoS(),
                                                          [this](Twist::ConstSharedPtr) {++echo_count_;});
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(controller_->get_node()->get_node_base_interface());
        executor_->add_node(pub_node_);
    }

    // publishes over the real subscription and spins until it is delivered, outside of any
    // measured window
    void send_twist(const Twist & t)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (twist_pub_->get_subscription_count() < 2) {   // controller + echo
            ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "twist subscription not matched";
            executor_->spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const int expected = echo_count_ + 1;
        twist_pub_->publish(t);
        // the echo subscriber gets the same delivery as the controller, spin a bit more after it
        // arrived so the controller's callback has run too
        while (echo_count_ < expected) {
            ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "twist not delivered";
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

    rclcpp::Node::SharedPtr pub_node_;
    rclcpp::Publisher<Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<Twist>::SharedPtr echo_sub_;
    int echo_count_ = 0;
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
            (void)p;
        }), 0u);

    // and nothing is counted while disarmed
    EXPECT_EQ(malloc_counter::count_allocations([] {
            double x = 1.0;
            (void)x;
        }), 0u);
    std::vector<int> outside(16);
    EXPECT_EQ(malloc_counter::count(), 0u);
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
    const StepResult r = step(2000);
    EXPECT_TRUE(r.ok);
    EXPECT_NO_RT_ALLOCATIONS(r);
    // sanity: the full ruckig / IK path ran
    EXPECT_GT(max_abs_dq_cmd(), 1e-3);
}

TEST_F(CartesianJogRtAllocTest, NewCommandPickupDoesNotAllocate)
{
    // a fresh message in the realtime box is copied out by try_get() inside update()
    start(regular_pose());
    const Twist cmds[] = {
        make_twist(0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
        make_twist(0.0, 0.05, 0.0, 0.0, 0.0, 0.2),
        make_twist(-0.05, 0.0, 0.03, 0.2, 0.0, 0.0),
        make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    };
    for (int round = 0; round < 3; ++round) {
        for (const Twist & t : cmds) {
            SCOPED_TRACE("round " + std::to_string(round));
            send_twist(t);
            const StepResult r = step(200);
            EXPECT_TRUE(r.ok);
            EXPECT_NO_RT_ALLOCATIONS(r);
        }
    }
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
    const Twist cmds[] = {
        make_twist(0.0, 0.0, 0.0, 3.0, 0.0, 0.0),
        make_twist(0.0, 0.0, 0.0, 0.0, 3.0, 0.0),
        make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 3.0),
        make_twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    };
    for (const Twist & t : cmds) {
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

}  // namespace

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    malloc_counter::prime_backtrace();
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
