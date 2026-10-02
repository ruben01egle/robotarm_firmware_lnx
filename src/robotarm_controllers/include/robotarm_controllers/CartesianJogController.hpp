#ifndef ROBOTARM_CONTROLLERS_CARTESIANJOGCONTROLLER_HPP
#define ROBOTARM_CONTROLLERS_CARTESIANJOGCONTROLLER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/subscription.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "controller_interface/controller_interface.hpp"

#include <string>
#include <vector>
#include <memory>
#include <ruckig/ruckig.hpp>

#include "robotarm_rbd/RobotarmRbd.hpp"

namespace cartesian_jog_controller
{

using CmdTypeTwist = geometry_msgs::msg::Twist;

class CartesianJogController : public controller_interface::ControllerInterface
{
public:
    struct CartesianLimits
    {
        struct Bounds { double velocity, acceleration, jerk; };
        Bounds linear;   // m/s, m/s², m/s³
        Bounds angular;  // rad/s, rad/s², rad/s³
    };

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CartesianJogController)

    CartesianJogController() = default;
    virtual ~CartesianJogController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    void declare_parameters();
    controller_interface::CallbackReturn read_parameters();

private:
    // custom rigid body dynamics class
    robotarm_rbd::RobotarmRbd rbd_;
    robotarm_rbd::RobotarmRbd::Config rbd_cfg_;
    std::vector<robotarm_rbd::RobotarmRbd::Limits> rbd_limits_;
    std::vector<std::string> rbd_joint_names_;

    // cartesian space ruckig
    std::unique_ptr<ruckig::Ruckig<6>> ruckig_;
    std::unique_ptr<ruckig::InputParameter<6>> ruckig_input_;
    std::unique_ptr<ruckig::OutputParameter<6>> ruckig_output_;

    CartesianLimits cartesian_limits_;

    rclcpp::Subscription<CmdTypeTwist>::SharedPtr twist_cmd_subscriber_;

    // the realtime container to exchange the reference from subscriber
    realtime_tools::RealtimeThreadSafeBox<CmdTypeTwist> rt_command_twist_;
    CmdTypeTwist twist_cmd_;

    static constexpr size_t num_states_per_joint_ = 2;
};

}

#endif