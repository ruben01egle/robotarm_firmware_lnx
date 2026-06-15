#ifndef TELEOPCONTROLLER_HPP
#define TELEOPCONTROLLER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "robotarm_interface/srv/set_float64.hpp"
#include <ruckig/ruckig.hpp>

#include <string>
#include <vector>
#include <memory>
#include <atomic>

namespace teleop_controller {

using CmdType = std_msgs::msg::Float64MultiArray;

class TeleopController : public controller_interface::ControllerInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(TeleopController)

    TeleopController();
    virtual ~TeleopController() = default;

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
    void handle_scale_speed(
        const std::shared_ptr<robotarm_interface::srv::SetFloat64::Request> request,
        std::shared_ptr<robotarm_interface::srv::SetFloat64::Response> response);

private:
    std::vector<std::string> joint_names_;
    std::string interface_name_;

    std::vector<std::string> command_interface_types_;

    std::unique_ptr<ruckig::Ruckig<ruckig::DynamicDOFs>> ruckig_;
    std::unique_ptr<ruckig::InputParameter<ruckig::DynamicDOFs>> ruckig_input_;
    std::unique_ptr<ruckig::OutputParameter<ruckig::DynamicDOFs>> ruckig_output_;

    bool update_reference_;

    rclcpp::Service<robotarm_interface::srv::SetFloat64>::SharedPtr scale_speed_service_;
    std::atomic<double> speed_scale_{1.0};
    double max_velocity_;
    double max_acceleration_;
    double max_jerk_;

    // the realtime container to exchange the reference from subscriber
    realtime_tools::RealtimeThreadSafeBox<CmdType> rt_command_;
    // save the last reference in case of unable to get value from box
    CmdType joint_commands_;

    rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

    static constexpr size_t num_states_per_joint_ = 2;
};

}

#endif