#ifndef ROBOTARM_CONTROLLERS_HOLDCONTROLLER_HPP
#define ROBOTARM_CONTROLLERS_HOLDCONTROLLER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/controller_interface.hpp"

#include <string>
#include <vector>

namespace hold_controller {

// Upstream of a chainable controller (ff_controller, cartesian_impedance_controller) that holds the
// pose measured at activation: position = latched q, velocity = acceleration = 0, every cycle.
// Keeps the downstream controller in chained mode while no motion controller is active, so the
// downstream is never restarted when the motion controllers are swapped.
class HoldController : public controller_interface::ControllerInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(HoldController)

    HoldController() = default;
    virtual ~HoldController() = default;

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
    std::vector<std::string> joint_names_;
    std::string chain_to_;
    std::vector<double> q_hold_;    // measured at activation, sized at configure

    static constexpr size_t num_cmd_interfaces_per_joint_ = 3;
};

}

#endif
