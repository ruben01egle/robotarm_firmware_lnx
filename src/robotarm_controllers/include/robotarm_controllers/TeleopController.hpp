#ifndef TELEOPCONTROLLER_HPP
#define TELEOPCONTROLLER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/controller_interface.hpp"

namespace teleop_controller {

class TeleopController : public controller_interface::ControllerInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(TeleopController)

    TeleopController() = default;
    virtual ~TeleopController() = default;
};

}

PLUGINLIB_EXPORT_CLASS(
  teleop_controller::TeleopController, 
  controller_interface::ControllerInterface
)

#endif