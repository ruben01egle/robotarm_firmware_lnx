#include "moteus_interface/MoteusInterface.hpp"

namespace moteus_interface
{

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    return hardware_interface::CallbackReturn();
}
std::vector<hardware_interface::StateInterface> moteus_interface::MoteusInterface::export_state_interfaces()
{
    return std::vector<hardware_interface::StateInterface>();
}

std::vector<hardware_interface::CommandInterface> moteus_interface::MoteusInterface::export_command_interfaces()
{
    return std::vector<hardware_interface::CommandInterface>();
}

hardware_interface::return_type moteus_interface::MoteusInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    return hardware_interface::return_type();
}

hardware_interface::return_type moteus_interface::MoteusInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    return hardware_interface::return_type();
}

hardware_interface::return_type moteus_interface::MoteusInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    return hardware_interface::return_type();
}

hardware_interface::return_type moteus_interface::MoteusInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    return hardware_interface::return_type();
}

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn();
}

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn();
}

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn();
}

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn();
}

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn();
}

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_error(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn();
}

}