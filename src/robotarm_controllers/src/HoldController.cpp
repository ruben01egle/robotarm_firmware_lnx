#include "robotarm_controllers/HoldController.hpp"

#include <cmath>

PLUGINLIB_EXPORT_CLASS(
  hold_controller::HoldController,
  controller_interface::ControllerInterface
)

namespace hold_controller
{

controller_interface::InterfaceConfiguration HoldController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_)
    {
        command_interfaces_config.names.push_back(chain_to_ + "/" + joint + "/position");
        command_interfaces_config.names.push_back(chain_to_ + "/" + joint + "/velocity");
        command_interfaces_config.names.push_back(chain_to_ + "/" + joint + "/acceleration");
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration HoldController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint : joint_names_) {
        state_interfaces_config.names.push_back(joint + "/position");
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn HoldController::on_init()
{
    try
    {
        auto node = get_node();
        node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
        node->declare_parameter<std::string>("chain_to", "");
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HoldController::on_configure(const rclcpp_lifecycle::State &/*previous_state*/)
{
    auto node = get_node();

    joint_names_ = node->get_parameter("joints").as_string_array();
    if (joint_names_.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "The 'joints' parameter list cannot be empty!");
        return controller_interface::CallbackReturn::FAILURE;
    }

    // only useful chained: standalone there would be nothing to keep in chained mode
    chain_to_ = node->get_parameter("chain_to").as_string();
    if (chain_to_.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "The 'chain_to' parameter cannot be empty!");
        return controller_interface::CallbackReturn::FAILURE;
    }

    q_hold_.assign(joint_names_.size(), 0.0);

    RCLCPP_INFO(node->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HoldController::on_activate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    // hold where the arm is, not where a previous upstream wanted it to be
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        auto pos_opt = state_interfaces_[i].get_optional();
        if (!pos_opt.has_value() || !std::isfinite(pos_opt.value()))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not read position of %s", joint_names_[i].c_str());
            return controller_interface::CallbackReturn::ERROR;   // refuse to activate rather than command garbage
        }
        q_hold_[i] = pos_opt.value();
    }

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HoldController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HoldController::update(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/)
{
    // written every cycle, so the downstream never sees a reference left behind by a previous upstream
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        (void)command_interfaces_[i * num_cmd_interfaces_per_joint_ + 0].set_value(q_hold_[i]);
        (void)command_interfaces_[i * num_cmd_interfaces_per_joint_ + 1].set_value(0.0);
        (void)command_interfaces_[i * num_cmd_interfaces_per_joint_ + 2].set_value(0.0);
    }

    return controller_interface::return_type::OK;
}

}
