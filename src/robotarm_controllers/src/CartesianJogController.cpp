#include "robotarm_controllers/CartesianJogController.hpp"

#include <Eigen/Geometry>

PLUGINLIB_EXPORT_CLASS(
  cartesian_jog_controller::CartesianJogController, 
  controller_interface::ControllerInterface
)

namespace
{  // utility

void reset_controller_reference_msg(cartesian_jog_controller::CmdTypeTwist & msg)
{
    msg = cartesian_jog_controller::CmdTypeTwist();
}

bool is_finite(const geometry_msgs::msg::Vector3 & v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

namespace cartesian_jog_controller
{

controller_interface::InterfaceConfiguration CartesianJogController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : rbd_joint_names_)
    {
        command_interfaces_config.names.push_back(joint + "/position");
        command_interfaces_config.names.push_back(joint + "/velocity");
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CartesianJogController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    for (const auto & joint : rbd_joint_names_) {
        state_interfaces_config.names.push_back(joint + "/position");
        state_interfaces_config.names.push_back(joint + "/velocity");
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn CartesianJogController::on_init()
{
    try
    {
        declare_parameters();
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianJogController::on_configure(const rclcpp_lifecycle::State &/*previous_state*/)
{
    auto ret = read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    std::string urdf_string = get_robot_description();

    if (!rbd_.initialize(urdf_string, rbd_cfg_)) {
        fprintf(stderr, "rbd failed to init: %s \n", rbd_.last_error());
    }
    if (!rbd_.get_joint_names(rbd_joint_names_)) {
        fprintf(stderr, "rbd not properly initialized: %s \n", rbd_.last_error());
    }
    if (!rbd_.get_joint_limits(rbd_limits_)) {
        fprintf(stderr, "rbd not properly initialized: %s \n", rbd_.last_error());
    }

    twist_cmd_subscriber_ = get_node()->create_subscription<CmdTypeTwist>(
        "~/twist_cmds", rclcpp::SystemDefaultsQoS(),
        [this](const CmdTypeTwist::SharedPtr msg)
        {

        if (!is_finite(msg->linear) || !is_finite(msg->angular)) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                "Non-finite value received. Dropping message");
            return;
        }
        rt_command_twist_.set(*msg);
        });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianJogController::on_activate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    // make sure hw is in defined state when claiming interfaces
    for (size_t i = 0; i < rbd_joint_names_.size(); ++i)
    {
        size_t pos_idx = i * num_states_per_joint_ + 0;
        size_t vel_idx = i * num_states_per_joint_ + 1;

        auto pos_opt = state_interfaces_[pos_idx].get_optional();
        if (!pos_opt.has_value())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not read position of %s", rbd_joint_names_[i].c_str());
            return controller_interface::CallbackReturn::ERROR;   // refuse to activate rather than command garbage
        }
        (void)command_interfaces_[pos_idx].set_value(pos_opt.value());
        (void)command_interfaces_[vel_idx].set_value(0.0);
    }

    reset_controller_reference_msg(twist_cmd_);
    rt_command_twist_.try_set(twist_cmd_);

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianJogController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianJogController::update(const rclcpp::Time &/*time*/, const rclcpp::Duration &period)
{
    return controller_interface::return_type::OK;
}

void CartesianJogController::declare_parameters()
{
    auto node = get_node();

    node->declare_parameter<double>("lambda", 0.01);

    node->declare_parameter<double>("max_linear_velocity", 0.1);       // in m/s
    node->declare_parameter<double>("max_linear_acceleration", 0.5);   // in m/s²
    node->declare_parameter<double>("max_linear_jerk", 5.0);           // in m/s³

    node->declare_parameter<double>("max_angular_velocity", 0.5);       // in rad/s
    node->declare_parameter<double>("max_angular_acceleration", 2.0);   // in rad/s²
    node->declare_parameter<double>("max_angular_jerk", 20.0);           // in rad/s³
}

controller_interface::CallbackReturn CartesianJogController::read_parameters()
{
    auto node = get_node();
    rbd_cfg_.lambda = node->get_parameter("lambda").as_double();

    cartesian_limits_.linear.velocity = node->get_parameter("max_linear_velocity").as_double();
    cartesian_limits_.linear.acceleration = node->get_parameter("max_linear_acceleration").as_double();
    cartesian_limits_.linear.jerk = node->get_parameter("max_linear_jerk").as_double();

    cartesian_limits_.angular.velocity = node->get_parameter("max_angular_velocity").as_double();
    cartesian_limits_.angular.acceleration = node->get_parameter("max_angular_acceleration").as_double();
    cartesian_limits_.angular.jerk = node->get_parameter("max_angular_jerk").as_double();

    return controller_interface::CallbackReturn::SUCCESS;
}
}