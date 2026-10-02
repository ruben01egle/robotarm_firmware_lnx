#include "robotarm_controllers/TeleopController.hpp"

PLUGINLIB_EXPORT_CLASS(
  teleop_controller::TeleopController, 
  controller_interface::ControllerInterface
)

namespace
{  // utility

// called from RT control loop
void reset_controller_reference_msg(teleop_controller::CmdType & msg)
{
  for (auto & data : msg.data)
  {
    data = std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace

namespace teleop_controller
{

TeleopController::TeleopController()
: controller_interface::ControllerInterface(), joints_command_subscriber_(nullptr)
{
    update_reference_ = true;
}

controller_interface::InterfaceConfiguration TeleopController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_)
    {
        command_interfaces_config.names.push_back(joint + "/position");
        command_interfaces_config.names.push_back(joint + "/velocity");
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration TeleopController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    for (const auto & joint : joint_names_) {
        state_interfaces_config.names.push_back(joint + "/position");
        state_interfaces_config.names.push_back(joint + "/velocity");
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn TeleopController::on_init()
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

controller_interface::CallbackReturn TeleopController::on_configure(const rclcpp_lifecycle::State &/*previous_state*/)
{
    auto ret = read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
        "~/commands", rclcpp::SystemDefaultsQoS(),
        [this](const CmdType::SharedPtr msg)
        {

        if (!std::all_of(
                msg->data.cbegin(), msg->data.cend(),
                [](const auto & value) { return std::isfinite(value); }))
        {
            RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *(get_node()->get_clock()), 1000,
            "Non-finite value received. Dropping message");
            return;
        }
        rt_command_.set(*msg);
        });

    scale_speed_service_ = get_node()->create_service<robotarm_interface::srv::SetFloat64>(
        "~/scale_speed",
        std::bind(&TeleopController::handle_scale_speed, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TeleopController::on_activate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    // make sure hw is in defined state when claiming interfaces
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        size_t pos_idx = i * num_states_per_joint_ + 0;
        size_t vel_idx = i * num_states_per_joint_ + 1;

        auto pos_opt = state_interfaces_[pos_idx].get_optional();
        if (!pos_opt.has_value())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not read position of %s", joint_names_[i].c_str());
            return controller_interface::CallbackReturn::ERROR;   // refuse to activate rather than command garbage
        }
        (void)command_interfaces_[pos_idx].set_value(pos_opt.value());
        (void)command_interfaces_[vel_idx].set_value(0.0);
    }

    reset_controller_reference_msg(joint_commands_);
    update_reference_ = true;
    rt_command_.try_set(joint_commands_);

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TeleopController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TeleopController::update(const rclcpp::Time &/*time*/, const rclcpp::Duration &period)
{
    rt_command_.try_get([this](const CmdType & msg){joint_commands_.data = msg.data;});

    // no command received yet
    if (
        std::all_of(
        joint_commands_.data.cbegin(), joint_commands_.data.cend(),
        [](const auto & value) { return std::isnan(value); }))
    {
        return controller_interface::return_type::OK;
    }

    if (joint_commands_.data.size() != joint_names_.size())
    {
        RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "command size (%zu) does not match number of joints (%zu)", joint_commands_.data.size(),
        joint_names_.size());
        return controller_interface::return_type::ERROR;
    }

    if (!update_reference_)
    {
        ruckig_output_->pass_to_input(*ruckig_input_);
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
        size_t pos_state_idx = i * num_states_per_joint_ + 0;
        size_t vel_state_idx = i * num_states_per_joint_ + 1;

        if (update_reference_) 
        {
            auto pos_opt = state_interfaces_[pos_state_idx].get_optional();
            auto vel_opt = state_interfaces_[vel_state_idx].get_optional();

            if (!pos_opt.has_value() || !vel_opt.has_value())
            {
                RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                    "Hardware-State for joint %s could not be read", joint_names_[i].c_str());
                return controller_interface::return_type::ERROR;
            }

            ruckig_input_->current_position[i] = pos_opt.value();
            ruckig_input_->current_velocity[i] = vel_opt.value();
            ruckig_input_->current_acceleration[i] = 0;
        }

        ruckig_input_->target_position[i] = joint_commands_.data[i];
        ruckig_input_->target_velocity[i] = 0.0;

        double speed_scale = speed_scale_.load();
        ruckig_input_->max_velocity[i] = max_velocity_ * speed_scale;
        ruckig_input_->max_acceleration[i] = max_acceleration_ * speed_scale * speed_scale;
        ruckig_input_->max_jerk[i] = max_jerk_ * speed_scale * speed_scale * speed_scale;
    }

    update_reference_ = false;

    ruckig_->delta_time = period.seconds();
    auto result = ruckig_->update(*ruckig_input_, *ruckig_output_);
    if (result == ruckig::Result::ErrorInvalidInput)
    {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000, 
                            "Ruckig: invalid input!");
        return controller_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        size_t pos_state_idx = i * num_states_per_joint_ + 0;
        size_t vel_state_idx = i * num_states_per_joint_ + 1;

        (void)command_interfaces_[pos_state_idx].set_value(ruckig_output_->new_position[i]);
        (void)command_interfaces_[vel_state_idx].set_value(ruckig_output_->new_velocity[i]);
    }

    return controller_interface::return_type::OK;
}

void TeleopController::declare_parameters()
{
    auto node = get_node();

    node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());

    node->declare_parameter<double>("max_velocity", 1.5);       // in rad/s
    node->declare_parameter<double>("max_acceleration", 2.0);   // in rad/s²
    node->declare_parameter<double>("max_jerk", 2.0);           // in rad/s³

    node->declare_parameter<bool>("sync_joints", false);
}

controller_interface::CallbackReturn TeleopController::read_parameters()
{
    auto node = get_node();

    // Read parameters from YAML
    joint_names_ = node->get_parameter("joints").as_string_array();

    // Validate that lists are not empty
    if (joint_names_.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "The 'joints' parameter list cannot be empty!");
        return controller_interface::CallbackReturn::FAILURE;
    }

    size_t dofs = joint_names_.size();
    try 
    {
        max_velocity_ = node->get_parameter("max_velocity").as_double();
        max_acceleration_ = node->get_parameter("max_acceleration").as_double();
        max_jerk_ = node->get_parameter("max_jerk").as_double();

        ruckig_ = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(dofs);
        ruckig_input_ = std::make_unique<ruckig::InputParameter<ruckig::DynamicDOFs>>(dofs);
        ruckig_output_ = std::make_unique<ruckig::OutputParameter<ruckig::DynamicDOFs>>(dofs);

        for (size_t i = 0; i < dofs; ++i)
        {

            ruckig_input_->max_velocity[i] = max_velocity_;
            ruckig_input_->max_acceleration[i] = max_acceleration_;
            ruckig_input_->max_jerk[i] = max_jerk_;
        }
        bool sync_joints = node->get_parameter("sync_joints").as_bool();
        if (sync_joints) {
            ruckig_input_->synchronization = ruckig::Synchronization::Time;
            RCLCPP_INFO(get_node()->get_logger(), "Teleop controller sync joints: ON");
        }
        else {
            ruckig_input_->synchronization = ruckig::Synchronization::None;
            RCLCPP_INFO(get_node()->get_logger(), "Teleop controller sync joints: OFF");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error initialising ruckig object: %s", e.what());
        return controller_interface::CallbackReturn::FAILURE;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

void TeleopController::handle_scale_speed(
    const std::shared_ptr<robotarm_interface::srv::SetFloat64::Request> request,
    std::shared_ptr<robotarm_interface::srv::SetFloat64::Response> response)
{
    speed_scale_.store(request->data);
    response->success = true;
}

}