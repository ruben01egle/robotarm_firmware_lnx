#include "moteus_interface/MoteusInterface.hpp"

namespace moteus_interface
{

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    size_t num_joints = info_.joints.size();
    if (num_joints == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MoteusInterface"), "No joints found in URDF!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    hw_commands_position_.assign(num_joints, 0.0);
    hw_commands_velocity_.assign(num_joints, 0.0);
    hw_commands_effort_.assign(num_joints, 0.0);
    
    hw_states_position_.assign(num_joints, 0.0);
    hw_states_velocity_.assign(num_joints, 0.0);
    hw_states_effort_.assign(num_joints, 0.0);

    can_ids_.resize(num_joints);
    controllers_.resize(num_joints);
    command_frames_.resize(num_joints);
    replies_frames_.reserve(num_joints);            // reserve instead of resize for clear/pushback in transport cycle
    joint_updated_.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i)
    {
        if (!check_joint_interface(info_.joints[i])) return hardware_interface::CallbackReturn::ERROR;

        auto param_it = info_.joints[i].parameters.find("can_id");
        if (param_it != info_.joints[i].parameters.end())
        {
            can_ids_[i] = std::stoi(param_it->second);
            RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), 
                        "Joint %s using CAN-ID: %d registered", info_.joints[i].name.c_str(), can_ids_[i]);
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Joint %s missing 'can_id' parameter!", info_.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> moteus_interface::MoteusInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_states_position_[i]
        ));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &hw_states_velocity_[i]
        ));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &hw_states_effort_[i]
        ));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> moteus_interface::MoteusInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_commands_position_[i]
        ));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &hw_commands_velocity_[i]
        ));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &hw_commands_effort_[i]
        ));
    }
    return command_interfaces;
}

hardware_interface::return_type moteus_interface::MoteusInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type moteus_interface::MoteusInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type moteus_interface::MoteusInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    using namespace mjbots;
    size_t num_joints = can_ids_.size();
    std::fill(joint_updated_.begin(), joint_updated_.end(), false);

    for (const auto& frame : replies_frames_)
    {
        auto it = std::find(can_ids_.begin(), can_ids_.end(), frame.source);
        if (it != can_ids_.end()) 
        {
            size_t joint_index = std::distance(can_ids_.begin(), it);
            
            auto layout = mjbots::moteus::Query::Parse(frame.data, frame.size);

            hw_states_position_[joint_index] = layout.position * 2.0 * M_PI;
            hw_states_velocity_[joint_index] = layout.velocity * 2.0 * M_PI;
            hw_states_effort_[joint_index]   = layout.torque;

            // TODO: error abfragen von motor und belt slips erkennen

            joint_updated_[joint_index] = true;
        }
    }

    hardware_interface::return_type ret = hardware_interface::return_type::OK;
    for (size_t i = 0; i < num_joints; ++i) {
        if (!joint_updated_[i]) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                         "Axis %zu (CAN-ID: %d) did not respond!", i, can_ids_[i]);
            ret = hardware_interface::return_type::ERROR;
        }
    }
    return ret;
}

hardware_interface::return_type moteus_interface::MoteusInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    using namespace mjbots;
    const size_t num_joints = can_ids_.size();

    if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        for (size_t i = 0; i < num_joints; ++i)
        {
            moteus::PositionMode::Command cmd;
            cmd.position = hw_commands_position_[i] / (2.0 * M_PI);
            cmd.velocity = hw_commands_velocity_[i] / (2.0 * M_PI);
            cmd.feedforward_torque = hw_commands_effort_[i];

            command_frames_[i] = controllers_[i]->MakePosition(cmd);
        }
    }
    else
    {
        for (size_t i = 0; i < num_joints; ++i)
        {
            command_frames_[i] = controllers_[i]->MakeStop();
        }
    }
    // Blocking at end of each control loop to ensure data coherency and avoid race conditions
    transport_->BlockingCycle(&command_frames_[0], command_frames_.size(), &replies_frames_);

    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn moteus_interface::MoteusInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Configuring moteus interface...");

    try {
        using namespace mjbots;
        transport_ = moteus::Controller::MakeSingletonTransport({});
        if (!transport_)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                         "Moteus-Transport not initialized!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        mjbots::moteus::PositionMode::Format write_format;
        write_format.position = mjbots::moteus::kFloat;
        write_format.velocity = mjbots::moteus::kFloat;
        write_format.feedforward_torque = mjbots::moteus::kFloat;

        mjbots::moteus::Query::ItemFormat encoder1;                 // extra field for secondary encoder
        encoder1.register_number = mjbots::moteus::Register::kEncoder1Position;
        encoder1.resolution = mjbots::moteus::Resolution::kFloat;
        mjbots::moteus::Query::Format read_format;
        read_format.position           = mjbots::moteus::Resolution::kInt16;
        read_format.velocity           = mjbots::moteus::Resolution::kInt16;
        read_format.torque             = mjbots::moteus::Resolution::kInt16;
        read_format.fault              = mjbots::moteus::Resolution::kInt8;
        read_format.extra[0]           = encoder1;

        size_t num_joints = can_ids_.size();
        for (size_t i = 0; i < num_joints; ++i)
        {
            mjbots::moteus::Controller::Options options;
            options.id = can_ids_[i];
            options.query_format = read_format;
            options.position_format = write_format;
            controllers_[i] = std::make_shared<mjbots::moteus::Controller>(options);
            command_frames_[i] = controllers_[i]->MakeStop();
        }
        transport_->BlockingCycle(&command_frames_[0], command_frames_.size(), &replies_frames_);

        RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), 
                    "Moteus-Interface configured: %zu controller ready.", num_joints);

    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                     "Exception occured during configuration: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
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

bool MoteusInterface::check_joint_interface(hardware_interface::ComponentInfo joint)
{
    if (joint.command_interfaces.size() != 3){
        RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                    "Joint %s needs 3 Command-Interfaces!", joint.name.c_str());
        return false;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
        joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
        RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                    "Joint %s invalid command interface! Expected: position, velocity, effort.", joint.name.c_str());
        return false;
    }

    if (joint.state_interfaces.size() != 3){
        RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                    "Joint %s needs 3 state-Interfaces!", joint.name.c_str());
        return false;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
        joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
        RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                    "Joint %s invalid state interface! Expected: position, velocity, effort.", joint.name.c_str());
        return false;
    }
    return true;
}

}