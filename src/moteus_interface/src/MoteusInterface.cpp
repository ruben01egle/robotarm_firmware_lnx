#include "moteus_interface/MoteusInterface.hpp"

PLUGINLIB_EXPORT_CLASS(
  moteus_interface::MoteusInterface, 
  hardware_interface::SystemInterface
)

namespace moteus_interface
{

hardware_interface::CallbackReturn MoteusInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    is_active_ = false;

    size_t num_joints = info_.joints.size();
    if (num_joints == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MoteusInterface"), "No joints found in URDF!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    hw_commands_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocity_.assign(num_joints, 0.0);
    hw_commands_effort_.assign(num_joints, 0.0);
    
    hw_states_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.assign(num_joints, 0.0);
    hw_states_effort_.assign(num_joints, 0.0);

    joints_.resize(num_joints);
    joint_updated_.resize(num_joints);
    command_frames_.resize(num_joints);
    replies_frames_.reserve(num_joints);            // reserve instead of resize for clear/pushback in transport cycle
    joint_results_.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i)
    {
        if (!check_joint_interface(info_.joints[i])) return hardware_interface::CallbackReturn::ERROR;

        joints_[i].name = info_.joints[i].name;

        auto param_it_can = info_.joints[i].parameters.find("can_id");
        if (param_it_can != info_.joints[i].parameters.end())
        {
            joints_[i].can_id = std::stoi(param_it_can->second);
            RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), 
                        "Joint %s using CAN-ID: %d registered", info_.joints[i].name.c_str(), joints_[i].can_id);
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Joint %s missing 'can_id' parameter!", info_.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        auto param_it_enc = info_.joints[i].parameters.find("encoder_offset");
        if (param_it_enc != info_.joints[i].parameters.end())
        {
            joints_[i].encoder_offset = std::stod(param_it_enc->second);
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Joint %s missing 'encoder_offset' parameter!", info_.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        auto param_it_gear = info_.joints[i].parameters.find("gear_ratio");
        if (param_it_gear != info_.joints[i].parameters.end())
        {
            joints_[i].gear_ratio = std::stod(param_it_gear->second);
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Joint %s missing 'gear_ratio' parameter!", info_.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MoteusInterface::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> MoteusInterface::export_command_interfaces()
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

hardware_interface::return_type MoteusInterface::prepare_command_mode_switch(const std::vector<std::string> &/*start_interfaces*/, const std::vector<std::string> &/*stop_interfaces*/)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteusInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    for (size_t i = 0; i < joints_.size(); ++i) {
        
        for (const auto& interface : start_interfaces) {
            if (interface == joints_[i].name + "/position") joints_[i].pos_active = true;
            if (interface == joints_[i].name + "/velocity") joints_[i].vel_active = true;
            if (interface == joints_[i].name + "/effort")   joints_[i].effort_active = true;
        }

        for (const auto& interface : stop_interfaces) {
            if (interface == joints_[i].name + "/position") joints_[i].pos_active = false;
            if (interface == joints_[i].name + "/velocity") joints_[i].vel_active = false;
            if (interface == joints_[i].name + "/effort")   joints_[i].effort_active = false;
        }
    }

    for (const auto& joint : joints_) {
        RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"),
            "Joint [%s] active interfaces: Pos=%s, Vel=%s, Eff=%s",
            joint.name.c_str(),
            joint.pos_active ? "ON" : "OFF",
            joint.vel_active ? "ON" : "OFF",
            joint.effort_active ? "ON" : "OFF");
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteusInterface::read(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/)
{
    using namespace mjbots;
    const size_t num_joints = joints_.size();

    // In inactive only read is called -> read needs query new data
    // In active this is done by write() paired with the cmds to ensure only one write cycle per loop at all times 
    if (!is_active_)
    {
        for (size_t i = 0; i < num_joints; ++i)
        {
            command_frames_[i] = joints_[i].controller->MakeStop();
        }
        // Blocking at end of each control loop to ensure data coherency and avoid race conditions
        transport_->BlockingCycle(&command_frames_[0], command_frames_.size(), &replies_frames_);
    }
    
    if (!parse_result_frames()) return hardware_interface::return_type::ERROR;

    for (size_t i = 0; i < joint_results_.size(); ++i) 
    {
        const auto& result = joint_results_[i];
        if (result.fault != 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                            "HARDWARE FAULT on Joint %s (CAN-ID: %d)! Error Code: %d", 
                            joints_[i].name.c_str(), joints_[i].can_id, result.fault);
            return hardware_interface::return_type::ERROR;
        }
        /*
        auto output_position_raw = get_extra_register_value(result, moteus::Register::kEncoder1Position);
        if (!output_position_raw.has_value()) 
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                        "Joint %s (CAN-ID: %d): Required register kEncoder1Position was not found in the extra array! "
                        "Check your query configuration.", joints_[i].name.c_str(), joints_[i].can_id);
            return hardware_interface::return_type::ERROR;
        }

        double output_pos = *output_position_raw * 2.0 * M_PI - joints_[i].encoder_offset;
        double position = result.position * 2.0 * M_PI;
        static constexpr double belt_slip_threshold = 0.08;
        if (abs(output_pos - position) > belt_slip_threshold) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                    "Joint %s (CAN-ID: %d): Belt slip detected: encoder positions do not match!", joints_[i].name.c_str(), joints_[i].can_id);
            return hardware_interface::return_type::ERROR;
        }
        */
        hw_states_position_[i] = result.position * 2.0 * M_PI;
        hw_states_velocity_[i] = result.velocity * 2.0 * M_PI;
        hw_states_effort_[i]   = result.torque;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteusInterface::write(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/)
{
    using namespace mjbots;
    const size_t num_joints = joints_.size();
    // Older ROS versions call write in state inactive -> include this if to ensure backwards compatability
    if (!is_active_) return hardware_interface::return_type::OK;
    
    for (size_t i = 0; i < num_joints; ++i)
    {
        auto& joint = joints_[i];
        // standard mode: pos + vel + torque
        moteus::PositionMode::Command cmd;
        if (std::isnan(hw_commands_position_[i])) {
            cmd.position = std::numeric_limits<double>::quiet_NaN();
        } else {
            cmd.position = hw_commands_position_[i] / (2.0 * M_PI);
        }
        if (std::isnan(hw_commands_velocity_[i])) {
            cmd.velocity = 0;
        } else {
            cmd.velocity = hw_commands_velocity_[i] / (2.0 * M_PI);
        }
        if (std::isnan(hw_commands_effort_[i])) {
            cmd.feedforward_torque = 0;
        } else {
            cmd.feedforward_torque = hw_commands_effort_[i];
        }
        cmd.kp_scale = 1.0;
        cmd.kd_scale = 1.0;
        cmd.ilimit_scale = 1.0;

        // pos inactive
        if (!joint.pos_active) {
            cmd.position = std::numeric_limits<double>::quiet_NaN();
            cmd.kp_scale = 0;
        }
        // vel inactive
        if (!joint.vel_active) {
            cmd.velocity = 0;
        }
        // torque inactive
        if (!joint.effort_active) {
            cmd.feedforward_torque = 0;
        }
        else {
            // special case: torque only mode
            if (!joint.pos_active && !joint.vel_active) {
                cmd.kd_scale = 0;
                cmd.ilimit_scale = 0;
            }
        }

        command_frames_[i] = joint.controller->MakePosition(cmd);
    }
    // Blocking at end of each control loop to ensure data coherency and avoid race conditions
    transport_->BlockingCycle(&command_frames_[0], command_frames_.size(), &replies_frames_);

    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MoteusInterface::on_configure(const rclcpp_lifecycle::State &/*previous_state*/)
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

        moteus::PositionMode::Format write_format;
        write_format.position = moteus::kFloat;
        write_format.velocity = moteus::kFloat;
        write_format.feedforward_torque = moteus::kFloat;
        write_format.kp_scale = moteus::kFloat;
        write_format.kd_scale = moteus::kFloat;
        write_format.ilimit_scale = moteus::kFloat;

        moteus::Query::ItemFormat encoder1;                 // extra field for secondary encoder
        encoder1.register_number = moteus::Register::kEncoder1Position;
        encoder1.resolution = moteus::Resolution::kFloat;
        moteus::Query::Format read_format;
        read_format.position           = moteus::Resolution::kInt16;
        read_format.velocity           = moteus::Resolution::kInt16;
        read_format.torque             = moteus::Resolution::kInt16;
        read_format.fault              = moteus::Resolution::kInt8;
        read_format.extra[0]           = encoder1;

        size_t num_joints = joints_.size();
        for (size_t i = 0; i < num_joints; ++i)
        {
            moteus::Controller::Options options;
            options.id = joints_[i].can_id;
            options.query_format = read_format;
            options.position_format = write_format;
            joints_[i].controller = std::make_shared<moteus::Controller>(options);
            command_frames_[i] = joints_[i].controller->MakeStop();
        }
        // reset controller to ensure defined state on startup
        transport_->BlockingCycle(&command_frames_[0], command_frames_.size(), &replies_frames_);
        
        if (!parse_result_frames()) return hardware_interface::CallbackReturn::ERROR;
        for (size_t i = 0; i < joint_results_.size(); ++i) 
        {
            const auto& result = joint_results_[i];
            auto output_position_raw = get_extra_register_value(result, moteus::Register::kEncoder1Position);

            if (!output_position_raw.has_value()) 
            {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                            "Joint %s (CAN-ID: %d): Required register kEncoder1Position was not found in the extra array! "
                            "Check your query configuration.", joints_[i].name.c_str(), joints_[i].can_id);
                return hardware_interface::CallbackReturn::ERROR;
            }

            moteus::OutputExact::Command cmd;
            //cmd.position = *output_position_raw - joints_[i].encoder_offset/(2.0 * M_PI);
            cmd.position = 0;
            command_frames_[i] = joints_[i].controller->MakeOutputExact(cmd);
        }

        // Set internal encoder to absolute position
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

hardware_interface::CallbackReturn MoteusInterface::on_activate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Activating Hardware...");
    using namespace mjbots;
    for (size_t i = 0; i < joints_.size(); ++i)
        {
            // expects at least one succesful data query before being called
            if (std::isnan(hw_states_position_[i])) {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                     "Joint %s undefined state, no valid position on startup", joints_[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            hw_commands_position_[i] = hw_states_position_[i];
            hw_commands_velocity_[i] = 0;
            hw_commands_effort_[i] = 0;

            moteus::PositionMode::Command cmd;
            cmd.position = hw_commands_position_[i] / (2.0 * M_PI);
            cmd.velocity = 0;
            cmd.feedforward_torque = 0;

            command_frames_[i] = joints_[i].controller->MakePosition(cmd);
        }
    transport_->BlockingCycle(&command_frames_[0], command_frames_.size(), &replies_frames_);
    is_active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusInterface::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Deactivating Hardware...");
    for (size_t i = 0; i < joints_.size(); ++i)
    {
        command_frames_[i] = joints_[i].controller->MakeStop();
    }
    transport_->BlockingCycle(&command_frames_[0], command_frames_.size(), &replies_frames_);
    is_active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusInterface::on_cleanup(const rclcpp_lifecycle::State &/*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Shutdown requested, stopping hardware...");
    return on_deactivate(previous_state);
}

hardware_interface::CallbackReturn MoteusInterface::on_error(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Hardware interface encountered an error, executing emergency stop!");
    return on_deactivate(previous_state);
}

bool MoteusInterface::parse_result_frames()
{
    using namespace mjbots;
    size_t num_joints = joints_.size();
    std::fill(joint_updated_.begin(), joint_updated_.end(), false);

    for (const auto& frame : replies_frames_)
    {
        auto it = std::find_if(joints_.begin(), joints_.end(),
            [&frame](const Joint& j) { 
                return j.can_id == frame.source; 
            }
        );
        if (it != joints_.end()) 
        {
            size_t joint_index = std::distance(joints_.begin(), it);
            
            joint_results_[joint_index] = moteus::Query::Parse(frame.data, frame.size);

            joint_updated_[joint_index] = true;
        }
    }

    bool ret = true;
    for (size_t i = 0; i < num_joints; ++i) {
        if (!joint_updated_[i]) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                         "Joint %s (CAN-ID: %d) did not respond!", joints_[i].name.c_str(), joints_[i].can_id);
            ret = false;
        }
    }
    return ret;
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