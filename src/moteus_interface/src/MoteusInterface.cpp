#include "moteus_interface/MoteusInterface.hpp"

#include <ctime>

PLUGINLIB_EXPORT_CLASS(
  moteus_interface::MoteusInterface, 
  hardware_interface::SystemInterface
)

namespace moteus_interface
{

constexpr uint32_t read_timeout_us = 200;


// Parse static parameters from the URDF here.
// Dynamic/ROS parameters from the YAML configuration require 'get_node()' 
// and are therefore parsed later in on_configure().
hardware_interface::CallbackReturn MoteusInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    is_active_ = false;
    transport_timing_ = false;

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
    joint_updated_.assign(num_joints, false);
    command_frames_.resize(num_joints);
    replies_frames_.reserve(num_joints*2);            // reserve instead of resize for clear/pushback in transport cycle, some headroom to avoid heap alloc
    joint_results_.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i)
    {
        if (!check_joint_interface(info_.joints[i])) return hardware_interface::CallbackReturn::ERROR;

        joints_[i].name_ = info_.joints[i].name;

        auto param_it_can = info_.joints[i].parameters.find("can_id");
        if (param_it_can != info_.joints[i].parameters.end())
        {
            joints_[i].can_id_ = std::stoi(param_it_can->second);
            RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), 
                        "Joint %s using CAN-ID: %d registered", info_.joints[i].name.c_str(), joints_[i].can_id_);
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Joint %s missing 'can_id' parameter!", info_.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        auto param_it_enc = info_.joints[i].parameters.find("encoder_offset");
        if (param_it_enc != info_.joints[i].parameters.end())
        {
            joints_[i].encoder_offset_ = std::stod(param_it_enc->second);
        }
        else
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Joint %s missing 'encoder_offset' parameter!", info_.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        auto param_it_gear = info_.joints[i].parameters.find("gear_ratio");
        if (param_it_gear != info_.joints[i].parameters.end())
        {
            joints_[i].gear_ratio_ = std::stod(param_it_gear->second);
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
            if (interface == joints_[i].name_ + "/position") joints_[i].pos_active_ = true;
            if (interface == joints_[i].name_ + "/velocity") joints_[i].vel_active_ = true;
            if (interface == joints_[i].name_ + "/effort")   joints_[i].effort_active_ = true;
        }

        for (const auto& interface : stop_interfaces) {
            if (interface == joints_[i].name_ + "/position") joints_[i].pos_active_ = false;
            if (interface == joints_[i].name_ + "/velocity") joints_[i].vel_active_ = false;
            if (interface == joints_[i].name_ + "/effort")   joints_[i].effort_active_ = false;
        }
    }

    for (const auto& joint : joints_) {
        RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"),
            "Joint [%s] active interfaces: Pos=%s, Vel=%s, Eff=%s",
            joint.name_.c_str(),
            joint.pos_active_ ? "ON" : "OFF",
            joint.vel_active_ ? "ON" : "OFF",
            joint.effort_active_ ? "ON" : "OFF");
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteusInterface::read(const rclcpp::Time &/*time*/, const rclcpp::Duration &period)
{
    using namespace mjbots;
    const size_t num_joints = joints_.size();
    
    // first ever call expects at least one previous transport::write or transport::cycle call
    // small timeout to tolerate overrun instead of crashing over small latency
    // for our case this is always true, for more flexibility this could be computed based on the last sent commands
    uint32_t expected_replies = num_joints;
    if (!transport_->read(replies_frames_, expected_replies, read_timeout_us)) return hardware_interface::return_type::ERROR;
    parse_result_frames();
    if (!watchdog()) return hardware_interface::return_type::ERROR;

    for (size_t i = 0; i < joint_results_.size(); ++i) 
    {
        const auto& result = joint_results_[i];
        if (result.fault != 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                            "HARDWARE FAULT on Joint %s (CAN-ID: %d)! Error Code: %d", 
                            joints_[i].name_.c_str(), joints_[i].can_id_, result.fault);
            return hardware_interface::return_type::ERROR;
        }
        /*
        auto output_position_raw = get_extra_register_value(result, moteus::Register::kEncoder1Position);
        if (!output_position_raw.has_value()) 
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                        "Joint %s (CAN-ID: %d): Required register kEncoder1Position was not found in the extra array! "
                        "Check your query configuration.", joints_[i].name_.c_str(), joints_[i].can_id_);
            return hardware_interface::return_type::ERROR;
        }

        double output_pos = *output_position_raw * 2.0 * M_PI - joints_[i].encoder_offset;
        RCLCPP_INFO(
            rclcpp::get_logger("MoteusInterface"),
            "Gelenk %zu - Berechnete Output-Position: %.4f", 
            i+1, 
            output_pos
        );
        */
        hw_states_position_[i] = result.position * 2.0 * M_PI;
        hw_states_velocity_[i] = result.velocity * 2.0 * M_PI;
        hw_states_effort_[i]   = result.torque;
    }

    // In inactive only read is called -> read needs query new data
    // In active this is done by read() or write() depending on mode
    uint32_t bus_timeout_us = static_cast<uint32_t>(period.nanoseconds() / 1000) - read_timeout_us;
    if (!is_active_)
    {
        for (size_t i = 0; i < num_joints; ++i)
        {
            command_frames_[i] = joints_[i].controller_->MakeStop();
        }
        if (!transport_->write(&command_frames_[0], command_frames_.size(), bus_timeout_us)) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport write failed");
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }
    
    if (execution_mode_ == ExecutionMode::PIPELINED) {
        return dispatch_cyclic_commands(bus_timeout_us);
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteusInterface::write(const rclcpp::Time &/*time*/, const rclcpp::Duration &period)
{
    // Older ROS versions call write in state inactive -> include this if to ensure backwards compatability
    if (!is_active_) return hardware_interface::return_type::OK;
    
    if (execution_mode_ == ExecutionMode::STRICT_SEQUENTIAL) {
        uint32_t bus_timeout_us = static_cast<uint32_t>(period.nanoseconds() / 1000) - read_timeout_us;
        return dispatch_cyclic_commands(bus_timeout_us);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MoteusInterface::on_configure(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Configuring moteus interface...");

    if (!read_ros_parameters()) { return hardware_interface::CallbackReturn::ERROR; }

    if (transport_mode_ == TransportMode::UDP)
    {
        transport_factory_ = [ip = udp_ip_, port = udp_port_]() {
            return std::make_shared<moteus_interface::transport::TransportUDP>(ip, port);
        };
    }
    else if (transport_mode_ == TransportMode::USB)
    {
        transport_factory_ = [device = usb_device_]() {
            return std::make_shared<moteus_interface::transport::TransportUSB>(device);
        };
    }

    try {
        using namespace mjbots;
        transport_ = transport_factory_();
        if (!transport_)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                         "Moteus-Transport not created!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!transport_->initialize()) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                         "Moteus-Transport not initialized!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        moteus::PositionMode::Format write_format;
        write_format.position = moteus::kFloat;
        write_format.velocity = moteus::kFloat;
        write_format.feedforward_torque = moteus::kFloat;
        write_format.kp_scale =  moteus::kIgnore;
        write_format.kd_scale =  moteus::kIgnore;
        write_format.maximum_torque =  moteus::kIgnore;
        write_format.stop_position =  moteus::kIgnore;
        write_format.watchdog_timeout =  moteus::kIgnore;
        write_format.velocity_limit =  moteus::kIgnore;
        write_format.accel_limit =  moteus::kIgnore;
        write_format.fixed_voltage_override =  moteus::kIgnore;
        write_format.ilimit_scale =  moteus::kIgnore;
        write_format.fixed_current_override =  moteus::kIgnore;
        write_format.ignore_position_bounds =  moteus::kIgnore;

        moteus::Query::Format read_format;
        read_format.mode = moteus::Resolution::kIgnore;
        read_format.position = moteus::Resolution::kInt16;
        read_format.velocity = moteus::Resolution::kInt16;
        read_format.torque = moteus::Resolution::kInt16;
        read_format.q_current = moteus::Resolution::kIgnore;
        read_format.d_current = moteus::Resolution::kIgnore;
        read_format.abs_position = moteus::Resolution::kIgnore;
        read_format.power = moteus::Resolution::kIgnore;
        read_format.motor_temperature = moteus::Resolution::kIgnore;
        read_format.trajectory_complete = moteus::Resolution::kIgnore;
        read_format.home_state = moteus::Resolution::kIgnore;
        read_format.voltage = moteus::Resolution::kIgnore;
        read_format.temperature = moteus::Resolution::kIgnore;
        read_format.fault = moteus::Resolution::kInt8;
        read_format.aux1_gpio = moteus::Resolution::kIgnore;
        read_format.aux2_gpio = moteus::Resolution::kIgnore;
        read_format.aux1_pwm_input_period_us = moteus::Resolution::kIgnore;
        read_format.aux1_pwm_input_duty_cycle = moteus::Resolution::kIgnore;
        read_format.aux2_pwm_input_period_us = moteus::Resolution::kIgnore;
        read_format.aux2_pwm_input_duty_cycle = moteus::Resolution::kIgnore;

        moteus::Query::Format read_override = read_format;
        moteus::Query::ItemFormat encoder1;                 // extra field for secondary encoder
        encoder1.register_number = moteus::Register::kEncoder1Position;
        encoder1.resolution = moteus::Resolution::kFloat;
        read_override.extra[0] = encoder1;

        size_t num_joints = joints_.size();
        for (size_t i = 0; i < num_joints; ++i)
        {
            moteus::Controller::Options options;
            options.id = joints_[i].can_id_;
            options.query_format = read_format;
            options.position_format = write_format;
            joints_[i].controller_ = std::make_shared<moteus::Controller>(options);
            command_frames_[i] = joints_[i].controller_->MakeStop(&read_override);
        }
        // reset controller to ensure defined state on startup
        // for our case this is always true, for more flexibility this could be computed based on the last sent commands
        uint32_t expected_replies = num_joints;
        if (!transport_->cycle(&command_frames_[0], command_frames_.size(), replies_frames_, expected_replies, 10000)) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                            "Cycle commands failed");
            return hardware_interface::CallbackReturn::ERROR;
        }
        parse_result_frames();
        if (!watchdog(true)) return hardware_interface::CallbackReturn::ERROR;
        for (size_t i = 0; i < joint_results_.size(); ++i) 
        {
            const auto& result = joint_results_[i];
            auto output_position_raw = get_extra_register_value(result, moteus::Register::kEncoder1Position);

            if (!output_position_raw.has_value()) 
            {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                            "Joint %s (CAN-ID: %d): Required register kEncoder1Position was not found in the extra array! "
                            "Check your query configuration.", joints_[i].name_.c_str(), joints_[i].can_id_);
                return hardware_interface::CallbackReturn::ERROR;
            }

            moteus::OutputExact::Command cmd;
            //cmd.position = *output_position_raw - joints_[i].encoder_offset/(2.0 * M_PI);
            cmd.position = 0;
            command_frames_[i] = joints_[i].controller_->MakeOutputExact(cmd);
        }

        // Set internal encoder to absolute position
        if (!transport_->cycle(&command_frames_[0], command_frames_.size(), replies_frames_, expected_replies, 10000)) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                            "Cycle commands failed");
            return hardware_interface::CallbackReturn::ERROR;
        }
            
        parse_result_frames();
        if (!watchdog(true)) return hardware_interface::CallbackReturn::ERROR;

        for (size_t i = 0; i < joints_.size(); ++i)
        {
            command_frames_[i] = joints_[i].controller_->MakeStop();
        }
        transport_->write(&command_frames_[0], command_frames_.size(), 8000);
        // guarante that transport layer replies before first read() call
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
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
    for (size_t i = 0; i < joints_.size(); ++i)
        {
            hw_commands_position_[i] = std::numeric_limits<double>::quiet_NaN();
            hw_commands_velocity_[i] = 0;
            hw_commands_effort_[i] = 0;
        }

    if (transport_timing_) {
        transport_->set_timing_enabled(true);
        RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Transport timing enabled.");
    }

    is_active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusInterface::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Deactivating Hardware...");
    for (size_t i = 0; i < joints_.size(); ++i)
    {
        command_frames_[i] = joints_[i].controller_->MakeStop();
    }
    if (!transport_->write(&command_frames_[0], command_frames_.size(), 1000)) {
        RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport write failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (transport_timing_) {
        std::time_t now = std::time(nullptr);
        char ts[32];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now));
        const std::string path = std::string("transport_timing_") + ts + ".csv";
        transport_->dump_timing_log(path);
        transport_->set_timing_enabled(false);
        RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Transport timing disabled, log saved to '%s'.", path.c_str());
    }

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

hardware_interface::return_type MoteusInterface::dispatch_cyclic_commands(const uint32_t bus_timeout_us)
{
    using namespace mjbots;
    const size_t num_joints = joints_.size();

    for (size_t i = 0; i < num_joints; ++i)
    {
        auto& joint = joints_[i];

        ControlMode control_type = (joint.effort_active_ && !joint.pos_active_ && !joint.vel_active_)
                                    ? ControlMode::TORQUE_CONTROL
                                    : ControlMode::STANDARD;

        if (control_type == ControlMode::STANDARD) {
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

            // pos inactive
            if (!joint.pos_active_) {
                cmd.position = std::numeric_limits<double>::quiet_NaN();
            }
            // vel inactive
            if (!joint.vel_active_) {
                cmd.velocity = 0;
            }
            // torque inactive
            if (!joint.effort_active_) {
                cmd.feedforward_torque = 0;
            }

            command_frames_[i] = joint.controller_->MakePosition(cmd);
        }
        else if (control_type == ControlMode::TORQUE_CONTROL) {
            moteus::PositionMode::Format write_format_override;
            write_format_override.position = moteus::kIgnore;
            write_format_override.velocity = moteus::kIgnore;
            write_format_override.feedforward_torque = moteus::kFloat;
            write_format_override.kp_scale = moteus::kInt8;
            write_format_override.kd_scale = moteus::kInt8;
            write_format_override.ilimit_scale = moteus::kInt8;

            moteus::PositionMode::Command cmd;
            cmd.position = std::numeric_limits<double>::quiet_NaN();
            cmd.velocity = 0;
            if (std::isnan(hw_commands_effort_[i])) {
                cmd.feedforward_torque = 0;
            } else {
                cmd.feedforward_torque = hw_commands_effort_[i];
            }
            cmd.kp_scale = 0;
            cmd.kd_scale = 0;
            cmd.ilimit_scale = 0;

            command_frames_[i] = joint.controller_->MakePosition(cmd, &write_format_override);
        }
        else {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Unknown control type");
            return hardware_interface::return_type::ERROR;
        }
    }

    if (!transport_->write(&command_frames_[0], command_frames_.size(), bus_timeout_us)) {
        RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport write failed");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

void MoteusInterface::parse_result_frames()
{
    using namespace mjbots;
    std::fill(joint_updated_.begin(), joint_updated_.end(), false);

    for (const auto& frame : replies_frames_)
    {
        auto it = std::find_if(joints_.begin(), joints_.end(),
            [&frame](const Joint& j) { 
                return j.can_id_ == frame.source; 
            }
        );
        if (it != joints_.end()) 
        {
            size_t joint_index = std::distance(joints_.begin(), it);
            
            joint_results_[joint_index] = moteus::Query::Parse(frame.data, frame.size);

            joint_updated_[joint_index] = true;
        }
    }
    replies_frames_.clear();
}

bool MoteusInterface::watchdog(bool strict)
{
    size_t num_joints = joints_.size();
    bool ret = true;
    for (size_t i = 0; i < num_joints; ++i) {
        auto& joint = joints_[i];
        joint.update_status(joint_updated_[i]);
        
        if (!joint_updated_[i]) {
            if (strict) {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                         "Joint %s (CAN-ID: %d) did not respond when requested!", joints_[i].name_.c_str(), joints_[i].can_id_);
                ret = false;
            }
            else {
                RCLCPP_WARN(rclcpp::get_logger("MoteusInterface"),
                         "Joint %s (CAN-ID: %d) did not respond", joints_[i].name_.c_str(), joints_[i].can_id_);
            }
        }

        if (joint.in_error_state()) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                         "Joint %s (CAN-ID: %d) communication error!", joints_[i].name_.c_str(), joints_[i].can_id_);
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

bool MoteusInterface::read_ros_parameters()
{
    auto node = get_node();
    if (!node)
    {
        RCLCPP_ERROR(get_logger(), "Framework-managed node not available");
        return false;
    }

    // declare only once; on subsequent on_configure() calls, just re-read the current value
    auto declare_or_get = [&node](const std::string & name, auto default_value)
    {
        if (!node->has_parameter(name))
        {
            return node->declare_parameter(name, default_value);
        }
        return node->get_parameter(name).get_value<decltype(default_value)>();
    };

    std::string transport_mode_str;
    std::string execution_mode_str;

    try
    {
        execution_mode_str = declare_or_get("execution_mode", std::string("auto"));
        transport_mode_str = declare_or_get("transport_mode", std::string("auto"));
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "Error reading parameters: %s", e.what());
        return false;
    }

    if (execution_mode_str == "auto") {
        execution_mode_ = ExecutionMode::STRICT_SEQUENTIAL;
        RCLCPP_INFO(get_logger(), "No execution_mode specified. Using default: STRICT_SEQUENTIAL");
    }
    else if (execution_mode_str == "pipelined") {
        execution_mode_ = ExecutionMode::PIPELINED;
        RCLCPP_INFO(get_logger(), "Execution Mode: pipelined");
    }
    else if (execution_mode_str == "strict_sequential") {
        execution_mode_ = ExecutionMode::STRICT_SEQUENTIAL;
        RCLCPP_INFO(get_logger(), "Execution Mode: strict_sequential");
    }
    else {
        RCLCPP_ERROR(get_logger(),
            "execution_mode must be 'pipelined' or 'strict_sequential', got '%s'", execution_mode_str.c_str());
        return false;
    }

    if (transport_mode_str == "auto") {
        transport_mode_ = TransportMode::USB;
        RCLCPP_INFO(get_logger(), "Default Transport Mode USB");
    }
    else if (transport_mode_str == "udp") {
        transport_mode_ = TransportMode::UDP;
        RCLCPP_INFO(get_logger(), "Transport Mode set to UDP");
    }
    else if (transport_mode_str == "usb") {
        transport_mode_ = TransportMode::USB;
        RCLCPP_INFO(get_logger(), "Transport Mode set to USB");
    }
    else {
        RCLCPP_ERROR(get_logger(), "transport_mode must be 'udp' or 'usb', got '%s'", transport_mode_str.c_str());
        return false;
    }

    try
    {
        if (transport_mode_ == TransportMode::UDP)
        {
            udp_ip_   = declare_or_get("udp_ip", std::string(""));
            udp_port_ = declare_or_get("udp_port", 0);
            RCLCPP_INFO(get_logger(), "UDP Config: %s:%d", udp_ip_.c_str(), udp_port_);
        }
        else if (transport_mode_ == TransportMode::USB)
        {
            usb_device_ = declare_or_get("usb_device", std::string("/dev/fdcanusb"));
            RCLCPP_INFO(get_logger(), "USB Config: %s", usb_device_.c_str());
        }

        transport_timing_ = declare_or_get("time_transport", false);
        RCLCPP_INFO(get_logger(), "Transport Timing: %s", transport_timing_ ? "true" : "false");
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "Error reading parameters: %s", e.what());
        return false;
    }

    return true;
}

}