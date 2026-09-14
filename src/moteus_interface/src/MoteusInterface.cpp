#include "moteus_interface/MoteusInterface.hpp"

#include "moteus_interface/TransportUSB.hpp"
#include "moteus_interface/IdentityTransmission.hpp"
#include "moteus_interface/DifferentialTransmission.hpp"

#include <ctime>
#include <numeric>

PLUGINLIB_EXPORT_CLASS(
  moteus_interface::MoteusInterface, 
  hardware_interface::SystemInterface
)

namespace moteus_interface
{

// Not a real wait budget: in PIPELINED mode replies should already be
// buffered by the time read() runs. This only absorbs scheduling jitter
// between arrival and this call
constexpr uint32_t pipelined_read_timeout_us = 200;


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

    // Joint space
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

    std::unordered_map<std::string, size_t> joint_name_to_idx;

    for (size_t i = 0; i < num_joints; ++i)
    {
        if (!check_joint_interface(info_.joints[i])) return hardware_interface::CallbackReturn::ERROR;

        joints_[i].name_ = info_.joints[i].name;
        joints_[i].command_handle_ = transmission::make_handle(hw_commands_position_, hw_commands_velocity_, hw_commands_effort_, i);
        joints_[i].state_handle_ = transmission::make_handle(hw_states_position_, hw_states_velocity_, hw_states_effort_, i);

        joint_name_to_idx[joints_[i].name_] = i;

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
    }

    // Actuator space
    if (info_.transmissions.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("MoteusInterface"), "No transmissions found in URDF!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    size_t num_actuators = 0;
    for (const auto& t_info : info_.transmissions)
    {
        num_actuators += t_info.actuators.size();
    }

    actuator_commands_position_.assign(num_actuators, std::numeric_limits<double>::quiet_NaN());
    actuator_commands_velocity_.assign(num_actuators, 0.0);
    actuator_commands_effort_.assign(num_actuators, 0.0);
    
    actuator_states_position_.assign(num_actuators, std::numeric_limits<double>::quiet_NaN());
    actuator_states_velocity_.assign(num_actuators, 0.0);
    actuator_states_effort_.assign(num_actuators, 0.0);

    actuators_.resize(num_actuators);
    actuators_updated_.assign(num_actuators, false);
    command_frames_.resize(num_actuators);
    send_order_.resize(num_actuators);
    std::iota(send_order_.begin(), send_order_.end(), 0);
    replies_frames_.reserve(num_actuators*2);            // reserve instead of resize for clear/pushback in transport cycle, some headroom to avoid heap alloc
    actuator_results_.resize(num_actuators);

    std::unordered_map<std::string, size_t> actuator_name_to_idx;
    size_t next_actuator_idx = 0;
    for (const auto& t_info : info_.transmissions)
    {
        for (const auto& act_info : t_info.actuators)
        {
            auto param_it_can = t_info.parameters.find(act_info.name + ".can_id");
            if (param_it_can == t_info.parameters.end())
            {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                    "Actuator %s (transmission %s) missing '%s.can_id' parameter!",
                    act_info.name.c_str(), t_info.name.c_str(), act_info.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            size_t idx = next_actuator_idx++;
            actuators_[idx].name_ = act_info.name;
            actuators_[idx].can_id_ = std::stoi(param_it_can->second);
            actuators_[idx].command_handle_ = transmission::make_handle(actuator_commands_position_, actuator_commands_velocity_, actuator_commands_effort_, idx);
            actuators_[idx].state_handle_= transmission::make_handle(actuator_states_position_, actuator_states_velocity_, actuator_states_effort_, idx);

            actuator_name_to_idx[act_info.name] = idx;

            RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"),
                "Actuator %s using CAN-ID: %d registered", act_info.name.c_str(), actuators_[idx].can_id_);
        }
    }

    // Transmission
    transmissions_.reserve(info_.transmissions.size());
    for (const auto& t_info : info_.transmissions)
    {
        std::unique_ptr<transmission::Transmission> t;

        auto claim_transmission = [&](auto& owner, const char* kind, transmission::Transmission* t_ptr) -> bool
        {
            if (owner.transmission_ != nullptr)
            {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                    "%s %s is already claimed by another transmission (attempted claim by '%s')!",
                    kind, owner.name_.c_str(), t_info.name.c_str());
                return false;
            }
            owner.transmission_ = t_ptr;
            return true;
        };

        if (t_info.type == "identity") {
            if (t_info.joints.size() != 1 || t_info.actuators.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                    "Transmission %s: 'identity' needs exactly 1 joint and 1 actuator!", t_info.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            size_t joint_idx = joint_name_to_idx[t_info.joints[0].name];
            size_t actuator_idx = actuator_name_to_idx[t_info.actuators[0].name];
            auto& joint = joints_[joint_idx];
            auto& actuator = actuators_[actuator_idx];

            t = std::make_unique<transmission::IdentityTransmission>(joint.command_handle_, joint.state_handle_,
                                                                    actuator.command_handle_, actuator.state_handle_);

            if (!claim_transmission(joint, "Joint", t.get())) return hardware_interface::CallbackReturn::ERROR;
            if (!claim_transmission(actuator, "Actuator", t.get())) return hardware_interface::CallbackReturn::ERROR;
        }

        else if (t_info.type == "differential") {
            if (t_info.joints.size() != 2 || t_info.actuators.size() != 2)
            {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                    "Transmission %s: 'differential' needs exactly 2 joints and 2 actuators!", t_info.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            size_t joint_1_idx = joint_name_to_idx[t_info.joints[0].name];
            size_t joint_2_idx = joint_name_to_idx[t_info.joints[1].name];
            size_t actuator_a_idx = actuator_name_to_idx[t_info.actuators[0].name];
            size_t actuator_b_idx = actuator_name_to_idx[t_info.actuators[1].name];
            auto& joint_1 = joints_[joint_1_idx];
            auto& joint_2 = joints_[joint_2_idx];
            auto& actuator_a = actuators_[actuator_a_idx];
            auto& actuator_b = actuators_[actuator_b_idx];

            t = std::make_unique<transmission::DifferentialTransmission>(joint_1.command_handle_, joint_1.state_handle_,
                                                                        joint_2.command_handle_, joint_2.state_handle_,
                                                                        actuator_a.command_handle_, actuator_a.state_handle_,
                                                                        actuator_b.command_handle_, actuator_b.state_handle_);

            if (!claim_transmission(joint_1, "Joint", t.get())) return hardware_interface::CallbackReturn::ERROR;
            if (!claim_transmission(joint_2, "Joint", t.get())) return hardware_interface::CallbackReturn::ERROR;
            if (!claim_transmission(actuator_a, "Actuator", t.get())) return hardware_interface::CallbackReturn::ERROR;
            if (!claim_transmission(actuator_b, "Actuator", t.get())) return hardware_interface::CallbackReturn::ERROR;
        }

        else {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                "Transmission %s: unknown type '%s'!", t_info.name.c_str(), t_info.type.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        transmissions_.push_back(std::move(t));
    }

    for (const auto& j : joints_)
    {
        if (j.transmission_ == nullptr)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Joint %s has no assigned transmission!", j.name_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    for (const auto& a : actuators_)
    {
        if (a.transmission_ == nullptr)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Actuator %s has no assigned transmission!", a.name_.c_str());
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
    // TODO: after claiming of joints transmission needs to claim the actuators and validate behavior (e.g differential joints must be in same mode)
    for (size_t i = 0; i < actuators_.size(); ++i) {
        
        for (const auto& interface : start_interfaces) {
            if (interface == actuators_[i].name_ + "/position") actuators_[i].pos_active_ = true;
            if (interface == actuators_[i].name_ + "/velocity") actuators_[i].vel_active_ = true;
            if (interface == actuators_[i].name_ + "/effort")   actuators_[i].effort_active_ = true;
        }

        for (const auto& interface : stop_interfaces) {
            if (interface == actuators_[i].name_ + "/position") actuators_[i].pos_active_ = false;
            if (interface == actuators_[i].name_ + "/velocity") actuators_[i].vel_active_ = false;
            if (interface == actuators_[i].name_ + "/effort")   actuators_[i].effort_active_ = false;
        }
    }

    for (const auto& joint : actuators_) {
        RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"),
            "Actuator [%s] active interfaces: Pos=%s, Vel=%s, Eff=%s",
            joint.name_.c_str(),
            joint.pos_active_ ? "ON" : "OFF",
            joint.vel_active_ ? "ON" : "OFF",
            joint.effort_active_ ? "ON" : "OFF");
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteusInterface::read(const rclcpp::Time &/*time*/, const rclcpp::Duration& /*period*/)
{
    using namespace mjbots;
    const size_t num_joints = actuators_.size();

    // for our case this is always true, for more flexibility this could be computed based on the last sent commands
    uint32_t expected_replies = num_joints;

    // STRICT_SEQUENTIAL while active: a full transport_->cycle() (write + blocking read)
    // was called prior and populated replies_frames_
    // PIPELINED while active: write() fire and forget, collect replies here 
    if (execution_mode_ == ExecutionMode::PIPELINED) {
        // first ever call expects at least one previous transport::write or transport::cycle call
        // small timeout to tolerate overrun instead of crashing over small latency
        if (!transport_->read(replies_frames_, expected_replies, pipelined_read_timeout_us)) {
            return hardware_interface::return_type::ERROR;
        }
    }
    parse_result_frames();
    if (!watchdog()) return hardware_interface::return_type::ERROR;

    // TODO: read actuator values and use transmission to convert to joint space
    for (size_t i = 0; i < actuator_results_.size(); ++i) 
    {
        const auto& result = actuator_results_[i];
        if (result.fault != 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                            "HARDWARE FAULT on Actuator %s (CAN-ID: %d)! Error Code: %d", 
                            actuators_[i].name_.c_str(), actuators_[i].can_id_, result.fault);
            return hardware_interface::return_type::ERROR;
        }
        
        hw_states_position_[i] = result.position * 2.0 * M_PI;
        hw_states_velocity_[i] = result.velocity * 2.0 * M_PI;
        hw_states_effort_[i]   = result.torque;
    }

    // In inactive only read is called -> read needs query new data
    // In active this is done by read() or write() depending on mode
    if (!is_active_)
    {
        for (size_t i = 0; i < num_joints; ++i)
        {
            command_frames_[i] = actuators_[i].controller_->MakeStop();
        }

        if (execution_mode_ == ExecutionMode::PIPELINED) {
            if (!transport_->write(&command_frames_[0], command_frames_.size())) {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport write failed");
                return hardware_interface::return_type::ERROR;
            }
        }
        else {
            uint32_t expected_replies = actuators_.size();
            if (!transport_->cycle(&command_frames_[0], command_frames_.size(), replies_frames_, expected_replies, timeout_us_)) {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport cycle failed");
                return hardware_interface::return_type::ERROR;
            }
        }
    }
    else {
        if (execution_mode_ == ExecutionMode::PIPELINED) {
            if (!make_cyclic_commands()) {
                return hardware_interface::return_type::ERROR;
            }
            // PIPELINED: fire-and-forget
            if (!transport_->write(&command_frames_[0], command_frames_.size())) {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport write failed");
                return hardware_interface::return_type::ERROR;
            }
        }
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteusInterface::write(const rclcpp::Time &/*time*/, const rclcpp::Duration& /*period*/)
{
    // Older ROS versions call write in state inactive -> include this if to ensure backwards compatability
    if (!is_active_) return hardware_interface::return_type::OK;
    
    if (execution_mode_ == ExecutionMode::STRICT_SEQUENTIAL) {
        if (!make_cyclic_commands()) {
            return hardware_interface::return_type::ERROR;
        }

        uint32_t expected_replies = actuators_.size();
        if (!transport_->cycle(&command_frames_[0], command_frames_.size(), replies_frames_, expected_replies, timeout_us_)) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport cycle failed");
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MoteusInterface::on_configure(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Configuring moteus interface...");

    if (!read_ros_parameters()) { return hardware_interface::CallbackReturn::ERROR; }

    try {
        using namespace mjbots;
        transport_ = std::make_shared<moteus_interface::transport::TransportUSB>();
        if (!transport_->declare_and_read_parameters(get_node()->get_node_parameters_interface()))
        {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport parameter setup failed");
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

        size_t num_joints = actuators_.size();
        for (size_t i = 0; i < num_joints; ++i)
        {
            moteus::Controller::Options options;
            options.id = actuators_[i].can_id_;
            options.query_format = read_format;
            options.position_format = write_format;
            // options.transport left unset: controller_ is only used to build/parse frames (Make*),
            // never to send them, so Controller::transport() is never invoked. Do not call
            // Set*/Async*/Execute* on controller_ -- that would lazily spin up moteus's own
            // auto-detected transport alongside our custom transport_.
            actuators_[i].controller_ = std::make_shared<moteus::Controller>(options);
            command_frames_[i] = actuators_[i].controller_->MakeStop(&read_override);
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
        for (size_t i = 0; i < actuator_results_.size(); ++i) 
        {
            const auto& result = actuator_results_[i];
            auto output_position_raw = get_extra_register_value(result, moteus::Register::kEncoder1Position);

            if (!output_position_raw.has_value()) 
            {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                            "Actuator %s (CAN-ID: %d): Required register kEncoder1Position was not found in the extra array! "
                            "Check your query configuration.", actuators_[i].name_.c_str(), actuators_[i].can_id_);
                return hardware_interface::CallbackReturn::ERROR;
            }

            moteus::OutputExact::Command cmd;
            // TODO: encoder offset in joint space must be converted to actuator space
            cmd.position = std::remainder(*output_position_raw - actuators_[i].encoder_offset_/(2.0 * M_PI), 1.0);
            command_frames_[i] = actuators_[i].controller_->MakeOutputExact(cmd);
        }

        // Set internal encoder to absolute position
        if (!transport_->cycle(&command_frames_[0], command_frames_.size(), replies_frames_, expected_replies, 10000)) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), 
                            "Cycle commands failed");
            return hardware_interface::CallbackReturn::ERROR;
        }
            
        parse_result_frames();
        if (!watchdog(true)) return hardware_interface::CallbackReturn::ERROR;

        for (size_t i = 0; i < actuators_.size(); ++i)
        {
            command_frames_[i] = actuators_[i].controller_->MakeStop();
        }

        // guarante that transport layer replies before first read() call
        if (execution_mode_ == ExecutionMode::PIPELINED) {
            if (!transport_->write(&command_frames_[0], command_frames_.size())) {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport write failed");
                return hardware_interface::CallbackReturn::ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        else {
            if (!transport_->cycle(&command_frames_[0], command_frames_.size(), replies_frames_, expected_replies, 5000)) {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport cycle failed");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        
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
    for (size_t i = 0; i < actuators_.size(); ++i)
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
    if (!is_active_) {
        // Already deactivated (e.g. on_shutdown after on_deactivate). Nothing to do.
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(rclcpp::get_logger("MoteusInterface"), "Deactivating Hardware...");
    for (size_t i = 0; i < actuators_.size(); ++i)
    {
        command_frames_[i] = actuators_[i].controller_->MakeStop();
    }
    if (!transport_->write(&command_frames_[0], command_frames_.size())) {
        RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Transport write failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    is_active_ = false;

    // Dump timing log after the hardware is safe
    if (transport_timing_) {
        std::time_t now = std::time(nullptr);
        char ts[32];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now));
        const std::string path = std::string("transport_timing_") + ts + ".csv";
        transport_->dump_timing_log(path);
        transport_->set_timing_enabled(false);
    }

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

bool MoteusInterface::make_cyclic_commands()
{
    // TODO: transform joint space commands with transmission to actuator space before using them
    using namespace mjbots;
    const size_t num_joints = actuators_.size();

    std::rotate(send_order_.begin(), send_order_.begin() + 1, send_order_.end());

    for (size_t i = 0; i < num_joints; ++i)
    {
        auto& joint = actuators_[i];
        size_t send_idx = send_order_[i];

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

            command_frames_[send_idx] = joint.controller_->MakePosition(cmd);
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

            command_frames_[send_idx] = joint.controller_->MakePosition(cmd, &write_format_override);
        }
        else {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"), "Unknown control type");
            return false;
        }
    }

    return true;
}

void MoteusInterface::parse_result_frames()
{
    using namespace mjbots;
    std::fill(actuators_updated_.begin(), actuators_updated_.end(), false);

    for (const auto& frame : replies_frames_)
    {
        auto it = std::find_if(actuators_.begin(), actuators_.end(),
            [&frame](const Actuator& j) { 
                return j.can_id_ == frame.source; 
            }
        );
        if (it != actuators_.end()) 
        {
            size_t joint_index = std::distance(actuators_.begin(), it);
            
            actuator_results_[joint_index] = moteus::Query::Parse(frame.data, frame.size);

            actuators_updated_[joint_index] = true;
        }
    }
    replies_frames_.clear();
}

bool MoteusInterface::watchdog(bool strict)
{
    size_t num_joints = actuators_.size();
    bool ret = true;
    for (size_t i = 0; i < num_joints; ++i) {
        auto& joint = actuators_[i];
        joint.update_status(actuators_updated_[i]);
        
        if (!actuators_updated_[i]) {
            if (strict) {
                RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                         "Actuator %s (CAN-ID: %d) did not respond when requested!", actuators_[i].name_.c_str(), actuators_[i].can_id_);
                ret = false;
            }
            else {
                RCLCPP_WARN(rclcpp::get_logger("MoteusInterface"),
                         "Actuator %s (CAN-ID: %d) did not respond", actuators_[i].name_.c_str(), actuators_[i].can_id_);
            }
        }

        if (joint.in_error_state()) {
            RCLCPP_FATAL(rclcpp::get_logger("MoteusInterface"),
                         "Actuator %s (CAN-ID: %d) communication error!", actuators_[i].name_.c_str(), actuators_[i].can_id_);
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

    std::string execution_mode_str;

    try
    {
        execution_mode_str = declare_or_get("execution_mode", std::string("auto"));
        transport_timing_ = declare_or_get("time_transport", false);
        timeout_us_ = static_cast<uint32_t>(
            declare_or_get("timeout_transport_us", 0));
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

    if (timeout_us_ == 0) {
        if (execution_mode_ == ExecutionMode::STRICT_SEQUENTIAL)
        {
            RCLCPP_ERROR(get_logger(),
                "'timeout_transport_us' must be explicitly set in STRICT_SEQUENTIAL mode.");
            return false;
        }
        else // PIPELINED
        {
            RCLCPP_WARN(get_logger(),
                "'timeout_transport_us' not set — defaulting to 0. This might be problematic "
                "depending on the used transport layer! Check the specific behavior!");
        }
    }

    RCLCPP_INFO(get_logger(), "Transport Timing: %s", transport_timing_ ? "true" : "false");

    return true;
}

}