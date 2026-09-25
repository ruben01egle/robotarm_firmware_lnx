#ifndef MOTEUSINTERFACE_HPP
#define MOTEUSINTERFACE_HPP

#include <string_view>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "moteus.h"

#include <unordered_map>

#include "moteus_interface/CommandMode.hpp"
#include "moteus_interface/Transport.hpp"
#include "moteus_interface/Transmission.hpp"

namespace moteus_interface
{

constexpr char HW_IF_TORQUE_FF[] = "torque_ff";


class MoteusInterface : public hardware_interface::SystemInterface
{
    // test fixture in test/test_mode_switch.cpp, reads the internal mode state
    friend class MoteusInterfaceTest;

private:
    class Joint
    {
    public:
        std::string name_;
        double encoder_offset_;
        // URDF limits, parsed by hardware_interface into info_.limits. max_effort is used to clamp
        // torque_ff by hand, since the controller_manager's limiter does not know custom interfaces
        joint_limits::JointLimits limits_;

        CommandMode cmd_mode_ = CommandMode::IDLE;
        ActiveInterfaces interfaces_;

        transmission::Handle command_handle_;   // points to joint_commands_*_[i]
        transmission::Handle state_handle_;     // points to hw_states_*_[i]
        transmission::Transmission* transmission_ = nullptr;
    };

    class Actuator 
    {
    public:
        std::string name_;
        int can_id_;
        // Dual-use homing scratch: on_configure() first writes the raw kEncoder1Position
        // reading here for every actuator, then each transmission's home() reads it back
        // (via ActuatorPort::home, which points at this field), offset-corrects/combines
        // it in joint space, and overwrites this same field in place with the final
        // actuator-space value that gets sent via MakeOutputExact. Never used outside homing.
        double home_position_ = 0.0;

        CommandMode cmd_mode_ = CommandMode::IDLE;

        bool is_updated_ = false;
        size_t consecutive_failures_ = 0;
        double failure_rate_ = 0.0;
        static constexpr size_t MAX_CONSECUTIVE_FAILURES = 3;
        static constexpr double MAX_FAILURE_RATE = 0.10;
        static constexpr double FILTER_ALPHA = 0.01;
        
        std::shared_ptr<mjbots::moteus::Controller> controller_;
        
        transmission::Handle command_handle_;   // points to actuator_commands_*_[i]
        transmission::Handle state_handle_;     // points to actuator_states_*_[i]
        transmission::Transmission* transmission_ = nullptr;

    public:
        void update_status(bool updated) {
            is_updated_ = updated;

            if (is_updated_) {
                consecutive_failures_ = 0;
            }
            else {
                consecutive_failures_++;
            }

            double current_error = is_updated_ ? 0.0 : 1.0;
            failure_rate_ = (1.0 - FILTER_ALPHA) * failure_rate_ + FILTER_ALPHA * current_error;
        }
        bool in_error_state() {
            return (consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES) ||
                (failure_rate_ > MAX_FAILURE_RATE);
        }
    };

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MoteusInterface)

    MoteusInterface() = default;
    virtual ~MoteusInterface() = default;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;


    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces) override;


    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;


    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;

private:
    void joint_interface_to_joint_physical();
    void apply_interfaces_to_joint_flags(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces);

    bool make_cyclic_commands();
    void parse_result_frames();
    bool watchdog(bool strict=false);
    bool check_joint_interface(hardware_interface::ComponentInfo joint);
    bool read_ros_parameters();

    template <typename T = double>
    std::optional<T> get_extra_register_value(
        const mjbots::moteus::Query::Result& result, 
        mjbots::moteus::Register reg_to_find)
    {
        for (size_t j = 0; j < mjbots::moteus::Query::kMaxExtra; ++j)
        {
            if (result.extra[j].register_number == reg_to_find)
            {
                return static_cast<T>(result.extra[j].value); 
            }
        }
        return std::nullopt;
    }

private:
    enum class ExecutionMode : uint8_t
    {
        STRICT_SEQUENTIAL = 1,
        PIPELINED = 2
    };

private:
    bool is_active_;
    ExecutionMode execution_mode_;

    // joint space interfaces
    // group position-velocity control
    std::vector<double> hw_commands_position_;
    std::vector<double> hw_commands_velocity_;
    std::vector<double> hw_commands_torque_ff_;

    // group torque control
    std::vector<double> hw_commands_effort_;

    // joint state interfaces
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
    std::vector<double> hw_states_effort_;

    // joint space physical
    std::vector<double> joint_commands_position_;
    std::vector<double> joint_commands_velocity_;
    std::vector<double> joint_commands_torque_;

    // actuator space physical
    std::vector<double> actuator_commands_position_;
    std::vector<double> actuator_commands_velocity_;
    std::vector<double> actuator_commands_torque_;

    // actuator state physical
    std::vector<double> actuator_states_position_;
    std::vector<double> actuator_states_velocity_;
    std::vector<double> actuator_states_torque_;

    std::vector<Joint> joints_;
    std::vector<std::unique_ptr<transmission::Transmission>> transmissions_;
    std::vector<Actuator> actuators_;
    std::vector<bool> actuators_updated_;

    std::unordered_map<std::string, size_t> joint_name_to_idx_;
    std::unordered_map<std::string, size_t> actuator_name_to_idx_;

    std::shared_ptr<transport::Transport> transport_;
    uint32_t timeout_us_;
    bool transport_timing_;
    
    std::vector<mjbots::moteus::Query::Result> actuator_results_;
    std::vector<mjbots::moteus::CanFdFrame> command_frames_;
    std::vector<mjbots::moteus::CanFdFrame> replies_frames_;

    std::vector<size_t> send_order_;
};

}

#endif