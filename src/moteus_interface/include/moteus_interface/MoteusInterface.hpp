#ifndef MOTEUSINTERFACE_HPP
#define MOTEUSINTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "moteus.h"

#include "moteus_interface/TransportUDP.hpp"

namespace moteus_interface
{

class MoteusInterface : public hardware_interface::SystemInterface 
{
public:
    using transport_type = transport::TransportUDP;

private:
    class Joint 
    {
    public:
        std::string name_;
        int can_id_;
        double gear_ratio_;
        double encoder_offset_;

        bool pos_active_ = false;
        bool vel_active_ = false;
        bool effort_active_ = false;

        bool is_updated_ = false;
        size_t consecutive_failures_ = 0;
        double failure_rate_ = 0.0;
        static constexpr size_t MAX_CONSECUTIVE_FAILURES = 3;
        static constexpr double MAX_FAILURE_RATE = 0.10;
        static constexpr double FILTER_ALPHA = 0.01;
        
        std::shared_ptr<mjbots::moteus::Controller> controller_;

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
    hardware_interface::return_type dispatch_cyclic_commands(const uint32_t bus_timeout_us);
    void parse_result_frames();
    bool watchdog(bool strict=false);
    bool check_joint_interface(hardware_interface::ComponentInfo joint);

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
    enum class ControlType : uint8_t 
    {
        STANDARD = 1,
        TORQUE_CONTROL = 2
    };

private:
    bool is_active_;
    ExecutionMode execution_mode_;

    std::vector<double> hw_commands_position_;
    std::vector<double> hw_commands_velocity_;
    std::vector<double> hw_commands_effort_;

    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
    std::vector<double> hw_states_effort_;

    std::vector<Joint> joints_;
    std::vector<bool> joint_updated_;

    std::shared_ptr<transport::Transport> transport_;
    
    std::vector<mjbots::moteus::Query::Result> joint_results_;
    std::vector<mjbots::moteus::CanFdFrame> command_frames_;
    std::vector<mjbots::moteus::CanFdFrame> replies_frames_;

    // TODO: move to urdf
    const std::string IP_GATEWAY = "192.168.000.200";
    const uint16_t PORT_GATEWAY = 6666;
};

}

#endif