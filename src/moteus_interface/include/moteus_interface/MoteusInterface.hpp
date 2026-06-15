#ifndef MOTEUSINTERFACE_HPP
#define MOTEUSINTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "moteus.h"


namespace moteus_interface
{

class MoteusInterface : public hardware_interface::SystemInterface 
{
public:
    struct Joint {
        std::string name;
        int can_id;
        double gear_ratio;
        double encoder_offset;

        bool pos_active = false;
        bool vel_active = false;
        bool effort_active = false;
        
        std::shared_ptr<mjbots::moteus::Controller> controller;
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
    bool parse_result_frames();
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
    bool is_active_;

    std::vector<double> hw_commands_position_;
    std::vector<double> hw_commands_velocity_;
    std::vector<double> hw_commands_effort_;

    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
    std::vector<double> hw_states_effort_;

    std::vector<Joint> joints_;
    std::vector<bool> joint_updated_;

    std::shared_ptr<mjbots::moteus::Transport> transport_;
    
    std::vector<mjbots::moteus::Query::Result> joint_results_;
    std::vector<mjbots::moteus::CanFdFrame> command_frames_;
    std::vector<mjbots::moteus::CanFdFrame> replies_frames_;
};

}

#endif