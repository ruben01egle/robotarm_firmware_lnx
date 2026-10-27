#ifndef ROBOTARM_MUJOCO_HARDWARE_ROBOTARMMUJOCOSYSTEM_HPP
#define ROBOTARM_MUJOCO_HARDWARE_ROBOTARMMUJOCOSYSTEM_HPP

#include "mujoco_ros2_control/mujoco_system_interface.hpp"
#include "robotarm_mujoco_hardware/JointServo.hpp"
#include "robotarm_mujoco_hardware/ServoBridge.hpp"

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotarm_mujoco_hardware {

// Reads the servo gains file: one entry per joint, {kp, ki, kd}. Throws std::runtime_error with the reason if
// the file is unreadable or a joint or gain is missing or invalid.
std::unordered_map<std::string, ServoGains> load_servo_gains(
    const std::string& path, const std::vector<std::string>& joints);

// mujoco_ros2_control's MujocoSystemInterface, made to behave like the real arm towards the controllers.
//
// The base class only simulates: it gets effort commands and nothing else (position/velocity and its PIDs are
// stripped from its HardwareInfo). The interfaces the controllers see are this class's own, like on the
// moteus:
//  - effort claimed: the torque goes to the joint as it is
//  - position and/or velocity claimed: a JointServo per joint, tau = kp·(p* − p) + kd·(v* − v) + ki·∫(p* − p),
//    on the measured velocity. It runs in the ServoPlugin on the physics thread before every physics step: this
//    class only posts the target (p*, v*, and the controller manager time it belongs to), the plugin moves it
//    to the physics time, so target and state belong to the same moment however late the controller manager
//    runs. A NaN position, or a claim of velocity only, integrates the velocity command from the position the
//    joint had when it was claimed
//  - nothing claimed: holds its current position, like the moteus in its idle mode
//  - a newly claimed command starts at NaN position / 0 velocity / 0 effort, like in moteus_interface
//  - effort mixed with position/velocity on one joint is rejected, like by moteus_interface
//  - after a world reset (reset_world service, viewer), idle joints hold the reset pose
class RobotarmMujocoSystem : public mujoco_ros2_control::MujocoSystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr long NONE = -1;

    enum Kind { POSITION = 0, VELOCITY = 1, EFFORT = 2, KINDS = 3 };

    struct Joint
    {
        std::string name;
        bool exported[KINDS] = {false, false, false};   // command interfaces of the URDF
        bool claimed[KINDS] = {false, false, false};
        long state_position = NONE;   // index into states_
        long state_velocity = NONE;
        long base_effort = NONE;      // index into base_commands_
        double hold = NaN;            // position target while no position command is given
        ServoGains gains;
        uint64_t epoch = 0;           // increased when the servo has to start fresh

        bool idle() const { return !claimed[POSITION] && !claimed[VELOCITY] && !claimed[EFFORT]; }
    };

    // "<joint>/<interface>" -> (joint index, kind), for the command interfaces of the URDF
    bool find_command(const std::string& interface, size_t& joint, Kind& kind) const;
    void copy_states();
    double state(long index) const;
    double& command(size_t joint, Kind kind) { return commands_[joint * KINDS + kind]; }
    void restart_servo(Joint& joint);

    std::vector<Joint> joints_;

    // the base class's interfaces, and the copies / own interfaces the controllers get
    std::vector<hardware_interface::StateInterface> base_states_;
    std::vector<hardware_interface::CommandInterface> base_commands_;
    std::vector<double> states_;
    std::vector<double> commands_;   // KINDS per joint
    std::vector<ServoTarget> targets_;   // posted to the ServoPlugin every write()
    bool servo_ = false;                 // a joint has position or velocity
};

}  // namespace robotarm_mujoco_hardware

#endif
