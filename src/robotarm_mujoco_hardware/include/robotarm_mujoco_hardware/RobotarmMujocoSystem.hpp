#ifndef ROBOTARM_MUJOCO_HARDWARE_ROBOTARMMUJOCOSYSTEM_HPP
#define ROBOTARM_MUJOCO_HARDWARE_ROBOTARMMUJOCOSYSTEM_HPP

#include "mujoco_ros2_control/mujoco_system_interface.hpp"

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotarm_mujoco_hardware {

// Returns start_interfaces with every "<joint>/position" moved to the end, the order otherwise kept.
// MujocoSystemInterface applies the started interfaces one by one and each one replaces the joint's
// control mode, so the last one wins. Moved to the end, position wins over velocity on joints where a
// controller claims both.
std::vector<std::string> position_last(const std::vector<std::string>& start_interfaces);

// mujoco_ros2_control's MujocoSystemInterface, made to behave like the real arm towards the controllers:
//  - position wins over velocity when a controller claims both (position_last), the position PID runs
//  - a joint with nothing claimed holds its current position, like the moteus in its idle mode, instead of
//    driving to the start pose (after activation) or keeping the last torque (after a release)
//  - a newly claimed command starts at NaN position / 0 velocity / 0 effort like in moteus_interface, and a
//    NaN position holds the position the joint had at the claim
//  - after a world reset (reset_world service, viewer), which puts every joint into position hold, the claimed
//    modes are restored and idle joints hold the reset pose
//
// It sits between the controllers and the base class: the interfaces the controllers see are its own
// copies, read() copies the states in, write() the commands out.
class RobotarmMujocoSystem : public mujoco_ros2_control::MujocoSystemInterface
{
public:
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr long NONE = -1;

    enum class Kind { POSITION, VELOCITY, EFFORT };

    struct Joint
    {
        std::string name;
        long state_position = NONE;   // index into states_
        long command[3] = {NONE, NONE, NONE};   // index into commands_, by Kind
        bool claimed[3] = {false, false, false};
        bool touched = false;   // part of the switch being performed
        double hold = NaN;   // position held while nothing is claimed, or for a NaN position command

        bool idle() const { return !claimed[0] && !claimed[1] && !claimed[2]; }
    };

    void index_joints();
    void copy_states();
    double position(const Joint& joint) const;
    void hold_idle_joints(std::vector<std::string>& start_interfaces);
    bool base_was_reset() const;
    bool apply_claims_after_reset();

    // the base class's interfaces, and the copies the controllers get
    std::vector<hardware_interface::StateInterface> base_states_;
    std::vector<hardware_interface::CommandInterface> base_commands_;
    std::vector<double> states_;
    std::vector<double> commands_;
    // what write() last gave the base class, a reset changes the base values behind its back
    std::vector<double> written_;

    std::vector<Joint> joints_;
    // "<joint>/<interface>" -> (joint index, kind)
    std::unordered_map<std::string, std::pair<size_t, Kind>> joint_commands_;
};

}  // namespace robotarm_mujoco_hardware

#endif
