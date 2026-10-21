#include "robotarm_mujoco_hardware/RobotarmMujocoSystem.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

// the base class exports its interfaces with the pointer based (deprecated) constructors, the copies here
// do the same so they stay plain doubles
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace robotarm_mujoco_hardware {

namespace {

bool is_position(const std::string& interface)
{
    static const std::string suffix = std::string("/") + hardware_interface::HW_IF_POSITION;
    return interface.size() >= suffix.size()
        && interface.compare(interface.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

std::vector<std::string> position_last(const std::vector<std::string>& start_interfaces)
{
    std::vector<std::string> ordered = start_interfaces;
    std::stable_partition(ordered.begin(), ordered.end(),
                          [](const std::string& interface) { return !is_position(interface); });
    return ordered;
}

std::vector<hardware_interface::StateInterface> RobotarmMujocoSystem::export_state_interfaces()
{
    base_states_ = MujocoSystemInterface::export_state_interfaces();
    states_.assign(base_states_.size(), NaN);

    std::vector<hardware_interface::StateInterface> states;
    states.reserve(base_states_.size());
    for (size_t i = 0; i < base_states_.size(); ++i) {
        states.emplace_back(base_states_[i].get_prefix_name(), base_states_[i].get_interface_name(), &states_[i]);
    }
    copy_states();
    index_joints();
    return states;
}

std::vector<hardware_interface::CommandInterface> RobotarmMujocoSystem::export_command_interfaces()
{
    base_commands_ = MujocoSystemInterface::export_command_interfaces();
    commands_.assign(base_commands_.size(), NaN);
    written_.assign(base_commands_.size(), NaN);

    std::vector<hardware_interface::CommandInterface> commands;
    commands.reserve(base_commands_.size());
    for (size_t i = 0; i < base_commands_.size(); ++i) {
        commands.emplace_back(
            base_commands_[i].get_prefix_name(), base_commands_[i].get_interface_name(), &commands_[i]);
    }
    index_joints();
    return commands;
}

void RobotarmMujocoSystem::index_joints()
{
    joints_.clear();
    joint_commands_.clear();
    for (const auto& joint_info : get_hardware_info().joints) {
        Joint joint;
        joint.name = joint_info.name;
        for (size_t i = 0; i < base_states_.size(); ++i) {
            if (base_states_[i].get_name() == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
                joint.state_position = static_cast<long>(i);
            }
        }
        const std::pair<Kind, const char*> kinds[] = {
            {Kind::POSITION, hardware_interface::HW_IF_POSITION},
            {Kind::VELOCITY, hardware_interface::HW_IF_VELOCITY},
            {Kind::EFFORT, hardware_interface::HW_IF_EFFORT},
        };
        for (const auto& [kind, interface] : kinds) {
            const std::string name = joint.name + "/" + interface;
            joint_commands_[name] = {joints_.size(), kind};
            for (size_t i = 0; i < base_commands_.size(); ++i) {
                if (base_commands_[i].get_name() == name) {
                    joint.command[static_cast<int>(kind)] = static_cast<long>(i);
                }
            }
        }
        joints_.push_back(joint);
    }
}

void RobotarmMujocoSystem::copy_states()
{
    for (size_t i = 0; i < base_states_.size(); ++i) {
        states_[i] = base_states_[i].get_optional<double>().value_or(NaN);
    }
}

double RobotarmMujocoSystem::position(const Joint& joint) const
{
    return joint.state_position == NONE ? NaN : states_[static_cast<size_t>(joint.state_position)];
}

void RobotarmMujocoSystem::hold_idle_joints(std::vector<std::string>& start_interfaces)
{
    // the position PID with the current position as its target, the base class only needs the mode.
    // Only joints of this switch: an idle joint the switch does not touch keeps holding its old target.
    for (auto& joint : joints_) {
        if (joint.touched && joint.idle()) {
            joint.hold = position(joint);
            start_interfaces.push_back(joint.name + "/" + hardware_interface::HW_IF_POSITION);
        }
    }
}

hardware_interface::CallbackReturn RobotarmMujocoSystem::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    const auto result = MujocoSystemInterface::on_activate(previous_state);
    if (result != hardware_interface::CallbackReturn::SUCCESS) {
        return result;
    }
    // nothing is claimed on activation, every joint holds where it hangs
    copy_states();
    for (auto& joint : joints_) {
        std::fill(std::begin(joint.claimed), std::end(joint.claimed), false);
        joint.touched = true;
    }
    std::vector<std::string> start;
    hold_idle_joints(start);
    if (MujocoSystemInterface::perform_command_mode_switch(start, {}) != hardware_interface::return_type::OK) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < base_commands_.size(); ++i) {
        written_[i] = base_commands_[i].get_optional<double>().value_or(NaN);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool RobotarmMujocoSystem::base_was_reset() const
{
    // outside a reset, the base class never changes a command value of a (non mimic) joint
    for (const auto& joint : joints_) {
        const long index = joint.command[static_cast<int>(Kind::POSITION)];
        if (index == NONE) {
            continue;
        }
        const double base = base_commands_[static_cast<size_t>(index)].get_optional<double>().value_or(NaN);
        const double written = written_[static_cast<size_t>(index)];
        if (base != written && !(std::isnan(base) && std::isnan(written))) {
            return true;
        }
    }
    return false;
}

bool RobotarmMujocoSystem::apply_claims_after_reset()
{
    // the reset switched every joint to position hold at the reset pose, give the claimed ones their mode back
    std::vector<std::string> start;
    for (auto& joint : joints_) {
        joint.touched = true;
        joint.hold = position(joint);
        const std::pair<Kind, const char*> kinds[] = {
            {Kind::POSITION, hardware_interface::HW_IF_POSITION},
            {Kind::VELOCITY, hardware_interface::HW_IF_VELOCITY},
            {Kind::EFFORT, hardware_interface::HW_IF_EFFORT},
        };
        for (const auto& [kind, interface] : kinds) {
            if (joint.claimed[static_cast<int>(kind)]) {
                start.push_back(joint.name + "/" + interface);
            }
        }
    }
    start = position_last(start);
    hold_idle_joints(start);
    RCLCPP_INFO(get_logger(), "World reset: claimed control modes restored, idle joints hold the reset pose");
    return MujocoSystemInterface::perform_command_mode_switch(start, {}) == hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotarmMujocoSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    for (auto& joint : joints_) {
        joint.touched = false;
    }
    for (const auto& interface : stop_interfaces) {
        const auto it = joint_commands_.find(interface);
        if (it != joint_commands_.end()) {
            auto& joint = joints_[it->second.first];
            joint.claimed[static_cast<int>(it->second.second)] = false;
            joint.touched = true;
        }
    }
    for (const auto& interface : start_interfaces) {
        const auto it = joint_commands_.find(interface);
        if (it == joint_commands_.end()) {
            continue;
        }
        auto& joint = joints_[it->second.first];
        const int kind = static_cast<int>(it->second.second);
        joint.claimed[kind] = true;
        joint.touched = true;
        // safe start values like moteus_interface, a stale command of the previous controller is never used
        if (joint.command[kind] != NONE) {
            commands_[static_cast<size_t>(joint.command[kind])] = it->second.second == Kind::POSITION ? NaN : 0.0;
        }
        joint.hold = position(joint);
    }

    std::vector<std::string> start = position_last(start_interfaces);
    hold_idle_joints(start);
    return MujocoSystemInterface::perform_command_mode_switch(start, stop_interfaces);
}

hardware_interface::return_type RobotarmMujocoSystem::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    const auto result = MujocoSystemInterface::read(time, period);
    copy_states();
    return result;
}

hardware_interface::return_type RobotarmMujocoSystem::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    if (base_was_reset() && !apply_claims_after_reset()) {
        return hardware_interface::return_type::ERROR;
    }

    std::vector<double> out = commands_;   // small, the sim does not need to be allocation free
    for (const auto& joint : joints_) {
        const long position = joint.command[static_cast<int>(Kind::POSITION)];
        const long velocity = joint.command[static_cast<int>(Kind::VELOCITY)];
        const long effort = joint.command[static_cast<int>(Kind::EFFORT)];
        if (position != NONE && (joint.idle() || std::isnan(out[static_cast<size_t>(position)]))) {
            out[static_cast<size_t>(position)] = joint.hold;
        }
        for (const long index : {velocity, effort}) {
            if (index != NONE && std::isnan(out[static_cast<size_t>(index)])) {
                out[static_cast<size_t>(index)] = 0.0;
            }
        }
    }
    for (size_t i = 0; i < base_commands_.size(); ++i) {
        std::ignore = base_commands_[i].set_value(out[i]);
        written_[i] = out[i];
    }
    return MujocoSystemInterface::write(time, period);
}

}  // namespace robotarm_mujoco_hardware

PLUGINLIB_EXPORT_CLASS(robotarm_mujoco_hardware::RobotarmMujocoSystem, hardware_interface::SystemInterface)
