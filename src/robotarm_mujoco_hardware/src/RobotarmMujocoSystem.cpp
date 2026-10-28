#include "robotarm_mujoco_hardware/RobotarmMujocoSystem.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>

// the base class exports its interfaces with the pointer based (deprecated) constructors, the copies here
// do the same so they stay plain doubles
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace robotarm_mujoco_hardware {

namespace {

constexpr const char* PIDS_PARAMETER = "pids_config_file";
constexpr const char* HW_IF_TORQUE_FF = "torque_ff";   // as in moteus_interface

const char* interface_name(int kind)
{
    static const char* names[] = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
                                  HW_IF_TORQUE_FF, hardware_interface::HW_IF_EFFORT};
    return names[kind];
}

}  // namespace

std::unordered_map<std::string, ServoGains> load_servo_gains(
    const std::string& path, const std::vector<std::string>& joints)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("cannot read servo gains '" + path + "': " + e.what());
    }
    std::unordered_map<std::string, ServoGains> gains;
    for (const auto& joint : joints) {
        const YAML::Node node = root[joint];
        if (!node || !node.IsMap()) {
            throw std::runtime_error("servo gains '" + path + "' have no entry for joint " + joint);
        }
        ServoGains g;
        for (const auto& [key, value] : {std::pair<const char*, double*>{"kp", &g.kp}, {"ki", &g.ki}, {"kd", &g.kd}}) {
            try {
                *value = node[key].as<double>();
            } catch (const YAML::Exception&) {
                throw std::runtime_error("servo gains '" + path + "', joint " + joint + ": " + key + " missing");
            }
            if (!std::isfinite(*value) || *value < 0.0) {
                throw std::runtime_error("servo gains '" + path + "', joint " + joint + ": " + key + " must be >= 0");
            }
        }
        gains[joint] = g;
    }
    return gains;
}

hardware_interface::CallbackReturn RobotarmMujocoSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
    const auto& info = params.hardware_info;

    // the joints and their command interfaces as the URDF declares them, for the controllers
    std::vector<std::string> names;
    for (const auto& joint_info : info.joints) {
        Joint joint;
        joint.name = joint_info.name;
        for (const auto& interface : joint_info.command_interfaces) {
            for (int kind = 0; kind < KINDS; ++kind) {
                if (interface.name == interface_name(kind)) {
                    joint.exported[kind] = true;
                }
            }
        }
        if (!joint.exported[EFFORT]) {
            RCLCPP_FATAL(get_logger(), "Joint %s needs an effort command interface, the sim is driven by torques",
                         joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        names.push_back(joint.name);
        joints_.push_back(joint);
    }

    // servo gains, needed as soon as a joint has position, velocity or torque_ff (only acts through the servo)
    servo_ = std::any_of(joints_.begin(), joints_.end(), [](const Joint& j) {
        return j.exported[POSITION] || j.exported[VELOCITY] || j.exported[TORQUE_FF];
    });
    const auto pids = info.hardware_parameters.find(PIDS_PARAMETER);
    if (servo_) {
        if (pids == info.hardware_parameters.end() || pids->second.empty()) {
            RCLCPP_FATAL(get_logger(), "Position/velocity/torque_ff interfaces need the servo gains, hardware parameter %s",
                         PIDS_PARAMETER);
            return hardware_interface::CallbackReturn::ERROR;
        }
        try {
            const auto gains = load_servo_gains(pids->second, names);
            for (auto& joint : joints_) {
                ServoGains g = gains.at(joint.name);
                const auto limit = info.limits.find(joint.name);
                if (limit != info.limits.end() && limit->second.has_effort_limits && limit->second.max_effort > 0.0) {
                    g.effort_limit = limit->second.max_effort;
                }
                joint.gains = g;
                RCLCPP_INFO(get_logger(), "Servo %s: kp %.4g, ki %.4g, kd %.4g, effort limit %.4g",
                            joint.name.c_str(), g.kp, g.ki, g.kd, g.effort_limit);
            }
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "%s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // the base class only gets effort, and no PIDs: it never controls a position itself
    hardware_interface::HardwareComponentInterfaceParams base_params = params;
    base_params.hardware_info.hardware_parameters.erase(PIDS_PARAMETER);
    for (auto& joint_info : base_params.hardware_info.joints) {
        auto& interfaces = joint_info.command_interfaces;
        interfaces.erase(std::remove_if(interfaces.begin(), interfaces.end(),
                                        [](const hardware_interface::InterfaceInfo& i) {
                                            return i.name != hardware_interface::HW_IF_EFFORT;
                                        }),
                         interfaces.end());
    }
    const auto result = MujocoSystemInterface::on_init(base_params);
    if (result != hardware_interface::CallbackReturn::SUCCESS || !servo_) {
        return result;
    }

    // the base class loads the mujoco_plugins in on_init, the ServoPlugin has attached to the bridge by now
    if (!ServoBridge::instance().attached()) {
        RCLCPP_FATAL(get_logger(), "Position/velocity/torque_ff interfaces need the ServoPlugin: add "
                     "robotarm_mujoco_hardware/ServoPlugin to the mujoco_plugins parameter (config/sim_plugins.yaml)");
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::vector<std::string> names_in_order;
    std::vector<ServoGains> gains;
    for (const auto& joint : joints_) {
        names_in_order.push_back(joint.name);
        gains.push_back(joint.gains);
    }
    ServoBridge::instance().configure(names_in_order, gains);
    targets_.assign(joints_.size(), ServoTarget());
    return result;
}

std::vector<hardware_interface::StateInterface> RobotarmMujocoSystem::export_state_interfaces()
{
    base_states_ = MujocoSystemInterface::export_state_interfaces();
    states_.assign(base_states_.size(), NaN);

    std::vector<hardware_interface::StateInterface> states;
    states.reserve(base_states_.size());
    for (size_t i = 0; i < base_states_.size(); ++i) {
        states.emplace_back(base_states_[i].get_prefix_name(), base_states_[i].get_interface_name(), &states_[i]);
        for (auto& joint : joints_) {
            if (base_states_[i].get_name() == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
                joint.state_position = static_cast<long>(i);
            }
            if (base_states_[i].get_name() == joint.name + "/" + hardware_interface::HW_IF_VELOCITY) {
                joint.state_velocity = static_cast<long>(i);
            }
        }
    }
    copy_states();
    return states;
}

std::vector<hardware_interface::CommandInterface> RobotarmMujocoSystem::export_command_interfaces()
{
    base_commands_ = MujocoSystemInterface::export_command_interfaces();
    for (size_t i = 0; i < base_commands_.size(); ++i) {
        for (auto& joint : joints_) {
            if (base_commands_[i].get_name() == joint.name + "/" + hardware_interface::HW_IF_EFFORT) {
                joint.base_effort = static_cast<long>(i);
            }
        }
    }

    commands_.assign(joints_.size() * KINDS, NaN);
    std::vector<hardware_interface::CommandInterface> commands;
    for (size_t j = 0; j < joints_.size(); ++j) {
        for (int kind = 0; kind < KINDS; ++kind) {
            if (joints_[j].exported[kind]) {
                commands.emplace_back(joints_[j].name, interface_name(kind), &command(j, static_cast<Kind>(kind)));
            }
        }
    }
    return commands;
}

bool RobotarmMujocoSystem::find_command(const std::string& interface, size_t& joint, Kind& kind) const
{
    for (size_t j = 0; j < joints_.size(); ++j) {
        for (int k = 0; k < KINDS; ++k) {
            if (joints_[j].exported[k] && interface == joints_[j].name + "/" + interface_name(k)) {
                joint = j;
                kind = static_cast<Kind>(k);
                return true;
            }
        }
    }
    return false;
}

void RobotarmMujocoSystem::copy_states()
{
    for (size_t i = 0; i < base_states_.size(); ++i) {
        states_[i] = base_states_[i].get_optional<double>().value_or(NaN);
    }
}

double RobotarmMujocoSystem::state(long index) const
{
    return index == NONE ? NaN : states_[static_cast<size_t>(index)];
}

void RobotarmMujocoSystem::restart_servo(Joint& joint)
{
    joint.hold = state(joint.state_position);
    ++joint.epoch;
}

hardware_interface::CallbackReturn RobotarmMujocoSystem::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    const auto result = MujocoSystemInterface::on_activate(previous_state);
    if (result != hardware_interface::CallbackReturn::SUCCESS) {
        return result;
    }
    // nothing is claimed on activation, every joint holds where it hangs. The base class runs on effort only.
    copy_states();
    std::vector<std::string> base_start;
    for (auto& joint : joints_) {
        std::fill(std::begin(joint.claimed), std::end(joint.claimed), false);
        restart_servo(joint);
        base_start.push_back(joint.name + "/" + hardware_interface::HW_IF_EFFORT);
    }
    if (MujocoSystemInterface::perform_command_mode_switch(base_start, {}) != hardware_interface::return_type::OK) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    ServoBridge::instance().take_reset();   // a reset before activation does not matter
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotarmMujocoSystem::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    // the servos let go, like the moteus when the hardware is stopped
    for (auto& target : targets_) {
        target.active = false;
        target.torque_ff = 0.0;
    }
    if (servo_) {
        ServoBridge::instance().set_targets(targets_);
    }
    for (const auto& joint : joints_) {
        if (joint.base_effort != NONE) {
            std::ignore = base_commands_[static_cast<size_t>(joint.base_effort)].set_value(0.0);
        }
    }
    return MujocoSystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type RobotarmMujocoSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    // like moteus_interface: per joint either effort, or position/velocity/torque_ff, never both
    std::vector<std::array<bool, KINDS>> claims(joints_.size());
    for (size_t j = 0; j < joints_.size(); ++j) {
        std::copy(std::begin(joints_[j].claimed), std::end(joints_[j].claimed), claims[j].begin());
    }
    size_t j;
    Kind kind;
    for (const auto& interface : stop_interfaces) {
        if (find_command(interface, j, kind)) claims[j][kind] = false;
    }
    for (const auto& interface : start_interfaces) {
        if (find_command(interface, j, kind)) claims[j][kind] = true;
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
        if (claims[i][EFFORT] && (claims[i][POSITION] || claims[i][VELOCITY] || claims[i][TORQUE_FF])) {
            RCLCPP_ERROR(get_logger(), "Joint %s: effort cannot be claimed together with position/velocity/torque_ff",
                         joints_[i].name.c_str());
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotarmMujocoSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    std::vector<bool> touched(joints_.size(), false);
    size_t j;
    Kind kind;
    for (const auto& interface : stop_interfaces) {
        if (find_command(interface, j, kind)) {
            joints_[j].claimed[kind] = false;
            touched[j] = true;
        }
    }
    for (const auto& interface : start_interfaces) {
        if (find_command(interface, j, kind)) {
            joints_[j].claimed[kind] = true;
            touched[j] = true;
            // safe start values like moteus_interface, a stale command of the previous controller is never used
            command(j, kind) = kind == POSITION ? NaN : 0.0;
        }
    }
    // a joint whose claims change starts from where it is
    for (size_t i = 0; i < joints_.size(); ++i) {
        if (touched[i]) {
            restart_servo(joints_[i]);
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotarmMujocoSystem::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    const auto result = MujocoSystemInterface::read(time, period);
    copy_states();
    return result;
}

hardware_interface::return_type RobotarmMujocoSystem::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    if (servo_ && ServoBridge::instance().take_reset()) {
        for (auto& joint : joints_) {
            restart_servo(joint);
        }
        RCLCPP_INFO(get_logger(), "World reset: joints without a position command hold the reset pose");
    }

    // sim time can stand still for a cycle (period 0), then the velocity is not integrated
    const double dt = std::max(period.seconds(), 0.0);
    for (size_t j = 0; j < joints_.size(); ++j) {
        auto& joint = joints_[j];
        double tau = 0.0;   // servo joints: the ServoPlugin sets the torque before every physics step
        if (joint.claimed[EFFORT]) {
            tau = command(j, EFFORT);
            if (!std::isfinite(tau)) {
                tau = 0.0;
            }
            if (servo_) {
                targets_[j].active = false;
            }
        } else if (servo_) {
            const double velocity_command = joint.claimed[VELOCITY] ? command(j, VELOCITY) : 0.0;
            const double v_ref = std::isfinite(velocity_command) ? velocity_command : 0.0;
            const double position_command = joint.claimed[POSITION] ? command(j, POSITION) : NaN;
            if (std::isfinite(position_command)) {
                joint.hold = position_command;   // a later NaN continues from the last target
            } else {
                joint.hold += v_ref * dt;          // moteus: NaN position integrates the velocity
            }
            auto& target = targets_[j];
            target.active = std::isfinite(joint.hold);
            target.position = joint.hold;
            target.velocity = v_ref;
            // moteus: clamped to the effort limit, NaN → 0
            const double torque_ff = joint.claimed[TORQUE_FF] ? command(j, TORQUE_FF) : 0.0;
            target.torque_ff = std::isfinite(torque_ff)
                ? std::clamp(torque_ff, -joint.gains.effort_limit, joint.gains.effort_limit) : 0.0;
            target.time = time.seconds();
            target.epoch = joint.epoch;
        }
        if (joint.base_effort != NONE) {
            std::ignore = base_commands_[static_cast<size_t>(joint.base_effort)].set_value(tau);
        }
    }
    if (servo_) {
        ServoBridge::instance().set_targets(targets_);
    }
    return MujocoSystemInterface::write(time, period);
}

}  // namespace robotarm_mujoco_hardware

PLUGINLIB_EXPORT_CLASS(robotarm_mujoco_hardware::RobotarmMujocoSystem, hardware_interface::SystemInterface)
