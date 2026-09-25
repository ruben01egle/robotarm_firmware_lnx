#ifndef MOTEUS_INTERFACE_COMMANDMODE_HPP
#define MOTEUS_INTERFACE_COMMANDMODE_HPP

#include <cstdint>

namespace moteus_interface
{

struct ActiveInterfaces
{
    bool position = false;
    bool velocity = false;
    bool torque_ff = false;
    bool effort = false;

    bool operator==(const ActiveInterfaces& other) const
    {
        return position == other.position && velocity == other.velocity &&
               torque_ff == other.torque_ff && effort == other.effort;
    }
    bool operator!=(const ActiveInterfaces& other) const { return !(*this == other); }

    bool any_pv() const    { return position || velocity || torque_ff; }
    bool any_torque() const { return effort; }
    bool none() const       { return !any_pv() && !any_torque(); }
    bool valid() const { return !(any_pv() && any_torque()); }
};

enum class CommandMode : uint8_t 
    {
        IDLE = 0,
        PVC = 1,     // position-velocity-control
        TC = 2      // torque-control
    };

inline const char* to_string(CommandMode mode)
{
    switch (mode) {
        case CommandMode::IDLE: return "IDLE";
        case CommandMode::PVC:  return "PVC";
        case CommandMode::TC:   return "TC";
    }
    return "UNKNOWN";
}

[[nodiscard]] inline bool mode_of_active_interfaces(const ActiveInterfaces& interfaces, CommandMode& mode) {
    if (!interfaces.valid()) return false;
    if (interfaces.none()) {
        mode = CommandMode::IDLE;
        return true;
    }
    if (interfaces.any_pv()) {
        mode = CommandMode::PVC;
        return true;
    }
    if (interfaces.any_torque()) {
        mode = CommandMode::TC;
        return true;
    }
    return false;
}

}

#endif