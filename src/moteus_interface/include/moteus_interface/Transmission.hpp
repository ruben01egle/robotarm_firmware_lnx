#ifndef MOTEUS_INTERFACE_TRANSMISSION_HPP
#define MOTEUS_INTERFACE_TRANSMISSION_HPP

#include <vector>
#include <cstddef>

namespace moteus_interface::transmission
{

struct Handle
{
    double* position = nullptr;
    double* velocity = nullptr;
    double* effort = nullptr;
};

inline Handle make_handle(std::vector<double>& pos, std::vector<double>& vel,
                           std::vector<double>& eff, size_t idx)
{
    return Handle{&pos[idx], &vel[idx], &eff[idx]};
}

struct ModeFlags
{
    bool* pos_active = nullptr;
    bool* vel_active = nullptr;
    bool* effort_active = nullptr;
};

template <typename T>
inline ModeFlags make_mode_flags(T& owner)
{
    return ModeFlags{&owner.pos_active_, &owner.vel_active_, &owner.effort_active_};
}

struct JointPort
{
    Handle command;
    Handle state;
    ModeFlags mode;
};

struct ActuatorPort
{
    Handle command;
    Handle state;
    ModeFlags mode;
    // Homing-only scratch: raw kEncoder1Position reading in, corrected/combined actuator-space
    // home command out (see Actuator::home_position_ for the dual-use explanation). Joint-space
    // calibration data doesn't need an equivalent -- it's passed into the Transmission
    // constructor by value instead.
    double* home = nullptr;
};

class Transmission
{
public:
    Transmission() = default;
    virtual ~Transmission() = default;

    virtual void actuator_to_joint() = 0;
    virtual void joint_to_actuator() = 0;

    // Read-only check: may this transmission's owned joints move
    // to the mode combination currently staged on their ModeFlags
    // by updating the actuators accordingly
    virtual bool validate_mode_switch() const = 0;
    // Commit: propagate the (already-validated) joint-space mode onto the
    // actuator-space ModeFlags this transmission owns.
    virtual void perform_mode_switch() = 0;

    // One-shot startup calculation: reads the raw, uncorrected absolute encoder reading(s)
    // staged in ActuatorPort::home, offset-corrects them using per-joint calibration data
    // supplied at construction, and overwrites ActuatorPort::home in place with the
    // resulting absolute actuator-space command(s). Position-only, no velocity/effort/mode
    // involvement, entirely separate from command/state.
    virtual void home() = 0;
};

}

#endif