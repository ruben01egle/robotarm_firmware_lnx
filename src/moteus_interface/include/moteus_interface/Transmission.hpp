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

template <typename Tag>
struct Port
{
    Handle command;
    Handle state;
    ModeFlags mode;
};

struct JointTag {};
struct ActuatorTag {};

using JointPort = Port<JointTag>;
using ActuatorPort = Port<ActuatorTag>;

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
};

}

#endif