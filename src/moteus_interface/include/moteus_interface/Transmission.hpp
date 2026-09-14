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

class Transmission
{
public:
    Transmission() = default;
    virtual ~Transmission() = default;

    virtual void actuator_to_joint() = 0;
    virtual void joint_to_actuator() = 0;
};

}

#endif