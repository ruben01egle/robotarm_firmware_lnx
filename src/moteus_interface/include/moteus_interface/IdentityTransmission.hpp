#ifndef MOTEUS_INTERFACE_IDENTITYTRANSMISSION_HPP
#define MOTEUS_INTERFACE_IDENTITYTRANSMISSION_HPP

#include "moteus_interface/Transmission.hpp"

namespace moteus_interface::transmission
{

class IdentityTransmission: public Transmission
{
public:
    IdentityTransmission(Handle joint_cmd, Handle joint_state,
                        Handle actuator_cmd, Handle actuator_state);
    virtual ~IdentityTransmission() = default;

    virtual void actuator_to_joint() override;
    virtual void joint_to_actuator() override;

private:
    Handle joint_cmd_, joint_state_;
    Handle actuator_cmd_, actuator_state_;
};

}

#endif