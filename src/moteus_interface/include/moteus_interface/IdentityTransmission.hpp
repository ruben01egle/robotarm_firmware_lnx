#ifndef MOTEUS_INTERFACE_IDENTITYTRANSMISSION_HPP
#define MOTEUS_INTERFACE_IDENTITYTRANSMISSION_HPP

#include "moteus_interface/Transmission.hpp"

namespace moteus_interface::transmission
{

class IdentityTransmission: public Transmission
{
public:
    IdentityTransmission(JointPort joint, ActuatorPort actuator);
    virtual ~IdentityTransmission() = default;

    virtual void actuator_to_joint() override;
    virtual void joint_to_actuator() override;

    virtual bool validate_mode_switch() const override;
    virtual void perform_mode_switch() override;

private:
    JointPort joint_;
    ActuatorPort actuator_;
};

}

#endif