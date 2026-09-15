#ifndef MOTEUS_INTERFACE_DIFFERENTIALTRANSMISSION_HPP
#define MOTEUS_INTERFACE_DIFFERENTIALTRANSMISSION_HPP

#include "moteus_interface/Transmission.hpp"

namespace moteus_interface::transmission
{

class DifferentialTransmission: public Transmission
{
public:
    DifferentialTransmission(JointPort joint1, JointPort joint2,
                            ActuatorPort actuator_a, ActuatorPort actuator_b);
    virtual ~DifferentialTransmission() = default;

    virtual void actuator_to_joint() override;
    virtual void joint_to_actuator() override;

    virtual bool validate_mode_switch() const override;
    virtual void perform_mode_switch() override;

private:
    JointPort joint1_, joint2_;
    ActuatorPort actuator_a_, actuator_b_;
};

}

#endif