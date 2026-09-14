#ifndef MOTEUS_INTERFACE_DIFFERENTIALTRANSMISSION_HPP
#define MOTEUS_INTERFACE_DIFFERENTIALTRANSMISSION_HPP

#include "moteus_interface/Transmission.hpp"

namespace moteus_interface::transmission
{

class DifferentialTransmission: public Transmission
{
public:
    DifferentialTransmission(Handle joint1_cmd, Handle joint1_state,
                            Handle joint2_cmd, Handle joint2_state,
                            Handle actuator_a_cmd, Handle actuator_a_state,
                            Handle actuator_b_cmd, Handle actuator_b_state);
    virtual ~DifferentialTransmission() = default;

    virtual void actuator_to_joint() override;
    virtual void joint_to_actuator() override;

private:
    Handle joint1_cmd_, joint1_state_, joint2_cmd_, joint2_state_;
    Handle actuator_a_cmd_, actuator_a_state_, actuator_b_cmd_, actuator_b_state_;
};

}

#endif