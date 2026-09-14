#include "moteus_interface/IdentityTransmission.hpp"

moteus_interface::transmission::IdentityTransmission::IdentityTransmission(
                        Transmission::Handle joint_cmd,
                        Transmission::Handle joint_state,
                        Transmission::Handle actuator_cmd,
                        Transmission::Handle actuator_state):
                        joint_cmd_(joint_cmd),
                        joint_state_(joint_state),
                        actuator_cmd_(actuator_cmd),
                        actuator_state_(actuator_state)
{
}

void moteus_interface::transmission::IdentityTransmission::actuator_to_joint()
{
    *joint_state_.position = *actuator_state_.position;
    *joint_state_.velocity = *actuator_state_.velocity;
    *joint_state_.effort = *actuator_state_.effort;
}

void moteus_interface::transmission::IdentityTransmission::joint_to_actuator()
{
    *actuator_cmd_.position = *joint_cmd_.position;
    *actuator_cmd_.velocity = *joint_cmd_.velocity;
    *actuator_cmd_.effort = *joint_cmd_.effort;
}
