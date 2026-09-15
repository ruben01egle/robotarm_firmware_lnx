#include "moteus_interface/IdentityTransmission.hpp"

moteus_interface::transmission::IdentityTransmission::IdentityTransmission(
                        JointPort joint,
                        ActuatorPort actuator):
                        joint_(joint),
                        actuator_(actuator)
{
}

void moteus_interface::transmission::IdentityTransmission::actuator_to_joint()
{
    *joint_.state.position = *actuator_.state.position;
    *joint_.state.velocity = *actuator_.state.velocity;
    *joint_.state.effort = *actuator_.state.effort;
}

void moteus_interface::transmission::IdentityTransmission::joint_to_actuator()
{
    *actuator_.command.position = *joint_.command.position;
    *actuator_.command.velocity = *joint_.command.velocity;
    *actuator_.command.effort = *joint_.command.effort;
}

bool moteus_interface::transmission::IdentityTransmission::validate_mode_switch() const
{
    return true;
}

void moteus_interface::transmission::IdentityTransmission::perform_mode_switch()
{
    *actuator_.mode.pos_active = *joint_.mode.pos_active;
    *actuator_.mode.vel_active = *joint_.mode.vel_active;
    *actuator_.mode.effort_active = *joint_.mode.effort_active;
}
