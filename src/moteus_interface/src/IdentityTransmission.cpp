#include "moteus_interface/IdentityTransmission.hpp"

#include <cmath>

moteus_interface::transmission::IdentityTransmission::IdentityTransmission(
                        JointPort joint,
                        ActuatorPort actuator,
                        double encoder_offset):
                        joint_(joint),
                        actuator_(actuator),
                        encoder_offset_(encoder_offset)
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

void moteus_interface::transmission::IdentityTransmission::home()
{
    // actuator_.home holds the raw kEncoder1Position reading on entry (single actuator,
    // so which one is unambiguous); overwrite it in place with the offset-corrected value.
    *actuator_.home = std::remainder(*actuator_.home - encoder_offset_ / (2.0 * M_PI), 1.0);
}
