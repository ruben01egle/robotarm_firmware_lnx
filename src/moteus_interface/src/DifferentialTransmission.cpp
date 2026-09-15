#include "moteus_interface/DifferentialTransmission.hpp"

moteus_interface::transmission::DifferentialTransmission::DifferentialTransmission(
                        JointPort joint1, JointPort joint2,
                        ActuatorPort actuator_a, ActuatorPort actuator_b):
                        joint1_(joint1),
                        joint2_(joint2),
                        actuator_a_(actuator_a),
                        actuator_b_(actuator_b)
{
}

void moteus_interface::transmission::DifferentialTransmission::actuator_to_joint()
{
    *joint1_.state.position = (*actuator_a_.state.position + *actuator_b_.state.position) / 2.0;
    *joint1_.state.velocity = (*actuator_a_.state.velocity + *actuator_b_.state.velocity) / 2.0;
    *joint1_.state.effort = (*actuator_a_.state.effort + *actuator_b_.state.effort) / 2.0;

    *joint2_.state.position = (*actuator_a_.state.position - *actuator_b_.state.position) / 2.0;
    *joint2_.state.velocity = (*actuator_a_.state.velocity - *actuator_b_.state.velocity) / 2.0;
    *joint2_.state.effort = (*actuator_a_.state.effort - *actuator_b_.state.effort) / 2.0;
}

void moteus_interface::transmission::DifferentialTransmission::joint_to_actuator()
{
    *actuator_a_.command.position = *joint1_.command.position + *joint2_.command.position;
    *actuator_a_.command.velocity = *joint1_.command.velocity + *joint2_.command.velocity;
    *actuator_a_.command.effort = *joint1_.command.effort + *joint2_.command.effort;

    *actuator_b_.command.position = *joint1_.command.position - *joint2_.command.position;
    *actuator_b_.command.velocity = *joint1_.command.velocity - *joint2_.command.velocity;
    *actuator_b_.command.effort = *joint1_.command.effort - *joint2_.command.effort;
}

bool moteus_interface::transmission::DifferentialTransmission::validate_mode_switch() const
{
    return *joint1_.mode.pos_active == *joint2_.mode.pos_active &&
           *joint1_.mode.vel_active == *joint2_.mode.vel_active &&
           *joint1_.mode.effort_active == *joint2_.mode.effort_active;
}

void moteus_interface::transmission::DifferentialTransmission::perform_mode_switch()
{
    *actuator_a_.mode.pos_active = *joint1_.mode.pos_active;
    *actuator_a_.mode.vel_active = *joint1_.mode.vel_active;
    *actuator_a_.mode.effort_active = *joint1_.mode.effort_active;

    *actuator_b_.mode.pos_active = *actuator_a_.mode.pos_active;
    *actuator_b_.mode.vel_active = *actuator_a_.mode.vel_active;
    *actuator_b_.mode.effort_active = *actuator_a_.mode.effort_active;
}
