#include "moteus_interface/DifferentialTransmission.hpp"

#include <cmath>

moteus_interface::transmission::DifferentialTransmission::DifferentialTransmission(
                        JointPort joint1, JointPort joint2,
                        ActuatorPort actuator_a, ActuatorPort actuator_b,
                        double joint1_encoder_offset, double joint2_encoder_offset,
                        bool joint1_uses_actuator_a):
                        joint1_(joint1),
                        joint2_(joint2),
                        actuator_a_(actuator_a),
                        actuator_b_(actuator_b),
                        joint1_encoder_offset_(joint1_encoder_offset),
                        joint2_encoder_offset_(joint2_encoder_offset),
                        joint1_uses_actuator_a_(joint1_uses_actuator_a)
{
}

void moteus_interface::transmission::DifferentialTransmission::actuator_to_joint()
{
    *joint1_.state.position = (*actuator_a_.state.position + *actuator_b_.state.position) / 2.0;
    *joint1_.state.velocity = (*actuator_a_.state.velocity + *actuator_b_.state.velocity) / 2.0;
    *joint1_.state.effort = *actuator_a_.state.effort + *actuator_b_.state.effort;

    *joint2_.state.position = (*actuator_a_.state.position - *actuator_b_.state.position) / 2.0;
    *joint2_.state.velocity = (*actuator_a_.state.velocity - *actuator_b_.state.velocity) / 2.0;
    *joint2_.state.effort = *actuator_a_.state.effort - *actuator_b_.state.effort;
}

void moteus_interface::transmission::DifferentialTransmission::combine(
    double j1, double j2, double& a, double& b)
{
    a = j1 + j2;
    b = j1 - j2;
}

void moteus_interface::transmission::DifferentialTransmission::joint_to_actuator()
{
    combine(*joint1_.command.position, *joint2_.command.position,
            *actuator_a_.command.position, *actuator_b_.command.position);
    combine(*joint1_.command.velocity, *joint2_.command.velocity,
            *actuator_a_.command.velocity, *actuator_b_.command.velocity);

    *actuator_a_.command.effort = (*joint1_.command.effort + *joint2_.command.effort) / 2.0;
    *actuator_b_.command.effort = (*joint1_.command.effort - *joint2_.command.effort) / 2.0;
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

void moteus_interface::transmission::DifferentialTransmission::home()
{
    // actuator_a_.home / actuator_b_.home hold raw kEncoder1Position readings on entry.
    // Capture both into locals before writing either back, since combine() below needs
    // both raw values and would otherwise read an already-overwritten one.
    double raw_a = *actuator_a_.home;
    double raw_b = *actuator_b_.home;

    double raw_for_joint1 = joint1_uses_actuator_a_ ? raw_a : raw_b;
    double raw_for_joint2 = joint1_uses_actuator_a_ ? raw_b : raw_a;

    double joint1_home = std::remainder(raw_for_joint1 - joint1_encoder_offset_ / (2.0 * M_PI), 1.0);
    double joint2_home = std::remainder(raw_for_joint2 - joint2_encoder_offset_ / (2.0 * M_PI), 1.0);

    combine(joint1_home, joint2_home, *actuator_a_.home, *actuator_b_.home);
}
