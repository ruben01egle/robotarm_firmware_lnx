#include "DifferentialTransmission.hpp"

moteus_interface::transmission::DifferentialTransmission::DifferentialTransmission(
                        Handle joint1_cmd, Handle joint1_state,
                        Handle joint2_cmd, Handle joint2_state,
                        Handle actuator_a_cmd, Handle actuator_a_state,
                        Handle actuator_b_cmd, Handle actuator_b_state):
                        joint1_cmd_(joint1_cmd),
                        joint1_state_(joint1_state),
                        joint2_cmd_(joint2_cmd),
                        joint2_state_(joint2_state),
                        actuator_a_cmd_(actuator_a_cmd),
                        actuator_a_state_(actuator_a_state),
                        actuator_b_cmd_(actuator_b_cmd),
                        actuator_b_state_(actuator_b_state)
{
}

void moteus_interface::transmission::DifferentialTransmission::actuator_to_joint()
{
    *joint1_state_.position = (*actuator_a_state_.position + *actuator_b_state_.position) / 2.0;
    *joint1_state_.velocity = (*actuator_a_state_.velocity + *actuator_b_state_.velocity) / 2.0;
    *joint1_state_.effort = (*actuator_a_state_.effort + *actuator_b_state_.effort) / 2.0;

    *joint2_state_.position = (*actuator_a_state_.position - *actuator_b_state_.position) / 2.0;
    *joint2_state_.velocity = (*actuator_a_state_.velocity - *actuator_b_state_.velocity) / 2.0;
    *joint2_state_.effort = (*actuator_a_state_.effort - *actuator_b_state_.effort) / 2.0;
}

void moteus_interface::transmission::DifferentialTransmission::joint_to_actuator()
{
    *actuator_a_cmd_.position = *joint1_cmd_.position + *joint2_cmd_.position;
    *actuator_a_cmd_.velocity = *joint1_cmd_.velocity + *joint2_cmd_.velocity;
    *actuator_a_cmd_.effort = *joint1_cmd_.effort + *joint2_cmd_.effort;

    *actuator_b_cmd_.position = *joint1_cmd_.position - *joint2_cmd_.position;
    *actuator_b_cmd_.velocity = *joint1_cmd_.velocity - *joint2_cmd_.velocity;
    *actuator_b_cmd_.effort = *joint1_cmd_.effort - *joint2_cmd_.effort;
}
