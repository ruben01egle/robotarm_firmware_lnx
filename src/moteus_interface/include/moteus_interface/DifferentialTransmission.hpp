#ifndef MOTEUS_INTERFACE_DIFFERENTIALTRANSMISSION_HPP
#define MOTEUS_INTERFACE_DIFFERENTIALTRANSMISSION_HPP

#include "moteus_interface/Transmission.hpp"

namespace moteus_interface::transmission
{

class DifferentialTransmission: public Transmission
{
public:
    DifferentialTransmission(JointPort joint1, JointPort joint2,
                            ActuatorPort actuator_a, ActuatorPort actuator_b,
                            double joint1_encoder_offset, double joint2_encoder_offset,
                            bool joint1_uses_actuator_a);
    virtual ~DifferentialTransmission() = default;

    virtual void actuator_to_joint() override;
    virtual void joint_to_actuator() override;

    virtual bool validate_mode_switch() const override;
    virtual void perform_mode_switch() override;
    virtual void home() override;

private:
    static void combine(double j1, double j2, double& a, double& b);

    JointPort joint1_, joint2_;
    ActuatorPort actuator_a_, actuator_b_;
    double joint1_encoder_offset_, joint2_encoder_offset_;
    // Which of this transmission's two actuators feeds joint1's dedicated absolute encoder
    // (the other actuator feeds joint2's) -- set once from the URDF at construction, since
    // this wiring is arbitrary per hardware harness and cannot be derived structurally.
    bool joint1_uses_actuator_a_;
};

}

#endif