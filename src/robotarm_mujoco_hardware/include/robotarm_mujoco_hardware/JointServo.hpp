#ifndef ROBOTARM_MUJOCO_HARDWARE_JOINTSERVO_HPP
#define ROBOTARM_MUJOCO_HARDWARE_JOINTSERVO_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace robotarm_mujoco_hardware {

struct ServoGains
{
    double kp = 0.0;   // Nm/rad
    double ki = 0.0;   // Nm/(rad·s)
    double kd = 0.0;   // Nm·s/rad, on the velocity error
    double effort_limit = std::numeric_limits<double>::infinity();   // Nm, output and integral clamp
};

// Position servo of one simulated joint, the structure of the moteus position loop:
//   tau = kp·(p_ref − p) + kd·(v_ref − v) + ki·∫(p_ref − p) dt
// v is the measured joint velocity, nothing is differentiated, so irregular cycle times do not turn into
// torque noise. The output and the integral are clamped to the effort limit, and the integral only runs
// while the output is not saturated (conditional integration).
class JointServo
{
public:
    explicit JointServo(const ServoGains& gains = ServoGains()) : gains_(gains) {}

    void reset() { integral_ = 0.0; }
    double integral() const { return integral_; }
    const ServoGains& gains() const { return gains_; }

    // dt <= 0: no time passed, the integral stays as it is
    double update(double p_ref, double v_ref, double p, double v, double dt)
    {
        const double error = p_ref - p;
        const double tau = gains_.kp * error + gains_.kd * (v_ref - v) + integral_;
        const double limit = gains_.effort_limit;
        if (dt > 0.0 && std::abs(tau) < limit) {
            integral_ = std::clamp(integral_ + gains_.ki * error * dt, -limit, limit);
        }
        return std::clamp(tau, -limit, limit);
    }

private:
    ServoGains gains_;
    double integral_ = 0.0;
};

}  // namespace robotarm_mujoco_hardware

#endif
