#ifndef ROBOTARM_MUJOCO_HARDWARE_SERVOBRIDGE_HPP
#define ROBOTARM_MUJOCO_HARDWARE_SERVOBRIDGE_HPP

#include "robotarm_mujoco_hardware/JointServo.hpp"

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace robotarm_mujoco_hardware {

// Target of one joint's servo, valid at the controller manager time t
struct ServoTarget
{
    bool active = false;     // false: the servo does not touch the joint (effort claimed, hardware inactive)
    double position = 0.0;   // rad, at time t
    double velocity = 0.0;   // rad/s
    double torque_ff = 0.0;  // Nm, added to the servo output, held (not extrapolated) until the next target
    double time = 0.0;       // s, sim time the target belongs to
    uint64_t epoch = 0;      // changes when the servo has to start fresh (integral reset)
};

// Hands the servo targets from the hardware interface (controller manager thread) to the ServoPlugin (physics
// thread). One per process: both live in this library, the hardware interface and the plugin are loaded by
// the same ros2_control_node.
class ServoBridge
{
public:
    static ServoBridge& instance();

    // plugin side
    void attach();
    void detach();
    // copies the targets into the plugin's own buffers (sized on configure), returns false before configure
    bool read(std::vector<std::string>& joints, std::vector<ServoGains>& gains, std::vector<ServoTarget>& targets,
              uint64_t& config_version) const;
    void notify_reset();

    // hardware side
    bool attached() const;
    void configure(const std::vector<std::string>& joints, const std::vector<ServoGains>& gains);
    void set_targets(const std::vector<ServoTarget>& targets);
    bool take_reset();   // true once after each world reset

private:
    ServoBridge() = default;

    mutable std::mutex mutex_;
    bool attached_ = false;
    bool reset_ = false;
    uint64_t config_version_ = 0;
    std::vector<std::string> joints_;
    std::vector<ServoGains> gains_;
    std::vector<ServoTarget> targets_;
};

}  // namespace robotarm_mujoco_hardware

#endif
