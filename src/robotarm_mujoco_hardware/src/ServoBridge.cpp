#include "robotarm_mujoco_hardware/ServoBridge.hpp"

namespace robotarm_mujoco_hardware {

ServoBridge& ServoBridge::instance()
{
    static ServoBridge bridge;
    return bridge;
}

void ServoBridge::attach()
{
    std::lock_guard<std::mutex> lock(mutex_);
    attached_ = true;
}

void ServoBridge::detach()
{
    std::lock_guard<std::mutex> lock(mutex_);
    attached_ = false;
}

bool ServoBridge::read(std::vector<std::string>& joints, std::vector<ServoGains>& gains,
                       std::vector<ServoTarget>& targets, uint64_t& config_version) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_version_ == 0) {
        return false;
    }
    if (config_version != config_version_) {
        joints = joints_;
        gains = gains_;
        config_version = config_version_;
    }
    targets = targets_;   // same size as before after the first copy, no allocation
    return true;
}

void ServoBridge::notify_reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    reset_ = true;
}

bool ServoBridge::attached() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return attached_;
}

void ServoBridge::configure(const std::vector<std::string>& joints, const std::vector<ServoGains>& gains)
{
    std::lock_guard<std::mutex> lock(mutex_);
    joints_ = joints;
    gains_ = gains;
    targets_.assign(joints.size(), ServoTarget());
    ++config_version_;
}

void ServoBridge::set_targets(const std::vector<ServoTarget>& targets)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (targets.size() == targets_.size()) {
        targets_ = targets;
    }
}

bool ServoBridge::take_reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const bool reset = reset_;
    reset_ = false;
    return reset;
}

}  // namespace robotarm_mujoco_hardware
