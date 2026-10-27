#include "robotarm_mujoco_hardware/ServoBridge.hpp"

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>

namespace robotarm_mujoco_hardware {

// Runs the joint servos of RobotarmMujocoSystem on the physics thread, right before every physics step, with
// the live state and the exact physics time. The target the controller manager posted for its time t_c is
// moved to the physics time t (p* + v*·(t − t_c)), so target and state belong to the same moment however late
// the controller manager runs. Effort-claimed joints are not touched, their torque comes through the base
// class as usual.
class ServoPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
    bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/) override
    {
        logger_ = node->get_logger();
        model_ = model;
        ServoBridge::instance().attach();
        return true;
    }

    void pre_step(mjData* data) override
    {
        if (!ServoBridge::instance().read(joints_, gains_, targets_, config_version_)) {
            return;
        }
        if (config_version_ != mapped_version_ && !map_joints()) {
            return;
        }
        const double t = data->time;
        const double dt = model_->opt.timestep;
        for (size_t j = 0; j < joints_.size(); ++j) {
            const ServoTarget& target = targets_[j];
            if (!target.active || actuator_[j] < 0) {
                continue;
            }
            if (target.epoch != epoch_[j]) {
                servos_[j].reset();
                epoch_[j] = target.epoch;
            }
            // capped, so a stalled controller manager does not let the target run away
            const double ahead = std::clamp(t - target.time, 0.0, MAX_EXTRAPOLATION);
            const double p_ref = target.position + target.velocity * ahead;
            data->ctrl[actuator_[j]] = servos_[j].update(
                p_ref, target.velocity, data->qpos[qpos_[j]], data->qvel[qvel_[j]], dt);
        }
    }

    void on_reset(mjData* /*data*/) override
    {
        for (auto& servo : servos_) {
            servo.reset();
        }
        ServoBridge::instance().notify_reset();
    }

    void cleanup() override { ServoBridge::instance().detach(); }

private:
    static constexpr double MAX_EXTRAPOLATION = 0.05;   // s

    bool map_joints()
    {
        const size_t n = joints_.size();
        qpos_.assign(n, -1);
        qvel_.assign(n, -1);
        actuator_.assign(n, -1);
        epoch_.assign(n, 0);
        servos_.clear();
        for (size_t j = 0; j < n; ++j) {
            const int joint = mj_name2id(model_, mjOBJ_JOINT, joints_[j].c_str());
            const int actuator = mj_name2id(model_, mjOBJ_ACTUATOR, joints_[j].c_str());
            if (joint < 0 || actuator < 0) {
                RCLCPP_ERROR(logger_, "Servo: no MuJoCo joint and actuator named %s, joint not driven",
                             joints_[j].c_str());
            } else {
                qpos_[j] = model_->jnt_qposadr[joint];
                qvel_[j] = model_->jnt_dofadr[joint];
                actuator_[j] = actuator;
            }
            servos_.emplace_back(gains_[j]);
        }
        mapped_version_ = config_version_;
        RCLCPP_INFO(logger_, "Servo: driving %zu joints on the physics thread", n);
        return true;
    }

    rclcpp::Logger logger_ = rclcpp::get_logger("ServoPlugin");
    const mjModel* model_ = nullptr;

    // copies of the bridge, refreshed every step
    std::vector<std::string> joints_;
    std::vector<ServoGains> gains_;
    std::vector<ServoTarget> targets_;
    uint64_t config_version_ = 0;
    uint64_t mapped_version_ = 0;

    std::vector<int> qpos_, qvel_, actuator_;
    std::vector<uint64_t> epoch_;
    std::vector<JointServo> servos_;
};

}  // namespace robotarm_mujoco_hardware

PLUGINLIB_EXPORT_CLASS(robotarm_mujoco_hardware::ServoPlugin, mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
