#ifndef ROBOTARM_CONTROLLERS_FEEDFORWARDCONTROLLER_HPP
#define ROBOTARM_CONTROLLERS_FEEDFORWARDCONTROLLER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"

#include <string>
#include <vector>
#include <Eigen/Geometry>

#include "robotarm_rbd/RobotarmRbd.hpp"

namespace feedforward_controller
{

class FeedforwardController : public controller_interface::ChainableControllerInterface
{
public:
    struct Measured {
        Eigen::VectorXd q;
        Eigen::VectorXd tau;                 // only read on activation (ramp_start measured)
        void resize(size_t n) { q.resize(n); tau.resize(n); }
    };

    struct Reference {                       // persistent, reset on activate
        Eigen::VectorXd tau_cmd;
        Eigen::VectorXd tau_start;
        Eigen::VectorXd q_ref;
        Eigen::VectorXd dq_ref;
        Eigen::VectorXd ddq_ref;
        void resize(size_t n) {
            tau_cmd.resize(n);
            tau_start.resize(n);
            q_ref.resize(n);
            dq_ref.resize(n);
            ddq_ref.resize(n);
        }
    };

    struct RobotarmData {
        Measured state;
        Reference ref;
        Eigen::VectorXd tau_new;
        void resize(size_t n) { state.resize(n); ref.resize(n); tau_new.resize(n); }
    };

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FeedforwardController)

    FeedforwardController() = default;
    virtual ~FeedforwardController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

protected:
    // reference interfaces <name>/<joint>/{position,velocity,acceleration}, backed by reference_interfaces_
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    // standalone only, called before update_and_write_commands()
    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    void declare_parameters();
    controller_interface::CallbackReturn read_parameters();
    bool valid_ramp_start(const std::string& ramp_start);

    void fetch_references();
    bool fetch_robotarm_state_strict();    // all joints or fail, for a valid start in on_activate
    
private:
    // custom rigid body dynamics class
    robotarm_rbd::RobotarmRbd rbd_;
    robotarm_rbd::RobotarmRbd::Config rbd_cfg_;
    std::vector<std::string> rbd_joint_names_;
    std::vector<robotarm_rbd::RobotarmRbd::Limits> joint_limits_;  // from urdf, torque scaled by torque_scale_
    double torque_scale_ = 1.0;
    double torque_rise_time_ = 0;

    double ramp_time_ = 1.0;
    double ramp_elapsed_ = 0.0;

    RobotarmData data_;

    static constexpr size_t num_refs_per_joint_ = 3;
    static constexpr size_t num_states_per_joint_ = 2;
    static constexpr size_t num_cmd_interfaces_per_joint_ = 3;
};

}

#endif