#ifndef ROBOTARM_CONTROLLERS_CARTESIANIMPEDANCECONTROLLER_HPP
#define ROBOTARM_CONTROLLERS_CARTESIANIMPEDANCECONTROLLER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/subscription.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"

#include <string>
#include <vector>
#include <Eigen/Geometry>

#include "robotarm_rbd/RobotarmRbd.hpp"

namespace cartesian_impedance_controller
{

class CartesianImpedanceController : public controller_interface::ChainableControllerInterface
{
public:
    struct Measured {
        Eigen::VectorXd q, dq, tau;
        void resize(size_t n) { q.resize(n); dq.resize(n); tau.resize(n); }
    };

    struct Reference {                       // persistent, reset on activate
        Eigen::VectorXd tau_cmd;
        Eigen::VectorXd tau_start;
        Eigen::VectorXd q_anchor;
        Eigen::VectorXd q_ref;
        Eigen::VectorXd dq_ref;
        Eigen::VectorXd ddq_ref;
        Eigen::Vector3d F_tcp_ref;
        Eigen::Vector3d M_tcp_ref;
        void resize(size_t n) {
            tau_cmd.resize(n);
            tau_start.resize(n);
            q_anchor.resize(n);
            q_ref.resize(n);
            dq_ref.resize(n);
            ddq_ref.resize(n);
        }
    };

    struct Scratch {                        // per cycle only
        Eigen::VectorXd tau_new;
        Eigen::VectorXd tau_ff;
        Eigen::VectorXd tau_joint_damped;
        Eigen::VectorXd tau_breakaway;
        void resize(size_t n) {
            tau_new.resize(n);
            tau_ff.resize(n);
            tau_joint_damped.resize(n);
            tau_breakaway.resize(n); }
    };

    struct RobotarmData {
        Measured state;
        Reference ref;
        Scratch tmp;
        void resize(size_t n) { state.resize(n); ref.resize(n); tmp.resize(n); }
    };

    // GC: gravity compensation with joint damping and breakaway hold, references are ignored
    // IMPEDANCE: gravity compensation with joint damping and the cartesian impedance, no breakaway hold
    enum class Mode { GC, IMPEDANCE };

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CartesianImpedanceController)

    CartesianImpedanceController() = default;
    virtual ~CartesianImpedanceController() = default;

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
    bool compute_gc_torque();
    bool compute_impedance_torque();
    void declare_parameters();
    controller_interface::CallbackReturn read_parameters();
    // all joints or fail, for a valid start in on_activate
    bool fetch_robotarm_state_strict();
    // best effort, missed reads keep their last value
    void fetch_robotarm_state();

private:
    // custom rigid body dynamics class
    robotarm_rbd::RobotarmRbd rbd_;
    robotarm_rbd::RobotarmRbd::Config rbd_cfg_;
    Mode mode_ = Mode::GC;
    std::vector<std::string> rbd_joint_names_;
    std::vector<robotarm_rbd::RobotarmRbd::Limits> joint_limits_;  // from urdf, torque scaled by torque_scale_
    double torque_scale_ = 1.0;
    double torque_rise_time_ = 0;
    Eigen::VectorXd joint_damping_;   // Nm·s/rad, per joint in rbd joint order
    // breakaway hold, a spring to an anchor that is dragged along once the spring torque exceeds the
    // breakaway torque, i.e. virtual static friction band_width: eps_max = breakaway_tq/stiffness
    Eigen::VectorXd hold_stiffness_;          // Nm/rad
    Eigen::VectorXd hold_breakaway_torque_;   // Nm

    double ramp_time_ = 1.0;
    double ramp_elapsed_ = 0.0;

    RobotarmData data_;

    static constexpr size_t num_states_per_joint_ = 3;
    static constexpr size_t num_refs_per_joint_ = 3;
};

}

#endif