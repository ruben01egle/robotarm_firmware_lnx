#ifndef ROBOTARM_CONTROLLERS_CARTESIANJOGCONTROLLER_HPP
#define ROBOTARM_CONTROLLERS_CARTESIANJOGCONTROLLER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/subscription.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "controller_interface/controller_interface.hpp"

#include <string>
#include <vector>
#include <ruckig/ruckig.hpp>
#include <chrono>
#include <Eigen/Geometry>

#include "robotarm_interface/msg/cartesian_increment.hpp"
#include "robotarm_rbd/RobotarmRbd.hpp"

namespace cartesian_jog_controller
{

class CartesianJogController : public controller_interface::ControllerInterface
{
public:
    struct Measured {
        Eigen::VectorXd q, dq;
        void resize(size_t n) { q.resize(n); dq.resize(n); }
    };

    struct Reference {                       // persistent, reset on activate
        Eigen::VectorXd q_cmd;
        Eigen::VectorXd dq_cmd;
        Eigen::Isometry3d x_ref = Eigen::Isometry3d::Identity();
        void resize(size_t n) { q_cmd.resize(n); dq_cmd.resize(n); }
    };

    struct Scratch {                        // per cycle only
        Eigen::VectorXd dq_pred;
        Eigen::Matrix<double, 6, Eigen::Dynamic> j;
        Eigen::Matrix<double, Eigen::Dynamic, 6> j_inv;
        void resize(size_t n) { dq_pred.resize(n); j.resize(6, n); j_inv.resize(n, 6); }
    };

    struct RobotarmData {
        Measured state;
        Reference ref;
        Scratch  tmp;
        void resize(size_t n) { state.resize(n); ref.resize(n); tmp.resize(n); }
    };

    struct CartesianLimits
    {
        struct Bounds { double velocity, acceleration, jerk; };
        Bounds linear;   // m/s, m/s², m/s³
        Bounds angular;  // rad/s, rad/s², rad/s³
    };

    // P-correction of the drift between x_ref and FK(q_cmd)
    struct CorrectionParams
    {
        struct Gains { double kp, max; };
        Gains linear;    // kp in 1/s, max in m/s
        Gains angular;   // kp in 1/s, max in rad/s
    };

    // one sided slow down in front of joint position limits
    struct PositionLimitParams
    {
        double zone = 0.0;      // rad, width of the slow down zone
        double margin = 0.0;    // rad, stop distance before the limit
    };

    struct IncrementLimit
    {
        double linear = 0.0;        // m
        double angular = 0.0;       // rad
    };

    enum class Frame { BASE, TOOL };
    enum class Mode { JOG, INCREMENT };

    // local timestamp to each twist cmd for watchdog
    struct TwistCmd
    {
        geometry_msgs::msg::Twist twist{};     // no string (frame_id), copied in update()
        Frame frame = Frame::BASE;
        size_t seq = 0;
    };
    struct TwistCmdStamped
    {
        TwistCmd cmd;
        rclcpp::Time stamp;
    };

    struct PendingIncrementCmd
    {
        Eigen::Matrix<double, 6, 1> base = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 1> tool = Eigen::Matrix<double, 6, 1>::Zero();
    };

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CartesianJogController)

    CartesianJogController() = default;
    virtual ~CartesianJogController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    void watchdog(const rclcpp::Time &time);
    void start_increment_run(Frame frame);
    void end_increment_run();
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
    std::vector<robotarm_rbd::RobotarmRbd::Limits> joint_limits_;  // from urdf, velocity scaled by joint_velocity_scale_
    std::vector<std::string> rbd_joint_names_;
    std::string rbd_base_name_;
    std::string rbd_tcp_name_;

    RobotarmData data_;

    // cartesian space ruckig
    ruckig::Ruckig<6> ruckig_;
    ruckig::InputParameter<6> ruckig_input_;
    ruckig::OutputParameter<6> ruckig_output_;
    Frame ruckig_frame_ = Frame::BASE;

    CartesianLimits cartesian_limits_;
    CorrectionParams correction_;
    double max_tracking_error_ = 0.0;   // rad, max |q_cmd - q_meas| per joint before stopping
    double joint_velocity_scale_ = 1.0; // fraction of the urdf joint velocity limits used while jogging
    PositionLimitParams position_limit_;
    IncrementLimit increment_limits_;

    Mode mode_ = Mode::JOG;

    // sub and rt container for twist cmd
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_subscriber_;
    realtime_tools::RealtimeThreadSafeBox<TwistCmd> rt_command_twist_;
    TwistCmdStamped twist_cmd_;
    rclcpp::Duration twist_cmd_timeout_ = {std::chrono::milliseconds(250)};

    //sub and rt container for increment cmd
    rclcpp::Subscription<robotarm_interface::msg::CartesianIncrement>::SharedPtr inc_cmd_subscriber_;
    realtime_tools::RealtimeThreadSafeBox<PendingIncrementCmd> rt_command_inc_;

    static constexpr size_t num_states_per_joint_ = 2;
    static constexpr double min_increment_scale_ = 0.01;    // below: increment run aborted in front of a joint limit
};

}

#endif