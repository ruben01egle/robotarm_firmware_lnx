#include "robotarm_controllers/CartesianJogController.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <memory>

PLUGINLIB_EXPORT_CLASS(
  cartesian_jog_controller::CartesianJogController, 
  controller_interface::ControllerInterface
)

namespace
{  // utility

void reset_controller_reference_msg(cartesian_jog_controller::CmdTypeTwist& msg)
{
    msg = cartesian_jog_controller::CmdTypeTwist();
}

bool is_finite(const geometry_msgs::msg::Vector3 & v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void clamp_twist(
    cartesian_jog_controller::CmdTypeTwist& msg,
    const cartesian_jog_controller::CartesianJogController::CartesianLimits& limits)
{
    msg.linear.x = std::clamp(msg.linear.x, -limits.linear.velocity, limits.linear.velocity);
    msg.linear.y = std::clamp(msg.linear.y, -limits.linear.velocity, limits.linear.velocity);
    msg.linear.z = std::clamp(msg.linear.z, -limits.linear.velocity, limits.linear.velocity);

    msg.angular.x = std::clamp(msg.angular.x, -limits.angular.velocity, limits.angular.velocity);
    msg.angular.y = std::clamp(msg.angular.y, -limits.angular.velocity, limits.angular.velocity);
    msg.angular.z = std::clamp(msg.angular.z, -limits.angular.velocity, limits.angular.velocity);
}

Eigen::Vector3d clamp_norm(const Eigen::Vector3d & v, double max)
{
    const double n = v.norm();
    return (n > max) ? Eigen::Vector3d(v * (max / n)) : v;
}


double velocity_scale(const Eigen::VectorXd& q_dot, const std::vector<robotarm_rbd::RobotarmRbd::Limits>& lim) {
    double s = 1.0;
    for (Eigen::Index i = 0; i < q_dot.size(); ++i)
        s = std::min(s, lim[i].velocity / std::max(std::abs(q_dot[i]), 1e-9));
    return s;
}

// one sided: only motion towards a position limit is slowed down, moving away is unrestricted.
// joints without position limits (max <= min, e.g. continuous) are skipped
double position_limit_scale(
    const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot,
    const std::vector<robotarm_rbd::RobotarmRbd::Limits>& lim, double zone, double margin)
{
    double s = 1.0;
    for (Eigen::Index i = 0; i < q.size(); ++i) {
        if (lim[i].max <= lim[i].min || std::abs(q_dot[i]) < 1e-9) continue;
        // distance to the limit the joint is moving towards, minus the safety margin
        const double d = (q_dot[i] > 0.0 ? lim[i].max - q[i] : q[i] - lim[i].min) - margin;
        // allowed joint speed ramps linearly from full at d >= zone down to 0 at d <= 0
        const double v_allowed = lim[i].velocity * std::clamp(d / zone, 0.0, 1.0);
        s = std::min(s, v_allowed / std::abs(q_dot[i]));
    }
    return s;
}


}  // namespace

namespace cartesian_jog_controller
{

controller_interface::InterfaceConfiguration CartesianJogController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : rbd_joint_names_)
    {
        command_interfaces_config.names.push_back(joint + "/position");
        command_interfaces_config.names.push_back(joint + "/velocity");
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CartesianJogController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    for (const auto & joint : rbd_joint_names_) {
        state_interfaces_config.names.push_back(joint + "/position");
        state_interfaces_config.names.push_back(joint + "/velocity");
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn CartesianJogController::on_init()
{
    try
    {
        declare_parameters();
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianJogController::on_configure(const rclcpp_lifecycle::State &/*previous_state*/)
{
    auto ret = read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    std::string urdf_string = get_robot_description();

    if (!rbd_.initialize(urdf_string, rbd_cfg_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd failed to init: %s", rbd_.last_error());
        return controller_interface::CallbackReturn::ERROR;
    }
    if (!rbd_.get_joint_names(rbd_joint_names_) ||
        !rbd_.get_joint_limits(joint_limits_) ||
        !rbd_.get_tcp_link_name(rbd_tcp_name_))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd not properly initialized: %s", rbd_.last_error());
        return controller_interface::CallbackReturn::ERROR;
    }
    // urdf velocity limits are hardware limits, way too fast for jogging
    for (auto & lim : joint_limits_) {
        lim.velocity *= joint_velocity_scale_;
    }
    data_.resize(rbd_joint_names_.size());

    twist_cmd_subscriber_ = get_node()->create_subscription<CmdTypeTwist>(
        "~/twist_cmds", rclcpp::SystemDefaultsQoS(),
        [this](const CmdTypeTwist::SharedPtr msg)
        {

        if (!is_finite(msg->linear) || !is_finite(msg->angular)) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                "Non-finite value received. Dropping message");
            return;
        }
        rt_command_twist_.set(*msg);
        });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianJogController::on_activate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    // make sure hw is in defined state when claiming interfaces
    if (!fetch_robotarm_state_strict()) return controller_interface::CallbackReturn::ERROR;

    data_.ref.q_cmd = data_.state.q;      // joint space: start from measured
    data_.ref.dq_cmd.setZero();
    for (size_t i = 0; i < rbd_joint_names_.size(); ++i)
    {
        size_t pos_idx = i * num_states_per_joint_ + 0;
        size_t vel_idx = i * num_states_per_joint_ + 1;

        (void)command_interfaces_[pos_idx].set_value(data_.state.q[i]);
        (void)command_interfaces_[vel_idx].set_value(0.0);
    }

    // update x_ref to q_cmd with fk
    if (!rbd_.calculate_link_transform(data_.ref.q_cmd, rbd_tcp_name_, data_.ref.x_ref)) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return controller_interface::CallbackReturn::ERROR;
    }

    ruckig_input_.control_interface = ruckig::ControlInterface::Velocity;
    ruckig_input_.synchronization = ruckig::Synchronization::None;
    for(size_t i=0; i<3; ++i) {
        ruckig_input_.max_velocity[i] = cartesian_limits_.linear.velocity;
        ruckig_input_.max_acceleration[i] = cartesian_limits_.linear.acceleration;
        ruckig_input_.max_jerk[i] = cartesian_limits_.linear.jerk;

        ruckig_input_.max_velocity[i+3] = cartesian_limits_.angular.velocity;
        ruckig_input_.max_acceleration[i+3] = cartesian_limits_.angular.acceleration;
        ruckig_input_.max_jerk[i+3] = cartesian_limits_.angular.jerk;
    }
    for(size_t i=0; i<6; ++i) {
        ruckig_input_.current_velocity[i] = 0;
        ruckig_input_.current_acceleration[i] = 0;

        ruckig_input_.target_velocity[i] = 0;
        ruckig_input_.target_acceleration[i] = 0;
    }

    reset_controller_reference_msg(twist_cmd_);
    rt_command_twist_.set(twist_cmd_);

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianJogController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

// TODO:
// - singularity detection and avoidance/slow down
// - watchdog
// - base/tcp switch
controller_interface::return_type CartesianJogController::update(const rclcpp::Time &/*time*/, const rclcpp::Duration &period)
{
    // get and validate input
    rt_command_twist_.try_get([this](const CmdTypeTwist & msg){twist_cmd_ = msg;});
    clamp_twist(twist_cmd_, cartesian_limits_);

    fetch_robotarm_state();

    // tracking check: commanded vs measured, catches blocked arm, collisions, drive faults, stale state
    for (Eigen::Index i = 0; i < data_.ref.q_cmd.size(); ++i) {
        const double dev = std::abs(data_.ref.q_cmd[i] - data_.state.q[i]);
        if (dev > max_tracking_error_) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Tracking error on joint %s: |q_cmd - q_meas| = %f > %f rad, stopping",
                         rbd_joint_names_[i].c_str(), dev, max_tracking_error_);
            return controller_interface::return_type::ERROR;
        }
    }

    if (!rbd_.calculate_jacobian_inverse(data_.ref.q_cmd, rbd_tcp_name_, data_.tmp.j_inv)) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return controller_interface::return_type::ERROR;
    }
    if (!rbd_.calculate_jacobian(data_.ref.q_cmd, rbd_tcp_name_, data_.tmp.j)) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return controller_interface::return_type::ERROR;
    }

    // predict and scale down motion before ruckig for smooth motion
    Eigen::Matrix<double, 6, 1> dx_target;
    dx_target << twist_cmd_.linear.x,  twist_cmd_.linear.y,  twist_cmd_.linear.z,
         twist_cmd_.angular.x, twist_cmd_.angular.y, twist_cmd_.angular.z;

    data_.tmp.dq_pred.noalias() = data_.tmp.j_inv * dx_target;
    const double scale_vel = velocity_scale(data_.tmp.dq_pred, joint_limits_);
    const double scale_pos = position_limit_scale(data_.ref.q_cmd, data_.tmp.dq_pred, joint_limits_,
                                                  position_limit_.zone, position_limit_.margin);
    dx_target *= std::min(scale_vel, scale_pos);

    // update cartesian space ruckig
    for(size_t i=0; i<6; ++i) {
        ruckig_input_.target_velocity[i] = dx_target[i];
    }

    ruckig_.delta_time = period.seconds();
    auto result = ruckig_.update(ruckig_input_, ruckig_output_);
    if (result < 0)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Ruckig: runtime error %d!", static_cast<int>(result));
        return controller_interface::return_type::ERROR;
    }

    // convert cartesian ruckig output to joint space
    Eigen::Matrix<double, 6, 1> dx_ff;
    for(size_t i=0; i<6; ++i) {
        dx_ff[i] = ruckig_output_.new_velocity[i];
    }

    // calc ik correction
    Eigen::Matrix<double, 6, 1> dx_ik_corr;
    Eigen::Isometry3d x_cur;
    if (!rbd_.calculate_link_transform(data_.ref.q_cmd, rbd_tcp_name_, x_cur)) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return controller_interface::return_type::ERROR;
    }
    // linear error
    dx_ik_corr.head<3>() = clamp_norm(correction_.linear.kp * (data_.ref.x_ref.translation() - x_cur.translation()),
                                      correction_.linear.max);
    // angular error
    Eigen::Matrix3d R_ik_err = data_.ref.x_ref.linear() * x_cur.linear().transpose();
    Eigen::AngleAxisd aa_ik_err(R_ik_err);
    dx_ik_corr.tail<3>() = clamp_norm(correction_.angular.kp * (aa_ik_err.angle() * aa_ik_err.axis()),
                                      correction_.angular.max);

    // apply ik drift correction
    Eigen::Matrix<double, 6, 1> dx_cmd = dx_ff + dx_ik_corr;

    // calculate dq_cmd and limit/scale joint space as last resort
    data_.ref.dq_cmd.noalias() = data_.tmp.j_inv * dx_cmd;
    double scale = velocity_scale(data_.ref.dq_cmd, joint_limits_);
    data_.ref.dq_cmd = scale * data_.ref.dq_cmd;
    // part of dx_ff the damped inverse actually realizes (J·J⁺·dx_ff). Near singularities the
    // rest cannot be followed and must not wind up in x_ref
    // tradeoff: the damping error of J⁺ is no longer corrected, but x_ref cannot run away
    Eigen::Matrix<double, 6, 1> dx_ff_realized = scale * data_.tmp.j * data_.tmp.j_inv * dx_ff;
    
    // update q_cmd and x_ref
    data_.ref.q_cmd += data_.ref.dq_cmd * period.seconds();

    data_.ref.x_ref.translation() += dx_ff_realized.head<3>() * period.seconds();
    Eigen::Vector3d dx_ref_rot = dx_ff_realized.tail<3>() * period.seconds();
    data_.ref.x_ref.linear() = Eigen::AngleAxisd(dx_ref_rot.norm(), dx_ref_rot.normalized()) * data_.ref.x_ref.linear();

    // hard position limit as last resort, should never trigger with the position limit scaling above
    // known limitation: the clamp is not represented in x_ref -> winds up while active,
    // acceptable because position_limit_scale should prevent it from ever triggering
    for (Eigen::Index i = 0; i < data_.ref.q_cmd.size(); ++i) {
        const auto & lim = joint_limits_[i];
        if (lim.max <= lim.min) continue;   // no position limits
        if (data_.ref.q_cmd[i] > lim.max || data_.ref.q_cmd[i] < lim.min) {
            data_.ref.q_cmd[i] = std::clamp(data_.ref.q_cmd[i], lim.min, lim.max);
            data_.ref.dq_cmd[i] = 0.0;
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                "Joint %s clamped to its hard position limit", rbd_joint_names_[i].c_str());
        }
    }

    // update ruckig for next cycle
    if (scale < 1.0) {
        for (size_t i = 0; i < 6; ++i) {
            ruckig_input_.current_velocity[i]     = scale * ruckig_output_.new_velocity[i];
            ruckig_input_.current_acceleration[i] = scale * ruckig_output_.new_acceleration[i];
        }
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000, 
                            "Joint space clipping: scaled to %f", scale);
    }
    else {
        ruckig_output_.pass_to_input(ruckig_input_);
    }

    for (size_t i = 0; i < rbd_joint_names_.size(); ++i)
    {
        size_t pos_idx = i * num_states_per_joint_ + 0;
        size_t vel_idx = i * num_states_per_joint_ + 1;

        (void)command_interfaces_[pos_idx].set_value(data_.ref.q_cmd[i]);
        (void)command_interfaces_[vel_idx].set_value(data_.ref.dq_cmd[i]);
    }

    return controller_interface::return_type::OK;
}

void CartesianJogController::declare_parameters()
{
    auto node = get_node();

    node->declare_parameter<double>("lambda", 0.01);

    node->declare_parameter<double>("max_linear_velocity", 0.1);        // in m/s
    node->declare_parameter<double>("max_linear_acceleration", 0.5);    // in m/s²
    node->declare_parameter<double>("max_linear_jerk", 5.0);            // in m/s³

    node->declare_parameter<double>("max_angular_velocity", 0.5);       // in rad/s
    node->declare_parameter<double>("max_angular_acceleration", 2.0);   // in rad/s²
    node->declare_parameter<double>("max_angular_jerk", 20.0);          // in rad/s³

    node->declare_parameter<double>("kp_linear", 10.0);                 // 1/s
    node->declare_parameter<double>("kp_angular", 10.0);                // 1/s
    node->declare_parameter<double>("max_correction_linear", 0.01);     // m/s
    node->declare_parameter<double>("max_correction_angular", 0.05);    // rad/s

    node->declare_parameter<double>("max_tracking_error", 0.1);         // rad, |q_cmd - q_meas| per joint

    node->declare_parameter<double>("joint_velocity_scale", 0.2);       // fraction of the urdf joint velocity limits

    node->declare_parameter<double>("joint_limit_zone", 0.2);           // rad, slow down zone in front of a position limit
    node->declare_parameter<double>("joint_limit_margin", 0.02);        // rad, stop this far before a position limit
}

controller_interface::CallbackReturn CartesianJogController::read_parameters()
{
    auto node = get_node();
    rbd_cfg_.lambda = node->get_parameter("lambda").as_double();

    cartesian_limits_.linear.velocity = node->get_parameter("max_linear_velocity").as_double();
    cartesian_limits_.linear.acceleration = node->get_parameter("max_linear_acceleration").as_double();
    cartesian_limits_.linear.jerk = node->get_parameter("max_linear_jerk").as_double();

    cartesian_limits_.angular.velocity = node->get_parameter("max_angular_velocity").as_double();
    cartesian_limits_.angular.acceleration = node->get_parameter("max_angular_acceleration").as_double();
    cartesian_limits_.angular.jerk = node->get_parameter("max_angular_jerk").as_double();

    correction_.linear.kp = node->get_parameter("kp_linear").as_double();
    correction_.angular.kp = node->get_parameter("kp_angular").as_double();
    correction_.linear.max = node->get_parameter("max_correction_linear").as_double();
    correction_.angular.max = node->get_parameter("max_correction_angular").as_double();

    max_tracking_error_ = node->get_parameter("max_tracking_error").as_double();
    if (!std::isfinite(max_tracking_error_) || max_tracking_error_ <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Parameter max_tracking_error must be finite and > 0, got %f",
                     max_tracking_error_);
        return controller_interface::CallbackReturn::ERROR;
    }

    joint_velocity_scale_ = node->get_parameter("joint_velocity_scale").as_double();
    if (!std::isfinite(joint_velocity_scale_) || joint_velocity_scale_ <= 0.0 || joint_velocity_scale_ > 1.0) {
        RCLCPP_ERROR(node->get_logger(), "Parameter joint_velocity_scale must be in (0, 1], got %f",
                     joint_velocity_scale_);
        return controller_interface::CallbackReturn::ERROR;
    }

    position_limit_.zone = node->get_parameter("joint_limit_zone").as_double();
    position_limit_.margin = node->get_parameter("joint_limit_margin").as_double();
    if (!std::isfinite(position_limit_.zone) || position_limit_.zone <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Parameter joint_limit_zone must be finite and > 0, got %f",
                     position_limit_.zone);
        return controller_interface::CallbackReturn::ERROR;
    }
    if (!std::isfinite(position_limit_.margin) || position_limit_.margin < 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Parameter joint_limit_margin must be finite and >= 0, got %f",
                     position_limit_.margin);
        return controller_interface::CallbackReturn::ERROR;
    }

    // sanity check correction params
    for (const auto & [name, value] : {
            std::pair{"kp_linear", correction_.linear.kp},
            std::pair{"kp_angular", correction_.angular.kp},
            std::pair{"max_correction_linear", correction_.linear.max},
            std::pair{"max_correction_angular", correction_.angular.max}})
    {
        if (!std::isfinite(value) || value < 0.0) {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s must be finite and >= 0, got %f", name, value);
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    // kp * dt >= 1 overshoots the error within one cycle and oscillates
    if (get_update_rate() == 0) {
        RCLCPP_ERROR(node->get_logger(), "Controller update rate is 0, cannot validate correction gains");
        return controller_interface::CallbackReturn::ERROR;
    }
    const double dt = 1.0 / static_cast<double>(get_update_rate());
    for (const auto & [name, kp] : {
            std::pair{"kp_linear", correction_.linear.kp},
            std::pair{"kp_angular", correction_.angular.kp}})
    {
        if (kp * dt >= 1.0) {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s = %f too high for update rate %u Hz, kp * dt must be < 1 (kp < %f)",
                         name, kp, get_update_rate(), 1.0 / dt);
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

bool CartesianJogController::fetch_robotarm_state_strict()
{
    for (size_t i = 0; i < rbd_joint_names_.size(); ++i)
    {
        size_t pos_idx = i * num_states_per_joint_ + 0;
        size_t vel_idx = i * num_states_per_joint_ + 1;

        auto pos_opt = state_interfaces_[pos_idx].get_optional();
        auto vel_opt = state_interfaces_[vel_idx].get_optional();

        if (!pos_opt.has_value() || !vel_opt.has_value())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Hardware-State for joint %s could not be read",
                         rbd_joint_names_[i].c_str());
            return false;
        }
        data_.state.q[i] = pos_opt.value();
        data_.state.dq[i] = vel_opt.value();
    }
    return true;
}

void CartesianJogController::fetch_robotarm_state()
{
    // best effort: a missed read (lock contention) keeps the value of the last successful read,
    // a persistently stale state is caught by the deviation check
    for (size_t i = 0; i < rbd_joint_names_.size(); ++i)
    {
        size_t pos_idx = i * num_states_per_joint_ + 0;
        size_t vel_idx = i * num_states_per_joint_ + 1;

        auto pos_opt = state_interfaces_[pos_idx].get_optional();
        auto vel_opt = state_interfaces_[vel_idx].get_optional();

        if (pos_opt.has_value()) data_.state.q[i] = pos_opt.value();
        if (vel_opt.has_value()) data_.state.dq[i] = vel_opt.value();

        if (!pos_opt.has_value() || !vel_opt.has_value())
        {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                "Hardware-State for joint %s could not be read, keeping last value",
                                rbd_joint_names_[i].c_str());
        }
    }
}

}