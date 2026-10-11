#include "robotarm_controllers/CartesianImpedanceController.hpp"

#include <algorithm>

PLUGINLIB_EXPORT_CLASS(
  cartesian_impedance_controller::CartesianImpedanceController, 
  controller_interface::ControllerInterface
)

namespace
{
    bool clamp_torque(
        Eigen::VectorXd& t,
        const std::vector<robotarm_rbd::RobotarmRbd::Limits>& limits)
    {
        if (!t.allFinite()) return false;
        if (static_cast<size_t>(t.size()) != limits.size()) return false;
        for (Eigen::Index i = 0; i < t.size(); ++i) {
            t(i) = std::clamp(t(i), -limits[i].effort, limits[i].effort);
        }
        return true;
    }

    // per joint array parameter, every entry finite and >= 0. The length is checked in on_configure,
    // the dof is only known after rbd init
    bool read_non_negative_joint_array(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
        const std::string& name,
        Eigen::VectorXd& out)
    {
        const std::vector<double> values = node->get_parameter(name).as_double_array();
        for (size_t i = 0; i < values.size(); ++i) {
            if (!std::isfinite(values[i]) || values[i] < 0.0) {
                RCLCPP_ERROR(node->get_logger(), "Parameter %s[%zu] must be finite and >= 0, got %f",
                             name.c_str(), i, values[i]);
                return false;
            }
        }
        out = Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
        return true;
    }
}

namespace cartesian_impedance_controller
{

controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : rbd_joint_names_)
    {
        command_interfaces_config.names.push_back(joint + "/effort");
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    for (const auto & joint : rbd_joint_names_) {
        state_interfaces_config.names.push_back(joint + "/position");
        state_interfaces_config.names.push_back(joint + "/velocity");
        state_interfaces_config.names.push_back(joint + "/effort");
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_init()
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

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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
        !rbd_.get_joint_limits(joint_limits_))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd not properly initialized: %s", rbd_.last_error());
        return controller_interface::CallbackReturn::ERROR;
    }

    // scale urdf torque limits for testing
    for (auto & lim : joint_limits_) {
        lim.effort *= torque_scale_;
    }

    const std::pair<const char*, const Eigen::VectorXd*> joint_arrays[] = {
        {"joint_damping", &joint_damping_},
        {"hold_stiffness", &hold_stiffness_},
        {"hold_breakaway_torque", &hold_breakaway_torque_},
    };
    for (const auto & [name, values] : joint_arrays) {
        if (static_cast<size_t>(values->size()) != rbd_.get_dof()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter %s needs one entry per joint (%zu), got %ld",
                         name, rbd_.get_dof(), values->size());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    // a breakaway torque above the torque clamp can never be reached, the hold would never slide
    for (size_t i = 0; i < rbd_.get_dof(); ++i) {
        if (hold_breakaway_torque_[i] > joint_limits_[i].effort) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Parameter hold_breakaway_torque[%zu] = %f exceeds the scaled torque limit %f of joint %s",
                         i, hold_breakaway_torque_[i], joint_limits_[i].effort, rbd_joint_names_[i].c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    data_.resize(rbd_.get_dof());

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    // make sure hw is in defined state when claiming interfaces
    if (!fetch_robotarm_state_strict()) return controller_interface::CallbackReturn::ERROR;

    data_.ref.q_anchor = data_.state.q;
    data_.ref.tau_start = data_.state.tau;
    ramp_elapsed_ = 0;

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    fetch_robotarm_state();

    for (size_t i=0; i<rbd_.get_dof(); ++i) {
        if (hold_stiffness_[i] <= 0.0 || hold_breakaway_torque_[i] <= 0.0) continue;
        double eps_max = hold_breakaway_torque_[i]/hold_stiffness_[i];
        if (data_.state.q(i) - data_.ref.q_anchor(i) > eps_max) {
            data_.ref.q_anchor(i) = data_.state.q(i) - eps_max;
        }
        else if (data_.state.q(i) - data_.ref.q_anchor(i) < -eps_max) {
            data_.ref.q_anchor(i) = data_.state.q(i) + eps_max;
        }
    }

    data_.ref.dq_ref.setZero();
    data_.ref.ddq_ref.setZero();
    data_.ref.F_tcp_ref.setZero();
    data_.ref.M_tcp_ref.setZero();

    if (!rbd_.recursive_newton_euler(
            data_.state.q,
            data_.ref.dq_ref,
            data_.ref.ddq_ref,
            data_.tmp.tau_ff,
            data_.ref.F_tcp_ref,
            data_.ref.M_tcp_ref)) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return controller_interface::return_type::ERROR;
    }
    data_.tmp.tau_joint_damped.noalias() = - joint_damping_.cwiseProduct(data_.state.dq);
    data_.tmp.tau_breakaway.noalias() = - hold_stiffness_.cwiseProduct(data_.state.q - data_.ref.q_anchor);
    data_.ref.tau_cmd.noalias() = data_.tmp.tau_ff + data_.tmp.tau_joint_damped + data_.tmp.tau_breakaway;

    // smooth ramp in on startup
    if (ramp_elapsed_ < ramp_time_) {
        data_.ref.tau_cmd.noalias() = (data_.ref.tau_cmd - data_.ref.tau_start)/ramp_time_*ramp_elapsed_ + data_.ref.tau_start;
        ramp_elapsed_ += period.seconds();
    }

    if (!clamp_torque(data_.ref.tau_cmd, joint_limits_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error clamping torque: nan or invalid limits");
        return controller_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < rbd_.get_dof(); ++i)
    {
        (void)command_interfaces_[i].set_value(data_.ref.tau_cmd[i]);
    }

    return controller_interface::return_type::OK;
}

void CartesianImpedanceController::declare_parameters()
{
    auto node = get_node();

    node->declare_parameter<double>("lambda", 0.01);
    node->declare_parameter<std::string>("jinv_method", "svd");         // "svd" or "ldlt"

    node->declare_parameter<double>("torque_scale", 1.0);
    node->declare_parameter<std::vector<double>>("joint_damping", std::vector<double>());
    node->declare_parameter<std::vector<double>>("hold_stiffness", std::vector<double>());
    node->declare_parameter<std::vector<double>>("hold_breakaway_torque", std::vector<double>());
}

controller_interface::CallbackReturn CartesianImpedanceController::read_parameters()
{
    auto node = get_node();
    rbd_cfg_.lambda = node->get_parameter("lambda").as_double();

    // "svd": selective, adaptive damping (accurate), "ldlt": constant damping (fast)
    const std::string jinv_method = node->get_parameter("jinv_method").as_string();
    if (jinv_method == "svd") {
        rbd_cfg_.jinv_method = robotarm_rbd::RobotarmRbd::JinvMethod::SVD;
    } else if (jinv_method == "ldlt") {
        rbd_cfg_.jinv_method = robotarm_rbd::RobotarmRbd::JinvMethod::LDLT;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Parameter jinv_method must be 'svd' or 'ldlt', got '%s'",
                     jinv_method.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    torque_scale_ = node->get_parameter("torque_scale").as_double();
    if (!std::isfinite(torque_scale_) || torque_scale_ <= 0.0 || torque_scale_ > 1.0) {
        RCLCPP_ERROR(node->get_logger(), "Parameter torque_scale must be in (0, 1], got %f",
                     torque_scale_);
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!read_non_negative_joint_array(node, "joint_damping", joint_damping_) ||
        !read_non_negative_joint_array(node, "hold_stiffness", hold_stiffness_) ||
        !read_non_negative_joint_array(node, "hold_breakaway_torque", hold_breakaway_torque_)) {
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

bool CartesianImpedanceController::fetch_robotarm_state_strict()
{
    for (size_t i = 0; i < rbd_.get_dof(); ++i)
    {
        size_t pos_idx = i * num_states_per_joint_ + 0;
        size_t vel_idx = i * num_states_per_joint_ + 1;
        size_t eff_idx = i * num_states_per_joint_ + 2;

        auto pos_opt = state_interfaces_[pos_idx].get_optional();
        auto vel_opt = state_interfaces_[vel_idx].get_optional();
        auto eff_opt = state_interfaces_[eff_idx].get_optional();

        if (!pos_opt.has_value() || !vel_opt.has_value() || !eff_opt.has_value())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Hardware-State for joint %s could not be read",
                         rbd_joint_names_[i].c_str());
            return false;
        }
        data_.state.q[i] = pos_opt.value();
        data_.state.dq[i] = vel_opt.value();
        data_.state.tau[i] = eff_opt.value();
    }
    return true;
}

void CartesianImpedanceController::fetch_robotarm_state()
{
    // best effort: a missed read (lock contention) keeps the value of the last successful read,
    // a persistently stale state is caught by the deviation check
    for (size_t i = 0; i < rbd_.get_dof(); ++i)
    {
        size_t pos_idx = i * num_states_per_joint_ + 0;
        size_t vel_idx = i * num_states_per_joint_ + 1;
        size_t eff_idx = i * num_states_per_joint_ + 2;

        auto pos_opt = state_interfaces_[pos_idx].get_optional();
        auto vel_opt = state_interfaces_[vel_idx].get_optional();
        auto eff_opt = state_interfaces_[eff_idx].get_optional();

        if (pos_opt.has_value()) data_.state.q[i] = pos_opt.value();
        if (vel_opt.has_value()) data_.state.dq[i] = vel_opt.value();
        if (eff_opt.has_value()) data_.state.tau[i] = eff_opt.value();

        if (!pos_opt.has_value() || !vel_opt.has_value() || !eff_opt.has_value())
        {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                "Hardware-State for joint %s could not be read, keeping last value",
                                rbd_joint_names_[i].c_str());
        }
    }
}

}