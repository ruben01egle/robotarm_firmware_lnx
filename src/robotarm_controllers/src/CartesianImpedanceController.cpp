#include "robotarm_controllers/CartesianImpedanceController.hpp"

#include <algorithm>
#include <limits>

PLUGINLIB_EXPORT_CLASS(
  cartesian_impedance_controller::CartesianImpedanceController, 
  controller_interface::ChainableControllerInterface
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

    bool clamp_torque_rate(
        Eigen::VectorXd& t_new,
        const Eigen::VectorXd& t,
        const std::vector<robotarm_rbd::RobotarmRbd::Limits>& limits,
        const double dt,
        const double rise_time,
        bool* clamped = nullptr)
    {
        if (clamped != nullptr) *clamped = false;

        if (!t_new.allFinite() || !t.allFinite()) return false;
        if (t_new.size() != t.size()) return false;
        if (static_cast<size_t>(t.size()) != limits.size()) return false;

        if (rise_time <= 0) return true;
        for (Eigen::Index i = 0; i < t.size(); ++i) {
            const double max_step = limits[i].effort / rise_time * dt;
            const double t_new_i_clamped = std::clamp(t_new(i), t(i) - max_step, t(i) + max_step);
            if (t_new_i_clamped != t_new(i)) {
                t_new(i) = t_new_i_clamped;
                if (clamped != nullptr) *clamped = true;
            }
        }
        return true;
    }

    Eigen::Vector3d clamp_norm(const Eigen::Vector3d & v, double max)
    {
        const double n = v.norm();
        return (n > max) ? Eigen::Vector3d(v * (max / n)) : v;
    }

    // array parameter, every entry finite and >= 0. The length is checked by the caller, for per joint arrays
    // in on_configure, the dof is only known after rbd init
    bool read_non_negative_array(
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
        !rbd_.get_joint_limits(joint_limits_)   ||
        !rbd_.get_tcp_link_name(rbd_tcp_name_))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd not properly initialized: %s", rbd_.last_error());
        return controller_interface::CallbackReturn::ERROR;
    }

    // scale urdf torque limits for testing
    for (auto & lim : joint_limits_) {
        lim.effort *= torque_scale_;
    }

    // the breakaway hold only runs in gc mode
    const bool needs_hold = (mode_ == Mode::GC);
    const struct { const char* name; const Eigen::VectorXd* values; bool required; } joint_arrays[] = {
        {"joint_damping", &joint_damping_, true},
        {"joint_limit_zone", &joint_limit_zone_, true},
        {"hold_stiffness", &gc_.hold_stiffness, needs_hold},
        {"hold_breakaway_torque", &gc_.hold_breakaway_torque, needs_hold},
    };
    for (const auto & [name, values, required] : joint_arrays) {
        if (!required) continue;
        if (static_cast<size_t>(values->size()) != rbd_.get_dof()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter %s needs one entry per joint (%zu), got %ld",
                         name, rbd_.get_dof(), values->size());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    // the upper and lower zone of a joint must not overlap
    for (size_t i = 0; i < rbd_.get_dof(); ++i) {
        const auto & lim = joint_limits_[i];
        if (lim.max <= lim.min) continue;   // no position limits
        if (2.0 * joint_limit_zone_[i] >= lim.max - lim.min) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Parameter joint_limit_zone[%zu] = %f must be less than half the range %f of joint %s",
                         i, joint_limit_zone_[i], lim.max - lim.min, rbd_joint_names_[i].c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    // a breakaway torque above the torque clamp can never be reached, the hold would never slide
    for (size_t i = 0; needs_hold && i < rbd_.get_dof(); ++i) {
        if (gc_.hold_breakaway_torque[i] > joint_limits_[i].effort) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Parameter hold_breakaway_torque[%zu] = %f exceeds the scaled torque limit %f of joint %s",
                         i, gc_.hold_breakaway_torque[i], joint_limits_[i].effort, rbd_joint_names_[i].c_str());
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
    data_.ref.q_ref = data_.state.q;
    // drop references of a previous activation, NaN holds q_ref until an upstream controller writes
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(),
              std::numeric_limits<double>::quiet_NaN());
    data_.ref.tau_start = data_.state.tau;
    data_.ref.tau_cmd = data_.state.tau;
    ramp_elapsed_ = 0;

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> CartesianImpedanceController::on_export_reference_interfaces()
{
    // per joint [position, velocity, acceleration], same layout as the upstream controllers write it.
    // NaN until an upstream controller writes them
    reference_interfaces_.assign(rbd_joint_names_.size() * num_refs_per_joint_,
                                 std::numeric_limits<double>::quiet_NaN());

    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(reference_interfaces_.size());
    for (size_t i = 0; i < rbd_joint_names_.size(); ++i) {
        const std::string & joint = rbd_joint_names_[i];
        reference_interfaces.emplace_back(get_node()->get_name(), joint + "/position",
                                          &reference_interfaces_[i * num_refs_per_joint_ + 0]);
        reference_interfaces.emplace_back(get_node()->get_name(), joint + "/velocity",
                                          &reference_interfaces_[i * num_refs_per_joint_ + 1]);
        reference_interfaces.emplace_back(get_node()->get_name(), joint + "/acceleration",
                                          &reference_interfaces_[i * num_refs_per_joint_ + 2]);
    }
    return reference_interfaces;
}

controller_interface::return_type CartesianImpedanceController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // no standalone input yet, the references keep their values
    return controller_interface::return_type::OK;
}

controller_interface::return_type CartesianImpedanceController::update_and_write_commands(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    fetch_robotarm_state();

    // always compute gravity compensation to data_.tmp.tau_gravity
    if (!rbd_.recursive_newton_euler(
            data_.state.q,
            data_.tmp.zero,
            data_.tmp.zero,
            data_.tmp.tau_gravity,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero())) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return controller_interface::return_type::ERROR;
    }

    // write mode torque into data_.tmp.tau_mode
    switch (mode_)
    {
    case Mode::GC:
        if (!compute_gc_torque()) return controller_interface::return_type::ERROR;
        break;

    case Mode::IMPEDANCE:
        if (!compute_impedance_torque()) return controller_interface::return_type::ERROR;
        break;
    }

    // joint limit protection: inside the limit zone scale mode torque towards limit
    // + spring pushes outwards with up to joint_limit_push · effort
    data_.tmp.tau_limiter.setZero();
    for (size_t i = 0; i < rbd_.get_dof(); ++i) {
        const auto & lim = joint_limits_[i];
        const double zone = joint_limit_zone_[i];
        if (lim.max <= lim.min || zone <= 0.0) continue;   // no position limits or protection disabled

        // 0 outside the zone, 1 at (and beyond) the limit
        const double s_upper = std::clamp((data_.state.q[i] - (lim.max - zone)) / zone, 0.0, 1.0);
        const double s_lower = std::clamp(((lim.min + zone) - data_.state.q[i]) / zone, 0.0, 1.0);

        // scale only torque towards the limit
        if (s_upper > 0.0) {
            if (data_.tmp.tau_mode[i] > 0.0) data_.tmp.tau_mode[i] *= (1.0 - s_upper);
            data_.tmp.tau_limiter[i] = - s_upper * joint_limit_push_ * lim.effort;
        }
        else if (s_lower > 0.0) {
            if (data_.tmp.tau_mode[i] < 0.0) data_.tmp.tau_mode[i] *= (1.0 - s_lower);
            data_.tmp.tau_limiter[i] = s_lower * joint_limit_push_ * lim.effort;
        }
    }

    // compute tau new
    data_.tmp.tau_new.noalias() = data_.tmp.tau_gravity + data_.tmp.tau_mode + data_.tmp.tau_limiter;

    // smooth ramp in on startup
    if (ramp_elapsed_ < ramp_time_) {
        data_.tmp.tau_new.noalias() = (data_.tmp.tau_new - data_.ref.tau_start)/ramp_time_*ramp_elapsed_ + data_.ref.tau_start;
        ramp_elapsed_ += period.seconds();
    }

    bool rate_limited = false;
    if (!clamp_torque_rate(data_.tmp.tau_new,
                           data_.ref.tau_cmd,
                           joint_limits_,
                           period.seconds(),
                           torque_rise_time_,
                           &rate_limited)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error clamping torque rate: nan, invalid limits or length mismatch");
        return controller_interface::return_type::ERROR;
    }
    if (rate_limited) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                            "Torque rate limited");
    }
    if (!clamp_torque(data_.tmp.tau_new, joint_limits_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error clamping torque: nan or invalid limits");
        return controller_interface::return_type::ERROR;
    }
    data_.ref.tau_cmd = data_.tmp.tau_new;

    for (size_t i = 0; i < rbd_.get_dof(); ++i)
    {
        (void)command_interfaces_[i].set_value(data_.ref.tau_cmd[i]);
    }

    return controller_interface::return_type::OK;
}

bool CartesianImpedanceController::compute_gc_torque()
{
    for (size_t i=0; i<rbd_.get_dof(); ++i) {
        if (gc_.hold_stiffness[i] <= 0.0 || gc_.hold_breakaway_torque[i] <= 0.0) continue;
        double eps_max = gc_.hold_breakaway_torque[i]/gc_.hold_stiffness[i];
        if (data_.state.q(i) - data_.ref.q_anchor(i) > eps_max) {
            data_.ref.q_anchor(i) = data_.state.q(i) - eps_max;
        }
        else if (data_.state.q(i) - data_.ref.q_anchor(i) < -eps_max) {
            data_.ref.q_anchor(i) = data_.state.q(i) + eps_max;
        }
    }

    data_.tmp.tau_joint_damped.noalias() = - joint_damping_.cwiseProduct(data_.state.dq);
    data_.tmp.tau_breakaway.noalias() = - gc_.hold_stiffness.cwiseProduct(data_.state.q - data_.ref.q_anchor);
    data_.tmp.tau_mode.noalias() = data_.tmp.tau_joint_damped + data_.tmp.tau_breakaway;
    return true;
}

bool CartesianImpedanceController::compute_impedance_torque()
{
    // update references
    fetch_references();

    // compute intermediate values 
    if (!rbd_.calculate_jacobian(data_.state.q, rbd_tcp_name_, data_.tmp.j)){
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return false;
    }
    if (!rbd_.calculate_jacobian(data_.ref.q_ref, rbd_tcp_name_, data_.tmp.j_ref)){
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return false;
    }
    Eigen::Isometry3d T;
    if (!rbd_.calculate_link_transform(data_.state.q, rbd_tcp_name_, T)){
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return false;
    }
    Eigen::Isometry3d T_ref;
    if (!rbd_.calculate_link_transform(data_.ref.q_ref, rbd_tcp_name_, T_ref)){
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return false;
    }

    // compute error
    Eigen::Matrix<double, 6, 1> e;
    // translation error
    e.head<3>() = T.translation() - T_ref.translation();
    // orientation error
    Eigen::Matrix3d R_err = T.linear() * T_ref.linear().transpose();
    Eigen::AngleAxisd aa_err(R_err);
    e.tail<3>() = aa_err.angle() * aa_err.axis();

    Eigen::Matrix<double, 6, 1> de;
    de.noalias() = data_.tmp.j * data_.state.dq - data_.tmp.j_ref * data_.ref.dq_ref;

    // compute damping
    // Λ = L·Lᵀ
    Eigen::Matrix<double, 6, 6> Lambda;
    if (!rbd_.calculate_task_space_inertia(
            data_.state.q,
            rbd_tcp_name_,
            Lambda)) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return false;
    }
    const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt_lambda(Lambda);
    if (llt_lambda.info() != Eigen::Success) {
		RCLCPP_ERROR(get_node()->get_logger(), "Task space inertia is not positive definite");
        return false;
	}
    Eigen::Matrix<double, 6, 6> L = llt_lambda.matrixL();
    const Eigen::Matrix<double, 6, 6> L_inv = L.triangularView<Eigen::Lower>().solve(Eigen::Matrix<double, 6, 6>::Identity());

    // K̃ = L⁻¹·K·L⁻ᵀ, then symmetrize (rounding)
    const auto& K = impedance_.stiffness.asDiagonal();
    Eigen::Matrix<double, 6, 6> K_tilde = L_inv * K * L_inv.transpose();
    K_tilde = 0.5 * (K_tilde + K_tilde.transpose()).eval();   // eval: K_tilde is read and written

    // Eigenvalues of K̃
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(K_tilde);
    const Eigen::Vector<double, 6> k0 = eig.eigenvalues().cwiseMax(0.0);
    const Eigen::Matrix<double, 6, 6> V  = eig.eigenvectors();

    // damp each 1d oscillator
    const auto& zeta = impedance_.damping_ratio;
    const Eigen::Vector<double, 6> d_w = 2.0*k0.cwiseSqrt()*zeta;

    // transform back to cartesian coordinates
    const Eigen::Matrix<double, 6, 6> LV = L * V;
    const Eigen::Matrix<double, 6, 6> D = LV * d_w.asDiagonal() * LV.transpose();

    // compute impedance
    Eigen::Matrix<double, 6, 1> F_spring = - impedance_.stiffness.cwiseProduct(e);
    F_spring.head<3>() = clamp_norm(F_spring.head<3>(), impedance_.max_spring_force);
    F_spring.tail<3>() = clamp_norm(F_spring.tail<3>(), impedance_.max_spring_moment);
    Eigen::Matrix<double, 6, 1> F_damper = - D * de;

    // compute tau
    if (!rbd_.recursive_newton_euler(
            data_.state.q,
            data_.state.dq,
            data_.ref.ddq_ref,
            data_.tmp.tau_dyn_ff,
            Eigen::Vector3d::Zero(),     // no F_ext
            Eigen::Vector3d::Zero(),     // no M_ext
            Eigen::Vector3d::Zero())) {  // no gravity, only dynamics
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return false;
    }
    data_.tmp.tau_impedance.noalias() = data_.tmp.j.transpose() * (F_spring + F_damper);
    data_.tmp.tau_joint_damped.noalias() = - joint_damping_.cwiseProduct(data_.state.dq - data_.ref.dq_ref);
    data_.tmp.tau_mode.noalias() = data_.tmp.tau_dyn_ff + data_.tmp.tau_impedance + data_.tmp.tau_joint_damped;
    return true;
}

void CartesianImpedanceController::declare_parameters()
{
    auto node = get_node();

    node->declare_parameter<std::string>("mode", "gc");                 // "gc" or "impedance"

    node->declare_parameter<double>("lambda", 0.01);
    node->declare_parameter<std::string>("jinv_method", "svd");         // "svd" or "ldlt"

    node->declare_parameter<double>("torque_scale", 1.0);
    node->declare_parameter<double>("torque_rise_time", 0.0);
    node->declare_parameter<std::vector<double>>("joint_damping", std::vector<double>());
    node->declare_parameter<std::vector<double>>("hold_stiffness", std::vector<double>());
    node->declare_parameter<std::vector<double>>("hold_breakaway_torque", std::vector<double>());

    // both modes, per joint
    node->declare_parameter<std::vector<double>>("joint_limit_zone", std::vector<double>());
    node->declare_parameter<double>("joint_limit_push", 0.0);

    // impedance mode only, [x, y, z, rx, ry, rz] in the base frame
    node->declare_parameter<std::vector<double>>("cartesian_stiffness", std::vector<double>());
    node->declare_parameter<double>("cartesian_damping_ratio", 0.0);
    node->declare_parameter<double>("max_spring_force", 0.0);
    node->declare_parameter<double>("max_spring_moment", 0.0);
}

controller_interface::CallbackReturn CartesianImpedanceController::read_parameters()
{
    auto node = get_node();

    const std::string mode = node->get_parameter("mode").as_string();
    if (mode == "gc") {
        mode_ = Mode::GC;
    } else if (mode == "impedance") {
        mode_ = Mode::IMPEDANCE;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Parameter mode must be 'gc' or 'impedance', got '%s'",
                     mode.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

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

    torque_rise_time_ = node->get_parameter("torque_rise_time").as_double();
    if (!std::isfinite(torque_rise_time_) || torque_rise_time_ < 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Parameter torque_rise_time must be >= 0, got %f",
                     torque_rise_time_);
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!read_non_negative_array(node, "joint_damping", joint_damping_) ||
        !read_non_negative_array(node, "joint_limit_zone", joint_limit_zone_) ||
        !read_non_negative_array(node, "hold_stiffness", gc_.hold_stiffness) ||
        !read_non_negative_array(node, "hold_breakaway_torque", gc_.hold_breakaway_torque)) {
        return controller_interface::CallbackReturn::ERROR;
    }

    joint_limit_push_ = node->get_parameter("joint_limit_push").as_double();
    if (!std::isfinite(joint_limit_push_) || joint_limit_push_ < 0.0 || joint_limit_push_ > 1.0) {
        RCLCPP_ERROR(node->get_logger(), "Parameter joint_limit_push must be in [0, 1], got %f",
                     joint_limit_push_);
        return controller_interface::CallbackReturn::ERROR;
    }

    Eigen::VectorXd cartesian_stiffness;
    if (!read_non_negative_array(node, "cartesian_stiffness", cartesian_stiffness)) {
        return controller_interface::CallbackReturn::ERROR;
    }

    const double cartesian_damping_ratio = node->get_parameter("cartesian_damping_ratio").as_double();
    if (!std::isfinite(cartesian_damping_ratio) || cartesian_damping_ratio < 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Parameter cartesian_damping_ratio must be finite and >= 0, got %f",
                     cartesian_damping_ratio);
        return controller_interface::CallbackReturn::ERROR;
    }

    const std::pair<const char*, double*> spring_limits[] = {
        {"max_spring_force", &impedance_.max_spring_force},
        {"max_spring_moment", &impedance_.max_spring_moment},
    };
    for (const auto & [name, value] : spring_limits) {
        *value = node->get_parameter(name).as_double();
        if (!std::isfinite(*value) || *value < 0.0) {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s must be finite and >= 0, got %f", name, *value);
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    switch (mode_)
    {
    case Mode::GC:
        if (cartesian_stiffness.size() > 0 || cartesian_damping_ratio > 0.0 ||
            impedance_.max_spring_force > 0.0 || impedance_.max_spring_moment > 0.0) {
            RCLCPP_WARN(node->get_logger(), "Cartesian impedance parameters are ignored in gc mode");
        }
        impedance_ = ImpedanceParams{};
        break;

    case Mode::IMPEDANCE: {
        if (cartesian_stiffness.size() != 6) {
            RCLCPP_ERROR(node->get_logger(),
                         "Parameter cartesian_stiffness needs 6 entries [x, y, z, rx, ry, rz] in impedance mode, got %ld",
                         cartesian_stiffness.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        for (const auto & [name, value] : spring_limits) {
            if (*value <= 0.0) {
                RCLCPP_ERROR(node->get_logger(), "Parameter %s must be > 0 in impedance mode, got %f", name, *value);
                return controller_interface::CallbackReturn::ERROR;
            }
        }
        impedance_.stiffness = cartesian_stiffness;
        impedance_.damping_ratio = cartesian_damping_ratio;

        if (impedance_.damping_ratio == 0.0 && (impedance_.stiffness.array() > 0.0).any()) {
            RCLCPP_WARN(node->get_logger(),
                        "cartesian_stiffness > 0 with cartesian_damping_ratio = 0: undamped spring, "
                        "only joint_damping damps the cartesian axes");
        }
        if (gc_.hold_stiffness.size() > 0 || gc_.hold_breakaway_torque.size() > 0) {
            RCLCPP_WARN(node->get_logger(), "Breakaway hold parameters are ignored in impedance mode");
        }
        break;
    }
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

void CartesianImpedanceController::fetch_references()
{
    for (size_t i = 0; i < rbd_joint_names_.size(); ++i) {
        const double q   = reference_interfaces_[i * num_refs_per_joint_ + 0];
        const double dq  = reference_interfaces_[i * num_refs_per_joint_ + 1];
        const double ddq = reference_interfaces_[i * num_refs_per_joint_ + 2];
        // NaN means no valid upstream reference, hold the last valid position at rest
        if (std::isfinite(q)) data_.ref.q_ref(i) = q;
        data_.ref.dq_ref(i)  = std::isfinite(dq)  ? dq  : 0.0;
        data_.ref.ddq_ref(i) = std::isfinite(ddq) ? ddq : 0.0;
    }
}
}