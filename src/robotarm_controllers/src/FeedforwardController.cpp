#include "robotarm_controllers/FeedforwardController.hpp"

#include <algorithm>
#include <limits>

PLUGINLIB_EXPORT_CLASS(
  feedforward_controller::FeedforwardController, 
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
        // no time passed (in the sim, sim time can stand still for a cycle while the controller manager runs on):
        // the torque must not change, but that is not a rate violation
        if (dt <= 0) {
            t_new = t;
            return true;
        }
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
}

namespace feedforward_controller
{

controller_interface::InterfaceConfiguration FeedforwardController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : rbd_joint_names_)
    {
        command_interfaces_config.names.push_back(joint + "/position");
        command_interfaces_config.names.push_back(joint + "/velocity");
        command_interfaces_config.names.push_back(joint + "/torque_ff");
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration FeedforwardController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    for (const auto & joint : rbd_joint_names_) {
        state_interfaces_config.names.push_back(joint + "/position");
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn FeedforwardController::on_init()
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

controller_interface::CallbackReturn FeedforwardController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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

    data_.resize(rbd_.get_dof());

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedforwardController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    // make sure hw is in defined state when claiming interfaces
    if (!fetch_robotarm_state_strict()) return controller_interface::CallbackReturn::ERROR;

    // set ref to hold, fetch references only overwrittes them with valid data
    data_.ref.q_ref = data_.state.q;
    data_.ref.dq_ref.setZero();
    data_.ref.ddq_ref.setZero();
    // start at torque zero because moteus pid is still active -> smooth blend in so i can unwind 
    data_.ref.tau_start.setZero();
    data_.ref.tau_cmd.setZero();
    // drop references of a previous activation, NaN holds q_ref until an upstream controller writes
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(),
              std::numeric_limits<double>::quiet_NaN());
    ramp_elapsed_ = 0;

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedforwardController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> FeedforwardController::on_export_reference_interfaces()
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

controller_interface::return_type FeedforwardController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // no standalone input, only as downstream controller
    return controller_interface::return_type::OK;
}

controller_interface::return_type FeedforwardController::update_and_write_commands(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    fetch_references();
    // compute ff
    if (!rbd_.recursive_newton_euler(
            data_.ref.q_ref,
            data_.ref.dq_ref,
            data_.ref.ddq_ref,
            data_.tau_new,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero())) {
        RCLCPP_ERROR(get_node()->get_logger(), "rbd runtime error: %s \n", rbd_.last_error());
        return controller_interface::return_type::ERROR;
    }

    // smooth ramp in on startup
    if (ramp_elapsed_ < ramp_time_) {
        data_.tau_new = (data_.tau_new - data_.ref.tau_start)/ramp_time_*ramp_elapsed_ + data_.ref.tau_start;
        ramp_elapsed_ += period.seconds();
    }

    bool rate_limited = false;
    if (!clamp_torque_rate(data_.tau_new,
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
    if (!clamp_torque(data_.tau_new, joint_limits_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error clamping torque: nan or invalid limits");
        return controller_interface::return_type::ERROR;
    }
    data_.ref.tau_cmd = data_.tau_new;

    for (size_t i = 0; i < rbd_.get_dof(); ++i)
    {
        size_t pos_idx = i * num_cmd_interfaces_per_joint_ + 0;
        size_t vel_idx = i * num_cmd_interfaces_per_joint_ + 1;
        size_t tff_idx = i * num_cmd_interfaces_per_joint_ + 2;

        (void)command_interfaces_[pos_idx].set_value(data_.ref.q_ref[i]);
        (void)command_interfaces_[vel_idx].set_value(data_.ref.dq_ref[i]);
        (void)command_interfaces_[tff_idx].set_value(data_.ref.tau_cmd[i]);
    }

    return controller_interface::return_type::OK;
}

void FeedforwardController::declare_parameters()
{
    auto node = get_node();

    node->declare_parameter<double>("torque_scale", 1.0);
    node->declare_parameter<double>("torque_rise_time", 0.0);
}

controller_interface::CallbackReturn FeedforwardController::read_parameters()
{
    auto node = get_node();

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

    return controller_interface::CallbackReturn::SUCCESS;
}

bool FeedforwardController::fetch_robotarm_state_strict()
{
    for (size_t i = 0; i < rbd_.get_dof(); ++i)
    {
        auto pos_opt = state_interfaces_[i].get_optional();

        if (!pos_opt.has_value())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Hardware-State for joint %s could not be read",
                         rbd_joint_names_[i].c_str());
            return false;
        }
        data_.state.q[i] = pos_opt.value();
    }
    return true;
}

void FeedforwardController::fetch_references()
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