#include <arm_control/whole_body_controller.h>

#include <cmath>
#include <memory>
#include <thread>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "helper_functions.h"

namespace arm_control
{

    bool WholeBodyController::init(hardware_interface::RobotHW *robot_hw,
                                  ros::NodeHandle &node_handle)
    {
        // Load ROS parameters
        int ahjsdfhkjsdfh = 0;
        if (!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR_STREAM("WholeBodyController: Could not read parameter arm_id");
            return false;
        }
        if (!node_handle.getParam("lower_joint_limits", lower_joint_limits_) || lower_joint_limits_.size() != 7)
        {
            ROS_ERROR(
                "WholeBodyController: Invalid or no lower_joint_limits parameters provided, "
                "aborting controller init!");
            return false;
        }
        if (!node_handle.getParam("upper_joint_limits", upper_joint_limits_) || upper_joint_limits_.size() != 7)
        {
            ROS_ERROR(
                "WholeBodyController: Invalid or no upper_joint_limits parameters provided, "
                "aborting controller init!");
            return false;
        }
        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
        {
            ROS_ERROR(
                "WholeBodyController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
            return false;
        }
        bool success = init_arm(robot_hw, arm_id, joint_names);

        // Dynamic reconfigure
        dynamic_reconfigure_server_ = std::make_unique<
            dynamic_reconfigure::Server<alpine_msgs::TaskSpaceConfig>>(
            node_handle);
        dynamic_reconfigure_server_->setCallback(
            boost::bind(&WholeBodyController::dynamic_reconfigure_callback, this, _1, _2));

        boost::function<void(const geometry_msgs::PoseStamped &)> update_target =
            [&mutex_ = mutex_, &arm = arms_.at(arm_id)](const geometry_msgs::PoseStamped &goal)
        {
            mutex_.lock();
            Eigen::Quaterniond last_orientation_d_target(arm.orientation_d_target_);
            poseToEigenQuaterniond(goal.pose, arm.orientation_d_target_);
            adjustConsecutiveQuaternions(arm.orientation_d_target_, last_orientation_d_target);

            Eigen::Vector3d position = createEigenVector3dFromPose(goal.pose);
            confineToVirtualWalls(arm.position_d_target_, position, arm.position_init_, arm.virtual_walls_);
            mutex_.unlock();
        };

        boost::function<void(const geometry_msgs::PoseStamped &)> update_relative_target =
            [&mutex_ = mutex_, &arm = arms_.at(arm_id)](const geometry_msgs::PoseStamped &relative_goal)
        {
            mutex_.lock();
            Eigen::Quaterniond relative_goal_orientation = createEigenQuaterniondFromPose(relative_goal.pose);
            Eigen::Vector3d relative_goal_position = createEigenVector3dFromPose(relative_goal.pose);
            Eigen::Quaterniond goal_orientation;
            if (relative_goal.header.frame_id == "flange")
            {
                relative_goal_position = arm.orientation_init_ * relative_goal_position;
                goal_orientation = arm.orientation_init_ * relative_goal_orientation;
            }
            else
            {
                goal_orientation = relative_goal_orientation * arm.orientation_init_;
            }
            Eigen::Vector3d goal_position = arm.position_init_ + relative_goal_position;

            Eigen::Quaterniond last_orientation_d_target(arm.orientation_d_target_);
            arm.orientation_d_target_ = goal_orientation;
            adjustConsecutiveQuaternions(arm.orientation_d_target_, last_orientation_d_target);
            confineToVirtualWalls(arm.position_d_target_, goal_position, arm.position_init_, arm.virtual_walls_);
            mutex_.unlock();
        };

        // sub_ = node_handle.subscribe<geometry_msgs::PoseStamped>("pose", 1, update_target);
        sub_relative_ = node_handle.subscribe<geometry_msgs::PoseStamped>("displacement", 1, update_relative_target);

        // ROS subscriber for garmi_joint_postion and garmi_joint_torque
        // exoLeftJointStateSub = node_handle.subscribe<sensor_msgs::JointState>("exo_joint_states", 10, update_target);

        // ROS publisher for exo_joint_position

        return success;
    }

    bool WholeBodyController::init_arm(hardware_interface::RobotHW *robot_hw, const std::string arm_id,
                                      std::vector<std::string> joint_names)
    {
        // Arm container
        ArmContainer arm;
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "WholeBodyController: Error getting model interface from hardware");
            return false;
        }
        try
        {
            arm.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM(
                "WholeBodyController: Exception getting model handle from interface: "
                << ex.what());
            return false;
        }

        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "WholeBodyController: Error getting state interface from hardware");
            return false;
        }
        try
        {
            arm.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle(arm_id + "_robot"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM(
                "WholeBodyController: Exception getting state handle from interface: "
                << ex.what());
            return false;
        }

        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "WholeBodyController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i)
        {
            try
            {
                arm.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &ex)
            {
                ROS_ERROR_STREAM(
                    "WholeBodyController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }
        arm.q_d_nullspace_.setZero();
        arm.nullspace_stiffness_ = 0.0;
        arm.nullspace_stiffness_target_ = 0.0;
        arm.position_d_.setZero();
        arm.orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        arm.orientation_init_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        arm.position_d_target_.setZero();
        arm.orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        arm.cartesian_stiffness_.setZero();
        arm.cartesian_damping_.setZero();
        arm.filter_params_ = 0.0;

        arm.cartesian_P_.setZero();
        arm.cartesian_K_.setZero();

        arms_.emplace(std::make_pair(arm_id, std::move(arm)));
        return true;
    }

    void WholeBodyController::starting(const ros::Time & /*time*/)
    {
        start_arm(arms_.at(arm_id));
    }

    void WholeBodyController::start_arm(ArmContainer &arm)
    {
        franka::RobotState initial_state = arm.state_handle_->getRobotState();
        std::array<double, 42> jacobian_array = arm.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        arm.transform_ = initial_transform;
        arm.position_init_ = initial_transform.translation();
        arm.position_ee_ = initial_transform.translation();
        arm.position_ = initial_transform.translation();
        //arm.position_d_ = initial_transform.translation();
        arm.position_d_temp_ = arm.position_d_;
        arm.orientation_init_ = Eigen::Quaterniond(initial_transform.linear());
        arm.orientation_ = Eigen::Quaterniond(initial_transform.linear());
        //arm.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
        arm.position_d_target_ = initial_transform.translation();
        arm.orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
        arm.jacobian_ = jacobian;
        arm.q_d_nullspace_ = q_initial;
        arm.q_init_ = q_initial;
        arm.q_ = q_initial;
        arm.dq_ = dq_initial;
        arm.homing_duration_ = 1.5;
        arm.time_ = 0.0;
        arm.exo_time_ = 0.0;
        arm.tau_d_.setZero();
        arm.spatialOffset_ << 0.0,0.4,0.0;

        // passivity based controller
        const double translational_P{7.0}; // 10
        const double rotational_P{5.0};    // 5
        arm.cartesian_P_.setZero();
        arm.cartesian_P_.topLeftCorner(3, 3) << translational_P * Eigen::MatrixXd::Identity(3, 3);
        arm.cartesian_P_.bottomRightCorner(3, 3) << rotational_P * Eigen::MatrixXd::Identity(3, 3);

        const double translational_K{50.0}; // max 150
        const double rotational_K{10.0};    // max 20
        arm.cartesian_K_.setZero();
        arm.cartesian_K_.topLeftCorner(3, 3) << translational_K * Eigen::MatrixXd::Identity(3, 3);
        arm.cartesian_K_.bottomRightCorner(3, 3) << rotational_K * Eigen::MatrixXd::Identity(3, 3);
    }

    void WholeBodyController::stopping(const ros::Time & /*time*/)
    {
    }

    void WholeBodyController::step(ArmContainer &arm, const ros::Duration &period)
    {
        mutex_.lock();
        franka::RobotState robot_state = arm.state_handle_->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_joint_limits(7), tau_d(7);

        tau_d << arm.tau_d_;

        tau_d << saturateTorqueRate(tau_d, tau_J_d);

        for (size_t i = 0; i < 7; ++i)
        {
            arm.joint_handles_[i].setCommand(tau_d(i));
        }

        mutex_.unlock();
    }

    void WholeBodyController::update_parameters(ArmContainer &arm)
    {
        mutex_.lock();
        franka::RobotState robot_state = arm.state_handle_->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        std::array<double, 42> jacobian_array = arm.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        arm.q_ = q;
        arm.dq_ = dq;
        arm.tau_garmi_ = tau_J;
        arm.time_ += 0.001;
        arm.position_ = transform.translation();
        arm.orientation_ = Eigen::Quaterniond(transform.linear());
        arm.jacobian_ = jacobian;
        // Linear interpolation to update targeted stiffness and position
        arm.cartesian_stiffness_ =
            arm.filter_params_ * arm.cartesian_stiffness_target_ + (1.0 - arm.filter_params_) * arm.cartesian_stiffness_;
        arm.cartesian_damping_ =
            arm.filter_params_ * arm.cartesian_damping_target_ + (1.0 - arm.filter_params_) * arm.cartesian_damping_;
        arm.nullspace_stiffness_ =
            arm.filter_params_ * arm.nullspace_stiffness_target_ + (1.0 - arm.filter_params_) * arm.nullspace_stiffness_;
        //arm.position_d_ = arm.filter_params_ * arm.position_d_target_ + (1.0 - arm.filter_params_) * arm.position_d_;

        //arm.orientation_d_ = arm.orientation_d_.slerp(arm.filter_params_, arm.orientation_d_target_); // franka style
        mutex_.unlock();
    }

    void WholeBodyController::update(const ros::Time & /*time*/,
                                    const ros::Duration &period)
    {
        step(arms_.at(arm_id), period);
        update_parameters(arms_.at(arm_id));
    }

    Eigen::Matrix<double, 7, 1> WholeBodyController::saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d)
    {
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++)
        {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }

    void WholeBodyController::dynamic_reconfigure_callback(
        alpine_msgs::TaskSpaceConfig &config,
        uint32_t /*level*/)
    {
        auto &arm = arms_.at(arm_id);
        mutex_.lock();
        arm.cartesian_stiffness_target_.setIdentity();
        arm.cartesian_stiffness_target_.topLeftCorner(3, 3)
            << config.translational_stiffness * Eigen::Matrix3d::Identity();
        arm.cartesian_stiffness_target_.bottomRightCorner(3, 3)
            << config.rotational_stiffness * Eigen::Matrix3d::Identity();
        arm.cartesian_damping_target_.setIdentity();
        // Damping ratio = 1
        arm.cartesian_damping_target_.topLeftCorner(3, 3)
            << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
        arm.cartesian_damping_target_.bottomRightCorner(3, 3)
            << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
        arm.nullspace_stiffness_target_ = config.nullspace_stiffness;
        arm.filter_params_ = config.filter_params;
        arm.virtual_walls_ << config.virtual_walls_1, config.virtual_walls_2,
            config.virtual_walls_3, config.virtual_walls_4,
            config.virtual_walls_5, config.virtual_walls_6;
        mutex_.unlock();
    }

} // namespace arm_control

PLUGINLIB_EXPORT_CLASS(arm_control::WholeBodyController,
                       controller_interface::ControllerBase)
