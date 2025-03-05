#include <arm_control/dual_whole_body_controller.h>

#include <cmath>
#include <memory>
#include <thread>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <Eigen/Geometry>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <alpine_msgs/DualTwist.h>
#include <alpine_msgs/OdomAndJoints.h>

#include "helper_functions.h"

namespace arm_control
{

    bool DualWholeBodyController::init(hardware_interface::RobotHW *robot_hw,
                                      ros::NodeHandle &node_handle)
    {
        // initialize whole body controller variables
        tauR_wb = VectorXd::Zero(16);
        X = VectorXd::Zero(12);
        X_task = VectorXd::Zero(12);
        X_init = VectorXd::Zero(12);
        X_home = VectorXd::Zero(12);
        X_goal = VectorXd::Zero(12);
        X_taskSub = VectorXd::Zero(12);
        Q_nullSub = VectorXd::Zero(17);
        q_home_right = VectorXd::Zero(7);
        q_home_left = VectorXd::Zero(7);
        q_grasping_right = VectorXd::Zero(7);
        q_grasping_left = VectorXd::Zero(7);
        q_towel_right = VectorXd::Zero(7);
        q_handover_right = VectorXd::Zero(7);
        q_post_handover_right = VectorXd::Zero(7);
        q_get_my_hand_right = VectorXd::Zero(7);
        q_get_my_hand_left = VectorXd::Zero(7);
        Q = VectorXd::Zero(17);
        Qdot = VectorXd::Zero(17);
        Q_null = VectorXd::Zero(17);
        Q_home = VectorXd::Zero(17);
        tau_mes = VectorXd::Zero(17);
        tauR_ext = VectorXd::Zero(16);
        delta_frc = VectorXd::Zero(12);
        F_taskSub = VectorXd::Zero(12);
        F_ext = VectorXd::Zero(12);
        pose_base.setZero();
        twist_base.setZero();
        pose_base_admitance_.setZero();
        twist_base_admitance_.setZero();
        subscribe_arm_wrench_ = VectorXd::Zero(12);
        garmi_user_stopped = false;
        rightHandStatus.data = 0.0;
        leftHandStatus.data = 0.0;

        // Initialize base pose and twist vectors
        subscribe_base_pose_.resize(6);
        subscribe_base_twist_.resize(6);
        subscribe_base_pose_.setZero();
        subscribe_base_twist_.setZero();
        Jreduced = Eigen::MatrixXd::Zero(12,16);

        // initialize recorder variables
        record_time = 500.0;
        SampletimeInit = 0.1;
        NODataRec = 100;
        rec.setParams(record_time, SampletimeInit, NODataRec, "/home/msrm/Documents/whole_body_control/DATA1");

        // Load ROS parameters
        if (!node_handle.getParam("left/arm_id", arm_id_left_))
        {
            ROS_ERROR_STREAM("DualWholeBodyController: Could not read parameter left/arm_id");
            return false;
        }
        if (!node_handle.getParam("right/arm_id", arm_id_right_))
        {
            ROS_ERROR_STREAM("DualWholeBodyController: Could not read parameter right/arm_id");
            return false;
        }
        if (!node_handle.getParam("lower_joint_limits", lower_joint_limits_) || lower_joint_limits_.size() != 7)
        {
            ROS_ERROR(
                "DualWholeBodyController: Invalid or no lower_joint_limits parameters provided, "
                "aborting controller init!");
            return false;
        }
        if (!node_handle.getParam("upper_joint_limits", upper_joint_limits_) || upper_joint_limits_.size() != 7)
        {
            ROS_ERROR(
                "DualWholeBodyController: Invalid or no upper_joint_limits parameters provided, "
                "aborting controller init!");
            return false;
        }
        std::vector<std::string> joint_names_left, joint_names_right;
        if (!node_handle.getParam("left/joint_names", joint_names_left) || joint_names_left.size() != 7)
        {
            ROS_ERROR(
                "DualWholeBodyController: Invalid or no left/joint_names parameters provided, "
                "aborting controller init!");
            return false;
        }
        if (!node_handle.getParam("right/joint_names", joint_names_right) || joint_names_right.size() != 7)
        {
            ROS_ERROR(
                "DualWholeBodyController: Invalid or no right/joint_names parameters provided, "
                "aborting controller init!");
            return false;
        }
        bool success_left = init_arm(robot_hw, arm_id_left_, joint_names_left);
        bool success_right = init_arm(robot_hw, arm_id_right_, joint_names_right);

        // Dynamic reconfigure
        dynamic_reconfigure_server_ = std::make_unique<
            dynamic_reconfigure::Server<alpine_msgs::DualTaskSpaceConfig>>(
            node_handle);
        dynamic_reconfigure_server_->setCallback(
            boost::bind(&DualWholeBodyController::dynamic_reconfigure_callback, this, _1, _2));

        boost::function<void(const alpine_msgs::DualPose &)> update_target =
            [&mutex_ = mutex_,
             &arm_left = arms_.at(arm_id_left_), &arm_right = arms_.at(arm_id_right_)](
                const alpine_msgs::DualPose &goal)
        {
            mutex_.lock();
            // Left arm
            Eigen::Quaterniond last_orientation_d_target_left(arm_left.orientation_d_target_);
            poseToEigenQuaterniond(goal.left, arm_left.orientation_d_target_);
            adjustConsecutiveQuaternions(arm_left.orientation_d_target_, last_orientation_d_target_left);

            Eigen::Vector3d position_left = createEigenVector3dFromPose(goal.left);
            confineToVirtualWalls(arm_left.position_d_target_, position_left, arm_left.position_init_, arm_left.virtual_walls_);
            // Right arm
            Eigen::Quaterniond last_orientation_d_target_right(arm_right.orientation_d_target_);
            poseToEigenQuaterniond(goal.right, arm_right.orientation_d_target_);
            adjustConsecutiveQuaternions(arm_right.orientation_d_target_, last_orientation_d_target_right);

            Eigen::Vector3d position_right = createEigenVector3dFromPose(goal.right);
            confineToVirtualWalls(arm_right.position_d_target_, position_right, arm_right.position_init_, arm_right.virtual_walls_);
            mutex_.unlock();
        };
        sub_ = node_handle.subscribe<alpine_msgs::DualPose>("pose", 1, update_target);

        // Publisher for /garmi/cmd_vel
        garmiCmdVelPub_ = node_handle.advertise<geometry_msgs::Twist>("/garmi/cmd_vel", 1);

        // Publisher for garmi hands
        handRPub = node_handle.advertise<std_msgs::Float64>("/garmi/hand_control/right/command", 10);
        handLPub = node_handle.advertise<std_msgs::Float64>("/garmi/hand_control/left/command", 10);

        // Publishers for garmi task and null space states
        garmiTaskSpacePub = node_handle.advertise<alpine_msgs::DualTwist>("/garmi/wb_task_space_state", 10);
        garmiNullSpacePub = node_handle.advertise<alpine_msgs::OdomAndJoints>("/garmi/wb_null_space_state", 10);

        boost::function<void(const alpine_msgs::DualTwist&)> garmiTaskSpaceCallback =
        [this](const alpine_msgs::DualTwist& msg)
        {
            X_taskSub[0] = msg.right.linear.x;
            X_taskSub[1] = msg.right.linear.y;
            X_taskSub[2] = msg.right.linear.z;
            X_taskSub[3] = msg.right.angular.x;
            X_taskSub[4] = msg.right.angular.y;
            X_taskSub[5] = msg.right.angular.z;

            X_taskSub[6] = msg.left.linear.x;
            X_taskSub[7] = msg.left.linear.y;
            X_taskSub[8] = msg.left.linear.z;
            X_taskSub[9] = msg.left.angular.x;
            X_taskSub[10] = msg.left.angular.y;
            X_taskSub[11] = msg.left.angular.z;
        };

        boost::function<void(const alpine_msgs::OdomAndJoints&)> garmiNullSpaceCallback =
        [this](const alpine_msgs::OdomAndJoints& msg)
        {
            // Base pose (odom)
            Q_nullSub[0] = msg.odom[0]; // x
            Q_nullSub[1] = msg.odom[1]; // y
            Q_nullSub[2] = msg.odom[2]; // theta

            // Right arm joints
            for (int i = 0; i < 7; ++i) {
                Q_nullSub[3 + i] = msg.right[i];
            }

            // Left arm joints
            for (int i = 0; i < 7; ++i) {
                Q_nullSub[10 + i] = msg.left[i];
            }
            ratioStiffnessTask = msg.ContParam[0];
            ratioDampingTask = msg.ContParam[1];
            ratioStiffnessNull = msg.ContParam[2];
            ratioDampingNull = msg.ContParam[3];
        };
        
        boost::function<void(const nav_msgs::Odometry &)> baseOdomCallback =
            [this](const nav_msgs::Odometry &base_odom)
        {
            auto quaternionToEuler = [](const geometry_msgs::Quaternion& q) -> Eigen::Vector3d
            {
                Eigen::Quaterniond eigen_quaternion(q.w, q.x, q.y, q.z);
                Eigen::Vector3d euler_angles = eigen_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
                return euler_angles; // Returns a vector of Euler angles (roll, pitch, yaw)
            };
            // Assuming the first three elements of subscribe_base_pose_ are for position
            subscribe_base_pose_[0] = base_odom.pose.pose.position.x; 
            subscribe_base_pose_[1] = base_odom.pose.pose.position.y;
            subscribe_base_pose_[2] = base_odom.pose.pose.position.z;

            // Convert quaternion to Euler angles for the next three elements
            // This requires a quaternion to Euler conversion function, which you need to define or use from a library
            auto euler_angles = quaternionToEuler(base_odom.pose.pose.orientation);
            subscribe_base_pose_[3] = euler_angles[0]; // Roll
            subscribe_base_pose_[4] = euler_angles[1]; // Pitch
            subscribe_base_pose_[5] = euler_angles[2]; // Yaw

            // Assuming the first three elements of subscribe_base_twist_ are for linear velocity
            subscribe_base_twist_[0] = base_odom.twist.twist.linear.x;
            subscribe_base_twist_[1] = base_odom.twist.twist.linear.y;
            subscribe_base_twist_[2] = base_odom.twist.twist.linear.z;

            // The next three elements for angular velocity
            subscribe_base_twist_[3] = base_odom.twist.twist.angular.x;
            subscribe_base_twist_[4] = base_odom.twist.twist.angular.y;
            subscribe_base_twist_[5] = base_odom.twist.twist.angular.z;
        };

        // Callback updates external wrench on robot's right arm
        boost::function<void(const franka_msgs::FrankaState&)> righArmStatesCallback =
            [this](const franka_msgs::FrankaState &state)
        {
            // right arm
            // Define threshold values inside this function
            double threshold_force = 0.25; 
            double threshold_torque = 0.25;
            garmi_user_stopped = state.robot_mode == state.ROBOT_MODE_REFLEX ||
            state.robot_mode == state.ROBOT_MODE_USER_STOPPED;


            // Force components with threshold applied
            subscribe_arm_wrench_(0,0) = std::fabs(state.O_F_ext_hat_K[0]) < threshold_force ? 0 : state.O_F_ext_hat_K[0] - (std::signbit(state.O_F_ext_hat_K[0]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(1,0) = std::fabs(state.O_F_ext_hat_K[1]) < threshold_force ? 0 : state.O_F_ext_hat_K[1] - (std::signbit(state.O_F_ext_hat_K[1]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(2,0) = std::fabs(state.O_F_ext_hat_K[2]) < threshold_force ? 0 : state.O_F_ext_hat_K[2] - (std::signbit(state.O_F_ext_hat_K[2]) ? -threshold_force : threshold_force);
            // Torque components with threshold applied
            subscribe_arm_wrench_(3,0) = std::fabs(state.O_F_ext_hat_K[3]) < threshold_torque ? 0 : state.O_F_ext_hat_K[3] - (std::signbit(state.O_F_ext_hat_K[3]) ? -threshold_torque : threshold_torque);
            subscribe_arm_wrench_(4,0) = std::fabs(state.O_F_ext_hat_K[4]) < threshold_torque ? 0 : state.O_F_ext_hat_K[4] - (std::signbit(state.O_F_ext_hat_K[4]) ? -threshold_torque : threshold_torque);
            subscribe_arm_wrench_(5,0) = std::fabs(state.O_F_ext_hat_K[5]) < threshold_torque ? 0 : state.O_F_ext_hat_K[5] - (std::signbit(state.O_F_ext_hat_K[5]) ? -threshold_torque : threshold_torque);
        };

        // Callback updates external wrench on robot's left arm
        boost::function<void(const franka_msgs::FrankaState&)> leftArmStatesCallback =
            [this](const franka_msgs::FrankaState &state)
        {
            // left arm
            // Define threshold values inside this function
            double threshold_force = 0.25;
            double threshold_torque = 0.25;

            // Apply thresholds similarly as done for the right arm
            subscribe_arm_wrench_(0,1) = std::fabs(state.O_F_ext_hat_K[0]) < threshold_force ? 0 : state.O_F_ext_hat_K[0] - (std::signbit(state.O_F_ext_hat_K[0]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(1,1) = std::fabs(state.O_F_ext_hat_K[1]) < threshold_force ? 0 : state.O_F_ext_hat_K[1] - (std::signbit(state.O_F_ext_hat_K[1]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(2,1) = std::fabs(state.O_F_ext_hat_K[2]) < threshold_force ? 0 : state.O_F_ext_hat_K[2] - (std::signbit(state.O_F_ext_hat_K[2]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(3,1) = std::fabs(state.O_F_ext_hat_K[3]) < threshold_torque ? 0 : state.O_F_ext_hat_K[3] - (std::signbit(state.O_F_ext_hat_K[3]) ? -threshold_torque : threshold_torque);
            subscribe_arm_wrench_(4,1) = std::fabs(state.O_F_ext_hat_K[4]) < threshold_torque ? 0 : state.O_F_ext_hat_K[4] - (std::signbit(state.O_F_ext_hat_K[4]) ? -threshold_torque : threshold_torque);
            subscribe_arm_wrench_(5,1) = std::fabs(state.O_F_ext_hat_K[5]) < threshold_torque ? 0 : state.O_F_ext_hat_K[5] - (std::signbit(state.O_F_ext_hat_K[5]) ? -threshold_torque : threshold_torque);
        };

        arms_.at(arm_id_left_).arm_left = true;
        arms_.at(arm_id_right_).arm_left = false;

        // get franka arm states:
        sub_right_arm = node_handle.subscribe<franka_msgs::FrankaState>("/garmi/right_state_controller/franka_states", 1, righArmStatesCallback);
        sub_left_arm = node_handle.subscribe<franka_msgs::FrankaState>("/garmi/left_state_controller/franka_states", 1, leftArmStatesCallback);

        // subscribe to whole body controller desired values for task space and null space:
        garmiTaskSpaceSub = node_handle.subscribe<alpine_msgs::DualTwist>("garmi_task_space", 1, garmiTaskSpaceCallback);
        garmiNullSpaceSub = node_handle.subscribe<alpine_msgs::OdomAndJoints>("garmi_null_space", 1, garmiNullSpaceCallback);
        garmiBaseStateSub = node_handle.subscribe<nav_msgs::Odometry>("/garmi/base_control/mo_odom", 1, baseOdomCallback);  // get measured odometry from base state topic. 
        
        return success_left && success_right;
    }

    void
    DualWholeBodyController::starting(const ros::Time & /*time*/)
    {
        counter = 0.0;
        last_counter = 0.0;
        data_saved = false;
        initTime = -1.0;
        currentTime = -1.0;
        homing_duration = 3.0;
        ratioStiffnessTask = 1.0;
        ratioDampingTask = 1.0;
        ratioStiffnessNull = 1.0;
        ratioDampingNull = 1.0;

        // initialize exo variables
        prev_exo_time = 0.0;
        exo_time_fixed_time_out = 0;
        frequency = 1000;
        first_time = true;
        
        start_arm(arms_.at(arm_id_left_));
        start_arm(arms_.at(arm_id_right_));
        
         // set task space publisher
        task_space_state_msg.right.linear.x = 0.0;
        task_space_state_msg.right.linear.y = 0.0;
        task_space_state_msg.right.linear.z = 0.0;
        task_space_state_msg.right.angular.x = 0.0;
        task_space_state_msg.right.angular.y = 0.0;
        task_space_state_msg.right.angular.z = 0.0;
        
        task_space_state_msg.left.linear.x = 0.0;
        task_space_state_msg.left.linear.y = 0.0;
        task_space_state_msg.left.linear.z = 0.0;
        task_space_state_msg.left.angular.x = 0.0;
        task_space_state_msg.left.angular.y = 0.0;
        task_space_state_msg.left.angular.z = 0.0;
        
        null_space_state_msg.odom[0] = 0.0;
        null_space_state_msg.odom[1] = 0.0;
        null_space_state_msg.odom[2] = 0.0;

        // set null space publisher
        null_space_state_msg.right[0] = 0.0;
        null_space_state_msg.right[1] = 0.0;
        null_space_state_msg.right[2] = 0.0;
        null_space_state_msg.right[3] = 0.0;
        null_space_state_msg.right[4] = 0.0;
        null_space_state_msg.right[5] = 0.0;
        null_space_state_msg.right[6] = 0.0;

        null_space_state_msg.left[0] = 0.0;
        null_space_state_msg.left[1] = 0.0;
        null_space_state_msg.left[2] = 0.0;
        null_space_state_msg.left[3] = 0.0;
        null_space_state_msg.left[4] = 0.0;
        null_space_state_msg.left[5] = 0.0;
        null_space_state_msg.left[6] = 0.0;

        garmiTaskSpacePub.publish(task_space_state_msg);
        garmiNullSpacePub.publish(null_space_state_msg);
        
    }

    void DualWholeBodyController::stopping(const ros::Time & /*time*/)
    {

    }

    void DualWholeBodyController::publish(const ros::Time & /*time*/,
                                         const ros::Duration &period, ros::NodeHandle &node_handle)

    {
        auto &arm_left = arms_.at(arm_id_left_);
        auto &arm_right = arms_.at(arm_id_right_);
    }

    void DualWholeBodyController::update(const ros::Time &time,
                                        const ros::Duration &period)
    {
        // define arm ids
        auto &arm_right = arms_.at(arm_id_right_);
        auto &arm_left = arms_.at(arm_id_left_);

        // base variables
        pose_base << subscribe_base_pose_[0], subscribe_base_pose_[1], subscribe_base_pose_[5];
        twist_base << subscribe_base_twist_[0] , subscribe_base_twist_[1], subscribe_base_twist_[5];
        // double dt = frequency > 0 ? 1.0 / frequency : 0.001; // Ensure dt remains correct
        // pose_base(0) += (twist_base(0) * cos(pose_base(2)) - twist_base(1) * sin(pose_base(2))) * dt;
        // pose_base(1) += (twist_base(0) * sin(pose_base(2)) + twist_base(1) * cos(pose_base(2))) * dt;
        // pose_base(2) += twist_base(2) * dt;


        // joint space variables
        Q << pose_base, arm_right.q_, arm_left.q_;
        Qdot << twist_base, arm_right.dq_, arm_left.dq_;
        tau_mes << 0.0, 0.0, 0.0, arm_right.tau_garmi_, arm_left.tau_garmi_;

        // task space variables
        X.segment(0, 6) = pose_arm(Q, 7, true);
        X.segment(6, 6) = pose_arm(Q, 7, false);

        // initialize variables
        // if (currentTime < 0.0 || garmi_user_stopped) {
        if (currentTime < 0.0) {
            initTime = (ros::Time::now()).toSec();
            currentTime = 0.0;
            last_print_time = 0.0;
            last_saved_time = 0.0;
            data_saved = false;

            q_home_right << 0.7868695385414257, -1.2978995921318992, 0.13646228756819237, -1.9315505626147775, -1.79132963498441, 2.31294490814209, -0.13204289436340333;
            q_home_left << -0.6812877810294168, -1.4436166144529643, -0.09821582495414966, -1.9202956345876059, 1.6644231454928713, 2.551902171929677, 0.082411083178437;
            Q_home << pose_base, q_home_right, q_home_left;

            X_init.segment(0, 6) = pose_arm(Q, 7, true);
            X_init.segment(6, 6) = pose_arm(Q, 7, false);
            X_home.segment(0, 6) = pose_arm(Q_home, 7, true);
            X_home.segment(6, 6) = pose_arm(Q_home, 7, false);

        } else {
            currentTime = (ros::Time::now()).toSec() - initTime;
            double homingTime = 3.0;

        // Default: Position Desired Interpolation for Homing Trajectory
        if (currentTime <= homingTime) {
            double t = currentTime / homingTime; // Normalized time [0, 1]
            X_task = X_init + t * (X_home - X_init);
            Q_null = Q_home;
            delta_frc.setZero();
            F_taskSub.setZero();

        } else {
            X_task = X_taskSub + X_home;
            Q_null = Q_nullSub + Q_home;
            delta_frc << 00.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         00.0, 0.0, 0.1, 0.0, 0.0, 0.0;
            F_taskSub << 20.0, 0.0, 20.0, 0.0, 0.0, 0.0,
                         20.0, 0.0, 20.0, 0.0, 0.0, 0.0;
        }
        }

        counter += 1;
        // Start timer
        auto start = std::chrono::high_resolution_clock::now();
        // Call whole body controller
        wbControllerOutput = wholeBodyController(X_task, Q_null, F_taskSub, delta_frc, Q, Qdot,tau_mes, currentTime, ratioStiffnessTask, ratioDampingTask, ratioStiffnessNull, ratioDampingNull);            
        // Stop timer
        auto stop = std::chrono::high_resolution_clock::now();
        // Calculate duration
        duration_function = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        tauR_wb = wbControllerOutput.tauR_wb;
        cmd_vel_msg.linear.x = wbControllerOutput.v_wb(0);
        cmd_vel_msg.angular.z = wbControllerOutput.v_wb(1);
        tauR_ext = wbControllerOutput.tauR_ext;
        F_ext = wbControllerOutput.F_ext;

        // send torque desired commands for each arm
        arm_right.tau_d_ << tauR_wb.segment(2,7);
        arm_left.tau_d_ << tauR_wb.segment(9,7);

        // Publish the cmd_vel message
        garmiCmdVelPub_.publish(cmd_vel_msg);
        // handRPub.publish(rightHandStatus);
        // handLPub.publish(leftHandStatus);  

        // set task space publisher
        task_space_state_msg.right.linear.x = X[0] - X_home[0];
        task_space_state_msg.right.linear.y = X[1] - X_home[1];
        task_space_state_msg.right.linear.z = X[2] - X_home[2];
        task_space_state_msg.right.angular.x = X[3] - X_home[3];
        task_space_state_msg.right.angular.y = X[4] - X_home[4];
        task_space_state_msg.right.angular.z = X[5] - X_home[5];
        
        task_space_state_msg.left.linear.x = X[6] - X_home[6];
        task_space_state_msg.left.linear.y = X[7] - X_home[7];
        task_space_state_msg.left.linear.z = X[8] - X_home[8];
        task_space_state_msg.left.angular.x = X[9] - X_home[9];
        task_space_state_msg.left.angular.y = X[10] - X_home[10];
        task_space_state_msg.left.angular.z = X[11] - X_home[11];
        
        null_space_state_msg.odom[0] = Q[0] - Q_home[0];
        null_space_state_msg.odom[1] = Q[1] - Q_home[1];
        null_space_state_msg.odom[2] = Q[2] - Q_home[2];

        // set null space publisher
        null_space_state_msg.right[0] = Q[3] - Q_home[3];
        null_space_state_msg.right[1] = Q[4] - Q_home[4];
        null_space_state_msg.right[2] = Q[5] - Q_home[5];
        null_space_state_msg.right[3] = Q[6] - Q_home[6];
        null_space_state_msg.right[4] = Q[7] - Q_home[7];
        null_space_state_msg.right[5] = Q[8] - Q_home[8];
        null_space_state_msg.right[6] = Q[9] - Q_home[9];

        null_space_state_msg.left[0] = Q[10] - Q_home[10];
        null_space_state_msg.left[1] = Q[11] - Q_home[11];
        null_space_state_msg.left[2] = Q[12] - Q_home[12];
        null_space_state_msg.left[3] = Q[13] - Q_home[13];
        null_space_state_msg.left[4] = Q[14] - Q_home[14];
        null_space_state_msg.left[5] = Q[15] - Q_home[15];
        null_space_state_msg.left[6] = Q[16] - Q_home[16];

        garmiTaskSpacePub.publish(task_space_state_msg);
        garmiNullSpacePub.publish(null_space_state_msg);

        // Check if 1.5 seconds have passed since the last print
        // Printing the values
        if (currentTime - last_print_time >= 0.5)
        {
            frequency = (counter - last_counter)/0.5;
            std::cout << "\ntime: " << currentTime << " sec"<< std::endl;
            std::cout << "wholeBody Controller execution time: " << duration_function.count() << " microseconds." << std::endl;
            std::cout << "frequency: " << frequency << " hz"<<std::endl;
            std::cout << "Q_rel: " <<  (Q - Q_home).transpose() << std::endl;
            std::cout << "X_rel: " <<  (X - X_home).transpose() << std::endl;
            std::cout << "X_taskSub: " <<  X_taskSub.transpose() << std::endl;
            std::cout << "Q_nullSub: " <<  Q_nullSub.transpose() << std::endl;
            std::cout << "tauR_ext: " <<  tauR_ext.transpose() << std::endl;
            std::cout << "F_ext: " <<  F_ext.transpose() << std::endl;
            
            // Begin the array representation
            std::cout << "X_goal = np.array([";

            // Print right linear and angular values
            std::cout << std::fixed
                    << task_space_state_msg.right.linear.x << ", "
                    << task_space_state_msg.right.linear.y << ", "
                    << task_space_state_msg.right.linear.z << ", "
                    << task_space_state_msg.right.angular.x << ", "
                    << task_space_state_msg.right.angular.y << ", "
                    << task_space_state_msg.right.angular.z << ", ";

            // Print left linear and angular values
            std::cout << task_space_state_msg.left.linear.x << ", "
                    << task_space_state_msg.left.linear.y << ", "
                    << task_space_state_msg.left.linear.z << ", "
                    << task_space_state_msg.left.angular.x << ", "
                    << task_space_state_msg.left.angular.y << ", "
                    << task_space_state_msg.left.angular.z;
     // End the array representation
        std::cout << "])" << std::endl;
        
        // Begin the array representation
            std::cout << "Q_null = np.array([";
        // Print right linear and angular values
            std::cout << std::fixed
                    << null_space_state_msg.odom[0] << ", "
                    << null_space_state_msg.odom[1] << ", "
                    << null_space_state_msg.odom[2] << ", "

                    << null_space_state_msg.right[0] << ", "
                    << null_space_state_msg.right[1] << ", "
                    << null_space_state_msg.right[2] << ", "
                    << null_space_state_msg.right[3] << ", "
                    << null_space_state_msg.right[4] << ", "
                    << null_space_state_msg.right[5] << ", "
                    << null_space_state_msg.right[6] << ", "

                    << null_space_state_msg.left[0] << ", "
                    << null_space_state_msg.left[1] << ", "
                    << null_space_state_msg.left[2] << ", "
                    << null_space_state_msg.left[3] << ", "
                    << null_space_state_msg.left[4] << ", "
                    << null_space_state_msg.left[5] << ", "
                    << null_space_state_msg.left[6];
        // End the array representation
        std::cout << "])" << std::endl;

            // Update the last print time
            last_print_time = currentTime;
            last_counter = counter;
        }

       if (currentTime < record_time && currentTime - last_saved_time > 0.1)
        {
            VectorXd X_rel(12);
            // Recording data with correct sequence numbers in comments
            rec.addToRec(currentTime);                              // 1
            rec.addToRec(Q);                                        // 2-18
            X_rel << X - X_home;
            rec.addToRec(X_rel);                                    // 19-30
            
            rec.addToRec(X_taskSub);                                    // 31-42
            
            rec.addToRec(F_ext);                                    // 43-54

            rec.addToRec(tauR_ext);                                 // 55-70

            double time_wb = duration_function.count();
            rec.addToRec(time_wb);                        // 71

            rec.next();
            last_saved_time = currentTime;
        }
        else if (currentTime >= record_time && !data_saved)
        {
            rec.saveData();
            data_saved = true;
        }

        // Spin ROS callbacks
        ros::spinOnce();

        step(arms_.at(arm_id_left_), period);
        step(arms_.at(arm_id_right_), period);
        update_parameters(arms_.at(arm_id_left_));
        update_parameters(arms_.at(arm_id_right_));
    }

    void DualWholeBodyController::dynamic_reconfigure_callback(
        alpine_msgs::DualTaskSpaceConfig &config,
        uint32_t /*level*/)
    {
        auto &arm_left = arms_.at(arm_id_left_);
        auto &arm_right = arms_.at(arm_id_right_);
        mutex_.lock();
        // Left arm
        arm_left.cartesian_stiffness_target_.setIdentity();
        arm_left.cartesian_stiffness_target_.diagonal() << config.translational_stiffness_x_left,
            config.translational_stiffness_y_left,
            config.translational_stiffness_z_left,
            config.rotational_stiffness_x_left,
            config.rotational_stiffness_y_left,
            config.rotational_stiffness_z_left;
        // Damping ratio = 1
        arm_left.cartesian_damping_target_.setIdentity();
        arm_left.cartesian_damping_target_.diagonal() << 2.0 * sqrt(config.translational_stiffness_x_left),
            2.0 * sqrt(config.translational_stiffness_y_left),
            2.0 * sqrt(config.translational_stiffness_z_left),
            2.0 * sqrt(config.rotational_stiffness_x_left),
            2.0 * sqrt(config.rotational_stiffness_y_left),
            2.0 * sqrt(config.rotational_stiffness_z_left);
        arm_left.nullspace_stiffness_target_ = config.nullspace_stiffness_left;
        arm_left.filter_params_ = config.filter_params;
        arm_left.virtual_walls_ << config.virtual_walls_1, config.virtual_walls_2,
            config.virtual_walls_3, config.virtual_walls_4,
            config.virtual_walls_5, config.virtual_walls_6;
        // Right arm
        arm_right.cartesian_stiffness_target_.setIdentity();
        arm_right.cartesian_stiffness_target_.diagonal() << config.translational_stiffness_x_right,
            config.translational_stiffness_y_right,
            config.translational_stiffness_z_right,
            config.rotational_stiffness_x_right,
            config.rotational_stiffness_y_right,
            config.rotational_stiffness_z_right;
        // Damping ratio = 1
        arm_right.cartesian_damping_target_.setIdentity();
        arm_right.cartesian_damping_target_.diagonal() << 2.0 * sqrt(config.translational_stiffness_x_right),
            2.0 * sqrt(config.translational_stiffness_y_right),
            2.0 * sqrt(config.translational_stiffness_z_right),
            2.0 * sqrt(config.rotational_stiffness_x_right),
            2.0 * sqrt(config.rotational_stiffness_y_right),
            2.0 * sqrt(config.rotational_stiffness_z_right);
        arm_right.nullspace_stiffness_target_ = config.nullspace_stiffness_right;
        arm_right.filter_params_ = config.filter_params;
        arm_right.virtual_walls_ << config.virtual_walls_1, config.virtual_walls_2,
            config.virtual_walls_3, config.virtual_walls_4,
            config.virtual_walls_5, config.virtual_walls_6;
        mutex_.unlock();
    }

} // namespace arm_control

PLUGINLIB_EXPORT_CLASS(arm_control::DualWholeBodyController,
                       controller_interface::ControllerBase)
