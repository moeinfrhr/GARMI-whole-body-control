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

#include "helper_functions.h"

namespace arm_control
{

    bool DualWholeBodyController::init(hardware_interface::RobotHW *robot_hw,
                                      ros::NodeHandle &node_handle)
    {
        // initialize control/trajectory variables
        tau_wb = VectorXd::Zero(17);
        X = VectorXd::Zero(12);
        X_task = VectorXd::Zero(12);
        X_init = VectorXd::Zero(12);
        X_home = VectorXd::Zero(12);
        q_home_right = VectorXd::Zero(7);
        q_home_left = VectorXd::Zero(7);
        Q = VectorXd::Zero(17);
        Qdot = VectorXd::Zero(17);
        Q_null = VectorXd::Zero(17);
        pose_base.setZero();
        twist_base.setZero();
        pose_base_admitance_.setZero();
        twist_base_admitance_.setZero();
        subscribe_arm_wrench_ = VectorXd::Zero(12);

        // Initialize base pose and twist vectors
        subscribe_base_pose_.resize(6);
        subscribe_base_twist_.resize(6);
        subscribe_base_pose_.setZero();
        subscribe_base_twist_.setZero();
        Jreduced = Eigen::MatrixXd::Zero(12,16);

        counter = 0.0;
        last_counter = 0.0;
        prev_exo_time = 0.0;
        exo_time_fixed_time_out = 0;
        frequency = 1000;
        first_time = true;
        data_saved = false;
        initTime = -1;
        currentTime = -1;
        homing_duration = 2.0;
        record_time = 45.0;
        SampletimeInit = 0.1;
        NODataRec = 100;
        rec.setParams(record_time, SampletimeInit, NODataRec, "/home/msrm/Documents/whole_body_control/DATA8");

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

        boost::function<void(const alpine_msgs::DualPose &)> update_relative_target =
            [&mutex_ = mutex_,
             &arm_left = arms_.at(arm_id_left_), &arm_right = arms_.at(arm_id_right_)](
                const alpine_msgs::DualPose &relative_goal)
        {
            mutex_.lock();
            // Left arm
            Eigen::Quaterniond relative_goal_orientation_left = createEigenQuaterniondFromPose(relative_goal.left);
            Eigen::Vector3d relative_goal_position_left = createEigenVector3dFromPose(relative_goal.left);
            Eigen::Quaterniond goal_orientation_left;
            if (relative_goal.header.frame_id == "flange")
            {
                relative_goal_position_left = arm_left.orientation_init_ * relative_goal_position_left;
                goal_orientation_left = arm_left.orientation_init_ * relative_goal_orientation_left;
            }
            else
            {
                goal_orientation_left = relative_goal_orientation_left * arm_left.orientation_init_;
            }
            Eigen::Vector3d goal_position_left = arm_left.position_init_ + relative_goal_position_left;

            Eigen::Quaterniond last_orientation_d_target_left(arm_left.orientation_d_target_);
            arm_left.orientation_d_target_ = goal_orientation_left;
            adjustConsecutiveQuaternions(arm_left.orientation_d_target_, last_orientation_d_target_left);
            confineToVirtualWalls(arm_left.position_d_target_, goal_position_left, arm_left.position_init_, arm_left.virtual_walls_);
            // Right arm
            Eigen::Quaterniond relative_goal_orientation_right = createEigenQuaterniondFromPose(relative_goal.right);
            Eigen::Vector3d relative_goal_position_right = createEigenVector3dFromPose(relative_goal.right);
            Eigen::Quaterniond goal_orientation_right;
            if (relative_goal.header.frame_id == "flange")
            {
                relative_goal_position_right = arm_right.orientation_init_ * relative_goal_position_right;
                goal_orientation_right = arm_right.orientation_init_ * relative_goal_orientation_right;
            }
            else
            {
                goal_orientation_right = relative_goal_orientation_right * arm_right.orientation_init_;
            }
            Eigen::Vector3d goal_position_right = arm_right.position_init_ + relative_goal_position_right;

            Eigen::Quaterniond last_orientation_d_target_right(arm_right.orientation_d_target_);
            arm_right.orientation_d_target_ = goal_orientation_right;
            adjustConsecutiveQuaternions(arm_right.orientation_d_target_, last_orientation_d_target_right);
            confineToVirtualWalls(arm_right.position_d_target_, goal_position_right, arm_right.position_init_, arm_right.virtual_walls_);
            mutex_.unlock();
        };
        // sub_relative_ = node_handle.subscribe<alpine_msgs::DualPose>("displacement", 1, update_relative_target);

        // Definition of a boost::function named update_exo_joint_state that takes a const reference to sensor_msgs::JointState as a parameter
        boost::function<void(const nav_msgs::Odometry &)> update_base_odom =
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
        boost::function<void(const franka_msgs::FrankaState&)> update_right_arm_wrench =
            [this](const franka_msgs::FrankaState &state)
        {
            // right arm. 
            /*subscribe_arm_wrench_(0,0) = state.O_F_ext_hat_K[0]; // Force x
            subscribe_arm_wrench_(1,0) = state.O_F_ext_hat_K[1]; // Force y
            subscribe_arm_wrench_(2,0) = state.O_F_ext_hat_K[2]; // Force z
            subscribe_arm_wrench_(3,0) = state.O_F_ext_hat_K[3]; // Torque x
            subscribe_arm_wrench_(4,0) = state.O_F_ext_hat_K[4]; // Torque y
            subscribe_arm_wrench_(5,0) = state.O_F_ext_hat_K[5]; // Torque z*/
            
            // Define threshold values inside this function
            double threshold_force = 1.5; // Example threshold for force
            double threshold_torque = 1.0; // Example threshold for torque

            // Force components with threshold applied
            subscribe_arm_wrench_(0,0) = 0*std::fabs(state.O_F_ext_hat_K[0]) < threshold_force ? 0 : state.O_F_ext_hat_K[0] - (std::signbit(state.O_F_ext_hat_K[0]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(1,0) = 0*std::fabs(state.O_F_ext_hat_K[1]) < threshold_force ? 0 : state.O_F_ext_hat_K[1] - (std::signbit(state.O_F_ext_hat_K[1]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(2,0) = 0*std::fabs(state.O_F_ext_hat_K[2]) < threshold_force ? 0 : state.O_F_ext_hat_K[2] - (std::signbit(state.O_F_ext_hat_K[2]) ? -threshold_force : threshold_force);
            // Torque components with threshold applied
            subscribe_arm_wrench_(3,0) = 0*std::fabs(state.O_F_ext_hat_K[3]) < threshold_torque ? 0 : state.O_F_ext_hat_K[3] - (std::signbit(state.O_F_ext_hat_K[3]) ? -threshold_torque : threshold_torque);
            subscribe_arm_wrench_(4,0) = 0*std::fabs(state.O_F_ext_hat_K[4]) < threshold_torque ? 0 : state.O_F_ext_hat_K[4] - (std::signbit(state.O_F_ext_hat_K[4]) ? -threshold_torque : threshold_torque);
            subscribe_arm_wrench_(5,0) = 0*std::fabs(state.O_F_ext_hat_K[5]) < threshold_torque ? 0 : state.O_F_ext_hat_K[5] - (std::signbit(state.O_F_ext_hat_K[5]) ? -threshold_torque : threshold_torque);
        };

        // Callback updates external wrench on robot's left arm
        boost::function<void(const franka_msgs::FrankaState&)> update_left_arm_wrench =
            [this](const franka_msgs::FrankaState &state)
        {
            /*// left arm. 
            subscribe_arm_wrench_(0,1) = state.O_F_ext_hat_K[0]; // Force x
            subscribe_arm_wrench_(1,1) = state.O_F_ext_hat_K[1]; // Force y
            subscribe_arm_wrench_(2,1) = state.O_F_ext_hat_K[2]; // Force z
            subscribe_arm_wrench_(3,1) = state.O_F_ext_hat_K[3]; // Torque x
            subscribe_arm_wrench_(4,1) = state.O_F_ext_hat_K[4]; // Torque y
            subscribe_arm_wrench_(5,1) = state.O_F_ext_hat_K[5]; // Torque z*/
            double threshold_force = 1.5; // Example threshold for force
            double threshold_torque = 1.0; // Example threshold for torque

            // Apply thresholds similarly as done for the right arm
            subscribe_arm_wrench_(0,1) = std::fabs(state.O_F_ext_hat_K[0]) < threshold_force ? 0 : state.O_F_ext_hat_K[0] - (std::signbit(state.O_F_ext_hat_K[0]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(1,1) = 0*std::fabs(state.O_F_ext_hat_K[1]) < threshold_force ? 0 : state.O_F_ext_hat_K[1] - (std::signbit(state.O_F_ext_hat_K[1]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(2,1) = 0*std::fabs(state.O_F_ext_hat_K[2]) < threshold_force ? 0 : state.O_F_ext_hat_K[2] - (std::signbit(state.O_F_ext_hat_K[2]) ? -threshold_force : threshold_force);
            subscribe_arm_wrench_(3,1) = 0*std::fabs(state.O_F_ext_hat_K[3]) < threshold_torque ? 0 : state.O_F_ext_hat_K[3] - (std::signbit(state.O_F_ext_hat_K[3]) ? -threshold_torque : threshold_torque);
            subscribe_arm_wrench_(4,1) = 0*std::fabs(state.O_F_ext_hat_K[4]) < threshold_torque ? 0 : state.O_F_ext_hat_K[4] - (std::signbit(state.O_F_ext_hat_K[4]) ? -threshold_torque : threshold_torque);
            subscribe_arm_wrench_(5,1) = 0*std::fabs(state.O_F_ext_hat_K[5]) < threshold_torque ? 0 : state.O_F_ext_hat_K[5] - (std::signbit(state.O_F_ext_hat_K[5]) ? -threshold_torque : threshold_torque);
        };

        // Definition of a boost::function named update_exo_joint_state that takes a const reference to sensor_msgs::JointState as a parameter
        boost::function<void(const sensor_msgs::JointState &)> update_exo_joint_state =
            [this](const sensor_msgs::JointState &exo_joint_states)
        {
            // Get references to the left and right arms from the arms_ container
            auto &arm_left = arms_.at(arm_id_left_);
            auto &arm_right = arms_.at(arm_id_right_);

            // Retrieve the timestamp from the received exo_joint_states message
            double current_exo_time = exo_joint_states.header.stamp.toSec();

            // Check if the current_exo_time has changed from the previous time
            if (current_exo_time != prev_exo_time)
            {
                // Time has changed, update the values for the left arm
                //arm_left.exo_time_ = current_exo_time;
                arm_left.exo_time_ += 0.001;
                arm_right.exo_time_ = 0.0;
                exo_time_fixed_time_out = 0.0;

                // Assign specific positions, velocities, and efforts from exo_joint_states to arm_left
                arm_left.q_home_ << -1.2952, -1.3288, -0.1990, -2.0017, -0.0461, 2.5764, -0.5245;

                {
                    for (size_t i = 0; i < 4; ++i)
                    {
                        arm_left.relative_joint_position_d_target_[i] = exo_joint_states.position[i];
                        arm_left.joint_velocity_d_target_[i] = exo_joint_states.velocity[i];
                        arm_left.tau_exo_[i] = exo_joint_states.effort[i];
                    }
                }

                // Initialize the remaining elements to zero for the left arm
                for (size_t i = 4; i < arm_left.relative_joint_position_d_target_.size(); ++i)
                {
                    arm_left.relative_joint_position_d_target_[i] = 0.0;
                    arm_left.joint_velocity_d_target_[i] = 0.0;
                    arm_left.tau_exo_[i] = 0.0;
                }
            }
            else if (exo_time_fixed_time_out < 0.2)
            {
                // Wait for resetting the values. Increment the exo_time_fixed_time_out by 0.001.
                exo_time_fixed_time_out += 0.001;
            }
            else
            {
                // If the exo_time is fixed for a certain period, reset the values for the left arm
                arm_left.exo_time_ = 0.0;
                arm_right.exo_time_ = 0.0;
                arm_left.relative_joint_position_d_target_.setZero(exo_joint_states.position.size());
                arm_left.joint_velocity_d_target_.setZero(exo_joint_states.velocity.size());
                arm_left.tau_exo_.setZero(exo_joint_states.effort.size());
            }

            // Reset the values for the right arm
            if(first_time){
                arm_right.q_home_ << arm_right.q_;
                first_time = false;
            }
            arm_right.relative_joint_position_d_target_.setZero(exo_joint_states.position.size());
            arm_right.joint_velocity_d_target_.setZero(exo_joint_states.velocity.size());
            arm_right.tau_exo_.setZero(exo_joint_states.effort.size());

            // Update the previous exo_time with the current_exo_time for future comparisons
            prev_exo_time = current_exo_time;

            // Print the values of arm_left for debugging purposes
            // std::cout << "Exotime = " << std::setw(6) << std::fixed << std::setprecision(4) << arm_left.exo_time_ << " | "
            //           << "q_des: " << arm_left.relative_joint_position_d_target_.transpose() << " | "
            //           << "qd_des: " << arm_left.joint_velocity_d_target_.transpose() << " | "
            //           << "tau_exo: " << arm_left.tau_exo_.transpose() << std::endl;
        };

        arms_.at(arm_id_left_).arm_left = true;
        arms_.at(arm_id_right_).arm_left = false;
        zero_q_.setZero();
        exoJointStateSub = node_handle.subscribe<sensor_msgs::JointState>("exo_joint_states", 1, update_exo_joint_state);
        garmiBaseStateSub = node_handle.subscribe<nav_msgs::Odometry>("/garmi/odom", 1, update_base_odom);
        sub_right_arm = node_handle.subscribe<franka_msgs::FrankaState>("/garmi/right_state_controller/franka_states", 1, update_right_arm_wrench);
        sub_left_arm = node_handle.subscribe<franka_msgs::FrankaState>("/garmi/left_state_controller/franka_states", 1, update_left_arm_wrench);

        return success_left && success_right;
    }

    void
    DualWholeBodyController::starting(const ros::Time & /*time*/)
    {
        start_arm(arms_.at(arm_id_left_));
        start_arm(arms_.at(arm_id_right_));
        
    }

    void DualWholeBodyController::stopping(const ros::Time & /*time*/)
    {

    }

    void DualWholeBodyController::publish(const ros::Time & /*time*/,
                                         const ros::Duration &period, ros::NodeHandle &node_handle)

    {
        auto &arm_left = arms_.at(arm_id_left_);
        auto &arm_right = arms_.at(arm_id_right_);

        ros::Publisher garmiLeftJointStatesPub = node_handle.advertise<sensor_msgs::JointState>("garmi_left_joint_states", 1000);
        sensor_msgs::JointState garmi_left_joint_states;

        garmi_left_joint_states.name[0] = "Joint 1";
        garmi_left_joint_states.name[1] = "Joint 2";
        garmi_left_joint_states.name[2] = "Joint 3";
        garmi_left_joint_states.name[3] = "Joint 4";
        garmi_left_joint_states.name[4] = "Joint 5";
        garmi_left_joint_states.name[5] = "Joint 6";
        garmi_left_joint_states.name[6] = "Joint 7";

        garmi_left_joint_states.header.stamp = ros::Time().fromSec(arm_right.time_);

        for (int i = 0; i < 7; ++i)
        {
            garmi_left_joint_states.position[i] = arm_left.q_[i];
            garmi_left_joint_states.velocity[i] = arm_left.dq_[i];
            garmi_left_joint_states.effort[i] = arm_left.tau_garmi_[i];
        }

        // Publish the JointState message
        garmiLeftJointStatesPub.publish(garmi_left_joint_states);       

    }

    void DualWholeBodyController::update(const ros::Time &time,
                                        const ros::Duration &period)
    {
        // define arm ids
        auto &arm_right = arms_.at(arm_id_right_);
        auto &arm_left = arms_.at(arm_id_left_);

        // base variables
        pose_base << subscribe_base_pose_[0],subscribe_base_pose_[1],subscribe_base_pose_[5];
        twist_base << subscribe_base_twist_[0],subscribe_base_twist_[1],subscribe_base_twist_[5];
        // joint space variables
        Q << pose_base,arm_right.q_,arm_left.q_;
        Qdot << twist_base,arm_right.dq_,arm_left.dq_;
        // task space variables
        X.segment(0,6) = pose_arm(Q, 7, true);
        X.segment(6,6) = pose_arm(Q, 7, false);

        // initialize variables
        if(currentTime < 0)
        {
            initTime = (ros::Time::now()).toSec();
            last_print_time = 0.0;
            last_saved_time = 0.0;
            data_saved = false;
            q_home_right << 0.942613, -1.37299, -0.0288793, -1.84589,  0.131607, 1.92806, -0.40;
            q_home_left << -0.942613, -1.37299, -0.0288793, -1.84589, -0.131607, 1.92806,  0.40;
            Q_null << pose_base, q_home_right, q_home_left;
            X_home.segment(0,6) = pose_arm(Q_null, 7, true);
            X_home.segment(6,6) = pose_arm(Q_null, 7, false);
            X_init.segment(0,6) = pose_arm(Q, 7, true);
            X_init.segment(6,6) = pose_arm(Q, 7, false);
        }
        currentTime = (ros::Time::now()).toSec() - initTime;

        // Position Desired Interpolation
        if (currentTime <= homing_duration)
        {
            double t = currentTime / homing_duration; // Normalized time [0, 1]
            X_task = X_init + t * (X_home - X_init);
        }
        else
        {
            if(frequency < 500) {
                X_task = X;
            } else {
                // If robot is activated then adjust X_task
                // Q_null remains unchanged since its adjustment is not specified in the provided logic
            }
        }

        counter += 1;
        // Start timer
        auto start = std::chrono::high_resolution_clock::now();
        // Call whole body controller
        wbControllerOutput = wholeBodyController(X_task, Q_null, Q, Qdot, currentTime);            
        // Stop timer
        auto stop = std::chrono::high_resolution_clock::now();
        // Calculate duration
        duration_function = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        tau_wb = wbControllerOutput.tau_wb;
        cmd_vel_msg.linear.x = wbControllerOutput.v_wb(0);
        cmd_vel_msg.angular.z = wbControllerOutput.v_wb(1);

        // send torque desired commands for each arm
        arm_right.tau_d_ << tau_wb.segment(3,7);
        arm_left.tau_d_ << tau_wb.segment(10,7);

        // Publish the cmd_vel message
        garmiCmdVelPub_.publish(cmd_vel_msg);

        // Check if 1.5 seconds have passed since the last print
        // Printing the values
        if (currentTime - last_print_time >= 1.5)
        {
            frequency = (counter - last_counter)/1.5;
            std::cout << "\ntime: " << currentTime - homing_duration << " sec"<< std::endl;
            std::cout << "Function execution time: " << duration_function.count() << " microseconds." << std::endl;
            std::cout << "frequency: " << frequency << " hz"<<std::endl;
            std::cout << "Q_: " <<  Q.transpose() << std::endl;
            std::cout << "X_: " <<  X.transpose() << std::endl;
            //std::cout << "subscribe_arm_wrench_: " <<  subscribe_arm_wrench_.transpose() << std::endl;

            // Update the last print time
            last_print_time = currentTime;
            last_counter = counter;
        }

       if (currentTime < record_time && currentTime - last_saved_time > 0.1)
        {
            Matrix<double, 6, 1> X_right_arm;
            Matrix<double, 6, 1> X_left_arm;
            
            // Recording data with correct sequence numbers in comments
            rec.addToRec(currentTime);                // 1
            rec.addToRec(Q);                                            // 2-18
            
            X_right_arm << X.segment(0,6);
            rec.addToRec(X_right_arm);                                  // 19-24
            
            X_left_arm << X.segment(6,6);
            rec.addToRec(X_left_arm);                                   // 25-30
            
            rec.next();
            last_saved_time = currentTime;
        }
        else if (currentTime >= record_time && !data_saved)
        {
            rec.saveData();
            data_saved = true;
        }

        // Spin ROS callbacks
        //ros::spinOnce();

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
