#pragma once

#include <arm_control/whole_body_controller.h>
#include <alpine_msgs/DualTaskSpaceConfig.h>
#include <alpine_msgs/DualPose.h>
#include <alpine_msgs/OdomAndJoints.h>
#include <alpine_msgs/DualTwist.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include "../src/redundancy_resolution/recorder.h"
#include <arm_control/moein_helpers/redundancy_resolution.h>

namespace arm_control
{

    class DualWholeBodyController : public WholeBodyController
    {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;

        void starting(const ros::Time &) override;

        void stopping(const ros::Time &) override;

        void publish(const ros::Time &time, const ros::Duration &period, ros::NodeHandle &node_handle);

        void update(const ros::Time &, const ros::Duration &period) override;

    protected:
        // define control/trajectory variables
        VectorXd tauR_wb;
        VectorXd X, X_task;
        VectorXd X_init;
        VectorXd X_home,X_goal;
        VectorXd X_taskSub;
        VectorXd Q_nullSub;
        VectorXd q_home_right, q_home_left;
        VectorXd q_grasping_right, q_grasping_left, q_towel_right, q_handover_right, q_post_handover_right, q_get_my_hand_right, q_get_my_hand_left;
        VectorXd Q, Qdot, Q_null, Q_home;  // define generalized coordinate
        VectorXd tau_mes;  // define whole body measured torques
        VectorXd tauR_ext;
        VectorXd delta_frc;
        VectorXd F_taskSub;
        VectorXd F_ext;
        Vector3d pose_base;
        Vector3d twist_base;
        VectorXd subscribe_arm_wrench_;
        double ratioStiffnessTask;
        double ratioDampingTask;
        double ratioStiffnessNull;
        double ratioDampingNull;
        
        std::string arm_id_left_, arm_id_right_;
        double manipulability_right;
        double manipulability_left;
        double prev_exo_time;
        double exo_time_fixed_time_out;
        double last_print_time;
        double last_saved_time;
        double initTime;
        double currentTime;
        bool data_saved;
        bool running;
        bool first_time;
        bool garmi_user_stopped;
        double counter;
        double last_counter;
        double frequency;
        std::chrono::microseconds duration_function;
        Eigen::Vector3d pose_base_admitance_;
        Eigen::Vector3d twist_base_admitance_;
        Eigen::MatrixXd Jreduced;
        ControlOutput wbControllerOutput;

        double homing_duration;
        double timeSinceHoming;
        double record_time;
        double SampletimeInit;
        int NODataRec;

        void dynamic_reconfigure_callback(alpine_msgs::DualTaskSpaceConfig &config,
                                          uint32_t level);

    private:
        // Dynamic reconfigure
        std::unique_ptr<dynamic_reconfigure::Server<alpine_msgs::DualTaskSpaceConfig>>
            dynamic_reconfigure_server_;
        
        // define publishers
        ros::Publisher garmiCmdVelPub_;
        ros::Publisher handRPub;
        ros::Publisher handLPub;
        ros::Publisher garmiTaskSpacePub;
        ros::Publisher garmiNullSpacePub;

        ros::Subscriber garmiBaseStateSub;
        ros::Subscriber sub_right_arm;
        ros::Subscriber sub_left_arm;
        ros::Subscriber garmiTaskSpaceSub;
        ros::Subscriber garmiNullSpaceSub;
        ros::Subscriber exoTimeSub;
        
        // Creating a geometry_msgs/Twist message
        geometry_msgs::Twist cmd_vel_msg;
        nav_msgs::Odometry base_odom;
        std_msgs::Float64 rightHandStatus;
        std_msgs::Float64 leftHandStatus;
        Eigen::VectorXd subscribe_base_pose_;
        Eigen::VectorXd subscribe_base_twist_;
        alpine_msgs::DualTwist task_space_state_msg;
        alpine_msgs::OdomAndJoints null_space_state_msg;
        Recorder rec;
    };

} // namespace arm_control
