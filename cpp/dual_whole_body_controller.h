#pragma once

#include <arm_control/whole_body_controller.h>
#include <alpine_msgs/DualTaskSpaceConfig.h>
#include <alpine_msgs/DualPose.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "../src/redundancy_resolution/recorder.cpp"
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
        VectorXd tau_wb;
        VectorXd X, X_task;
        VectorXd X_init;
        VectorXd X_home;
        VectorXd q_home_right, q_home_left;
        VectorXd Q, Qdot, Q_null, Q_home;  // define generalized coordinate
        Vector3d pose_base;
        Vector3d twist_base;
        VectorXd subscribe_arm_wrench_;
        
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
        double counter;
        double last_counter;
        double frequency;
        std::chrono::microseconds duration_function;
        Eigen::Vector3d pose_base_admitance_;
        Eigen::Vector3d twist_base_admitance_;
        Eigen::MatrixXd Jreduced;
        ControlOutput wbControllerOutput;

        double homing_duration;
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
        ros::Subscriber garmiBaseStateSub;
        ros::Subscriber sub_right_arm;
        ros::Subscriber sub_left_arm;
        // Creating a geometry_msgs/Twist message
        geometry_msgs::Twist cmd_vel_msg;
        nav_msgs::Odometry base_odom;
        Eigen::VectorXd subscribe_base_pose_;
        Eigen::VectorXd subscribe_base_twist_;
        Recorder rec;
        Eigen::Matrix<double, 7, 1> zero_q_;

    };

} // namespace arm_control
