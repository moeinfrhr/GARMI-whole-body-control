#pragma once

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>

#include <alpine_msgs/TaskSpaceConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace arm_control
{

class ExoGarmiController : public controller_interface::MultiInterfaceController<
                                franka_hw::FrankaModelInterface,
                                hardware_interface::EffortJointInterface,
                                franka_hw::FrankaStateInterface> {
public:

    struct ArmContainer {
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        double time_;
        double filter_params_;
        double nullspace_stiffness_;
        double nullspace_stiffness_target_;
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
        Eigen::Matrix<double, 7, 1> q_;
        Eigen::Matrix<double, 7, 1> dq_;
        Eigen::Matrix<double, 7, 1> q_init_;
        Eigen::Matrix<double, 7, 1> q_target_;
        Eigen::Matrix<double, 7, 1> q_home_end_;
        Eigen::Matrix<double, 7, 1> q_home_;
        Eigen::Matrix<double, 7, 4> q_home_coeficients_;
        Eigen::Matrix<double, 7, 1> q_d_nullspace_;
        bool arm_left;
        
        // exo commands to garmi:
        double exo_time_;
        double homing_duration_;
        Eigen::Matrix<double, 7, 1> relative_joint_position_d_target_;
        Eigen::Matrix<double, 7, 1> joint_velocity_d_target_;
        Eigen::Matrix<double, 7, 1> tau_exo_;
        Eigen::Matrix<double, 7, 1> tau_garmi_;

        Eigen::Vector3d position_init_;
        Eigen::Vector3d position_d_;
        Eigen::Vector3d position_d_temp_;
        Eigen::Quaterniond orientation_d_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;
        Eigen::Quaterniond orientation_init_;
        Eigen::VectorXd virtual_walls_{6};

        Eigen::Matrix<double, 6, 6> cartesian_P_;
        Eigen::Matrix<double, 6, 6> cartesian_K_;

        bool coeff_cal_{false};
    };

    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;

    void starting(const ros::Time &) override;

    void stopping(const ros::Time &) override;

    void update(const ros::Time &, const ros::Duration &period) override;

protected:
    std::map<std::string, ArmContainer> arms_;

    void dynamic_reconfigure_callback(alpine_msgs::TaskSpaceConfig &config,
                                      uint32_t level);

    void step(ArmContainer &, const ros::Duration &period );

    void start_arm(ArmContainer &);

    bool init_arm(hardware_interface::RobotHW *robot_hw, const std::string arm_id,
                           std::vector<std::string> joint_names);

    void update_parameters(ArmContainer &);

    // Saturation
    const double delta_tau_max_{1.0};
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    Eigen::Matrix<double, 7, 1> gohome(ArmContainer &arm, double t);

    std::string arm_id;
    std::vector<double> lower_joint_limits_, upper_joint_limits_;
    ros::Subscriber sub_;
    ros::Subscriber sub_relative_;

    ros::Subscriber exoJointStateSub;
    ros::Subscriber exoTimeSub;
    std::mutex mutex_;

private:
    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<alpine_msgs::TaskSpaceConfig>>
        dynamic_reconfigure_server_;
};

} // namespace arm_control
