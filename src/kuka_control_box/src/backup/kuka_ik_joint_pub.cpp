#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "Eigen/Dense"
#include "kuka_control_box/Robotics.h"
#include <iostream>

class KukaIKJointPub : public rclcpp::Node
{
public:
    KukaIKJointPub() : Node("kuka_ik_joint_pub")
    {
        // Publisher for Joint angles
        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

        // Timer to periodically prompt for input and publish joint angles
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&KukaIKJointPub::publish_joint_angles, this));
    }

private:
    void publish_joint_angles()
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d P;
        double angle_x_deg, angle_y_deg, angle_z_deg;
        double angle_x_rad, angle_y_rad, angle_z_rad;

        // Prompt user for input for P
        std::cout << "Enter position vector P (3 values in mm): ";
        std::cin >> P[0] >> P[1] >> P[2];
        
        // Convert P from mm to m
        P = P / 1000.0;

        // Prompt user for input for rotation angles
        std::cout << "Enter rotation angles for X, Y, Z (in degrees): ";
        std::cin >> angle_x_deg >> angle_y_deg >> angle_z_deg;
        
        // Convert degrees to radians
        angle_x_rad = angle_x_deg * M_PI / 180.0;
        angle_y_rad = angle_y_deg * M_PI / 180.0;
        angle_z_rad = angle_z_deg * M_PI / 180.0;
    
        // Create the rotation matrix from input angles
        R = Eigen::AngleAxisd(angle_x_rad, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(angle_y_rad, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(angle_z_rad, Eigen::Vector3d::UnitZ());
        
        std::cout << "Rotation matrix R:\n" << R << std::endl;

        // Call to inverse kinematics
        std::vector<Eigen::VectorXd> solutions = Robotics::kuka_ik(R, P);

        // Publish joint angles (only the first solution for simplicity)
        if (!solutions.empty())
        {
            std::vector<double> positions(6);
            for (int i = 0; i < solutions[0].size(); ++i)
            {
                positions[i] = solutions[0][i];
                RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i + 1, solutions[0][i]);
            }

            trajectory_msgs::msg::JointTrajectory traj_msg;

            traj_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = positions;
            point.time_from_start.sec = 1; // Example duration, adjust as needed
            point.time_from_start.nanosec = 0;

            traj_msg.points.push_back(point);

            RCLCPP_INFO(this->get_logger(), "Publishing joint trajectory");
            joint_pub_->publish(traj_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No IK solutions found");
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KukaIKJointPub>());
    rclcpp::shutdown();
    return 0;
}
