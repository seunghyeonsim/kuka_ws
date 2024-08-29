#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "kuka_control_box_srvs/srv/kuka_transform_input.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "Eigen/Dense"
#include "kuka_control_box/Robotics.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class KukaIKServiceServer : public rclcpp::Node
{
public:
    KukaIKServiceServer() : Node("kuka_ik_service_server")
    {
        // Publisher for Joint angles
        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

        // Service server for computing IK
        service_ = this->create_service<kuka_control_box_srvs::srv::KukaTransformInput>(
            "kuka_transform_input",
            std::bind(&KukaIKServiceServer::compute_ik_callback, this, std::placeholders::_1, std::placeholders::_2));

        end_effector_pose_received_ = false;
    }

private:
    void compute_ik_callback(const std::shared_ptr<kuka_control_box_srvs::srv::KukaTransformInput::Request> request,
                             std::shared_ptr<kuka_control_box_srvs::srv::KukaTransformInput::Response> response)
    {
        // 요청으로부터 TransformStamped 메시지를 추출
        const auto &transform = request->target_transform.transform;
        const auto &translation = transform.translation;
        const auto &rotation = transform.rotation;

        // Translation 벡터 추출
        double x = translation.x; // 이미 m 단위
        double y = translation.y;
        double z = translation.z;

        // Rotation matrix를 Quaternion에서 변환
        Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
        Eigen::Matrix3d R = q.toRotationMatrix();

        Eigen::Vector3d target_position(x, y, z);

        // Inverse kinematics 계산 호출
        std::vector<Eigen::VectorXd> solutions = Robotics::kuka_ik(R, target_position);

        // Joint angles를 게시 (첫 번째 솔루션만 사용)
        if (!solutions.empty())
        {
            std::vector<double> positions(7);
            for (int i = 0; i < solutions[0].size(); ++i)
            {
                positions[i] = solutions[0][i];
                RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i + 1, solutions[0][i]);
            }
            positions[6] = 0.0; // End effector joint

            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "end_effector_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = positions;
            point.time_from_start.sec = 1; // Example duration, adjust as needed
            point.time_from_start.nanosec = 0;

            traj_msg.points.push_back(point);

            RCLCPP_INFO(this->get_logger(), "Publishing joint trajectory");
            joint_pub_->publish(traj_msg);

            // Wait for a brief moment to simulate robot motion
            std::this_thread::sleep_for(2s);

            // Subscribe to end effector pose
            end_effector_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/gazebo/end_effector_pose", 10,
                [this, response](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    end_effector_pose_ = msg->pose.pose;
                    end_effector_pose_received_ = true;
                    RCLCPP_INFO(this->get_logger(), "End effector pose updated.");

                    // Print end effector pose
                    print_end_effector_pose();

                    // Unsubscribe
                    end_effector_pose_sub_.reset();
                });

            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Joint trajectory published. Subscribing to end effector pose.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No IK solutions found");
            response->success = false;
        }
    }
    // REMOVABLE LATER
    void print_end_effector_pose()
    {
        if (end_effector_pose_)
        {
            const auto& position = end_effector_pose_->position;
            const auto& orientation = end_effector_pose_->orientation;

            // Convert position to mm
            double x_mm = position.x * 1000.0;
            double y_mm = position.y * 1000.0;
            double z_mm = position.z * 1000.0;

            // Convert orientation to roll, pitch, yaw
            Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
            Eigen::Matrix3d rotation_matrix_from_quaternion = q.toRotationMatrix();

            // Print results
            RCLCPP_INFO(this->get_logger(), "End Effector Position: [x: %f mm, y: %f mm, z: %f mm]", x_mm, y_mm, z_mm);
            
            RCLCPP_INFO(this->get_logger(), "Rotation Matrix from Quaternion:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
                        rotation_matrix_from_quaternion(0, 0), rotation_matrix_from_quaternion(0, 1), rotation_matrix_from_quaternion(0, 2),
                        rotation_matrix_from_quaternion(1, 0), rotation_matrix_from_quaternion(1, 1), rotation_matrix_from_quaternion(1, 2),
                        rotation_matrix_from_quaternion(2, 0), rotation_matrix_from_quaternion(2, 1), rotation_matrix_from_quaternion(2, 2));

        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "End effector pose not available");
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::Service<kuka_control_box_srvs::srv::KukaTransformInput>::SharedPtr service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr end_effector_pose_sub_;
    std::optional<geometry_msgs::msg::Pose> end_effector_pose_;
    bool end_effector_pose_received_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KukaIKServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
