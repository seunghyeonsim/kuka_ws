#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "kuka_control_box_srvs/srv/kuka_task_input.hpp"
#include "nav_msgs/msg/odometry.hpp" // Include Odometry message
#include "Eigen/Dense"
#include "kuka_control_box/Robotics.h"
#include <iostream>

using namespace std::chrono_literals;

class KukaIKServiceServer : public rclcpp::Node
{
public:
    KukaIKServiceServer() : Node("kuka_ik_service_server")
    {
        // Publisher for Joint angles
        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

        // Service server for computing IK
        service_ = this->create_service<kuka_control_box_srvs::srv::KukaTaskInput>(
            "kuka_task_input",
            std::bind(&KukaIKServiceServer::compute_ik_callback, this, std::placeholders::_1, std::placeholders::_2));

        end_effector_pose_received_ = false; // 초기화
    }

private:
    void compute_ik_callback(const std::shared_ptr<kuka_control_box_srvs::srv::KukaTaskInput::Request> request,
                             std::shared_ptr<kuka_control_box_srvs::srv::KukaTaskInput::Response> response)
    {
        // Convert input values to meters and radians
        double x = request->x / 1000.0;  // mm to meters
        double y = request->y / 1000.0;  // mm to meters
        double z = request->z / 1000.0;  // mm to meters
        double roll = request->roll * M_PI / 180.0;  // degrees to radians
        double pitch = request->pitch * M_PI / 180.0;  // degrees to radians
        double yaw = request->yaw * M_PI / 180.0;  // degrees to radians

        // Create rotation matrix from roll, pitch, yaw
        Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Vector3d target_position(x, y, z);

        // Call to inverse kinematics
        std::vector<Eigen::VectorXd> solutions = Robotics::kuka_ik(R, target_position);

        // Publish joint angles (only the first solution for simplicity)
        if (!solutions.empty())
        {
            std::vector<double> positions(7);
            for (int i = 0; i < solutions[0].size(); ++i)
            {
                positions[i] = solutions[0][i];
                RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i + 1, solutions[0][i]);
            }
            positions[6] = 0.0;

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

            // 로봇이 다 움직인 후에 구독자 생성
            end_effector_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/gazebo/end_effector_pose", 10,
                [this, response, roll, pitch, yaw](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    end_effector_pose_ = msg->pose.pose;
                    end_effector_pose_received_ = true;
                    RCLCPP_INFO(this->get_logger(), "End effector pose updated.");

                    // 목표 위치와의 차이를 비교할 수 있습니다.
                    // 예시로 위치를 출력합니다.
                    print_end_effector_pose(roll, pitch, yaw);

                    // 구독 해제
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

    void print_end_effector_pose(double roll, double pitch, double yaw)
    {
        if (end_effector_pose_)
        {
            const auto& position = end_effector_pose_->position;
            const auto& orientation = end_effector_pose_->orientation;

            // 위치를 mm로 변환
            double x_mm = position.x * 1000.0;
            double y_mm = position.y * 1000.0;
            double z_mm = position.z * 1000.0;

            // 오리엔테이션을 rad에서 deg로 변환
            Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
            Eigen::Matrix3d rotation_matrix_from_quaternion = q.toRotationMatrix();

            // 변환 행렬들
            Eigen::Matrix3d transformation_matrix1;
            transformation_matrix1 << -1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1;
            Eigen::Matrix3d transformation_matrix2;
            transformation_matrix2 << 1, 0, 0,
                                      0, -1, 0,
                                      0, 0, 1;
            Eigen::Matrix3d transformation_matrix3;
            transformation_matrix3 << 1, 0, 0,
                                      0, 1, 0,
                                      0, 0, -1;
            Eigen::Matrix3d transformation_matrix4;
            transformation_matrix4 << -1, 0, 0,
                                      0, -1, 0,
                                      0, 0, 1;
            Eigen::Matrix3d transformation_matrix5;
            transformation_matrix5 << -1, 0, 0,
                                      0, 1, 0,
                                      0, 0, -1;
            Eigen::Matrix3d transformation_matrix6;
            transformation_matrix6 << -1, 0, 0,
                                      0, -1, 0,
                                      0, 0, -1;
            Eigen::Matrix3d transformation_matrix7;
            transformation_matrix7 << -1, 0, 0,
                                      0, -1, 0,
                                      0, 0, -1;

            std::vector<Eigen::Matrix3d> transformed_rotation_matrices(7);
            transformed_rotation_matrices[0] = transformation_matrix1 * rotation_matrix_from_quaternion;
            transformed_rotation_matrices[1] = transformation_matrix2 * rotation_matrix_from_quaternion;
            transformed_rotation_matrices[2] = transformation_matrix3 * rotation_matrix_from_quaternion;
            transformed_rotation_matrices[3] = transformation_matrix4 * rotation_matrix_from_quaternion;
            transformed_rotation_matrices[4] = transformation_matrix5 * rotation_matrix_from_quaternion;
            transformed_rotation_matrices[5] = transformation_matrix6 * rotation_matrix_from_quaternion;
            transformed_rotation_matrices[6] = transformation_matrix7 * rotation_matrix_from_quaternion;

            // 결과 출력
            RCLCPP_INFO(this->get_logger(), "End Effector Position: [x: %f mm, y: %f mm, z: %f mm]", x_mm, y_mm, z_mm);

            // 27가지 경우의 수로 오일러 각도 계산 및 출력
            for (int j = 0; j < 6; ++j)
            {
                std::vector<Eigen::Vector3d> euler(27);
                euler[0] = transformed_rotation_matrices[j].eulerAngles(0, 0, 0) * (180.0 / M_PI);
                euler[1] = transformed_rotation_matrices[j].eulerAngles(0, 0, 1) * (180.0 / M_PI);
                euler[2] = transformed_rotation_matrices[j].eulerAngles(0, 0, 2) * (180.0 / M_PI);
                euler[3] = transformed_rotation_matrices[j].eulerAngles(0, 1, 0) * (180.0 / M_PI);
                euler[4] = transformed_rotation_matrices[j].eulerAngles(0, 1, 1) * (180.0 / M_PI);
                euler[5] = transformed_rotation_matrices[j].eulerAngles(0, 1, 2) * (180.0 / M_PI);
                euler[6] = transformed_rotation_matrices[j].eulerAngles(0, 2, 0) * (180.0 / M_PI);
                euler[7] = transformed_rotation_matrices[j].eulerAngles(0, 2, 1) * (180.0 / M_PI);
                euler[8] = transformed_rotation_matrices[j].eulerAngles(0, 2, 2) * (180.0 / M_PI);
                euler[9] = transformed_rotation_matrices[j].eulerAngles(1, 0, 0) * (180.0 / M_PI);
                euler[10] = transformed_rotation_matrices[j].eulerAngles(1, 0, 1) * (180.0 / M_PI);
                euler[11] = transformed_rotation_matrices[j].eulerAngles(1, 0, 2) * (180.0 / M_PI);
                euler[12] = transformed_rotation_matrices[j].eulerAngles(1, 1, 0) * (180.0 / M_PI);
                euler[13] = transformed_rotation_matrices[j].eulerAngles(1, 1, 1) * (180.0 / M_PI);
                euler[14] = transformed_rotation_matrices[j].eulerAngles(1, 1, 2) * (180.0 / M_PI);
                euler[15] = transformed_rotation_matrices[j].eulerAngles(1, 2, 0) * (180.0 / M_PI);
                euler[16] = transformed_rotation_matrices[j].eulerAngles(1, 2, 1) * (180.0 / M_PI);
                euler[17] = transformed_rotation_matrices[j].eulerAngles(1, 2, 2) * (180.0 / M_PI);
                euler[18] = transformed_rotation_matrices[j].eulerAngles(2, 0, 0) * (180.0 / M_PI);
                euler[19] = transformed_rotation_matrices[j].eulerAngles(2, 0, 1) * (180.0 / M_PI);
                euler[20] = transformed_rotation_matrices[j].eulerAngles(2, 0, 2) * (180.0 / M_PI);
                euler[21] = transformed_rotation_matrices[j].eulerAngles(2, 1, 0) * (180.0 / M_PI);
                euler[22] = transformed_rotation_matrices[j].eulerAngles(2, 1, 1) * (180.0 / M_PI);
                euler[23] = transformed_rotation_matrices[j].eulerAngles(2, 1, 2) * (180.0 / M_PI);
                euler[24] = transformed_rotation_matrices[j].eulerAngles(2, 2, 0) * (180.0 / M_PI);
                euler[25] = transformed_rotation_matrices[j].eulerAngles(2, 2, 1) * (180.0 / M_PI);
                euler[26] = transformed_rotation_matrices[j].eulerAngles(2, 2, 2) * (180.0 / M_PI);

                for (int i = 0; i < 27; ++i)
                {
                    RCLCPP_INFO(this->get_logger(), "Transformed End Effector Orientation Matrix %d Case %d: [roll: %f deg, pitch: %f deg, yaw: %f deg]", j + 1, i + 1, euler[i][0], euler[i][1], euler[i][2]);
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "End effector pose not available");
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::Service<kuka_control_box_srvs::srv::KukaTaskInput>::SharedPtr service_;
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
