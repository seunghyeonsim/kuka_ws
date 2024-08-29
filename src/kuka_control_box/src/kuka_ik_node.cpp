#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "kuka_control_box_srvs/srv/kuka_transform_input.hpp"
#include "kuka_control_box_srvs/srv/kuka_joint.hpp"
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
    KukaIKServiceServer() : Node("kuka_ik_node")
    {
        // Service server for receiving input transform and computing IK
        service_ = this->create_service<kuka_control_box_srvs::srv::KukaTransformInput>(
            "kuka_transform_input",
            std::bind(&KukaIKServiceServer::compute_ik_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Service client for sending joint commands to the actual robot
        client_ = this->create_client<kuka_control_box_srvs::srv::KukaJoint>("/kuka_joint");

        while (!client_->wait_for_service(1s) && rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the kuka_joint service to become available...");
        }

        if (!rclcpp::ok()) 
        {
            RCLCPP_INFO(this->get_logger(), "ROS is shutting down before the service became available.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Service server is ready and waiting for requests.");
    }

private:
    void compute_ik_callback(const std::shared_ptr<kuka_control_box_srvs::srv::KukaTransformInput::Request> request,
                             std::shared_ptr<kuka_control_box_srvs::srv::KukaTransformInput::Response> response)
    {
        // 요청으로부터 TransformStamped 메시지를 추출
        const auto &transform = request->target_transform.transform;
        const auto &translation = transform.translation;
        const auto &rotation = transform.rotation;

        double x = translation.x;
        double y = translation.y;
        double z = translation.z;

        Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
        Eigen::Matrix3d R = q.toRotationMatrix();

        Eigen::Vector3d target_position(x, y, z);

        std::vector<Eigen::VectorXd> solutions = Robotics::kuka_ik(R, target_position);

        if (!solutions.empty())
        {
            std::vector<double> positions(7);
            for (int i = 0; i < solutions[0].size(); ++i)
            {
                positions[i] = solutions[0][i];
                RCLCPP_INFO(this->get_logger(), "Joint %d: %f (radians)", i + 1, solutions[0][i]);
            }
            positions[6] = 0.0;

            std::vector<double> positions_in_degrees(7);
            for (int i = 0; i < 6; ++i)
            {
                positions_in_degrees[i] = positions[i] * (180.0 / M_PI);
                RCLCPP_INFO(this->get_logger(), "Joint %d: %f (degrees)", i + 1, positions_in_degrees[i]);
            }
            positions_in_degrees[6] = 0.0;

            auto joint_request = std::make_shared<kuka_control_box_srvs::srv::KukaJoint::Request>();
            joint_request->a1 = positions_in_degrees[0];
            joint_request->a2 = positions_in_degrees[1];
            joint_request->a3 = positions_in_degrees[2];
            joint_request->a4 = positions_in_degrees[3];
            joint_request->a5 = positions_in_degrees[4];
            joint_request->a6 = positions_in_degrees[5];

            // 비동기 서비스 요청 보내기
            auto result_future = client_->async_send_request(joint_request);

            // 결과를 기다리기 위해 wait_for 사용
            if (result_future.wait_for(5s) == std::future_status::ready)
            {
                try
                {
                    auto result = result_future.get();
                    if (result->success)
                    {
                        RCLCPP_INFO(this->get_logger(), "Successfully sent joint command to robot.");
                        response->success = true;
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Failed to send joint command to robot.");
                        response->success = false;
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                    response->success = false;
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for the joint command response.");
                response->success = false;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No IK solutions found");
            response->success = false;
        }
    }

    rclcpp::Service<kuka_control_box_srvs::srv::KukaTransformInput>::SharedPtr service_;
    rclcpp::Client<kuka_control_box_srvs::srv::KukaJoint>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KukaIKServiceServer>();

    // SingleThreadedExecutor 사용
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Executor 실행
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
