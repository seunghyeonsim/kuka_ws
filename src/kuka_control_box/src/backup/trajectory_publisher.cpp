#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "Eigen/Dense"
#include "kuka_control_box/Robotics.h"

using std::placeholders::_1;

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher()
        : Node("trajectory_publisher")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

        // Timer to periodically prompt for input and publish the message
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TrajectoryPublisher::publish_trajectory, this));
    }

private:
    void publish_trajectory()
    {
        // Prompt user for input
        std::vector<double> positions(6);
        int sec;
        int nanosec;

        std::cout << "Enter positions for joints (6 values): ";
        for (int i = 0; i < 6; ++i)
        {
            std::cin >> positions[i];
        }
        std::cout << "Enter time_from_start (sec nanosec): ";
        std::cin >> sec >> nanosec;

        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start.sec = sec;
        point.time_from_start.nanosec = nanosec;

        message.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "Publishing joint trajectory");
        publisher_->publish(message);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
