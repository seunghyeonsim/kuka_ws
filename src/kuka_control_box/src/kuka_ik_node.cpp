#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "kuka_control_box/Robotics.h" // Robotics 라이브러리 포함

using namespace Eigen;


class KukaIKNode : public rclcpp::Node {
public:
    KukaIKNode() : Node("kuka_ik_node") {
        RCLCPP_INFO(this->get_logger(), "Kuka IK Node has been started.");
        // Eigen 버전 정보를 출력
        printEigenVersion();
        
        // Nearzero 함수 테스트
        testNearzeroFunction();
    }

private:
    void printEigenVersion() {
        RCLCPP_INFO(this->get_logger(), "Eigen version: %d.%d.%d",
                    EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
    }
    
    void testNearzeroFunction() {

        float testValue1 = 0.0000001;
        float testValue2 = 0.00001;
        bool result1 = Nearzero(testValue1);
        bool result2 = Nearzero(testValue2);

        RCLCPP_INFO(this->get_logger(), "Testing Nearzero with %f: %s", testValue1, result1 ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Testing Nearzero with %f: %s", testValue2, result2 ? "true" : "false");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KukaIKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
