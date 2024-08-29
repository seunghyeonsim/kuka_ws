#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "kuka_control_box/Robotics.h"

using namespace Eigen;

class KukaIKNode : public rclcpp::Node {
public:
    KukaIKNode() : Node("kuka_ik_node") {
        RCLCPP_INFO(this->get_logger(), "Kuka IK Node has been started.");
        // kuka_ik 함수 테스트
        testKukaIKFunction();
    }

private:
    void testKukaIKFunction() {
        Matrix3d R = Matrix3d::Identity();
        Vector3d P(0.2, 0.5, 2.5);

        std::vector<VectorXd> solutions = kuka_ik(R, P);

        for (size_t i = 0; i < solutions.size(); ++i) {
            std::stringstream ss;
            ss << solutions[i].transpose().format(IOFormat(StreamPrecision, DontAlignCols, ", ", "\n"));
            RCLCPP_INFO(this->get_logger(), "Solution %zu: %s", i + 1, ss.str().c_str());
        }
    }

    std::vector<VectorXd> kuka_ik(const Matrix3d& R, const Vector3d& P) {
        std::vector<VectorXd> sols;
        VectorXd Theta0(6);
        Theta0 << M_PI, M_PI, M_PI/2, M_PI, M_PI, 0;
        VectorXd D(6);
        D << 0.645, 0, 0, 1.220, 0, 0.240;
        VectorXd A(6);
        A << -0.330, 1.150, -0.115, 0, 0, 0;
        VectorXd Alpha(6);
        Alpha << M_PI/2, 0, M_PI/2, M_PI/2, M_PI/2, 0;

        Matrix4d S6;
        S6.block<3,3>(0,0) = R;
        S6.block<3,1>(0,3) = P;
        S6.block<1,3>(3,0) = Vector3d::Zero().transpose();
        S6(3,3) = 1;

        Vector3d p = S6.block<3,1>(0,3);
        Matrix3d R6 = S6.block<3,3>(0,0);
        Vector3d w6(0,0,D(5));

        Vector3d c = p - R6 * w6;
        double px = c(0), py = c(1), pz = c(2);

        double d1 = D(0);
        double L2 = A(1);
        double L3 = std::sqrt(D(3)*D(3) + A(2)*A(2));

        // theta1
        double theta1_1 = std::atan2(py, px);
        double theta1_2 = std::atan2(-py, -px);
        double c1_1 = std::cos(theta1_1), s1_1 = std::sin(theta1_1);
        double c1_2 = std::cos(theta1_2), s1_2 = std::sin(theta1_2);

        // theta3 for theta1_1
        double c3_1 = ((px - 0.330*c1_1)*(px - 0.330*c1_1) + (py - 0.330*s1_1)*(py - 0.330*s1_1) + (pz - d1)*(pz - d1) - L2*L2 - L3*L3) / (2*L2*L3);
        double s3_1 = std::sqrt(1 - c3_1*c3_1);
        double theta3_1_1 = std::atan2(s3_1, c3_1) - std::atan2(0.115, 1.220);
        double theta3_1_2 = std::atan2(-s3_1, c3_1) - std::atan2(0.115, 1.220);

        // theta3 for theta1_2
        double c3_2 = ((px - 0.330*c1_2)*(px - 0.330*c1_2) + (py - 0.330*s1_2)*(py - 0.330*s1_2) + (pz - d1)*(pz - d1) - L2*L2 - L3*L3) / (2*L2*L3);
        double s3_2 = std::sqrt(1 - c3_2*c3_2);
        double theta3_2_1 = std::atan2(s3_2, c3_2) - std::atan2(0.115, 1.220);
        double theta3_2_2 = std::atan2(-s3_2, c3_2) - std::atan2(0.115, 1.220);

        std::vector<VectorXd> theta(4, VectorXd(3));
        Matrix2d A2;
        Vector2d B2;

        // theta1_1, theta3_1_1 and theta3_1_2
        A2 << L2 + L3*c3_1, -L3*s3_1,
              L3*s3_1, L2 + L3*c3_1;
        B2 << c1_1*px + s1_1*py - 0.330,
              pz - d1;
        Vector2d c2s2 = A2.colPivHouseholderQr().solve(B2);
        double theta2 = std::atan2(c2s2(1), c2s2(0));
        theta[0] << theta1_1, -theta2, -theta3_1_1;

        A2 << L2 + L3*c3_1, -L3*(-s3_1),
              L3*(-s3_1), L2 + L3*c3_1;
        c2s2 = A2.colPivHouseholderQr().solve(B2);
        theta2 = std::atan2(c2s2(1), c2s2(0));
        theta[1] << theta1_1, -theta2, -theta3_1_2;

        // theta1_2, theta3_2_1 and theta3_2_2
        A2 << L2 + L3*c3_2, -L3*s3_2,
              L3*s3_2, L2 + L3*c3_2;
        B2 << c1_2*px + s1_2*py - 0.330,
              pz - d1;
        c2s2 = A2.colPivHouseholderQr().solve(B2);
        theta2 = std::atan2(c2s2(1), c2s2(0));
        theta[2] << theta1_2, -theta2, -theta3_2_1;

        A2 << L2 + L3*c3_2, -L3*(-s3_2),
              L3*(-s3_2), L2 + L3*c3_2;
        c2s2 = A2.colPivHouseholderQr().solve(B2);
        theta2 = std::atan2(c2s2(1), c2s2(0));
        theta[3] << theta1_2, -theta2, -theta3_2_2;

        for (int i = 0; i < 4; ++i) {
            VectorXd thetalist = theta[i];
            if (thetalist(0) <= 185*M_PI/180 && thetalist(0) >= -185*M_PI/180 && (thetalist(1) <= -5*M_PI/180 && thetalist(1) >= -140*M_PI/180) && thetalist(2) <= 168*M_PI/180 && thetalist(2) >= -120*M_PI/180) {
                sols.push_back(thetalist);
            }
        }

        Matrix4d M;
        M << 0, 0, 1, 2.7,
             0, 1, 0, 0,
            -1, 0, 0, 0.76,
             0, 0, 0, 1;
        Eigen::Matrix<double, 6, 1> S1, S2, S3;
        S1 << 0, 0, 1, 0, 0, 0;
        S2 << 0, 1, 0, -0.645, 0, 0.330;
        S3 << 0, 1, 0, -0.645, 0, 1.480;

        Eigen::Matrix<double, 6, 3> Slist;
        Slist << S1, S2, S3;

        Matrix3d R06 = R;
        Matrix3d R03;
        std::vector<VectorXd> f_sols;

        for (const auto& sol : sols) {
            R03 = Robotics::FKinSpace(M, Slist, sol).block<3, 3>(0, 0);
            std::cout << "R03: \n" << R03 << std::endl;

            Matrix3d R36 = R03.transpose() * R06;

            double theta4_1 = std::atan2(R36(1,2), R36(0,2));
            double theta5_1 = std::atan2(std::sqrt(R36(0,2)*R36(0,2) + R36(1,2)*R36(1,2)), R36(2,2));
            double theta6_1 = std::atan2(-R36(2,1), R36(2,0));
            if (theta4_1 <= 350*M_PI/180 && theta4_1 >= -350*M_PI/180 && theta5_1 <= 122.5*M_PI/180 && theta5_1 >= -122.5*M_PI/180 && theta6_1 <= 350*M_PI/180 && theta6_1 >= -350*M_PI/180) {
                VectorXd temp(6);
                temp << sol(0), sol(1), sol(2), theta4_1, theta5_1, theta6_1;
                f_sols.push_back(temp);
                break;
            }

            double theta4_2 = theta4_1 + M_PI;
            double theta5_2 = -theta5_1;
            double theta6_2 = theta6_1 + M_PI;
            if (theta4_2 <= 350*M_PI/180 && theta4_2 >= -350*M_PI/180 && theta5_2 <= 122.5*M_PI/180 && theta5_2 >= -122.5*M_PI/180 && theta6_2 <= 350*M_PI/180 && theta6_2 >= -350*M_PI/180) {
                VectorXd temp(6);
                temp << sol(0), sol(1), sol(2), theta4_2, theta5_2, theta6_2;
                f_sols.push_back(temp);
                break;
            }
        }

        return f_sols;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KukaIKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
