#ifndef ROBOTICS_H
#define ROBOTICS_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

namespace Robotics {
    bool Nearzero(double value);
    VectorXd so3ToVec(const MatrixXd& so3mat);
    VectorXd se3ToVec(const MatrixXd& se3mat);
    MatrixXd VecToso3(const VectorXd& omg);
    MatrixXd VecTose3(const VectorXd& V);
    MatrixXd MatrixExp3(const MatrixXd& so3mat);
    MatrixXd MatrixExp6(const MatrixXd& se3mat);
    MatrixXd FKinBody(const MatrixXd& M, const MatrixXd& Blist, const VectorXd& thetalist);
    MatrixXd FKinSpace(const MatrixXd& M, const MatrixXd& Slist, const VectorXd& thetalist);
    MatrixXd Adjoint(const MatrixXd& T);
    MatrixXd JacobianBody(const MatrixXd& Blist, const VectorXd& thetalist);
    std::vector<VectorXd> kuka_ik(const Matrix3d& R, const Vector3d& P);
}

#endif // ROBOTICS_H
