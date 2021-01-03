#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter() {
    A_ = Identity3d();
    B_ = Identity3d();
    C_ = Identity3d();
    D_ = Identity3d();
    P_ = Zero3d();
    Q_ = Zero3d();
    R_ = Zero3d();
    x_ = Vector3d(0.0, 0.0, 0.0);
}

void KalmanFilter::InitState(const Vector3d& x) {
    x_ = x;
}

void KalmanFilter::SetStateEquation(const Matrix3d& A, const Matrix3d& B) {
    A_ = A;
    B_ = B;
}

void KalmanFilter::SetObserveEquation(const Matrix3d& C, const Matrix3d& D) {
    C_ = C;
    D_ = D;
}

void KalmanFilter::SetVariance(const Matrix3d& Q, const Matrix3d& R) {
    Q_ = Q;
    R_ = R;
}

Vector3d KalmanFilter::Apply(const Vector3d& y, const Vector3d& u) {
    // update x from odometry
    const Vector3d x_odometry = A_ * x_ + B_ * u;
    #ifdef USE_BLA
    const Matrix3d P_odometry = A_ * P_ * ~A_ + Q_;
    // update x from observation
    const Matrix3d K_kalman = P_odometry * ~C_ * (C_ * P_odometry * ~C_ + R_).Inverse();
    #else
    const Matrix3d P_odometry = A_ * P_ * A_.transpose() + Q_;
    // update x from observation
    const Matrix3d K_kalman = P_odometry * C_.transpose() * (C_ * P_odometry * C_.transpose() + R_).inverse();
    #endif
    const Vector3d y_odometry = C_ * x_odometry + D_ * u;
    x_ = x_odometry + K_kalman * (y - y_odometry);
    P_ = (Identity3d() - K_kalman * C_) * P_odometry;

    return x_;
}

Vector3d KalmanFilter::GetState() const {
    return this->x_;
}
