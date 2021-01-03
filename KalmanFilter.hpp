#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include "MathUtility.hpp"

class KalmanFilter {
    public:
    KalmanFilter();
    void InitState(const Vector3d& x);
    void SetStateEquation(const Matrix3d& A, const Matrix3d& B);
    void SetObserveEquation(const Matrix3d& C, const Matrix3d& D);
    void SetVariance(const Matrix3d& Q, const Matrix3d& R);
    Vector3d Apply(const Vector3d& y, const Vector3d& u);
    Vector3d GetState() const;
    private:
    Vector3d x_;
    Matrix3d A_, B_, C_, D_;
    Matrix3d P_, Q_, R_;
};

#endif
