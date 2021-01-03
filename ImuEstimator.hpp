#ifndef _IMU_ESTIMATOR_H
#define _IMU_ESTIMATOR_H

#include "KalmanFilter.hpp"

#define NUM_CALIBRATION 100

class ImuEstimator {
    public:
    ImuEstimator();
    void StartCalibration();
    Vector3d UpdateRpy(const Vector3d& alpha, const Vector3d& omega, double dt);
    private:
    KalmanFilter estimator_;
    bool calibration_;
    unsigned int counter_;
    Vector3d rpy_observe_mean_, rpy_observe_var_;
    Vector3d omega_bias_;
    Vector3d RpyFromAccel(const Vector3d& alpha);
    Vector3d RpyFromGyro(const Vector3d& rpy_pre, const Vector3d& omega, double dt);
    Matrix3d JacobianRpy(const Vector3d& rpy, const Vector3d& omega, double dt);
    void Calibration(const Vector3d& rpy_observe);
};

#endif
