#include "ImuEstimator.hpp"
#include "MathConst.hpp"

ImuEstimator::ImuEstimator() {
    estimator_.InitState(Vector3d(0.0, 0.0, 0.0));
    estimator_.SetObserveEquation(Identity3d(), Zero3d());
    Matrix3d Q = Identity3d() * DEG_TO_RAD * DEG_TO_RAD; // sigma = 1 deg^2
    Matrix3d R = Identity3d() * 1e-6; // sigma = 1 deg^2
    estimator_.SetVariance(Q, R);
    calibration_ = false;
    counter_ = 0;
    omega_bias_ = Vector3d(0.0, 0.0, 0.0);
}

void ImuEstimator::StartCalibration() {
    if (calibration_ == false) {
        calibration_ = true;
        counter_ = 0;
        omega_bias_ = Vector3d(0.0, 0.0, 0.0);
        rpy_observe_mean_ = Vector3d(0.0, 0.0, 0.0);
        rpy_observe_var_ = Vector3d(0.0, 0.0, 0.0);
    }
}

Vector3d ImuEstimator::UpdateRpy(const Vector3d& alpha, const Vector3d& omega, double dt) {
    // update constant
    const Vector3d& rpy_kalman_pre = estimator_.GetState();
    const Matrix3d A = JacobianRpy(rpy_kalman_pre, omega, dt);
    estimator_.SetStateEquation(A, Zero3d());
    // estimate pose 
    const Vector3d rpy_observe = RpyFromAccel(alpha);
    const Vector3d u = Vector3d(0.0, 0.0, 0.0);
    const Vector3d rpy_estimate = estimator_.Apply(rpy_observe, u);

    if (calibration_) {
        // calibrate gyro bias
        omega_bias_ = (omega_bias_ * counter_ + omega) / (counter_ + 1);
        Calibration(rpy_observe);
    }
    return rpy_estimate;
}

Vector3d ImuEstimator::RpyFromAccel(const Vector3d& alpha) {
    const Vector3d alpha_0 = Vector3d(0.0, 0.0, -sqrt(alpha(X) * alpha(X) + alpha(Y) * alpha(Y) + alpha(Z) * alpha(Z)));
    return Rpy(alpha_0, alpha); // rotate coordinate
}

Vector3d ImuEstimator::RpyFromGyro(const Vector3d& rpy_pre, const Vector3d& omega, double dt) {
    return rpy_pre + omega * dt;
}

Matrix3d ImuEstimator::JacobianRpy(const Vector3d& rpy, const Vector3d& omega, double dt) {
    Matrix3d jacobian;
    const double delta_rpy = 1e-5;
    for (int i = 0; i < XYZ; i++) {
        // +
        Vector3d rpy_p = rpy;
        rpy_p(i) += delta_rpy;
        const Vector3d rpy_p_d = RpyFromGyro(rpy, MatrixFromRpy(rpy_p) * (omega - omega_bias_), dt);
        // -
        Vector3d rpy_m = rpy;
        rpy_m(i) -= delta_rpy;
        const Vector3d rpy_m_d = RpyFromGyro(rpy, MatrixFromRpy(rpy_m) * (omega - omega_bias_), dt);

        const Vector3d d_rpy = (rpy_p_d - rpy_m_d) / (2 * delta_rpy);
        for (int j = 0; j < XYZ; j++) {
            jacobian(j, i) = d_rpy(j);
        }
    }
    return jacobian;
}

void ImuEstimator::Calibration(const Vector3d& rpy_observe) {
    // prepare for updating rpy observe
    const Vector3d rpy_observe_mean_next = (rpy_observe_mean_ * counter_ + rpy_observe) / (counter_ + 1);
    const Vector3d rpy_observe_mean_2 = Vector3d(rpy_observe_mean_(X) * rpy_observe_mean_(X), 
                                                    rpy_observe_mean_(Y) * rpy_observe_mean_(Y), 
                                                    rpy_observe_mean_(Z) * rpy_observe_mean_(Z));
    const Vector3d rpy_observe_mean_next_2 = Vector3d(rpy_observe_mean_next(X) * rpy_observe_mean_next(X), 
                                                        rpy_observe_mean_next(Y) * rpy_observe_mean_next(Y), 
                                                        rpy_observe_mean_next(Z) * rpy_observe_mean_next(Z));
    const Vector3d rpy_observe_2 = Vector3d(rpy_observe(X) * rpy_observe(X), 
                                            rpy_observe(Y) * rpy_observe(Y), 
                                            rpy_observe(Z) * rpy_observe(Z));
    const Vector3d rpy_observe_var_next = ((rpy_observe_var_ + rpy_observe_mean_2) * counter_ + rpy_observe_2) / (counter_ + 1) - rpy_observe_mean_next_2;
    // update rpy observe -> R
    rpy_observe_mean_ = rpy_observe_mean_next;
    rpy_observe_var_ = rpy_observe_var_next;
    
    counter_++;
    if (counter_ > NUM_CALIBRATION) {
        // reset
        calibration_ = false;
        counter_ = 0;
        // set variance
        double observe_var = 0.0;
        for (int i = 0; i < XYZ; i++) {
            observe_var += rpy_observe_var_(i);
        }
        observe_var /= XYZ;
        Matrix3d Q = Identity3d() * DEG_TO_RAD * DEG_TO_RAD; // sigma = 1 deg^2
        Matrix3d R = Identity3d();
        for (int i = 0; i < XYZ; i++) {
            R(i, i) = observe_var;
        }
        estimator_.SetVariance(Q, R);
    }
}
