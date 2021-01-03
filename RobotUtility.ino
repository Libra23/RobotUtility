#include "Kinematic.hpp"
#include "ImuEstimator.hpp"

#ifdef USE_BLA
Kinematic kinematic_;
ImuEstimator imu_estimator_;

void setup() {
  // put your setup code here, to run once:
  KinematicModel model;    
  model.xyz = {{
              {{27.5, 47.63, 0.0}},   // Yaw
              {{33.25, 0.0, 0.0}},    // Pitch1
              {{60.0, 0.0, 0.0}},     // Pitch2
              {{120.0, 0.0, 0.0}},    // Tip
              }};
  model.axis = {{
                {{0.0, 0.0, 1.0}},      // Yaw
                {{0.0, 1.0, 0.0}},      // Pitch1
                {{0.0, 1.0, 0.0}},      // Pitch2
                {{0.0, 0.0, 0.0}},      // Tip
                }};
  model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
  kinematic_.Config(model);
  kinematic_.SetConstant(1500.0, 0.01);   // set w & k

  // check FK
  //std::cout << " check FK " << std::endl;
  Joint q;
  q << 45.0, 0.0, 45.0;
  q *= DEG_TO_RAD;
  Affine3d tip_trans;
  kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
  //std::cout << std::fixed << std::setprecision(3) << q.transpose() * RAD_TO_DEG << std::endl;
  //std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
  
  // check Ik
  //std::cout << " check IK " << std::endl;
  const Joint q_pre = q;
  const Affine3d tip_trans_pre = tip_trans;
  q = q_pre;
  tip_trans.translation() = tip_trans_pre.translation() + Vector3d(0.0, 0.0, 0.002);
  bool ik_ret = kinematic_.Inverse(tip_trans, Affine3d::Identity(), q_pre, q);
  //std::cout << " result " << ik_ret << std::endl;
  //std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
  //std::cout << std::fixed << std::setprecision(5) << q.transpose() * RAD_TO_DEG << std::endl;
  kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
  //std::cout << " compare IK & FK " << ik_ret << std::endl;
  //std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
  
  //std::cout << " check imu " << std::endl;
  
  const double pitch = 10 * DEG_TO_RAD;
  const double dt = 1e-3;
  const Vector3d alpha = Vector3d(sin(pitch), 0.0, -cos(pitch));
  const Vector3d omega = Vector3d(0.0, pitch / dt, 0.0);
  const Vector3d rpy = imu_estimator_.UpdateRpy(alpha, omega, dt);
  //std::cout << std::fixed << std::setprecision(3) << rpy.transpose() * RAD_TO_DEG << std::endl;
}

void loop() {
  // put your main code here, to run repeatedly:

}

#else
#include <iostream>
#include <iomanip>
int main() {
    Kinematic kinematic_;
    KinematicModel model;
    model.xyz = {{
                {{27.5, 47.63, 0.0}},   // Yaw
                {{33.25, 0.0, 0.0}},    // Pitch1
                {{60.0, 0.0, 0.0}},     // Pitch2
                {{120.0, 0.0, 0.0}},    // Tip
                }};
    model.axis = {{
                 {{0.0, 0.0, 1.0}},      // Yaw
                 {{0.0, 1.0, 0.0}},      // Pitch1
                 {{0.0, 1.0, 0.0}},      // Pitch2
                 {{0.0, 0.0, 0.0}},      // Tip
                 }};
    model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    kinematic_.Config(model);
    kinematic_.SetConstant(1500.0, 0.01);   // set w & k
    
    // check FK
    std::cout << " check FK " << std::endl;
    Joint q;
    q << 45.0, 0.0, 45.0;
    q *= DEG_TO_RAD;
    Affine3d tip_trans;
    kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
    std::cout << std::fixed << std::setprecision(3) << q.transpose() * RAD_TO_DEG << std::endl;
    std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
    
    // check Ik
    std::cout << " check IK " << std::endl;
    const Joint q_pre = q;
    const Affine3d tip_trans_pre = tip_trans;
    q = q_pre;
    tip_trans.translation() = tip_trans_pre.translation() + Vector3d(0.0, 0.0, 0.002);
    bool ik_ret = kinematic_.Inverse(tip_trans, Affine3d::Identity(), q_pre, q);
    std::cout << " result " << ik_ret << std::endl;
    std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
    std::cout << std::fixed << std::setprecision(5) << q.transpose() * RAD_TO_DEG << std::endl;
    kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
    std::cout << " compare IK & FK " << ik_ret << std::endl;
    std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
    
    std::cout << " check imu " << std::endl;
    ImuEstimator imu_estimator_;
    const double pitch = 10 * DEG_TO_RAD;
    const double dt = 1e-3;
    const Vector3d alpha = Vector3d(sin(pitch), 0.0, -cos(pitch));
    const Vector3d omega = Vector3d(0.0, pitch / dt, 0.0);
    const Vector3d rpy = imu_estimator_.UpdateRpy(alpha, omega, dt);
    std::cout << std::fixed << std::setprecision(3) << rpy.transpose() * RAD_TO_DEG << std::endl;
    
    return 0;
}

#endif
