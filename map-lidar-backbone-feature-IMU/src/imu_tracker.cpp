/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "imu_tracker/imu_tracker.h"
#include <cmath>
#include <limits>

ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const double time,
                       const Eigen::Quaterniond orientation)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(time),
      orientation_(orientation),
      gravity_vector_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

// Quaternion to Euler angle
Eigen::Vector3d ImuTracker::toEulerianAngle(
  const Eigen::Quaterniond& q) {
  double ysqr = q.y() * q.y();

  // roll (x-axis rotation)
  double t0 = +2.0f * (q.w() * q.x() + q.y() * q.z());
  double t1 = +1.0f - 2.0f * (q.x() * q.x() + ysqr);
  double roll = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0f * (q.w() * q.y() - q.z() * q.x());
  t2 = t2 > 1.0f ? 1.0f : t2;
  t2 = t2 < -1.0f ? -1.0f : t2;
  double pitch = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0f * (q.w() * q.z() + q.x() *q.y());
  double t4 = +1.0f - 2.0f * (ysqr + q.z() * q.z());  
  double yaw = std::atan2(t3, t4);

  return Eigen::Vector3d(roll, pitch, yaw);
}

// Euler angle to quaternion
Eigen::Quaterniond ImuTracker::toQuaternion(const Eigen::Vector3d& v)
{
  Eigen::Quaterniond q;

  double pitch = v(1), roll = v(0), yaw = v(2);
  double t0 = std::cos(yaw * 0.5f);
  double t1 = std::sin(yaw * 0.5f);
  double t2 = std::cos(roll * 0.5f);
  double t3 = std::sin(roll * 0.5f);
  double t4 = std::cos(pitch * 0.5f);
  double t5 = std::sin(pitch * 0.5f);

  q.w() = t0 * t2 * t4 + t1 * t3 * t5;
  q.x() = t0 * t3 * t4 - t1 * t2 * t5;
  q.y() = t0 * t2 * t5 + t1 * t3 * t4;
  q.z() = t1 * t2 * t4 - t0 * t3 * t5;
  return q;
}

Eigen::Quaternion<double> ImuTracker::AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<double, 3, 1>& angle_axis) {
  double scale = 0.5;
  double w = 1.;
  const double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const double norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<double, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<double>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}

Eigen::Matrix<double, 3, 1> ImuTracker::RotationQuaternionToAngleAxisVector(
    const Eigen::Quaternion<double>& quaternion) {
  Eigen::Quaternion<double> normalized_quaternion = quaternion.normalized();
  // We choose the quaternion with positive 'w', i.e., the one with a smaller
  // angle that represents this orientation.
  if (normalized_quaternion.w() < 0.) {
    // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
    normalized_quaternion.w() *= -1.;
    normalized_quaternion.x() *= -1.;
    normalized_quaternion.y() *= -1.;
    normalized_quaternion.z() *= -1.;
  }
  // We convert the normalized_quaternion into a vector along the rotation axis
  // with length of the rotation angle.
  const double angle = 2. * atan2(normalized_quaternion.vec().norm(),
                                normalized_quaternion.w());
  const double kCutoffAngle = 1e-7;  // We linearize below this angle.
  const double scale = angle < kCutoffAngle ? 2. : angle / sin(angle / 2.);
  return Eigen::Matrix<double, 3, 1>(scale * normalized_quaternion.x(),
                                scale * normalized_quaternion.y(),
                                scale * normalized_quaternion.z());
}

void ImuTracker::Advance(const double time) {
  if (time > time_) {
    const double delta_t = time - time_;
    const Eigen::Quaterniond rotation =
       toQuaternion(Eigen::Vector3d(imu_angular_velocity_ * delta_t));
    orientation_ = (orientation_ * rotation).normalized();
    gravity_vector_ = rotation.inverse() * gravity_vector_;
    time_ = time;
  }
}

void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  const double delta_t = time_ - last_linear_acceleration_time_;
  last_linear_acceleration_time_ = time_;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.inverse() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  //CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  //CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}
