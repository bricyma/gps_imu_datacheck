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

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include <Eigen/Geometry>

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, double time,
    Eigen::Quaterniond orientation);

  // Transform anglesAxisVector to quaternion (not used here)
  Eigen::Quaternion<double> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<double, 3, 1>& angle_axis);

  // Transform quaternion to rotation angles (not used here)
  Eigen::Matrix<double, 3, 1> RotationQuaternionToAngleAxisVector(
      const Eigen::Quaternion<double>& quaternion);

  // Transform quaternion to euler angle
  static Eigen::Vector3d toEulerianAngle(const Eigen::Quaterniond& q);

  // Transform euler angle to quaternion
  static Eigen::Quaterniond toQuaternion(const Eigen::Vector3d& v);

  // Advances to the given 'time' and updates the orientation to reflect this.
  void Advance(double time);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current orientation estimation.
  Eigen::Vector3d orientation() { return toEulerianAngle(orientation_); }
  // Query the current orientation estimation.
  Eigen::Quaterniond orientation_q() { return orientation_;};
  // Query the current gravity vector for debug.
  Eigen::Vector3d gravity() { return gravity_vector_;}

 private:
  const double imu_gravity_time_constant_;
  double time_;
  double last_linear_acceleration_time_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d gravity_vector_;
  Eigen::Vector3d imu_angular_velocity_;
};

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
