#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <memory>
#include <Eigen/Geometry>
#include <math.h>
#include "imu_tracker/imu_tracker.h"
#include "imu_tracker/gps_tracker.h"
# define M_G            9.80665

class ImuSubAndPub {
  public:
    ImuSubAndPub(ros::NodeHandle n) {
      // Assuming 10Hz data flow
      sub_imu_ = n.subscribe<sensor_msgs::Imu>("/imu_data", 20, 
        &ImuSubAndPub::imuHandler, this);
      sub_gps_ = n.subscribe<sensor_msgs::NavSatFix>("/gps_raw", 20,
        &ImuSubAndPub::gpsHandler, this);
      sub_spd_ = n.subscribe<std_msgs::Float64>("/ground_speed", 20,
        &ImuSubAndPub::spdHandler, this);
      pub_ori_ = n.advertise<geometry_msgs::Quaternion>("/imu_ori", 20);
      pub_oridiff_ = n.advertise<std_msgs::Float64>("/ori_diff", 20);
      init_gps_tracker_ = false;
      init_imu_tracker_ = false;
    }
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_msg) {
      double time = imu_msg -> header.stamp.toSec();
      if (!init_imu_tracker_) {
        init_imu_tracker_ = true;
        orientation_ = Eigen::Quaterniond::Identity();
        if (have_init_orientation_) {
          orientation_ = Eigen::Quaterniond(imu_msg -> orientation.w, 
            imu_msg -> orientation.x, imu_msg -> orientation.y, 
            imu_msg -> orientation.z);
        }
        imu_tracker_ = std::unique_ptr<ImuTracker>(new ImuTracker(
          gravity_time_constant, time, orientation_));
        std::cout << "Initialize IMU tracker" << std::endl;
      } else {
        // Accl m/s^2; Gryo rad/s
        Eigen::Vector3d linear_acceleration(imu_msg -> linear_acceleration.x
          / M_G, imu_msg -> linear_acceleration.y / M_G, 
          imu_msg -> linear_acceleration.z / M_G);
        Eigen::Vector3d angular_velocity(imu_msg -> angular_velocity.x, 
          imu_msg -> angular_velocity.y, imu_msg -> angular_velocity.z);
        imu_tracker_ -> Advance(time);
        imu_tracker_ -> AddImuAngularVelocityObservation(angular_velocity);
        imu_tracker_ -> Advance(time);
        imu_tracker_ -> AddImuLinearAccelerationObservation(
          linear_acceleration);
        orientation_ = imu_tracker_ -> orientation_q();    
        oriPublisher();
      }
    }
    void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
      if (!init_gps_tracker_) {
        init_gps_tracker_ = true;
        gps_tracker_ = std::unique_ptr<GPSTracker>(new GPSTracker(
          gps_msg -> latitude, gps_msg -> longitude, alpha_, spd_ct_));
        std::cout << "Initialize GPS tracker" << std::endl;
      } else {
        gps_tracker_ -> AddLatLong(gps_msg -> latitude, gps_msg -> longitude, speed_);
        heading_ = gps_tracker_ -> getHeading();
        oridiffPublisher();
      }
    }
    void spdHandler(const std_msgs::Float64::ConstPtr& spd_msg) {
      speed_ = spd_msg -> data;
    }
    void oriPublisher() {
      geometry_msgs::Quaternion ori_msg;
      ori_msg.x = orientation_.x();
      ori_msg.y = orientation_.y();
      ori_msg.z = orientation_.z();
      ori_msg.w = orientation_.w();
      pub_ori_.publish(ori_msg);
    }
    void oridiffPublisher() {
      std_msgs::Float64 diff_msg;
      diff_msg.data = (imu_tracker_ -> orientation()(2) - heading_) / M_PI * 180;
      pub_oridiff_.publish(diff_msg);
    }
  private:
    // Subscribe to imu data via /imu_data
    ros::Subscriber sub_imu_;
    // Subscribe to gps data via /gps_raw
    ros::Subscriber sub_gps_;
    // Subscribe to speed dat via /ground_speed
    ros::Subscriber sub_spd_;
    // Publish orientation via /imu_ori
    ros::Publisher pub_ori_;
    // Publish heading differences calculated by imu and gps
    ros::Publisher pub_oridiff_;
    bool init_imu_tracker_;
    bool have_init_orientation_ = true;
    bool init_gps_tracker_;
    std::unique_ptr<ImuTracker> imu_tracker_;
    std::unique_ptr<GPSTracker> gps_tracker_;
    Eigen::Quaterniond orientation_;
    double speed_;
    double heading_;
    // Constant used in exponential averaging accl
    const double gravity_time_constant = 1e1;
    // Constant used in gps low pass filter
    const double alpha_ = 0.25;
    const double spd_ct_ = 1.75;
};

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "imu_tracker");
  ros::NodeHandle n;
  ImuSubAndPub imu_sub_imu_pub(n);
  ros::spin();

  return 0;
}