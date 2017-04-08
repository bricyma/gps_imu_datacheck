#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <Eigen/Geometry>

Eigen::Quaterniond toQuaternion(const Eigen::Vector3d& v)
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

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 20);
  ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("/gps_raw", 20);
  ros::Publisher spd_pub = n.advertise<std_msgs::Float64>("/ground_speed", 20);
  // Publish kitti data for testing
  const std::string file_id = "9";
  std::ifstream infile_imu;
  infile_imu.open("/home/bolunzhang/workspace/ImuTracker/ImuTracker/kitti_test/imu_kitti_000" + file_id + ".txt");
  std::ifstream infile_time;
  infile_time.open("/home/bolunzhang/workspace/ImuTracker/ImuTracker/kitti_test/time_kitti_000" + file_id + ".txt");
  std::ifstream infile_gps;
  infile_gps.open("/home/bolunzhang/workspace/ImuTracker/ImuTracker/kitti_test/gps_kitti_000" + file_id + ".txt");
  std::string line_imu, line_time, line_gps;

  ros::Rate loop_rate(10); // simulate at 10Hz
  ros::Rate pause(2);
  pause.sleep(); // otherwise loss some messages at the beginning
  int count = 0;
  while (ros::ok() && std::getline(infile_imu, line_imu) && 
    std::getline(infile_time, line_time) &&
    std::getline(infile_gps, line_gps)) {
    loop_rate.sleep();
    // Fetch imu data
    std::vector<std::string> tokens_imu;
    std::istringstream iss_imu(line_imu);
    for (std::string s; iss_imu >> s;)
      tokens_imu.push_back(s);  
    // Fetch time stamp
    std::vector<std::string> tokens_time;
    std::istringstream iss_time(line_time);
    for (std::string s; iss_time >> s;)
      tokens_time.push_back(s);
    // Fetch gps data
    std::vector<std::string> tokens_gps;
    std::istringstream iss_gps(line_gps);
    for (std::string s; iss_gps >> s;)
      tokens_gps.push_back(s);

    double time = std::stod(tokens_time[0]) * 60. + std::stod(tokens_time[1]);
    double sec, nsec;
    nsec = modf(time, &sec);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nsec = nsec * 1e9;
    imu_msg.angular_velocity.x = std::stod(tokens_imu[3]);
    imu_msg.angular_velocity.y = std::stod(tokens_imu[4]);
    imu_msg.angular_velocity.z = std::stod(tokens_imu[5]);
    imu_msg.linear_acceleration.x = std::stod(tokens_imu[0]);
    imu_msg.linear_acceleration.y = std::stod(tokens_imu[1]);
    imu_msg.linear_acceleration.z = std::stod(tokens_imu[2]);
    Eigen::Vector3d v(std::stod(tokens_imu[6]), std::stod(tokens_imu[7]), std::stod(tokens_imu[8]));
    Eigen::Quaterniond q = toQuaternion(v);
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    imu_pub.publish(imu_msg);

    std_msgs::Float64 spd_msg;
    spd_msg.data = 10.0;
    spd_pub.publish(spd_msg);

    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.stamp.sec = sec;
    gps_msg.header.stamp.nsec = nsec * 1e9;
    gps_msg.latitude = std::stod(tokens_gps[0]);
    gps_msg.longitude = std::stod(tokens_gps[1]);
    gps_pub.publish(gps_msg);

    ros::spinOnce();
    count++;
    //if (count == 10) break;
  }
  return 0;
}