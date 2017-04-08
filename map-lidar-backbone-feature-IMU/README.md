#map-lidar-backbone

Add Imu tracker feature. Use IMU angular velocity and linear acceleration to estimate orientation using a complementary filter. Compare this orientation with that computed from gps trajectory using a low pass filter.

## ROS node

### imu_tracker_node
Subscribe imu data  
topic: /imu_data     type: sensor_msgs/Imu  
Subscribe gps data  
topic: /gps_raw      type: sensor_msgs/NavSatFix  
Subscribe vel data  
topic: /ground_speed type: std_msgs/Float64  

Publish orientation estimation  
topic: /imu_ori      type: geometry_msgs/Quaternion  
Publish heading difference calculated from imu and gps (degree)
topic: /ori_diff     type: std_msgs/Float64

### imu_publisher
Publish kitti imu gps data for testing
