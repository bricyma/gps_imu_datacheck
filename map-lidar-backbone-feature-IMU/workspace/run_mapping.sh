pkill -f roscore
pkill -f rosmaster
pkill -f rosout
pkill -f rosrun
pkill -f roslaunch
pkill -f laserOdometry
pkill -f laserRobustOdometry
pkill -f scanRegistration
pkill -f laserMapping
pkill -f transformMaintenance
pkill -f rviz
sleep 1s

roscore &
sleep 1s
roslaunch map_lidar_backbone robust_submap.launch &


