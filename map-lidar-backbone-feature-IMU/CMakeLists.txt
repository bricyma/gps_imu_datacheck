cmake_minimum_required(VERSION 2.8.3)
project(map_lidar_backbone)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

### Eigen3
find_package(Eigen3 REQUIRED)
### PCL 
find_package(PCL 1.7 REQUIRED)
### OpenCV
find_package(OpenCV REQUIRED)
### CERES
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
#MESSAGE("ceres found ${Ceres_FOUND} at ${Ceres_DIR}")
### GFLAGS
FIND_PACKAGE(gflags REQUIRED)
INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})
#MESSAGE("gflags found ${Gflags_FOUND} at ${GFLAGS_INCLUDE_DIRS} with namespace: ${GFLAGS_NAMESPACE}")

include_directories(
  include
	  ${catkin_INCLUDE_DIRS} 
	  ${EIGEN3_INCLUDE_DIR}
	  ${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)

add_library(CeresICP src/CeresICP.cpp)
TARGET_LINK_LIBRARIES(CeresICP ${OpenCV_LIBS} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES} ${PCL_LIBRARIES})

add_executable(scanRegistration src/scanRegistration.cpp)
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserOdometry src/laserOdometry.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserMapping src/laserMapping.cpp)
target_link_libraries(laserMapping CeresICP ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(transformMaintenance src/transformMaintenance.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(PointCloudPublisher src/PointCloudPublisher.cpp)
target_link_libraries(PointCloudPublisher ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} -lmysqlcppconn)

add_executable(laserRobustOdometry src/laserRobustOdometry.cpp)
target_link_libraries(laserRobustOdometry CeresICP ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(resetMapping src/resetMapping.cpp)
target_link_libraries(resetMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(imu_tracker_node src/imu_tracker_node.cpp src/imu_tracker.cpp src/gps_tracker.cpp)
target_link_libraries(imu_tracker_node ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

add_executable(imu_publisher src/imu_publisher.cpp)
target_link_libraries(imu_publisher ${catkin_LIBRARIES})
