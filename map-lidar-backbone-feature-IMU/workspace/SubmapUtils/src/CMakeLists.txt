FIND_PACKAGE( OpenCV 3 REQUIRED )
FIND_PACKAGE( PCL 1.8 REQUIRED )

# OpenCV
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${OpenCV_LIBRARY_DIRS} )

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# G2O
#FIND_PACKAGE( G2O REQUIRED )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )
#INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${CSPARSE_INCLUDE_DIR} )
# Eigen
FIND_PACKAGE( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR})
### CERES
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE("ceres found ${Ceres_FOUND} at ${Ceres_DIR}")
### GFLAGS
FIND_PACKAGE(gflags REQUIRED)
INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})
MESSAGE("gflags found ${Gflags_FOUND} at ${GFLAGS_INCLUDE_DIRS} with namespace: ${GFLAGS_NAMESPACE}")
### Sophus
# use lovegroves version of sophus with ceres extensions!
#ADD_SUBDIRECTORY(ext/sophus-ceres)
#SET(Sophus_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ext/sophus-ceres)
#INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIR})

add_library(CeresICP CeresICP.cpp)
TARGET_LINK_LIBRARIES(CeresICP ${OpenCV_LIBS} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES} ${PCL_LIBRARIES})

add_executable (SubmapUtils main.cpp)
TARGET_LINK_LIBRARIES(SubmapUtils CeresICP
    ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CSPARSE_LIBRARY} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES})
# g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_icp
