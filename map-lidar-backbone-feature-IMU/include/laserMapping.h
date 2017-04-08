/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   laserMapping.h
 * Author: yiluo
 *
 * Created on December 15, 2016, 3:14 PM
 */

#ifndef LASERMAPPING_H
#define LASERMAPPING_H

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <math.h>

#include <common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "../include/CeresICP.h"

// Path
static std::string submapPath = "/home/yiluo/LIDAR/result/submap/";
static std::string trajectoryPath = "/home/yiluo/LIDAR/result/trajectory/";

static bool systemInited = false;
static double startFrameTimestamp = -1.0;

static const float scanPeriod = 0.1;

static const int stackFrameNum = 1;
static const int mapFrameNum = 5;

static double timeLaserCloudCornerLast = 0;
static double timeLaserCloudSurfLast = 0;
static double timeLaserCloudFullRes = 0;
static double timeLaserOdometry = 0;

static bool newLaserCloudCornerLast = false;
static bool newLaserCloudSurfLast = false;
static bool newLaserCloudFullRes = false;
static bool newLaserOdometry = false;

static int laserCloudCenWidth = 10;
static int laserCloudCenHeight = 5;
static int laserCloudCenDepth = 10;
static const int laserCloudWidth = 21;
static const int laserCloudHeight = 11;
static const int laserCloudDepth = 21;
static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

static int laserCloudValidInd[125];
static int laserCloudSurroundInd[125];

// For GPS
static int stationaryCounter = 0;
static const int stationaryCounterThreshold = 5; // if stationaryCounter > threshold, the odometry is static or in failure
static double startLat = 39.9080145000;
static double startLon = 116.5321005000;
static double er = 6378137.0; //6378137.0
static double pi = 3.14159265359;
static double scale = cos(startLat * pi / 180.);
static double fixedGPS_X = scale * startLon * pi * er / 180.0;
static double fixedGPS_Y = scale * er * log(tan((90.0 + startLat) * pi / 360.0));
static double startGPS_X = -1.0; // Relative to GPS fixed position
static double startGPS_Y = -1.0;
static double curGPS_X = -1.0; // Relative to GPS fixed position
static double curGPS_Y = -1.0;
static bool curGPSPosUpated = false;
static bool startGPSPosInited = false;
static const double GPSMotionThreshold = 20.0;
static const double GPSDifThreshold = 0.05;
static int frameNumCounter = 0;
static double speedPrior = 3.0;

// Robust Fitting
static float huberWidth = 3.0;

// PointCloud
static pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurround2(new pcl::PointCloud<PointType>()); // More dense than laserCloudSurround
static pcl::PointCloud<PointType>::Ptr laserCloudSurroundWhole(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurroundWholeFiltered(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudFullMap(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
static pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
static pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];
static pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];
static pcl::PointCloud<PointType>::Ptr resetSignalPointCloud(new pcl::PointCloud<PointType>()); // To reset the system

static pcl::PointCloud<PointType>::Ptr laserCloudCornerStackMapped(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurfStackMapped(new pcl::PointCloud<PointType>());

static pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
static pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

// Transformation
static float transformSum[6] = {0}; // from cur odometry frame to initial
static float transformIncre[6] = {0};
static float transformTobeMapped[6] = {0}; // from cur undistorted laser frame to map frame
static float transformTobeMappedLast[6] = {0}; // from last undistorted laser frame to map frame
static float transformTobeMappedBeforeLOAM[6] = {0}; // store the transformTobeMapped before LOAM mapping
static float transformFromLastToCur[6] = {0}; // from last mapped laser frame to cur mapped laser frame
static float transformFromLastToCurIncre[6] = {0}; // from last mapped laser frame to cur mapped laser frame
static float transformBefMapped[6] = {0}; // from last odometry frame to initial
static float transformAftMapped[6] = {0}; // transformTobeMapped fused with IMU
static float transformAccumulated[6] = {0}; // accumulate motion from the start GPS
static float transformFromSecondLastToLast[6] = {0}; // from second last mapped laser frame to last mapped laser frame
static float transformTobeMappedDif[6] = {0};

// For Failure Check
static bool transformFromSecondLastToLastSkip = true;

static int imuPointerFront = 0;
static int imuPointerLast = -1;
static const int imuQueLength = 200;

static double imuTime[imuQueLength] = {0};
static float imuRoll[imuQueLength] = {0};
static float imuPitch[imuQueLength] = {0};

// For Reset System
static ros::Publisher pubResetSignal;
static ros::Publisher pubLaserCloudSubmap;

// For Record Trajectory
static std::ofstream trajectoryFileOut;

// Functions

void GetEulerAnglesAndTranslation(Eigen::Affine3f& Transformation, float &rx, float &ry, float &rz, float &tx, float &ty, float &tz) {
    rx = asin(Transformation(2, 1));
    ry = atan(-Transformation(2, 0) / Transformation(2, 2));
    rz = atan(-Transformation(0, 1) / Transformation(1, 1));
    tx = Transformation(0, 3);
    ty = Transformation(1, 3);
    tz = Transformation(2, 3);
}

void GetTransformationFromEulerAnglesAndTranslation(Eigen::Affine3f& Transformation, float rx, float ry, float rz, float tx, float ty, float tz) {
    float crx = cos(rx);
    float cry = cos(ry);
    float crz = cos(rz);
    float srx = sin(rx);
    float sry = sin(ry);
    float srz = sin(rz);
    // Set rotation
    Transformation(0, 0) = cry * crz - srx * sry * srz;
    Transformation(1, 0) = cry * srz + srx * sry * crz;
    Transformation(2, 0) = -crx*sry;

    Transformation(0, 1) = -crx*srz;
    Transformation(1, 1) = crx*crz;
    Transformation(2, 1) = srx;

    Transformation(0, 2) = sry * crz + srx * cry * srz;
    Transformation(1, 2) = sry * srz - srx * cry * crz;
    Transformation(2, 2) = crx*cry;

    // Set translation
    Transformation(0, 3) = tx;
    Transformation(1, 3) = ty;
    Transformation(2, 3) = tz;
}

void transformAssociateToMap() {
    float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
            - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    float y1 = transformBefMapped[4] - transformSum[4];
    float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
            + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

    float x2 = x1;
    float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
    float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

    transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
    transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
    transformIncre[5] = z2;

    float sbcx = sin(transformSum[0]);
    float cbcx = cos(transformSum[0]);
    float sbcy = sin(transformSum[1]);
    float cbcy = cos(transformSum[1]);
    float sbcz = sin(transformSum[2]);
    float cbcz = cos(transformSum[2]);

    float sblx = sin(transformBefMapped[0]);
    float cblx = cos(transformBefMapped[0]);
    float sbly = sin(transformBefMapped[1]);
    float cbly = cos(transformBefMapped[1]);
    float sblz = sin(transformBefMapped[2]);
    float cblz = cos(transformBefMapped[2]);

    float salx = sin(transformAftMapped[0]);
    float calx = cos(transformAftMapped[0]);
    float saly = sin(transformAftMapped[1]);
    float caly = cos(transformAftMapped[1]);
    float salz = sin(transformAftMapped[2]);
    float calz = cos(transformAftMapped[2]);

    float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz)
            - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly)
            - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly)
            - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz)
            - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
    transformTobeMapped[0] = -asin(srx);

    float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly)
            - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx)
            - cbcx * cbcy * ((caly * calz + salx * saly * salz)*(cblz * sbly - cbly * sblx * sblz)
            + (caly * salz - calz * salx * saly)*(sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly)
            + cbcx * sbcy * ((caly * calz + salx * saly * salz)*(cbly * cblz + sblx * sbly * sblz)
            + (caly * salz - calz * salx * saly)*(cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
    float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz)
            - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx)
            + cbcx * cbcy * ((saly * salz + caly * calz * salx)*(sbly * sblz + cbly * cblz * sblx)
            + (calz * saly - caly * salx * salz)*(cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly)
            - cbcx * sbcy * ((saly * salz + caly * calz * salx)*(cbly * sblz - cblz * sblx * sbly)
            + (calz * saly - caly * salx * salz)*(cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
    transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
            crycrx / cos(transformTobeMapped[0]));

    float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz)*(calx * salz * (cblz * sbly - cbly * sblx * sblz)
            - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx)
            - (cbcy * cbcz + sbcx * sbcy * sbcz)*(calx * calz * (cbly * sblz - cblz * sblx * sbly)
            - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly)
            + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
    float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy)*(calx * calz * (cbly * sblz - cblz * sblx * sbly)
            - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly)
            - (sbcy * sbcz + cbcy * cbcz * sbcx)*(calx * salz * (cblz * sbly - cbly * sblx * sblz)
            - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx)
            + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
    transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
            crzcrx / cos(transformTobeMapped[0]));

    x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
    y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
    z1 = transformIncre[5];

    x2 = x1;
    y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    transformTobeMapped[3] = transformAftMapped[3]
            - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
    transformTobeMapped[4] = transformAftMapped[4] - y2;
    transformTobeMapped[5] = transformAftMapped[5]
            - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void transformUpdate() {
    if (imuPointerLast >= 0) {
        float imuRollLast = 0, imuPitchLast = 0;
        while (imuPointerFront != imuPointerLast) {
            if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
                break;
            }
            imuPointerFront = (imuPointerFront + 1) % imuQueLength;
        }

        if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
            imuRollLast = imuRoll[imuPointerFront];
            imuPitchLast = imuPitch[imuPointerFront];
        } else {
            int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
            float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack])
                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod)
                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

            imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
            imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
        }

        //        transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
        //        transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
    }

    for (int i = 0; i < 6; i++) {
        transformBefMapped[i] = transformSum[i]; // cur odometry frame to initial pose
        transformAftMapped[i] = transformTobeMapped[i]; // new scan to map
    }
}

void pointAssociateToMap(PointType const * const pi, PointType * const po) {
    float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
    float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
            + transformTobeMapped[3];
    po->y = y2 + transformTobeMapped[4];
    po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
            + transformTobeMapped[5];
    po->intensity = pi->intensity;
}

// Order Y -> X -> Z

void pointAssociateTobeMapped(PointType const * const pi, PointType * const po) {
    float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
            - sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
    float y1 = pi->y - transformTobeMapped[4];
    float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
            + cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);

    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
    float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    po->x = cos(transformTobeMapped[2]) * x2
            + sin(transformTobeMapped[2]) * y2;
    po->y = -sin(transformTobeMapped[2]) * x2
            + cos(transformTobeMapped[2]) * y2;
    po->z = z2;
    po->intensity = pi->intensity;
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2) {
    timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

    laserCloudCornerLast->clear();
    pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

    newLaserCloudCornerLast = true;
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2) {
    timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

    laserCloudSurfLast->clear();
    pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

    newLaserCloudSurfLast = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) {
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

    laserCloudFullRes->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

    newLaserCloudFullRes = true;
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) {
    timeLaserOdometry = laserOdometry->header.stamp.toSec();

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    transformSum[0] = -pitch;
    transformSum[1] = -yaw;
    transformSum[2] = roll;

    transformSum[3] = laserOdometry->pose.pose.position.x;
    transformSum[4] = laserOdometry->pose.pose.position.y;
    transformSum[5] = laserOdometry->pose.pose.position.z;

    newLaserOdometry = true;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn) {
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    imuPointerLast = (imuPointerLast + 1) % imuQueLength;

    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
}

void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gpsIn) {
    double lat = gpsIn->latitude;
    double lon = gpsIn->longitude;
    if (!startGPSPosInited) {
        // Calculate startGPS_X and startGPS_Y
        startGPS_X = scale * lon * pi * er / 180.0 - fixedGPS_X;
        startGPS_Y = scale * er * log(tan((90.0 + lat) * pi / 360.0)) - fixedGPS_Y;
        // Set Flag
        startGPSPosInited = true;
    } else {
        // Calculate curGPS_X and curGPS_Y
        curGPS_X = scale * lon * pi * er / 180.0 - fixedGPS_X;
        curGPS_Y = scale * er * log(tan((90.0 + lat) * pi / 360.0)) - fixedGPS_Y;
        // Set Flag
        curGPSPosUpated = true;
    }
    //    std::cout << "Get GPS! startGPS_X: " << startGPS_X << ", startGPS_Y" << startGPS_Y << ", startGPS_Y" << startGPS_Y << ", startGPS_Y" << curGPS_X << ", startGPS_Y" << curGPS_Y << "\n";
}

void speedHandler(const sensor_msgs::NavSatFix::ConstPtr& speedIn) {
    speedPrior = speedIn->latitude;
}

bool failureCheck() {
    bool isFail = false;

    // Rotation
    for (int i = 0; i < 3; i++) {
        if (abs(transformFromLastToCur[i]) > 0.03) {
            std::cout << "Reset by 1\n";
            trajectoryFileOut << "Reset by 1\n";
            isFail = true;
        }
    }

    // Translation in non-forward direction
    if (abs(transformFromLastToCur[4]) > 1.0) {
        isFail = true;
        std::cout << "Reset by 2\n";
        trajectoryFileOut << "Reset by 2\n";
    }
    if (abs(transformFromLastToCur[5]) > 0.2) {
        isFail = true;
        std::cout << "Reset by 3\n";
        trajectoryFileOut << "Reset by 3\n";
    }
    // Translation in forward direction
    if (abs(transformFromLastToCur[3]) > 4.0) {
        isFail = true;
        std::cout << "Reset by 4\n";
        trajectoryFileOut << "Reset by 4\n";
    }
    // Check if constant velocity model valid
    if (transformFromSecondLastToLastSkip) {
        transformFromSecondLastToLastSkip = false;
    } else {
        for (int i = 0; i < 6; i++) {
            if (abs(transformFromSecondLastToLast[i] - transformFromLastToCur[i]) > 1.0) {
                if (i != 3) // forward direction
                {
                    isFail = true;
                    std::cout << "Reset by 5\n";
                    trajectoryFileOut << "Reset by 5\n";
                }
            }
        }
    }
    // GPS position: 1.stationary 2. odometry mismatch
    // check stationary
    bool isStationary = true;
    for (int i = 0; i < 6; i++) {
        if (fabs(transformFromLastToCur[i]) > 0.2)
            isStationary = false;
    }

    if ((frameNumCounter % 10) == 0) {
        if (!isStationary) stationaryCounter = 0;

        if (startGPSPosInited && curGPSPosUpated) {
            double GPSMotion_X = curGPS_X - startGPS_X;
            double GPSMotion_Y = curGPS_Y - startGPS_Y;
            curGPSPosUpated = false; // mark the GPS Pos is used

            if (isStationary) {
                stationaryCounter++;
                if (stationaryCounter > stationaryCounterThreshold) {
                    // Check GPS to confirm failure
                    if ((GPSMotion_X * GPSMotion_X + GPSMotion_Y * GPSMotion_Y) > GPSMotionThreshold * GPSMotionThreshold) {
                        isFail = true;
                        std::cout << "Reset by 6\n";
                        trajectoryFileOut << "Reset by 6\n";
                    }
                }
            }

            // check odometry mismatch
            double transformAccumulated_X = (double) transformAftMapped[3];
            double transformAccumulated_Y = (double) transformAftMapped[4];
            double transformAccumulated_Z = (double) transformAftMapped[5];
            double squareOdom = transformAccumulated_X * transformAccumulated_X + transformAccumulated_Y * transformAccumulated_Y +
                    transformAccumulated_Z * transformAccumulated_Z;
            double OdomLen = sqrt(squareOdom);
            double squareGPS = GPSMotion_X * GPSMotion_X + GPSMotion_Y * GPSMotion_Y;
            double GPSLen = sqrt(squareGPS);
            double DifOdomGPS = OdomLen - GPSLen;
            double DifOdomGPSRatio = 0;
            if (GPSLen > 100)
                DifOdomGPSRatio = fabs(DifOdomGPS) / GPSLen;
            if (DifOdomGPSRatio > GPSDifThreshold) {
                isFail = true;
                std::cout << "Reset by 7, GPSLen: " << GPSLen << ", OdomLen: " << OdomLen << ", DifOdomGPSRatio: " << DifOdomGPSRatio << "\n";
                trajectoryFileOut << "Reset by 7, GPSLen: " << GPSLen << ", OdomLen: " << OdomLen << ", DifOdomGPSRatio: " << DifOdomGPSRatio << "\n";
            }

            // check GPS len
            if (GPSLen > 10000) {
                isFail = true;
                std::cout << "Reset by 8, GPSLen: " << GPSLen << ", OdomLen: " << OdomLen << "\n";
                trajectoryFileOut << "Reset by 8, GPSLen: " << GPSLen << ", OdomLen: " << OdomLen << "\n";
            }
        } else {
            std::cout << "GPS Pos not ready!\n";
        }
    } else {
        if (!isStationary) stationaryCounter = 0;
    }

    frameNumCounter++;

    // Check time, restart system after a few minute?
    //    if (abs(timeLaserOdometry - startFrameTimestamp) > 1.0)
    //        isFail = true;

    return isFail;
}

void resetMappingSystem() {
    // Reset system variable
    systemInited = false;

    // Send signal
    resetSignalPointCloud->clear();
    PointType resetPoint;
    resetPoint.x = -1.0;
    resetPoint.y = -1.0;
    resetPoint.z = -1.0;

    resetSignalPointCloud->push_back(resetPoint);
    sensor_msgs::PointCloud2 resetSignalPointCloudMsg;
    pcl::toROSMsg(*resetSignalPointCloud, resetSignalPointCloudMsg);
    resetSignalPointCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    resetSignalPointCloudMsg.header.frame_id = "/camera_init";
    pubResetSignal.publish(resetSignalPointCloudMsg);

    // Pub Submap
    // This might take a while
    //    sensor_msgs::PointCloud2 submapPointCloudMsg;
    //    //    pcl::toROSMsg(*laserCloudSurroundWhole, submapPointCloudMsg); // Not Full Res
    //    pcl::toROSMsg(*laserCloudFullMap, submapPointCloudMsg); // Full Res laserCloudFullMap
    //    std::cout << "Publishing submap with " << std::setprecision(12) << startFrameTimestamp << " time and " << laserCloudFullMap->points.size() << " points.\n";
    //    submapPointCloudMsg.header.stamp = ros::Time().fromSec(startFrameTimestamp);
    //    submapPointCloudMsg.header.frame_id = "/camera_init";
    //    pubLaserCloudSubmap.publish(submapPointCloudMsg);

    std::cout << "Writing, Submap has " << laserCloudFullMap->width << " point before filtering...\n";
    // Write onto disk
    std::string frameName;
    std::stringstream StrStm;
    StrStm << std::setprecision(12) << timeLaserOdometry;
    StrStm >> frameName;
    std::string outputFileName = submapPath + frameName + ".pcd";
    //    if ((int) laserCloudSubmapFiltered->points.size() > 0)
    if (laserCloudFullMap->width > 0) {
        pcl::io::savePCDFileASCII(outputFileName.c_str(), *laserCloudFullMap);
    } else {
        std::cout << "Submap has no point...\n";
    }

    // Reset transformation
    for (int i = 0; i < 6; i++) {
        transformSum[i] = 0;
        transformIncre[i] = 0;
        transformTobeMapped[i] = 0;
        transformTobeMappedLast[i] = 0;
        transformFromLastToCur[i] = 0;
        transformBefMapped[i] = 0;
        transformAftMapped[i] = 0;
        transformAccumulated[i] = 0;
        transformFromSecondLastToLast[i] = 0.0;
        transformFromLastToCurIncre[i] = 0.0;
    }
    transformFromSecondLastToLastSkip = true;

    // Reset pointcloud
    laserCloudCornerLast->clear();
    laserCloudSurfLast->clear();
    laserCloudCornerStack->clear();
    laserCloudSurfStack->clear();
    laserCloudCornerStack2->clear();
    laserCloudSurfStack2->clear();
    laserCloudOri->clear();
    coeffSel->clear();
    laserCloudSurround->clear();
    laserCloudSurround2->clear();
    laserCloudSurroundWhole->clear();
    laserCloudSurroundWholeFiltered->clear();
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    laserCloudFullRes->clear();
    laserCloudFullMap->clear();
    for (int i = 0; i < laserCloudNum; i++) {
        laserCloudCornerArray[i]->clear();
        laserCloudSurfArray[i]->clear();
        laserCloudCornerArray2[i]->clear();
        laserCloudSurfArray2[i]->clear();
    }

    // Reset Time
    startFrameTimestamp = -1.0;

    timeLaserCloudCornerLast = 0;
    timeLaserCloudSurfLast = 0;
    timeLaserCloudFullRes = 0;
    timeLaserOdometry = 0;
    newLaserCloudCornerLast = false;
    newLaserCloudSurfLast = false;
    newLaserCloudFullRes = false;
    newLaserOdometry = false;

    // Reset GPS
    stationaryCounter = 0;
    frameNumCounter = 0;
    curGPS_X = -1.0;
    curGPS_Y = -1.0;
    startGPS_X = -1.0;
    startGPS_Y = -1.0;
    startGPSPosInited = false;
    curGPSPosUpated = false;
    speedPrior = 3.0;

    // Rest IMU, To Do

    // Close the trajectory output
    trajectoryFileOut.close();
}

void resetMappingSignalHandler(const sensor_msgs::PointCloud2ConstPtr& submapPointCloudMsg) {
    resetMappingSystem();
}

void startMappingSystem() {
    std::cout << "Reset Mapping System!!!\n";
    // Set system variable
    systemInited = true;
    startFrameTimestamp = timeLaserOdometry;
    std::string frameName;
    std::stringstream StrStm;
    StrStm << std::setprecision(12) << startFrameTimestamp;
    StrStm >> frameName;
    std::string outputFileName = trajectoryPath + frameName + ".txt";
    trajectoryFileOut.open(outputFileName.c_str());
}

void useMotionPrior() {
    // Use GPS Info
    //    transformTobeMapped[0] = transformTobeMappedLast[0];
    //    transformTobeMapped[1] = transformTobeMappedLast[1];
    //    transformTobeMapped[2] = transformTobeMappedLast[2];
    //    transformTobeMapped[3] = transformTobeMappedLast[3] - speedPrior;
    //    transformTobeMapped[4] = transformTobeMappedLast[4];
    //    transformTobeMapped[5] = transformTobeMappedLast[5];
    transformTobeMapped[0] = transformTobeMappedLast[0] + transformFromLastToCurIncre[0];
    transformTobeMapped[1] = transformTobeMappedLast[1] + transformFromLastToCurIncre[1];
    transformTobeMapped[2] = transformTobeMappedLast[2] + transformFromLastToCurIncre[2];
    transformTobeMapped[3] = transformTobeMappedLast[3] - speedPrior;
    transformTobeMapped[4] = transformTobeMappedLast[4] + transformFromLastToCurIncre[4];
    transformTobeMapped[5] = transformTobeMappedLast[5] + transformFromLastToCurIncre[5];
}

void checkMotionEstimation() {
    Eigen::Affine3f fromLastToWorld = Eigen::Affine3f::Identity();
    Eigen::Affine3f fromCurToWorld = Eigen::Affine3f::Identity();
    Eigen::Affine3f fromLastToCur = Eigen::Affine3f::Identity();
    GetTransformationFromEulerAnglesAndTranslation(fromLastToWorld,
            transformTobeMappedLast[0], transformTobeMappedLast[1], transformTobeMappedLast[2],
            transformTobeMappedLast[3], transformTobeMappedLast[4], transformTobeMappedLast[5]);
    GetTransformationFromEulerAnglesAndTranslation(fromCurToWorld,
            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2],
            transformTobeMapped[3], transformTobeMappedLast[4], transformTobeMapped[5]);
    fromLastToCur = fromCurToWorld.inverse() * fromLastToWorld;
    GetEulerAnglesAndTranslation(fromLastToCur,
            transformFromLastToCur[0], transformFromLastToCur[1], transformFromLastToCur[2],
            transformFromLastToCur[3], transformFromLastToCur[4], transformFromLastToCur[5]);

    //    std::cout << "Checking transformFromLastToCur: ";
    //    for (int i = 0; i < 6; i++)
    //        std::cout << transformFromLastToCur[i] << " ";
    //    std::cout << "\n";
    //
    //    std::cout << "Checking transformFromLastToCurIncre: ";
    //    for (int i = 0; i < 6; i++)
    //        std::cout << transformFromLastToCurIncre[i] << " ";
    //    std::cout << "\n";

    //    std::cout << "Checking transformTobeMapped: ";
    //    for (int i = 0; i < 6; i++)
    //        std::cout << transformTobeMapped[i] << " ";
    //    std::cout << "\n";
    //
    //    std::cout << "Checking transformTobeMappedLast: ";
    //    for (int i = 0; i < 6; i++)
    //        std::cout << transformTobeMappedLast[i] << " ";
    //    std::cout << "\n";
    //
    //    std::cout << "Checking transformTobeMapped - transformTobeMappedLast: ";
    //    for (int i = 0; i < 6; i++)
    //        std::cout << fabs(transformTobeMapped[i] - transformTobeMappedLast[i]) << " ";
    //    std::cout << "\n";
    //    std::cout << "Checking transformFromLastToCur: ";
    //    for (int i = 0; i < 6; i++)
    //        std::cout << transformFromLastToCur[i] << " ";
    //    std::cout << "\n";
    //
    //    std::cout << "Checking transformTobeMappedDif: ";
    //    for (int i = 0; i < 6; i++)
    //        //        std::cout << transformTobeMappedDif[i] << " ";
    //        std::cout << fabs(transformTobeMappedDif[i] - fabs(transformTobeMapped[i] - transformTobeMappedLast[i])) << " ";
    //    std::cout << "\n";


    double difInMovingDirection = speedPrior * 0.3;
    double SpeedInSecondDirections = speedPrior * 0.333;
    double turningScale = 1.0;
    if (speedPrior < 0.7) {
        difInMovingDirection = 1.0;
        SpeedInSecondDirections = 0.333;
        turningScale = 3.0;
    }

    // Rotation
    for (int i = 0; i < 3; i++) {
        if (abs(transformFromLastToCur[i]) > 0.03) {
            std::cout << "Reset by 1\n";
            trajectoryFileOut << "Reset by 1\n";
            resetMappingSystem();
        }
    }
    // Translation in non-forward direction
    if (abs(transformFromLastToCur[4]) > 1.0) {
        std::cout << "Reset by 2\n";
        trajectoryFileOut << "Reset by 2\n";
        resetMappingSystem();
    }
    if (abs(transformFromLastToCur[5]) > 0.2) {
        std::cout << "Reset by 3\n";
        trajectoryFileOut << "Reset by 3\n";
        resetMappingSystem();
    }
    // Translation in forward direction
    if (abs(transformFromLastToCur[3]) > 4.0) {
        std::cout << "Reset by 4\n";
        trajectoryFileOut << "Reset by 4\n";
        resetMappingSystem();
    }

    // Use GPS info to correct
    if (fabs(transformFromLastToCur[3] - speedPrior) > difInMovingDirection ||
            fabs(transformFromLastToCur[4]) > SpeedInSecondDirections * turningScale ||
            fabs(transformFromLastToCur[5]) > 0.2 ||
            fabs(transformFromLastToCur[0]) > 0.01 * turningScale ||
            fabs(transformFromLastToCur[1]) > 0.01 * turningScale ||
            fabs(transformFromLastToCur[2]) > 0.01 * turningScale)
        //    if(true)
    {
        transformTobeMapped[0] = transformTobeMappedLast[0] + transformFromLastToCurIncre[0];
        transformTobeMapped[1] = transformTobeMappedLast[1] + transformFromLastToCurIncre[1];
        transformTobeMapped[2] = transformTobeMappedLast[2] + transformFromLastToCurIncre[2];
        transformTobeMapped[3] = transformTobeMappedLast[3] - speedPrior;
        transformTobeMapped[4] = transformTobeMappedLast[4] + transformFromLastToCurIncre[4];
        transformTobeMapped[5] = transformTobeMappedLast[5] + transformFromLastToCurIncre[5];

        std::cout << "Use Motion Prior for Final Result in Mapping!\n";
    }

    // Store Result
    transformFromSecondLastToLast[0] = transformFromLastToCur[0];
    transformFromSecondLastToLast[1] = transformFromLastToCur[1];
    transformFromSecondLastToLast[2] = transformFromLastToCur[2];
    transformFromSecondLastToLast[3] = transformFromLastToCur[3];
    transformFromSecondLastToLast[4] = transformFromLastToCur[4];
    transformFromSecondLastToLast[5] = transformFromLastToCur[5];
    transformFromLastToCurIncre[0] = transformTobeMapped[0] - transformTobeMappedLast[0];
    transformFromLastToCurIncre[1] = transformTobeMapped[1] - transformTobeMappedLast[1];
    transformFromLastToCurIncre[2] = transformTobeMapped[2] - transformTobeMappedLast[2];
    transformFromLastToCurIncre[3] = transformTobeMapped[3] - transformTobeMappedLast[3];
    transformFromLastToCurIncre[4] = transformTobeMapped[4] - transformTobeMappedLast[4];
    transformFromLastToCurIncre[5] = transformTobeMapped[5] - transformTobeMappedLast[5];
    transformTobeMappedLast[0] = transformTobeMapped[0];
    transformTobeMappedLast[1] = transformTobeMapped[1];
    transformTobeMappedLast[2] = transformTobeMapped[2];
    transformTobeMappedLast[3] = transformTobeMapped[3];
    transformTobeMappedLast[4] = transformTobeMapped[4];
    transformTobeMappedLast[5] = transformTobeMapped[5];

    std::cout << "\n";
}

#endif /* LASERMAPPING_H */

