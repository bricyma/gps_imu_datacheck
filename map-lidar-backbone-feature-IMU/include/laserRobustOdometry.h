/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   laserRobustOdometry.h
 * Author: yiluo
 *
 * Created on December 15, 2016, 3:46 PM
 */

#ifndef LASERROBUSTODOMETRY_H
#define LASERROBUSTODOMETRY_H

#include <cmath>
#include <string>
#include <iostream>

#include <common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/eigen.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/registration/transformation_validation_euclidean.h>

// boost
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "../include/CeresICP.h"

static const bool prinfDebugInfo = false;

static const float scanPeriod = 0.1;

static float groundThreshold = -1.0;
static float closePointThreshold = 50.0;
static float validScoreThreshold = 0.18;
static float validScoreDivergedThreshold = 0.12;
static double speedPrior = 3.0;

static int frameCountSinceStart = 0;

// Robust Fitting
static float huberWidth = 3.0;

static const int skipFrameNum = 0;
static bool systemInited = false;

static double timeCornerPointsSharp = 0;
static double timeCornerPointsLessSharp = 0;
static double timeSurfPointsFlat = 0;
static double timeSurfPointsLessFlat = 0;
static double timeLaserCloudFullRes = 0;
static double timeImuTrans = 0;

static bool newCornerPointsSharp = false;
static bool newCornerPointsLessSharp = false;
static bool newSurfPointsFlat = false;
static bool newSurfPointsLessFlat = false;
static bool newLaserCloudFullRes = false;
static bool newImuTrans = false;

// LOAM
static pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
static pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>());
static pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<PointType>());
static pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<PointType>());

static pcl::PointCloud<PointType>::Ptr cornerPointsSharpTransformed(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr surfPointsFlatTransformed(new pcl::PointCloud<PointType>());

// For ICP
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCur(new pcl::PointCloud<pcl::PointXYZI>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudLast(new pcl::PointCloud<pcl::PointXYZI>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCurFiltered(new pcl::PointCloud<pcl::PointXYZI>);

static int laserCloudCornerLastNum;
static int laserCloudSurfLastNum;

static int pointSelCornerInd[40000];
static float pointSearchCornerInd1[40000];
static float pointSearchCornerInd2[40000];

static int pointSelSurfInd[40000];
static float pointSearchSurfInd1[40000];
static float pointSearchSurfInd2[40000];
static float pointSearchSurfInd3[40000];

static float transform[6] = {0}; // k timestep to k+1
static float transformSum[6] = {0};
static float transformLast[6] = {0};
static float transformLOAM[6] = {0};
static double initFrameROSTime = -1.0;

static float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
static float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
static float imuShiftFromStartX = 0, imuShiftFromStartY = 0, imuShiftFromStartZ = 0;
static float imuVeloFromStartX = 0, imuVeloFromStartY = 0, imuVeloFromStartZ = 0;

void TransformToStart(PointType const * const pi, PointType * const po) {
    float s = 10 * (pi->intensity - int(pi->intensity));

    float rx = s * transform[0];
    float ry = s * transform[1];
    float rz = s * transform[2];
    float tx = s * transform[3];
    float ty = s * transform[4];
    float tz = s * transform[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    po->x = cos(ry) * x2 - sin(ry) * z2;
    po->y = y2;
    po->z = sin(ry) * x2 + cos(ry) * z2;
    po->intensity = pi->intensity;
}

void TransformToEnd(PointType const * const pi, PointType * const po) {
    float s = 10 * (pi->intensity - int(pi->intensity));

    float rx = s * transform[0];
    float ry = s * transform[1];
    float rz = s * transform[2];
    float tx = s * transform[3];
    float ty = s * transform[4];
    float tz = s * transform[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    float x3 = cos(ry) * x2 - sin(ry) * z2;
    float y3 = y2;
    float z3 = sin(ry) * x2 + cos(ry) * z2;

    rx = transform[0];
    ry = transform[1];
    rz = transform[2];
    tx = transform[3];
    ty = transform[4];
    tz = transform[5];

    float x4 = cos(ry) * x3 + sin(ry) * z3;
    float y4 = y3;
    float z4 = -sin(ry) * x3 + cos(ry) * z3;

    float x5 = x4;
    float y5 = cos(rx) * y4 - sin(rx) * z4;
    float z5 = sin(rx) * y4 + cos(rx) * z4;

    float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
    float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
    float z6 = z5 + tz;

    float x7 = cos(imuRollStart) * (x6 - imuShiftFromStartX)
            - sin(imuRollStart) * (y6 - imuShiftFromStartY);
    float y7 = sin(imuRollStart) * (x6 - imuShiftFromStartX)
            + cos(imuRollStart) * (y6 - imuShiftFromStartY);
    float z7 = z6 - imuShiftFromStartZ;

    float x8 = x7;
    float y8 = cos(imuPitchStart) * y7 - sin(imuPitchStart) * z7;
    float z8 = sin(imuPitchStart) * y7 + cos(imuPitchStart) * z7;

    float x9 = cos(imuYawStart) * x8 + sin(imuYawStart) * z8;
    float y9 = y8;
    float z9 = -sin(imuYawStart) * x8 + cos(imuYawStart) * z8;

    float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
    float y10 = y9;
    float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

    float x11 = x10;
    float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
    float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

    po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
    po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
    po->z = z11;
    po->intensity = int(pi->intensity);
}

void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz,
        float alx, float aly, float alz, float &acx, float &acy, float &acz) {
    float sbcx = sin(bcx);
    float cbcx = cos(bcx);
    float sbcy = sin(bcy);
    float cbcy = cos(bcy);
    float sbcz = sin(bcz);
    float cbcz = cos(bcz);

    float sblx = sin(blx);
    float cblx = cos(blx);
    float sbly = sin(bly);
    float cbly = cos(bly);
    float sblz = sin(blz);
    float cblz = cos(blz);

    float salx = sin(alx);
    float calx = cos(alx);
    float saly = sin(aly);
    float caly = cos(aly);
    float salz = sin(alz);
    float calz = cos(alz);

    float srx = -sbcx * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly)
            - cbcx * cbcz * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
            - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
            - cbcx * sbcz * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
            - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz);
    acx = -asin(srx);

    float srycrx = (cbcy * sbcz - cbcz * sbcx * sbcy)*(calx * saly * (cbly * sblz - cblz * sblx * sbly)
            - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
            - (cbcy * cbcz + sbcx * sbcy * sbcz)*(calx * caly * (cblz * sbly - cbly * sblx * sblz)
            - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz)
            + cbcx * sbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
    float crycrx = (cbcz * sbcy - cbcy * sbcx * sbcz)*(calx * caly * (cblz * sbly - cbly * sblx * sblz)
            - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz)
            - (sbcy * sbcz + cbcy * cbcz * sbcx)*(calx * saly * (cbly * sblz - cblz * sblx * sbly)
            - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
            + cbcx * cbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
    acy = atan2(srycrx / cos(acx), crycrx / cos(acx));

    float srzcrx = sbcx * (cblx * cbly * (calz * saly - caly * salx * salz)
            - cblx * sbly * (caly * calz + salx * saly * salz) + calx * salz * sblx)
            - cbcx * cbcz * ((caly * calz + salx * saly * salz)*(cbly * sblz - cblz * sblx * sbly)
            + (calz * saly - caly * salx * salz)*(sbly * sblz + cbly * cblz * sblx)
            - calx * cblx * cblz * salz) + cbcx * sbcz * ((caly * calz + salx * saly * salz)*(cbly * cblz
            + sblx * sbly * sblz) + (calz * saly - caly * salx * salz)*(cblz * sbly - cbly * sblx * sblz)
            + calx * cblx * salz * sblz);
    float crzcrx = sbcx * (cblx * sbly * (caly * salz - calz * salx * saly)
            - cblx * cbly * (saly * salz + caly * calz * salx) + calx * calz * sblx)
            + cbcx * cbcz * ((saly * salz + caly * calz * salx)*(sbly * sblz + cbly * cblz * sblx)
            + (caly * salz - calz * salx * saly)*(cbly * sblz - cblz * sblx * sbly)
            + calx * calz * cblx * cblz) - cbcx * sbcz * ((saly * salz + caly * calz * salx)*(cblz * sbly
            - cbly * sblx * sblz) + (caly * salz - calz * salx * saly)*(cbly * cblz + sblx * sbly * sblz)
            - calx * calz * cblx * sblz);
    acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
}

void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
        float &ox, float &oy, float &oz) {
    float srx = cos(lx) * cos(cx) * sin(ly) * sin(cz) - cos(cx) * cos(cz) * sin(lx) - cos(lx) * cos(ly) * sin(cx);
    ox = -asin(srx);

    float srycrx = sin(lx)*(cos(cy) * sin(cz) - cos(cz) * sin(cx) * sin(cy)) + cos(lx) * sin(ly)*(cos(cy) * cos(cz)
            + sin(cx) * sin(cy) * sin(cz)) + cos(lx) * cos(ly) * cos(cx) * sin(cy);
    float crycrx = cos(lx) * cos(ly) * cos(cx) * cos(cy) - cos(lx) * sin(ly)*(cos(cz) * sin(cy)
            - cos(cy) * sin(cx) * sin(cz)) - sin(lx)*(sin(cy) * sin(cz) + cos(cy) * cos(cz) * sin(cx));
    oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

    float srzcrx = sin(cx)*(cos(lz) * sin(ly) - cos(ly) * sin(lx) * sin(lz)) + cos(cx) * sin(cz)*(cos(ly) * cos(lz)
            + sin(lx) * sin(ly) * sin(lz)) + cos(lx) * cos(cx) * cos(cz) * sin(lz);
    float crzcrx = cos(lx) * cos(lz) * cos(cx) * cos(cz) - cos(cx) * sin(cz)*(cos(ly) * sin(lz)
            - cos(lz) * sin(lx) * sin(ly)) - sin(cx)*(sin(ly) * sin(lz) + cos(ly) * cos(lz) * sin(lx));
    oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

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

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharp2) {
    timeCornerPointsSharp = cornerPointsSharp2->header.stamp.toSec();

    cornerPointsSharp->clear();
    pcl::fromROSMsg(*cornerPointsSharp2, *cornerPointsSharp);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
    newCornerPointsSharp = true;
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharp2) {
    timeCornerPointsLessSharp = cornerPointsLessSharp2->header.stamp.toSec();

    cornerPointsLessSharp->clear();
    pcl::fromROSMsg(*cornerPointsLessSharp2, *cornerPointsLessSharp);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, indices);
    newCornerPointsLessSharp = true;
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlat2) {
    timeSurfPointsFlat = surfPointsFlat2->header.stamp.toSec();

    surfPointsFlat->clear();
    pcl::fromROSMsg(*surfPointsFlat2, *surfPointsFlat);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsFlat, *surfPointsFlat, indices);
    newSurfPointsFlat = true;
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlat2) {
    timeSurfPointsLessFlat = surfPointsLessFlat2->header.stamp.toSec();

    surfPointsLessFlat->clear();
    pcl::fromROSMsg(*surfPointsLessFlat2, *surfPointsLessFlat);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, indices);
    newSurfPointsLessFlat = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) {
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

    laserCloudFullRes->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudFullRes, *laserCloudFullRes, indices);
    newLaserCloudFullRes = true;
}

void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTrans2) {
    timeImuTrans = imuTrans2->header.stamp.toSec();

    imuTrans->clear();
    pcl::fromROSMsg(*imuTrans2, *imuTrans);

    imuPitchStart = imuTrans->points[0].x;
    imuYawStart = imuTrans->points[0].y;
    imuRollStart = imuTrans->points[0].z;

    imuPitchLast = imuTrans->points[1].x;
    imuYawLast = imuTrans->points[1].y;
    imuRollLast = imuTrans->points[1].z;

    imuShiftFromStartX = imuTrans->points[2].x;
    imuShiftFromStartY = imuTrans->points[2].y;
    imuShiftFromStartZ = imuTrans->points[2].z;

    imuVeloFromStartX = imuTrans->points[3].x;
    imuVeloFromStartY = imuTrans->points[3].y;
    imuVeloFromStartZ = imuTrans->points[3].z;

    newImuTrans = true;
}

void resetOdometrySystem() {
    std::cout << "Reset Odometry System!!!\n";
    // Reset system variable
    frameCountSinceStart = 0;
    systemInited = false;
    speedPrior = 3.0;

    // Reset Time
    timeCornerPointsSharp = 0;
    timeCornerPointsLessSharp = 0;
    timeSurfPointsFlat = 0;
    timeSurfPointsLessFlat = 0;
    timeLaserCloudFullRes = 0;
    timeImuTrans = 0;

    // Reset PointCloud Count
    newCornerPointsSharp = false;
    newCornerPointsLessSharp = false;
    newSurfPointsFlat = false;
    newSurfPointsLessFlat = false;
    newLaserCloudFullRes = false;
    newImuTrans = false;

    // Reset PointCloud
    cornerPointsSharp->clear();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();
    laserCloudCornerLast->clear();
    laserCloudSurfLast->clear();
    laserCloudOri->clear();
    coeffSel->clear();
    laserCloudFullRes->clear();
    imuTrans->clear();

    cornerPointsSharpTransformed->clear();
    surfPointsFlatTransformed->clear();
    cloud_in->clear();
    cloud_out->clear();
    laserCloudCur->clear();
    laserCloudCurFiltered->clear();

    // Reset transformation
    for (int i = 0; i < 6; i++) {
        transform[i] = 0;
        transformSum[i] = 0;
        transformLast[i] = 0;
        transformLOAM[i] = 0;
    }

    // Reset Time
    initFrameROSTime = -1.0;

    // Reset IMU
    imuRollStart = 0;
    imuPitchStart = 0;
    imuYawStart = 0;
    imuRollLast = 0;
    imuPitchLast = 0;
    imuYawLast = 0;
    imuShiftFromStartX = 0;
    imuShiftFromStartY = 0;
    imuShiftFromStartZ = 0;
    imuVeloFromStartX = 0;
    imuVeloFromStartY = 0;
    imuVeloFromStartZ = 0;
}

void resetSignalHandler(const sensor_msgs::PointCloud2ConstPtr& submapPointCloudMsg) {
    resetOdometrySystem();
}

void speedHandler(const sensor_msgs::NavSatFix::ConstPtr& speedIn) {
    speedPrior = speedIn->latitude;
//    std::cout << "Get speed: " << speedPrior << std::endl;
}

void useMotionPrior() {
    // Use GPS Info and constant velocity model
    //    if (speedPrior > 0.7)
    transform[3] = speedPrior;
    //    transform[4] = 0.0; // No Motion on Y axis
    //    transform[5] = 0.0; // No Motion on Z axis
}

void checkMotionEstimation() {
    double turningScale = 1.0;
    double difInMovingDirection = speedPrior * 0.3;
    double SpeedInSecondDirections = speedPrior * 0.333;
    if (speedPrior < 0.7) {
        turningScale = 3.0;
        difInMovingDirection = 1.0;
        SpeedInSecondDirections = 0.333;
    }

//    std::cout << "Checking transform: ";
//    for (int i = 0; i < 6; i++)
//        std::cout << transform[i] << " ";
//    std::cout << "\n";

    // Check if the motion valid
    if (fabs(transform[3] - speedPrior) > difInMovingDirection ||
            fabs(transform[4]) > SpeedInSecondDirections * turningScale ||
            fabs(transform[5]) > 0.1 ||
            fabs(transform[0]) > 0.01 * turningScale ||
            fabs(transform[1]) > 0.01 * turningScale ||
            fabs(transform[2]) > 0.01 * turningScale) {
        //        for (int i = 0; i < 6; i++)
        //            transform[i] = transformLast[i];
        transform[3] = speedPrior;
        std::cout << "Use Motion Prior for Final Result in Odometry!\n";
    }

    // Store last transformation
    transformLast[0] = transform[0];
    transformLast[1] = transform[1];
    transformLast[2] = transform[2];
    transformLast[3] = transform[3];
    transformLast[4] = transform[4];
    transformLast[5] = transform[5];
}

#endif /* LASERROBUSTODOMETRY_H */

