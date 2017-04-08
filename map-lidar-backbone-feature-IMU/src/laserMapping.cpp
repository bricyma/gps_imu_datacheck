// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.


#include "../include/laserMapping.h"

int main(int argc, char** argv)
{
    // Init ros and NodeHandle
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    // Define Publisher and Subscriber
    ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_corner_last", 2, laserCloudCornerLastHandler);

    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_surf_last", 2, laserCloudSurfLastHandler); // last frame respected to the next frame, in fact is cur frame 

    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>
            ("/laser_odom_to_init", 5, laserOdometryHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_3", 2, laserCloudFullResHandler);

    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);
    ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix> ("/gps_raw", 50, gpsHandler);
    ros::Subscriber subSpeed = nh.subscribe<sensor_msgs::NavSatFix> ("/speed_raw", 50, speedHandler);
    ros::Subscriber subResetSignal = nh.subscribe<sensor_msgs::PointCloud2>
            ("/reset_mapping_signal", 2, resetMappingSignalHandler);

    ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_surround", 1);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_registered", 1);

    ros::Publisher pubLaserCloudFullMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_full", 2);

    pubResetSignal = nh.advertise<sensor_msgs::PointCloud2>("/reset_signal", 2);
    pubLaserCloudSubmap = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_submap", 2);

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/camera_init";
    odomAftMapped.child_frame_id = "/aft_mapped";

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform aftMappedTrans;
    aftMappedTrans.frame_id_ = "/camera_init";
    aftMappedTrans.child_frame_id_ = "/aft_mapped";

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel, pointProj, coeff;

    cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
    cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

    bool isDegenerate = false;
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

    pcl::VoxelGrid<PointType> downSizeFilterMap;
    downSizeFilterMap.setLeafSize(0.3, 0.3, 0.3);

    for (int i = 0; i < laserCloudNum; i++)
    {
        laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
    }

    int frameCount = stackFrameNum - 1;
    int mapFrameCount = mapFrameNum - 1;
    ros::Rate rate(100);
    bool status = ros::ok();

    while (status)
    {
        ros::spinOnce();

        if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
                fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
                fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
                fabs(timeLaserCloudFullRes - timeLaserOdometry) < 0.005)
        {

            bool isConverged = false;

            // Check system status and start system
            if (!systemInited)
                startMappingSystem();

            newLaserCloudCornerLast = false;
            newLaserCloudSurfLast = false;
            newLaserCloudFullRes = false;
            newLaserOdometry = false;


            useMotionPrior();

            frameCount++;
            if (frameCount >= stackFrameNum)
            {
                //transformAssociateToMap(); // not stable??

                // Cache transformTobeMapped for recovery
                for (int i = 0; i < 6; i++)
                    transformTobeMappedBeforeLOAM[i] = transformTobeMapped[i];

                int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
                for (int i = 0; i < laserCloudCornerLastNum; i++)
                {
                    pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
                    laserCloudCornerStack2->push_back(pointSel);
                }

                int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
                for (int i = 0; i < laserCloudSurfLastNum; i++)
                {
                    pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
                    laserCloudSurfStack2->push_back(pointSel);
                }
            }

            if (frameCount >= stackFrameNum)
            {
                frameCount = 0;

                PointType pointOnYAxis;
                pointOnYAxis.x = 0.0;
                pointOnYAxis.y = 10.0;
                pointOnYAxis.z = 0.0;
                pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

                int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
                int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
                int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;

                if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
                if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
                if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;

                while (centerCubeI < 3)
                {
                    for (int j = 0; j < laserCloudHeight; j++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int i = laserCloudWidth - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; i >= 1; i--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeI++;
                    laserCloudCenWidth++;
                }

                while (centerCubeI >= laserCloudWidth - 3)
                {
                    for (int j = 0; j < laserCloudHeight; j++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int i = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; i < laserCloudWidth - 1; i++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeI--;
                    laserCloudCenWidth--;
                }

                while (centerCubeJ < 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int j = laserCloudHeight - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; j >= 1; j--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeJ++;
                    laserCloudCenHeight++;
                }

                while (centerCubeJ >= laserCloudHeight - 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int j = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; j < laserCloudHeight - 1; j++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeJ--;
                    laserCloudCenHeight--;
                }

                while (centerCubeK < 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int j = 0; j < laserCloudHeight; j++)
                        {
                            int k = laserCloudDepth - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; k >= 1; k--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeK++;
                    laserCloudCenDepth++;
                }

                while (centerCubeK >= laserCloudDepth - 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int j = 0; j < laserCloudHeight; j++)
                        {
                            int k = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; k < laserCloudDepth - 1; k++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeK--;
                    laserCloudCenDepth--;
                }

                int laserCloudValidNum = 0;
                int laserCloudSurroundNum = 0;
                for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
                {
                    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
                    {
                        for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
                        {
                            if (i >= 0 && i < laserCloudWidth &&
                                    j >= 0 && j < laserCloudHeight &&
                                    k >= 0 && k < laserCloudDepth)
                            {

                                float centerX = 50.0 * (i - laserCloudCenWidth);
                                float centerY = 50.0 * (j - laserCloudCenHeight);
                                float centerZ = 50.0 * (k - laserCloudCenDepth);

                                bool isInLaserFOV = false;
                                for (int ii = -1; ii <= 1; ii += 2)
                                {
                                    for (int jj = -1; jj <= 1; jj += 2)
                                    {
                                        for (int kk = -1; kk <= 1; kk += 2)
                                        {
                                            float cornerX = centerX + 25.0 * ii;
                                            float cornerY = centerY + 25.0 * jj;
                                            float cornerZ = centerZ + 25.0 * kk;

                                            float squaredSide1 = (transformTobeMapped[3] - cornerX)
                                                    * (transformTobeMapped[3] - cornerX)
                                                    + (transformTobeMapped[4] - cornerY)
                                                    * (transformTobeMapped[4] - cornerY)
                                                    + (transformTobeMapped[5] - cornerZ)
                                                    * (transformTobeMapped[5] - cornerZ);

                                            float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
                                                    + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                                    + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                                            float check1 = 100.0 + squaredSide1 - squaredSide2
                                                    - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                                            float check2 = 100.0 + squaredSide1 - squaredSide2
                                                    + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                                            if (check1 < 0 && check2 > 0)
                                            {
                                                isInLaserFOV = true;
                                            }
                                        }
                                    }
                                }

                                if (isInLaserFOV)
                                {
                                    laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j
                                            + laserCloudWidth * laserCloudHeight * k;
                                    laserCloudValidNum++;
                                }
                                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j
                                        + laserCloudWidth * laserCloudHeight * k;
                                laserCloudSurroundNum++;
                            }
                        }
                    }
                }

                laserCloudCornerFromMap->clear();
                laserCloudSurfFromMap->clear();
                for (int i = 0; i < laserCloudValidNum; i++)
                {
                    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
                    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
                }
                int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
                int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

                int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
                for (int i = 0; i < laserCloudCornerStackNum2; i++)
                {
                    pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
                }

                int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
                for (int i = 0; i < laserCloudSurfStackNum2; i++)
                {
                    pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
                }

                laserCloudCornerStack->clear();
                downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
                downSizeFilterCorner.filter(*laserCloudCornerStack);
                int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

                laserCloudSurfStack->clear();
                downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
                downSizeFilterSurf.filter(*laserCloudSurfStack);
                int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

                laserCloudCornerStack2->clear();
                laserCloudSurfStack2->clear();
                laserCloudSurfStackMapped->clear();
                laserCloudCornerStackMapped->clear();

                if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
                {
                    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
                    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

                    float sumCost = 0.0;
                    for (int iterCount = 0; iterCount < 25; iterCount++)
                    {
                        sumCost = 0.0;
                        laserCloudOri->clear();
                        coeffSel->clear();

                        // Doesn't use line feature here
                        //                        for (int i = 0; i < laserCloudCornerStackNum; i++)
                        if (false)
                        {
                            int i = 0;
                            pointOri = laserCloudCornerStack->points[i];
                            pointAssociateToMap(&pointOri, &pointSel);
                            if (iterCount == 0)
                                laserCloudCornerStackMapped->push_back(pointSel);

                            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                            if (pointSearchSqDis[4] < 1.0)
                            {
                                float cx = 0;
                                float cy = 0;
                                float cz = 0;
                                for (int j = 0; j < 5; j++)
                                {
                                    cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
                                    cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
                                    cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
                                }
                                cx /= 5;
                                cy /= 5;
                                cz /= 5;

                                float a11 = 0;
                                float a12 = 0;
                                float a13 = 0;
                                float a22 = 0;
                                float a23 = 0;
                                float a33 = 0;
                                for (int j = 0; j < 5; j++)
                                {
                                    float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
                                    float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
                                    float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

                                    a11 += ax * ax;
                                    a12 += ax * ay;
                                    a13 += ax * az;
                                    a22 += ay * ay;
                                    a23 += ay * az;
                                    a33 += az * az;
                                }
                                a11 /= 5;
                                a12 /= 5;
                                a13 /= 5;
                                a22 /= 5;
                                a23 /= 5;
                                a33 /= 5;

                                matA1.at<float>(0, 0) = a11;
                                matA1.at<float>(0, 1) = a12;
                                matA1.at<float>(0, 2) = a13;
                                matA1.at<float>(1, 0) = a12;
                                matA1.at<float>(1, 1) = a22;
                                matA1.at<float>(1, 2) = a23;
                                matA1.at<float>(2, 0) = a13;
                                matA1.at<float>(2, 1) = a23;
                                matA1.at<float>(2, 2) = a33;

                                cv::eigen(matA1, matD1, matV1);

                                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
                                {

                                    float x0 = pointSel.x;
                                    float y0 = pointSel.y;
                                    float z0 = pointSel.z;
                                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                            * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                            + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                            * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                            + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                            * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                                    float ld2 = a012 / l12;

                                    pointProj = pointSel;
                                    pointProj.x -= la * ld2;
                                    pointProj.y -= lb * ld2;
                                    pointProj.z -= lc * ld2;

                                    float s = 1 - 0.9 * fabs(ld2);

                                    // Use Huber norm here
                                    if (ld2 > huberWidth)
                                        s = huberWidth / fabs(ld2);
                                    else
                                        s = 1.0;
                                    s = sqrt(s);

                                    coeff.x = s * la;
                                    coeff.y = s * lb;
                                    coeff.z = s * lc;
                                    coeff.intensity = s * ld2;

                                    //                                    if (s > 0.1)
                                    //                                    {
                                    //                                        laserCloudOri->push_back(pointOri);
                                    //                                        coeffSel->push_back(coeff);
                                    //                                    }

                                    laserCloudOri->push_back(pointOri);
                                    coeffSel->push_back(coeff);
                                }
                            }
                        }

                        int numOfPlanePoints = 0;
                        for (int i = 0; i < laserCloudSurfStackNum; i++)
                        {
                            pointOri = laserCloudSurfStack->points[i];
                            pointAssociateToMap(&pointOri, &pointSel);
                            if (iterCount == 0)
                                laserCloudSurfStackMapped->push_back(pointSel);

                            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                            if (pointSearchSqDis[4] < 1.0)
                            {
                                for (int j = 0; j < 5; j++)
                                {
                                    matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                                    matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                                    matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                                }
                                cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                                float pa = matX0.at<float>(0, 0);
                                float pb = matX0.at<float>(1, 0);
                                float pc = matX0.at<float>(2, 0);
                                float pd = 1;

                                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                                pa /= ps;
                                pb /= ps;
                                pc /= ps;
                                pd /= ps;

                                bool planeValid = true;
                                for (int j = 0; j < 5; j++)
                                {
                                    if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                                            pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                                            pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
                                    {
                                        planeValid = false;
                                        break;
                                    }
                                }

                                if (planeValid)
                                {
                                    numOfPlanePoints++;
                                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                                    pointProj = pointSel;
                                    pointProj.x -= pa * pd2;
                                    pointProj.y -= pb * pd2;
                                    pointProj.z -= pc * pd2;

                                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                                    //                                    // Use Huber norm here
                                    //                                    if (pd2 > huberWidth)
                                    //                                        s = huberWidth / fabs(pd2);
                                    //                                    else
                                    //                                        s = 1.0;
                                    //                                    s = sqrt(s);

                                    coeff.x = s * pa;
                                    coeff.y = s * pb;
                                    coeff.z = s * pc;
                                    coeff.intensity = s * pd2;

                                    if (s > 0.1)
                                    {
                                        laserCloudOri->push_back(pointOri);
                                        coeffSel->push_back(coeff);
                                    }

                                    //                                    laserCloudOri->push_back(pointOri);
                                    //                                    coeffSel->push_back(coeff);

                                    sumCost += pd2*pd2;
                                }
                            }
                        }

                        //                        std::cout << "sumCost: " << sumCost << "\n";
                        //                        std::cout << "PlanePoints: " << float(numOfPlanePoints) / float(laserCloudSurfStackNum) << "\n";

                        float srx = sin(transformTobeMapped[0]);
                        float crx = cos(transformTobeMapped[0]);
                        float sry = sin(transformTobeMapped[1]);
                        float cry = cos(transformTobeMapped[1]);
                        float srz = sin(transformTobeMapped[2]);
                        float crz = cos(transformTobeMapped[2]);

                        Eigen::Affine3f RotationTobeMapped = Eigen::Affine3f::Identity();
                        GetTransformationFromEulerAnglesAndTranslation(RotationTobeMapped,
                                transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[3],
                                0.0, 0.0, 0.0);
                        //                        std::cout << "RotationTobeMapped:\n " << RotationTobeMapped.matrix() << "\n";

                        int laserCloudSelNum = laserCloudOri->points.size();
                        if (laserCloudSelNum < 50)
                        {
                            std::cout << "Continue here!\n";
                            continue;
                        }

                        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
                        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
                        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
                        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
                        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
                        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
                        for (int i = 0; i < laserCloudSelNum; i++)
                        {
                            pointOri = laserCloudOri->points[i];
                            coeff = coeffSel->points[i];

                            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x
                                    + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y
                                    + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

                            float ary = ((cry * srx * srz - crz * sry) * pointOri.x
                                    + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x
                                    + ((-cry * crz - srx * sry * srz) * pointOri.x
                                    + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

                            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x
                                    + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y
                                    + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

                            matA.at<float>(i, 0) = arx;
                            matA.at<float>(i, 1) = ary;
                            matA.at<float>(i, 2) = arz;
                            matA.at<float>(i, 3) = coeff.x;
                            matA.at<float>(i, 4) = coeff.y;
                            matA.at<float>(i, 5) = coeff.z;
                            matB.at<float>(i, 0) = -coeff.intensity;
                        }
                        cv::transpose(matA, matAt);
                        matAtA = matAt * matA;
                        matAtB = matAt * matB;
                        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

                        if (iterCount == 0)
                        {
                            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
                            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
                            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

                            cv::eigen(matAtA, matE, matV);
                            matV.copyTo(matV2);

                            isDegenerate = false;
                            float eignThre[6] = {100, 100, 100, 100, 100, 100};
                            for (int i = 5; i >= 0; i--)
                            {
                                if (matE.at<float>(0, i) < eignThre[i])
                                {
                                    for (int j = 0; j < 6; j++)
                                    {
                                        matV2.at<float>(i, j) = 0;
                                    }
                                    isDegenerate = true;
                                } else
                                {
                                    break;
                                }
                            }
                            matP = matV.inv() * matV2;
                        }

                        if (isDegenerate)
                        {
                            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
                            matX.copyTo(matX2);
                            matX = matP * matX2;
                        }

                        transformTobeMapped[0] += matX.at<float>(0, 0);
                        transformTobeMapped[1] += matX.at<float>(1, 0);
                        transformTobeMapped[2] += matX.at<float>(2, 0);
                        transformTobeMapped[3] += matX.at<float>(3, 0);
                        transformTobeMapped[4] += matX.at<float>(4, 0);
                        transformTobeMapped[5] += matX.at<float>(5, 0);

                        float deltaR = sqrt(
                                pow(rad2deg(matX.at<float>(0, 0)), 2) +
                                pow(rad2deg(matX.at<float>(1, 0)), 2) +
                                pow(rad2deg(matX.at<float>(2, 0)), 2));
                        float deltaT = sqrt(
                                pow(matX.at<float>(3, 0) * 100, 2) +
                                pow(matX.at<float>(4, 0) * 100, 2) +
                                pow(matX.at<float>(5, 0) * 100, 2));

                        if (deltaR < 0.05 && deltaT < 0.05)
                        {
                            isConverged = true;
                            if (false)
                            {
                                std::cout << "LOAM Mapping Optimization Converge in " << iterCount << " iterations!\n";
                            }
                            break;
                        }
                    }

                    //                    if (isConverged)
                    transformUpdate();
                }

                // Do LM ICP if loam Mapping doesn't converge           
                //                std::cout << "LOAM Mapping Converge?  " << (isConverged ? "True!" : "False!") << std::endl;
                //                if (!isConverged)
                if (false)
                {
                    // Recover transformTobeMapped from transformTobeMappedBeforeLOAM
                    for (int i = 0; i < 6; i++)
                        transformTobeMapped[i] = transformTobeMappedBeforeLOAM[i];

                    // Keep track of transformed cur laser points(surf only)
                    pcl::PointCloud<pcl::PointXYZI>::Ptr SurfStackMappedTransformed(new pcl::PointCloud<pcl::PointXYZI>);
                    for (int i = 0; i < laserCloudSurfStackMapped->width; i++)
                    {
                        SurfStackMappedTransformed->push_back(laserCloudSurfStackMapped->points[i]);
                    }

                    std::vector<bool> inliers;
                    bool isLMConverged = false;
                    int iter = 0;
                    Eigen::Isometry3d fromCurToRef_estimated = Eigen::Isometry3d::Identity();

                    while (!isLMConverged && iter < 15)
                    {
                        double outlierThreshold = 2.0;
                        Eigen::Isometry3d fromCurToRef_inc = MappingICP(laserCloudSurfFromMap, SurfStackMappedTransformed, inliers, outlierThreshold);
                        std::cout << "iter: " << iter++ << std::endl;
                        //                        std::cout << fromCurToRef_inc.matrix() << std::endl;
                        //        std::cout << fromCurToRef_inc.translation()[0] << std::endl;
                        //        std::cout << fromCurToRef_inc.translation()[1] << std::endl;
                        //        std::cout << fromCurToRef_inc.translation()[2] << std::endl;

                        if (fabs(fromCurToRef_inc.translation()[0]) < 0.001 && fabs(fromCurToRef_inc.translation()[1]) < 0.001 && fabs(fromCurToRef_inc.translation()[2]) < 0.001)
                            isLMConverged = true;

                        for (int i = 0; i < SurfStackMappedTransformed->points.size(); i++)
                        {
                            Eigen::Vector4d oriPoint;
                            oriPoint << SurfStackMappedTransformed->points[i].x, SurfStackMappedTransformed->points[i].y, SurfStackMappedTransformed->points[i].z, 1.0;
                            Eigen::Vector4d transformedPoint = fromCurToRef_inc * oriPoint;
                            //        Eigen::Vector4f transformedPoint = oriPoint;
                            SurfStackMappedTransformed->points[i].x = transformedPoint[0];
                            SurfStackMappedTransformed->points[i].y = transformedPoint[1];
                            SurfStackMappedTransformed->points[i].z = transformedPoint[2];
                        }
                        fromCurToRef_estimated = fromCurToRef_inc*fromCurToRef_estimated;
                        //                        iter++;
                    }
                    std::cout << "Final: " << fromCurToRef_estimated.matrix() << std::endl;
                    SurfStackMappedTransformed->width = (int) SurfStackMappedTransformed->points.size();
                    SurfStackMappedTransformed->height = 1;

                    // Select Inlier
                    //                    double outlierThreshold = 0.5;
                    //                    selectInlier(SurfFromMap, SurfStackMappedTransformed, inliers, 0.5);

                    // Update transformation with LM estimation
                    Eigen::Matrix4d fromCurToRef_mat = fromCurToRef_estimated.matrix();
                    Eigen::Affine3f fromCurToRef_aff = Eigen::Affine3f::Identity();

                    for (int i = 0; i < 4; i++)
                        for (int j = 0; j < 4; j++)
                            fromCurToRef_aff(i, j) = (float) fromCurToRef_mat(i, j);

                    //                float x, y, z, roll, pitch, yaw;
                    //                pcl::getTranslationAndEulerAngles(T_prev_to_cur_aff, x, y, z, roll, pitch, yaw);
                    //                //                    std::cout << roll << " " << pitch << " " << yaw << " " << x << " " << y << " " << z << "\n";
                    //                //    std::cout << -sin(pitch)  << "\n";

                    float rx, ry, rz, tx, ty, tz;
                    GetEulerAnglesAndTranslation(fromCurToRef_aff, rx, ry, rz, tx, ty, tz);

                    transformTobeMapped[0] += rx;
                    transformTobeMapped[1] += ry;
                    transformTobeMapped[2] += rz;
                    transformTobeMapped[3] += tx;
                    transformTobeMapped[4] += ty;
                    transformTobeMapped[5] += tz;

                    transformUpdate();
                }

                checkMotionEstimation();
                transformUpdate();

                // Done for cur lidar frame to map
                //                if (laserCloudCornerFromMap->width > 0 && laserCloudSurfFromMap->width > 0 && laserCloudCornerStackMapped->points.size() > 0 && laserCloudSurfStackMapped->points.size() > 0 && laserCloudFullRes->points.size() > 0)
                //                {
                //                    std::string numFileName;
                //                    std::stringstream StrStm;
                //                    StrStm << std::setprecision(12) << timeLaserOdometry;
                //                    StrStm >> numFileName;
                //
                //                    pcl::io::savePCDFileASCII("/home/yiluo/LIDAR/workspace/Debug/mappoints/" + numFileName + "_CornerFromMap" + ".pcd", *laserCloudCornerFromMap);
                //                    pcl::io::savePCDFileASCII("/home/yiluo/LIDAR/workspace/Debug/mappoints/" + numFileName + "_SurfFromMap" + ".pcd", *laserCloudSurfFromMap);
                //                    pcl::io::savePCDFileASCII("/home/yiluo/LIDAR/workspace/Debug/mappoints/" + numFileName + "_CornerStackMapped" + ".pcd", *laserCloudCornerStackMapped);
                //                    pcl::io::savePCDFileASCII("/home/yiluo/LIDAR/workspace/Debug/mappoints/" + numFileName + "_SurfStackMapped" + ".pcd", *laserCloudSurfStackMapped);
                //                    pcl::io::savePCDFileASCII("/home/yiluo/LIDAR/workspace/Debug/mappoints/" + numFileName + "_FullRes" + ".pcd", *laserCloudFullRes);
                //
                //                }

                for (int i = 0; i < laserCloudCornerStackNum; i++)
                {
                    pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

                    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

                    if (pointSel.x + 25.0 < 0) cubeI--;
                    if (pointSel.y + 25.0 < 0) cubeJ--;
                    if (pointSel.z + 25.0 < 0) cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth &&
                            cubeJ >= 0 && cubeJ < laserCloudHeight &&
                            cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        laserCloudCornerArray[cubeInd]->push_back(pointSel);
                    }
                }

                for (int i = 0; i < laserCloudSurfStackNum; i++)
                {
                    pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

                    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

                    if (pointSel.x + 25.0 < 0) cubeI--;
                    if (pointSel.y + 25.0 < 0) cubeJ--;
                    if (pointSel.z + 25.0 < 0) cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth &&
                            cubeJ >= 0 && cubeJ < laserCloudHeight &&
                            cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        laserCloudSurfArray[cubeInd]->push_back(pointSel);
                    }
                }

                for (int i = 0; i < laserCloudValidNum; i++)
                {
                    int ind = laserCloudValidInd[i];

                    laserCloudCornerArray2[ind]->clear();
                    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
                    downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

                    laserCloudSurfArray2[ind]->clear();
                    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
                    downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

                    pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
                    laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
                    laserCloudCornerArray2[ind] = laserCloudTemp;

                    laserCloudTemp = laserCloudSurfArray[ind];
                    laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
                    laserCloudSurfArray2[ind] = laserCloudTemp;
                }

                mapFrameCount++;
                if (mapFrameCount >= mapFrameNum)
                {
                    mapFrameCount = 0;

                    laserCloudSurround2->clear();
                    // std::cout << "Clear PointCloud laserCloudSurround2!\n";

                    for (int i = 0; i < laserCloudSurroundNum; i++)
                    {
                        int ind = laserCloudSurroundInd[i];
                        *laserCloudSurround2 += *laserCloudCornerArray[ind];
                        *laserCloudSurround2 += *laserCloudSurfArray[ind];
                    }

                    // Concatenate clouds
                    // Do it only if it not stationary
                    if (stationaryCounter < 4)
                    {
                        *laserCloudSurroundWhole += *laserCloudSurround;
                        laserCloudSurroundWhole->width = (int) laserCloudSurroundWhole->points.size();
                        laserCloudSurroundWhole->height = 1;
                    }

                    laserCloudSurround->clear();
                    downSizeFilterCorner.setInputCloud(laserCloudSurround2);
                    downSizeFilterCorner.filter(*laserCloudSurround);

                    sensor_msgs::PointCloud2 laserCloudSurround3;
                    pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
                    laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                    laserCloudSurround3.header.frame_id = "/camera_init";
                    pubLaserCloudSurround.publish(laserCloudSurround3);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }

                // Concatenate map clouds
                // Do it only if it not stationary
                if (stationaryCounter < 4)
                {
                    *laserCloudFullMap += *laserCloudFullRes;
                    laserCloudFullMap->width = (int) laserCloudFullMap->points.size();
                    laserCloudFullMap->height = 1;
                }

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                laserCloudFullRes3.header.frame_id = "/camera_init";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);

                //                sensor_msgs::PointCloud2 laserCloudFullMapMsg;
                //                pcl::toROSMsg(*laserCloudFullMap, laserCloudFullMapMsg);
                //                laserCloudFullMapMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                //                laserCloudFullMapMsg.header.frame_id = "/camera_init";
                //                pubLaserCloudFullMap.publish(laserCloudFullMapMsg);

                geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                        (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

                odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
                odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
                odomAftMapped.pose.pose.orientation.z = geoQuat.x;
                odomAftMapped.pose.pose.orientation.w = geoQuat.w;
                odomAftMapped.pose.pose.position.x = transformAftMapped[3];
                odomAftMapped.pose.pose.position.y = transformAftMapped[4];
                odomAftMapped.pose.pose.position.z = transformAftMapped[5];
                odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
                odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
                odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
                odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
                odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
                odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
                pubOdomAftMapped.publish(odomAftMapped);

                aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
                aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
                aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3],
                        transformAftMapped[4], transformAftMapped[5]));
                tfBroadcaster.sendTransform(aftMappedTrans);

                // Print Debug Info 
                //                std::cout << "Timestamp: " << ros::Time().fromSec(timeLaserOdometry) << "; Last Frame Pose: " << transformTobeMappedLast[0] << " " << transformTobeMappedLast[1] << " " << transformTobeMappedLast[2] << " " << transformTobeMappedLast[3] << " " << transformTobeMappedLast[4] << " " << transformTobeMappedLast[5] << std::endl;
                //                std::cout << "Timestamp: " << ros::Time().fromSec(timeLaserOdometry) << "; Pose: " << transformAftMapped[0] << " " << transformAftMapped[1] << " " << transformAftMapped[2] << " " << transformAftMapped[3] << " " << transformAftMapped[4] << " " << transformAftMapped[5] << std::endl;

                //                std::cout << "Timestamp:" << ros::Time().fromSec(timeLaserOdometry)
                //                        << ";Pose:" << transformAftMapped[0] << "," << transformAftMapped[1] << "," << transformAftMapped[2] << "," << transformAftMapped[3] << "," << transformAftMapped[4] << "," << transformAftMapped[5]
                //                        << ";GPSPose:" << (curGPS_X - startGPS_X) << "," << (curGPS_Y - startGPS_Y)
                //                        << ";stationaryCounter: " << stationaryCounter << std::endl;

                //                        << ";startGPSPosInited:" << startGPSPosInited << ",curGPSPosUpated:" << curGPSPosUpated << std::endl;


                //                std::cout << "Timestamp: " << ros::Time().fromSec(timeLaserOdometry) << "; Pose from last to cur: " << transformFromLastToCur[0] << " " << transformFromLastToCur[1] << " " << transformFromLastToCur[2] << " " << transformFromLastToCur[3] << " " << transformFromLastToCur[4] << " " << transformFromLastToCur[5] << std::endl << std::endl;
                //                std::cout << "Timestamp: " << ros::Time().fromSec(timeLaserOdometry) << "; Pose Dif: " << transformFromLastToCur[0] - transformFromSecondLastToLast[0] << " " << transformFromLastToCur[1] - transformFromSecondLastToLast[1] << " " << transformFromLastToCur[2] - transformFromSecondLastToLast[2] << " " << transformFromLastToCur[3] - transformFromSecondLastToLast[3] << " " << transformFromLastToCur[4] - transformFromSecondLastToLast[4] << " " << transformFromLastToCur[5] - transformFromSecondLastToLast[5] << "\n";




                // Check fail!!!!
                bool isFail = failureCheck();

                if (isFail)
                {
                    resetMappingSystem();
                } else
                {
                    // Log trajectory
                    trajectoryFileOut << std::setprecision(12) << "Timestamp:" << ros::Time().fromSec(timeLaserOdometry) << ";Pose:" << transformAftMapped[0] << "," << transformAftMapped[1] << "," << transformAftMapped[2] << "," << transformAftMapped[3] << "," << transformAftMapped[4] << "," << transformAftMapped[5] << ";GPSPose:" << curGPS_X << "," << curGPS_Y << std::endl;
                }
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    // Close the system
    resetMappingSystem();

    return 0;
}
